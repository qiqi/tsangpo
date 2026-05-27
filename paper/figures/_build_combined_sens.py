"""
Build one combined 3x3 sensitivity-sweep figure per config family,
overlaying cruise (green), takeoff (orange), and landing (red) on the
same axes.  Replaces the 3-phase side-by-side panels formerly used for
cont-high and gap-low.

Outputs:
    paper/figures/cont_high_combined_sens.png
    paper/figures/gap_low_combined_sens.png
"""
from __future__ import annotations
import csv, importlib.util, sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(REPO / "post"))
sys.path.insert(0, str(REPO))
import params as P
from _phase_plot import PhaseSpec, by_sweep, fit_linear

PHASE_COLORS = {
    "cruise":  "#1b7e3a",   # green
    "takeoff": "#d97706",   # orange
    "landing": "#c0392b",   # red
}
NUMERIC_KEYS = {"value", "CL", "CD", "CMy", "CMy_CG", "CFx",
                "CT_delivered", "F_AD_delivered_N"}


def load_spec(fam: str, phase: str) -> PhaseSpec:
    pp = REPO / "post" / fam / f"plot_{phase}_sensitivities.py"
    s  = importlib.util.spec_from_file_location(f"{fam}_{phase}", pp)
    m  = importlib.util.module_from_spec(s); s.loader.exec_module(m)
    return m.SPEC


def load_sweep(spec: PhaseSpec) -> dict:
    """Read the sweep CSV and augment with thrust-inclusive lift/drag.

    The actuator-disk thrust acts along the body x-axis.  Projected into
    wind axes at incidence alpha:
        CL_total = CL_aero + CT * sin(alpha)
        CD_total = CD_aero - CT * cos(alpha)
    With these definitions, CD_total = 0 corresponds to level flight,
    CD_total < 0 to climb, CD_total > 0 to descent (the wind-axis
    streamwise force balance).
    """
    from math import radians, sin, cos
    csv_path = spec.out_dir / f"{spec.phase_name}_sweep_data.csv"
    with csv_path.open() as f:
        rows = [{k: (float(v) if k in NUMERIC_KEYS else v)
                 for k, v in r.items()}
                for r in csv.DictReader(f)]
    a, da = by_sweep(rows, "alpha",  spec.qS)
    h, dh = by_sweep(rows, "htail",  spec.qS)
    t, dt = by_sweep(rows, "thrust", spec.qS)

    # alpha sweep: alpha varies, thrust ~ fixed (BO T_mult)
    a_rad = np.radians(a)
    da["CL_total"] = da["CL"] + da["CT_delivered"] * np.sin(a_rad)
    da["CD_total"] = da["CD"] - da["CT_delivered"] * np.cos(a_rad)

    # htail sweep: alpha = BO alpha (constant)
    ab = radians(spec.alpha_b)
    dh["CL_total"] = dh["CL"] + dh["CT_delivered"] * sin(ab)
    dh["CD_total"] = dh["CD"] - dh["CT_delivered"] * cos(ab)

    # thrust sweep: alpha = BO alpha, CT varies
    dt["CL_total"] = dt["CL"] + dt["CT_delivered"] * sin(ab)
    dt["CD_total"] = dt["CD"] - dt["CT_delivered"] * cos(ab)

    return {
        "alpha":  (a, da, spec.mask_alpha(a)),
        "htail":  (h, dh, spec.mask_htail(h)),
        "thrust": (t, dt, spec.mask_thrust(t)),
    }


def _plot_one_config(axes, fam: str, sweeps, cols, *,
                     primary: bool, legend_in_ax=(0, 2)):
    """Draw one config's three phases onto the existing 3x3 axes grid.

    primary=True  ->  larger circles + thicker connecting lines;
                       emits the legend for the phase colors.
    primary=False ->  small circles + thin lines, half scale, lower alpha
                       (the "overlay" of the previous step's data).
    """
    if primary:
        ms = 6; lw = 1.5; alpha = 1.0
    else:
        ms = 3; lw = 0.75; alpha = 0.6
    phase_data = {}
    bo_tl = {}
    for ph in ("cruise", "takeoff", "landing"):
        spec = load_spec(fam, ph)
        try:
            phase_data[ph] = (spec, load_sweep(spec))
        except FileNotFoundError:
            phase_data[ph] = None  # phase has no cached sweep yet
            continue
        # T/L at the actual BO operating point.  Use the htail-sweep row at
        # θ_ht = BO_θ (htail-sweep is at α=BO_α, T_mult=BO_T, varying θ_ht,
        # so this row IS the BO trim point — unlike the α-sweep's row at
        # α=BO_α, which after the resweep is at a different θ_ht).
        _, dh, _ = phase_data[ph][1]["htail"]
        i_bo = int(np.argmin(np.abs(
            np.array([r for r in phase_data[ph][1]["htail"][0]]) - spec.theta_ht_b)))
        bo_tl[ph] = float(dh["CT_delivered"][i_bo] /
                           max(dh["CL_total"][i_bo], 1e-9))

    for row, (sweep_name, _) in enumerate(sweeps):
        for col, (key, _) in enumerate(cols):
            ax = axes[row, col]
            for ph in ("cruise", "takeoff", "landing"):
                if phase_data[ph] is None:
                    continue
                spec, data = phase_data[ph]
                x, data_dict, mask = data[sweep_name]
                y = data_dict[key]
                # T/L = C_T / C_{L,total}, point-by-point.  L includes
                # the thrust's vertical (wind-axis lift) component, so
                # CL_total = CL_aero + CT*sin(alpha).  This makes T/L
                # a pure CFD-derived nondimensional ratio that does
                # not require an assumed aircraft weight.
                tl_arr = data_dict["CT_delivered"] / np.maximum(
                    data_dict["CL_total"], 1e-9)
                if sweep_name == "thrust":
                    x_disp = tl_arr
                else:
                    x_disp = x
                # T/L on the legend is always the BO trim T/L (from
                # htail-sweep at θ=BO), regardless of which sweep row this
                # plot panel displays.
                bo_label = f"T/L={bo_tl[ph]:.2f}"
                color = PHASE_COLORS[ph]
                # If the α-sweep was rerun at a different θ_ht (to keep the
                # htail unstalled), annotate the legend so the reader knows
                # which θ_ht each sweep row corresponds to.
                a_sweep_th = getattr(spec, "alpha_sweep_theta_ht", None)
                if a_sweep_th is not None and sweep_name == "alpha":
                    th_str = (f"θ_α={a_sweep_th:+.0f}°  "
                               f"(BO θ={spec.theta_ht_b:+.0f}°)")
                else:
                    th_str = f"θ={spec.theta_ht_b:+.0f}°"
                label = (f"{ph} (BO α={spec.alpha_b:+.0f}°, "
                         f"{th_str}, {bo_label})") if (primary and (row, col) == legend_in_ax) else None
                ax.plot(x_disp, y, "o", color=color, mfc="white",
                        ms=ms, alpha=alpha, label=label)
                order = np.argsort(x_disp)
                xs, ys, ms_ord = x_disp[order], y[order], mask[order]
                for i in range(len(xs) - 1):
                    ls = "-" if (ms_ord[i] and ms_ord[i + 1]) else "--"
                    ax.plot(xs[i:i + 2], ys[i:i + 2], linestyle=ls,
                            color=color, lw=lw, alpha=alpha * 0.85)


def plot_combined(fam: str, out_path: Path, title: str,
                   overlay_fam: str | None = None):
    fig, axes = plt.subplots(3, 3, figsize=(13.5, 10.5))
    plt.subplots_adjust(left=0.07, right=0.97, top=0.92, bottom=0.06,
                        hspace=0.34, wspace=0.27)
    sweeps = [
        ("alpha",  r"$\alpha$ [deg]"),
        ("htail",  r"$\theta_{\rm htail}$ [deg]"),
        ("thrust", r"delivered $T/L = C_T / C_{L,\rm total}$"),
    ]
    # Include thrust contribution in CL and CD so the reader can infer
    # flight angle directly: with these definitions
    #     CL_total = CL_aero + CT sin(alpha)
    #     CD_total = CD_aero - CT cos(alpha)
    # CD_total = 0 is exactly level flight; CD_total < 0 is climb,
    # CD_total > 0 is descent (wind-axis streamwise balance).
    cols = [
        ("CL_total", r"$C_L + C_T\sin\alpha$  (lift incl. thrust)"),
        ("CD_total", r"$C_D - C_T\cos\alpha$  (zero = level flight)"),
        ("CMy_CG",   r"$C_{m,y}$  (CG, incl. thrust)"),
    ]

    # Overlay first (smaller, thinner) so primary draws on top.
    if overlay_fam is not None:
        _plot_one_config(axes, overlay_fam, sweeps, cols, primary=False)
    _plot_one_config(axes, fam, sweeps, cols, primary=True)

    for row, (sweep_name, xlabel) in enumerate(sweeps):
        for col, (key, ylabel) in enumerate(cols):
            ax = axes[row, col]
            ax.set(xlabel=xlabel, ylabel=ylabel)
            ax.grid(True, alpha=0.3)
            if col == 0:
                ax.set_title(f"{sweep_name} sweep", loc="left",
                              fontsize=10, pad=4)
            if row == 0 and col == 2:
                ax.legend(fontsize=8, loc="best", framealpha=0.92)
    fig.suptitle(title, fontsize=12, y=0.985)
    fig.savefig(out_path, dpi=160, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out_path}")


if __name__ == "__main__":
    # Each config is plotted on its own — no overlay of the previous step
    # (per user direction: "get rid of the thin lines, just do each case
    # separately").
    plot_combined("v2_continuous",
                  HERE / "cont_low_combined_sens.png",
                  "cont-low (Step 0; X-57-class): low htail + continuous flap")
    plot_combined("v2_continuous_high",
                  HERE / "cont_high_combined_sens.png",
                  "cont-high (Step 1): T-tail + continuous flap")
    plot_combined("v2_gapped_high",
                  HERE / "gap_high_combined_sens.png",
                  "gap-high (Step 2): $40\\%$ inboard flap gap + T-tail")
    plot_combined("v2_gapped",
                  HERE / "gap_low_combined_sens.png",
                  "gap-low (Step 3): $40\\%$ inboard flap gap + low htail in slipstream")
    plot_combined("v3",
                  HERE / "v3_combined_sens.png",
                  "v3 (Step 4): gap + low htail + shortened tail boom (partial CFD)")
