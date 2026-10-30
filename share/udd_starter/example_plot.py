"""12-pane diagnostic plot for a UDD chunk.

For every measurable quantity Flow360 reports per pseudo iteration,
plot it vs cumulative pseudo-iter so you can eyeball:
  • whether the body's θ has settled
  • whether per-surface forces are converged
  • whether the actuator disk is forcing the body as expected
  • whether the residuals are decreasing (per physical-step gate)
  • whether the flow stays physical (min ρ, min p > 0)

Reads /tmp/udd_starter_ids.json written by example_submit.py.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import flow360 as fl
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

RESULTS = [
    "udd_pitch_cyl3int_airframe_v2.csv",
    "total_forces_v2.csv",
    "surface_forces_v2.csv",
    "actuatorDisk_force_coefficients_v2.csv",
    "nonlinear_residual_v2.csv",
    "minmax_state_v2.csv",
    "cfl_v2.csv",
]


def load_case(case_id: str) -> dict:
    """Download all per-iteration CSVs and the case manifest."""
    target = Path(f"/tmp/udd_starter_diag_{case_id[:13]}/")
    target.mkdir(parents=True, exist_ok=True)
    c = fl.Case.from_cloud(case_id=case_id)
    data = {}
    for fn in RESULTS:
        local = target / fn
        if not local.exists():
            try:
                c._download_file(f"results/{fn}", to_folder=str(target),
                                 overwrite=True)
            except Exception:
                data[fn] = None; continue
        data[fn] = pd.read_csv(local, sep=r"\s*,\s*", engine="python")
    # constants for the moment-balance conversion (see README "Pitfalls" #5, #6)
    try:
        c._download_file("flow360.json", to_folder=str(target), overwrite=True)
        j = json.loads((target / "flow360.json").read_text())
        data["constants"] = next(
            u["constants"] for u in j["userDefinedDynamics"]
            if u["dynamicsName"] == "pitch_cyl3int_airframe")
    except Exception:
        data["constants"] = {}
    return data


def plot_case(case_id: str, title_extra: str = "") -> Path:
    d = load_case(case_id)
    udd  = d["udd_pitch_cyl3int_airframe_v2.csv"]
    tot  = d["total_forces_v2.csv"]
    surf = d["surface_forces_v2.csv"]
    ad   = d["actuatorDisk_force_coefficients_v2.csv"]
    nl   = d["nonlinear_residual_v2.csv"]
    mm   = d["minmax_state_v2.csv"]
    cfl  = d["cfl_v2.csv"]
    consts = d.get("constants", {})

    # The coef ratio bridges UDD's solver-nondim moments and the airframe
    # coefficient form used by total_forces (see README "Pitfalls" #5).
    ratio = 1.0
    cmy_ad_coef = 0.0
    if tot is not None and udd is not None and consts:
        end_t = tot.groupby("physical_step").tail(1).reset_index(drop=True)
        end_u = udd.groupby("physical_step").tail(1).reset_index(drop=True)
        with np.errstate(divide="ignore", invalid="ignore"):
            r = end_t["CMy"].to_numpy() / end_u["momentY"].to_numpy()
            r = r[np.isfinite(r) & (np.abs(r) < 1000)]
            ratio = float(np.median(r)) if len(r) else 1.0
        cmy_ad_coef = -consts["prop_z_m"] * consts["T_disk_nd"] * ratio

    x = np.arange(len(udd)) if udd is not None else None
    fig, axes = plt.subplots(4, 3, figsize=(17, 12))
    fig.suptitle(f"UDD diag: {case_id[:13]}  {title_extra}", fontsize=11)

    # 1) θ_body
    ax = axes[0, 0]
    if udd is not None:
        ax.plot(x, np.degrees(udd["state[4]"]), lw=0.7)
        # mark physical-step boundaries
        for ps in udd["physical_step"].unique()[1:]:
            ax.axvline(udd.index[udd["physical_step"] == ps][0],
                       color="gray", alpha=0.2, lw=0.5)
    ax.set_title("θ_body (deg) [UDD state[4]]"); ax.grid(alpha=0.3)

    # 2) q (state[5])
    ax = axes[0, 1]
    if udd is not None: ax.plot(x, udd["state[5]"], lw=0.7)
    ax.axhline(0, color="k", alpha=0.3)
    ax.set_title("q (state[5], solver nondim)"); ax.grid(alpha=0.3)

    # 3) CL, CD (total)
    ax = axes[0, 2]
    if tot is not None:
        xt = np.arange(len(tot))
        ax.plot(xt, tot["CL"], lw=0.6, label="CL", color="C0")
        ax.plot(xt, tot["CD"], lw=0.6, label="CD", color="C3")
    ax.set_title("CL, CD (total wall)"); ax.legend(fontsize=8); ax.grid(alpha=0.3)

    # 4) Moment balance: wall CMy + AD reaction = Q_TORQUE
    # The sign of (wall + AD) is what drives θ — not the wall alone.
    ax = axes[1, 0]
    if tot is not None:
        xt = np.arange(len(tot))
        ax.plot(xt, tot["CMy"], lw=0.6, color="C0", label="CMy_wall")
        if consts:
            ax.axhline(cmy_ad_coef, color="C3", ls="--", lw=1.0,
                       label=f"CMy_AD = {cmy_ad_coef:+.3f}")
            ax.plot(xt, tot["CMy"].to_numpy() + cmy_ad_coef, lw=0.7,
                    color="C2", alpha=0.8, label="Q_TORQUE = wall + AD")
    ax.axhline(0, color="k", alpha=0.4)
    ax.set_title("Q_TORQUE (airframe coef = wall + AD reaction)")
    ax.legend(fontsize=7); ax.grid(alpha=0.3)

    # 5) Per-surface CL
    ax = axes[1, 1]
    if surf is not None:
        xs = np.arange(len(surf))
        for label, frag in [("main_wing", "main_wing_CL"),
                            ("htail",     "htail_CL"),
                            ("vane_R",    "vane_right_CL"),
                            ("vane_L",    "vane_left_CL"),
                            ("flap_R",    "aft_flap_right_CL"),
                            ("flap_L",    "aft_flap_left_CL")]:
            cols = [c for c in surf.columns if c.endswith(frag)]
            if cols: ax.plot(xs, surf[cols[0]], lw=0.6, label=label, alpha=0.8)
    ax.set_title("Per-surface CL"); ax.legend(fontsize=7, ncol=2); ax.grid(alpha=0.3)

    # 6) Per-surface CMy
    ax = axes[1, 2]
    if surf is not None:
        xs = np.arange(len(surf))
        for label, frag in [("main_wing", "main_wing_CMy"),
                            ("htail",     "htail_CMy"),
                            ("flap_R",    "aft_flap_right_CMy"),
                            ("flap_L",    "aft_flap_left_CMy")]:
            cols = [c for c in surf.columns if c.endswith(frag)]
            if cols: ax.plot(xs, surf[cols[0]], lw=0.6, label=label, alpha=0.8)
    ax.axhline(0, color="k", alpha=0.3)
    ax.set_title("Per-surface CMy"); ax.legend(fontsize=7); ax.grid(alpha=0.3)

    # 7) AD total CFx
    ax = axes[2, 0]
    if ad is not None:
        xa = np.arange(len(ad))
        sum_cfx = sum(ad[f"Disk{i}_CFx"] for i in range(10)
                      if f"Disk{i}_CFx" in ad.columns)
        ax.plot(xa, sum_cfx, lw=0.7, color="C2", label="Σ Disk_CFx")
    ax.set_title("Actuator disk total CFx"); ax.legend(fontsize=8); ax.grid(alpha=0.3)

    # 8) AD total CMy — raw + airframe-coef effective on right axis
    ax = axes[2, 1]
    if ad is not None:
        xa = np.arange(len(ad))
        sum_cmy = sum(ad[f"Disk{i}_CMy"] for i in range(10)
                      if f"Disk{i}_CMy" in ad.columns)
        ax.plot(xa, sum_cmy, lw=0.7, color="C2", alpha=0.6,
                label="Σ Disk_CMy (rotor units)")
        ax.legend(fontsize=7, loc="upper right")
        if consts:
            ax2 = ax.twinx()
            ax2.axhline(cmy_ad_coef, color="C3", ls="--", lw=1.2,
                        label=f"airframe coef = {cmy_ad_coef:+.3f}")
            ax2.set_ylabel("airframe coef", color="C3", fontsize=8)
            ax2.tick_params(axis="y", labelcolor="C3", labelsize=7)
            ax2.legend(fontsize=7, loc="lower right")
    ax.axhline(0, color="k", alpha=0.3); ax.grid(alpha=0.3)
    ax.set_title("Actuator disk total CMy  (two normalizations)")

    # 9) CG world + link angles
    ax = axes[2, 2]
    if udd is not None:
        ax.plot(x, udd["state[0]"], lw=0.6, label="CG_x = state[0]")
        ax.plot(x, udd["state[1]"], lw=0.6, label="CG_z = state[1]")
        ax.plot(x, np.degrees(udd["state[6]"]), lw=0.5, color="C3", alpha=0.6,
                label="link2 (deg)")
        ax.plot(x, np.degrees(udd["state[7]"]), lw=0.5, color="C4", alpha=0.6,
                label="link1 (deg)")
    ax.set_title("CG world (m) + link angles (deg)")
    ax.legend(fontsize=7); ax.grid(alpha=0.3)

    # 10) Nonlinear residuals
    ax = axes[3, 0]
    if nl is not None:
        xn = np.arange(len(nl))
        for col, lbl in [("0_cont","cont"), ("1_momx","momx"),
                         ("2_momy","momy"), ("3_momz","momz"),
                         ("4_energ","energ"), ("5_nuHat","nuHat")]:
            if col in nl.columns:
                ax.semilogy(xn, nl[col].clip(lower=1e-20),
                            lw=0.5, label=lbl, alpha=0.8)
    ax.set_title("Nonlinear residuals (log)")
    ax.legend(fontsize=7, ncol=2); ax.grid(alpha=0.3)

    # 11) CFL number
    ax = axes[3, 1]
    if cfl is not None:
        xc = np.arange(len(cfl))
        for col, lbl in [("0_NavierStokes_cfl","NS"),
                         ("1_SpalartAllmaras_cfl","SA")]:
            if col in cfl.columns:
                ax.plot(xc, cfl[col], lw=0.7, label=lbl)
    ax.set_title("CFL number"); ax.legend(fontsize=8); ax.grid(alpha=0.3)

    # 12) min ρ, min p, max |u|  — physical realizability
    ax = axes[3, 2]
    if mm is not None:
        xm = np.arange(len(mm))
        ax.plot(xm, mm["min_rho"],  lw=0.6, label="min ρ", color="C0")
        ax.plot(xm, mm["min_p"],    lw=0.6, label="min p", color="C3")
        ax.plot(xm, mm["max_umag"], lw=0.6, label="max |u|", color="C2")
    ax.set_title("min ρ, min p, max |u|")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    for ax in axes[-1, :]:
        ax.set_xlabel("cumulative pseudo-iteration")

    plt.tight_layout(rect=(0, 0, 1, 0.985))
    fn = Path(f"/tmp/udd_starter_diag_{case_id[:13]}.png")
    fig.savefig(fn, dpi=100)
    plt.close(fig)
    return fn


if __name__ == "__main__":
    ids = json.loads(Path("/tmp/udd_starter_ids.json").read_text())
    title = f"θ_ht={ids['theta_ht_deg']:+.2f}°  V={ids['V_inf_m_s']} m/s"
    fn = plot_case(ids["chunk"], title)
    print(f"plot → {fn}")
