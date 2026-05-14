"""
Takeoff (flap phase 1) sensitivity post-processing — gapped-flap geometry
counterpart to `post/v2_continuous/plot_takeoff_sensitivities.py`.

Auto-discovers sweep case IDs from project `prj-e0e11ed5` by name pattern
`gap40_TO_{sweep}_{tok}` and the parent case (`*_parent`).

    python3 post/v2_gapped/plot_takeoff_sensitivities.py [--refresh]
"""
from __future__ import annotations
import argparse, csv, sys
from math import cos, radians, sin
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
import params as P
sys.path.insert(0, str(Path(__file__).resolve().parent))
from _common import discover_sweep_cases

OUT = REPO / "post" / "out" / "v2_gapped"
OUT.mkdir(parents=True, exist_ok=True)

PROJECT_ID  = "prj-e0e11ed5-2f3a-4e7d-8af7-c79baaf3ed22"   # tsangpo_v2_takeoff_gapped40
ALPHA_B, THETA_HT_B, T_B = +8.0, -5.0, +16.0
NAME_PREFIX = "gap40_TO"

CSV_PATH  = OUT / "takeoff_sweep_data.csv"
qS        = P.Q_TAKEOFF_PA * P.WING_AREA_M2
rho_a2_L2 = P.RHO_TAKEOFF_KG_M3 * P.A_SOUND_CRUISE_M_S ** 2 * 1.0 ** 2
gamma_rad = radians(P.CLIMB_ANGLE_DEG)
W_cos_g   = P.W_GROSS_N * cos(gamma_rad)
W_sin_g   = P.W_GROSS_N * sin(gamma_rad)


def fetch_all():
    import flow360 as fl
    sweeps = discover_sweep_cases(PROJECT_ID, NAME_PREFIX, ALPHA_B, THETA_HT_B, T_B)
    rows = []
    for sweep_name in ("alpha", "htail", "thrust"):
        for val, cid in sweeps[sweep_name]:
            try:
                c = fl.Case.from_cloud(cid)
                tf = c.results.total_forces; tf.load_from_remote(); v = tf.values
                ps = np.array(v["physical_step"]); last = int(np.where(ps == ps.max())[0][-1])
                try:
                    ad = c.results.actuator_disks; ad.load_from_remote()
                    av = ad.values
                    F_AD = sum(np.array(av[f"Disk{i}_Force"])[-1] for i in range(10)) * rho_a2_L2
                except Exception:
                    F_AD = float("nan")
                rows.append(dict(sweep=sweep_name, value=val, case_id=cid,
                                 CL=float(v["CL"][last]), CD=float(v["CD"][last]),
                                 CMy=float(v["CMy"][last]), CFx=float(v["CFx"][last]),
                                 F_AD_delivered_N=F_AD,
                                 physical_step=int(ps[last]),
                                 pseudo_step=int(v["pseudo_step"][last])))
                print(f"  {sweep_name:6s} {val:+6.2f}  {cid[:18]}  "
                      f"CL={rows[-1]['CL']:+.4f}  CMy={rows[-1]['CMy']:+.4f}  F_AD={F_AD:+.0f}")
            except Exception as e:
                print(f"  {sweep_name:6s} {val:+6.2f}  {cid[:18]}  SKIP ({type(e).__name__})")
    return rows


def load_or_fetch(refresh):
    if CSV_PATH.exists() and not refresh:
        with CSV_PATH.open() as f:
            return [{k: (float(v) if k in {"value","CL","CD","CMy","CFx","F_AD_delivered_N"} else v)
                     for k, v in r.items()} for r in csv.DictReader(f)]
    print("Fetching …")
    rows = fetch_all()
    if rows:
        with CSV_PATH.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader()
            for r in rows: w.writerow(r)
        print(f"Wrote {CSV_PATH}")
    return rows


def by_sweep(rows, name):
    sel = sorted([r for r in rows if r["sweep"] == name], key=lambda r: r["value"])
    x = np.array([r["value"] for r in sel])
    data = {k: np.array([r[k] for r in sel]) for k in ("CL","CD","CMy","CFx","F_AD_delivered_N")}
    CT_delivered = data["F_AD_delivered_N"] / qS
    data["CMy_CG"] = data["CMy"] - 0.1 * CT_delivered
    data["CT_delivered"] = CT_delivered
    return x, data


def fit_linear(x, y, mask):
    slope, intercept = np.polyfit(x[mask], y[mask], 1)
    return float(slope), float(intercept)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--refresh", action="store_true")
    args = ap.parse_args()
    rows = load_or_fetch(args.refresh)
    if not rows:
        print("No data — sweep cases not yet complete."); return

    a, da = by_sweep(rows, "alpha")
    h, dh = by_sweep(rows, "htail")
    t, dt = by_sweep(rows, "thrust")

    # Masks — start with continuous-flap values; adjust later as gap40 stall shifts.
    mask_a = a <= 11.0
    mask_h = (h >= 0.0) & (h <= 25.0)
    mask_t = t <= 22.0

    sens = {
        "dCL/dα   [/deg]":   fit_linear(a, da["CL"],     mask_a)[0],
        "dCD/dα   [/deg]":   fit_linear(a, da["CD"],     mask_a)[0],
        "dCMy/dα  [/deg]":   fit_linear(a, da["CMy_CG"], mask_a)[0],
        "dCL/dθ_h [/deg]":   fit_linear(h, dh["CL"],     mask_h)[0],
        "dCD/dθ_h [/deg]":   fit_linear(h, dh["CD"],     mask_h)[0],
        "dCMy/dθ_h [/deg]":  fit_linear(h, dh["CMy_CG"], mask_h)[0],
        "dCL/dT_m  [/unit]": fit_linear(t, dt["CL"],     mask_t)[0],
        "dCD/dT_m  [/unit]": fit_linear(t, dt["CD"],     mask_t)[0],
        "dCMy/dT_m [/unit]": fit_linear(t, dt["CMy_CG"], mask_t)[0],
        "dCT_del/dT_m [/unit]": fit_linear(t, dt["CT_delivered"], mask_t)[0],
    }
    print('\n=== Takeoff gap40 sensitivities (about α=+8°, θ_ht=-5°, T_mult=+16) ===')
    for k, v in sens.items(): print(f"  {k:24s} = {v:+.5f}")

    # Plots --------------------------------------------------------------------
    fig, axes = plt.subplots(3, 3, figsize=(13, 11))
    plt.subplots_adjust(left=0.08, right=0.97, top=0.93, bottom=0.07, hspace=0.34, wspace=0.30)
    sweeps_grid = [
        ("alpha",  a, da, r"$\alpha$ [deg]",
         f"α sweep (θ_htail={THETA_HT_B:+.0f}°, T_mult={T_B:.0f})"),
        ("htail",  h, dh, r"$\theta_{\rm htail}$ [deg]",
         f"H-tail sweep (α={ALPHA_B:+.0f}°, T_mult={T_B:.0f})"),
        ("thrust", t, dt, "T_mult",
         f"Thrust sweep (α={ALPHA_B:+.0f}°, θ_htail={THETA_HT_B:+.0f}°)"),
    ]
    cols = [("CL", r"$C_L$"), ("CD", r"$C_D$"), ("CMy_CG", r"$C_{m,y}$  (CG, incl. thrust)")]
    sweep_masks = {"alpha": mask_a, "htail": mask_h, "thrust": mask_t}
    for row, (name, x, data, xlabel, title) in enumerate(sweeps_grid):
        for col, (key, ylabel) in enumerate(cols):
            ax = axes[row, col]; y = data[key]; mask = sweep_masks[name]
            ax.plot(x[mask], y[mask], "o-", color="C0", lw=1.4, mfc="white", ms=6, label="fit pts")
            if (~mask).any():
                ax.plot(x[~mask], y[~mask], "x", color="C7", ms=7, label="stalled/saturated")
            ax.set_xlabel(xlabel); ax.set_ylabel(ylabel); ax.grid(True, alpha=0.3)
            if mask.sum() >= 2:
                slope, icpt = fit_linear(x, y, mask)
                xx = np.linspace(x[mask].min(), x[mask].max(), 50)
                ax.plot(xx, icpt + slope * xx, "--", color="C3", lw=0.9, alpha=0.7,
                        label=f"slope = {slope:+.4f}")
                ax.legend(fontsize=7, loc="best", framealpha=0.85)
            if col == 0: ax.set_title(title, loc="left", fontsize=9, pad=8)
    fig.suptitle("Tsangpo v2 GAPPED40 TAKEOFF calibration sweeps "
                 f"— BO α=+8°, θ_ht=-5°, T_mult=+16 — V=18 m/s, γ=+30°",
                 fontsize=11, y=0.995)
    fig.savefig(OUT / "takeoff_sensitivities.png", dpi=160); plt.close(fig)
    print(f"Wrote {OUT / 'takeoff_sensitivities.png'}")

    fig2, ax = plt.subplots(1, 1, figsize=(7, 5))
    drag_N = dt["CFx"] * qS; cmd_thrust_N = t * 10 * P.T_CRUISE_PER_PROP_N
    delivered_N = dt["F_AD_delivered_N"]
    drag_plus_g_N = drag_N + np.full_like(t, W_sin_g)
    ax.plot(t, drag_N,        "o-", lw=1.5, mfc="white", label="aircraft drag")
    ax.plot(t, drag_plus_g_N, "x-", lw=1.0, alpha=0.7,   label="drag + W·sin(γ)")
    ax.plot(t, cmd_thrust_N,  "s--", lw=1.0, alpha=0.7,  label="commanded AD")
    if np.isfinite(delivered_N).all():
        ax.plot(t, delivered_N, "^-", lw=1.5, mfc="white", label="AD delivered")
    ax.set_xlabel("T_mult"); ax.set_ylabel("force [N]")
    ax.set_title(f"gap40 takeoff thrust vs drag — γ={P.CLIMB_ANGLE_DEG:.0f}°, V={P.V_TAKEOFF_M_S:.1f} m/s")
    ax.grid(True, alpha=0.3); ax.legend()
    fig2.tight_layout(); fig2.savefig(OUT / "takeoff_thrust_balance.png", dpi=160); plt.close(fig2)
    print(f"Wrote {OUT / 'takeoff_thrust_balance.png'}")

    # Trim -----
    def baseline_of(arr, x, x0):
        return float(arr[int(np.argmin(np.abs(x - x0)))])
    CL_b  = baseline_of(da["CL"], a, ALPHA_B)
    CMy_b = baseline_of(da["CMy_CG"], a, ALPHA_B)
    CFx_b = baseline_of(da["CFx"], a, ALPHA_B)
    CT_b  = baseline_of(dt["CT_delivered"], t, T_B)
    a_b   = radians(ALPHA_B)
    print(f"\nBaseline: CL={CL_b:+.4f} CMy_CG={CMy_b:+.4f} CFx={CFx_b:+.4f} CT_del={CT_b:+.4f}")
    res = np.array([(W_cos_g - CT_b * qS * sin(a_b))/qS - CL_b, -CMy_b,
                    (W_sin_g/qS) - (CT_b*cos(a_b) - CFx_b)])
    dCL  = [sens["dCL/dα   [/deg]"], sens["dCL/dθ_h [/deg]"], sens["dCL/dT_m  [/unit]"]]
    dCMy = [sens["dCMy/dα  [/deg]"], sens["dCMy/dθ_h [/deg]"], sens["dCMy/dT_m [/unit]"]]
    dCFx = [sens["dCD/dα   [/deg]"], sens["dCD/dθ_h [/deg]"], sens["dCD/dT_m  [/unit]"]]
    dCT_T = sens["dCT_del/dT_m [/unit]"]
    dAx  = [-CT_b*sin(a_b) - dCFx[0], -dCFx[1], dCT_T*cos(a_b) - dCFx[2]]
    A = np.array([dCL, dCMy, dAx])
    try:
        dx = np.linalg.solve(A, res)
        sm = -dCMy[0]/dCL[0]
        print(f"Refined gap40 takeoff trim: α={ALPHA_B+dx[0]:+.2f}°, θ_ht={THETA_HT_B+dx[1]:+.2f}°, T_mult={T_B+dx[2]:+.2f}")
        print(f"gap40 static margin = {sm:.3f}  ({sm*100:.1f}% MAC)")
    except np.linalg.LinAlgError as e:
        print(f"trim solve failed: {e}; cond={np.linalg.cond(A):.2e}")


if __name__ == "__main__":
    main()
