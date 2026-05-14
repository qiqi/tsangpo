"""
Cruise-condition sensitivity post-processing — v2 geometry counterpart
to `plot_sweep_sensitivities.py`.  Same structure; differences:

  • PROJECT_ID + case-id lists point at the v2 cruise campaign
    (`tsangpo_v2_cruise` / prj-ee96bbf8).
  • DROPS the `+0.4·CFx` CMy_CG shift used in v1, because the v2 CFD
    has moment_center=(0,0,0)=CG (in v1 the origin was wing-root c/4
    and we had to translate down to the assumed CG at z=-0.4c).
  • KEEPS the `-0.1·CT_delivered` thrust contribution (thrust line is
    0.1 c above CG in v2, same as v1's assumed CG).
  • Outputs to `cruise_v2_sweep_data.csv` / `cruise_v2_sensitivities.png`
    / `cruise_v2_thrust_balance.png` so v1 outputs stay intact.

Geometry-v2 reference commit: a3b1d86.

    python3 post/plot_cruise_v2_sensitivities.py             # use cache if present
    python3 post/plot_cruise_v2_sensitivities.py --refresh   # refetch
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
import params as P

OUT = REPO / "post" / "out"
OUT.mkdir(exist_ok=True)

# Live v2 cruise project — commit e8d7c63 (legacy mesher + enclosed_entities fix).
PROJECT_ID     = "prj-16082511-6d6a-447e-8c54-828d476c0a85"   # tsangpo_v2_cruise (live)
PARENT_CASE_ID = "case-e30a9610-248f-40af-aa9f-9f30846c419d"  # BO point (α=+7°, θ_ht=0, T_mult=+1)

ALPHA_CASES = [
    (-3.0, "case-2e1796ec-8c78-407b-a899-87ec3aa129ad"),
    (-1.0, "case-cb9a32a6-ba86-4f1d-ad09-6148b7ad8a8b"),
    (+1.0, "case-101f50c3-0ecc-40c5-aa6e-0172a2645fe6"),
    (+3.0, "case-4b6825ad-30b4-40e0-880d-fe4f3eb8bd19"),
    (+5.0, "case-62a7fbe8-384f-4665-9b54-e6f063648fce"),
    (+7.0, "case-613a231c-4b0a-4069-bf02-1286b71c4a04"),   # BO α
    (+9.0, "case-a47e6871-a282-43d6-aef8-edaadafd49a8"),
    (+11.0,"case-09da495b-fbb9-4e48-9916-1a4610feb56d"),
    (+13.0,"case-9facaaf0-5d15-4bb5-bc7a-e8d8f8f73ff2"),
    (+15.0,"case-183d90aa-4e6f-4098-b6b1-127f2fce5ca1"),
]
HTAIL_CASES = [
    (-12.0,"case-da4a2f75-9395-4fc7-a534-f8eefa434278"),
    ( -9.0,"case-4e4241bd-5255-443b-bd46-9935d495af99"),
    ( -6.0,"case-0b14f0bf-5dc5-400f-8262-785fe825a9a4"),
    ( -3.0,"case-17d72a5c-8c81-4e68-b7e2-7858f26bf96f"),
    (  0.0, PARENT_CASE_ID),                              # BO θ_ht=0 (dedup → parent)
    ( +3.0,"case-f9a4f1ab-88bf-461a-9cc5-47f225fa20d4"),
    ( +6.0,"case-2e85577a-0177-4b99-89b4-a4e092ccb825"),
    ( +9.0,"case-58963732-6f49-450b-bb23-1928cbe89f02"),
    (+12.0,"case-15a66d25-6ab1-4096-bbbd-a87517e25bc0"),
    (+15.0,"case-2d8eee0c-c8c3-44d9-a9fe-001836f80faa"),
]
THRUST_CASES = [
    (0.00, "case-3b70fc15-df36-464f-970c-4f82a5ce8f52"),
    (0.25, "case-a71ab2e0-e428-4e25-b583-a444437c3469"),
    (0.50, "case-02d921bc-5068-40d9-96c8-d6ef07299014"),
    (0.75, "case-1b3946ca-5757-4254-93d4-dac110325054"),
    (1.00, PARENT_CASE_ID),                              # BO T_mult=1 (dedup → parent)
    (1.25, "case-0eb50376-ce67-47b9-b7d1-b783a0229ed1"),
    (1.50, "case-600e809a-f61d-4a7a-a058-bf15b95c4038"),
    (2.00, "case-8b604f09-e9ef-4666-9fe6-a0187834dbc2"),
    (2.50, "case-354681e9-3940-4a4f-8e8c-dc236719423a"),
    (3.00, "case-cbcf5285-6fdb-4680-8af3-84a039ab95f4"),
]

CSV_PATH = OUT / "cruise_v2_sweep_data.csv"
qS        = 0.5 * P.RHO_CRUISE_KG_M3 * P.V_CRUISE_M_S ** 2 * P.WING_AREA_M2
rho_a2_L2 = P.RHO_CRUISE_KG_M3 * P.A_SOUND_CRUISE_M_S ** 2 * 1.0 ** 2


def fetch_all() -> list[dict]:
    import flow360 as fl
    rows = []
    for sweep_name, cases in [("alpha", ALPHA_CASES),
                              ("htail", HTAIL_CASES),
                              ("thrust", THRUST_CASES)]:
        for val, cid in cases:
            try:
                c = fl.Case.from_cloud(cid)
                tf = c.results.total_forces; tf.load_from_remote()
                v = tf.values
                ps = np.array(v["physical_step"])
                last = int(np.where(ps == ps.max())[0][-1])
                try:
                    ad = c.results.actuator_disks; ad.load_from_remote()
                    av = ad.values
                    F_AD = sum(np.array(av[f"Disk{i}_Force"])[-1] for i in range(10)) * rho_a2_L2
                except Exception:
                    F_AD = float("nan")
                rows.append(dict(
                    sweep=sweep_name, value=val, case_id=cid,
                    CL=float(v["CL"][last]),
                    CD=float(v["CD"][last]),
                    CMy=float(v["CMy"][last]),
                    CFx=float(v["CFx"][last]),
                    F_AD_delivered_N=F_AD,
                    physical_step=int(ps[last]),
                    pseudo_step=int(v["pseudo_step"][last]),
                ))
                print(f"  {sweep_name:6s} {val:+6.2f}  {cid[:18]}  "
                      f"CL={rows[-1]['CL']:+.4f}  CD={rows[-1]['CD']:+.4f}  "
                      f"CMy={rows[-1]['CMy']:+.4f}  F_AD={F_AD:+.0f} N")
            except Exception as e:
                print(f"  {sweep_name:6s} {val:+6.2f}  {cid[:18]}  SKIP ({type(e).__name__})")
    return rows


def load_or_fetch(refresh: bool) -> list[dict]:
    if CSV_PATH.exists() and not refresh:
        with CSV_PATH.open() as f:
            return [
                {k: (float(v) if k in {"value", "CL", "CD", "CMy", "CFx", "F_AD_delivered_N"} else v)
                 for k, v in r.items()}
                for r in csv.DictReader(f)
            ]
    print("Fetching from Flow360 …")
    rows = fetch_all()
    with CSV_PATH.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        for r in rows: w.writerow(r)
    print(f"Wrote {CSV_PATH}")
    return rows


def by_sweep(rows: list[dict], name: str):
    sel = [r for r in rows if r["sweep"] == name]
    sel.sort(key=lambda r: r["value"])
    x = np.array([r["value"] for r in sel])
    data = {k: np.array([r[k] for r in sel])
            for k in ("CL", "CD", "CMy", "CFx", "F_AD_delivered_N")}
    CT_delivered = data["F_AD_delivered_N"] / qS
    # === v2 moment convention =========================================
    # CFD's moment_center=(0,0,0) IS the CG in v2 (was wing c/4 in v1,
    # where we added 0.4·CFx to translate down to the CG).  Drop that
    # shift; keep the thrust-line-above-CG contribution (-0.1·CT_del).
    data["CMy_CG"]       = data["CMy"] - 0.1 * CT_delivered
    data["CT_delivered"] = CT_delivered
    return x, data


def fit_linear(x, y, mask=None):
    if mask is None: mask = np.ones_like(x, dtype=bool)
    slope, intercept = np.polyfit(x[mask], y[mask], 1)
    return float(slope), float(intercept)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--refresh", action="store_true")
    args = ap.parse_args()
    rows = load_or_fetch(args.refresh)

    a, da = by_sweep(rows, "alpha")
    h, dh = by_sweep(rows, "htail")
    t, dt = by_sweep(rows, "thrust")

    mask_lin = a <= 9.0
    sens = {
        "dCL/dα   [/deg]":   fit_linear(a, da["CL"],     mask_lin)[0],
        "dCD/dα   [/deg]":   fit_linear(a, da["CD"],     mask_lin)[0],
        "dCMy/dα  [/deg]":   fit_linear(a, da["CMy_CG"], mask_lin)[0],
        "dCL/dθ_h [/deg]":   fit_linear(h, dh["CL"])[0],
        "dCD/dθ_h [/deg]":   fit_linear(h, dh["CD"])[0],
        "dCMy/dθ_h [/deg]":  fit_linear(h, dh["CMy_CG"])[0],
        "dCL/dT_m  [/unit]": fit_linear(t, dt["CL"])[0],
        "dCD/dT_m  [/unit]": fit_linear(t, dt["CD"])[0],
        "dCMy/dT_m [/unit]": fit_linear(t, dt["CMy_CG"])[0],
        "dCT_del/dT_m [/unit]": fit_linear(t, dt["CT_delivered"])[0],
    }
    print("\n=== Cruise v2 sensitivities (about α=+7°, θ_ht=0°, T_mult=1.0) ===")
    for k, v in sens.items(): print(f"  {k:24s} = {v:+.5f}")

    # Plots ----------------------------------------------------------------
    fig, axes = plt.subplots(3, 3, figsize=(13, 11))
    plt.subplots_adjust(left=0.08, right=0.97, top=0.93, bottom=0.07,
                        hspace=0.34, wspace=0.30)
    sweeps = [
        ("alpha",  a, da, r"$\alpha$ [deg]",
         "α sweep (θ_htail=0°, T_mult=1.0)"),
        ("htail",  h, dh, r"$\theta_{\rm htail}$ [deg]",
         "H-tail sweep (α=+7°, T_mult=1.0)"),
        ("thrust", t, dt, "T_mult",
         "Thrust sweep (α=+7°, θ_htail=0°)"),
    ]
    cols = [("CL", r"$C_L$"), ("CD", r"$C_D$"),
            ("CMy_CG", r"$C_{m,y}$  (about CG, incl. thrust)")]
    for row, (name, x, data, xlabel, title) in enumerate(sweeps):
        for col, (key, ylabel) in enumerate(cols):
            ax = axes[row, col]
            y = data[key]
            ax.plot(x, y, "o-", color="C0", lw=1.4, mfc="white", ms=6)
            ax.set_xlabel(xlabel); ax.set_ylabel(ylabel); ax.grid(True, alpha=0.3)
            mask = x <= 9.0 if name == "alpha" else np.ones_like(x, dtype=bool)
            if mask.sum() >= 2:
                slope, icpt = fit_linear(x, y, mask)
                xx = np.linspace(x.min(), x.max(), 50)
                ax.plot(xx, icpt + slope * xx, "--", color="C3", lw=0.9, alpha=0.7,
                        label=f"slope = {slope:+.4f}")
                ax.legend(fontsize=7, loc="best", framealpha=0.85)
            if col == 0: ax.set_title(title, loc="left", fontsize=9, pad=8)
    fig.suptitle("Tsangpo v2 CRUISE calibration sweeps (geometry commit a3b1d86)  "
                 "— BO α=+7°, V=45.72 m/s, level flight",
                 fontsize=11, y=0.995)
    fig.savefig(OUT / "cruise_v2_sensitivities.png", dpi=160)
    plt.close(fig)
    print(f"Wrote {OUT / 'cruise_v2_sensitivities.png'}")

    # Thrust-balance plot --------------------------------------------------
    fig2, ax = plt.subplots(1, 1, figsize=(7, 5))
    drag_N        = dt["CFx"] * qS
    cmd_thrust_N  = t * 10 * P.T_CRUISE_PER_PROP_N
    delivered_N   = dt["F_AD_delivered_N"]
    ax.plot(t, drag_N,       "o-", lw=1.5, mfc="white", label="aircraft drag (wall integral)")
    ax.plot(t, cmd_thrust_N, "s--", lw=1.0, alpha=0.7,   label="commanded AD thrust")
    if np.isfinite(delivered_N).all():
        ax.plot(t, delivered_N, "^-", lw=1.5, mfc="white",
                label="AD-delivered thrust (×ρ·a²·L²)")
    ax.set_xlabel("T_mult")
    ax.set_ylabel("force  [N]")
    ax.set_title(f"v2 cruise thrust vs drag balance, V={P.V_CRUISE_M_S:.1f} m/s",
                 fontsize=11)
    ax.grid(True, alpha=0.3); ax.legend()
    if np.isfinite(delivered_N).all():
        f = drag_N - delivered_N
        idx = np.where(np.diff(np.sign(f)))[0]
        if len(idx):
            i = idx[0]
            t_bal = t[i] - f[i] * (t[i+1] - t[i]) / (f[i+1] - f[i])
            ax.axvline(t_bal, color="C2", ls=":", alpha=0.6)
            ax.annotate(f"trim ≈ ×{t_bal:.2f}", (t_bal, drag_N.mean()),
                        xytext=(8, 0), textcoords="offset points",
                        fontsize=9, color="C2")
    fig2.tight_layout()
    fig2.savefig(OUT / "cruise_v2_thrust_balance.png", dpi=160)
    plt.close(fig2)
    print(f"Wrote {OUT / 'cruise_v2_thrust_balance.png'}")

    # Trim solve -----------------------------------------------------------
    # Cruise (γ=0): CL = W/qS, CMy_CG = 0, CFx = CT_delivered
    from math import cos, sin, radians
    def baseline_of(arr, x, x0):
        return float(arr[int(np.argmin(np.abs(x - x0)))])
    CL_base   = baseline_of(da["CL"], a, 7.0)
    CMy_base  = baseline_of(da["CMy_CG"], a, 7.0)
    CFx_base  = baseline_of(da["CFx"], a, 7.0)
    CT_base   = baseline_of(dt["CT_delivered"], t, 1.0)
    a_b = radians(7.0)
    print(f"\nBaseline v2 cruise (α=+7°, θ_ht=0, T_mult=1):")
    print(f"  CL={CL_base:+.4f}, CMy_CG={CMy_base:+.4f}, CFx={CFx_base:+.4f}, CT_del={CT_base:+.4f}")
    CL_target = P.W_GROSS_N / qS
    res_CL  = CL_target - CL_base
    res_CMy = 0.0 - CMy_base
    res_axial = 0.0 - (CT_base * cos(a_b) - CFx_base)
    dCL_a, dCL_h, dCL_T = sens["dCL/dα   [/deg]"], sens["dCL/dθ_h [/deg]"], sens["dCL/dT_m  [/unit]"]
    dCMy_a, dCMy_h, dCMy_T = sens["dCMy/dα  [/deg]"], sens["dCMy/dθ_h [/deg]"], sens["dCMy/dT_m [/unit]"]
    dCFx_a, dCFx_h, dCFx_T = sens["dCD/dα   [/deg]"], sens["dCD/dθ_h [/deg]"], sens["dCD/dT_m  [/unit]"]
    dCT_T = sens["dCT_del/dT_m [/unit]"]
    dAx_a = -CT_base * sin(a_b) - dCFx_a
    dAx_h = -dCFx_h
    dAx_T = dCT_T * cos(a_b) - dCFx_T
    A = np.array([[dCL_a, dCL_h, dCL_T], [dCMy_a, dCMy_h, dCMy_T], [dAx_a, dAx_h, dAx_T]])
    b = np.array([res_CL, res_CMy, res_axial])
    dx = np.linalg.solve(A, b)
    print(f"\nRefined cruise trim:  α={7.0+dx[0]:+.2f}°,  θ_ht={0.0+dx[1]:+.2f}°,  T_mult={1.0+dx[2]:+.2f}")
    sm = -dCMy_a / dCL_a
    print(f"v2 static margin (-dCMy_CG/dα / dCL/dα) = {sm:.3f}  (= {sm*100:.1f}% MAC)")


if __name__ == "__main__":
    main()
