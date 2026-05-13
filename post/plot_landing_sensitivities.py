"""
Landing (phase-2 flap) sensitivity post-processing — counterpart to
`plot_sweep_sensitivities.py` for cruise.

Pulls final-step total_forces + actuator_disks for the 29 v2 forks of
the landing coarse campaign on `prj-25133de4`, fits linear sensitivities
about the back-of-envelope baseline (α=+8°, θ_ht=−5°, T_mult=+16),
writes plots + CSV to post/out/.  Solves the 3×3 trim system for
γ=+30° climb-out.

    python3 post/plot_landing_sensitivities.py             # use cache if present
    python3 post/plot_landing_sensitivities.py --refresh   # refetch all 29 forks
"""
from __future__ import annotations

import argparse
import csv
import sys
from math import cos, radians, sin
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
import params as P

OUT = REPO / "post" / "out"
OUT.mkdir(exist_ok=True)

PROJECT_ID     = "prj-17a37c9d-fe5d-4454-88b6-b8c6c45f4721"
PARENT_CASE_ID = "case-9f8147fa-2872-4f0c-b41a-7aa1fe7318bb"

ALPHA_BASELINE_DEG    = +8.0
THETA_HT_BASELINE_DEG = -6.0
T_MULT_BASELINE       = +12.0

ALPHA_CASES = [
    (-2.0, "case-b8b89b25-77ad-41f6-ab96-dd6abd0ea26e"),
    (+2.0, "case-c77ec4e3-4efd-4536-a6ce-e5164c27b95e"),
    (+5.0, "case-cc5b367c-e4dc-4a37-8a4a-394b0a579d84"),
    (+8.0, "case-41ee6b25-be62-4503-89b4-dbcd9f334baa"),  # BO baseline (dedup)
    (+11.0,"case-01f7cab6-0643-4622-a904-17deb7f254c6"),
    (+14.0,"case-387f40df-a941-48d9-9613-2f8ae9deb00a"),
    (+17.0,"case-c7f2b811-a383-4076-b0dc-bce6212dba88"),
    (+20.0,"case-c14dd229-e9c4-4f94-89fc-c40d149b9f98"),
    (+25.0,"case-cd448854-a393-4369-8939-b9ad3f8beff5"),
    (+30.0,"case-514e40e7-830b-447b-bd22-c228ecb1d20b"),
]
HTAIL_CASES = [
    (-15.0,"case-84033a94-6e54-4778-9a7f-819cded75db6"),
    (-12.0,"case-235c14e1-db0a-4ba9-8bfc-4f1f957512f8"),
    ( -9.0,"case-3b2a4c9f-08a7-4e28-b813-ce42f6883d16"),
    ( -6.0,"case-41ee6b25-be62-4503-89b4-dbcd9f334baa"),  # BO baseline (dedup)
    ( -3.0,"case-b591e117-3bdd-4235-8d7b-c193eb24b9c1"),
    (  0.0,"case-3b065be7-180e-4848-87fb-dda3359ca02d"),
    ( +3.0,"case-4b830f23-b050-44ed-b430-045d38037cfb"),
    ( +6.0,"case-645343d1-b5fd-4873-8470-c4cb7992c0f8"),
    ( +9.0,"case-050490df-44dc-49ea-aefd-5e8343e16fd1"),
    (+12.0,"case-49a6fe19-b13e-4801-92e2-1505679b4d85"),
]
THRUST_CASES = [
    ( 4.0,"case-1d6d8c8b-54fb-494b-aece-fc1acf134def"),
    ( 7.0,"case-08d16ca6-9605-4ff9-bf12-582cff549970"),
    (10.0,"case-eb7ad909-3712-4520-a6d0-76a594048c79"),
    (12.0,"case-41ee6b25-be62-4503-89b4-dbcd9f334baa"),   # BO baseline (dedup)
    (14.0,"case-0742f620-e24d-4749-be13-ca707184f4cd"),
    (16.0,"case-2fd3e0e9-e5d8-4806-8304-acdb0092bef0"),
    (18.0,"case-8d2c2bdb-afd2-42bc-8b86-89bc34c7fc98"),
    (21.0,"case-a64b58a1-c6e7-4f41-ba1f-a0be2e91269c"),
    (25.0,"case-96451c6d-7f39-4f20-85f0-414f3ac03d78"),
    (30.0,"case-60c3b34d-cec7-4063-9bbb-6189ac340278"),
]

CSV_PATH = OUT / "landing_sweep_data.csv"

qS         = P.Q_LANDING_PA * P.WING_AREA_M2          # 2109 N
qSc        = qS * P.WING_MAC_M
rho_a2_L2  = P.RHO_LANDING_KG_M3 * P.A_SOUND_CRUISE_M_S ** 2 * 1.0 ** 2
gamma_rad  = radians(P.DESCENT_ANGLE_DEG)
W_cos_g    = P.W_GROSS_N * cos(gamma_rad)
W_sin_g    = P.W_GROSS_N * sin(gamma_rad)


def fetch_all() -> list[dict]:
    import flow360 as fl
    rows = []
    for sweep_name, cases in [("alpha",  ALPHA_CASES),
                              ("htail",  HTAIL_CASES),
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
                    F_AD_N = sum(np.array(av[f"Disk{i}_Force"])[-1]
                                 for i in range(10)) * rho_a2_L2
                except Exception:
                    F_AD_N = float("nan")
                rows.append(dict(
                    sweep=sweep_name, value=val, case_id=cid,
                    CL=float(v["CL"][last]),
                    CD=float(v["CD"][last]),
                    CMy=float(v["CMy"][last]),
                    CFx=float(v["CFx"][last]),
                    F_AD_delivered_N=F_AD_N,
                    physical_step=int(ps[last]),
                    pseudo_step=int(v["pseudo_step"][last]),
                ))
                print(f"  {sweep_name:6s} {val:+6.2f}  {cid[:18]}  "
                      f"CL={rows[-1]['CL']:+.4f}  CD={rows[-1]['CD']:+.4f}  "
                      f"CMy={rows[-1]['CMy']:+.4f}  F_AD={F_AD_N:+.0f} N")
            except Exception as e:
                print(f"  {sweep_name:6s} {val:+6.2f}  {cid[:18]}  "
                      f"SKIPPED ({type(e).__name__})")
    return rows


def load_or_fetch(refresh: bool) -> list[dict]:
    if CSV_PATH.exists() and not refresh:
        with CSV_PATH.open() as f:
            return [
                {k: (float(v) if k in {"value", "CL", "CD", "CMy", "CFx",
                                       "F_AD_delivered_N"} else v)
                 for k, v in r.items()}
                for r in csv.DictReader(f)
            ]
    print("Fetching from Flow360 …")
    rows = fetch_all()
    with CSV_PATH.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        for r in rows:
            w.writerow(r)
    print(f"Wrote {CSV_PATH}")
    return rows


def by_sweep(rows: list[dict], name: str):
    sel = [r for r in rows if r["sweep"] == name]
    sel.sort(key=lambda r: r["value"])
    x = np.array([r["value"] for r in sel])
    data = {k: np.array([r[k] for r in sel])
            for k in ("CL", "CD", "CMy", "CFx", "F_AD_delivered_N")}
    CT_delivered = data["F_AD_delivered_N"] / qS
    # CG at z=-0.4 c; thrust line at z=-0.3 c.
    # CMy_CG = CMy_origin + 0.4·CFx − 0.1·CT_delivered.
    data["CMy_CG"]       = data["CMy"] + 0.4 * data["CFx"] - 0.1 * CT_delivered
    data["CT_delivered"] = CT_delivered
    return x, data


def fit_linear(x, y, mask=None):
    if mask is None:
        mask = np.ones_like(x, dtype=bool)
    slope, intercept = np.polyfit(x[mask], y[mask], 1)
    return float(slope), float(intercept)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--refresh", action="store_true")
    args = ap.parse_args()
    rows = load_or_fetch(args.refresh)

    a, da = by_sweep(rows, "alpha")
    h, dh = by_sweep(rows, "htail")
    t, dt = by_sweep(rows, "thrust")

    # Pre-stall mask for the alpha fit — clearly CL flattens after ~+14°
    mask_lin = a <= 11.0
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
    print('\n=== Sensitivities (about α=+8°, θ_ht=-5°, T_mult=+16) ===')
    for k, v in sens.items():
        print(f"  {k:24s} = {v:+.5f}")

    # Plots — 3×3 grid -----------------------------------------------------
    fig, axes = plt.subplots(3, 3, figsize=(13, 11))
    plt.subplots_adjust(left=0.08, right=0.97, top=0.93, bottom=0.07,
                        hspace=0.34, wspace=0.30)
    sweeps = [
        ("alpha",  a, da, r"$\alpha$ [deg]",
         f"α sweep (θ_htail={THETA_HT_BASELINE_DEG:+.0f}°, T_mult={T_MULT_BASELINE:.0f})"),
        ("htail",  h, dh, r"$\theta_{\rm htail}$ [deg]",
         f"H-tail sweep (α={ALPHA_BASELINE_DEG:+.0f}°, T_mult={T_MULT_BASELINE:.0f})"),
        ("thrust", t, dt, "T_mult",
         f"Thrust sweep (α={ALPHA_BASELINE_DEG:+.0f}°, θ_htail={THETA_HT_BASELINE_DEG:+.0f}°)"),
    ]
    cols = [("CL", r"$C_L$"), ("CD", r"$C_D$"),
            ("CMy_CG", r"$C_{m,y}$  (CG at $z=-0.4c$, incl. thrust)")]
    for row, (name, x, data, xlabel, title) in enumerate(sweeps):
        for col, (key, ylabel) in enumerate(cols):
            ax = axes[row, col]
            y = data[key]
            ax.plot(x, y, "o-", color="C0", lw=1.4, mfc="white", ms=6)
            ax.set_xlabel(xlabel); ax.set_ylabel(ylabel)
            ax.grid(True, alpha=0.3)
            mask = x <= 11.0 if name == "alpha" else np.ones_like(x, dtype=bool)
            if mask.sum() >= 2:
                slope, icpt = fit_linear(x, y, mask)
                xx = np.linspace(x.min(), x.max(), 50)
                ax.plot(xx, icpt + slope * xx, "--", color="C3", lw=0.9, alpha=0.7,
                        label=f"slope = {slope:+.4f}")
                ax.legend(fontsize=7, loc="best", framealpha=0.85)
            if col == 0:
                ax.set_title(title, loc="left", fontsize=9, pad=8)
    fig.suptitle("Tsangpo LANDING (phase-2 flap) calibration sweeps  "
                 "— BO baseline α=+8°, θ_ht=−5°, T_mult=+16 — V=18 m/s, γ=+30°",
                 fontsize=11, y=0.995)
    fig.savefig(OUT / "landing_sensitivities.png", dpi=160)
    plt.close(fig)
    print(f"Wrote {OUT / 'landing_sensitivities.png'}")

    # Thrust-balance plot --------------------------------------------------
    fig2, ax = plt.subplots(1, 1, figsize=(7, 5))
    drag_N        = dt["CFx"] * qS
    cmd_thrust_N  = t * 10 * P.T_CRUISE_PER_PROP_N
    delivered_N   = dt["F_AD_delivered_N"]
    W_sin_arr     = np.full_like(t, W_sin_g)
    drag_plus_g_N = drag_N + W_sin_arr   # needed thrust to climb at γ
    ax.plot(t, drag_N,        "o-", lw=1.5, mfc="white", label="aircraft drag (wall integral)")
    ax.plot(t, drag_plus_g_N, "x-", lw=1.0, alpha=0.7,   label="drag + W·sin(γ) (needed thrust)")
    ax.plot(t, cmd_thrust_N,  "s--", lw=1.0, alpha=0.7,  label="commanded AD thrust")
    if np.isfinite(delivered_N).all():
        ax.plot(t, delivered_N, "^-", lw=1.5, mfc="white",
                label="AD-delivered thrust (×ρ·a²·L²)")
    ax.set_xlabel("thrust multiplier  (×T_CRUISE_PER_PROP_N)")
    ax.set_ylabel("force  [N]")
    ax.set_title(f"Landing thrust vs drag balance — γ={P.DESCENT_ANGLE_DEG:.0f}°, "
                 f"V={P.V_LANDING_M_S:.1f} m/s", fontsize=11)
    ax.grid(True, alpha=0.3); ax.legend()
    if np.isfinite(delivered_N).all():
        f = drag_plus_g_N - delivered_N
        idx = np.where(np.diff(np.sign(f)))[0]
        if len(idx):
            i = idx[0]
            x1, x2, f1, f2 = t[i], t[i + 1], f[i], f[i + 1]
            t_bal = x1 - f1 * (x2 - x1) / (f2 - f1)
            ax.axvline(t_bal, color="C2", ls=":", alpha=0.6)
            ax.annotate(f"trim ≈ ×{t_bal:.2f}", (t_bal, drag_plus_g_N.mean()),
                        xytext=(8, 0), textcoords="offset points",
                        fontsize=9, color="C2")
    fig2.tight_layout()
    fig2.savefig(OUT / "landing_thrust_balance.png", dpi=160)
    plt.close(fig2)
    print(f"Wrote {OUT / 'landing_thrust_balance.png'}")

    # 3×3 trim solve -------------------------------------------------------
    # Baseline values at (α=+8°, θ_ht=-5°, T_mult=+16)
    def baseline_of(arr, x, x0):
        idx = int(np.argmin(np.abs(x - x0)))
        return float(arr[idx])
    CL_base   = baseline_of(da["CL"],     a, ALPHA_BASELINE_DEG)
    CMy_base  = baseline_of(da["CMy_CG"], a, ALPHA_BASELINE_DEG)
    CFx_base  = baseline_of(da["CFx"],    a, ALPHA_BASELINE_DEG)
    CT_base   = baseline_of(dt["CT_delivered"], t, T_MULT_BASELINE)
    alpha_b   = radians(ALPHA_BASELINE_DEG)
    print(f"\nBaseline (α={ALPHA_BASELINE_DEG:+.0f}°, θ_ht={THETA_HT_BASELINE_DEG:+.0f}°, "
          f"T_mult={T_MULT_BASELINE:.1f}):")
    print(f"  CL_base   = {CL_base:+.4f}    CMy_CG_base = {CMy_base:+.4f}")
    print(f"  CFx_base  = {CFx_base:+.4f}    CT_del_base = {CT_base:+.4f}")
    # Target residuals at the baseline:
    CL_target_at_base  = (W_cos_g - CT_base * qS * sin(alpha_b)) / qS  # = (L - T·sinα)/qS but approximated at baseline α
    # Net axial constraint:  CT_delivered · cos(α) − CFx = W·sin(γ)/qS
    # At baseline, the residual:  RHS − LHS = W·sin(γ)/qS − (CT_base·cos(α_b) − CFx_base)
    res_CL   = CL_target_at_base - CL_base
    res_CMy  = 0.0 - CMy_base
    res_axial = (W_sin_g / qS) - (CT_base * cos(alpha_b) - CFx_base)
    print(f"\nResiduals to drive to zero:")
    print(f"  ΔCL    target − base = {res_CL:+.4f}")
    print(f"  ΔCMy   target − base = {res_CMy:+.4f}")
    print(f"  Δ(axial) target − base = {res_axial:+.4f}")
    # Linear Jacobian.  ΔCL and ΔCMy come directly from sensitivities.  The
    # axial constraint uses dCT_del/dT_mult and the dCFx/d... sensitivities.
    dCL_da, dCL_dh, dCL_dT = sens["dCL/dα   [/deg]"], sens["dCL/dθ_h [/deg]"], sens["dCL/dT_m  [/unit]"]
    dCMy_da, dCMy_dh, dCMy_dT = sens["dCMy/dα  [/deg]"], sens["dCMy/dθ_h [/deg]"], sens["dCMy/dT_m [/unit]"]
    dCFx_da, dCFx_dh, dCFx_dT = sens["dCD/dα   [/deg]"], sens["dCD/dθ_h [/deg]"], sens["dCD/dT_m  [/unit]"]
    dCT_dT = sens["dCT_del/dT_m [/unit]"]
    # d(axial)/dα = (dCT_del/dα)·cos(α_b) − CT_base·sin(α_b) − dCFx/dα
    # dCT_del/dα ≈ 0 (thrust is not a function of α in our scripts)
    dAxial_da = -CT_base * sin(alpha_b) - dCFx_da   # /deg α
    dAxial_dh = -dCFx_dh                            # /deg θ_ht
    dAxial_dT = dCT_dT * cos(alpha_b) - dCFx_dT     # /unit T_mult
    A = np.array([
        [dCL_da,    dCL_dh,    dCL_dT],
        [dCMy_da,   dCMy_dh,   dCMy_dT],
        [dAxial_da, dAxial_dh, dAxial_dT],
    ])
    b = np.array([res_CL, res_CMy, res_axial])
    print(f"\nJacobian:\n{A}\nResidual vector:\n{b}")
    dx = np.linalg.solve(A, b)
    print(f"\nΔα = {dx[0]:+.3f}°,  Δθ_ht = {dx[1]:+.3f}°,  ΔT_mult = {dx[2]:+.3f}")
    trim = (ALPHA_BASELINE_DEG + dx[0],
            THETA_HT_BASELINE_DEG + dx[1],
            T_MULT_BASELINE + dx[2])
    print(f"\nRefined trim: α = {trim[0]:+.2f}°, θ_ht = {trim[1]:+.2f}°, "
          f"T_mult = {trim[2]:+.2f}")


if __name__ == "__main__":
    main()
