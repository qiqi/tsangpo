"""
Landing (phase-2 flap) sensitivity post-processing — v2 geometry
counterpart to `plot_landing_sensitivities.py`.

Pulls final-step total_forces + actuator_disks for the 30 v2 forks of
the landing coarse campaign on `prj-e7dc7d6d`, fits linear sensitivities
about the BO baseline (α=+8°, θ_ht=−6°, T_mult=+12), writes plots + CSV
to post/out/.  Solves the 3×3 trim system for γ=−30° steep descent.

v2 differences from v1:
  * PROJECT_ID + case-id lists point at the v2 landing campaign
    (commit e8d7c63 — legacy mesher + enclosed_entities fix).
  * htail sweep extended to **(−10..+50)** (was −15..+12 in v1) to
    bracket the positive-deflection unstall region — v1's whole sweep
    was inside the stalled regime due to ~−45° wing+flap downwash.
  * DROPS the `+0.4·CFx` CMy_CG shift (v2 origin IS the CG).
  * KEEPS the `-0.1·CT_delivered` thrust contribution.
  * Outputs to `landing_v2_*.csv/png` so v1 outputs stay intact.

    python3 post/plot_landing_v2_sensitivities.py             # use cache if present
    python3 post/plot_landing_v2_sensitivities.py --refresh   # refetch
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

PROJECT_ID     = "prj-e7dc7d6d-4baf-4101-8818-1173da5a359a"   # tsangpo_v2_landing_coarse
PARENT_CASE_ID = "case-fed9edf1-681b-49fa-b4e9-c0513aaf987a"  # landing_coarse_BO_estimate

ALPHA_BASELINE_DEG    = +8.0
THETA_HT_BASELINE_DEG = -6.0
T_MULT_BASELINE       = +12.0

ALPHA_CASES = [
    (-2.0, "case-4a19cd87-a347-4f80-b831-86edf3e2bd54"),
    (+2.0, "case-76eed5ea-fda2-473b-8024-28704d84d76d"),
    (+5.0, "case-cd6044ac-26d1-4136-94dc-d3c380c3cbda"),
    (+8.0, "case-8b38ad8f-14b2-48f7-aca2-39b17bc6da78"),   # BO α
    (+11.0,"case-92522e46-b8ef-425c-981a-db5ed847fe2b"),
    (+14.0,"case-c8336d21-09e9-4019-9baa-272472a9f7ec"),
    (+17.0,"case-65daeabe-7e44-4d37-804c-88f2a5837538"),
    (+20.0,"case-c580fb93-3a01-4134-81d8-af900524e1e5"),
    (+25.0,"case-629ff60d-dbc3-4208-9d4e-2eaa154dc8ba"),
    (+30.0,"case-dad7046b-053e-4b48-894a-c81f608a14a2"),
]
HTAIL_CASES = [
    (-10.0,"case-39abf737-b29f-4332-99d5-ab29b5257bf2"),
    ( -6.0, PARENT_CASE_ID),                              # BO θ_ht=-6° (dedup → parent)
    (  0.0,"case-e06f2fad-a56d-4f94-9be2-8591af07b3e2"),
    (+10.0,"case-dd6d1f20-eb82-4cd3-a84e-8f9ab0431137"),
    (+15.0,"case-1f31d69e-b590-4c1f-8746-c3467b3b4e58"),
    (+20.0,"case-b8a7f63a-627e-4030-a850-94c77980d8c1"),
    (+25.0,"case-e574996e-c559-4690-b49a-9d16cc1ebc9c"),
    (+30.0,"case-60004125-a27f-4f1d-8c01-963edd25fff1"),
    (+35.0,"case-cfe4d654-7a4d-4efe-bb69-afabc73e7c50"),
    (+40.0,"case-8708f424-66fc-480a-bd4f-dbaadf4b006b"),
    (+50.0,"case-0c69ed66-46bd-474f-8c5b-45dae0808d9e"),
]
THRUST_CASES = [
    ( 4.0,"case-0a798775-bcb4-4cc5-817b-685c9794a10a"),
    ( 7.0,"case-3a024fd9-03b6-488c-83d3-a82f9db98e3f"),
    (10.0,"case-efe1b3f9-a493-4d4f-99b8-984ee8528ea9"),
    (12.0, PARENT_CASE_ID),                              # BO T_mult=12 (dedup → parent)
    (14.0,"case-f84530b3-6e4b-4ab2-a832-4771aae1822c"),
    (16.0,"case-3c26f654-347d-482d-be1c-a03c4468c1f2"),
    (18.0,"case-010c5da7-65ea-4da7-bcf5-934be80e522a"),
    (21.0,"case-cb363951-64a1-4f9c-b445-f9627ab22423"),
    (25.0,"case-2d0c0737-c507-4d84-983d-9f251b477ad7"),
    (30.0,"case-d1077fe0-7c79-47cc-8a05-d1049285132b"),
]

CSV_PATH = OUT / "landing_v2_sweep_data.csv"

qS         = P.Q_LANDING_PA * P.WING_AREA_M2
qSc        = qS * P.WING_MAC_M
rho_a2_L2  = P.RHO_LANDING_KG_M3 * P.A_SOUND_CRUISE_M_S ** 2 * 1.0 ** 2
gamma_rad  = radians(P.DESCENT_ANGLE_DEG)              # -30° (negative)
W_cos_g    = P.W_GROSS_N * cos(gamma_rad)
W_sin_g    = P.W_GROSS_N * sin(gamma_rad)              # negative — descent component


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
    # v2 moment convention: moment_center IS the CG; only the thrust-line
    # offset contribution (-0.1·CT_del) remains.
    data["CMy_CG"]       = data["CMy"] - 0.1 * CT_delivered
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

    # Pre-stall mask for the alpha fit
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
    print('\n=== Landing v2 sensitivities (about α=+8°, θ_ht=-6°, T_mult=+12) ===')
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
            ("CMy_CG", r"$C_{m,y}$  (about CG, incl. thrust)")]
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
    fig.suptitle("Tsangpo v2 LANDING (phase-2 flap) calibration sweeps  "
                 f"— BO baseline α=+8°, θ_ht=−6°, T_mult=+12 — "
                 f"V={P.V_LANDING_M_S} m/s, γ={P.DESCENT_ANGLE_DEG:+.0f}°",
                 fontsize=11, y=0.995)
    fig.savefig(OUT / "landing_v2_sensitivities.png", dpi=160)
    plt.close(fig)
    print(f"Wrote {OUT / 'landing_v2_sensitivities.png'}")

    # Thrust-balance plot --------------------------------------------------
    fig2, ax = plt.subplots(1, 1, figsize=(7, 5))
    drag_N        = dt["CFx"] * qS
    cmd_thrust_N  = t * 10 * P.T_CRUISE_PER_PROP_N
    delivered_N   = dt["F_AD_delivered_N"]
    W_sin_arr     = np.full_like(t, W_sin_g)
    drag_plus_g_N = drag_N + W_sin_arr   # γ<0: W_sin_g<0 → gravity ASSISTS the descent thrust
    ax.plot(t, drag_N,        "o-", lw=1.5, mfc="white", label="aircraft drag (wall integral)")
    ax.plot(t, drag_plus_g_N, "x-", lw=1.0, alpha=0.7,   label="drag + W·sin(γ) (needed thrust)")
    ax.plot(t, cmd_thrust_N,  "s--", lw=1.0, alpha=0.7,  label="commanded AD thrust")
    if np.isfinite(delivered_N).all():
        ax.plot(t, delivered_N, "^-", lw=1.5, mfc="white",
                label="AD-delivered thrust (×ρ·a²·L²)")
    ax.set_xlabel("thrust multiplier  (×T_CRUISE_PER_PROP_N)")
    ax.set_ylabel("force  [N]")
    ax.set_title(f"v2 landing thrust vs drag balance — γ={P.DESCENT_ANGLE_DEG:.0f}°, "
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
    fig2.savefig(OUT / "landing_v2_thrust_balance.png", dpi=160)
    plt.close(fig2)
    print(f"Wrote {OUT / 'landing_v2_thrust_balance.png'}")

    # 3×3 trim solve -------------------------------------------------------
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
    CL_target_at_base = (W_cos_g - CT_base * qS * sin(alpha_b)) / qS
    res_CL    = CL_target_at_base - CL_base
    res_CMy   = 0.0 - CMy_base
    res_axial = (W_sin_g / qS) - (CT_base * cos(alpha_b) - CFx_base)
    print(f"\nResiduals to drive to zero:")
    print(f"  ΔCL    target − base = {res_CL:+.4f}")
    print(f"  ΔCMy   target − base = {res_CMy:+.4f}")
    print(f"  Δ(axial) target − base = {res_axial:+.4f}")
    dCL_da, dCL_dh, dCL_dT = sens["dCL/dα   [/deg]"], sens["dCL/dθ_h [/deg]"], sens["dCL/dT_m  [/unit]"]
    dCMy_da, dCMy_dh, dCMy_dT = sens["dCMy/dα  [/deg]"], sens["dCMy/dθ_h [/deg]"], sens["dCMy/dT_m [/unit]"]
    dCFx_da, dCFx_dh, dCFx_dT = sens["dCD/dα   [/deg]"], sens["dCD/dθ_h [/deg]"], sens["dCD/dT_m  [/unit]"]
    dCT_dT = sens["dCT_del/dT_m [/unit]"]
    dAxial_da = -CT_base * sin(alpha_b) - dCFx_da
    dAxial_dh = -dCFx_dh
    dAxial_dT = dCT_dT * cos(alpha_b) - dCFx_dT
    A = np.array([
        [dCL_da,    dCL_dh,    dCL_dT],
        [dCMy_da,   dCMy_dh,   dCMy_dT],
        [dAxial_da, dAxial_dh, dAxial_dT],
    ])
    b = np.array([res_CL, res_CMy, res_axial])
    print(f"\nJacobian:\n{A}\nResidual vector:\n{b}")
    print(f"cond(A) = {np.linalg.cond(A):.3e}")
    try:
        dx = np.linalg.solve(A, b)
    except np.linalg.LinAlgError as e:
        print(f"\nTrim solve FAILED (singular Jacobian: {e})")
        return
    print(f"\nΔα = {dx[0]:+.3f}°,  Δθ_ht = {dx[1]:+.3f}°,  ΔT_mult = {dx[2]:+.3f}")
    trim = (ALPHA_BASELINE_DEG + dx[0],
            THETA_HT_BASELINE_DEG + dx[1],
            T_MULT_BASELINE + dx[2])
    print(f"\nRefined v2 landing trim: α = {trim[0]:+.2f}°, θ_ht = {trim[1]:+.2f}°, "
          f"T_mult = {trim[2]:+.2f}")
    sm = -sens["dCMy/dα  [/deg]"] / sens["dCL/dα   [/deg]"]
    print(f"v2 landing static margin (-dCMy_CG/dα / dCL/dα) = {sm:.3f}  (= {sm*100:.1f}% MAC)")


if __name__ == "__main__":
    main()
