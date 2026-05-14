"""
Takeoff (phase-1 flap) sensitivity post-processing — v2 geometry
counterpart to `plot_takeoff_sensitivities.py`.

Pulls final-step total_forces + actuator_disks for the 29 v2 forks of
the takeoff coarse campaign on `prj-9cd3ad10`, fits linear sensitivities
about the BO baseline (α=+8°, θ_ht=−5°, T_mult=+16), writes plots +
CSV to post/out/.  Solves the 3×3 trim system for γ=+30° climb-out.

v2 differences from v1:
  * PROJECT_ID + case-id lists point at the v2 takeoff campaign
    (commit e8d7c63 — legacy mesher + enclosed_entities fix).
  * DROPS the `+0.4·CFx` CMy_CG shift (v2 origin IS the CG).
  * KEEPS the `-0.1·CT_delivered` thrust contribution.
  * Outputs to `takeoff_v2_sweep_data.csv` / `takeoff_v2_sensitivities.png` /
    `takeoff_v2_thrust_balance.png` so v1 outputs stay intact.

Geometry-v2 reference commit: a3b1d86.

    python3 post/plot_takeoff_v2_sensitivities.py             # use cache if present
    python3 post/plot_takeoff_v2_sensitivities.py --refresh   # refetch all 29 forks
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

PROJECT_ID     = "prj-9cd3ad10-ea47-41d9-9e33-0096fc30d6c1"   # tsangpo_v2_takeoff_coarse
PARENT_CASE_ID = "case-118aced8-e380-45a5-b374-04a303f1802a"  # takeoff_coarse_BO_estimate

ALPHA_BASELINE_DEG    = +8.0
THETA_HT_BASELINE_DEG = -5.0
T_MULT_BASELINE       = +16.0

ALPHA_CASES = [
    (-2.0, "case-bbaaceb6-44c9-43dd-9f35-13bd03a78019"),
    (+2.0, "case-b0ee68d2-b798-47fc-8e0c-77d19641958b"),
    (+5.0, "case-54822df6-5716-45c2-a492-de13f82c76fc"),
    (+8.0, "case-6f827029-1d41-4c5f-b088-15b45acb5961"),   # BO α (=θ_ht=-5°/T_mult=16 baseline)
    (+11.0,"case-35914c04-aeb3-4a61-b9cf-0f6a31cba6e5"),
    (+14.0,"case-e1c6f304-bada-43de-b421-b2161f48f67b"),
    (+17.0,"case-de93ad58-2d50-49fa-8f13-1702317d8720"),
    (+20.0,"case-0bc7abd8-b043-4e7f-9dfd-0fbf272d2738"),
    (+25.0,"case-abcd69b8-e88a-4f4a-811d-ef4e12f312d4"),
    (+30.0,"case-38fbc526-6137-4f97-9f12-8856d4b6d419"),
]
HTAIL_CASES = [
    (-15.0,"case-d1419e53-b81d-431e-9cf5-cf3b71967593"),
    (-10.0,"case-cb26521c-20fc-428d-a6e2-6eece28b084b"),
    ( -5.0, PARENT_CASE_ID),                              # BO θ_ht=-5° (dedup → parent)
    (  0.0,"case-e2708773-8fbc-49ec-985c-be74dc980065"),
    ( +5.0,"case-61006241-3051-49fe-a2c0-db98274e353a"),
    (+10.0,"case-879ae2eb-4f73-452a-8e2e-fbf62eb840ec"),
    (+15.0,"case-91f43738-d65a-4e85-8bdd-bcd0d0551f7e"),
    (+20.0,"case-c688177d-245a-472b-b786-b55a12b4bcc6"),
    (+25.0,"case-a0020ec0-846a-4148-8598-1a9a080e7f54"),
    (+30.0,"case-75011804-b973-4a06-a354-271f14b144e0"),
]
THRUST_CASES = [
    ( 6.0,"case-ae12056a-0be1-4a8d-b04e-f5d9dfd6ba95"),
    ( 9.0,"case-a668891c-503e-44af-b0c3-92e98af6669c"),
    (12.0,"case-fc4cdebc-f9b1-485f-84af-a6d8edd8070d"),
    (14.0,"case-99f54b8a-7212-4e7e-a58f-650b256da5b2"),
    (16.0, PARENT_CASE_ID),                              # BO T_mult=16 (dedup → parent)
    (18.0,"case-807609eb-ce17-47e3-8a9a-673cd469db99"),
    (20.0,"case-6c63626d-b85e-4b3c-99fd-7396f04986c8"),
    (22.0,"case-20003ef9-ddc3-4772-9536-de867b11e832"),
    (25.0,"case-ca8e4f30-7a1e-4185-ba13-8d8a63ca9619"),
    (30.0,"case-58ef4498-23a8-4f6a-b57e-939374ac2ff1"),
]

CSV_PATH = OUT / "takeoff_v2_sweep_data.csv"

qS         = P.Q_TAKEOFF_PA * P.WING_AREA_M2          # 2109 N
qSc        = qS * P.WING_MAC_M
rho_a2_L2  = P.RHO_TAKEOFF_KG_M3 * P.A_SOUND_CRUISE_M_S ** 2 * 1.0 ** 2
gamma_rad  = radians(P.CLIMB_ANGLE_DEG)
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
    # === v2 moment convention =========================================
    # CFD's moment_center=(0,0,0) IS the CG in v2 (was wing c/4 in v1,
    # where we added +0.4·CFx to translate down to CG).  DROP that shift.
    # KEEP the thrust-line-above-CG contribution (thrust line at +0.1c
    # above CG, so engine thrust contributes a nose-down moment of
    # -0.1·CT_delivered).
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

    # Pre-stall mask for the alpha fit — CL flattens after ~+14° in v1; check.
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
    print('\n=== Takeoff v2 sensitivities (about α=+8°, θ_ht=-5°, T_mult=+16) ===')
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
    fig.suptitle("Tsangpo v2 TAKEOFF (phase-1 flap) calibration sweeps  "
                 "— BO baseline α=+8°, θ_ht=−5°, T_mult=+16 — V=18 m/s, γ=+30°",
                 fontsize=11, y=0.995)
    fig.savefig(OUT / "takeoff_v2_sensitivities.png", dpi=160)
    plt.close(fig)
    print(f"Wrote {OUT / 'takeoff_v2_sensitivities.png'}")

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
    ax.set_title(f"v2 takeoff thrust vs drag balance — γ={P.CLIMB_ANGLE_DEG:.0f}°, "
                 f"V={P.V_TAKEOFF_M_S:.1f} m/s", fontsize=11)
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
    fig2.savefig(OUT / "takeoff_v2_thrust_balance.png", dpi=160)
    plt.close(fig2)
    print(f"Wrote {OUT / 'takeoff_v2_thrust_balance.png'}")

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
    try:
        dx = np.linalg.solve(A, b)
    except np.linalg.LinAlgError as e:
        print(f"\nTrim solve FAILED (singular Jacobian: {e})")
        print(f"Condition number: {np.linalg.cond(A):.2e}")
        return
    print(f"\nΔα = {dx[0]:+.3f}°,  Δθ_ht = {dx[1]:+.3f}°,  ΔT_mult = {dx[2]:+.3f}")
    trim = (ALPHA_BASELINE_DEG + dx[0],
            THETA_HT_BASELINE_DEG + dx[1],
            T_MULT_BASELINE + dx[2])
    print(f"\nRefined v2 takeoff trim: α = {trim[0]:+.2f}°, θ_ht = {trim[1]:+.2f}°, "
          f"T_mult = {trim[2]:+.2f}")
    sm = -sens["dCMy/dα  [/deg]"] / sens["dCL/dα   [/deg]"]
    print(f"v2 takeoff static margin (-dCMy_CG/dα / dCL/dα) = {sm:.3f}  (= {sm*100:.1f}% MAC)")


if __name__ == "__main__":
    main()
