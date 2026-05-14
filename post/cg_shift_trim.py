"""
CG-shift trim study.  Given the existing v2 sensitivities and BO baselines
from `*_v2_sweep_data.csv`, re-solve the 3×3 trim system at a NEW CG
that has been moved AFT by Δx (in chord-fractions) to target a chosen
cruise static margin.

The shift affects the pitching-moment sensitivities (and the baseline
CMy_CG itself) via the parallel-axis transfer:
    CMy_CG_new = CMy_CG_old + (Δx/c) · CFz_at_origin
    dCMy_new/dα = dCMy_old/dα + (Δx/c) · dCFz/dα   ≈  + (Δx/c) · dCL/dα
    dCMy_new/dθh = dCMy_old/dθh + (Δx/c) · dCL/dθh
    dCMy_new/dT   = dCMy_old/dT   + (Δx/c) · dCL/dT

(CFz ≈ CL for the small body-rotations we're working at — α ∈ [-3, +30]°
gives cos α > 0.866.)

CL, CD/CFx and CT_delivered sensitivities are unchanged (they don't
depend on where the moment_center is).  The thrust line stays at
+0.1c above the CG (the −0.1·CT contribution in CMy_CG is unchanged by
horizontal CG shifts).

    python3 post/cg_shift_trim.py            # default Δx/c chosen for SM_cruise = 10%
    python3 post/cg_shift_trim.py 0.30       # explicit Δx/c
"""
from __future__ import annotations

import csv, sys
from math import cos, radians, sin
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
import params as P

OUT = REPO / "post" / "out" / "v2_continuous"   # this study uses continuous-flap CSVs


def load_rows(phase: str) -> list[dict]:
    csvp = OUT / f"{phase}_sweep_data.csv"
    with csvp.open() as f:
        rows = []
        for r in csv.DictReader(f):
            r["value"] = float(r["value"])
            for k in ("CL", "CD", "CMy", "CFx", "F_AD_delivered_N"):
                r[k] = float(r[k])
            rows.append(r)
    return rows


def fit_linear(x, y, mask):
    slope, _ = np.polyfit(x[mask], y[mask], 1)
    return float(slope)


def by_sweep(rows, name, qS):
    sel = sorted([r for r in rows if r["sweep"] == name], key=lambda r: r["value"])
    x   = np.array([r["value"] for r in sel])
    data = {k: np.array([r[k] for r in sel])
            for k in ("CL", "CD", "CMy", "CFx", "F_AD_delivered_N")}
    data["CT_delivered"] = data["F_AD_delivered_N"] / qS
    # v2 CMy about CG with thrust-line offset of -0.1c (above-CG thrust line).
    data["CMy_CG"] = data["CMy"] - 0.1 * data["CT_delivered"]
    return x, data


def baseline(arr, x, x0):
    return float(arr[int(np.argmin(np.abs(x - x0)))])


def trim_one(phase: str, dx_over_c: float):
    # Phase-specific BO baselines and flight constants.
    if phase == "cruise":
        ALPHA_B, THETA_HT_B, T_B = +7.0, 0.0,   +1.0
        qS = 0.5 * P.RHO_CRUISE_KG_M3 * P.V_CRUISE_M_S**2 * P.WING_AREA_M2
        gamma_rad = 0.0
        W_cos_g = P.W_GROSS_N; W_sin_g = 0.0
        mask_a = lambda a: a <= 9.0
        mask_h = lambda h: np.ones_like(h, dtype=bool)
        mask_t = lambda t: np.ones_like(t, dtype=bool)
    elif phase == "takeoff":
        ALPHA_B, THETA_HT_B, T_B = +8.0, -5.0,  +16.0
        qS = P.Q_TAKEOFF_PA * P.WING_AREA_M2
        gamma_rad = radians(P.CLIMB_ANGLE_DEG)
        W_cos_g = P.W_GROSS_N * cos(gamma_rad)
        W_sin_g = P.W_GROSS_N * sin(gamma_rad)
        mask_a = lambda a: a <= 11.0
        mask_h = lambda h: (h >= 0.0) & (h <= 25.0)
        mask_t = lambda t: t <= 22.0
    elif phase == "landing":
        ALPHA_B, THETA_HT_B, T_B = +8.0, -6.0,  +12.0
        qS = P.Q_LANDING_PA * P.WING_AREA_M2
        gamma_rad = radians(P.DESCENT_ANGLE_DEG)
        W_cos_g = P.W_GROSS_N * cos(gamma_rad)
        W_sin_g = P.W_GROSS_N * sin(gamma_rad)
        mask_a = lambda a: a <= 8.0
        mask_h = lambda h: (h >= 10.0) & (h <= 40.0)
        mask_t = lambda t: t <= 25.0
    else:
        raise ValueError(phase)

    rows = load_rows(phase)
    a, da = by_sweep(rows, "alpha",  qS)
    h, dh = by_sweep(rows, "htail",  qS)
    t, dt = by_sweep(rows, "thrust", qS)

    Ma, Mh, Mt = mask_a(a), mask_h(h), mask_t(t)
    s = {
        "dCL_da":  fit_linear(a, da["CL"],     Ma),
        "dCFx_da": fit_linear(a, da["CFx"],    Ma),
        "dCMy_da": fit_linear(a, da["CMy_CG"], Ma),
        "dCL_dh":  fit_linear(h, dh["CL"],     Mh),
        "dCFx_dh": fit_linear(h, dh["CFx"],    Mh),
        "dCMy_dh": fit_linear(h, dh["CMy_CG"], Mh),
        "dCL_dT":  fit_linear(t, dt["CL"],     Mt),
        "dCFx_dT": fit_linear(t, dt["CFx"],    Mt),
        "dCMy_dT": fit_linear(t, dt["CMy_CG"], Mt),
        "dCT_dT":  fit_linear(t, dt["CT_delivered"], Mt),
    }

    CL_b   = baseline(da["CL"],     a, ALPHA_B)
    CMy_b  = baseline(da["CMy_CG"], a, ALPHA_B)
    CFx_b  = baseline(da["CFx"],    a, ALPHA_B)
    CT_b   = baseline(dt["CT_delivered"], t, T_B)

    # CG shift transfer (Δx aft → CMy_CG more nose-up by Δx·CL/c).
    dCMy_da_new = s["dCMy_da"] + dx_over_c * s["dCL_da"]
    dCMy_dh_new = s["dCMy_dh"] + dx_over_c * s["dCL_dh"]
    dCMy_dT_new = s["dCMy_dT"] + dx_over_c * s["dCL_dT"]
    CMy_b_new   = CMy_b + dx_over_c * CL_b

    SM_new = -dCMy_da_new / s["dCL_da"]
    SM_old = -s["dCMy_da"] / s["dCL_da"]

    # Trim residuals (same convention as the per-phase plotters).
    alpha_b = radians(ALPHA_B)
    CL_target_at_base = (W_cos_g - CT_b * qS * sin(alpha_b)) / qS
    res_CL    = CL_target_at_base - CL_b
    res_CMy   = 0.0 - CMy_b_new
    res_axial = (W_sin_g / qS) - (CT_b * cos(alpha_b) - CFx_b)

    dAx_da = -CT_b * sin(alpha_b) - s["dCFx_da"]
    dAx_dh = -s["dCFx_dh"]
    dAx_dT = s["dCT_dT"] * cos(alpha_b) - s["dCFx_dT"]
    A = np.array([
        [s["dCL_da"],  s["dCL_dh"],  s["dCL_dT"]],
        [dCMy_da_new, dCMy_dh_new,   dCMy_dT_new],
        [dAx_da,      dAx_dh,        dAx_dT],
    ])
    b = np.array([res_CL, res_CMy, res_axial])
    dx_state = np.linalg.solve(A, b)
    return dict(
        phase=phase, SM_old=SM_old, SM_new=SM_new,
        alpha_b=ALPHA_B, theta_ht_b=THETA_HT_B, T_b=T_B,
        alpha_trim=ALPHA_B + dx_state[0],
        theta_ht_trim=THETA_HT_B + dx_state[1],
        T_trim=T_B + dx_state[2],
        CL_b=CL_b, CMy_b_old=CMy_b, CMy_b_new=CMy_b_new,
        sens_old=s,
        dCMy_da_new=dCMy_da_new, dCMy_dh_new=dCMy_dh_new, dCMy_dT_new=dCMy_dT_new,
    )


def main():
    # Δx/c so that cruise SM goes from current to target.  From cruise sweep:
    # SM_target = SM_current - Δx/c → Δx/c = SM_current - SM_target.
    SM_target = 0.10
    cruise_baseline = trim_one("cruise", 0.0)
    SM_current = cruise_baseline["SM_new"]      # at dx=0 this equals SM_old
    if len(sys.argv) > 1:
        dx_over_c = float(sys.argv[1])
    else:
        dx_over_c = SM_current - SM_target
    print(f"SM_current = {SM_current:+.4f}  →  target SM_cruise = {SM_target:+.4f}")
    print(f"Δx/c = {dx_over_c:+.4f}  (CG moves AFT by Δx = {dx_over_c * P.WING_MAC_M:.4f} m)")

    results = []
    for ph in ("cruise", "takeoff", "landing"):
        r = trim_one(ph, dx_over_c)
        results.append(r)
        print(f"\n=== {ph} ===")
        print(f"  SM (old → new) : {r['SM_old']:+.4f} → {r['SM_new']:+.4f}")
        print(f"  CMy_b (old→new): {r['CMy_b_old']:+.4f} → {r['CMy_b_new']:+.4f}")
        print(f"  Refined trim   : α = {r['alpha_trim']:+.2f}°,  "
              f"θ_ht = {r['theta_ht_trim']:+.2f}°,  T_mult = {r['T_trim']:+.2f}")
    return dx_over_c, results


if __name__ == "__main__":
    main()
