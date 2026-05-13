"""
Tsangpo eSTOL — Central Parameter Repository.

Single source of truth.  SI units (m, kg, N, s, rad).
Origin at wing root quarter-chord on the symmetry plane;
+X aft, +Y starboard, +Z up.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import pi, sqrt

# Wing — Hershey bar (rectangular, no taper/sweep/twist/dihedral) -------
WING_AREA_M2      = 15.33                  # ≈ 165 ft²
WING_AR           = 8.0
WING_SPAN_M       = sqrt(WING_AR * WING_AREA_M2)
WING_SEMI_SPAN_M  = WING_SPAN_M / 2
WING_CHORD_M      = WING_AREA_M2 / WING_SPAN_M
WING_MAC_M        = WING_CHORD_M

# Weights ---------------------------------------------------------------
W_GROSS_N         = 11_565.0               # ≈ 2,600 lbf  (= 1,180 kgf)
WING_LOADING_NM2  = W_GROSS_N / WING_AREA_M2

# 10-prop distributed array --------------------------------------------
N_PROPS              = 10
PROP_DIAM_M          = 1.067               # ≈ 3.5 ft
PROP_RADIUS_M        = PROP_DIAM_M / 2
A_DISK_PER_PROP_M2   = pi * PROP_RADIUS_M ** 2
DISK_LOADING_NM2     = (0.5 * W_GROSS_N) / (N_PROPS * A_DISK_PER_PROP_M2)

TW_DESIGN     = 0.5
T_PER_PROP_N  = TW_DESIGN * W_GROSS_N / N_PROPS

# 5 props per semi-span at the centers of 5 equal bays: 0.1, 0.3, 0.5, 0.7, 0.9
PROP_Y_NONDIM = (0.1, 0.3, 0.5, 0.7, 0.9)
PROP_Y_M      = tuple(eta * WING_SEMI_SPAN_M for eta in PROP_Y_NONDIM)
PROP_X_M      = -0.45 * WING_MAC_M         # 0.2 c ahead of LE (LE @ x = -0.25 c)
PROP_Z_M      = -0.30 * WING_MAC_M         # 0.3 c below LE chord plane
PROP_HEIGHT_M = 0.10 * WING_MAC_M          # disk thickness, set wide enough to resolve

# Flap / inboard gap ---------------------------------------------------
GAP_FRACTION_BASELINE = 0.0
GAP_FRACTION_PROPOSED = 0.35

# H-tail (Hershey bar) -------------------------------------------------
HTAIL_SPAN_FRAC   = 0.35
HTAIL_AR          = 4.5
HTAIL_SPAN_M      = HTAIL_SPAN_FRAC * WING_SPAN_M
HTAIL_AREA_M2     = HTAIL_SPAN_M ** 2 / HTAIL_AR
HTAIL_CHORD_M     = HTAIL_AREA_M2 / HTAIL_SPAN_M
HTAIL_MAC_M       = HTAIL_CHORD_M

X_TAIL_DEFAULT_M  = 3.5 * WING_MAC_M       # tail c/4 from wing c/4 (= CG)
Z_TAIL_HIGH_CHORDS = 2.5                   # T-tail position, in c
Z_TAIL_LOW_CHORDS  = 0.0                   # H-tail in the wing/flap wake plane

# Stability ratios ----------------------------------------------------
S_HTAIL_OVER_S_WING = HTAIL_AREA_M2 / WING_AREA_M2
TAIL_VOLUME_COEF    = (HTAIL_AREA_M2 * X_TAIL_DEFAULT_M) / (WING_AREA_M2 * WING_MAC_M)

# Atmosphere — ISA, 3,658 m (≈ 12,000 ft) -----------------------------
RHO_CRUISE_KG_M3   = 0.8491
A_SOUND_CRUISE_M_S = 325.95
MU_CRUISE_PA_S     = 1.6928e-5

# Flight conditions ----------------------------------------------------
# High-lift design point (takeoff / landing, full flap deflection)
ALPHA_DESIGN_DEG = 10.0
V_INF_M_S        = 24.38                   # ≈ 80 ft/s
Q_INF_PA         = 0.5 * RHO_CRUISE_KG_M3 * V_INF_M_S ** 2
MACH_INF         = V_INF_M_S / A_SOUND_CRUISE_M_S
RE_MAC           = RHO_CRUISE_KG_M3 * V_INF_M_S * WING_MAC_M / MU_CRUISE_PA_S

# Cruise point (clean wing, stowed flap, level flight at altitude)
ALT_CRUISE_M       = 3658.0                # ≈ 12,000 ft
V_CRUISE_M_S       = 45.72                 # ≈ 150 ft/s (~89 kt)
ALPHA_CRUISE_DEG   = 3.0
Q_CRUISE_PA        = 0.5 * RHO_CRUISE_KG_M3 * V_CRUISE_M_S ** 2
T_CRUISE_TOTAL_N   = Q_CRUISE_PA * WING_AREA_M2 * 0.04   # CD ≈ 0.04 initial guess
T_CRUISE_PER_PROP_N = T_CRUISE_TOTAL_N / N_PROPS

# Takeoff / climb-out point (flap phase 1 deployed, blown lift from props)
V_TAKEOFF_M_S      = 18.0                  # 35 knots
ALT_TAKEOFF_M      = ALT_CRUISE_M          # same altitude (high-altitude STOL)
RHO_TAKEOFF_KG_M3  = RHO_CRUISE_KG_M3
Q_TAKEOFF_PA       = 0.5 * RHO_TAKEOFF_KG_M3 * V_TAKEOFF_M_S ** 2
CLIMB_ANGLE_DEG    = 30.0                  # target climb-out flight-path angle


@dataclass(frozen=True)
class Case:
    name:          str
    label:         str
    gap_fraction:  float
    z_tail_chords: float
    x_tail_m:      float = X_TAIL_DEFAULT_M
    alpha_deg:     float = ALPHA_DESIGN_DEG
    tw:            float = TW_DESIGN


def study1_matrix() -> tuple[Case, ...]:
    """The 2×2 SciTech matrix: gap_fraction × Z_tail_chords."""
    return (
        Case("industry_baseline",  "Industry Baseline",  GAP_FRACTION_BASELINE, Z_TAIL_HIGH_CHORDS),
        Case("downwash_failure",   "Downwash Failure",   GAP_FRACTION_BASELINE, Z_TAIL_LOW_CHORDS),
        Case("bad_tradeoff",       "Bad Trade-off",      GAP_FRACTION_PROPOSED, Z_TAIL_HIGH_CHORDS),
        Case("proposed_synthesis", "Proposed Synthesis", GAP_FRACTION_PROPOSED, Z_TAIL_LOW_CHORDS),
    )


if __name__ == "__main__":
    print(f"Wing    S = {WING_AREA_M2:.2f} m²   b = {WING_SPAN_M:.3f} m   "
          f"c = {WING_CHORD_M:.3f} m   AR = {WING_AR}")
    print(f"H-tail  S = {HTAIL_AREA_M2:.3f} m²  b = {HTAIL_SPAN_M:.3f} m  "
          f"c = {HTAIL_CHORD_M:.3f} m   AR = {HTAIL_AR}")
    print(f"        S_ht/S_w = {S_HTAIL_OVER_S_WING:.3f}   "
          f"V_H = S_ht · ℓ_ht / (S_w · c_w) = {TAIL_VOLUME_COEF:.3f}")
    print(f"Weight  W = {W_GROSS_N:.0f} N  (≈ {W_GROSS_N/9.81:.0f} kgf)   "
          f"W/S = {WING_LOADING_NM2:.1f} N/m²")
    print(f"Props   N = {N_PROPS}  D = {PROP_DIAM_M:.3f} m  "
          f"T/prop_design = {T_PER_PROP_N:.0f} N  DL = {DISK_LOADING_NM2:.0f} N/m²")
    print(f"Cruise  V = {V_CRUISE_M_S:.2f} m/s  M = {V_CRUISE_M_S/A_SOUND_CRUISE_M_S:.3f}  "
          f"q = {Q_CRUISE_PA:.0f} Pa  Re_MAC = {RHO_CRUISE_KG_M3*V_CRUISE_M_S*WING_MAC_M/MU_CRUISE_PA_S:.2e}")
    print()
    htail_tip = HTAIL_SPAN_M / 2
    for i, y in enumerate(PROP_Y_M, 1):
        loc = "blows H-tail" if y < htail_tip else "over flap"
        print(f"  prop {i}: y = {y:6.3f} m ({y / WING_SEMI_SPAN_M:.2f} b/2)  {loc}")
