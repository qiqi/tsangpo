"""
Tsangpo eSTOL — Central Parameter Repository.

Single source of truth.  SI units (m, kg, N, s, rad).

GEOMETRY VERSION 2 (design revision; previous v1 was untrim-feasible
because htail in heavy slipstream wake had near-zero authority).
Changes vs v1:
  * Origin is now the CG (was wing-root quarter-chord).
  * CG sits under the wing's HALF-chord (was quarter-chord) ⇒ wing
    shifts forward 0.25 c relative to CG ⇒ wing LE at x = -0.5 c.
  * Wing chord plane is 0.4 c ABOVE the CG (CG-to-wing offset unchanged
    in magnitude, just re-anchored).
  * H-tail leading edge is 4.0 c (was 3.5 c-c/4-to-c/4 ≈ 3.75 c LE-to-LE)
    aft of the wing LE; htail chord = wing chord (was ~0.62 c);
    htail span = 0.40 b_wing (was 0.35).  Bigger area, longer arm:
    tail-volume coefficient V_H ≈ 1.6 (was 0.76).
  * Props are unchanged relative to the wing.

Coordinate system: origin at CG; +X aft, +Y starboard, +Z up.
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

# Wing position relative to CG (origin) -------------------------------
# CG sits under the wing's half-chord and 0.4 c below the wing chord
# plane.  Therefore wing LE is 0.5 c forward of CG and wing chord plane
# is 0.4 c above CG.
WING_X_LE_M       = -0.5 * WING_MAC_M      # = -0.692 m (wing LE forward of CG)
WING_X_CQUARTER_M = -0.25 * WING_MAC_M     # wing c/4 location (for V_H bookkeeping)
WING_X_CHALF_M    = 0.0                    # by construction
WING_Z_M          = +0.4 * WING_MAC_M      # = +0.554 m (chord plane above CG)

# Weights ---------------------------------------------------------------
W_GROSS_N         = 11_565.0               # ≈ 2,600 lbf  (= 1,180 kgf)
WING_LOADING_NM2  = W_GROSS_N / WING_AREA_M2

# 10-prop distributed array (unchanged relative to wing) ---------------
N_PROPS              = 10
PROP_DIAM_M          = 1.067               # ≈ 3.5 ft
PROP_RADIUS_M        = PROP_DIAM_M / 2
A_DISK_PER_PROP_M2   = pi * PROP_RADIUS_M ** 2
DISK_LOADING_NM2     = (0.5 * W_GROSS_N) / (N_PROPS * A_DISK_PER_PROP_M2)

TW_DESIGN     = 0.5
T_PER_PROP_N  = TW_DESIGN * W_GROSS_N / N_PROPS

PROP_Y_NONDIM = (0.1, 0.3, 0.5, 0.7, 0.9)
PROP_Y_M      = tuple(eta * WING_SEMI_SPAN_M for eta in PROP_Y_NONDIM)
PROP_X_M      = WING_X_LE_M - 0.20 * WING_MAC_M   # 0.2 c ahead of wing LE  (= -0.70 c)
PROP_Z_M      = WING_Z_M    - 0.30 * WING_MAC_M   # 0.3 c below wing chord plane (= +0.10 c above CG)
PROP_HEIGHT_M = 0.10 * WING_MAC_M                 # disk thickness, set wide enough to resolve

# Flap / inboard gap ---------------------------------------------------
GAP_FRACTION_BASELINE = 0.0
GAP_FRACTION_PROPOSED = 0.35

# H-tail (v2: bigger, full-chord, longer arm) --------------------------
HTAIL_SPAN_FRAC   = 0.40                   # was 0.35
HTAIL_CHORD_FRAC  = 1.00                   # c_htail = c_wing (was ≈ 0.62)
HTAIL_SPAN_M      = HTAIL_SPAN_FRAC * WING_SPAN_M
HTAIL_CHORD_M     = HTAIL_CHORD_FRAC * WING_CHORD_M
HTAIL_AREA_M2     = HTAIL_SPAN_M * HTAIL_CHORD_M
HTAIL_AR          = HTAIL_SPAN_M / HTAIL_CHORD_M       # ≈ 3.2 (was 4.5)
HTAIL_MAC_M       = HTAIL_CHORD_M

# H-tail position: LE at 4 c_wing aft of wing LE; low htail in the wing
# chord plane, high htail 1.5 c above the wing chord plane.
X_TAIL_LE_M       = WING_X_LE_M + 4.0 * WING_MAC_M      # = +3.5 c_w
X_TAIL_CQUARTER_M = X_TAIL_LE_M + 0.25 * HTAIL_CHORD_M  # = +3.75 c_w
X_TAIL_DEFAULT_M  = X_TAIL_CQUARTER_M                   # for CFD use
Z_TAIL_LOW_CHORDS = 0.0                                 # offset above wing plane (= absolute +0.4 c)
Z_TAIL_HIGH_CHORDS = 1.5                                # offset above wing plane (= absolute +1.9 c)
Z_TAIL_LOW_M      = WING_Z_M + Z_TAIL_LOW_CHORDS  * WING_MAC_M
Z_TAIL_HIGH_M     = WING_Z_M + Z_TAIL_HIGH_CHORDS * WING_MAC_M

# Stability ratios -----------------------------------------------------
S_HTAIL_OVER_S_WING = HTAIL_AREA_M2 / WING_AREA_M2                                # ≈ 0.400
L_TAIL_M            = X_TAIL_CQUARTER_M - WING_X_CQUARTER_M                       # wing c/4 → htail c/4
TAIL_VOLUME_COEF    = (HTAIL_AREA_M2 * L_TAIL_M) / (WING_AREA_M2 * WING_MAC_M)    # ≈ 1.60

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

# Landing / steep-descent point (flap phase 2 deployed)
V_LANDING_M_S      = 12.86                 # 25 knots
ALT_LANDING_M      = ALT_CRUISE_M
RHO_LANDING_KG_M3  = RHO_CRUISE_KG_M3
Q_LANDING_PA       = 0.5 * RHO_LANDING_KG_M3 * V_LANDING_M_S ** 2
DESCENT_ANGLE_DEG  = -30.0                 # target descent (negative = downward) flight-path angle


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
          f"c = {HTAIL_CHORD_M:.3f} m   AR = {HTAIL_AR:.2f}")
    print(f"        S_ht/S_w = {S_HTAIL_OVER_S_WING:.3f}   "
          f"V_H = S_ht · ℓ_ht / (S_w · c_w) = {TAIL_VOLUME_COEF:.3f}")
    print(f"Weight  W = {W_GROSS_N:.0f} N  (≈ {W_GROSS_N/9.81:.0f} kgf)   "
          f"W/S = {WING_LOADING_NM2:.1f} N/m²")
    print(f"Props   N = {N_PROPS}  D = {PROP_DIAM_M:.3f} m  "
          f"T/prop_design = {T_PER_PROP_N:.0f} N  DL = {DISK_LOADING_NM2:.0f} N/m²")
    print(f"Cruise  V = {V_CRUISE_M_S:.2f} m/s  M = {V_CRUISE_M_S/A_SOUND_CRUISE_M_S:.3f}  "
          f"q = {Q_CRUISE_PA:.0f} Pa  Re_MAC = {RHO_CRUISE_KG_M3*V_CRUISE_M_S*WING_MAC_M/MU_CRUISE_PA_S:.2e}")
    print()
    print(f"Geometry v2 layout (origin = CG):")
    print(f"  wing  LE  at x = {WING_X_LE_M:+.3f} m,   c/4 at x = {WING_X_CQUARTER_M:+.3f} m")
    print(f"  wing  plane z = {WING_Z_M:+.3f} m  (CG is {abs(WING_Z_M):.3f} m below wing)")
    print(f"  htail LE  at x = {X_TAIL_LE_M:+.3f} m,   c/4 at x = {X_TAIL_CQUARTER_M:+.3f} m")
    print(f"  htail z   (low)  = {Z_TAIL_LOW_M:+.3f} m   (high) = {Z_TAIL_HIGH_M:+.3f} m")
    print(f"  prop  x = {PROP_X_M:+.3f} m,    z = {PROP_Z_M:+.3f} m")
    print()
    htail_tip = HTAIL_SPAN_M / 2
    for i, y in enumerate(PROP_Y_M, 1):
        loc = "blows H-tail" if y < htail_tip else "over flap"
        print(f"  prop {i}: y = {y:6.3f} m ({y / WING_SEMI_SPAN_M:.2f} b/2)  {loc}")
