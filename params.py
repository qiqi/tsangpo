"""
Tsangpo eSTOL — Central Parameter Repository.

Single source of truth. US customary units (ft, slug, lbf, s).
Origin at wing root quarter-chord on the symmetry plane; +X aft, +Y stbd, +Z up.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import pi, sqrt

# Wing — Hershey bar (rectangular, no taper/sweep/twist/dihedral) -------
WING_AREA_FT2     = 165.0
WING_AR           = 8.0
WING_SPAN_FT      = sqrt(WING_AR * WING_AREA_FT2)
WING_SEMI_SPAN_FT = WING_SPAN_FT / 2
WING_CHORD_FT     = WING_AREA_FT2 / WING_SPAN_FT
WING_MAC_FT       = WING_CHORD_FT

# Weights ---------------------------------------------------------------
W_GROSS_LBF       = 2600.0
WING_LOADING_PSF  = W_GROSS_LBF / WING_AREA_FT2

# 10-prop distributed array --------------------------------------------
N_PROPS              = 10
PROP_DIAM_FT         = 3.5
PROP_RADIUS_FT       = PROP_DIAM_FT / 2
A_DISK_PER_PROP_FT2  = pi * PROP_RADIUS_FT ** 2
DISK_LOADING_PSF     = (0.5 * W_GROSS_LBF) / (N_PROPS * A_DISK_PER_PROP_FT2)

TW_DESIGN     = 0.5
T_PER_PROP_LBF = TW_DESIGN * W_GROSS_LBF / N_PROPS

# 5 props per semi-span at the centers of 5 equal bays: 0.1, 0.3, 0.5, 0.7, 0.9
PROP_Y_NONDIM = (0.1, 0.3, 0.5, 0.7, 0.9)
PROP_Y_FT     = tuple(eta * WING_SEMI_SPAN_FT for eta in PROP_Y_NONDIM)
PROP_X_FT     = -0.30 * WING_MAC_FT
PROP_Z_FT     = -0.05 * WING_MAC_FT      # Electra-style: just below wing chord plane

# Flap / inboard gap ---------------------------------------------------
GAP_FRACTION_BASELINE = 0.0
GAP_FRACTION_PROPOSED = 0.35

# H-tail (Hershey bar) -------------------------------------------------
HTAIL_SPAN_FRAC = 0.35
HTAIL_AR        = 4.5
HTAIL_SPAN_FT   = HTAIL_SPAN_FRAC * WING_SPAN_FT
HTAIL_AREA_FT2  = HTAIL_SPAN_FT ** 2 / HTAIL_AR
HTAIL_CHORD_FT  = HTAIL_AREA_FT2 / HTAIL_SPAN_FT
HTAIL_MAC_FT    = HTAIL_CHORD_FT

X_TAIL_DEFAULT_FT     = 3.5 * WING_MAC_FT
Z_TAIL_HIGH_CHORDS    = 2.5    # T-tail, well above wake
Z_TAIL_LOW_CHORDS     = 0.0    # H-tail in the wing/flap wake plane

# Atmosphere — ISA, 12,000 ft ------------------------------------------
RHO_12K_SLUG_FT3   = 1.6480e-3
A_SOUND_12K_FT_S   = 1069.4
MU_12K_SLUG_FT_S   = 3.5343e-7

# Flight conditions ----------------------------------------------------
# High-lift design point (takeoff / landing, full flap deflection)
ALPHA_DESIGN_DEG = 10.0
V_INF_FT_S       = 80.0
Q_INF_PSF        = 0.5 * RHO_12K_SLUG_FT3 * V_INF_FT_S ** 2
MACH_INF         = V_INF_FT_S / A_SOUND_12K_FT_S
RE_MAC           = RHO_12K_SLUG_FT3 * V_INF_FT_S * WING_MAC_FT / MU_12K_SLUG_FT_S

# Cruise point (clean wing, stowed flap, level flight at altitude)
ALT_CRUISE_FT     = 12000.0
V_CRUISE_FT_S     = 150.0                # ~89 kts
ALPHA_CRUISE_DEG  = 3.0                  # nominal cruise α; UDD trims h-tail
Q_CRUISE_PSF      = 0.5 * RHO_12K_SLUG_FT3 * V_CRUISE_FT_S ** 2
T_CRUISE_TOTAL_LBF = Q_CRUISE_PSF * WING_AREA_FT2 * 0.04   # CD ≈ 0.04 initial guess
T_CRUISE_PER_PROP_LBF = T_CRUISE_TOTAL_LBF / N_PROPS


@dataclass(frozen=True)
class Case:
    name:           str
    label:          str            # human description used in the paper/CSV
    gap_fraction:   float
    z_tail_chords:  float
    x_tail_ft:      float = X_TAIL_DEFAULT_FT
    alpha_deg:      float = ALPHA_DESIGN_DEG
    tw:             float = TW_DESIGN


def study1_matrix() -> tuple[Case, ...]:
    """The 2x2 SciTech matrix: gap_fraction x Z_tail_chords.

        Case 1  industry_baseline    gap=0.00  z=+2.5  (stable, heavy)
        Case 2  downwash_failure     gap=0.00  z= 0.0  (unstable -- low tail in downwash)
        Case 3  bad_tradeoff         gap=0.35  z=+2.5  (stable, heavy, lift penalty)
        Case 4  proposed_synthesis   gap=0.35  z= 0.0  (stable, lightweight, agile)
    """
    return (
        Case("industry_baseline",   "Industry Baseline",   GAP_FRACTION_BASELINE, Z_TAIL_HIGH_CHORDS),
        Case("downwash_failure",    "Downwash Failure",    GAP_FRACTION_BASELINE, Z_TAIL_LOW_CHORDS),
        Case("bad_tradeoff",        "Bad Trade-off",       GAP_FRACTION_PROPOSED, Z_TAIL_HIGH_CHORDS),
        Case("proposed_synthesis",  "Proposed Synthesis",  GAP_FRACTION_PROPOSED, Z_TAIL_LOW_CHORDS),
    )


if __name__ == "__main__":
    print(f"Wing   S={WING_AREA_FT2:.1f} ft^2  b={WING_SPAN_FT:.2f} ft  "
          f"c={WING_CHORD_FT:.2f} ft  AR={WING_AR}")
    print(f"Weight W={W_GROSS_LBF:.0f} lbf  W/S={WING_LOADING_PSF:.2f} psf  "
          f"T/prop={T_PER_PROP_LBF:.1f} lbf  DL={DISK_LOADING_PSF:.2f} psf")
    print(f"H-tail b_ht={HTAIL_SPAN_FT:.2f} ft  S_ht={HTAIL_AREA_FT2:.2f} ft^2  "
          f"c_ht={HTAIL_CHORD_FT:.2f} ft")
    print(f"Flight V={V_INF_FT_S:.0f} ft/s  M={MACH_INF:.3f}  q={Q_INF_PSF:.2f} psf  "
          f"Re_MAC={RE_MAC:.2e}")
    print()
    htail_tip = HTAIL_SPAN_FT / 2
    for i, y in enumerate(PROP_Y_FT, 1):
        loc = "blows H-tail" if y < htail_tip else "over flap"
        print(f"  prop {i}: y={y:6.3f} ft ({y / WING_SEMI_SPAN_FT:.2f} b/2)  {loc}")
    print()
    for i, c in enumerate(study1_matrix(), 1):
        print(f"  Case {i}  {c.name:22s} gap={c.gap_fraction:.2f}  "
              f"z_t={c.z_tail_chords:+.2f} c   ({c.label})")
