"""Two-panel figure for the dynamic-maneuver section of the paper.

Left:  whole-arm view of the four-cylinder robot-arm topology in the world
       frame (cyl 1 shoulder, cyl 2 elbow, cyl 3 airframe, cyl 4 htail).
Right: side view of the v3 airframe (the parts that exist in the
       simulation are drawn solid; the fuselage belly and landing gear,
       which the simulation omits, are dotted).  Both panels show the
       full set of four nested rotation cylinders, each in a distinct
       colour: cyl 1 grey, cyl 2 orange, cyl 3 blue, cyl 4 magenta.

All text labels, titles, and axis labels are intentionally suppressed;
the caption carries that information.
"""
from __future__ import annotations
import importlib.util, sys
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.patches as mp

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(REPO))
import params as P

spec = importlib.util.spec_from_file_location("bcm", HERE / "_build_config_montage.py")
bcm = importlib.util.module_from_spec(spec); spec.loader.exec_module(bcm)

c    = P.WING_MAC_M

# Cylinder constants are read from flow360/unsteady_setup.py (section 1) by
# regex rather than imported, because importing that module pulls in the
# Flow360 SDK, which the offline paper build does not require.  An earlier
# version of this script hard-coded 0.10 c_w interface gaps; the production
# mesh uses 110 c_w (see the gap-sizing rule in unsteady_setup.py).
import re as _re
_src = (REPO / "flow360" / "unsteady_setup.py").read_text()
def _const(name: str) -> float:
    m = _re.search(rf"^{name}\s*=\s*([0-9.]+)", _src, _re.M)
    if not m:
        raise RuntimeError(f"{name} not found in flow360/unsteady_setup.py")
    return float(m.group(1))
R_ARM_C     = _const("R_ARM_C")        # 750 c_w
R_GAP_2_3_C = _const("R_GAP_2_3_C")    # 110 c_w
R_GAP_1_2_C = _const("R_GAP_1_2_C")    # 110 c_w
R_CYL3_C    = _const("R_CYL3_C")       # 2.90 c_w
R_CYL4_C    = _const("R_CYL4_C")       # 0.85 c_w
L1 = R_ARM_C * c
L2 = R_ARM_C * c
R_CYL3 = R_CYL3_C * c
R_CYL4 = R_CYL4_C * c
R_CYL2 = L2 + R_CYL3 + R_GAP_2_3_C * c
R_CYL1 = L1 + R_CYL2 + R_GAP_1_2_C * c

AIRFRAME_X = +L1
AIRFRAME_Z = -L2
ELBOW_X    = 0.0
ELBOW_Z    = -L1

# Cylinder colours (consistent across both panels)
COL_CYL1 = "#777777"  # grey   — shoulder
COL_CYL2 = "#cc7a00"  # orange — elbow
COL_CYL3 = "#1f4ea8"  # blue   — airframe-pitch
COL_CYL4 = "#a02080"  # magenta — htail-pitch

# v3 htail c/4 pivot location in body frame (chord units, z above CG).
# In body coords (origin at CG, +x toward tail, +z up):
#   wing LE  at x = -0.5 c
#   wing TE  at x = +0.5 c
#   htail LE at x = +1.5 c   (= wing LE + 2 c spacing for v3)
#   htail c/4 at x = +1.75 c, z = +0.4 c (low htail)
#   htail TE at x = +2.5 c
# Cyl_4 (R = 0.85 c) centred at (1.75, 0.4) ⇒ outer at x = +2.60 c
#   → 0.10 c gap from htail TE on the +x side (design intent).
X_HTAIL_PIVOT = +1.75
Z_HTAIL_PIVOT = +0.4

fig = plt.figure(figsize=(12.5, 5.4), constrained_layout=False)
gs  = fig.add_gridspec(1, 2, width_ratios=[1.05, 1.0], wspace=0.04,
                       left=0.02, right=0.99, top=0.99, bottom=0.02)
axL = fig.add_subplot(gs[0, 0])
axR = fig.add_subplot(gs[0, 1])

# ===== LEFT PANEL ==============================================================
axL.set_aspect("equal", adjustable="box")
pad = 100.0
axL.set_xlim(-R_CYL1 - pad, R_CYL1 + pad)
axL.set_ylim(-R_CYL1 - pad, R_CYL1 + pad)

axL.add_patch(mp.Circle((0, 0), R_CYL1, fill=False, ec=COL_CYL1, lw=1.4))
axL.add_patch(mp.Circle((ELBOW_X, ELBOW_Z), R_CYL2, fill=False, ec=COL_CYL2, lw=1.4))
axL.add_patch(mp.Circle((AIRFRAME_X, AIRFRAME_Z), R_CYL3, fill=False,
                         ec=COL_CYL3, lw=1.6))
# cyl 4 is a small circle inside the airframe centered on the htail pivot.
# In world (after the two arm rotations) the body's +x axis points in the
# +x_world direction, so the body-frame pivot offset (X_HTAIL_PIVOT*c,
# Z_HTAIL_PIVOT*c) lands at (AIRFRAME_X + X_HTAIL_PIVOT*c,
# AIRFRAME_Z + Z_HTAIL_PIVOT*c).
axL.add_patch(mp.Circle((AIRFRAME_X + X_HTAIL_PIVOT * c,
                          AIRFRAME_Z + Z_HTAIL_PIVOT * c),
                         R_CYL4, fill=False, ec=COL_CYL4, lw=1.6))

axL.plot([0, ELBOW_X], [0, ELBOW_Z], "-", color=COL_CYL3, lw=2.5)
axL.plot([ELBOW_X, AIRFRAME_X], [ELBOW_Z, AIRFRAME_Z], "-", color=COL_CYL2, lw=2.5)
axL.plot(0, 0, "o", color="black", ms=7, zorder=5)
axL.plot(ELBOW_X, ELBOW_Z, "o", color="black", ms=5, zorder=5)
axL.plot(AIRFRAME_X, AIRFRAME_Z, "o", color="black", ms=4, zorder=5)

axL.set_axis_off()

# ===== RIGHT PANEL ==============================================================
axR.set_aspect("equal", adjustable="box")
axR.set_xlim(-3.0, +6.0)
axR.set_ylim(-2.2, +2.2)

# --- Items that EXIST in the simulation: solid ---
# Wing airfoil
axR.fill(bcm.wing_flat[:, 0] + bcm.WING_LE_X,
          bcm.wing_flat[:, 1] + bcm.WING_Z,
          facecolor=bcm.WING_FC, edgecolor=bcm.WING_EC, lw=1.2, zorder=3)
# Horizontal tail (v3 short boom, low position)
axR.fill(bcm.htail_flat[:, 0] + bcm.X_TAIL_LE_v3,
          bcm.htail_flat[:, 1] + bcm.Z_HTAIL_LOW,
          facecolor=bcm.HTAIL_FC, edgecolor=bcm.HTAIL_EC, lw=1.2, zorder=3)
# Actuator disk (axial rectangle in side view)
disk_x = bcm.PROP_X - bcm.DISK_THICK / 2
disk_z = bcm.PROP_Z - bcm.DISK_RAD
axR.add_patch(mp.Rectangle((disk_x, disk_z),
                            bcm.DISK_THICK, 2 * bcm.DISK_RAD,
                            facecolor=bcm.DISK_FC, edgecolor=bcm.DISK_EC,
                            lw=0.8, zorder=2))
# CG marker
axR.plot([0], [0], marker="+", markersize=12, color="#444444", zorder=5)

# --- Items that DO NOT exist in the simulation: dotted ---
BELLY_Z      = -0.40
BELLY_START  = -1.10
BELLY_END    = +0.80
R_NOSE       = 0.55
R_TAIL       = 1.10
xs, zs = bcm.fuselage_lower_contour(BELLY_START, BELLY_END, BELLY_Z,
                                     R_NOSE, R_TAIL)
axR.plot(xs, zs, linestyle=":", color="black", linewidth=1.6, zorder=1)
WHEEL_X, WHEEL_Z, WHEEL_R = +0.20, -0.80, 0.28
GROUND_Z = WHEEL_Z - WHEEL_R
# Wheel as dotted circle (approximated by many short dotted segments — use a
# Circle patch with a dotted linestyle).
axR.add_patch(mp.Circle((WHEEL_X, WHEEL_Z), WHEEL_R, fill=False,
                         edgecolor="black", lw=1.4, linestyle=":", zorder=4))
axR.plot([WHEEL_X, WHEEL_X], [BELLY_Z, WHEEL_Z],
          color="black", lw=1.2, linestyle=":", zorder=3)
axR.axhline(GROUND_Z, color="#a0a0a0", lw=0.8, linestyle=":", zorder=0)

# --- Four rotation cylinders ---
# cyl 3 (airframe-pitch): centred on CG, radius R_CYL3 in chord units
axR.add_patch(mp.Circle((0, 0), 2.90, fill=False, ec=COL_CYL3, lw=1.6, zorder=6))
# cyl 4 (htail-pitch): centred on htail c/4 pivot, radius R_CYL4
axR.add_patch(mp.Circle((X_HTAIL_PIVOT, Z_HTAIL_PIVOT), 0.85,
                         fill=False, ec=COL_CYL4, lw=1.6, zorder=6))

# cyl 2 (elbow) and cyl 1 (shoulder) cannot be drawn at airframe scale.
# With the production interface gaps (R_GAP_2_3_C, R_GAP_1_2_C = 110 c_w)
# cyl 2's nearest boundary point is R_CYL2 - L1 ≈ 113 c_w from the CG on
# the nose side, and cyl 1's nearest point is R_CYL1 - sqrt(2)·L ≈ 660 c_w
# away.  The caption carries these numbers; the left panel shows them.

axR.set_axis_off()

out = HERE / "v3_phugoid_setup.png"
fig.savefig(out, dpi=180, bbox_inches="tight", pad_inches=0.04)
print(f"wrote {out}")
