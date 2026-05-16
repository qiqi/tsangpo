"""
Build paper/figures/config_montage.png — a 2x3 schematic showing the
proposed configuration evolution.

Layout (no axes/labels):
  Top row    (planform / top view, flap stowed):
    (1) continuous flap, long boom
    (2) 40% inboard flap gap, long boom
    (3) 40% inboard flap gap, short boom (v3)
  Bottom row (side view, flap stowed):
    (1) low htail, long boom
    (2) high htail, long boom
    (3) low htail, short boom (v3)

Bottom row uses real airfoil cross-sections for both wing and
horizontal tail (htail = inverted LS(1)-0417 for negative camber).
Top and bottom rows share x-limits and use aspect='equal', so the
display scale (units per pixel) is the same across rows.

Fuselage in the side view is drawn as a single dotted line
representing only the LOWER contour: a flat belly between a 30 deg
nose-up arc forward and a 30 deg tail-flare arc aft.
"""
from __future__ import annotations
import importlib.util, sys
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.patches as mp
import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
AIRFOIL_DIR = REPO / "geometry" / "airfoils"
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(AIRFOIL_DIR))
import params as P

# Pull the LS(1)-0417 main-element coords via the geometry-build module.
spec = importlib.util.spec_from_file_location("besg", AIRFOIL_DIR / "build_estol_geometry.py")
besg = importlib.util.module_from_spec(spec); spec.loader.exec_module(besg)
cfg = yaml.safe_load((AIRFOIL_DIR / "estol_config.yaml").read_text())
blunt = cfg["trailing_edge"]["blunt_thickness"]
# LS(1)-0417 main element including the cove cutout (the function
# requires it; at this figure scale the cove geometry is barely visible).
_, wing_flat = besg.build_main_wing(cfg["airfoils"]["ls1_0417"],
                                     cfg["main_wing"]["cutouts"], blunt)
# Htail uses an INVERTED LS(1)-0417 (negative camber for down-force at
# zero incidence).  Same chord = c_w, so reuse wing_flat with y -> -y.
htail_flat = wing_flat.copy()
htail_flat[:, 1] *= -1.0

# ---- geometry pulled from params (SI -> chord units of c_w) -----------
c    = P.WING_MAC_M
b    = P.WING_SPAN_M
b2   = P.WING_SEMI_SPAN_M
S_ht = P.HTAIL_SPAN_M
c_ht = P.HTAIL_CHORD_M

WING_LE_X   = P.WING_X_LE_M / c                  # -0.5
WING_TE_X   = (P.WING_X_LE_M + c) / c            # +0.5
WING_LE_Y   = -b2 / c
WING_TE_Y   =  b2 / c
WING_Z      = P.WING_Z_M / c                     # +0.4

FLAP_X_LE   = +0.55
FLAP_X_TE   = +1.00
FLAP_X_LE_B = WING_LE_X + FLAP_X_LE
FLAP_X_TE_B = WING_LE_X + FLAP_X_TE
FLAP_GAP_HALF = 0.20 * b / c

X_TAIL_LE_v2 = P.X_TAIL_LE_M / c                 # +3.5
X_TAIL_TE_v2 = X_TAIL_LE_v2 + c_ht / c           # +4.5
X_TAIL_LE_v3 = +1.5
X_TAIL_TE_v3 = X_TAIL_LE_v3 + 1.0
HTAIL_HALF_Y = (S_ht / c) / 2
Z_HTAIL_LOW  = P.Z_TAIL_LOW_M / c                # +0.4
Z_HTAIL_HIGH = P.Z_TAIL_HIGH_M / c               # +1.9

PROP_X     = P.PROP_X_M / c                      # -0.7
PROP_Z     = P.PROP_Z_M / c                      # +0.1
DISK_THICK = 0.10
DISK_RAD   = P.PROP_RADIUS_M / c
PROP_YS    = list(P.PROP_Y_M) + [-y for y in P.PROP_Y_M]
PROP_YS_C  = [y / c for y in PROP_YS]

# Colors
WING_FC  = "#cfd8e6"; WING_EC  = "#1f3a68"
FLAP_FC  = "#cfe9c8"; FLAP_EC  = "#2c6b2c"
HTAIL_FC = "#fbd4b4"; HTAIL_EC = "#9c3d1a"
DISK_FC  = "#d8d8d8"; DISK_EC  = "#4a4a4a"
FUSE_FC  = "#eaeaea"; FUSE_EC  = "#666666"


def _rect(ax, x0, y0, w, h, fc, ec, lw=1.0, zorder=1, alpha=1.0):
    ax.add_patch(mp.Rectangle((x0, y0), w, h,
                              facecolor=fc, edgecolor=ec, lw=lw,
                              zorder=zorder, alpha=alpha))


# ---------------------------------------------------------------------------
# Fuselage lower-contour line (side view).  Per AIAA-paper draftsman spec:
#   * flat belly y = z_belly from x = x_belly_start to x = x_belly_end (length L)
#   * front (nose) arc of radius R_nose tangent to belly at (x_belly_start,
#     z_belly), curving 30 deg up-and-forward (into negative x)
#   * rear (tail-flare) arc of radius R_tail tangent to belly at (x_belly_end,
#     z_belly), curving 30 deg up-and-backward (into positive x)
# Returned as a single (xs, zs) polyline so it can be drawn as one
# continuous dotted black line of linewidth=2.
# Adjust L (= belly_end_x - belly_start_x), R_nose, R_tail to taste.
def fuselage_lower_contour(belly_start_x: float, belly_end_x: float,
                            z_belly: float, R_nose: float, R_tail: float,
                            sweep_deg: float = 30.0, n_arc: int = 64):
    sweep = np.radians(sweep_deg)

    # FRONT (nose) arc.
    # Center of curvature is directly above the start of the belly:
    #   center = (x_belly_start, z_belly + R_nose)
    # The tangent point on the belly is at polar angle -pi/2 from this
    # center (pointing straight down).  To sweep "up-and-into-negative-x"
    # we rotate CW (decreasing theta) by `sweep` radians.
    cx0, cz0 = belly_start_x, z_belly + R_nose
    theta = np.linspace(-np.pi / 2 - sweep, -np.pi / 2, n_arc)
    x_nose = cx0 + R_nose * np.cos(theta)
    z_nose = cz0 + R_nose * np.sin(theta)

    # BELLY segment: just two endpoints (the line is straight).
    x_belly = np.array([belly_start_x, belly_end_x])
    z_belly_pts = np.array([z_belly, z_belly])

    # REAR (tail-flare) arc.
    # Center directly above end of belly; tangent point at polar -pi/2;
    # sweep CCW by `sweep` radians to curve up-and-into-positive-x.
    cx1, cz1 = belly_end_x, z_belly + R_tail
    theta = np.linspace(-np.pi / 2, -np.pi / 2 + sweep, n_arc)
    x_tail = cx1 + R_tail * np.cos(theta)
    z_tail = cz1 + R_tail * np.sin(theta)

    # Concatenate (skip duplicate joining points)
    xs = np.concatenate([x_nose, x_belly[1:-1], x_tail])
    zs = np.concatenate([z_nose, z_belly_pts[1:-1], z_tail])
    return xs, zs


# ---------------------------------------------------------------------------
def draw_topview(ax, *, gap: bool, short_boom: bool):
    # Fuselage centerline strip (top view)
    body_x0 = WING_LE_X - 0.8
    body_x1 = (X_TAIL_TE_v3 if short_boom else X_TAIL_TE_v2) + 0.1
    _rect(ax, body_x0, -0.15, body_x1 - body_x0, 0.30, FUSE_FC, FUSE_EC, zorder=0)
    # Wing
    _rect(ax, WING_LE_X, WING_LE_Y, WING_TE_X - WING_LE_X, WING_TE_Y - WING_LE_Y,
          WING_FC, WING_EC, lw=1.2, zorder=1)
    # Flap (continuous or gapped)
    if gap:
        for ys, ye in [(-WING_TE_Y, -FLAP_GAP_HALF),
                       ( FLAP_GAP_HALF,  WING_TE_Y)]:
            _rect(ax, FLAP_X_LE_B, ys, FLAP_X_TE_B - FLAP_X_LE_B, ye - ys,
                  FLAP_FC, FLAP_EC, lw=1.0, zorder=2)
    else:
        _rect(ax, FLAP_X_LE_B, -WING_TE_Y, FLAP_X_TE_B - FLAP_X_LE_B, 2 * WING_TE_Y,
              FLAP_FC, FLAP_EC, lw=1.0, zorder=2)
    # Actuator disks: axial thickness x spanwise diameter
    for yc in PROP_YS_C:
        _rect(ax, PROP_X - DISK_THICK / 2, yc - DISK_RAD,
              DISK_THICK, 2 * DISK_RAD,
              DISK_FC, DISK_EC, lw=0.6, zorder=3)
    # Horizontal tail
    x_le = X_TAIL_LE_v3 if short_boom else X_TAIL_LE_v2
    x_te = X_TAIL_TE_v3 if short_boom else X_TAIL_TE_v2
    _rect(ax, x_le, -HTAIL_HALF_Y, x_te - x_le, 2 * HTAIL_HALF_Y,
          HTAIL_FC, HTAIL_EC, lw=1.2, zorder=2)


# Shared axis limits for "same scale" across both rows.
X_LIM_LO = WING_LE_X - 1.4
X_LIM_HI = X_TAIL_TE_v2 + 0.4
Y_LIM_TOP = (-(b2 / c + 0.3), (b2 / c + 0.3))   # ~ ±4.3 c
Z_LIM_SIDE = (-1.25, +2.4)                       # vertical extent for side view (room for ground+wheel)


def draw_sideview(ax, *, z_htail: float, short_boom: bool):
    # Fuselage LOWER contour (solid line).
    BELLY_Z      = -0.40
    BELLY_START  = -1.10
    BELLY_END    = +0.80 if short_boom else +1.60
    R_NOSE       = 0.55
    R_TAIL       = 1.10
    xs, zs = fuselage_lower_contour(BELLY_START, BELLY_END,
                                    BELLY_Z, R_NOSE, R_TAIL)
    ax.plot(xs, zs, linestyle="-", color="black", linewidth=2, zorder=1)
    # Wing airfoil section (clean LS(1)-0417).  Place LE at (WING_LE_X, WING_Z).
    ax.fill(wing_flat[:, 0] + WING_LE_X, wing_flat[:, 1] + WING_Z,
            facecolor=WING_FC, edgecolor=WING_EC, lw=1.2, zorder=3)
    # Horizontal-tail airfoil (inverted LS(1)-0417).
    x_le = X_TAIL_LE_v3 if short_boom else X_TAIL_LE_v2
    ax.fill(htail_flat[:, 0] + x_le, htail_flat[:, 1] + z_htail,
            facecolor=HTAIL_FC, edgecolor=HTAIL_EC, lw=1.2, zorder=3)
    # Actuator disk in side view: rectangle (axial thickness x vertical diameter)
    _rect(ax, PROP_X - DISK_THICK / 2, PROP_Z - DISK_RAD,
          DISK_THICK, 2 * DISK_RAD,
          DISK_FC, DISK_EC, lw=0.8, zorder=2)
    # Main landing gear: a single circle representing a (tundra-style
    # bush-plane) main wheel.  The wheel is positioned slightly AFT of
    # the CG and well BELOW the fuselage belly so the aircraft has
    # geometric clearance to rotate to a high (>= 30 deg) nose-up
    # landing attitude around the gear contact point before the
    # horizontal tail strikes the ground.  With the v3 short boom
    # (panel f), 30 deg rotation places the htail TE roughly
    # 0.25 c above the ground; with the v2 long boom (panels d, e)
    # the same rotation would drive the htail TE far below the ground
    # plane, i.e. tail-strike.
    WHEEL_X     = +0.50     # x position (aft of CG)
    WHEEL_Z     = -0.80     # wheel-center z (below belly at z = -0.40)
    WHEEL_R     = 0.20      # larger tundra-style tire
    GROUND_Z    = WHEEL_Z - WHEEL_R    # = -1.0  (ground contact point)
    ax.add_patch(mp.Circle((WHEEL_X, WHEEL_Z), WHEEL_R,
                            facecolor="#444444", edgecolor="black",
                            lw=1.2, zorder=4))
    # Thin strut from belly tangent point to the wheel
    ax.plot([WHEEL_X, WHEEL_X], [-0.40, WHEEL_Z],
            color="black", lw=1.2, zorder=3)
    # Ground line (very faint, full width of the panel) so the
    # rotation-clearance argument is visually obvious.
    ax.axhline(GROUND_Z, color="#a0a0a0", lw=0.8, linestyle="-", zorder=0)
    # CG marker
    ax.plot([0], [0], marker="+", markersize=12, color="#444444", zorder=5)


# ---------------------------------------------------------------------------
fig, axes = plt.subplots(2, 3, figsize=(13.5, 5.2),
                          gridspec_kw=dict(height_ratios=[3.0, 1.4]))
# Top row -- planforms (cruise / stowed flap)
draw_topview(axes[0, 0], gap=False, short_boom=False)
axes[0, 0].set_title("(a) continuous flap",          fontsize=11)
draw_topview(axes[0, 1], gap=True,  short_boom=False)
axes[0, 1].set_title("(b) $40\\%$ inboard flap gap", fontsize=11)
draw_topview(axes[0, 2], gap=True,  short_boom=True)
axes[0, 2].set_title("(c) v3: gap + short boom",     fontsize=11)
# Bottom row -- side views
draw_sideview(axes[1, 0], z_htail=Z_HTAIL_LOW,  short_boom=False)
axes[1, 0].set_title("(d) low htail",                fontsize=11)
draw_sideview(axes[1, 1], z_htail=Z_HTAIL_HIGH, short_boom=False)
axes[1, 1].set_title("(e) high T-tail",              fontsize=11)
draw_sideview(axes[1, 2], z_htail=Z_HTAIL_LOW,  short_boom=True)
axes[1, 2].set_title("(f) v3: low htail, short boom", fontsize=11)

# Same x-limits across both rows so the chord scale matches.
for ax in axes[0, :]:
    ax.set_xlim(X_LIM_LO, X_LIM_HI)
    ax.set_ylim(*Y_LIM_TOP)
    ax.set_aspect("equal", adjustable="box")
    ax.set_axis_off()
for ax in axes[1, :]:
    ax.set_xlim(X_LIM_LO, X_LIM_HI)
    ax.set_ylim(*Z_LIM_SIDE)
    ax.set_aspect("equal", adjustable="box")
    ax.set_axis_off()

fig.subplots_adjust(left=0.01, right=0.99, top=0.94, bottom=0.02,
                    wspace=0.02, hspace=0.05)
out = HERE / "config_montage.png"
fig.savefig(out, dpi=180, bbox_inches="tight", pad_inches=0.04)
print(f"wrote {out}")
