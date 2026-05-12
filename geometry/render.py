"""
Render the ESP-built Tsangpo eSTOL geometry as a 2×3 figure: rows are top
(planform) and side (chord/height) views; columns are the three Fowler
phases (stowed/takeoff/landing).

Top view uses the 2D convex hull of each body's STL vertices — faithful
for extruded planforms. Side view uses a true planar section at y=0,
preserving the main wing's cove cutout.

    env -u PYTHONPATH python3 geometry/render.py
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import trimesh
from matplotlib.patches import Circle, Patch, Polygon
from scipy.spatial import ConvexHull

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
import params as P

OUT     = REPO / "geometry" / "out"
PHASES  = ("stowed", "takeoff", "landing")
PARTS   = ("main_wing", "vane", "aft_flap", "htail")
COLOR   = {"main_wing": "#A0A6AE", "vane": "#E07A2A",
           "aft_flap":  "#3FA34D", "htail": "#3B82C4"}
LABEL   = {"main_wing": "Main (LS(1)-0417, coved)",
           "vane":      "Vane (NACA 9621)",
           "aft_flap":  "Aft flap (NACA 6311)",
           "htail":     "H-tail (LS(1)-0417, inverted)"}
ORDER   = ("main_wing", "htail", "vane", "aft_flap")  # painters' bottom-up


def load_phase(phase_dir):
    return {n: trimesh.load_mesh(str(phase_dir / f"{n}.stl"), force="mesh")
            for n in PARTS}


def hull_xy(mesh):
    pts = np.unique(mesh.vertices[:, (1, 0)].round(6), axis=0)
    return pts[ConvexHull(pts).vertices]


def section_xz(mesh, y0=0.0):
    sec = mesh.section(plane_origin=[0, y0, 0], plane_normal=[0, 1, 0])
    return [path[:, (0, 2)] for path in sec.discrete]


def draw_top(ax, parts):
    for name in ORDER:
        ax.add_patch(Polygon(hull_xy(parts[name]), closed=True,
                             facecolor=COLOR[name], edgecolor="#1a1a1a",
                             linewidth=0.9, alpha=0.93))


def draw_side(ax, parts):
    for name in ORDER:
        for poly in section_xz(parts[name]):
            ax.add_patch(Polygon(poly, closed=True,
                                 facecolor=COLOR[name], edgecolor="#1a1a1a",
                                 linewidth=0.9, alpha=0.93))


def props_top(ax):
    for sign in (+1, -1):
        for eta in P.PROP_Y_NONDIM:
            ax.add_patch(Circle((sign * eta * P.WING_SEMI_SPAN_FT, P.PROP_X_FT),
                                P.PROP_RADIUS_FT,
                                facecolor="#D04141", edgecolor="#9a2424",
                                alpha=0.30, linewidth=0.8))


def props_side(ax):
    ax.add_patch(Circle((P.PROP_X_FT, P.PROP_Z_FT), P.PROP_RADIUS_FT,
                        facecolor="#D04141", edgecolor="#9a2424",
                        alpha=0.30, linewidth=0.8))


def main():
    parts_all = {p: load_phase(OUT / p) for p in PHASES}

    bb = np.array([m.bounds for parts in parts_all.values() for m in parts.values()])
    extras = np.array([[P.PROP_X_FT - P.PROP_RADIUS_FT, 0, P.PROP_Z_FT - P.PROP_RADIUS_FT],
                       [P.PROP_X_FT + P.PROP_RADIUS_FT, 0, P.PROP_Z_FT + P.PROP_RADIUS_FT]])
    lo = np.minimum(bb[:, 0, :].min(axis=0), extras[0])
    hi = np.maximum(bb[:, 1, :].max(axis=0), extras[1])
    pad = 0.8
    x_lim, y_lim, z_lim = ((lo[i] - pad, hi[i] + pad) for i in (0, 1, 2))

    fig, axes = plt.subplots(2, 3, figsize=(20, 12))
    for col, phase in enumerate(PHASES):
        parts = parts_all[phase]

        ax_t = axes[0, col]
        props_top(ax_t)
        draw_top(ax_t, parts)
        ax_t.set(xlim=y_lim, ylim=x_lim, aspect="equal",
                 title=phase.capitalize())
        ax_t.invert_yaxis()
        ax_t.grid(alpha=0.3)
        if col == 0:
            ax_t.set_ylabel("x (chord-wise, ft) — nose ↑")
            ax_t.set_xlabel("y (span, ft)")

        ax_s = axes[1, col]
        props_side(ax_s)
        draw_side(ax_s, parts)
        ax_s.set(xlim=x_lim, ylim=z_lim, aspect="equal",
                 xlabel="x (chord-wise, ft)")
        ax_s.grid(alpha=0.3)
        if col == 0:
            ax_s.set_ylabel("z (vertical, ft)")

    handles = [Patch(facecolor=COLOR[k], edgecolor="#1a1a1a", label=LABEL[k])
               for k in ORDER]
    handles.append(Patch(facecolor="#D04141", alpha=0.30, edgecolor="#9a2424",
                         label=f"Propeller disk ×{2 * len(P.PROP_Y_NONDIM)}"))
    fig.legend(handles=handles, loc="lower center", ncol=5,
               frameon=False, fontsize=10, bbox_to_anchor=(0.5, 0.01))

    fig.suptitle("Tsangpo eSTOL — ESP-built geometry, Fowler kinematics over three phases",
                 fontsize=14)
    fig.tight_layout(rect=(0, 0.04, 1, 0.96))
    out_path = OUT / "tsangpo_geometry.png"
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    print(f"Wrote {out_path}")


if __name__ == "__main__":
    main()
