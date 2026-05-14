"""
Render the high-htail v2 geometries (continuous and gapped) for visual
verification BEFORE Flow360 submission.

Reads STLs already built by serveCSM under:
    geometry/out_high_htail/{stowed,takeoff,landing}/*.stl
    geometry/out_gapped_high_htail/{stowed,takeoff,landing}/*.stl

Produces 2×3 top+side figures (matplotlib + trimesh, same style as the
existing geometry/render.py).  Writes to geometry/_renders/.

    env -u PYTHONPATH python3 geometry/render_high_htail.py
"""
from __future__ import annotations
import sys, glob
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

RENDERS = REPO / "geometry" / "_renders"
RENDERS.mkdir(exist_ok=True)
PHASES = ("stowed", "takeoff", "landing")

# Colors and z-ordering shared by both configs.
COLOR = {"main_wing": "#A0A6AE", "vane": "#E07A2A", "aft_flap": "#3FA34D", "htail": "#3B82C4"}
LABEL = {"main_wing": "Main wing", "vane": "Vane", "aft_flap": "Aft flap", "htail": "H-tail"}
ORDER = ("main_wing", "htail", "vane", "aft_flap")


def family(stl_name: str) -> str | None:
    """Map a stl filename (no extension) to its base part family for color/legend."""
    base = stl_name
    for fam in ("main_wing", "aft_flap", "vane", "htail"):
        if base.startswith(fam): return fam
    return None


def load_phase(phase_dir: Path) -> dict[str, list[trimesh.Trimesh]]:
    """Return dict[family, list[mesh]] — handles 1 or many pieces per family (gapped vs continuous)."""
    out: dict[str, list[trimesh.Trimesh]] = {}
    for stl in sorted(phase_dir.glob("*.stl")):
        fam = family(stl.stem)
        if fam is None: continue
        out.setdefault(fam, []).append(trimesh.load_mesh(str(stl), force="mesh"))
    return out


def hull_xy(mesh):
    pts = np.unique(mesh.vertices[:, (1, 0)].round(6), axis=0)
    return pts[ConvexHull(pts).vertices]


def section_xz(mesh, y0=0.0):
    sec = mesh.section(plane_origin=[0, y0, 0], plane_normal=[0, 1, 0])
    if sec is None: return []
    return [path[:, (0, 2)] for path in sec.discrete]


def draw_top(ax, parts):
    for fam in ORDER:
        for mesh in parts.get(fam, []):
            ax.add_patch(Polygon(hull_xy(mesh), closed=True,
                                 facecolor=COLOR[fam], edgecolor="#1a1a1a",
                                 linewidth=0.9, alpha=0.93))


def draw_side(ax, parts):
    # For side view sample at y=0 for centerline cuts AND at y just inside each
    # piece (helps the split gapped pieces show up since y=0 may miss them).
    sample_ys = (0.0, P.WING_SEMI_SPAN_M * 0.6, -P.WING_SEMI_SPAN_M * 0.6)
    for fam in ORDER:
        for mesh in parts.get(fam, []):
            drawn = False
            for y0 in sample_ys:
                for poly in section_xz(mesh, y0=y0):
                    ax.add_patch(Polygon(poly, closed=True,
                                         facecolor=COLOR[fam], edgecolor="#1a1a1a",
                                         linewidth=0.9, alpha=0.93))
                    drawn = True
                if drawn: break


def props_top(ax):
    for sign in (+1, -1):
        for eta in P.PROP_Y_NONDIM:
            ax.add_patch(Circle((sign * eta * P.WING_SEMI_SPAN_M, P.PROP_X_M),
                                P.PROP_RADIUS_M, facecolor="#D04141",
                                edgecolor="#9a2424", alpha=0.30, linewidth=0.8))


def props_side(ax):
    ax.add_patch(Circle((P.PROP_X_M, P.PROP_Z_M), P.PROP_RADIUS_M,
                        facecolor="#D04141", edgecolor="#9a2424",
                        alpha=0.30, linewidth=0.8))


def render_config(out_root: Path, label: str, out_png: Path):
    parts_all = {ph: load_phase(out_root / ph) for ph in PHASES}

    # Common axis limits across all phases for direct visual comparison.
    bboxes = []
    for parts in parts_all.values():
        for fam_meshes in parts.values():
            for m in fam_meshes: bboxes.append(m.bounds)
    bb = np.array(bboxes)
    extras = np.array([[P.PROP_X_M - P.PROP_RADIUS_M, 0, P.PROP_Z_M - P.PROP_RADIUS_M],
                       [P.PROP_X_M + P.PROP_RADIUS_M, 0, P.PROP_Z_M + P.PROP_RADIUS_M]])
    lo = np.minimum(bb[:, 0, :].min(axis=0), extras[0])
    hi = np.maximum(bb[:, 1, :].max(axis=0), extras[1])
    pad = 0.8
    x_lim, y_lim, z_lim = ((lo[i] - pad, hi[i] + pad) for i in (0, 1, 2))

    fig, axes = plt.subplots(2, 3, figsize=(20, 12))
    for col, phase in enumerate(PHASES):
        parts = parts_all[phase]
        ax_t = axes[0, col]
        props_top(ax_t); draw_top(ax_t, parts)
        ax_t.set(xlim=y_lim, ylim=x_lim, aspect="equal", title=phase.capitalize())
        ax_t.invert_yaxis(); ax_t.grid(alpha=0.3)
        if col == 0:
            ax_t.set_ylabel("x (chord, m) — nose ↑"); ax_t.set_xlabel("y (span, m)")

        ax_s = axes[1, col]
        props_side(ax_s); draw_side(ax_s, parts)
        ax_s.set(xlim=x_lim, ylim=z_lim, aspect="equal", xlabel="x (chord, m)")
        ax_s.grid(alpha=0.3)
        if col == 0: ax_s.set_ylabel("z (vertical, m)")

    handles = [Patch(facecolor=COLOR[k], edgecolor="#1a1a1a", label=LABEL[k]) for k in ORDER]
    handles.append(Patch(facecolor="#D04141", alpha=0.30, edgecolor="#9a2424",
                         label=f"Propeller disk ×{2*len(P.PROP_Y_NONDIM)}"))
    fig.legend(handles=handles, loc="lower center", ncol=5, frameon=False, fontsize=10,
               bbox_to_anchor=(0.5, 0.01))
    fig.suptitle(f"Tsangpo eSTOL — {label}  (htail 1.5 c above wing chord plane)",
                 fontsize=14)
    fig.tight_layout(rect=(0, 0.04, 1, 0.96))
    fig.savefig(out_png, dpi=140); plt.close(fig)
    print(f"Wrote {out_png}")


if __name__ == "__main__":
    render_config(REPO / "geometry" / "out_high_htail",
                  "continuous flap + HIGH htail",
                  RENDERS / "tsangpo_high_htail.png")
    render_config(REPO / "geometry" / "out_gapped_high_htail",
                  "gapped flap + HIGH htail",
                  RENDERS / "tsangpo_gapped_high_htail.png")
