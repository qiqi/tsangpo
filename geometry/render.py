"""
Render the per-body STL files for every case in params.study1_matrix() as
a 2x2 grid (columns = gap_fraction, rows = Z_tail). Each cell shows the
orthographic top view (x-y) above the side view (x-z); both panels share
the same x range so the chordwise feature alignment reads vertically.

    env -u PYTHONPATH python3 geometry/render.py

(PYTHONPATH must be unset because ESP's ESPenv.sh points it at Python 3.12
site-packages, which collides with the system Python 3.10 numpy.)
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.patches import Circle

import trimesh

import params as P

OUT = REPO / "geometry" / "out"

PART_COLOR = {"wing": "#A0A6AE", "htail": "#3B82C4", "flap": "#E07A2A"}
PROP_IN, PROP_OUT = "#D04141", "#666666"
PART_FILES = ("wing_right", "wing_left", "htail_right", "htail_left",
              "flap_right", "flap_left")
# Painters' order: lowest-priority body drawn first.
DRAW_ORDER = ("wing", "flap", "htail")


def load_parts(case_dir: Path) -> dict[str, trimesh.Trimesh]:
    return {n: trimesh.load_mesh(str(case_dir / f"{n}.stl"), force="mesh")
            for n in PART_FILES if (case_dir / f"{n}.stl").exists()}


def prop_positions() -> list[tuple[float, str]]:
    """[(y_ft, color)] for each of the 10 props."""
    htail_tip = P.HTAIL_SPAN_FT / 2
    out = []
    for y_sign in (+1, -1):
        for eta in P.PROP_Y_NONDIM:
            y0 = y_sign * eta * P.WING_SEMI_SPAN_FT
            out.append((y0, PROP_IN if abs(y0) < htail_tip else PROP_OUT))
    return out


def draw_view(ax, parts: dict[str, trimesh.Trimesh], plane: str) -> None:
    """Three-view convention:
        'top'  -> page-x = Y (span horizontal), page-y = X (chord; nose-up via
                  axis inversion later in main).
        'side' -> page-x = X (chord), page-y = Z (up)."""
    cols = {"top": (1, 0), "side": (0, 2)}[plane]
    for prefix in DRAW_ORDER:
        for name, mesh in parts.items():
            if not name.startswith(prefix):
                continue
            tris2d = mesh.triangles[:, :, cols]            # (N, 3, 2)
            ax.add_collection(PolyCollection(
                tris2d, facecolor=PART_COLOR[prefix],
                edgecolor="#333", linewidth=0.15, alpha=0.92))

    if plane == "top":
        for y0, color in prop_positions():
            ax.add_patch(Circle((y0, P.PROP_X_FT), P.PROP_RADIUS_FT,
                                facecolor=color, edgecolor=color,
                                alpha=0.30, linewidth=0))
    else:  # 'side' -- all 10 props live on the same x,z line; one circle says it.
        ax.add_patch(Circle((P.PROP_X_FT, P.PROP_Z_FT), P.PROP_RADIUS_FT,
                            facecolor=PROP_OUT, edgecolor=PROP_OUT,
                            alpha=0.25, linewidth=0))


def global_bounds(parts_all) -> tuple[np.ndarray, np.ndarray]:
    bb = np.array([m.bounds for parts in parts_all for m in parts.values()])
    lo, hi = bb[:, 0, :].min(axis=0), bb[:, 1, :].max(axis=0)
    lo[0] = min(lo[0], P.PROP_X_FT - P.PROP_RADIUS_FT)
    hi[0] = max(hi[0], P.PROP_X_FT + P.PROP_RADIUS_FT)
    lo[2] = min(lo[2], P.PROP_Z_FT - P.PROP_RADIUS_FT)
    hi[2] = max(hi[2], P.PROP_Z_FT + P.PROP_RADIUS_FT)
    return lo, hi


def main() -> int:
    cases = list(P.study1_matrix())
    parts_all = [load_parts(OUT / c.name) for c in cases]
    lo, hi = global_bounds(parts_all)

    pad = 0.6
    x_lim = (lo[0] - pad, hi[0] + pad)
    y_lim = (lo[1] - pad, hi[1] + pad)
    z_lim = (lo[2] - pad, hi[2] + pad)

    # With equal aspect and a fixed cell width:
    #   top view  spans Y horizontally, X vertically → height/width = x/y
    #   side view spans X horizontally, Z vertically → height/width = z/x
    x_span = x_lim[1] - x_lim[0]
    y_span = y_lim[1] - y_lim[0]
    z_span = z_lim[1] - z_lim[0]
    h_top  = x_span / y_span
    h_side = z_span / x_span

    layout = {
        (P.GAP_FRACTION_BASELINE, P.Z_TAIL_HIGH_CHORDS): (0, 0),
        (P.GAP_FRACTION_PROPOSED, P.Z_TAIL_HIGH_CHORDS): (0, 1),
        (P.GAP_FRACTION_BASELINE, P.Z_TAIL_LOW_CHORDS):  (1, 0),
        (P.GAP_FRACTION_PROPOSED, P.Z_TAIL_LOW_CHORDS):  (1, 1),
    }

    fig = plt.figure(figsize=(16, 13), facecolor="white")
    outer = fig.add_gridspec(2, 2, hspace=0.20, wspace=0.06,
                             left=0.05, right=0.99, top=0.92, bottom=0.07)

    for idx, (c, parts) in enumerate(zip(cases, parts_all), start=1):
        r, col = layout[(c.gap_fraction, c.z_tail_chords)]
        inner = outer[r, col].subgridspec(2, 1, hspace=0.05,
                                          height_ratios=[h_top, h_side])
        ax_top  = fig.add_subplot(inner[0])
        ax_side = fig.add_subplot(inner[1])

        draw_view(ax_top,  parts, "top")
        draw_view(ax_side, parts, "side")

        ax_top.set_xlim(y_lim);  ax_top.set_ylim(x_lim)
        ax_top.invert_yaxis()                              # +X (aft) points down → nose up
        ax_side.set_xlim(x_lim); ax_side.set_ylim(z_lim)

        for ax, label in ((ax_top, "top  (looking down, span horizontal)"),
                          (ax_side, "side  (looking from +Y)")):
            ax.set_aspect("equal", adjustable="box")
            ax.set_xticks([]); ax.set_yticks([])
            for s in ax.spines.values():
                s.set_color("#bbb")
            ax.text(0.99, 0.93, label, transform=ax.transAxes,
                    ha="right", fontsize=9, color="#666", style="italic")

        ax_top.set_title(f"C{idx}  {c.label}", loc="left",
                         fontsize=12, weight="bold", pad=6)
        ax_side.text(0.01, 0.05,
                     fr"$gap = {c.gap_fraction:.2f}$   "
                     fr"$Z_{{tail}} = {c.z_tail_chords:+.2f}\;c$",
                     transform=ax_side.transAxes, fontsize=9, color="#444")

    fig.text(0.275, 0.945, "continuous flap   (gap = 0.00)",
             ha="center", fontsize=12, color="#222", weight="bold")
    fig.text(0.745, 0.945, "inboard gap      (gap = 0.35)",
             ha="center", fontsize=12, color="#222", weight="bold")
    fig.text(0.020, 0.72, "high T-tail\n$Z_{tail} = +2.5\\;c$",
             rotation=90, va="center", ha="left", fontsize=11,
             color="#222", weight="bold")
    fig.text(0.020, 0.30, "low H-tail\n$Z_{tail} = 0.0\\;c$",
             rotation=90, va="center", ha="left", fontsize=11,
             color="#222", weight="bold")

    handles = [
        plt.Line2D([0], [0], marker="s", color="w", markerfacecolor=clr,
                   markersize=15, markeredgecolor="#222", markeredgewidth=0.5,
                   label=lbl)
        for lbl, clr in zip(("Wing", "H-tail", "Outboard flap"),
                            PART_COLOR.values())
    ] + [
        plt.Line2D([0], [0], marker="o", color="w", markerfacecolor=PROP_IN,
                   markeredgecolor=PROP_IN, alpha=0.55, markersize=14,
                   label="Prop 1-2 (inboard, blows H-tail)"),
        plt.Line2D([0], [0], marker="o", color="w", markerfacecolor=PROP_OUT,
                   markeredgecolor=PROP_OUT, alpha=0.55, markersize=14,
                   label="Prop 3-5 (outboard, over flap)"),
    ]
    fig.legend(handles=handles, loc="lower center", ncol=5, frameon=False,
               fontsize=11, bbox_to_anchor=(0.5, 0.015))
    fig.suptitle("Himalayan eSTOL - 2x2 SciTech matrix  (ESP / OpenCSM parametric model)",
                 fontsize=15, y=0.985, weight="bold")

    out_png = OUT / "himalaya_geometry.png"
    fig.savefig(out_png, dpi=150, facecolor="white")
    print(f"wrote {out_png}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
