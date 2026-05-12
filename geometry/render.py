"""
Render the per-body STL files for every case in params.study1_matrix() as
a 2x2 grid (columns = gap_fraction, rows = Z_tail). Each cell shows the
orthographic top view (looking down, span horizontal) above the side view
(looking from +Y; airfoil contours from the ESP-generated STL).

    env -u PYTHONPATH python3 geometry/render.py

(PYTHONPATH must be unset because ESP's ESPenv.sh points it at Python 3.12
site-packages, which collides with the system Python 3.10 numpy.)

Each body is drawn as the 2D convex hull of its STL vertices projected onto
the view plane. The 30P30N slat, main, and flap contours are convex (or
near-convex) so this hull is visually faithful; ditto for the symmetric
H-tail. The hull is rendered as one filled polygon with a clean outline,
so airfoil silhouettes read crisply without the speckle of an STL
triangulation.
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
from matplotlib.patches import Circle, Polygon

from scipy.spatial import ConvexHull
import trimesh

import params as P

OUT = REPO / "geometry" / "out"

PART_COLOR = {
    "slat":  "#6B7178",      # deployed leading-edge slat
    "main":  "#A0A6AE",      # main wing element
    "htail": "#3B82C4",
    "flap":  "#E07A2A",      # deployed trailing-edge flap
}
PROP_IN, PROP_OUT = "#D04141", "#666666"
PART_FILES = ("slat_right",  "slat_left",
              "main_right",  "main_left",
              "htail_right", "htail_left",
              "flap_right",  "flap_left")
# Painters' order (drawn bottom-up). Slat and flap sit forward/aft of main
# and overlap it slightly in the side view; drawing them on top of main
# keeps the 30P30N slot geometry readable.
DRAW_ORDER = ("main", "slat", "flap", "htail")


def load_parts(case_dir: Path) -> dict[str, trimesh.Trimesh]:
    return {n: trimesh.load_mesh(str(case_dir / f"{n}.stl"), force="mesh")
            for n in PART_FILES if (case_dir / f"{n}.stl").exists()}


def prop_positions() -> list[tuple[float, str]]:
    htail_tip = P.HTAIL_SPAN_FT / 2
    out = []
    for y_sign in (+1, -1):
        for eta in P.PROP_Y_NONDIM:
            y0 = y_sign * eta * P.WING_SEMI_SPAN_FT
            out.append((y0, PROP_IN if abs(y0) < htail_tip else PROP_OUT))
    return out


def hull_polygon(mesh: trimesh.Trimesh, cols: tuple[int, int]) -> np.ndarray | None:
    pts = np.unique(mesh.vertices[:, cols].round(6), axis=0)
    if len(pts) < 3:
        return None
    try:
        hull = ConvexHull(pts)
    except Exception:
        return None
    return pts[hull.vertices]


def draw_view(ax, parts: dict[str, trimesh.Trimesh], plane: str) -> None:
    """
    plane = 'top'  -> page x = aircraft Y (span), page y = aircraft X (chord)
            'side' -> page x = aircraft X (chord), page y = aircraft Z (height)
    """
    cols = {"top": (1, 0), "side": (0, 2)}[plane]
    for prefix in DRAW_ORDER:
        for name, mesh in parts.items():
            if not name.startswith(prefix):
                continue
            poly = hull_polygon(mesh, cols)
            if poly is None:
                continue
            ax.add_patch(Polygon(poly, closed=True,
                                 facecolor=PART_COLOR[prefix],
                                 edgecolor="#1a1a1a", linewidth=0.9,
                                 alpha=0.92))

    if plane == "top":
        for y0, color in prop_positions():
            ax.add_patch(Circle((y0, P.PROP_X_FT), P.PROP_RADIUS_FT,
                                facecolor=color, edgecolor=color,
                                alpha=0.30, linewidth=0))
    else:  # 'side' -- all 10 props live on the same x,z; one circle says it
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
    x_lim   = (lo[0] - pad, hi[0] + pad)
    y_lim   = (lo[1] - pad, hi[1] + pad)
    z_lim   = (lo[2] - pad, hi[2] + pad)

    # Zoomed side view: focus on the slat+main+flap interaction region so
    # the 30P30N contour and the slot gaps read clearly. The H-tail is
    # only inside this z window in the low-tail cases; in the high-tail
    # cases we annotate its z position instead.
    x_lim_z = (-1.8, 6.4)
    z_lim_z = (-1.5, 1.0)

    x_span    = x_lim[1]   - x_lim[0]
    y_span    = y_lim[1]   - y_lim[0]
    z_span    = z_lim[1]   - z_lim[0]
    x_span_z  = x_lim_z[1] - x_lim_z[0]
    z_span_z  = z_lim_z[1] - z_lim_z[0]

    # All three panels share the same horizontal real estate W. With equal
    # aspect their heights become:
    #   top      = W * (x_span / y_span)
    #   side     = W * (z_span / x_span)
    #   side_zoom= W * (z_span_z / x_span_z)
    h_top       = x_span   / y_span
    h_side      = z_span   / x_span
    h_side_zoom = z_span_z / x_span_z

    layout = {
        (P.GAP_FRACTION_BASELINE, P.Z_TAIL_HIGH_CHORDS): (0, 0),
        (P.GAP_FRACTION_PROPOSED, P.Z_TAIL_HIGH_CHORDS): (0, 1),
        (P.GAP_FRACTION_BASELINE, P.Z_TAIL_LOW_CHORDS):  (1, 0),
        (P.GAP_FRACTION_PROPOSED, P.Z_TAIL_LOW_CHORDS):  (1, 1),
    }

    fig = plt.figure(figsize=(22, 26), facecolor="white")
    outer = fig.add_gridspec(2, 2, hspace=0.18, wspace=0.06,
                             left=0.05, right=0.99, top=0.94, bottom=0.05)

    for idx, (c, parts) in enumerate(zip(cases, parts_all), start=1):
        r, col = layout[(c.gap_fraction, c.z_tail_chords)]
        inner = outer[r, col].subgridspec(
            3, 1, hspace=0.07,
            height_ratios=[h_top, h_side, h_side_zoom])
        ax_top       = fig.add_subplot(inner[0])
        ax_side_full = fig.add_subplot(inner[1])
        ax_side_zoom = fig.add_subplot(inner[2])

        draw_view(ax_top,       parts, "top")
        draw_view(ax_side_full, parts, "side")
        draw_view(ax_side_zoom, parts, "side")

        ax_top.set_xlim(y_lim);   ax_top.set_ylim(x_lim)
        ax_top.invert_yaxis()                              # nose up
        ax_side_full.set_xlim(x_lim);   ax_side_full.set_ylim(z_lim)
        ax_side_zoom.set_xlim(x_lim_z); ax_side_zoom.set_ylim(z_lim_z)

        for ax, label in (
            (ax_top,       "top  (looking down, span horizontal)"),
            (ax_side_full, "side  (looking from +Y; full vertical extent)"),
            (ax_side_zoom, "side, zoomed on slat + main + flap  (30P30N contour)"),
        ):
            ax.set_aspect("equal", adjustable="box")
            ax.set_xticks([]); ax.set_yticks([])
            for s in ax.spines.values():
                s.set_color("#bbb")
            ax.text(0.99, 0.94, label, transform=ax.transAxes,
                    ha="right", fontsize=10, color="#666", style="italic")

        ax_top.set_title(f"C{idx}  {c.label}", loc="left",
                         fontsize=14, weight="bold", pad=6)
        ax_side_zoom.text(0.01, 0.06,
                          fr"$gap = {c.gap_fraction:.2f}$    "
                          fr"$Z_{{tail}} = {c.z_tail_chords:+.2f}\;c$",
                          transform=ax_side_zoom.transAxes,
                          fontsize=10, color="#444")

    fig.text(0.275, 0.955, "continuous flap   (gap = 0.00)",
             ha="center", fontsize=13, color="#222", weight="bold")
    fig.text(0.745, 0.955, "inboard gap      (gap = 0.35)",
             ha="center", fontsize=13, color="#222", weight="bold")
    fig.text(0.020, 0.72, "high T-tail\n$Z_{tail} = +2.5\\;c$",
             rotation=90, va="center", ha="left", fontsize=12,
             color="#222", weight="bold")
    fig.text(0.020, 0.30, "low H-tail\n$Z_{tail} = 0.0\\;c$",
             rotation=90, va="center", ha="left", fontsize=12,
             color="#222", weight="bold")

    legend_pairs = (
        ("Slat (30P30N)",   PART_COLOR["slat"]),
        ("Main element",    PART_COLOR["main"]),
        ("Outboard flap",   PART_COLOR["flap"]),
        ("H-tail",          PART_COLOR["htail"]),
    )
    handles = [
        plt.Line2D([0], [0], marker="s", color="w", markerfacecolor=clr,
                   markersize=15, markeredgecolor="#222", markeredgewidth=0.5,
                   label=lbl)
        for lbl, clr in legend_pairs
    ] + [
        plt.Line2D([0], [0], marker="o", color="w", markerfacecolor=PROP_IN,
                   markeredgecolor=PROP_IN, alpha=0.55, markersize=14,
                   label="Prop 1-2 (inboard, blows H-tail)"),
        plt.Line2D([0], [0], marker="o", color="w", markerfacecolor=PROP_OUT,
                   markeredgecolor=PROP_OUT, alpha=0.55, markersize=14,
                   label="Prop 3-5 (outboard, over flap)"),
    ]
    fig.legend(handles=handles, loc="lower center", ncol=6, frameon=False,
               fontsize=12, bbox_to_anchor=(0.5, 0.015))
    fig.suptitle("Tsangpo eSTOL - 2x2 SciTech matrix  (30P30N 3-element wing via ESP / OpenCSM)",
                 fontsize=17, y=0.985, weight="bold")

    out_png = OUT / "tsangpo_geometry.png"
    fig.savefig(out_png, dpi=150, facecolor="white")
    print(f"wrote {out_png}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
