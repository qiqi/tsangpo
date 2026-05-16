"""
Plot a y-slice from a Flow360 case's downloaded slice bundle.

Reads paper/figures/slices/<case-id>/slice_y=<value>.pvtu and produces
a PNG of the chosen field (velocity_magnitude or total_pressure) on the
slice, with overlaid wing + htail outlines for context.

Usage:
    python3 paper/figures/_plot_slice.py <case-id> [--y +0.000]
                                                  [--field velocity_magnitude]
                                                  [--xrange -2 6] [--zrange -1.5 3]
"""
from __future__ import annotations
import argparse, sys, re
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.collections as mcoll
import matplotlib.tri as mtri
import numpy as np
import pyvista as pv

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
SLICES_ROOT = HERE / "slices"
sys.path.insert(0, str(REPO))
import params as P


# Aircraft outline overlay in meters (use directly; slice is in meters).
c    = P.WING_MAC_M
WING_LE_X = P.WING_X_LE_M             # -0.692 m
WING_TE_X = P.WING_X_LE_M + c         # +0.692 m
WING_Z    = P.WING_Z_M                # +0.554 m
WING_THK_HALF = 0.07 * c              # cosmetic
PROP_X = P.PROP_X_M
PROP_Z = P.PROP_Z_M
PROP_R = P.PROP_RADIUS_M
PROP_THICK = 0.10 * c

X_HTAIL_LE = P.X_TAIL_LE_M            # +4.847 m
X_HTAIL_TE = P.X_TAIL_LE_M + P.HTAIL_CHORD_M
Z_HTAIL_LOW  = P.Z_TAIL_LOW_M         # +0.554 m
Z_HTAIL_HIGH = P.Z_TAIL_HIGH_M        # +2.630 m


def plot_slice(case_id: str, y_value: float = 0.0,
               field: str = "velocity_magnitude",
               xrange: tuple[float, float] = (-1.5, 7.5),
               zrange: tuple[float, float] = (-1.5, 3.5),
               out_path: Path | None = None,
               htail_z: float = Z_HTAIL_LOW,
               show_mesh: bool = True):
    case_dir = SLICES_ROOT / case_id
    # find the pvtu file matching the requested y
    targets = sorted(case_dir.glob("slice_y=*.pvtu"))
    if not targets:
        raise FileNotFoundError(f"no slice files in {case_dir}")
    best = min(targets, key=lambda p:
               abs(float(re.search(r"y=([+\-\d.]+?)(?=\.pvtu)", p.name).group(1))
                   - y_value))
    print(f"loading {best.name}")
    m = pv.read(best)
    pts = np.asarray(m.points)
    xs, zs = pts[:, 0], pts[:, 2]

    if field == "total_pressure":
        rho = np.asarray(m.point_data["rho"])
        v   = np.asarray(m.point_data["velocity"])
        p   = np.asarray(m.point_data["p"])
        vmag2 = np.sum(v * v, axis=1)
        vals = p + 0.5 * rho * vmag2
        cmap = "viridis"; label = r"total pressure  $p + \tfrac{1}{2}\rho|V|^2$ [Pa]"
    else:
        vals = np.asarray(m.point_data[field])
        cmap = "viridis"; label = field.replace("_", " ")

    # crop to a reasonable plotting box for tricontourf speed
    keep = (xs >= xrange[0] - 0.5) & (xs <= xrange[1] + 0.5) & \
           (zs >= zrange[0] - 0.5) & (zs <= zrange[1] + 0.5)
    xs, zs, vals = xs[keep], zs[keep], vals[keep]
    print(f"  {len(xs)} points after crop; field range "
          f"{vals.min():.2f} -- {vals.max():.2f}")

    fig, ax = plt.subplots(figsize=(11, 6))
    tri = mtri.Triangulation(xs, zs)
    # robust color range: 1st--99th percentile
    vmin, vmax = np.percentile(vals, [1, 99])
    cs = ax.tricontourf(tri, vals, levels=40, cmap=cmap, vmin=vmin, vmax=vmax,
                        extend="both")
    cbar = fig.colorbar(cs, ax=ax, shrink=0.85, pad=0.02)
    cbar.set_label(label, fontsize=10)

    if show_mesh:
        # Overlay the cell-edge mesh.  We extract the edges of every cell
        # face from the unstructured grid, then crop and draw as a thin
        # LineCollection.  Drawn over the contour so structure is visible.
        edges = m.extract_all_edges()
        edge_lines = []
        ep = np.asarray(edges.points)
        # `lines` in pyvista is a 1D array: [2, p0, p1, 2, p2, p3, ...]
        L = np.asarray(edges.lines).reshape(-1, 3)[:, 1:]
        for i, j in L:
            xa, _, za = ep[i]; xb, _, zb = ep[j]
            if (xrange[0] - 0.2 <= xa <= xrange[1] + 0.2 and
                xrange[0] - 0.2 <= xb <= xrange[1] + 0.2 and
                zrange[0] - 0.2 <= za <= zrange[1] + 0.2 and
                zrange[0] - 0.2 <= zb <= zrange[1] + 0.2):
                edge_lines.append([(xa, za), (xb, zb)])
        lc = mcoll.LineCollection(edge_lines, colors="black",
                                  linewidths=0.18, alpha=0.5, zorder=3)
        ax.add_collection(lc)
        print(f"  mesh edges drawn: {len(edge_lines)}")

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(*xrange); ax.set_ylim(*zrange)
    ax.set_xlabel("x [m]   (+x aft)"); ax.set_ylabel("z [m]   (+z up)")
    ax.grid(True, alpha=0.25)
    ax.set_title(f"{case_id}\ny = {y_value:+.3f} m   field: {field}",
                 fontsize=10, loc="left")
    fig.tight_layout()
    if out_path is None:
        out_path = case_dir / f"plot_y={y_value:+.3f}_{field}.png"
    fig.savefig(out_path, dpi=160, bbox_inches="tight")
    print(f"wrote {out_path}")
    plt.close(fig)
    return out_path


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("case_id")
    ap.add_argument("--y", type=float, default=0.0)
    ap.add_argument("--field", default="velocity_magnitude",
                    choices=("velocity_magnitude", "total_pressure",
                             "Mach", "Cp", "p", "rho"))
    ap.add_argument("--xrange", type=float, nargs=2, default=(-1.5, 7.5))
    ap.add_argument("--zrange", type=float, nargs=2, default=(-1.5, 3.5))
    ap.add_argument("--htail-z", type=float, default=Z_HTAIL_LOW,
                    help="z of htail mid-line for overlay (low=+0.554, high=+2.630)")
    ap.add_argument("--no-mesh", action="store_true",
                    help="suppress the mesh-edge overlay")
    ap.add_argument("--out-suffix", default="",
                    help="appended to default output filename, e.g. '_zoom'")
    a = ap.parse_args()
    out_path = None
    if a.out_suffix:
        case_dir = SLICES_ROOT / a.case_id
        out_path = case_dir / f"plot_y={a.y:+.3f}_{a.field}{a.out_suffix}.png"
    plot_slice(a.case_id, y_value=a.y, field=a.field,
               xrange=tuple(a.xrange), zrange=tuple(a.zrange),
               out_path=out_path,
               htail_z=a.htail_z, show_mesh=not a.no_mesh)
