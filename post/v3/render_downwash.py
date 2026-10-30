"""Render the 3×2 downwash yz-slice figure for the landing configurations.

For each of (cont-low, cont-high, gap-high) × (α=5°, α=17°):
  - Download the slice tarball from the corresponding fork case.
  - Read the .vtu, compute ε = atan2(-w, u) on the slice.
  - Resample onto a regular grid for matplotlib pcolormesh.
  - Render the panel with htail outline overlay.

Common colorbar across all panels for direct comparison.

Output: paper/figures/downwash_landing_3x2.png
"""
from __future__ import annotations

import sys
import json
import math
from io import StringIO
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mp
import pyvista as pv
import flow360 as fl

REPO = Path(__file__).resolve().parents[2]

# ---- Configuration metadata ----------------------------------------------
# Each landing case's htail z-position and span.  All v2 configs use the
# same htail (span 4.43 m, semispan ±2.215 m).
HTAIL_SPAN_M     = 4.43
HTAIL_SEMISPAN_M = HTAIL_SPAN_M / 2
HTAIL_LE_X_M     = 4.847

CONFIGS = {
    "cont_low":  {"label": "cont-low (low htail, cont flap)",
                   "htail_z_m": +0.554,
                   "T_W":       0.608,
                   "row_color": "#1f3a68"},
    "cont_high": {"label": "cont-high (T-tail, cont flap)",
                   "htail_z_m": +2.630,
                   "T_W":       0.387,
                   "row_color": "#cc7a00"},
    "gap_high":  {"label": "gap-high (T-tail, 40% inboard flap gap)",
                   "htail_z_m": +2.630,
                   "T_W":       0.386,
                   "row_color": "#1f7a3f"},
}
ALPHAS = [5, 17]                                  # degrees

# yz plot window — show htail vicinity, ±5 m beyond htail tip in y
Y_WINDOW = (-4.0, +4.0)                            # m
Z_WINDOW = (-2.0, +4.0)                            # m, encompasses both low + T-tail

# Common color limits for downwash angle ε = atan(−w/u) across all panels.
# Freestream is plain +x in world frame (alpha=0 in AerospaceCondition;
# the angle of attack effect is applied by rotating the airframe-pitch
# cylinder, NOT by tilting the freestream).  So the raw atan(−w/u) IS
# the physically meaningful downwash — no α-subtraction.
EPS_RANGE_DEG = (-10, +40)

CASE_IDS_PATH = REPO / "post/v3/downwash_slice_forks.json"


def download_slice(case_id: str) -> Path:
    """Download slice tarball and extract; return path to the .vtu."""
    target_dir = Path(f"/tmp/{case_id}_slices")
    target_dir.mkdir(exist_ok=True)
    # Skip download if .vtu already extracted
    vtus = list(target_dir.glob("*.vtu"))
    if vtus:
        return vtus[0]
    c = fl.Case.from_cloud(case_id=case_id)
    if 'COMPLETED' not in str(c.status):
        print(f"  case {case_id} not COMPLETED yet ({c.status}); skipping")
        return None
    c.results.slices.download(str(target_dir.with_suffix(".gz")), overwrite=True)
    import tarfile
    with tarfile.open(str(target_dir.with_suffix(".gz"))) as tar:
        tar.extractall(target_dir)
    vtus = list(target_dir.glob("*.vtu"))
    return vtus[0] if vtus else None


def slice_to_grid(vtu_path: Path, alpha_deg: float):
    """Read VTU, extract velocity, compute downwash ε = atan2(-w, u)
    in WORLD frame (no α-subtraction — α is applied via airframe-pitch
    cylinder rotation, so the freestream is plain +x).

    Velocity is in solver non-dim (= Mach number); ratios are
    unit-independent so the angle is correct as-is.

    `alpha_deg` is kept as a parameter only for panel labelling, not for
    velocity rotation."""
    m = pv.read(str(vtu_path))
    pts = m.points        # (n_points, 3) — column 0 ≈ slice_x (constant), 1=y, 2=z
    vel = m.point_data['velocity']     # (n_points, 3)  in solver non-dim (= Mach)
    vmag = m.point_data['velocity_magnitude']
    y = pts[:, 1]; z = pts[:, 2]
    u = vel[:, 0]; w = vel[:, 2]

    eps_deg = np.degrees(np.arctan2(-w, u))

    # Mask "no-flow" regions (inside the body — |v| < small threshold).
    # Freestream Mach at landing ~0.04; threshold at 10% of that.
    body_mask = vmag < 0.005
    eps_deg[body_mask] = np.nan

    # Keep only points in the plot window.
    in_window = ((y >= Y_WINDOW[0]) & (y <= Y_WINDOW[1])
                  & (z >= Z_WINDOW[0]) & (z <= Z_WINDOW[1]))
    y, z = y[in_window], z[in_window]
    u, w, eps_deg = u[in_window], w[in_window], eps_deg[in_window]

    from scipy.interpolate import griddata
    yg = np.linspace(Y_WINDOW[0], Y_WINDOW[1], 220)
    zg = np.linspace(Z_WINDOW[0], Z_WINDOW[1], 160)
    YY, ZZ = np.meshgrid(yg, zg)
    eps_grid = griddata((y, z), eps_deg, (YY, ZZ), method='linear')
    u_grid   = griddata((y, z), u,       (YY, ZZ), method='linear')
    w_grid   = griddata((y, z), w,       (YY, ZZ), method='linear')
    return yg, zg, eps_grid, u_grid, w_grid


def draw_htail(ax, htail_z_m: float):
    """Mark the htail position with a dotted horizontal line spanning
    ±htail_semispan at z=htail_z.  The slice plane is upstream of the
    htail LE, so the htail isn't actually in the slice — this line
    just indicates where the htail SITS in y-z."""
    ax.plot([-HTAIL_SEMISPAN_M, +HTAIL_SEMISPAN_M], [htail_z_m, htail_z_m],
            linestyle=':', color='black', linewidth=2.0, zorder=10)


def main():
    ids = json.loads(CASE_IDS_PATH.read_text())
    print(f"Cases:")
    for k, v in ids.items(): print(f"  {k:18s} {v}")

    fig, axes = plt.subplots(len(CONFIGS), len(ALPHAS),
                             figsize=(11, 11), sharex=True, sharey=True,
                             constrained_layout=True)
    # Contour levels every 5° from -10 to +40
    levels = list(range(EPS_RANGE_DEG[0], EPS_RANGE_DEG[1] + 1, 5))

    for row, (cfg_key, cfg) in enumerate(CONFIGS.items()):
        for col, alpha in enumerate(ALPHAS):
            key = f"{cfg_key}_alpha{alpha}"
            case_id = ids.get(key)
            ax = axes[row, col]
            if not case_id:
                ax.set_title(f"{cfg['label']} α={alpha}°  [missing]", fontsize=9)
                continue
            vtu = download_slice(case_id)
            if not vtu:
                ax.set_title(f"{cfg['label']} α={alpha}°  [not yet]", fontsize=9)
                continue
            yg, zg, eps_grid, u_grid, w_grid = slice_to_grid(vtu, alpha)
            CS = ax.contour(yg, zg, eps_grid, levels=levels,
                             colors='black', linewidths=0.9)
            ax.clabel(CS, inline=True, fontsize=8, fmt='%d°')
            draw_htail(ax, cfg['htail_z_m'])
            ax.set_xlim(Y_WINDOW); ax.set_ylim(Z_WINDOW)
            ax.set_aspect('equal')
            ax.set_title(f"{cfg['label']}   α = {alpha}°,  T/W = {cfg['T_W']:.2f}",
                          fontsize=10)
            ax.grid(alpha=0.3, lw=0.4)
            if col == 0:
                ax.set_ylabel("z [m] (+ up)")
            if row == len(CONFIGS) - 1:
                ax.set_xlabel("y [m] (+ starboard)")

    fig.suptitle('Downwash angle  ε = atan(−w/u)  in the yz-plane at '
                 'x = +4.40 m (just upstream of htail LE) — landing trim;\n'
                 'dotted black line = htail span at its z-position',
                 fontsize=11)
    out = REPO / "paper/figures/downwash_landing_3x2.png"
    fig.savefig(out, dpi=140)
    plt.close(fig)
    print(f"\n  wrote {out}")


if __name__ == "__main__":
    main()
