"""Body-frame downwash figure: slice perpendicular to body x-axis,
velocity rotated to body coordinates, downwash angle ε_body =
atan(−w_body / u_body) drawn as labeled contour lines.

This shows what the htail SEES in body-relative flow at each (config, α):
the angular tilt of the flow below the body x-axis.  For a stable
airframe, dε_body/dα at the htail span should be NEGATIVE (upwash
strengthens with α) — that's the stability mechanism we're after.
"""
from __future__ import annotations

import sys
import json
import math
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import pyvista as pv
import flow360 as fl

REPO = Path(__file__).resolve().parents[2]

HTAIL_SEMISPAN_M = 4.43 / 2
HTAIL_LE_X_M     = 4.847

CONFIGS = {
    "cont_low":  {"label": "cont-low (low htail, cont flap)",
                   "htail_z_body": +0.554, "T_W": 0.608},
    "cont_high": {"label": "cont-high (T-tail, cont flap)",
                   "htail_z_body": +2.630, "T_W": 0.387},
    "gap_high":  {"label": "gap-high (T-tail, 40% inboard flap gap)",
                   "htail_z_body": +2.630, "T_W": 0.386},
}
ALPHAS = [5, 17]

# Plot window in BODY coordinates (y, z_body).  Same as world-frame
# version since rotation is around y so y_body = y_world.
Y_WINDOW = (-4.0, +4.0)
Z_WINDOW = (-2.0, +4.0)
EPS_RANGE_DEG = (-10, +40)


def download_slice(case_id: str) -> Path:
    target_dir = Path(f"/tmp/{case_id}_slices")
    target_dir.mkdir(exist_ok=True)
    vtus = list(target_dir.glob("*.vtu"))
    if vtus:
        return vtus[0]
    c = fl.Case.from_cloud(case_id=case_id)
    if 'COMPLETED' not in str(c.status):
        print(f"  case {case_id} not COMPLETED yet ({c.status})")
        return None
    c.results.slices.download(str(target_dir.with_suffix(".gz")), overwrite=True)
    import tarfile
    with tarfile.open(str(target_dir.with_suffix(".gz"))) as tar:
        tar.extractall(target_dir)
    vtus = list(target_dir.glob("*.vtu"))
    return vtus[0] if vtus else None


def slice_to_body_grid(vtu_path: Path, alpha_deg: float):
    """Read body-frame VTU; rotate velocity to body coords; compute
    ε_body; resample onto regular (y_body, z_body) grid."""
    a = math.radians(alpha_deg)
    ca, sa = math.cos(a), math.sin(a)
    m = pv.read(str(vtu_path))
    pts = m.points
    vel = m.point_data['velocity']
    vmag = m.point_data['velocity_magnitude']

    x_w, y_w, z_w = pts[:, 0], pts[:, 1], pts[:, 2]
    u_w, w_w     = vel[:, 0],  vel[:, 2]

    # Position in body frame:
    #   x_body = x_w cos α − z_w sin α   (= +4.40 for all points on slice)
    #   y_body = y_w
    #   z_body = x_w sin α + z_w cos α
    y_body = y_w
    z_body = x_w * sa + z_w * ca

    # Velocity in body frame:
    u_body = u_w * ca - w_w * sa
    w_body = u_w * sa + w_w * ca

    eps_body = np.degrees(np.arctan2(-w_body, u_body))
    # Mask body interior (no flow)
    body_mask = vmag < 0.005
    eps_body[body_mask] = np.nan

    in_window = ((y_body >= Y_WINDOW[0]) & (y_body <= Y_WINDOW[1])
                  & (z_body >= Z_WINDOW[0]) & (z_body <= Z_WINDOW[1]))
    y_body, z_body, eps_body = y_body[in_window], z_body[in_window], eps_body[in_window]

    from scipy.interpolate import griddata
    yg = np.linspace(Y_WINDOW[0], Y_WINDOW[1], 220)
    zg = np.linspace(Z_WINDOW[0], Z_WINDOW[1], 160)
    YY, ZZ = np.meshgrid(yg, zg)
    eps_grid = griddata((y_body, z_body), eps_body, (YY, ZZ), method='linear')
    return yg, zg, eps_grid


def draw_htail(ax, htail_z_body_m: float):
    ax.plot([-HTAIL_SEMISPAN_M, +HTAIL_SEMISPAN_M],
             [htail_z_body_m, htail_z_body_m],
             linestyle=':', color='black', linewidth=2.0, zorder=10)


def main():
    ids = json.loads((REPO / "post/v3/downwash_slice_forks_body.json").read_text())
    fig, axes = plt.subplots(len(CONFIGS), len(ALPHAS),
                              figsize=(11, 11), sharex=True, sharey=True,
                              constrained_layout=True)
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
            yg, zg, eps_grid = slice_to_body_grid(vtu, alpha)
            CS = ax.contour(yg, zg, eps_grid, levels=levels,
                             colors='black', linewidths=0.9)
            ax.clabel(CS, inline=True, fontsize=8, fmt='%d°')
            draw_htail(ax, cfg['htail_z_body'])
            ax.set_xlim(Y_WINDOW); ax.set_ylim(Z_WINDOW)
            ax.set_aspect('equal')
            ax.set_title(f"{cfg['label']}   α = {alpha}°,  T/W = {cfg['T_W']:.2f}",
                          fontsize=10)
            ax.grid(alpha=0.3, lw=0.4)
            if col == 0: ax.set_ylabel("z_body [m]  (+ up in airframe)")
            if row == len(CONFIGS) - 1:
                ax.set_xlabel("y [m] (+ starboard)")

    fig.suptitle('Body-frame downwash  ε_body = atan(−w_body / u_body)  on the\n'
                 'body-fixed yz-slice at body-x = +4.40 m (just upstream of htail LE) — landing trim;\n'
                 'dotted black line = htail span at its body-z position',
                 fontsize=11)
    out = REPO / "paper/figures/downwash_landing_3x2_body.png"
    fig.savefig(out, dpi=140)
    plt.close(fig)
    print(f"  wrote {out}")


if __name__ == "__main__":
    main()
