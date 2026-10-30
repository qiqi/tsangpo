"""Render ONE body-frame downwash figure per config — 5 figures total
for the paper's configurational-evolution section.

Each figure: 1 row × 2 cols (α=5°, α=17°), labeled contour lines,
dotted htail-position line.
"""
from __future__ import annotations

import sys, json, math, tarfile
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import pyvista as pv
import flow360 as fl

REPO = Path(__file__).resolve().parents[2]

HTAIL_SEMISPAN_V2_M = 4.43 / 2          # v2 configs
HTAIL_SEMISPAN_V3_M = 4.43 / 2          # v3 same htail span

HTAIL_C4_X_V2_M = 5.193                 # v2 htail c/4 in body frame
HTAIL_C4_X_V3_M = 2.770                 # v3 htail c/4 in body frame

CONFIGS = [
    # key,         label,                                         z_body, T/W, semispan,           x_body (c/4)
    ("cont_low",   "(a) Cont-low (low htail, continuous flap)",   +0.554, 0.608, HTAIL_SEMISPAN_V2_M, HTAIL_C4_X_V2_M),
    ("cont_high",  "(b) Cont-high (T-tail, continuous flap)",      +2.630, 0.387, HTAIL_SEMISPAN_V2_M, HTAIL_C4_X_V2_M),
    ("gap_high",   "(c) Gap-high (T-tail, 40% inboard flap gap)",  +2.630, 0.386, HTAIL_SEMISPAN_V2_M, HTAIL_C4_X_V2_M),
    ("gap_low",    "(d) Gap-low (low htail, 40% inboard flap gap)",+0.554, None,  HTAIL_SEMISPAN_V2_M, HTAIL_C4_X_V2_M),
    ("v3",         "(e) v3 (low htail, gap, short boom)",          +0.554, None,  HTAIL_SEMISPAN_V3_M, HTAIL_C4_X_V3_M),
]
ALPHAS = [5, 17]

Y_WINDOW = (-4.0, +4.0)
Z_WINDOW = (-2.0, +4.0)
EPS_RANGE_DEG = (-30, +80)


def download_slice(case_id: str) -> Path | None:
    target_dir = Path(f"/tmp/{case_id}_slices")
    target_dir.mkdir(exist_ok=True)
    vtus = list(target_dir.glob("*.vtu"))
    if vtus: return vtus[0]
    c = fl.Case.from_cloud(case_id=case_id)
    if 'COMPLETED' not in str(c.status):
        return None
    c.results.slices.download(str(target_dir.with_suffix(".gz")), overwrite=True)
    with tarfile.open(str(target_dir.with_suffix(".gz"))) as tar:
        tar.extractall(target_dir)
    vtus = list(target_dir.glob("*.vtu"))
    return vtus[0] if vtus else None


def slice_to_grid(vtu_path: Path, alpha_deg: float):
    """The CFD slice is body-fixed (rotates with the airframe) — it sits at
    a constant body-x station.  Plot it on body-frame axes so the htail's
    position relative to the slice is identical across alpha.  The
    velocity field, however, comes from Flow360 in the world (inertial)
    frame because the freestream BC is fixed along world +x and only the
    geometry rotates (cfd_setup.py:258, with Flow360 alpha=0); the
    downwash angle is therefore ε = atan(-w_world / u_world) measured
    against the freestream direction, with no additional alpha rotation
    applied to (u, w).  The y- and z-axes of the plot are body-frame:
    the slice points have world coords (x_w, y_w, z_w) on the slice plane
    (body-x = const), so body-y = y_w and body-z = z_w cos α + x_w sin α
    (slice-script convention)."""
    a = math.radians(alpha_deg); ca, sa = math.cos(a), math.sin(a)
    m = pv.read(str(vtu_path))
    pts = m.points; vel = m.point_data['velocity']
    vmag = m.point_data['velocity_magnitude']
    x_w, y_w, z_w = pts[:, 0], pts[:, 1], pts[:, 2]
    u_w, w_w = vel[:, 0], vel[:, 2]
    y_body = y_w
    z_body = x_w * sa + z_w * ca
    eps = np.degrees(np.arctan2(-w_w, u_w))
    eps[vmag < 0.005] = np.nan
    keep = ((y_body >= Y_WINDOW[0]) & (y_body <= Y_WINDOW[1])
             & (z_body >= Z_WINDOW[0]) & (z_body <= Z_WINDOW[1]))
    y_body, z_body, eps = y_body[keep], z_body[keep], eps[keep]
    from scipy.interpolate import griddata
    yg = np.linspace(Y_WINDOW[0], Y_WINDOW[1], 220)
    zg = np.linspace(Z_WINDOW[0], Z_WINDOW[1], 160)
    YY, ZZ = np.meshgrid(yg, zg)
    eps_grid = griddata((y_body, z_body), eps, (YY, ZZ), method='linear')
    return yg, zg, eps_grid


def render_one(cfg_key: str, cfg_label: str, htail_z_body: float,
                T_W: float | None, semispan_m: float,
                ids_dict: dict[str, dict[str, str]],
                htail_x_body: float = 5.193) -> Path | None:
    """Render 1×2 figure for this config (α=5°, α=17°)."""
    case_a5  = ids_dict.get(cfg_key, {}).get('alpha5')
    case_a17 = ids_dict.get(cfg_key, {}).get('alpha17')
    if not (case_a5 and case_a17):
        print(f"  {cfg_key}: missing case IDs"); return None
    vtu5  = download_slice(case_a5)
    vtu17 = download_slice(case_a17)
    if not (vtu5 and vtu17):
        print(f"  {cfg_key}: slices not ready yet"); return None

    fig, axes = plt.subplots(1, 2, figsize=(11, 5.2),
                             sharex=True, sharey=True,
                             constrained_layout=True)
    levels = list(range(EPS_RANGE_DEG[0], EPS_RANGE_DEG[1] + 1, 5))
    for col, (alpha, vtu) in enumerate([(5, vtu5), (17, vtu17)]):
        ax = axes[col]
        yg, zg, eps_grid = slice_to_grid(vtu, alpha)
        CS = ax.contour(yg, zg, eps_grid, levels=levels,
                         colors='black', linewidths=0.9)
        ax.clabel(CS, inline=True, fontsize=8, fmt='%d°')
        # htail position line: body-fixed, constant across alpha.
        ax.plot([-semispan_m, +semispan_m],
                 [htail_z_body, htail_z_body],
                 linestyle=':', color='black', linewidth=2.0, zorder=10)
        ax.set_xlim(Y_WINDOW); ax.set_ylim(Z_WINDOW)
        ax.set_aspect('equal')
        tw_str = f", T/W = {T_W:.2f}" if T_W is not None else ""
        ax.set_title(f"α = {alpha}°{tw_str}", fontsize=10)
        ax.grid(alpha=0.3, lw=0.4)
        ax.set_xlabel("$y_\\mathrm{body}$ [m]  (+ starboard)")
        if col == 0:
            ax.set_ylabel("$z_\\mathrm{body}$ [m]  (+ up in airframe)")
    out = REPO / f"paper/figures/downwash_{cfg_key}.png"
    fig.savefig(out, dpi=140, bbox_inches='tight')
    plt.close(fig)
    print(f"  wrote {out}")
    return out


def main():
    # Merge ids from both fork batches
    ids: dict[str, dict[str, str]] = {}
    for fname in ["downwash_slice_forks_body.json",
                   "downwash_slice_forks_body_more.json"]:
        path = REPO / f"post/v3/{fname}"
        if not path.exists(): continue
        d = json.loads(path.read_text())
        for k, cid in d.items():
            # k looks like "cont_low_alpha5"
            cfg, alpha = k.rsplit("_alpha", 1)
            ids.setdefault(cfg, {})[f"alpha{alpha}"] = cid
    print(f"Configurations with submitted IDs: {sorted(ids.keys())}")

    for cfg_key, cfg_label, z_body, T_W, semispan, x_body in CONFIGS:
        if cfg_key not in ids:
            print(f"  {cfg_key}: no IDs submitted — skip")
            continue
        render_one(cfg_key, cfg_label, z_body, T_W, semispan, ids,
                   htail_x_body=x_body)


if __name__ == "__main__":
    main()
