"""Render the y=0 LIC + Mach-rel animation for the LANDING phugoid
chain (chunks 1-4), with velocity AND Mach computed RELATIVE TO the
ambient air (= subtract freestream).

Two-layer composition per frame:
  • LIC texture from V_rel (line integral convolution of relative
    velocity) → captures the local flow structure (streamlines,
    vortices, slipstream direction).
  • Mach_rel scalar coloring (turbo cmap) layered on top via
    multiplicative blending → bright in high-flow regions, dark where
    flow is near ambient.

Why "relative to ambient air":
  In the ground-fixed frame, an observer drifting with the freestream
  sees the body+slipstream moving.  Subtracting V_inf along +X gives
  the body's "wake" relative to the air mass — the physically
  meaningful flow structure (separation, slipstream jet, vortices).

Pre-requisite:
  Python `lic` library installed (pip install --user lic).  ParaView
  is NOT used — its GPU-accelerated Surface LIC requires a working X
  server on this box (only OSMesa software fallback), so we use a
  CPU LIC implementation instead.

Outputs:
  post/v3/phugoid_runs/landing/lic_frames/frame_<step>.png
  post/v3/phugoid_runs/landing/nokick_landing_slices_lic.mp4
"""
from __future__ import annotations

import csv
import glob
import os
import re
import subprocess
import sys
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pyvista as pv
import lic as lic_lib

REPO   = Path("/home/qiqi/himalaya")
OUT    = REPO / "post/v3/phugoid_runs/landing"
FRAMES = OUT / "lic_frames"
FRAMES.mkdir(parents=True, exist_ok=True)
CG_CSV = OUT / "cg_lookup_landing.csv"

# ---------------------- constants ----------------------
V_INF_M_S = 12.86
A_INF_M_S = 340.29
M_INF     = V_INF_M_S / A_INF_M_S       # ≈ 0.0378

WIN       = 8.0          # half-window of chase camera [m]
MACH_REL_MAX = 0.10
RES       = 600          # samples per axis on the regular grid
LIC_LEN   = 25           # LIC integration length (in pixels)
DT        = 0.025
WARMUP_OFFSET = 30


# ---------------------- step → CG lookup ----------------------
cg = {}
with CG_CSV.open() as f:
    for r in csv.DictReader(f):
        cg[int(r["step"])] = (float(r["x_cg_m"]),
                              float(r["z_cg_m"]),
                              float(r["theta_body_deg"]))
print(f"CG lookup: {len(cg)} steps")


# ---------------------- gather .pvtu files ----------------------
all_files: list[tuple[int, str]] = []
for chunk_dir in sorted(d for d in os.listdir(OUT)
                         if d.startswith("slices_chunk")):
    for fp in glob.glob(str(OUT / chunk_dir / "slice_y=0_time_*.pvtu")):
        m = re.search(r"time_(\d+)", Path(fp).name)
        if m:
            all_files.append((int(m.group(1)), fp))
all_files.sort()
print(f"pvtu files: {len(all_files)}")


# ---------------------- one-frame renderer ----------------------
def render_frame(step: int, pvtu_path: str) -> bool:
    out_png = FRAMES / f"frame_{step:05d}.png"
    if out_png.exists():
        return True
    if step not in cg:
        return False
    xc, zc, th = cg[step]

    m = pv.read(pvtu_path)
    if m.n_points == 0:
        return False

    # Sample to a regular RES×RES image around the chase window
    grid = pv.ImageData()
    grid.dimensions = (RES, 1, RES)
    grid.origin     = (xc - WIN, 0, zc - WIN)
    grid.spacing    = (2*WIN/(RES-1), 1, 2*WIN/(RES-1))
    sampled = grid.sample(m)
    v = sampled.point_data["velocity"].reshape(RES, RES, 3)

    # Relative velocity (subtract freestream Mach in +X)
    v_rel = v.copy()
    v_rel[:, :, 0] -= M_INF
    mag_rel = np.linalg.norm(v_rel, axis=2)

    # Component arrays for LIC (vx in screen-x, vz in screen-y).
    # The regular-grid axes are (X, Y, Z) flattened as (i, j, k) with
    # j=0 (singleton y), so v_rel[i, 0, k, :] corresponds to grid
    # coords (x[i], 0, z[k]).  After dropping j=0, axes are (X, Z).
    vx = v_rel[:, :, 0].astype(np.float32)
    vz = v_rel[:, :, 2].astype(np.float32)
    mag2d = mag_rel.astype(np.float32)

    # Build LIC texture from the velocity field (lic uses its own noise)
    tex = lic_lib.lic(vx, vz, length=LIC_LEN)
    # tex is in [0, 1]

    # Compose: LIC as grayscale brightness, modulated by Mach_rel turbo color
    norm_mach = np.clip(mag2d / MACH_REL_MAX, 0.0, 1.0)
    cmap_mach = plt.get_cmap("turbo")
    rgb_mach = cmap_mach(norm_mach)[..., :3]      # H×W×3

    # Combine: rgb = rgb_mach * (0.5 + 0.5*tex)
    # — preserves the Mach hue, lets LIC texture modulate brightness
    rgb = rgb_mach * (0.45 + 0.55 * tex[..., None])
    rgb = np.clip(rgb, 0.0, 1.0)

    # Render with matplotlib for axis labels + colorbar
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.imshow(rgb, origin="lower",
              extent=(xc - WIN, xc + WIN, zc - WIN, zc + WIN),
              aspect="equal")
    sm = plt.cm.ScalarMappable(cmap="turbo",
                                norm=plt.Normalize(0, MACH_REL_MAX))
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=ax, fraction=0.046)
    cbar.set_label("|V_rel|/a_inf  (relative Mach)")
    t_glob = (step - WARMUP_OFFSET) * DT
    ax.set_xlabel("x [m]"); ax.set_ylabel("z [m]")
    ax.set_title(f"LIC of V_rel  +  Mach_rel   t={t_glob:.2f}s  "
                  f"θ_body={th:+.1f}°", fontsize=11)
    fig.savefig(out_png, dpi=110, bbox_inches="tight",
                facecolor="black")
    plt.close(fig)
    return True


# ---------------------- main loop ----------------------
t0 = time.time()
for i, (step, pvtu) in enumerate(all_files):
    ok = render_frame(step, pvtu)
    if i % 20 == 0:
        dt = time.time() - t0
        print(f"  [{i:3d}/{len(all_files)}] step {step}  "
              f"{'OK' if ok else 'SKIP'}  elapsed={dt:.0f}s")

# ---------------------- stitch MP4 ----------------------
out_mp4 = OUT / "nokick_landing_slices_lic.mp4"
r = subprocess.run([
    "ffmpeg", "-y", "-framerate", "40", "-pattern_type", "glob",
    "-i", str(FRAMES / "frame_*.png"),
    "-c:v", "libx264", "-pix_fmt", "yuv420p",
    "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2",
    str(out_mp4),
], capture_output=True, text=True)
if r.returncode == 0:
    print(f"wrote {out_mp4}")
else:
    print(f"ffmpeg failed: {r.stderr[-300:]}")
