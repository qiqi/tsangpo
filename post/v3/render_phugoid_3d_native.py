"""Per-step body + slice 3D renderer for the landing phugoid chain.

No coordinate transforms anywhere — body surfaces and the y=0 slice
are both loaded in their native Flow360 world coordinates at the same
physical step.  Made possible by re-running the chain with
surface_frequency=4 (every 4 physical steps, matching the slice cadence
we use for the animation).

For each frame:
  step ── pick nearest surface-step in the same chunk (every 4 steps:
          32, 36, …, 228 in chunk1, 232, 236, … in chunk2, etc.)
  ── load 6 body part .pvtu files at that step
  ── load the y=0 slice .pvtu at the SAME step
  ── sample slice onto a chase-window grid centered on body's CG,
      build LIC + Mach_rel, save as a PNG
  ── place the textured plane and the 6 body parts in a 3D scene at
      their NATIVE world coordinates
  ── render with an oblique camera following the body's CG

The body's CG per frame is read from cg_lookup_landing.csv (which gives
us a continuous trajectory at every step, even when surfaces are only
written every 4 steps).
"""
from __future__ import annotations

import csv
import glob
import math
import os
import re
import subprocess
import sys
import time
from pathlib import Path

os.environ.setdefault("PYVISTA_OFF_SCREEN", "true")

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import pyvista as pv
import lic as lic_lib

REPO   = Path("/home/qiqi/himalaya")
OUT    = REPO / "post/v3/phugoid_runs/landing"
FRAMES = OUT / "lic_frames_3d_native"
FRAMES.mkdir(parents=True, exist_ok=True)
CG_CSV = OUT / "cg_lookup_landing.csv"

# ── physical constants / appearance ──────────────────────────────────────────
V_INF = 12.86
A_INF = 340.29
M_INF = V_INF / A_INF
WIN   = 10.0
RES   = 500
LIC_LEN = 25
MACH_REL_MAX = 0.10
DT    = 0.025
WARMUP_OFFSET = 30

# Per-step surface directory layout.  Each chunk has its own directory of
# per-time .pvtu files extracted from surfaces.tar.gz.
SURF_DIRS = {
    "chunk1": Path("/tmp/surf_chunk1_perstep"),
    "chunk2": Path("/tmp/surf_chunk2_perstep"),
    "chunk3": Path("/tmp/surf_chunk3_perstep"),
    "chunk4": Path("/tmp/surf_chunk4_perstep"),
}

# Map step → chunk (chunk1 = 30-229, chunk2 = 230-429, etc.).
def step_to_chunk(step: int) -> str:
    if step <= 229: return "chunk1"
    if step <= 429: return "chunk2"
    if step <= 629: return "chunk3"
    return "chunk4"

PART_NAMES = ["main_wing", "aft_flap_left", "aft_flap_right",
              "vane_left", "vane_right", "htail"]
PART_PREFIX = {
    "main_wing": "surface_cyl3_airframe_pitch_main_wing",
    "aft_flap_left":  "surface_cyl3_airframe_pitch_aft_flap_left",
    "aft_flap_right": "surface_cyl3_airframe_pitch_aft_flap_right",
    "vane_left":  "surface_cyl3_airframe_pitch_vane_left",
    "vane_right": "surface_cyl3_airframe_pitch_vane_right",
    "htail":      "surface_cyl4_htail_pitch_htail",
}
PART_COLORS = dict(
    main_wing="ivory", aft_flap_left="salmon", aft_flap_right="salmon",
    vane_left="lightblue", vane_right="lightblue", htail="gold",
)

# ── CG lookup (per-step trajectory of CG and θ) ──────────────────────────────
def load_cg_lookup() -> dict[int, tuple[float, float, float]]:
    cg: dict[int, tuple[float, float, float]] = {}
    with CG_CSV.open() as f:
        for r in csv.DictReader(f):
            cg[int(r["step"])] = (float(r["x_cg_m"]),
                                  float(r["z_cg_m"]),
                                  float(r["theta_body_deg"]))
    return cg

# ── find a surface step at or near the given step ────────────────────────────
def nearest_surface_step(step: int, surf_dir: Path) -> int | None:
    """Find the nearest surface output step in surf_dir.  Returns None
    if no surface files match."""
    pattern = re.compile(r"main_wing_time_(\d+)\.pvtu$")
    available = []
    for fn in os.listdir(surf_dir):
        m = pattern.search(fn)
        if m: available.append(int(m.group(1)))
    if not available: return None
    available.sort()
    # Find nearest in absolute distance
    return min(available, key=lambda s: abs(s - step))

# ── slice texture (same recipe as before; flipud only — empirically the
#     fliplr from earlier was wrong for this convention) ─────────────────────
def build_slice_texture(pvtu_path: str, CGx: float, CGz: float,
                        out_png: Path) -> None:
    sl = pv.read(pvtu_path)
    grid = pv.ImageData()
    grid.dimensions = (RES, 1, RES)
    grid.origin     = (CGx - WIN, 0, CGz - WIN)
    grid.spacing    = (2*WIN/(RES-1), 1, 2*WIN/(RES-1))
    sampled = grid.sample(sl)
    v = sampled.point_data["velocity"].reshape(RES, RES, 3)
    v_rel = v.copy(); v_rel[:, :, 0] -= M_INF
    vx = v_rel[:, :, 0].astype(np.float32)
    vz = v_rel[:, :, 2].astype(np.float32)
    mag = np.linalg.norm(v_rel, axis=2)
    tex = lic_lib.lic(vz, vx, length=LIC_LEN)
    rgb = (plt.get_cmap("turbo")(np.clip(mag/MACH_REL_MAX, 0, 1))[..., :3]
           * (0.45 + 0.55 * tex[..., None]))
    rgb = np.clip(rgb, 0, 1)
    rgb = np.flipud(rgb)   # PIL row 0 = top of PNG; OpenGL v=0 = bottom
    Image.fromarray((rgb*255).astype(np.uint8)).save(out_png)

# ── one-frame render ─────────────────────────────────────────────────────────
def render_frame(step: int, slice_pvtu: str,
                 cg: dict[int, tuple[float, float, float]]) -> bool:
    out_png = FRAMES / f"frame_{step:05d}.png"
    if out_png.exists(): return True
    if step not in cg:    return False
    CGx, CGz, TH_DEG = cg[step]

    # Pick surfaces from the same chunk, nearest in step
    chunk = step_to_chunk(step)
    surf_dir = SURF_DIRS[chunk]
    if not surf_dir.exists(): return False
    surf_step = nearest_surface_step(step, surf_dir)
    if surf_step is None: return False

    # Build slice texture
    tex_png = FRAMES / f".tex_{step:05d}.png"
    build_slice_texture(slice_pvtu, CGx, CGz, tex_png)

    # Build textured plane
    W = WIN
    plane_pts = np.array([
        [CGx - W, 0, CGz - W], [CGx + W, 0, CGz - W],
        [CGx + W, 0, CGz + W], [CGx - W, 0, CGz + W],
    ])
    plane = pv.PolyData(plane_pts, faces=np.array([4, 0, 1, 2, 3]))
    plane.active_texture_coordinates = np.array(
        [[0, 0], [1, 0], [1, 1], [0, 1]], dtype=np.float32)
    texture = pv.Texture(str(tex_png))

    p = pv.Plotter(off_screen=True, window_size=(1400, 900))
    p.add_mesh(plane, texture=texture)
    # Load body parts at this step's surface_step IN NATIVE WORLD COORDINATES
    for part_name, prefix in PART_PREFIX.items():
        path = surf_dir / f"{prefix}_time_{surf_step}.pvtu"
        if not path.exists(): continue
        m = pv.read(str(path))
        p.add_mesh(m, color=PART_COLORS[part_name], smooth_shading=True)
    p.set_background("black")
    p.camera_position = [
        (CGx - 6, -10, CGz + 5),     # camera behind-left-above CG
        (CGx, 0, CGz),
        (0, 0, +1.0),
    ]
    img = p.screenshot()
    p.close()

    # Overlay t and θ_body
    pil = Image.fromarray(img)
    try:
        font = None
        for fp in ("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
                    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
            if os.path.exists(fp):
                font = ImageFont.truetype(fp, 28); break
        draw = ImageDraw.Draw(pil)
        t_glob = (step - WARMUP_OFFSET) * DT
        for y, msg in zip((25, 60),
                          (f"t = {t_glob:6.2f} s",
                           f"θ_body = {TH_DEG:+6.2f}°")):
            if font is not None:
                draw.text((25, y), msg, fill="white", font=font,
                          stroke_width=2, stroke_fill="black")
            else:
                draw.text((20, y), msg, fill="white")
    except Exception:
        pass
    pil.save(out_png)
    try: tex_png.unlink()
    except Exception: pass
    return True


def main(skip: int = 4, max_frames: int | None = None) -> None:
    cg = load_cg_lookup()
    all_files: list[tuple[int, str]] = []
    for chunk_dir in sorted(d for d in os.listdir(OUT)
                             if d.startswith("slices_chunk")):
        for fp in glob.glob(str(OUT / chunk_dir / "slice_y=0_time_*.pvtu")):
            m = re.search(r"time_(\d+)", Path(fp).name)
            if m: all_files.append((int(m.group(1)), fp))
    all_files.sort()
    if skip > 1: all_files = all_files[::skip]
    if max_frames: all_files = all_files[:max_frames]
    print(f"frames to render: {len(all_files)}")

    t0 = time.time()
    for i, (step, slice_pvtu) in enumerate(all_files):
        ok = render_frame(step, slice_pvtu, cg)
        if i % 10 == 0:
            elapsed = time.time() - t0
            eta = elapsed * (len(all_files) - i - 1) / max(i + 1, 1)
            print(f"  [{i:4d}/{len(all_files)}] step {step}  "
                  f"{'OK' if ok else 'SKIP'}  elapsed={elapsed:.0f}s  ETA={eta:.0f}s")

    out_mp4 = OUT / "nokick_landing_3d_native.mp4"
    r = subprocess.run([
        "ffmpeg", "-y", "-framerate", "20", "-pattern_type", "glob",
        "-i", str(FRAMES / "frame_*.png"),
        "-c:v", "libx264", "-pix_fmt", "yuv420p",
        "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2",
        str(out_mp4),
    ], capture_output=True, text=True)
    if r.returncode == 0:
        print(f"wrote {out_mp4}")
    else:
        print(f"ffmpeg failed: {r.stderr[-300:]}")


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--skip", type=int, default=4)
    ap.add_argument("--max-frames", type=int, default=None)
    args = ap.parse_args()
    main(skip=args.skip, max_frames=args.max_frames)
