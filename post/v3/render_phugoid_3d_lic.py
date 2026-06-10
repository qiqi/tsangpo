"""Render a 3D oblique-view animation of the landing phugoid chain
(chunks 1-4) with the aircraft surfaces overlaid on a y=0 slice
textured with LIC + Mach-rel.

Two layers per frame:
  • the aircraft body, transformed from its reference pose
    (chunk2 end-pose, step 429) to the current (CG, θ_body) using
    the same rigid-body rotation our integrator uses;
  • a flat plane at y=0 carrying the LIC + Mach texture, computed
    by sampling the slice PVTU onto a chase-window grid centered
    on the body.

Output:
  post/v3/phugoid_runs/landing/lic_frames_3d/frame_<step>.png
  post/v3/phugoid_runs/landing/nokick_landing_3d_lic.mp4
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
from PIL import Image
import pyvista as pv
import lic as lic_lib

REPO   = Path("/home/qiqi/himalaya")
OUT    = REPO / "post/v3/phugoid_runs/landing"
FRAMES = OUT / "lic_frames_3d"
FRAMES.mkdir(parents=True, exist_ok=True)
CG_CSV = OUT / "cg_lookup_landing.csv"

# ── physical constants ────────────────────────────────────────────────────────
V_INF_M_S = 12.86
A_INF_M_S = 340.29
M_INF     = V_INF_M_S / A_INF_M_S       # ≈ 0.0378
WIN       = 10.0       # half-window of chase camera [m]
MACH_REL_MAX = 0.10
RES       = 500        # samples per axis on regular grid
LIC_LEN   = 25
DT        = 0.025
WARMUP_OFFSET = 30

# Reference pose for body geometry (chunk2 final pose, step 429)
CG_REF = (1088.27, 0.0, -1061.28)
TH_REF_DEG = 89.32
SURF_DIR = Path("/tmp/surf_chunk2")
PART_FILES = {
    "wing":   "surface_cyl3_airframe_pitch_main_wing.pvtu",
    "flap_L": "surface_cyl3_airframe_pitch_aft_flap_left.pvtu",
    "flap_R": "surface_cyl3_airframe_pitch_aft_flap_right.pvtu",
    "vane_L": "surface_cyl3_airframe_pitch_vane_left.pvtu",
    "vane_R": "surface_cyl3_airframe_pitch_vane_right.pvtu",
    "htail":  "surface_cyl4_htail_pitch_htail.pvtu",
}
PART_COLORS = dict(
    wing="ivory", flap_L="salmon", flap_R="salmon",
    vane_L="lightblue", vane_R="lightblue", htail="gold",
)

# ── load + body-frame transform of every surface part ─────────────────────────
def load_body_parts() -> dict[str, pv.PolyData]:
    parts: dict[str, pv.PolyData] = {}
    th_ref = math.radians(TH_REF_DEG)
    cR, sR = math.cos(th_ref), math.sin(th_ref)
    for name, fname in PART_FILES.items():
        m = pv.read(str(SURF_DIR / fname))
        pts = np.array(m.points)
        pts[:, 0] -= CG_REF[0]
        pts[:, 2] -= CG_REF[2]
        x = pts[:, 0].copy(); z = pts[:, 2].copy()
        # R_y(-θ_ref):  x' = c·x + s·z,  z' = -s·x + c·z
        pts[:, 0] = cR * x + sR * z
        pts[:, 2] = -sR * x + cR * z
        m_body = m.copy()
        m_body.points = pts
        parts[name] = m_body
    return parts

def place_body(part: pv.PolyData, CGx: float, CGz: float, th_deg: float) -> pv.PolyData:
    """Body-frame mesh → world at (CGx, 0, CGz) with pitch th_deg."""
    th = math.radians(th_deg)
    c, s = math.cos(th), math.sin(th)
    P = np.array(part.points).copy()
    x = P[:, 0].copy(); z = P[:, 2].copy()
    # R_y(+θ):  x' = c·x - s·z,  z' = s·x + c·z
    P[:, 0] = c * x - s * z + CGx
    P[:, 2] = s * x + c * z + CGz
    out = part.copy(); out.points = P
    return out

# ── CG lookup ─────────────────────────────────────────────────────────────────
def load_cg_lookup() -> dict[int, tuple[float, float, float]]:
    cg: dict[int, tuple[float, float, float]] = {}
    with CG_CSV.open() as f:
        for r in csv.DictReader(f):
            cg[int(r["step"])] = (float(r["x_cg_m"]),
                                  float(r["z_cg_m"]),
                                  float(r["theta_body_deg"]))
    return cg

# ── per-frame slice texture (LIC + Mach_rel) ──────────────────────────────────
def build_slice_texture(pvtu_path: str, CGx: float, CGz: float,
                        out_png: Path) -> Path:
    """Sample the y=0 slice onto a 2WIN×2WIN window centered on (CGx, 0, CGz),
    build LIC + Mach_rel composite, save as a PNG suitable for pv.Texture."""
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
    Image.fromarray((rgb*255).astype(np.uint8)).save(out_png)
    return out_png

# ── one-frame 3D render ───────────────────────────────────────────────────────
def render_frame(step: int, pvtu_path: str,
                 body_parts: dict[str, pv.PolyData],
                 cg: dict[int, tuple[float, float, float]],
                 t_offset: float = 0.0) -> bool:
    out_png = FRAMES / f"frame_{step:05d}.png"
    if out_png.exists(): return True
    if step not in cg:    return False
    CGx, CGz, TH_DEG = cg[step]

    tex_png = FRAMES / f".tex_{step:05d}.png"
    build_slice_texture(pvtu_path, CGx, CGz, tex_png)

    plane = pv.Plane(center=(CGx, 0.0, CGz), direction=(0.0, 1.0, 0.0),
                      i_size=2*WIN, j_size=2*WIN, i_resolution=1, j_resolution=1)
    plane.active_texture_coordinates = np.array(
        [[0,0],[1,0],[0,1],[1,1]], dtype=np.float32)
    texture = pv.Texture(str(tex_png))

    p = pv.Plotter(off_screen=True, window_size=(1400, 900))
    p.add_mesh(plane, texture=texture)
    for name, part in body_parts.items():
        placed = place_body(part, CGx, CGz, TH_DEG)
        p.add_mesh(placed, color=PART_COLORS[name], smooth_shading=True)
    p.set_background("black")
    # Camera: behind-left-above for an aircraft-side oblique view
    p.camera_position = [
        (CGx - 6, -10, CGz + 5),    # camera position
        (CGx, 0, CGz),               # focal point at CG
        (0, 0, 1.0),                 # up = +Z (Z-up convention)
    ]
    img = p.screenshot()
    p.close()

    # Text overlay (larger font)
    pil = Image.fromarray(img)
    try:
        from PIL import ImageDraw, ImageFont
        # try a common ttf; fall back to default
        font = None
        for fp in ("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
                    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
            if os.path.exists(fp):
                font = ImageFont.truetype(fp, 28)
                break
        draw = ImageDraw.Draw(pil)
        t_glob = (step - WARMUP_OFFSET) * DT + t_offset
        msg1 = f"t = {t_glob:6.2f} s"
        msg2 = f"θ_body = {TH_DEG:+6.2f}°"
        if font is not None:
            for y, msg in zip((25, 60), (msg1, msg2)):
                draw.text((25, y), msg, fill="white", font=font,
                          stroke_width=2, stroke_fill="black")
        else:
            draw.text((20, 20), f"{msg1}   {msg2}", fill="white")
    except Exception:
        pass
    pil.save(out_png)
    # cleanup
    try: tex_png.unlink()
    except Exception: pass
    return True

# ── main loop ────────────────────────────────────────────────────────────────
def main(skip: int = 1, max_frames: int | None = None) -> None:
    body_parts = load_body_parts()
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
    for i, (step, pvtu) in enumerate(all_files):
        ok = render_frame(step, pvtu, body_parts, cg)
        if i % 10 == 0:
            elapsed = time.time() - t0
            eta = elapsed * (len(all_files) - i - 1) / max(i + 1, 1)
            print(f"  [{i:4d}/{len(all_files)}] step {step}  "
                  f"{'OK' if ok else 'SKIP'}  elapsed={elapsed:.0f}s  ETA={eta:.0f}s")

    out_mp4 = OUT / "nokick_landing_3d_lic.mp4"
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
    # Default: every 4th step (≈ 200 frames over 60 s at 0.1 s spacing)
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--skip", type=int, default=4)
    ap.add_argument("--max-frames", type=int, default=None)
    args = ap.parse_args()
    main(skip=args.skip, max_frames=args.max_frames)
