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
CG_CSV = OUT / "cg_lookup_landing_new.csv"   # extracted from the NEW (surf-per-step) chain

# Per-render output paths (populated in main() from --lag-s).
FRAMES: Path = OUT / "lic_frames_3d_native"
FRAMES.mkdir(parents=True, exist_ok=True)
OUT_MP4: Path = OUT / "nokick_landing_3d_native.mp4"
# Camera-anchor lag (s).  0.0 ⇒ anchor at current CG (original behaviour).
LAG_S: float = 0.0

# Fixed camera offset from the (possibly-lagged) anchor in mesh-frame
# coordinates: 6 m downstream-side (-x), 10 m to the -y side, 5 m above.
CAM_OFFSET = (-6.0, -10.0, +5.0)

# ── physical constants / appearance ──────────────────────────────────────────
V_INF = 12.86
A_INF = 340.29
M_INF = V_INF / A_INF
# Dual coplanar slice planes around the current CG:
#   inner (high-res, captures near-body detail)   — 2·WIN_INNER × 2·WIN_INNER m
#   outer (low-res, fills the background)         — 2·WIN_OUTER × 2·WIN_OUTER m
# RES* is the LIC source-grid resolution; per-meter density is RES/(2·WIN).
# At WIN=10/RES=1024 the inner gets 51 source-px/m ⇒ display ratio ~2.5× on a
# 1440p output near the body.  The outer at WIN=40/RES=400 is only 5 source-
# px/m but the inferno fade turns near-zero |v_rel| pixels black anyway, so
# the coarse far-field reads as smooth dark background.
WIN_INNER = 10.0
RES_INNER = 1024
LIC_LEN_INNER = 40
WIN_OUTER = 40.0
RES_OUTER = 400
LIC_LEN_OUTER = 20
# Aliases kept for older callers and the auto-scale logic in main().
WIN   = WIN_OUTER
RES   = RES_OUTER
LIC_LEN = LIC_LEN_OUTER
MACH_REL_MAX = 0.10
DT    = 0.025
WARMUP_OFFSET = 30
# Output window (overridden by --width/--height).
WINDOW_SIZE: tuple[int, int] = (1400, 900)

# Per-step surface directory layout.  Each chunk has its own directory of
# per-time .pvtu files extracted from surfaces.tar.gz.
SURF_DIRS = {
    "chunk1": Path("/tmp/surf_chunk1_perstep"),
    "chunk2": Path("/tmp/surf_chunk2_perstep"),
    "chunk3": Path("/tmp/surf_chunk3_perstep"),
    "chunk4": Path("/tmp/surf_chunk4_perstep"),
}
# Per-step slice directory layout.  From the SAME new chain (so the slice's
# flow matches the surfaces' body pose at every step).
SLICE_DIRS = {
    "chunk1": Path("/tmp/slices_chunk1_new"),
    "chunk2": Path("/tmp/slices_chunk2_new"),
    "chunk3": Path("/tmp/slices_chunk3_new"),
    "chunk4": Path("/tmp/slices_chunk4_new"),
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


def compute_visible_y0_bbox(cam_pos: tuple[float, float, float],
                            look_at: tuple[float, float, float],
                            view_up: tuple[float, float, float],
                            view_angle_deg: float,
                            aspect: float,
                            margin_frac: float = 0.08,
                            max_extent: float = 250.0,
                            ) -> tuple[float, float, float, float]:
    """Axis-aligned bbox in the y=0 plane that exactly covers the camera's
    view frustum, plus a small margin so the plane edge is never on-screen.

    For each of the 4 screen corners, build the world-space ray and
    intersect with y=0.  Some rays may be near-parallel to y=0 → clamp the
    intersection to within ±max_extent of look_at to keep the plane finite.

    Returns (x_min, x_max, z_min, z_max).  The y=0 slice plane should be
    built with these bounds (replacing the old fixed WIN=±10 m square)."""
    cam = np.asarray(cam_pos, dtype=np.float64)
    lat = np.asarray(look_at, dtype=np.float64)
    up  = np.asarray(view_up, dtype=np.float64)
    fwd = lat - cam; fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, up); right /= np.linalg.norm(right)
    up_s  = np.cross(right, fwd); up_s /= np.linalg.norm(up_s)

    half_v = math.tan(math.radians(view_angle_deg) * 0.5)
    half_h = half_v * aspect
    corners = []
    for sx in (-1.0, +1.0):
        for sy in (-1.0, +1.0):
            ray = fwd + (sx * half_h) * right + (sy * half_v) * up_s
            # Intersect with y=0 plane:  cam_y + ray_y * t = 0
            if abs(ray[1]) < 1e-6:
                # Ray parallel to y=0; project look-at point with max_extent
                tx = lat[0]; tz = lat[2]
            else:
                t = -cam[1] / ray[1]
                if t < 0:
                    tx = lat[0]; tz = lat[2]   # ray goes away from y=0
                else:
                    tx = cam[0] + ray[0] * t
                    tz = cam[2] + ray[2] * t
            # Clamp the intersection to a sane region around look-at, so a
            # near-grazing ray doesn't extend the bbox to infinity.
            tx = float(np.clip(tx, lat[0] - max_extent, lat[0] + max_extent))
            tz = float(np.clip(tz, lat[2] - max_extent, lat[2] + max_extent))
            corners.append((tx, tz))
    xs = [c[0] for c in corners]
    zs = [c[1] for c in corners]
    x_min, x_max = min(xs), max(xs)
    z_min, z_max = min(zs), max(zs)
    dx = (x_max - x_min) * margin_frac
    dz = (z_max - z_min) * margin_frac
    return (x_min - dx, x_max + dx, z_min - dz, z_max + dz)


def chase_anchor(step: int, cg: dict[int, tuple[float, float, float]],
                 lag_s: float) -> tuple[float, float]:
    """Anchor point for the chase camera, in mesh chase-view coordinates.

    With lag_s == 0 the anchor is the current CG (matches the original
    fixed-on-body camera).  With lag_s > 0 the anchor is the body's
    GROUND-FRAME position from lag_s seconds ago, expressed in chase view
    via the same Galilean shift used for the trajectory polyline:

        anchor_x = x_mesh(s_lag) + V_inf · (t_now − t_lag)
        anchor_z = z_mesh(s_lag)

    Result: the camera trails behind the body through still air at a
    fixed time interval, like a real chase aircraft."""
    if lag_s <= 0:
        x, z, _ = cg[step]
        return x, z
    lag_steps = int(round(lag_s / DT))
    earliest  = min(cg.keys())
    s_lag     = max(step - lag_steps, earliest)
    dt_actual = (step - s_lag) * DT
    x_m, z_m, _ = cg[s_lag]
    return (x_m + V_INF * dt_actual, z_m)


def build_trajectory_polyline(cg: dict[int, tuple[float, float, float]],
                              up_to_step: int) -> pv.PolyData:
    """Polyline of the body's historical CG path in the GROUND frame
    (air at rest), expressed in chase-view coordinates anchored at the
    current body's mesh-frame CG.

    Mesh frame (CFD `WORLD`): freestream BC has air at (+V_inf, 0, 0) — the
    air moves in +x at V_inf m/s.  Ground frame: air at rest; the two are
    related by a Galilean shift X_ground(t) = x_mesh(t) − V_inf·t.

    For a chase camera fixed at the current body's mesh position, the past
    CG at step s lands at:

        x_chase(s) = x_mesh(s) + V_inf · (t_now − t_s)

    so the historical track trails downstream of the body (in +x) by
    V_inf·Δt — the physically-correct flight path through still air.
    Z is unchanged (no vertical wind)."""
    steps = sorted(s for s in cg.keys() if s <= up_to_step)
    pts = np.empty((len(steps), 3), dtype=np.float32)
    for i, s in enumerate(steps):
        x_m, z_m, _ = cg[s]
        dt_since = (up_to_step - s) * DT     # seconds since past step (≥ 0)
        pts[i] = (x_m + V_INF * dt_since, 0.0, z_m)
    if len(pts) < 2:
        return pv.PolyData(pts)
    n = len(pts)
    lines = np.empty(n + 1, dtype=np.int64)
    lines[0] = n
    lines[1:] = np.arange(n)
    return pv.PolyData(pts, lines=lines)

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
def _lic_panel(sl: "pv.PointSet", CGx: float, CGz: float,
               win: float, res: int, lic_len: int, out_png: Path) -> None:
    """Sample an already-loaded y=0 slice over a 2·win × 2·win m window and
    write the LIC+Mach_rel texture (inferno cmap, zero-mag pixels = black)."""
    grid = pv.ImageData()
    grid.dimensions = (res, 1, res)
    grid.origin     = (CGx - win, 0, CGz - win)
    grid.spacing    = (2*win/(res-1), 1, 2*win/(res-1))
    sampled = grid.sample(sl)
    v = sampled.point_data["velocity"].reshape(res, res, 3)
    v_rel = v.copy(); v_rel[:, :, 0] -= M_INF
    vx = v_rel[:, :, 0].astype(np.float32)
    vz = v_rel[:, :, 2].astype(np.float32)
    mag = np.linalg.norm(v_rel, axis=2)
    mag_norm = np.clip(mag / MACH_REL_MAX, 0, 1).astype(np.float32)
    tex = lic_lib.lic(vz, vx, length=lic_len)
    rgb = (plt.get_cmap("inferno")(mag_norm)[..., :3]
           * (0.45 + 0.55 * tex[..., None]))
    rgb = np.clip(rgb, 0, 1)
    rgb = np.flipud(rgb)   # PIL row 0 = top of PNG; OpenGL v=0 = bottom
    Image.fromarray((rgb*255).astype(np.uint8)).save(out_png)


def build_dual_slice_textures(pvtu_path: str, CGx: float, CGz: float,
                              outer_png: Path, inner_png: Path) -> None:
    """Read the y=0 slice once, then sample it twice — at the coarse outer
    window (large area, low source-density) and the fine inner window
    (small area centred on CG, high source-density).

    The two textures will be applied to two coplanar y≈0 planes; the inner
    plane is offset 0.05 m toward the camera so it z-orders on top of the
    outer plane.  `inferno(0) ≈ (0,0,0)` makes both textures' low-|v_rel|
    pixels black, so the inner→outer transition is essentially invisible
    in the dark-background regions and the outer's coarse pixels are
    naturally hidden behind the inner near the body."""
    sl = pv.read(pvtu_path)
    _lic_panel(sl, CGx, CGz, WIN_OUTER, RES_OUTER, LIC_LEN_OUTER, outer_png)
    _lic_panel(sl, CGx, CGz, WIN_INNER, RES_INNER, LIC_LEN_INNER, inner_png)

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

    # Camera anchor + pose (lagged in ground-frame chase coords, with the
    # camera offset added on top); the look-at always tracks the CURRENT CG.
    anchor_x, anchor_z = chase_anchor(step, cg, LAG_S)
    cam_pos = (anchor_x + CAM_OFFSET[0], CAM_OFFSET[1], anchor_z + CAM_OFFSET[2])
    look_at = (CGx, 0.0, CGz)
    view_up = (0.0, 0.0, 1.0)

    # Build the dual LIC textures (outer = full 80 m background, inner = 20 m
    # high-resolution near-body region).  Two coplanar y≈0 planes:
    #   outer at y = 0
    #   inner at y = -0.05  (toward camera, so it z-orders on top of outer)
    outer_png = FRAMES / f".tex_{step:05d}_o.png"
    inner_png = FRAMES / f".tex_{step:05d}_i.png"
    build_dual_slice_textures(slice_pvtu, CGx, CGz, outer_png, inner_png)

    def _make_plane(half: float, y_offset: float) -> pv.PolyData:
        pts = np.array([
            [CGx - half, y_offset, CGz - half], [CGx + half, y_offset, CGz - half],
            [CGx + half, y_offset, CGz + half], [CGx - half, y_offset, CGz + half],
        ])
        pl = pv.PolyData(pts, faces=np.array([4, 0, 1, 2, 3]))
        pl.active_texture_coordinates = np.array(
            [[0, 0], [1, 0], [1, 1], [0, 1]], dtype=np.float32)
        return pl

    plane_outer = _make_plane(WIN_OUTER, y_offset=0.0)
    plane_inner = _make_plane(WIN_INNER, y_offset=-0.05)
    tex_outer = pv.Texture(str(outer_png))
    tex_inner = pv.Texture(str(inner_png))

    p = pv.Plotter(off_screen=True, window_size=WINDOW_SIZE)
    p.add_mesh(plane_outer, texture=tex_outer)
    p.add_mesh(plane_inner, texture=tex_inner)
    # Load body parts at this step's surface_step IN NATIVE WORLD COORDINATES
    for part_name, prefix in PART_PREFIX.items():
        path = surf_dir / f"{prefix}_time_{surf_step}.pvtu"
        if not path.exists(): continue
        m = pv.read(str(path))
        p.add_mesh(m, color=PART_COLORS[part_name], smooth_shading=True)
    # Historical CG trajectory up to this step (polyline in y=0 plane).
    # Rendered as a thin tube in a bright accent colour so it shows through
    # the slice texture.
    traj = build_trajectory_polyline(cg, up_to_step=step)
    if traj.n_points >= 2:
        tube = traj.tube(radius=0.08)
        p.add_mesh(tube, color="white", smooth_shading=False)
        # Highlight the current position with a small sphere
        p.add_mesh(pv.Sphere(radius=0.20, center=(CGx, 0, CGz)),
                   color="white", smooth_shading=True)
    p.set_background("black")
    p.camera_position = [cam_pos, look_at, view_up]
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
    for tp in (outer_png, inner_png):
        try: tp.unlink()
        except Exception: pass
    return True


def main(skip: int = 4, max_frames: int | None = None,
         lag_s: float = 0.0,
         window_size: tuple[int, int] = (1400, 900)) -> None:
    global FRAMES, OUT_MP4, LAG_S, WINDOW_SIZE, RES, LIC_LEN
    LAG_S = lag_s
    WINDOW_SIZE = window_size
    W, H = window_size
    # Dual-plane mode: the LIC resolution is set per-plane via the
    # RES_INNER/RES_OUTER module globals (which the renderer reads
    # directly), independent of the output window size.  No per-frame
    # bump needed here.
    print(f"  slice: inner WIN={WIN_INNER} RES={RES_INNER} LIC_LEN={LIC_LEN_INNER}")
    print(f"  slice: outer WIN={WIN_OUTER} RES={RES_OUTER} LIC_LEN={LIC_LEN_OUTER}")
    suffix_parts = []
    if lag_s > 0:
        suffix_parts.append(f"lag{lag_s:g}s".replace(".", "p"))
    if (W, H) != (1400, 900):
        suffix_parts.append(f"{W}x{H}")
    suffix = ("_" + "_".join(suffix_parts)) if suffix_parts else ""
    FRAMES  = OUT / f"lic_frames_3d_native{suffix}"
    OUT_MP4 = OUT / f"nokick_landing_3d_native{suffix}.mp4"
    FRAMES.mkdir(parents=True, exist_ok=True)
    print(f"FRAMES = {FRAMES}")
    print(f"OUT_MP4 = {OUT_MP4}")
    print(f"LAG_S = {LAG_S} s")
    cg = load_cg_lookup()
    all_files: list[tuple[int, str]] = []
    # Iterate the NEW per-chunk slice dirs, NOT the legacy slices_chunk*/ ones
    # which were from the old chain (different trajectory).  The new chain
    # writes the y=0 slice as "slice_y=+0.0_time_<step>.pvtu" (the slice's
    # canonical name is `y={y_m:+.1f}` in unsteady_setup.py, so 0.0 → +0.0).
    for slice_dir in SLICE_DIRS.values():
        if not slice_dir.exists(): continue
        for fp in (glob.glob(str(slice_dir / "slice_y=+0.0_time_*.pvtu"))
                    + glob.glob(str(slice_dir / "slice_y=0_time_*.pvtu"))):
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

    out_mp4 = OUT_MP4
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
    ap.add_argument("--lag-s", type=float, default=0.0,
                    help="chase-camera lag (s); 0 = anchored on current CG")
    ap.add_argument("--width",  type=int, default=1400, help="output width px")
    ap.add_argument("--height", type=int, default=900,  help="output height px")
    args = ap.parse_args()
    main(skip=args.skip, max_frames=args.max_frames,
         lag_s=args.lag_s, window_size=(args.width, args.height))
