"""Render the y=0 symmetry-plane Mach animation for the LANDING phugoid chain.

Each chunk (1-6) carries per-step slices (slice_frequency=1 from the submit
scripts).  Chase camera: ±WIN m around the CG at each step.  Field = Mach.

Differences from the cruise renderer:
  • DT = 0.025 s (cruise 0.05 s)
  • WARMUP_OFFSET = 30 (cruise 20) — step-to-time formula is t = (step−30)·0.025
  • Sliced range = chunks 1-6 (cruise was 4-6 because chunk1 there had
    slice_frequency=-1; landing chunk1 also has slice_frequency=1)
  • CG lookup parses the user log directly via viz_nokick_landing.parse_log
    (no partial_traj_*.csv intermediate)

Outputs (idempotent — skips already-rendered frames):
  post/v3/phugoid_runs/landing/slice_frames/frame_<step>.png
  post/v3/phugoid_runs/landing/nokick_landing_slices.mp4
"""
from __future__ import annotations

import glob
import json
import re
import subprocess
import sys
import tarfile
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.tri as mtri
import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
import flow360 as fl
import pyvista as pv

# Reuse the landing-phugoid log parser.
sys.path.insert(0, str(REPO / "post" / "v3"))
from viz_nokick_landing import download_user_log, parse_log  # noqa: E402

OUT = REPO / "post/v3/phugoid_runs/landing"
FRAMES = OUT / "slice_frames"
FRAMES.mkdir(parents=True, exist_ok=True)

DT = 0.025
WARMUP_OFFSET = 30
WIN = 8.0           # chase-camera half-window [m] — tighter than cruise
MACH_MAX = 0.12     # landing V_inf = 12.86 m/s ⇒ M_∞ ≈ 0.038; slipstream
                    # peaks much higher near the disks but stays well below
                    # cruise's 0.25 ceiling

_ids = json.loads((REPO / "post/v3/unsteady_project_ids.json").read_text())


def build_cg_lookup() -> dict[int, tuple[float, float, float]]:
    """step → (x_cg, z_cg, θ_body_deg) parsed from each completed chunk's log."""
    table: dict[int, tuple[float, float, float]] = {}
    cids: list[str] = []
    if "phugoid_landing_gap110_nokick_diag_chunk1" in _ids:
        cids.append(_ids["phugoid_landing_gap110_nokick_diag_chunk1"])
    for n in range(2, 7):
        k = f"phugoid_landing_gap110_nokick_chunk{n}"
        if k in _ids:
            cids.append(_ids[k])
    for cid in cids:
        try:
            st = str(fl.Case.from_cloud(case_id=cid).status).split(".")[-1]
        except Exception as e:
            print(f"  {cid[:13]}: status err: {e}"); continue
        if st not in ("COMPLETED", "COMPLETED_WITH_WARNINGS"):
            continue
        log = download_user_log(cid)
        for r in parse_log(log):
            import math
            if math.isnan(r["x_cg_m"]):
                continue
            table[int(r["step"])] = (float(r["x_cg_m"]),
                                      float(r["z_cg_m"]),
                                      float(r["theta_body_deg"]))
    return table


def render_slice(pvtu_path: str, step: int, cg: dict) -> bool:
    frame_png = FRAMES / f"frame_{step:05d}.png"
    if frame_png.exists():
        return True
    if step not in cg:
        return False
    xc, zc, th = cg[step]
    m = pv.read(str(pvtu_path))
    if m.n_points == 0:
        return False
    pts = m.points
    x, z = pts[:, 0], pts[:, 2]
    mach = np.asarray(m.point_data["Mach"]).ravel()
    sel = (x > xc - WIN) & (x < xc + WIN) & (z > zc - WIN) & (z < zc + WIN)
    if sel.sum() < 50:
        return False
    xs, zs, ms = x[sel], z[sel], mach[sel]
    fig, ax = plt.subplots(figsize=(8, 8))
    try:
        tri = mtri.Triangulation(xs, zs)
        tcf = ax.tricontourf(tri, ms, levels=np.linspace(0, MACH_MAX, 41),
                              cmap="turbo", extend="max")
    except Exception:
        tcf = ax.scatter(xs, zs, c=ms, s=3, cmap="turbo",
                          vmin=0, vmax=MACH_MAX)
    plt.colorbar(tcf, ax=ax, label="Mach", fraction=0.046)
    ax.set_xlim(xc - WIN, xc + WIN)
    ax.set_ylim(zc - WIN, zc + WIN)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]"); ax.set_ylabel("z [m]")
    t_glob = (step - WARMUP_OFFSET) * DT
    ax.set_title(f"y=0 Mach   t={t_glob:.2f}s   θ_body={th:+.1f}°"
                  f"   (landing, no-kick)", fontsize=11)
    fig.savefig(frame_png, dpi=110, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    return True


def main() -> None:
    sliced: list[tuple[str, str]] = []
    if "phugoid_landing_gap110_nokick_diag_chunk1" in _ids:
        sliced.append(("chunk1", _ids["phugoid_landing_gap110_nokick_diag_chunk1"]))
    for n in range(2, 7):
        k = f"phugoid_landing_gap110_nokick_chunk{n}"
        if k in _ids:
            sliced.append((f"chunk{n}", _ids[k]))
    if not sliced:
        print("no landing chunks submitted yet"); return

    cg = build_cg_lookup()
    print(f"  CG lookup: {len(cg)} steps")

    for label, cid in sliced:
        try:
            st = str(fl.Case.from_cloud(case_id=cid).status).split(".")[-1]
        except Exception as e:
            print(f"  {label}: {e}"); continue
        if st not in ("COMPLETED", "COMPLETED_WITH_WARNINGS"):
            print(f"  {label}: {st} — not ready"); continue
        dl = OUT / f"slices_{label}"
        dl.mkdir(exist_ok=True)
        if not list(dl.glob("slice_y=0_time_*.pvtu")):
            if not (dl / "slices.tar.gz").exists():
                print(f"  {label}: downloading slices …")
                fl.Case.from_cloud(case_id=cid).results.slices.download(
                    to_folder=str(dl), overwrite=True)
            for tgz in glob.glob(str(dl / "*.tar.gz")):
                with tarfile.open(tgz) as tf:
                    tf.extractall(dl)
        files = {}
        for f in glob.glob(str(dl / "slice_y=0_time_*.pvtu")):
            mm = re.search(r"time_(\d+)", Path(f).name)
            if mm:
                files[int(mm.group(1))] = f
        print(f"  {label}: {len(files)} per-step slices")
        for s in sorted(files):
            render_slice(files[s], s, cg)

    n_frames = len(list(FRAMES.glob("frame_*.png")))
    print(f"  total frames on disk: {n_frames}")
    if n_frames == 0:
        print("  no frames — skipping mp4"); return
    out_mp4 = OUT / "nokick_landing_slices.mp4"
    r = subprocess.run(["ffmpeg", "-y", "-framerate", "20",
                         "-pattern_type", "glob",
                         "-i", str(FRAMES / "frame_*.png"),
                         "-c:v", "libx264", "-pix_fmt", "yuv420p",
                         "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2",
                         str(out_mp4)],
                        capture_output=True, text=True)
    if r.returncode == 0:
        print(f"  wrote {out_mp4}")
    else:
        print(f"  ffmpeg failed: {r.stderr[-300:]}")


if __name__ == "__main__":
    main()
