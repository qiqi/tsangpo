"""Render the y=0 symmetry-plane flow animation from the per-step slices of
the lag-fixed phugoid chunks that carry slice output (chunks 4-6).

Per-step slices are named slice_y=0_time_<physicalStep>.pvtu (201 per chunk).
The slice spans the whole farfield, so we use a CHASE CAMERA: each frame is
cropped to a fixed window around the body CG at that step (looked up from the
partial_traj CSVs).  Field = Mach, plotted with matplotlib tricontourf
(headless-safe; pyvista only reads the mesh).

Step → global time: t = (step - 20) * 0.05.  Chunk4 = steps 620-819 (30-40s),
chunk5 = 820-1019 (40-50s), chunk6 = 1020-1219 (50-60s).

Outputs (idempotent — skips already-rendered frames):
  - phugoid_runs/htail_p1p5deg/slice_frames/frame_<step>.png
  - phugoid_runs/htail_p1p5deg/nokick_lagfix_slices.mp4
"""
from __future__ import annotations
import sys, re, tarfile, glob, csv, subprocess
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.tri as mtri

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
import json
import flow360 as fl
import pyvista as pv

OUT = REPO / 'post/v3/phugoid_runs/htail_p1p5deg'
FRAMES = OUT / 'slice_frames'
FRAMES.mkdir(parents=True, exist_ok=True)
DT = 0.05
WIN = 12.0          # chase-camera half-window [m] around the CG
MACH_MAX = 0.25     # color scale ceiling

_ids = json.loads((REPO / 'post/v3/unsteady_project_ids.json').read_text())
CHUNK1 = 'case-84ed883f-2dfb-4f78-a390-db30c8e4385d'

# ---- step → (x_cg, z_cg, theta_body) from all partial_traj CSVs ----
def build_cg_lookup():
    cids = [CHUNK1]
    for n in range(2, 7):
        k = f'phugoid_gap110_nokick_lagfix_chunk{n}'
        if k in _ids: cids.append(_ids[k])
    table = {}
    for cid in cids:
        p = REPO / 'post/v3' / f'partial_traj_{cid}.csv'
        if not p.exists(): continue
        for r in csv.DictReader(p.open()):
            table[int(r['step'])] = (float(r['x_cg_m']), float(r['z_cg_m']),
                                     float(r['theta_body_deg']))
    return table

CG = build_cg_lookup()


def render_slice(pvtu_path, step):
    frame_png = FRAMES / f'frame_{step:05d}.png'
    if frame_png.exists():
        return True
    if step not in CG:
        return False
    xc, zc, th = CG[step]
    m = pv.read(str(pvtu_path))
    if m.n_points == 0:
        return False
    pts = m.points
    x, z = pts[:, 0], pts[:, 2]
    mach = np.asarray(m.point_data['Mach']).ravel()
    # crop to chase window
    sel = (x > xc - WIN) & (x < xc + WIN) & (z > zc - WIN) & (z < zc + WIN)
    if sel.sum() < 50:
        return False
    xs, zs, ms = x[sel], z[sel], mach[sel]

    fig, ax = plt.subplots(figsize=(8, 8))
    try:
        tri = mtri.Triangulation(xs, zs)
        tcf = ax.tricontourf(tri, ms, levels=np.linspace(0, MACH_MAX, 41),
                             cmap='turbo', extend='max')
    except Exception:
        tcf = ax.scatter(xs, zs, c=ms, s=3, cmap='turbo', vmin=0, vmax=MACH_MAX)
    plt.colorbar(tcf, ax=ax, label='Mach', fraction=0.046)
    ax.set_xlim(xc - WIN, xc + WIN); ax.set_ylim(zc - WIN, zc + WIN)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('x [m]'); ax.set_ylabel('z [m]')
    t_glob = (step - 20) * DT
    ax.set_title(f'y=0 Mach   t={t_glob:.2f}s   θ_body={th:+.1f}°', fontsize=11)
    fig.savefig(frame_png, dpi=110, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    return True


def main():
    sliced = []
    for n in range(4, 7):
        k = f'phugoid_gap110_nokick_lagfix_chunk{n}'
        if k in _ids: sliced.append((n, _ids[k]))
    if not sliced:
        print('no sliced chunks'); return

    rendered = []
    for n, cid in sliced:
        try:
            st = str(fl.Case.from_cloud(case_id=cid).status).split('.')[-1]
        except Exception as e:
            print(f'  chunk{n}: {e}'); continue
        if st not in ('COMPLETED', 'COMPLETED_WITH_WARNINGS'):
            print(f'  chunk{n}: {st} — not ready'); continue
        dl = OUT / f'slices_chunk{n}'
        dl.mkdir(exist_ok=True)
        if not list(dl.glob('slice_y=0_time_*.pvtu')):
            if not (dl / 'slices.tar.gz').exists():
                print(f'  chunk{n}: downloading slices …')
                fl.Case.from_cloud(case_id=cid).results.slices.download(
                    to_folder=str(dl), overwrite=True)
            for tgz in glob.glob(str(dl / '*.tar.gz')):
                with tarfile.open(tgz) as tf: tf.extractall(dl)
        # one frame per step, from the per-step .pvtu (aggregates proc parts)
        files = {}
        for f in glob.glob(str(dl / 'slice_y=0_time_*.pvtu')):
            mm = re.search(r'time_(\d+)', Path(f).name)
            if mm: files[int(mm.group(1))] = f
        print(f'  chunk{n}: {len(files)} per-step slices')
        for s in sorted(files):
            if render_slice(files[s], s):
                rendered.append(s)
    print(f'  total frames on disk: {len(list(FRAMES.glob("frame_*.png")))}')
    out_mp4 = OUT / 'nokick_lagfix_slices.mp4'
    r = subprocess.run(['ffmpeg', '-y', '-framerate', '20', '-pattern_type', 'glob',
                        '-i', str(FRAMES / 'frame_*.png'),
                        '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-vf',
                        'pad=ceil(iw/2)*2:ceil(ih/2)*2', str(out_mp4)],
                       capture_output=True, text=True)
    if r.returncode == 0:
        print(f'  wrote {out_mp4}')
    else:
        print(f'  ffmpeg failed: {r.stderr[-300:]}')


if __name__ == '__main__':
    main()
