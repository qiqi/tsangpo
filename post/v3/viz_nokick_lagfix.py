"""Trajectory + attitude visualization for the lag-fixed no-kick phugoid
chunk1 (case-84ed883f), first 10 s.

FRAME: air-relative.  The freestream blows +X at V_inf, so we subtract the
air's own motion (V_inf·t in X) to show the body's path THROUGH the air mass.
In this frame the body flies nose-first in -X and climbs — the intuitive
flight path.  CSM convention is +X = aft, so the nose unit vector is
(-cosθ, +sinθ) in (x,z): pointing -X, tilted up by the body pitch θ.

Outputs:
  - phugoid_runs/htail_p1p5deg/nokick_lagfix_trajectory.png  (static)
  - phugoid_runs/htail_p1p5deg/nokick_lagfix_attitude.mp4    (animation)
"""
from __future__ import annotations
import csv, sys
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
import json, subprocess
import flow360 as fl

# Auto-detect completed chunks of the lag-fixed no-kick chain, in time order.
# chunk1 is the diagnostic no-kick case; chunks 2-6 are registry-keyed forks.
# The physical-step counter continues across forks, so plot_running_case's
# t_s = (step-20)*0.05 is already globally continuous and x_cg/z_cg share one
# absolute frame.  We include every chunk that has COMPLETED.
CHUNK1 = 'case-84ed883f-2dfb-4f78-a390-db30c8e4385d'
_ids = json.loads((REPO / 'post/v3/unsteady_project_ids.json').read_text())
_ordered = [('chunk1', CHUNK1)]
for n in range(2, 7):
    k = f'phugoid_gap110_nokick_lagfix_chunk{n}'
    if k in _ids:
        _ordered.append((f'chunk{n}', _ids[k]))

CHUNK_CIDS = []
for label, cid in _ordered:
    try:
        st = str(fl.Case.from_cloud(case_id=cid).status).split('.')[-1]
    except Exception:
        st = 'ERR'
    if st not in ('COMPLETED', 'COMPLETED_WITH_WARNINGS'):
        print(f'  {label} ({cid[:13]}): {st} — stop including here')
        break
    # ensure the partial_traj CSV exists
    csv_p = REPO / 'post/v3' / f'partial_traj_{cid}.csv'
    if not csv_p.exists():
        print(f'  {label}: generating partial_traj …')
        subprocess.run(['python3', str(REPO / 'post/v3/plot_running_case.py'), cid, '80.0'],
                       check=True, capture_output=True)
    CHUNK_CIDS.append(cid)
    print(f'  including {label} ({cid[:13]})')

if not CHUNK_CIDS:
    print('no completed chunks'); sys.exit(0)

TSPAN = f'0-{len(CHUNK_CIDS)*10}s'
OUT = REPO / 'post/v3/phugoid_runs/htail_p1p5deg'
V_INF = 80.0 / 1.94384449     # freestream, +X (m/s)

t, x, z, th = [], [], [], []
for cid in CHUNK_CIDS:
    p = REPO / 'post/v3' / f'partial_traj_{cid}.csv'
    rows = list(csv.DictReader(p.open()))
    t  += [float(r['t_s']) for r in rows]
    x  += [float(r['x_cg_m']) for r in rows]
    z  += [float(r['z_cg_m']) for r in rows]
    th += [float(r['theta_body_deg']) for r in rows]
order = np.argsort(t)
t  = np.array(t)[order]
x  = np.array(x)[order]
z  = np.array(z)[order]
th = np.array(th)[order]

# Air-relative path: body displacement through the air mass
xr = (x - x[0]) - V_INF * t
zr = (z - z[0])

# Nose unit vector in (x,z): +X is aft → nose points -X, tilted up by θ.
a = np.radians(th)
nose = np.column_stack([-np.cos(a), np.sin(a)])     # (-cosθ, +sinθ)

# ---- Static figure ----
fig, axes = plt.subplots(2, 2, figsize=(14, 9), constrained_layout=True)

ax = axes[0, 0]
sc = ax.scatter(xr, zr, c=t, cmap='viridis', s=14, zorder=3)
Lb = 14.0
for i in range(0, len(t), 8):
    nx, nz = nose[i]
    # draw fuselage: tail (behind nose) to nose, with a dot at the nose
    ax.plot([xr[i] - nx*Lb, xr[i] + nx*Lb],
            [zr[i] - nz*Lb, zr[i] + nz*Lb], '-', color='crimson', lw=1.2, alpha=0.7, zorder=2)
    ax.plot([xr[i] + nx*Lb], [zr[i] + nz*Lb], 'o', color='crimson', ms=2.5, zorder=4)
ax.set_xlabel('x through air [m]  (−X = flight direction)')
ax.set_ylabel('altitude gain Δz [m]')
ax.set_title('Flight path through the air (color=time)\nred fuselage ticks, dot = nose')
plt.colorbar(sc, ax=ax, label='t [s]'); ax.grid(alpha=0.3); ax.axhline(0, color='k', lw=0.4)
ax.set_aspect('equal', adjustable='box')   # space-vs-space → equal aspect, no distortion
ax.invert_xaxis()   # so flight direction (-X) reads left→right as the body advances

ax = axes[0, 1]
ax.plot(t, th, '-', lw=2, color='#1f77b4')
ax.axhline(th[0], color='gray', ls=':', label=f'trim θ={th[0]:.1f}°')
ax.set_xlabel('t [s]'); ax.set_ylabel('θ_body [deg]  (+ = nose up)')
ax.set_title('Body pitch — phugoid oscillation'); ax.grid(alpha=0.3); ax.legend(fontsize=9)

ax = axes[1, 0]
ax.plot(t, zr, '-', lw=2, color='#2ca02c')
ax.set_xlabel('t [s]'); ax.set_ylabel('altitude gain Δz [m]')
ax.set_title('Altitude through the phugoid'); ax.grid(alpha=0.3); ax.axhline(0, color='k', lw=0.4)

ax = axes[1, 1]
omega = np.gradient(th, t)
ax.plot(t, omega, '-', lw=2, color='#d62728')
ax.set_xlabel('t [s]'); ax.set_ylabel('ω = dθ/dt [deg/s]')
ax.set_title('Pitch rate'); ax.grid(alpha=0.3); ax.axhline(0, color='k', lw=0.4)

fig.suptitle(f'Lag-fixed no-kick phugoid — first {TSPAN}, air-relative frame  '
             f'({len(CHUNK_CIDS)} chunks stitched)', fontsize=13, fontweight='bold')
out_png = OUT / 'nokick_lagfix_trajectory.png'
fig.savefig(out_png, dpi=130)
print(f'wrote {out_png}')

# ---- Animation ----
fig2, ax2 = plt.subplots(figsize=(10, 6))
ax2.set_xlim(xr.max()+20, xr.min()-20)    # inverted: flight dir left→right
ax2.set_ylim(zr.min()-20, zr.max()+20)
ax2.set_xlabel('x through air [m]  (flight direction →)'); ax2.set_ylabel('altitude Δz [m]')
ax2.grid(alpha=0.3); ax2.set_aspect('equal', adjustable='box')
path_line, = ax2.plot([], [], '-', color='#bbb', lw=1)
body_line, = ax2.plot([], [], '-', color='crimson', lw=3)
nose_dot,  = ax2.plot([], [], 'o', color='darkred', ms=6)
title = ax2.set_title('')

def anim(i):
    path_line.set_data(xr[:i+1], zr[:i+1])
    nx, nz = nose[i]; Lb = 16.0
    body_line.set_data([xr[i]-nx*Lb, xr[i]+nx*Lb], [zr[i]-nz*Lb, zr[i]+nz*Lb])
    nose_dot.set_data([xr[i]+nx*Lb], [zr[i]+nz*Lb])
    title.set_text(f't={t[i]:.2f}s   θ_body={th[i]:+.1f}°   (nose points −X, tilted up by θ)')
    return path_line, body_line, nose_dot, title

ani = animation.FuncAnimation(fig2, anim, frames=len(t), interval=50, blit=True)
out_mp4 = OUT / 'nokick_lagfix_attitude.mp4'
try:
    ani.save(str(out_mp4), writer='ffmpeg', dpi=110, fps=20)
    print(f'wrote {out_mp4}')
except Exception as e:
    print(f'mp4 save failed: {e}')
