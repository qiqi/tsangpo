"""Demonstrate partial-trajectory extraction from a (running or completed)
Flow360 case via `case.logs.print()`.

The user log is a streaming file: at any point during a run, we can pull
the current bytes via the SDK and parse them to get the per-physical-step
state.  This works for RUNNING, COMPLETED, and ERROR statuses identically.

Usage:
    python3 post/v3/plot_running_case.py <case_id>
"""
from __future__ import annotations

import re
import sys
import math
import csv
from pathlib import Path
from io import StringIO

import numpy as np
import matplotlib.pyplot as plt

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
import flow360 as fl

L1 = L2 = 1038.215114993035   # 3× R_arm case (was 346.07 in old runs)
A_INF        = 325.96                          # speed of sound at h = 3658 m
RHO_INF      = 0.8491                          # kg/m³
S_REF        = 15.33                           # wing area, m²
F_DIM_PER_ND = RHO_INF * A_INF * A_INF         # solver-non-dim → N
# Coefficient conversion depends on V_inf (qS).  Provide a helper so plots
# can be made at the correct V_freestream for each case.
def f_nd_to_cl_factor(V_inf_kt: float) -> float:
    """CF = F_nd · 2 / (Mach² · S_ref) where Mach = V_inf / a_inf."""
    Mach = (V_inf_kt / 1.94384449) / A_INF
    return 2.0 / (Mach * Mach * S_REF)


def download_log(case_id: str) -> str:
    """Pull whatever the user log has at this moment.  Returns local path."""
    c = fl.Case.from_cloud(case_id=case_id)
    print(f"  case status: {c.status}")
    c.logs.set_remote_log_file_name('logs/flow360_case.user.log')
    buf = StringIO(); save = sys.stdout; sys.stdout = buf
    c.logs.print()
    sys.stdout = save
    out = f"/tmp/{case_id}_userlog.txt"
    Path(out).write_text(buf.getvalue())
    return out


def parse(log_path: str) -> list[dict]:
    log = Path(log_path).read_text()
    rows = []
    parts = re.split(r'(?=Physical step = )', log)
    for part in parts:
        m = re.search(r'Physical step = (\d+)', part)
        if not m: continue
        step = int(m.group(1))
        head = '\n'.join(part.split('\n')[:200])
        udd = {}
        for tag in ['shoulder', 'elbow', 'airframe', 'htail']:
            block_pat = (r'User Defined Dynamics \(phugoid_6DOF_cruise_'
                          + tag + r'\):(.*?)(?=User Defined Dynamics|\Z)')
            mblk = re.search(block_pat, head, re.DOTALL)
            if not mblk: continue
            blk = mblk.group(1)
            def grab(v):
                m2 = re.search(r'\(' + v + r' = ([+-]?[\d.eE+-]+|nan)\)', blk)
                if not m2: return float('nan')
                return float('nan') if m2.group(1)=='nan' else float(m2.group(1))
            udd[tag] = {'fX': grab('forceX'), 'fZ': grab('forceZ'),
                        'mY': grab('momentY'), 'th': grab('theta')}
        if not udd: continue

        t1 = udd['shoulder']['th']; t2 = udd['elbow']['th']
        th_afr = udd['airframe']['th']
        fX = udd['shoulder']['fX']; fZ = udd['shoulder']['fZ']
        mY = udd['shoulder']['mY']
        try:
            cos_t2 = math.cos(t2)
            r2 = 2 * L1 * L2 * (1 - cos_t2)
            r = r2 ** 0.5
            half = math.atan(L2 * math.sin(t2) / max(1e-9, L1 - L2 * cos_t2))
            psi = half - t1
            x_cg = r * math.cos(psi); z_cg = r * math.sin(psi)
            th_body = th_afr + t1 + t2
        except Exception:
            x_cg = z_cg = th_body = float('nan')

        rows.append({'step': step, 't_s': (step-20)*0.05,
                     'fX_nd': fX, 'fZ_nd': fZ, 'mY_nd': mY,
                     'x_cg_m': x_cg, 'z_cg_m': z_cg,
                     'theta_body_rad': th_body,
                     'theta_body_deg': math.degrees(th_body) if not math.isnan(th_body) else float('nan')})
    return rows


def plot(rows: list[dict], case_id: str, V_inf_kt: float) -> Path:
    coef = f_nd_to_cl_factor(V_inf_kt)
    # Required level-flight CL = W / qS at V_inf.
    W = 11565.0
    V_si = V_inf_kt / 1.94384449
    qS = 0.5 * RHO_INF * V_si * V_si * S_REF
    CL_req = W / qS
    valid = [r for r in rows if not math.isnan(r['x_cg_m'])]
    t      = np.array([r['t_s']           for r in valid])
    x      = np.array([r['x_cg_m']        for r in valid])
    z      = np.array([r['z_cg_m']        for r in valid])
    th_deg = np.array([r['theta_body_deg'] for r in valid])
    CL     = np.array([r['fZ_nd']*coef for r in valid])
    CD     = np.array([r['fX_nd']*coef for r in valid])

    fig, axes = plt.subplots(2, 2, figsize=(11, 7))

    ax = axes[0, 0]
    ax.plot(x, z, '-', color='#1f3a68', lw=1.2)
    ax.plot(x[0], z[0], 'go', label=f'start (t={t[0]:.2f} s)')
    ax.plot(x[-1], z[-1], 'rx', markersize=10, label=f'last (t={t[-1]:.2f} s)')
    theta_circ = np.linspace(0, 2*np.pi, 200)
    ax.plot(2*L1*np.cos(theta_circ), 2*L1*np.sin(theta_circ),
            ':', color='#888', lw=0.8, label=f'workspace r=2L={2*L1:.0f} m')
    ax.set_xlabel('x_cg world [m]'); ax.set_ylabel('z_cg world [m]')
    ax.set_aspect('equal'); ax.grid(alpha=0.3); ax.legend(fontsize=8)
    ax.set_title('CG trajectory in world frame')

    ax = axes[0, 1]
    ax.plot(t, th_deg, '-', color='#9c3d1a', lw=1.2)
    ax.axhline(17, color='#888', ls=':', lw=0.8, label='θ_body initial = α_BO + γ = 17°')
    ax.set_xlabel('t [s]'); ax.set_ylabel('θ_body [deg]')
    ax.grid(alpha=0.3); ax.legend(fontsize=8)
    ax.set_title('Body pitch')

    ax = axes[1, 0]
    ax.plot(t, CL, '-', color='#1f7a3f', lw=1.2, label='CL_CFD')
    ax.axhline(CL_req, color='#888', ls=':', lw=0.8,
                label=f'CL_req level ({CL_req:.2f}) at V={V_inf_kt:.0f} kt')
    ax.set_xlabel('t [s]'); ax.set_ylabel('CL')
    ax.grid(alpha=0.3); ax.legend(fontsize=8)
    ax.set_title('Lift coefficient')

    ax = axes[1, 1]
    ax.plot(t, CD, '-', color='#6a3a8a', lw=1.2, label='CD_CFD')
    ax.set_xlabel('t [s]'); ax.set_ylabel('CD')
    ax.grid(alpha=0.3); ax.legend(fontsize=8)
    ax.set_title('Drag coefficient')

    fig.suptitle(f'Partial trajectory — {case_id} ({len(valid)} valid steps, '
                 f't_max = {t[-1]:.2f} s)', fontsize=10)
    fig.tight_layout()
    out = REPO / 'post' / 'v3' / f'partial_traj_{case_id}.png'
    fig.savefig(out, dpi=140); plt.close(fig)
    return out


def main() -> None:
    case_id  = sys.argv[1] if len(sys.argv) > 1 else 'case-740bfd58-3d86-4b97-8dfd-1a22cd83fb81'
    V_inf_kt = float(sys.argv[2]) if len(sys.argv) > 2 else 65.0
    log_path = download_log(case_id)
    rows = parse(log_path)
    print(f"  parsed {len(rows)} physical steps")
    valid = [r for r in rows if not math.isnan(r['x_cg_m'])]
    if valid:
        r_last = (valid[-1]['x_cg_m']**2 + valid[-1]['z_cg_m']**2)**0.5
        print(f"  last valid: step={valid[-1]['step']}, t={valid[-1]['t_s']:.2f} s, "
              f"x={valid[-1]['x_cg_m']:.1f}, z={valid[-1]['z_cg_m']:.1f}, "
              f"r={r_last:.1f} m, θ_body={valid[-1]['theta_body_deg']:.1f}°")
    out_png = plot(rows, case_id, V_inf_kt)
    print(f"  wrote {out_png}")

    # CSV too
    out_csv = REPO / 'post' / 'v3' / f'partial_traj_{case_id}.csv'
    with open(out_csv, 'w') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        for r in rows: w.writerow(r)
    print(f"  wrote {out_csv}  ({len(rows)} rows)")


if __name__ == '__main__':
    main()
