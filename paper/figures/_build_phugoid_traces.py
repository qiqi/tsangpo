"""Time-history figure for the dynamic-maneuver section.

Six panels (2x3) showing the chunk-1 phugoid trajectory:
  1. apparent airspeed V_app [kt]
  2. flight-path angle γ_app [deg]
  3. apparent AoA α_app [deg]
  4. wing system CL
  5. htail CL
  6. moment about CG  M_CG [N·m]   (= UDD's state[9])

Output: paper/figures/v3_phugoid_traces.png
"""
from __future__ import annotations
import csv, math, re, sys
from io import StringIO
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import flow360 as fl

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]

CASE = "case-0e98708e-7938-4632-a536-22a97f0220e1"

# Constants
RHO   = 0.8491
A_INF = 325.95
V_INF = 80.0 / 1.94384449
S_REF = 15.33
MAC   = 1.385
qS    = 0.5 * RHO * V_INF * V_INF * S_REF
F_DIM = RHO * A_INF * A_INF        # solver-non-dim → SI scaling

# ---- Pull trajectory CSV (already extracted) ----
traj_csv = REPO / f"post/v3/partial_traj_{CASE}.csv"
rows = list(csv.DictReader(traj_csv.open()))
t_traj    = np.array([float(r["t_s"])           for r in rows])
x_cg      = np.array([float(r["x_cg_m"])        for r in rows])
z_cg      = np.array([float(r["z_cg_m"])        for r in rows])
th_body   = np.array([float(r["theta_body_rad"]) for r in rows])
Vx_world  = np.gradient(x_cg, t_traj)
Vz_world  = np.gradient(z_cg, t_traj)

# Apparent wind: V_app = V_∞ − V_aircraft (in world)
Vapp_x = V_INF - Vx_world
Vapp_z = -Vz_world
V_app  = np.hypot(Vapp_x, Vapp_z)
gamma_app = np.arctan2(-Vapp_z, Vapp_x)
alpha_app = th_body - gamma_app

# ---- Pull per-component CL from surface_forces (one sample per phys step) ----
c = fl.Case.from_cloud(case_id=CASE)
sf = c.results.surface_forces; sf.load_from_remote()
v = sf.values
ps = np.array(v["physical_step"])
WING_KEYS = [f"cyl3_airframe_pitch/{k}" for k in
              ("main_wing","vane_left","vane_right","aft_flap_left","aft_flap_right")]
HTAIL_KEY = "cyl4_htail_pitch/htail"
def sum_keys(keys, suffix):
    out = None
    for k in keys:
        col = np.array(v[k+suffix])
        out = col if out is None else out + col
    return out
wL = sum_keys(WING_KEYS, "_CL")
hL = np.array(v[HTAIL_KEY + "_CL"])
t_sf = (ps - 20) * 0.05

# ---- Pull state[9] = M_CG_nd from htail.omegaDot in user log ----
buf = StringIO(); save = sys.stdout; sys.stdout = buf
c.logs.set_remote_log_file_name('logs/flow360_case.user.log')
c.logs.print()
sys.stdout = save
log = buf.getvalue()
M_CG_steps = []
M_CG_si    = []
parts = re.split(r'(?=Physical step = )', log)
for part in parts:
    m = re.match(r'Physical step = (\d+)', part)
    if not m: continue
    step = int(m.group(1))
    head = part[:6000]
    h = re.search(r'phugoid_6DOF_cruise_htail\):(.*?)(?=User Defined Dynamics|\Z)', head, re.DOTALL)
    if not h: continue
    od_m = re.search(r'\(omegaDot = ([+-]?[\d.eE+-]+)\)', h.group(1))
    if not od_m: continue
    M_nd = float(od_m.group(1))
    M_CG_steps.append(step)
    M_CG_si.append(M_nd * F_DIM)
t_M  = (np.array(M_CG_steps) - 20) * 0.05
M_CG = np.array(M_CG_si)

# ---- Plot ----
fig, axes = plt.subplots(2, 3, figsize=(14, 7), constrained_layout=True)

axes[0,0].plot(t_traj, V_app * 1.94384449, color="#1f3a68", lw=1.6)
axes[0,0].axhline(80.0, color="gray", lw=0.5, ls=":")
axes[0,0].text(t_traj[-1], 81, "  $V_\\infty=80\\,$kt", color="gray", fontsize=8, va="bottom", ha="right")
axes[0,0].set_title("(a) Apparent airspeed $V_{\\mathrm{app}}$")
axes[0,0].set_xlabel("t [s]"); axes[0,0].set_ylabel("$V_{\\mathrm{app}}$ [kt]")
axes[0,0].grid(alpha=0.3)

axes[0,1].plot(t_traj, np.degrees(gamma_app), color="#9c3d1a", lw=1.6)
axes[0,1].axhline(0, color="gray", lw=0.5, ls=":")
axes[0,1].set_title("(b) Apparent flight-path angle $\\gamma_{\\mathrm{app}}$")
axes[0,1].set_xlabel("t [s]"); axes[0,1].set_ylabel("$\\gamma_{\\mathrm{app}}$ [deg]")
axes[0,1].grid(alpha=0.3)

axes[0,2].plot(t_traj, np.degrees(alpha_app), color="#1f7a3f", lw=1.6)
axes[0,2].axhline(8.57, color="gray", lw=0.5, ls=":")
axes[0,2].text(t_traj[-1], 9, "  $\\alpha_0 + \\gamma_0 = 8.57^\\circ$",
                color="gray", fontsize=8, va="bottom", ha="right")
axes[0,2].set_title("(c) Apparent AoA  $\\alpha_{\\mathrm{app}} = \\theta_b - \\gamma_{\\mathrm{app}}$")
axes[0,2].set_xlabel("t [s]"); axes[0,2].set_ylabel("$\\alpha_{\\mathrm{app}}$ [deg]")
axes[0,2].grid(alpha=0.3)

axes[1,0].plot(t_sf, wL, color="#1f3a68", lw=1.6)
axes[1,0].set_title("(d) Wing system $C_L$  (main + vane + aft-flap)")
axes[1,0].set_xlabel("t [s]"); axes[1,0].set_ylabel("$C_L^{\\mathrm{wing}}$")
axes[1,0].grid(alpha=0.3)

axes[1,1].plot(t_sf, hL, color="#9c3d1a", lw=1.6)
axes[1,1].axhline(0, color="gray", lw=0.5)
axes[1,1].set_title("(e) Horizontal-tail $C_L$  (at $\\theta_{ht}=+1.5^\\circ$)")
axes[1,1].set_xlabel("t [s]"); axes[1,1].set_ylabel("$C_L^{\\mathrm{htail}}$")
axes[1,1].grid(alpha=0.3)

axes[1,2].plot(t_M, M_CG, color="#1f7a3f", lw=1.6)
axes[1,2].axhline(0, color="gray", lw=0.5)
axes[1,2].set_title("(f) Moment about CG  (UDD state[9])")
axes[1,2].set_xlabel("t [s]"); axes[1,2].set_ylabel("$M_{y,\\mathrm{CG}}$ [N$\\cdot$m]")
axes[1,2].grid(alpha=0.3)

fig.suptitle("v3 cruise phugoid (ps200, $\\theta_{ht}=+1.5^\\circ$, "
             "$\\gamma_0=+3^\\circ$): chunk-1 traces",
             fontsize=12)

out = HERE / "v3_phugoid_traces.png"
fig.savefig(out, dpi=180, bbox_inches="tight", pad_inches=0.04)
print(f"wrote {out}")
