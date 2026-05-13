"""
Takeoff Phase C — refined trim sweep on a tightened legacy mesh.

Damped Newton step from the Phase A v2 linear-trim solve
(post/TAKEOFF_PLAN.md "Phase B" section). The raw solve wanted
α=+3.3°, θ_ht=−33°, T_mult=+4, but Δθ_ht=−33° is unphysical (htail
slipstream-wake authority too weak — finding documented).  Phase C
target is a damped, envelope-clipped point:

    α = +5°, θ_ht = −12°, T_mult = +8

with tightened mesh — surface_max_edge_length=0.04 m and
curvature_resolution_angle=10° (vs Phase A's 0.075/15°). GAI is
still blocked (see post/FLOW360_GAI_BUG_REPORT.md), so legacy beta
mesher.

Submits a single parent at the refined trim.  Forks (if desired)
can be added on top with the same env-var contract as Phase A.

    python3 flow360/submit_takeoff_phaseC.py
"""
from __future__ import annotations

import os
import sys
from math import radians
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
import cfd_setup as C
import params as P
import flow360 as fl

ALPHA_TRIM_DEG    = +5.0
THETA_HT_TRIM_DEG = -12.0
T_MULT_TRIM       = +8.0

N_STEPS_TOTAL = 20
PSEUDO_STEPS  = 1000

# Tightened legacy-mesh settings for a finer answer than Phase A
SURFACE_MAX_EDGE_M = 0.04          # vs 0.075 Phase A (~2× finer)
CURVATURE_RES_DEG  = 10.0          # vs 15 Phase A

PROJECT_ID = os.environ.get(
    "TSANGPO_TAKEOFF_PROJECT_ID",
    "prj-25133de4-f045-4901-895a-5d3f241dc675",
)
project  = fl.Project.from_cloud(PROJECT_ID)
surfaces = C.get_geometry_surfaces(project)
print(f"Project: {project.id} ({project.metadata.name})")
print(f"Refined trim: α={ALPHA_TRIM_DEG:+.1f}°, θ_ht={THETA_HT_TRIM_DEG:+.1f}°, "
      f"T_mult={T_MULT_TRIM:.1f}")
print(f"Mesh: surface_max_edge={SURFACE_MAX_EDGE_M} m, "
      f"curvature_resolution={CURVATURE_RES_DEG}°")

params = C.build_params(
    surfaces,
    theta_ac_rad=radians(ALPHA_TRIM_DEG),
    theta_ht_rad=radians(THETA_HT_TRIM_DEG),
    thrust_mult=T_MULT_TRIM,
    velocity_m_s=P.V_TAKEOFF_M_S,
    altitude_m=P.ALT_TAKEOFF_M,
    n_steps_total=N_STEPS_TOTAL,
    max_pseudo_steps=PSEUDO_STEPS,
    surface_max_edge_length_m=SURFACE_MAX_EDGE_M,
    curvature_resolution_deg=CURVATURE_RES_DEG,
)
case = project.run_case(
    params=params,
    name="takeoff_phaseC_refined_trim",
    run_async=True,
    tags=["SI", "takeoff", "phase1", "phaseC", "tightened_mesh",
          "refined_trim", f"alpha{ALPHA_TRIM_DEG:+.0f}",
          f"htail{THETA_HT_TRIM_DEG:+.0f}", f"thrust{T_MULT_TRIM:.0f}"],
    use_beta_mesher=True,
)
print(f"\nPhase C parent: {case.id}")
