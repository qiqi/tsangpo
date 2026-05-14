"""
Takeoff (flap phase 1) coarse-mesh campaign.

Uploads (or reuses, via TSANGPO_TAKEOFF_PROJECT_ID) the Flow360 project
with `despmtr phase 1` (takeoff flap deflection) baked into the
geometry, then submits:

  • Parent case at the back-of-envelope trim
        α=+8°, θ_ht=-5°, T_mult=+16   (post/TAKEOFF_PLAN.md)
  • Three 10-fork sweeps around it.

Convergence-tuned settings (post-mortem from case-95963eff):
  • Forks: 6 new physical steps after the parent's 20.  Forces from
    the parent settle within <0.1 % by step 5; we add 1 step of safety
    margin for the across-step transition into the new fork condition.
  • max_pseudo_steps = 500 per step.  Within-step CL is flat to 0.008%
    across the full 1000 pseudo iters in the takeoff regime (residual
    floor at ~7e-8, the dual-time pseudo solve has nothing more to do).

The PARENT keeps 1000 pseudo iters because it starts from freestream
and crosses through a larger transient.  Refine → relaunch on GAI mesh
once Phase A lands.

Run:
    python3 flow360/submit_takeoff_coarse_campaign.py
"""
from __future__ import annotations

import os
import sys
import tempfile
from math import radians
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
import cfd_setup as C
import params as P
import flow360 as fl

CSM      = REPO / "geometry" / "tsangpo.csm"
AIRFOILS = REPO / "geometry" / "airfoils"

# === Back-of-envelope initial trim guess (post/TAKEOFF_PLAN.md) ===
ALPHA_GUESS_DEG    = +8.0
THETA_HT_GUESS_DEG = -5.0
T_MULT_GUESS       = +16.0

# === Sweep ranges centred on the guess ===
ALPHA_SWEEP_DEG = (-2.0, +2.0, +5.0, +8.0, +11.0, +14.0, +17.0, +20.0, +25.0, +30.0)
# Extended htail sweep covering both stalled-negative AND positive-deflection
# regions — the wing downwash in heavy slipstream pushes the htail-frame flow
# very negative, so positive θ_ht is needed to unstall the htail.
HTAIL_SWEEP_DEG = (-15.0, -10.0, -5.0, 0.0, +5.0, +10.0, +15.0, +20.0, +25.0, +30.0)
THRUST_SWEEP    = (6.0, 9.0, 12.0, 14.0, 16.0, 18.0, 20.0, 22.0, 25.0, 30.0)

# === Time stepping (see header for derivation) ===
N_PARENT_STEPS = 20
N_FORK_NEW     = 6
N_FORK_TOTAL   = N_PARENT_STEPS + N_FORK_NEW
PSEUDO_PARENT  = 1000
PSEUDO_FORK    = 500

project_id = os.environ.get("TSANGPO_TAKEOFF_PROJECT_ID")
parent_id  = os.environ.get("TSANGPO_TAKEOFF_PARENT_CASE_ID")

if parent_id:
    parent_case = fl.Case.from_cloud(parent_id)
    project = fl.Project.from_cloud(parent_case.project_id)
    print(f"Forking sweeps from existing parent {parent_case.id}")
elif project_id:
    project = fl.Project.from_cloud(project_id)
    parent_case = None
else:
    inlined = C.set_csm_phase(C.inline_udcs(CSM, AIRFOILS), 1)
    with tempfile.NamedTemporaryFile("w", suffix=".csm", delete=False) as f:
        f.write(inlined)
        tmp_csm = f.name
    print(f"Uploading takeoff geometry (phase=1, {len(inlined.splitlines())} lines)…")
    project = fl.Project.from_geometry(
        tmp_csm, name="tsangpo_v2_takeoff_coarse",
        length_unit="m", tags=["tsangpo", "v2", "takeoff", "SI", "phase1", "coarse"],
    )
    C.move_project_to_folder(project, "5_v2_continuous_low_htail")
    parent_case = None
print(f"Project: {project.id} ({project.metadata.name})")

surfaces = C.get_geometry_surfaces(project)
print(f"V_takeoff  = {P.V_TAKEOFF_M_S:.2f} m/s  ({P.V_TAKEOFF_M_S/0.5144:.1f} kt)")
print(f"q·S        = {P.Q_TAKEOFF_PA * P.WING_AREA_M2:.1f} N\n")

# ---------------------------------------------------------------------------
# Parent at the BO-envelope estimate (full 1000 pseudo for the first solve).
# Skipped if TSANGPO_TAKEOFF_PARENT_CASE_ID was provided.
# ---------------------------------------------------------------------------
if parent_case is None:
    print(f"=== Parent: α={ALPHA_GUESS_DEG:+.1f}°, θ_ht={THETA_HT_GUESS_DEG:+.1f}°, "
          f"T_mult={T_MULT_GUESS:.1f}× ===")
    parent_case = project.run_case(
        params=C.build_params(surfaces,
                              theta_ac_rad=radians(ALPHA_GUESS_DEG),
                              theta_ht_rad=radians(THETA_HT_GUESS_DEG),
                              thrust_mult=T_MULT_GUESS,
                              velocity_m_s=P.V_TAKEOFF_M_S,
                              altitude_m=P.ALT_TAKEOFF_M,
                              n_steps_total=N_PARENT_STEPS,
                              max_pseudo_steps=PSEUDO_PARENT),
        name="takeoff_coarse_BO_estimate",
        run_async=True,
        tags=["SI", "takeoff", "phase1", "coarse", "parent", "BO_estimate"],
        use_beta_mesher=True,
    )
    print(f"  parent case = {parent_case.id}\n")

# ---------------------------------------------------------------------------
# Sweeps (fewer steps, fewer pseudo iters — see header).
# ---------------------------------------------------------------------------
def sweep(label, values, vary):
    submitted = []
    print(f"=== {label} sweep ===")
    for v in values:
        ta = radians(v) if vary == "alpha"  else radians(ALPHA_GUESS_DEG)
        th = radians(v) if vary == "htail"  else radians(THETA_HT_GUESS_DEG)
        tm = v          if vary == "thrust" else T_MULT_GUESS
        name = (f"TO_coarse_{vary}_{v:+.2f}"
                .replace("+", "p").replace("-", "m").replace(".", "p"))
        case = project.run_case(
            params=C.build_params(surfaces, ta, th, tm,
                                  velocity_m_s=P.V_TAKEOFF_M_S,
                                  altitude_m=P.ALT_TAKEOFF_M,
                                  n_steps_total=N_FORK_TOTAL,
                                  max_pseudo_steps=PSEUDO_FORK),
            name=name, run_async=True, fork_from=parent_case,
            tags=["SI", "takeoff", "coarse", f"{vary}_sweep", f"{vary}{v:+.2f}",
                  "tight_iters"],
            use_beta_mesher=True,
        )
        submitted.append((v, case.id))
        print(f"  {vary} = {v:+6.2f}  →  {case.id}")
    return submitted


alpha_cases  = sweep("α",      ALPHA_SWEEP_DEG, "alpha")
htail_cases  = sweep("θ_htail", HTAIL_SWEEP_DEG, "htail")
thrust_cases = sweep("T_mult",  THRUST_SWEEP,    "thrust")

print(f"\n=== Summary ===\nproject: {project.id} ({project.metadata.name})")
print(f"parent:  {parent_case.id}")
for name, lst in (("α",      alpha_cases),
                  ("θ_htail", htail_cases),
                  ("T_mult",  thrust_cases)):
    print(f"{name} ({len(lst)}):")
    for v, cid in lst:
        print(f"  {v:+7.2f}  {cid}")
