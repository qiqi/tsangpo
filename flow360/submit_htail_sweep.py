"""
H-tail incidence sweep — 10 forks of a parent case, each pinning a
DIFFERENT constant θ_htail (α pinned at +7°, T_mult = 1).
Uses the shared `cfd_setup.build_params(...)` builder.

    TSANGPO_PARENT_CASE_ID=case-... python3 flow360/submit_htail_sweep.py
"""
from __future__ import annotations

import os
import sys
from math import radians
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
import cfd_setup as C
import flow360 as fl

ALPHA_EFF_DEG    = 7.0
HTAIL_VALUES_DEG = (-12.0, -9.0, -6.0, -3.0, 0.0, +3.0, +6.0, +9.0, +12.0, +15.0)
N_STEPS_TOTAL    = 20

parent_case = fl.Case.from_cloud(os.environ["TSANGPO_PARENT_CASE_ID"])
project     = fl.Project.from_cloud(parent_case.project_id)
surfaces    = C.get_geometry_surfaces(project)
print(f"Forking htail sweep from {parent_case.id} on {project.id} "
      f"({project.metadata.name})")
print(f"  α_eff (pinned) = {ALPHA_EFF_DEG}°")

submitted = []
for theta_deg in HTAIL_VALUES_DEG:
    params = C.build_params(surfaces,
                            theta_ac_rad=radians(ALPHA_EFF_DEG),
                            theta_ht_rad=radians(theta_deg),
                            thrust_mult=1.0,
                            n_steps_total=N_STEPS_TOTAL)
    name = (f"htail_{theta_deg:+.0f}deg"
            .replace("+", "p").replace("-", "m"))
    case = project.run_case(
        params=params, name=name, run_async=True,
        fork_from=parent_case,
        tags=["SI", f"alpha{int(ALPHA_EFF_DEG)}", "htail_sweep",
              f"htail{theta_deg:+.1f}", "constant_angle_fork"],
        use_beta_mesher=True,
    )
    submitted.append((theta_deg, case.id))
    print(f"  θ_htail = {theta_deg:+5.1f}°  →  {case.id}")

print("\nSubmitted htail-sweep forks:")
for v, cid in submitted:
    print(f"  θ_htail = {v:+5.1f}°  {cid}")
