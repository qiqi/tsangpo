"""
α calibration sweep — 10 forks of a parent case, each pinning a
DIFFERENT constant aircraft-pitch angle (θ_htail = 0, T_mult = 1).
Uses the shared `cfd_setup.build_params(...)` builder.

    TSANGPO_PARENT_CASE_ID=case-... python3 flow360/submit_alpha_sweep.py
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

ALPHA_VALUES_DEG = (-3.0, -1.0, +1.0, +3.0, +5.0, +7.0, +9.0, +11.0, +13.0, +15.0)
N_STEPS_TOTAL    = 20

parent_case = fl.Case.from_cloud(os.environ["TSANGPO_PARENT_CASE_ID"])
project     = fl.Project.from_cloud(parent_case.project_id)
surfaces    = C.get_geometry_surfaces(project)
print(f"Forking α-sweep from {parent_case.id} on {project.id} "
      f"({project.metadata.name})")

submitted = []
for alpha_deg in ALPHA_VALUES_DEG:
    params = C.build_params(surfaces,
                            theta_ac_rad=radians(alpha_deg),
                            theta_ht_rad=0.0,
                            thrust_mult=1.0,
                            n_steps_total=N_STEPS_TOTAL)
    name = (f"alpha_{alpha_deg:+.1f}deg"
            .replace("+", "p").replace("-", "m").replace(".", "p"))
    case = project.run_case(
        params=params, name=name, run_async=True,
        fork_from=parent_case,
        tags=["SI", "alpha_sweep", f"alpha{alpha_deg:+.1f}",
              "constant_angle_fork"],
        use_beta_mesher=True,
    )
    submitted.append((alpha_deg, case.id))
    print(f"  α = {alpha_deg:+5.1f}°  →  {case.id}")

print("\nSubmitted α-sweep forks:")
for v, cid in submitted:
    print(f"  α = {v:+5.1f}°  {cid}")
