"""
Actuator-disk thrust sweep — 10 forks of a parent case, each pinning a
DIFFERENT commanded `force_per_area` multiplier (α pinned at +7°,
θ_htail = 0).  Uses the shared `cfd_setup.build_params(...)` builder.

    TSANGPO_PARENT_CASE_ID=case-... python3 flow360/submit_thrust_sweep.py
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

THRUST_MULTIPLIERS = (0.0, 0.25, 0.50, 0.75, 1.00, 1.25, 1.50, 2.00, 2.50, 3.00)
N_STEPS_TOTAL      = 20

parent_case = fl.Case.from_cloud(os.environ["TSANGPO_PARENT_CASE_ID"])
project     = fl.Project.from_cloud(parent_case.project_id)
surfaces    = C.get_geometry_surfaces(project)
print(f"Forking thrust sweep from {parent_case.id} on {project.id} "
      f"({project.metadata.name})")

submitted = []
for mult in THRUST_MULTIPLIERS:
    params = C.build_params(surfaces,
                            theta_ac_rad=radians(7.0),
                            theta_ht_rad=0.0,
                            thrust_mult=mult,
                            n_steps_total=N_STEPS_TOTAL)
    name = f"thrust_x{mult:.2f}".replace(".", "p")
    case = project.run_case(
        params=params, name=name, run_async=True,
        fork_from=parent_case,
        tags=["SI", "alpha7", "htail0", "thrust_sweep",
              f"mult{mult:.2f}", "hardcoded_FPA_fork"],
        use_beta_mesher=True,
    )
    submitted.append((mult, case.id))
    print(f"  thrust × {mult:.2f}  →  {case.id}")

print("\nSubmitted thrust-sweep forks:")
for v, cid in submitted:
    print(f"  ×{v:.2f}  {cid}")
