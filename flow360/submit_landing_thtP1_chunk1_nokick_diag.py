"""No-kick chunk1 forked from the LANDING θ_ht=+1° warmup.

Quick fork to see the dynamics direction at the midpoint trim
between baseline (nose-up div at -3.67°) and +3° (nose-down div).

Same time-stepping as the other chunk1 runs.

Usage:
    python3 flow360/submit_landing_thtP1_chunk1_nokick_diag.py
"""
from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

import cfd_setup as C
import flow360 as fl

_spec = importlib.util.spec_from_file_location(
    "unsteady_setup", REPO / "flow360" / "unsteady_setup.py")
U = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(U)

PROJECT_KEY      = "landing_thtP1_warmup"
PARENT_CASE_KEY  = "landing_thtP1_warmup_case"
N_STEPS          = 200
DT_S             = 0.025
MAX_PSEUDO_STEPS = 200


def main() -> str:
    ids_path = REPO / "post" / "v3" / "unsteady_project_ids.json"
    d = json.loads(ids_path.read_text())
    project = fl.Project.from_cloud(project_id=d[PROJECT_KEY])
    parent  = fl.Case.from_cloud(case_id=d[PARENT_CASE_KEY])
    print(f"  parent (θ_ht=+1° warmup): {parent.id}  status={parent.status}")

    surfaces = C.get_gapped_geometry_surfaces(project)
    params = U.build_unsteady_params(
        surfaces, phase="landing_thtP1", mode="phugoid",
        n_physical_steps=N_STEPS,
        timestep_size_s=DT_S,
        max_pseudo_steps=MAX_PSEUDO_STEPS,
        gamma_kick_deg=0.0,
        use_analytic_omegaDot=False,
        diagnostics=True,
        slice_frequency=1,
    )

    name = "v3_phugoid_landing_thtP1_chunk1_nokick_diag_5s"
    case = project.run_case(
        params=params, name=name, run_async=True, fork_from=parent,
        tags=["SI", "fork", "v3", "phugoid", "gap110", "chunk1",
              "nokick", "diag", "landing", "thtP1_retrim", "5s"],
        use_beta_mesher=True,
    )
    print(f"  submitted: {case.id}")
    d["phugoid_landing_thtP1_chunk1"] = case.id
    ids_path.write_text(json.dumps(d, indent=2))
    print(f"  saved phugoid_landing_thtP1_chunk1 -> {case.id}")
    return case.id


if __name__ == "__main__":
    main()
