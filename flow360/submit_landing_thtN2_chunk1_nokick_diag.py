"""LANDING-retrim (θ_ht=-2°) phugoid chunk1 — no-kick, instrumented.

Fork from the retrim warmup (case-b574f5fc, project prj-fe988f58).
Same time-stepping as the baseline chunk1: dt=0.025s × 200 steps = 5s,
γ_kick=0, diagnostics=True, slice_frequency=1.

Tests whether the small CMy improvement from the retrim (+1.241 → +1.011)
yields a noticeably different open-loop trajectory.  Expected outcome:
qualitatively similar divergence (the retrim only reduced the residual
by ~18%, not eliminated it), but with somewhat slower initial pitch-up
rate.

Usage:
    python3 flow360/submit_landing_thtN2_chunk1_nokick_diag.py
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

PROJECT_KEY      = "landing_thtN2_warmup"
PARENT_CASE_KEY  = "landing_thtN2_warmup_case"
N_STEPS          = 200
DT_S             = 0.025
MAX_PSEUDO_STEPS = 200


def main() -> str:
    ids_path = REPO / "post" / "v3" / "unsteady_project_ids.json"
    d = json.loads(ids_path.read_text())
    project = fl.Project.from_cloud(project_id=d[PROJECT_KEY])
    parent  = fl.Case.from_cloud(case_id=d[PARENT_CASE_KEY])
    print(f"  parent (retrim warmup): {parent.id}  status={parent.status}")

    surfaces = C.get_gapped_geometry_surfaces(project)
    params = U.build_unsteady_params(
        surfaces, phase="landing_thtN2", mode="phugoid",
        n_physical_steps=N_STEPS,
        timestep_size_s=DT_S,
        max_pseudo_steps=MAX_PSEUDO_STEPS,
        gamma_kick_deg=0.0,
        use_analytic_omegaDot=False,
        diagnostics=True,
        slice_frequency=1,
    )

    name = "v3_phugoid_landing_thtN2_chunk1_nokick_diag_5s"
    case = project.run_case(
        params=params, name=name, run_async=True, fork_from=parent,
        tags=["SI", "fork", "v3", "phugoid", "gap110", "chunk1",
              "nokick", "diag", "landing", "thtN2_retrim", "5s"],
        use_beta_mesher=True,
    )
    print(f"  submitted: {case.id}")
    d["phugoid_landing_thtN2_chunk1"] = case.id
    ids_path.write_text(json.dumps(d, indent=2))
    print(f"  saved phugoid_landing_thtN2_chunk1 -> {case.id}")
    return case.id


if __name__ == "__main__":
    main()
