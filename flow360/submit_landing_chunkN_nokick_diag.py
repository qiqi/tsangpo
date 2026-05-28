"""Fork a follow-on no-kick LANDING phugoid chunk (chunks 2-6 of 30 s chain).

Continues the no-kick landing-phugoid trajectory for another 5 s.
diagnostics=True keeps the four omegaDot diagnostic channels so we can
extract the full state (θ_body, q, applied moment) across the chain and
measure the phugoid period/damping.

State continuity: Flow360 preserves UDD state across a fork, so the chunk
resumes the parent's final state (state_vars_initial_value is ignored on a
fork).  gamma_kick=0 → no re-perturbation.

Usage:
    python3 flow360/submit_landing_chunkN_nokick_diag.py <parent_case_id> <chunk_idx>
"""
from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

if len(sys.argv) != 3:
    print("usage: submit_landing_chunkN_nokick_diag.py <parent_case_id> <chunk_idx>",
          file=sys.stderr)
    sys.exit(2)

PARENT = sys.argv[1]
CHUNK  = int(sys.argv[2])

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

import cfd_setup as C
import flow360 as fl

_spec = importlib.util.spec_from_file_location(
    "unsteady_setup", REPO / "flow360" / "unsteady_setup.py")
U = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(U)

PROJECT_KEY      = "landing_warmup_gap110"
N_STEPS          = 200
DT_S             = 0.025
MAX_PSEUDO_STEPS = 200


def main() -> str:
    ids_path = REPO / "post" / "v3" / "unsteady_project_ids.json"
    d = json.loads(ids_path.read_text())
    project = fl.Project.from_cloud(project_id=d[PROJECT_KEY])
    parent  = fl.Case.from_cloud(case_id=PARENT)
    print(f"  parent: {parent.id}  status={parent.status}")

    surfaces = C.get_gapped_geometry_surfaces(project)
    params = U.build_unsteady_params(
        surfaces, phase="landing", mode="phugoid",
        n_physical_steps=N_STEPS,
        timestep_size_s=DT_S,
        max_pseudo_steps=MAX_PSEUDO_STEPS,
        gamma_kick_deg=0.0,
        use_analytic_omegaDot=False,
        diagnostics=True,
        slice_frequency=1,        # per-step y=0 slice → flow-field animation
    )

    name = f"v3_phugoid_landing_gap110_nokick_chunk{CHUNK}_5s"
    case = project.run_case(
        params=params, name=name, run_async=True, fork_from=parent,
        tags=["SI", "fork", "v3", "phugoid", "gap110", f"chunk{CHUNK}",
              "nokick", "landing", "symplectic", "diag", "5s"],
        use_beta_mesher=True,
    )
    print(f"  submitted: {case.id}")
    d[f"phugoid_landing_gap110_nokick_chunk{CHUNK}"] = case.id
    ids_path.write_text(json.dumps(d, indent=2))
    print(f"  saved phugoid_landing_gap110_nokick_chunk{CHUNK} -> {case.id}")
    return case.id


if __name__ == "__main__":
    main()
