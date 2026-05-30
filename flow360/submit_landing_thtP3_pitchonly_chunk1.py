"""1-DOF pitch-only stability test forked from the +3° landing warmup.

What this does:
  • cyl_1 (shoulder) and cyl_2 (elbow) prescribed via sine ramps with
    AMPLITUDE = 0 → they stay frozen at the L-pose.  The CG is locked at
    world (+L, 0, −L) for the whole run.
  • cyl_4 (htail) frozen at the prescribed θ_ht_rel.
  • cyl_3 (airframe pitch axis) driven by GENUINE moment-about-CG
    integration — q̇ = (momentY + lever-arm correction − prop_z·T_disk)
    / I_yy.  Same formula as the full phugoid mode.

Since the CG is locked in world, the body's α relative to the flat
freestream ≡ θ_body.  Sweeping θ_body via free rotation is equivalent
to scanning α through the natural moment curve M(θ_body, θ_ht).

  • At an EQUILIBRIUM θ_eq: M = 0.
  • Statically STABLE if ∂M/∂θ < 0 at θ_eq (body returns after kick).
  • Statically UNSTABLE if ∂M/∂θ > 0 (body diverges from θ_eq).

Initial condition: body at θ_body=−2.37° (warmup pose).  The pitch
dynamics will let the body rotate to find wherever M=0.  If the
design trim α=+27.6° is a stable equilibrium, the body will reach
θ_body=+27.6° and stay.  If not, it drifts to some other θ_eq or
oscillates / diverges.

Usage:
    python3 flow360/submit_landing_thtP3_pitchonly_chunk1.py
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

PROJECT_KEY      = "landing_thtP3_warmup"
PARENT_CASE_KEY  = "landing_thtP3_warmup_case"
N_STEPS          = 200
DT_S             = 0.025
MAX_PSEUDO_STEPS = 200
STEP_OFFSET      = 30   # landing warmup has 30 physical steps


def main() -> str:
    ids_path = REPO / "post" / "v3" / "unsteady_project_ids.json"
    d = json.loads(ids_path.read_text())
    project = fl.Project.from_cloud(project_id=d[PROJECT_KEY])
    parent  = fl.Case.from_cloud(case_id=d[PARENT_CASE_KEY])
    print(f"  parent (+3° warmup): {parent.id}  status={parent.status}")

    surfaces = C.get_gapped_geometry_surfaces(project)
    params = U.build_unsteady_params(
        surfaces,
        phase="landing_thtP3",
        mode="pitch_step_cyl3_integrated",
        n_physical_steps=N_STEPS,
        timestep_size_s=DT_S,
        max_pseudo_steps=MAX_PSEUDO_STEPS,
        pitch_step_link1_ramp_deg=0.0,   # cyl_1 frozen
        pitch_step_link2_ramp_deg=0.0,   # cyl_2 frozen
        pitch_step_step_offset=STEP_OFFSET,
        use_analytic_omegaDot=False,
        slice_frequency=1,               # multi-y slices
    )

    name = "v3_landing_thtP3_pitchonly_chunk1_5s"
    case = project.run_case(
        params=params, name=name, run_async=True, fork_from=parent,
        tags=["SI", "fork", "v3", "phugoid", "gap110", "pitch_only",
              "1dof", "diag", "landing", "thtP3", "5s"],
        use_beta_mesher=True,
    )
    print(f"  submitted: {case.id}")
    d["pitch_only_landing_thtP3_chunk1"] = case.id
    ids_path.write_text(json.dumps(d, indent=2))
    print(f"  saved pitch_only_landing_thtP3_chunk1 -> {case.id}")
    return case.id


if __name__ == "__main__":
    main()
