"""Heavily instrumented no-kick phugoid chunk1.

Same setup as submit_gap110_nokick.py (γ_kick=0, mode='phugoid',
forked from cruise_warmup_gap110) but with `diagnostics=True` so each
of the four UDDs publishes:

    diag_state0_x       — state[0] (x_cg in m, since L_ref=1m)
    diag_state1_z       — state[1] (z_cg)
    diag_state2_Vx      — state[2] (Vx in nd-time velocity)
    diag_state3_Vz      — state[3] (Vz)
    diag_state4_theta   — state[4] (theta_body in rad)
    diag_state5_q       — state[5] (body pitch rate in rad/nd-time)
    diag_state6_link2   — state[6]
    diag_state7_link1   — state[7]
    diag_M_about_CG     — state[8], repurposed: M_about_CG_nd (the residual
                            the UDD actually integrates)
    diag_Q_DOT          — state[9], repurposed: M_about_CG_nd / I_yy_nd
                            (= the angular acceleration applied to state[5])

Comparing these across the four UDDs (shoulder/elbow/airframe/htail) tells
us whether the lockstep assumption holds.  Comparing state[8] to the
externally reconstructed CSV-based moment tells us if the UDD is integrating
what we think it's integrating.

Usage:
    python3 flow360/submit_gap110_nokick_diag.py
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

PROJECT_KEY      = "cruise_warmup_gap110"
PARENT_CASE_KEY  = "cruise_warmup_gap110_case"
N_STEPS          = 200
DT_S             = 0.05
MAX_PSEUDO_STEPS = 200


def main() -> str:
    ids_path = REPO / "post" / "v3" / "unsteady_project_ids.json"
    d = json.loads(ids_path.read_text())
    project = fl.Project.from_cloud(project_id=d[PROJECT_KEY])
    parent  = fl.Case.from_cloud(case_id=d[PARENT_CASE_KEY])
    print(f"  parent: {parent.id}  status={parent.status}")

    surfaces = C.get_gapped_geometry_surfaces(project)
    params = U.build_unsteady_params(
        surfaces, phase="cruise", mode="phugoid",
        n_physical_steps=N_STEPS,
        timestep_size_s=DT_S,
        max_pseudo_steps=MAX_PSEUDO_STEPS,
        gamma_kick_deg=0.0,
        use_analytic_omegaDot=False,
        diagnostics=True,
    )

    name = "v3_phugoid_gap110_chunk1_nokick_diag_10s"
    case = project.run_case(
        params=params, name=name, run_async=True, fork_from=parent,
        tags=["SI", "fork", "v3", "phugoid", "gap110", "chunk1",
              "nokick", "diag", "instrumented", "10s"],
        use_beta_mesher=True,
    )
    print(f"  submitted: {case.id}")
    d["phugoid_gap110_nokick_diag_chunk1"] = case.id
    ids_path.write_text(json.dumps(d, indent=2))
    print(f"  saved phugoid_gap110_nokick_diag_chunk1 -> {case.id}")
    return case.id


if __name__ == "__main__":
    main()
