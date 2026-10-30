"""Minimal submission script for a UDD 1-DOF (pitch-only) cell.

Submits:
  1. a warmup case at the target trim pose (all cyls frozen)
  2. a chunk case forked from the warmup, with cyl_3 free under
     quasi-static damped pitch dynamics

Records IDs in /tmp/udd_starter_ids.json so the post-processing
example scripts can find them.

Adapt the CELL settings below, the geometry, and the trim parameters
to your problem.  The flow360 boilerplate (Project.from_geometry,
SimulationParams construction) follows the parent repo's pattern.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

# Make the parent repo importable (cfd_setup, unsteady_setup, params)
REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

import flow360 as fl
import cfd_setup as C
import params as P

import importlib.util
_spec = importlib.util.spec_from_file_location(
    "unsteady_setup", REPO / "flow360" / "unsteady_setup.py")
U = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(U)

# ---- Cell to run ------------------------------------------------------------
THETA_HT_DEG = 0.0            # horizontal-tail incidence
V_INF_M_S    = 12.86          # freestream speed
PHASE        = "landing"      # = trim point from params.PHASE_TRIM

# ---- Numerical recipe -------------------------------------------------------
WARMUP_N_STEPS    = 3
WARMUP_DT_S       = 1.0
WARMUP_MAX_PSEUDO = 1500

CHUNK_N_STEPS     = 8
CHUNK_DT_S        = 2.0
CHUNK_MAX_PSEUDO  = 1500
PITCH_DAMPING_SI  = 30000.0   # N·m·s/rad   (see README "Pitfalls" #10)
QUASI_STATIC_Q    = True      # q_new = Q_TORQUE / D   (no q memory)

STEP_OFFSET       = WARMUP_N_STEPS   # tells the chunk's UDD where to start

# ---- Geometry ---------------------------------------------------------------
CSM      = REPO / "geometry" / "tsangpo_v3.csm"
AIRFOILS = REPO / "geometry" / "airfoils"
# Phase-2 flap deployment for landing config
DESPMTRS = dict(phase=2, gap_fraction=0.40, mid_outer_semispan=0.39,
                flap_panel_frac=0.55, wing_side_frac=0.61)


def submit_warmup() -> tuple[str, str]:
    """Create a fresh project for this cell and submit the warmup case."""
    inlined = C.inline_udcs(CSM, AIRFOILS)
    inlined = C.set_csm_despmtrs(inlined, **DESPMTRS)
    import tempfile
    with tempfile.NamedTemporaryFile("w", suffix=".csm", delete=False) as f:
        f.write(inlined); tmp_csm = f.name

    project = fl.Project.from_geometry(
        tmp_csm,
        name=f"udd_starter_th{THETA_HT_DEG:+.2f}_V{V_INF_M_S:.2f}",
        length_unit="m",
        tags=["udd_starter", "warmup"],
    )
    print(f"  project: {project.id}")

    surfaces = C.get_gapped_geometry_surfaces(project)
    params = U.build_unsteady_params(
        surfaces, phase=PHASE, mode="warmup",
        n_physical_steps=WARMUP_N_STEPS,
        timestep_size_s=WARMUP_DT_S,
        max_pseudo_steps=WARMUP_MAX_PSEUDO,
        theta_ht_override_deg=THETA_HT_DEG,
        V_inf_override_m_s=V_INF_M_S,
    )
    # Always: beta mesher + y=0 slice  (see README "Pitfalls" #1, #2)
    case = project.run_case(
        params=params, name="warmup",
        run_async=True, use_beta_mesher=True,
        tags=["udd_starter", "warmup"],
    )
    print(f"  warmup case: {case.id}")
    return project.id, case.id


def submit_chunk(project_id: str, warmup_case_id: str) -> str:
    """Fork a damped 1-DOF chunk from the warmup."""
    proj = fl.Project.from_cloud(project_id=project_id)
    parent = fl.Case.from_cloud(case_id=warmup_case_id)
    surfaces = C.get_gapped_geometry_surfaces(proj)

    params = U.build_unsteady_params(
        surfaces, phase=PHASE, mode="pitch_step_cyl3_integrated",
        n_physical_steps=CHUNK_N_STEPS,
        timestep_size_s=CHUNK_DT_S,
        max_pseudo_steps=CHUNK_MAX_PSEUDO,
        theta_ht_override_deg=THETA_HT_DEG,
        V_inf_override_m_s=V_INF_M_S,
        # cyl_1 (shoulder) and cyl_2 (elbow) frozen:
        pitch_step_link1_ramp_deg=0.0,
        pitch_step_link2_ramp_deg=0.0,
        pitch_step_step_offset=STEP_OFFSET,
        # damping recipe:
        pitch_damping_si=PITCH_DAMPING_SI,
        quasi_static_q=QUASI_STATIC_Q,
    )
    case = proj.run_case(
        params=params, name="chunk2_qs_D30k",
        run_async=True, fork_from=parent,
        use_beta_mesher=True,
        tags=["udd_starter", "chunk2", "quasi_static"],
    )
    print(f"  chunk case: {case.id}")
    return case.id


def wait_for(case_id: str, label: str) -> None:
    """Block until the case reaches a terminal state."""
    import time
    while True:
        c = fl.Case.from_cloud(case_id=case_id)
        s = c.status.value
        if s in ("completed", "completed_with_warnings"):
            print(f"  {label} COMPLETE  elapsed={c.info.elapsedTimeInSeconds}s")
            return
        if "error" in s or "failed" in s or "diverged" in s:
            print(f"  {label} FAILED  status={s}")
            sys.exit(1)
        time.sleep(60)


if __name__ == "__main__":
    print(f"Submitting cell θ_ht={THETA_HT_DEG:+.2f}°, V={V_INF_M_S} m/s")
    project_id, warmup_id = submit_warmup()
    wait_for(warmup_id, "warmup")

    chunk_id = submit_chunk(project_id, warmup_id)
    wait_for(chunk_id, "chunk")

    ids = {"project": project_id,
           "warmup": warmup_id,
           "chunk":  chunk_id,
           "theta_ht_deg": THETA_HT_DEG,
           "V_inf_m_s": V_INF_M_S}
    Path("/tmp/udd_starter_ids.json").write_text(json.dumps(ids, indent=2))
    print(f"\nIDs saved to /tmp/udd_starter_ids.json — feed to example_extract.py")
