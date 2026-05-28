"""Landing-phugoid chunk1 — no-kick, fully instrumented.

Fork from landing warmup (PROJECT_KEY="landing_warmup_gap110"); kick the
phugoid into pseudoStep==0 integration with γ_0=0 (no initial perturbation
beyond the steady-state residual of the warmup itself), let it run for
5 s with full diagnostics + per-step y=0 slices for the flow animation.

Landing-specific:
  • PHASE_TRIM["landing"]: V=12.86 m/s SL, α_BO=+27.63°, θ_ht=-3.67°,
    T_mult=14.17 (T/W≈0.67, blown-lift), γ_trim=-30° (steep descent).
  • Faster phugoid period: T_phugoid ≈ π√2·V/g ≈ 5.8 s (vs cruise 17.4 s).
  • dt = 0.025 s (half of cruise's 0.05 s) to resolve the faster mode
    + the short-period transient (T_SP ~ 1-2 s) at high α.
  • Symplectic (Euler-Cromer) integrator: velocity-first slot order with
    NEW velocities feeding positions → no anti-damping, ω·dt ≡ Δθ exactly.

Usage:
    python3 flow360/submit_landing_chunk1_nokick_diag.py
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

PROJECT_KEY      = "landing_warmup_gap110"
PARENT_CASE_KEY  = "landing_warmup_gap110_case"
N_STEPS          = 200
DT_S             = 0.025
MAX_PSEUDO_STEPS = 200


def main() -> str:
    ids_path = REPO / "post" / "v3" / "unsteady_project_ids.json"
    d = json.loads(ids_path.read_text())
    project = fl.Project.from_cloud(project_id=d[PROJECT_KEY])
    parent  = fl.Case.from_cloud(case_id=d[PARENT_CASE_KEY])
    print(f"  parent (landing warmup): {parent.id}  status={parent.status}")

    surfaces = C.get_gapped_geometry_surfaces(project)
    params = U.build_unsteady_params(
        surfaces, phase="landing", mode="phugoid",
        n_physical_steps=N_STEPS,
        timestep_size_s=DT_S,
        max_pseudo_steps=MAX_PSEUDO_STEPS,
        gamma_kick_deg=0.0,
        use_analytic_omegaDot=False,
        diagnostics=True,
        slice_frequency=1,        # per-step y=0 slices for flow animation
    )

    name = "v3_phugoid_landing_gap110_chunk1_nokick_diag_5s"
    case = project.run_case(
        params=params, name=name, run_async=True, fork_from=parent,
        tags=["SI", "fork", "v3", "phugoid", "gap110", "chunk1",
              "nokick", "diag", "landing", "symplectic", "5s"],
        use_beta_mesher=True,
    )
    print(f"  submitted: {case.id}")
    d["phugoid_landing_gap110_nokick_diag_chunk1"] = case.id
    ids_path.write_text(json.dumps(d, indent=2))
    print(f"  saved phugoid_landing_gap110_nokick_diag_chunk1 -> {case.id}")
    return case.id


if __name__ == "__main__":
    main()
