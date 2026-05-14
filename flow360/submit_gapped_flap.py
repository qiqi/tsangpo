"""
Submit initial Flow360 calculations on the GAPPED-FLAP configuration
(`tsangpo_gapped.csm`: continuous main wing with the cove filled by the
tail-cap in the middle 39 % of span; 0.5 %-of-span flap gap on each
side of the no-cove middle region).

Three parents are submitted, one per flap phase:
  * cruise  (phase 0): α=+7°,  θ_ht=  0°, T_mult=+1.0,  V=45.72 m/s, level
  * takeoff (phase 1): α=+8°,  θ_ht=-5°,  T_mult=+16,   V=18.00 m/s, γ=+30°
  * landing (phase 2): α=+8°,  θ_ht=-6°,  T_mult=+12,   V=12.86 m/s, γ=-30°

These BO-estimate points match the continuous-flap v2 parents (live
projects `prj-16082511`, `prj-9cd3ad10`, `prj-e7dc7d6d`) so the gapped
geometry comparison is at the same conditions.

Each parent lands in its own Flow360 project under
`Tsangpo/3_v2_gapped_low_htail/`.  No sweep forks are submitted —
once mesh + per-surface forces look sane, sweeps can be added via the
existing `submit_alpha_sweep.py` / `_htail_sweep.py` / `_thrust_sweep.py`
by exporting `TSANGPO_PARENT_CASE_ID`.

    python3 flow360/submit_gapped_flap.py            # submit all three
    python3 flow360/submit_gapped_flap.py cruise     # one phase
    python3 flow360/submit_gapped_flap.py takeoff
    python3 flow360/submit_gapped_flap.py landing
"""
from __future__ import annotations

import os
import sys
import tempfile
from math import radians
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
import cfd_setup as C
import params as P
import flow360 as fl

CSM      = REPO / "geometry" / "tsangpo_gapped.csm"
AIRFOILS = REPO / "geometry" / "airfoils"

# Gapped-flap geometry constants (must remain mutually consistent — see
# tsangpo_gapped.csm header).
GAP_FRACTION         = 0.40
MID_OUTER_SEMISPAN   = 0.39      # 0.5%-of-span air gap (in flap) on each side
FLAP_PANEL_FRAC      = 0.55      # = flap_outboard_eta(0.95) − gap_fraction
WING_SIDE_FRAC       = 0.61      # = 1 − mid_outer_semispan  (per-side coved wing width)

# Per-phase BO baseline points (match the continuous-flap v2 parents).
PHASES = {
    "cruise": dict(
        phase            = 0,
        alpha_eff_deg    = +7.0,
        theta_htail_deg  = 0.0,
        thrust_mult      = +1.0,
        velocity_m_s     = P.V_CRUISE_M_S,
        altitude_m       = P.ALT_CRUISE_M,
        project_name     = "tsangpo_v2_cruise_gapped40",
        case_name        = "v2_cruise_gapped40_parent",
        tags_phase       = ["cruise", "level", "phase0"],
    ),
    "takeoff": dict(
        phase            = 1,
        alpha_eff_deg    = +8.0,
        theta_htail_deg  = -5.0,
        thrust_mult      = +16.0,
        velocity_m_s     = P.V_TAKEOFF_M_S,
        altitude_m       = 0.0,
        project_name     = "tsangpo_v2_takeoff_gapped40",
        case_name        = "v2_takeoff_gapped40_parent",
        tags_phase       = ["takeoff", "phase1", "gamma_p30"],
    ),
    "landing": dict(
        phase            = 2,
        alpha_eff_deg    = +8.0,
        theta_htail_deg  = -6.0,
        thrust_mult      = +12.0,
        velocity_m_s     = P.V_LANDING_M_S,
        altitude_m       = 0.0,
        project_name     = "tsangpo_v2_landing_gapped40",
        case_name        = "v2_landing_gapped40_parent",
        tags_phase       = ["landing", "phase2", "gamma_m30"],
    ),
}


def submit_one(phase_key: str) -> tuple[str, str]:
    spec = PHASES[phase_key]
    theta_ac = +radians(spec["alpha_eff_deg"])
    theta_ht = +radians(spec["theta_htail_deg"])

    print(f"\n=== {phase_key} (phase {spec['phase']}) ===")
    print(f"  α        = {spec['alpha_eff_deg']:+.2f}°   θ_ac = {theta_ac:+.4f} rad")
    print(f"  θ_htail  = {spec['theta_htail_deg']:+.2f}°    θ_ht = {theta_ht:+.4f} rad")
    print(f"  T_mult   = {spec['thrust_mult']:+.2f}")
    print(f"  V        = {spec['velocity_m_s']:.2f} m/s")

    # Reuse existing project via TSANGPO_PROJECT_ID_<PHASE>, else upload.
    env_key = f"TSANGPO_PROJECT_ID_{phase_key.upper()}"
    project_id = os.environ.get(env_key)
    if project_id:
        print(f"  Reusing project {project_id} (via ${env_key})")
        project = fl.Project.from_cloud(project_id)
    else:
        inlined = C.inline_udcs(CSM, AIRFOILS)
        inlined = C.set_csm_despmtrs(
            inlined,
            phase              = spec["phase"],
            gap_fraction       = GAP_FRACTION,
            mid_outer_semispan = MID_OUTER_SEMISPAN,
            flap_panel_frac    = FLAP_PANEL_FRAC,
            wing_side_frac     = WING_SIDE_FRAC,
        )
        with tempfile.NamedTemporaryFile("w", suffix=".csm", delete=False) as f:
            f.write(inlined)
            tmp_csm = f.name
        print(f"  Uploading inlined {CSM.name} ({len(inlined.splitlines())} lines) …")
        project = fl.Project.from_geometry(
            tmp_csm,
            name=spec["project_name"],
            length_unit="m",
            tags=["tsangpo", "v2", "gapped40", "low_htail", *spec["tags_phase"]],
        )
        print(f"  → project {project.id}")
        folder_id = C.move_project_to_folder(project, "3_v2_gapped_low_htail")
        print(f"  Moved into folder 3_v2_gapped_low_htail ({folder_id[:24]}…)")

    surfaces = C.get_gapped_geometry_surfaces(project)
    print(f"  Surfaces: main_wing={surfaces.wing_main_surfs[0].name}, "
          f"vane={[s.name for s in surfaces.wing_vane_surfs]}, "
          f"flap={[s.name for s in surfaces.wing_flap_surfs]}, "
          f"htail={surfaces.htail_surf.name}")

    case = project.run_case(
        params=C.build_params(
            surfaces,
            theta_ac_rad   = theta_ac,
            theta_ht_rad   = theta_ht,
            thrust_mult    = spec["thrust_mult"],
            velocity_m_s   = spec["velocity_m_s"],
            altitude_m     = spec["altitude_m"],
            n_steps_total  = 20,
            max_pseudo_steps = 1000,
        ),
        name=spec["case_name"],
        run_async=True,
        tags=["SI", "v2", "gapped40", "parent", *spec["tags_phase"]],
        use_beta_mesher=True,
    )
    print(f"  Case submitted: {case.id}")
    return project.id, case.id


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] in PHASES:
        keys = [sys.argv[1]]
    elif len(sys.argv) > 1:
        sys.exit(f"unknown phase {sys.argv[1]!r}; choices: {list(PHASES)}")
    else:
        keys = list(PHASES)

    results = []
    for k in keys:
        results.append((k, *submit_one(k)))

    print("\n=== Summary ===")
    for phase_key, pid, cid in results:
        print(f"  {phase_key:8s}  project {pid}   case {cid}")
