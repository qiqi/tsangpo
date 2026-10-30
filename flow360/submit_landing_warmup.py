"""
Submit the LANDING-phase phugoid UNSTEADY warmup (steady solve on the
4-cyl multi-zone mesh at the landing trim L-pose).  All four cylinders
frozen via AngleExpression; no UDD active.

Differences from the cruise warmup:
  • phase=2 in CSM despmtrs (landing flap deployed)
  • PHASE_TRIM["landing"]: V=12.86 m/s SL, α_BO=+27.63°, θ_ht=-3.67°,
    T_mult=14.17 (blown-lift), γ_trim=-30° (steep descent)
  • Inherits cfd_setup's AD-rotation fix (enclosed_entities includes
    cyl_htail + prop_cyls) from the start — no later re-meshing needed.

Usage:
    python3 flow360/submit_landing_warmup.py
"""
from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

import cfd_setup as C
import flow360 as fl

_spec = importlib.util.spec_from_file_location(
    "unsteady_setup", REPO / "flow360" / "unsteady_setup.py")
U = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(U)

CSM      = REPO / "geometry" / "tsangpo_v3.csm"
AIRFOILS = REPO / "geometry" / "airfoils"

# Same v3 gapped-flap config as cruise warmup.
GAP_FRACTION       = 0.40
MID_OUTER_SEMISPAN = 0.39
FLAP_PANEL_FRAC    = 0.55
WING_SIDE_FRAC     = 0.61

PROJECT_NAME = "tsangpo_v3_unsteady_warmup_landing_gap110"
CASE_NAME    = "v3_unsteady_warmup_landing_gap110"
PROJECT_KEY  = "landing_warmup_gap110"

PROJECT_IDS_PATH = REPO / "post" / "v3" / "unsteady_project_ids.json"


def _save_project_id(key: str, project_id: str) -> None:
    PROJECT_IDS_PATH.parent.mkdir(parents=True, exist_ok=True)
    d = {}
    if PROJECT_IDS_PATH.exists():
        try:
            d = json.loads(PROJECT_IDS_PATH.read_text())
        except Exception:
            d = {}
    d[key] = project_id
    PROJECT_IDS_PATH.write_text(json.dumps(d, indent=2))
    print(f"  wrote project_id[{key}] = {project_id} → "
          f"{PROJECT_IDS_PATH.relative_to(REPO)}")


def main() -> None:
    # ---- 1. Inline UDCs and set the LANDING despmtrs into a temp .csm ----
    inlined = C.inline_udcs(CSM, AIRFOILS)
    inlined = C.set_csm_despmtrs(
        inlined,
        phase              = 2,                      # 2 = LANDING flap
        gap_fraction       = GAP_FRACTION,
        mid_outer_semispan = MID_OUTER_SEMISPAN,
        flap_panel_frac    = FLAP_PANEL_FRAC,
        wing_side_frac     = WING_SIDE_FRAC,
    )
    with tempfile.NamedTemporaryFile("w", suffix=".csm", delete=False) as f:
        f.write(inlined)
        tmp_csm = f.name
    print("=== v3 phugoid UNSTEADY WARMUP — LANDING phase ===")
    print(f"  Inlining {CSM.name} ({len(inlined.splitlines())} lines, phase=2) → {tmp_csm}")

    # ---- 2. Upload to Flow360 as a new project ------------------------------
    project = fl.Project.from_geometry(
        tmp_csm,
        name=PROJECT_NAME,
        length_unit="m",
        tags=["tsangpo", "v3", "gapped40", "short_boom", "landing",
               "phugoid", "unsteady", "warmup", "4cyl"],
    )
    print(f"  → project {project.id}   ({PROJECT_NAME})")
    _save_project_id(PROJECT_KEY, project.id)

    try:
        C.move_project_to_folder(project, "6_v3_short_boom")
        print("  moved project to folder Tsangpo/6_v3_short_boom")
    except Exception as e:
        print(f"  WARNING: failed to move project to v3 folder: {e!r}")

    surfaces = C.get_gapped_geometry_surfaces(project)
    print(f"  Surfaces: main={surfaces.wing_main_surfs[0].name}, "
          f"vane={[s.name for s in surfaces.wing_vane_surfs]}, "
          f"flap={[s.name for s in surfaces.wing_flap_surfs]}, "
          f"htail={surfaces.htail_surf.name}")

    # ---- 3. Build the warmup SimulationParams -------------------------------
    # mode='warmup', phase='landing'.  Picks up PHASE_TRIM["landing"]
    # automatically (V=12.86, α=+27.63°, θ_ht=-3.67°, T_mult=14.17).
    # Increased pseudo-step budget because the highly-loaded landing flow
    # converges more slowly than cruise (high α, blown-lift slipstream).
    params = U.build_unsteady_params(
        surfaces,
        phase="landing",
        mode="warmup",
        n_physical_steps = 30,
        timestep_size_s  = 1.0,    # quasi-steady → large dt
        max_pseudo_steps = 1500,
    )

    # ---- 4. Submit the case (async — triggers mesh + solver) ----------------
    case = project.run_case(
        params=params,
        name=CASE_NAME,
        run_async=True,
        tags=["SI", "parent", "v3", "phugoid", "unsteady", "warmup", "landing"],
        use_beta_mesher=True,
    )
    print(f"\n  Warmup case submitted: {case.id}")
    _save_project_id(f"{PROJECT_KEY}_case", case.id)
    print(f"\n  ===> Watch mesh + solver status in the admin panel.")
    print(f"  Trim: V=12.86 m/s, alpha_BO=+27.63°, T_mult=14.17 (T/W≈0.67), gamma=-30°")
    print(f"  Once converged, the chunk1 script forks the phugoid from this.")


if __name__ == "__main__":
    main()
