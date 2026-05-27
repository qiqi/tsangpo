"""
Submit the v3 phugoid UNSTEADY warmup case (steady solve on the 4-cyl
multi-zone mesh).  All four nested rotation cylinders are frozen at the
trim L-pose via `AngleExpression` — no UDD active yet.  Purpose: get a
converged steady flow on the multi-zone mesh that the unsteady-with-UDD
case will fork from.

This script is the *meshing trigger*.  After submitting, wait for the
mesh status to confirm the topology is valid (no degenerate interfaces,
all four sliding interfaces map correctly).  If mesh works, we then fork
the UDD-driven unsteady case from this.

Usage:
    python3 flow360/submit_unsteady_warmup.py
"""
from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

import cfd_setup as C         # `import flow360 as fl` happens cleanly inside cfd_setup
import flow360 as fl

# `flow360/unsteady_setup.py` collides with the cloud `flow360` package on
# sys.path.  Load it directly by file path via importlib.
import importlib.util
_spec = importlib.util.spec_from_file_location(
    "unsteady_setup", REPO / "flow360" / "unsteady_setup.py")
U = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(U)

CSM      = REPO / "geometry" / "tsangpo_v3.csm"
AIRFOILS = REPO / "geometry" / "airfoils"

# v3 cruise gapped-flap config (matches submit_v3.py).
GAP_FRACTION       = 0.40
MID_OUTER_SEMISPAN = 0.39
FLAP_PANEL_FRAC    = 0.55
WING_SIDE_FRAC     = 0.61

PROJECT_NAME = "tsangpo_v3_unsteady_warmup_cruise_gap110"
CASE_NAME    = "v3_unsteady_warmup_cruise_gap110"
PROJECT_KEY  = "cruise_warmup_gap110"
# "gap110" = R_GAP_2_3_C = R_GAP_1_2_C = 110 c_w, sized so each cylinder's
# surface mesh matches the local volume mesh at its sliding interface.
# Replaces the earlier "widegap" (10c / 100c) which still had a 10× volume
# vs surface mesh mismatch at the cyl_2 outer boundary.

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
    # ---- 1. Inline UDCs and set the gapped-flap despmtrs into a temp .csm ----
    inlined = C.inline_udcs(CSM, AIRFOILS)
    inlined = C.set_csm_despmtrs(
        inlined,
        phase              = 0,                      # 0 = cruise
        gap_fraction       = GAP_FRACTION,
        mid_outer_semispan = MID_OUTER_SEMISPAN,
        flap_panel_frac    = FLAP_PANEL_FRAC,
        wing_side_frac     = WING_SIDE_FRAC,
    )
    with tempfile.NamedTemporaryFile("w", suffix=".csm", delete=False) as f:
        f.write(inlined)
        tmp_csm = f.name
    print(f"=== v3 phugoid UNSTEADY WARMUP — cruise phase ===")
    print(f"  Inlining {CSM.name} ({len(inlined.splitlines())} lines) → {tmp_csm}")

    # ---- 2. Upload to Flow360 as a new project ------------------------------
    project = fl.Project.from_geometry(
        tmp_csm,
        name=PROJECT_NAME,
        length_unit="m",
        tags=["tsangpo", "v3", "gapped40", "short_boom", "cruise",
               "phugoid", "unsteady", "warmup", "4cyl"],
    )
    print(f"  → project {project.id}   ({PROJECT_NAME})")
    _save_project_id(PROJECT_KEY, project.id)

    # Move into the Tsangpo/6_v3_short_boom folder.
    try:
        C.move_project_to_folder(project, "6_v3_short_boom")
        print(f"  moved project to folder Tsangpo/6_v3_short_boom")
    except Exception as e:
        print(f"  WARNING: failed to move project to v3 folder: {e!r}")

    surfaces = C.get_gapped_geometry_surfaces(project)
    print(f"  Surfaces: main={surfaces.wing_main_surfs[0].name}, "
          f"vane={[s.name for s in surfaces.wing_vane_surfs]}, "
          f"flap={[s.name for s in surfaces.wing_flap_surfs]}, "
          f"htail={surfaces.htail_surf.name}")

    # ---- 3. Build the warmup SimulationParams -------------------------------
    # mode='warmup' freezes all four cyl rotations at the trim L-pose via
    # AngleExpression (no UDD active).  Steady solve at trim α, V_∞=100 kt.
    # This case's converged flow is what the unsteady-with-UDD case will fork.
    params = U.build_unsteady_params(
        surfaces,
        phase="cruise",
        mode="warmup",
        # short steady-style run for the warmup; the actual unsteady case
        # picks its own timestep budget at fork time.
        n_physical_steps = 20,
        timestep_size_s  = 1.0,    # quasi-steady → large dt
        max_pseudo_steps = 1000,
    )

    # ---- 4. Submit the case (async — triggers mesh + solver) ----------------
    case = project.run_case(
        params=params,
        name=CASE_NAME,
        run_async=True,
        tags=["SI", "parent", "v3", "phugoid", "unsteady", "warmup"],
        use_beta_mesher=True,
    )
    print(f"\n  Warmup case submitted: {case.id}")
    print(f"\n  ===> Watch the mesh status in the admin panel.")
    print(f"  Once meshing succeeds, the case will start the steady solver.")
    print(f"  If mesh fails, the case will go to ERROR and we triage from there.")


if __name__ == "__main__":
    main()
