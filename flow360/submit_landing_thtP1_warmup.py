"""LANDING warmup with θ_ht = +1° LE-up retrim — between baseline and +3°.

Retrim sweep so far:
  θ_ht=-3.67°  CMy_warmup=+1.241  (baseline)  chunk1: nose-up to +98° apex
  θ_ht=-2.00°  CMy_warmup=+1.011             chunk1: nose-up to +36.9°
  θ_ht= 0.00°  CMy_warmup=+0.759             chunk1: not run
  θ_ht=+3.00°  CMy_warmup=+0.345             chunk1: nose-DOWN to -22.7° apex,
                                              **returning by t=5s** (apparent
                                              passive oscillation)

The divergence direction flipped between -2° and +3°.  +1° is the
midpoint of that range — if the apex amplitude is small, we may have
found a passively stable trim.  Predicted CMy_warmup ≈ +0.611
(slope -0.134/deg).

Usage:
    python3 flow360/submit_landing_thtP1_warmup.py
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
U = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(U)

CSM      = REPO / "geometry" / "tsangpo_v3.csm"
AIRFOILS = REPO / "geometry" / "airfoils"

GAP_FRACTION       = 0.40
MID_OUTER_SEMISPAN = 0.39
FLAP_PANEL_FRAC    = 0.55
WING_SIDE_FRAC     = 0.61

PROJECT_NAME = "tsangpo_v3_unsteady_warmup_landing_thtP1"
CASE_NAME    = "v3_unsteady_warmup_landing_thtP1"
PROJECT_KEY  = "landing_thtP1_warmup"

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
    print(f"  wrote project_id[{key}] = {project_id}")


def main() -> None:
    inlined = C.inline_udcs(CSM, AIRFOILS)
    inlined = C.set_csm_despmtrs(
        inlined, phase=2,
        gap_fraction=GAP_FRACTION,
        mid_outer_semispan=MID_OUTER_SEMISPAN,
        flap_panel_frac=FLAP_PANEL_FRAC,
        wing_side_frac=WING_SIDE_FRAC,
    )
    with tempfile.NamedTemporaryFile("w", suffix=".csm", delete=False) as f:
        f.write(inlined)
        tmp_csm = f.name
    print(f"=== LANDING warmup retrim (θ_ht = +1° LE-up) ===")

    project = fl.Project.from_geometry(
        tmp_csm, name=PROJECT_NAME, length_unit="m",
        tags=["tsangpo", "v3", "gapped40", "short_boom", "landing",
              "phugoid", "unsteady", "warmup", "4cyl", "thtP1_retrim"],
    )
    print(f"  → project {project.id}")
    _save_project_id(PROJECT_KEY, project.id)
    try:
        C.move_project_to_folder(project, "6_v3_short_boom")
    except Exception as e:
        print(f"  WARNING: failed to move project: {e!r}")

    surfaces = C.get_gapped_geometry_surfaces(project)
    params = U.build_unsteady_params(
        surfaces, phase="landing_thtP1", mode="warmup",
        n_physical_steps=30, timestep_size_s=1.0, max_pseudo_steps=1500,
    )
    case = project.run_case(
        params=params, name=CASE_NAME, run_async=True,
        tags=["SI", "parent", "v3", "phugoid", "unsteady", "warmup",
              "landing", "thtP1_retrim"],
        use_beta_mesher=True,
    )
    print(f"\n  Warmup submitted: {case.id}")
    _save_project_id(f"{PROJECT_KEY}_case", case.id)
    print(f"  θ_ht = +1° LE-up; predicted CMy_warmup ≈ +0.611")


if __name__ == "__main__":
    main()
