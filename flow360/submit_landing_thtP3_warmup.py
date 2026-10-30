"""LANDING warmup at aggressive retrim θ_ht = +3.0° LE-up.

Extrapolating the slope from baseline (θ_ht=-3.67°, CMy=+1.24) →
retrim (θ_ht=-2.00°, CMy=+1.01) gives ΔCMy/Δθ_ht ≈ -0.14/deg.
At θ_ht=+3°, predicted CMy_warmup ≈ +0.3 — much closer to a true
4-cyl trim point.  If the no-kick phugoid from this warmup stays
near trim (or diverges much more slowly), the trim CAN be passively
stabilized by trimming θ_ht alone.  If it still diverges, the
instability is intrinsic to the high-α blown-lift configuration.

Usage:
    python3 flow360/submit_landing_thtP3_warmup.py
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

PROJECT_NAME = "tsangpo_v3_unsteady_warmup_landing_thtP3"
CASE_NAME    = "v3_unsteady_warmup_landing_thtP3"
PROJECT_KEY  = "landing_thtP3_warmup"

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
        inlined,
        phase              = 2,
        gap_fraction       = GAP_FRACTION,
        mid_outer_semispan = MID_OUTER_SEMISPAN,
        flap_panel_frac    = FLAP_PANEL_FRAC,
        wing_side_frac     = WING_SIDE_FRAC,
    )
    with tempfile.NamedTemporaryFile("w", suffix=".csm", delete=False) as f:
        f.write(inlined)
        tmp_csm = f.name
    print("=== LANDING warmup retrim (θ_ht = +3.0° LE-UP) ===")
    print(f"  CSM → {tmp_csm}")

    project = fl.Project.from_geometry(
        tmp_csm,
        name=PROJECT_NAME,
        length_unit="m",
        tags=["tsangpo", "v3", "gapped40", "short_boom", "landing",
              "phugoid", "unsteady", "warmup", "4cyl", "thtP3_retrim"],
    )
    print(f"  → project {project.id}")
    _save_project_id(PROJECT_KEY, project.id)
    try:
        C.move_project_to_folder(project, "6_v3_short_boom")
    except Exception as e:
        print(f"  WARNING: failed to move project: {e!r}")

    surfaces = C.get_gapped_geometry_surfaces(project)
    params = U.build_unsteady_params(
        surfaces,
        phase="landing_thtP3",
        mode="warmup",
        n_physical_steps = 30,
        timestep_size_s  = 1.0,
        max_pseudo_steps = 1500,
    )
    case = project.run_case(
        params=params,
        name=CASE_NAME,
        run_async=True,
        tags=["SI", "parent", "v3", "phugoid", "unsteady", "warmup",
              "landing", "thtP3_retrim"],
        use_beta_mesher=True,
    )
    print(f"\n  Warmup submitted: {case.id}")
    _save_project_id(f"{PROJECT_KEY}_case", case.id)
    print(f"  θ_ht = +3.0° LE-up (vs -3.67° baseline, -2.0° prev retrim)")
    print(f"  Predicted CMy_warmup ≈ +0.3 if slope holds.")


if __name__ == "__main__":
    main()
