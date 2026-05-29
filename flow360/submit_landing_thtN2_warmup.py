"""LANDING warmup with retrimmed htail (θ_ht = −2.0° LE-down).

Hypothesis: the original landing trim (θ_ht=−3.67° from the steady 2-cyl
campaign) doesn't quite zero out on the 4-cyl phugoid mesh — the warmup
left a residual CMy=+1.24 (≈+2,660 N·m nose-up about initial CG).  That
residual drove the no-kick divergence we observed.

Test: reduce LE-down by 1.67° → θ_ht=−2.0°.  Htail now sees more α, makes
more positive lift, and contributes more nose-DOWN moment about CG —
should cancel the residual.  If the resulting warmup converges with
CMy≈0, the open-loop no-kick phugoid from this trim should be either
truly stable (no divergence) or diverge much more slowly, revealing
the actual unstable-mode timescale rather than the trim-residual kick.

Usage:
    python3 flow360/submit_landing_thtN2_warmup.py
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

PROJECT_NAME = "tsangpo_v3_unsteady_warmup_landing_thtN2"
CASE_NAME    = "v3_unsteady_warmup_landing_thtN2"
PROJECT_KEY  = "landing_thtN2_warmup"

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
    inlined = C.inline_udcs(CSM, AIRFOILS)
    inlined = C.set_csm_despmtrs(
        inlined,
        phase              = 2,                    # landing flap
        gap_fraction       = GAP_FRACTION,
        mid_outer_semispan = MID_OUTER_SEMISPAN,
        flap_panel_frac    = FLAP_PANEL_FRAC,
        wing_side_frac     = WING_SIDE_FRAC,
    )
    with tempfile.NamedTemporaryFile("w", suffix=".csm", delete=False) as f:
        f.write(inlined)
        tmp_csm = f.name
    print("=== LANDING warmup retrim (θ_ht = −2.0° LE-down) ===")
    print(f"  Inlining {CSM.name} (phase=2) → {tmp_csm}")

    project = fl.Project.from_geometry(
        tmp_csm,
        name=PROJECT_NAME,
        length_unit="m",
        tags=["tsangpo", "v3", "gapped40", "short_boom", "landing",
               "phugoid", "unsteady", "warmup", "4cyl", "thtN2_retrim"],
    )
    print(f"  → project {project.id}   ({PROJECT_NAME})")
    _save_project_id(PROJECT_KEY, project.id)

    try:
        C.move_project_to_folder(project, "6_v3_short_boom")
    except Exception as e:
        print(f"  WARNING: failed to move project: {e!r}")

    surfaces = C.get_gapped_geometry_surfaces(project)
    params = U.build_unsteady_params(
        surfaces,
        phase="landing_thtN2",        # ← retrim phase
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
              "landing", "thtN2_retrim"],
        use_beta_mesher=True,
    )
    print(f"\n  Warmup case submitted: {case.id}")
    _save_project_id(f"{PROJECT_KEY}_case", case.id)
    print(f"  Trim: V=12.86 m/s, alpha_BO=+27.63°, T_mult=14.17, "
          f"γ_trim=-30°, θ_ht=-2.0° (← retrim vs -3.67° baseline)")
    print(f"  Once converged, check CMy residual: should be < +1.24 if hypothesis holds.")


if __name__ == "__main__":
    main()
