"""
Re-submit one parent (BO) case for each of the 12 (config, phase)
projects with the current `cfd_setup.PROP_REFINE_M` (0.025 c_w ≈ 0.0346 m,
the finer AD refinement that brings disk delivery to ~2% of commanded vs
~8% at 0.05 c_w).  Each submission is a fresh case — no fork_from — so
Flow360 rebuilds the volume mesh with the new refinement.
NOTE: the once-suspected ~60% AD under-delivery was a measurement-script
unit error, not a solver bug — see FLOW360_AD_DELIVERY_RESOLVED.md.

Run:
    python3 flow360/submit_fine_ad_parents.py
"""
from __future__ import annotations
import importlib.util, sys
from math import radians
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "post"))
import params as P
import cfd_setup as C
import flow360 as fl


# (family, phase, surface accessor, htail z)
JOBS = [
    ("v2_continuous",       "cruise",  C.get_geometry_surfaces,        P.Z_TAIL_LOW_M),
    ("v2_continuous",       "takeoff", C.get_geometry_surfaces,        P.Z_TAIL_LOW_M),
    ("v2_continuous",       "landing", C.get_geometry_surfaces,        P.Z_TAIL_LOW_M),
    ("v2_continuous_high",  "cruise",  C.get_geometry_surfaces,        P.Z_TAIL_HIGH_M),
    ("v2_continuous_high",  "takeoff", C.get_geometry_surfaces,        P.Z_TAIL_HIGH_M),
    ("v2_continuous_high",  "landing", C.get_geometry_surfaces,        P.Z_TAIL_HIGH_M),
    ("v2_gapped",           "cruise",  C.get_gapped_geometry_surfaces, P.Z_TAIL_LOW_M),
    ("v2_gapped",           "takeoff", C.get_gapped_geometry_surfaces, P.Z_TAIL_LOW_M),
    ("v2_gapped",           "landing", C.get_gapped_geometry_surfaces, P.Z_TAIL_LOW_M),
    ("v2_gapped_high",      "cruise",  C.get_gapped_geometry_surfaces, P.Z_TAIL_HIGH_M),
    ("v2_gapped_high",      "takeoff", C.get_gapped_geometry_surfaces, P.Z_TAIL_HIGH_M),
    ("v2_gapped_high",      "landing", C.get_gapped_geometry_surfaces, P.Z_TAIL_HIGH_M),
]


def load_spec(fam: str, phase: str):
    pp = REPO / "post" / fam / f"plot_{phase}_sensitivities.py"
    s  = importlib.util.spec_from_file_location("s", pp)
    m  = importlib.util.module_from_spec(s); s.loader.exec_module(m)
    return m.SPEC


def submit_one(fam: str, phase: str, get_surf, htail_z: float):
    spec = load_spec(fam, phase)
    proj = fl.Project.from_cloud(project_id=spec.project_id)
    surfaces = get_surf(proj)
    altitude = P.ALT_CRUISE_M if phase == "cruise" else 0.0
    params = C.build_params(
        surfaces,
        theta_ac_rad     = radians(spec.alpha_b),
        theta_ht_rad     = radians(spec.theta_ht_b),
        thrust_mult      = spec.T_b,
        velocity_m_s     = spec.velocity,
        altitude_m       = altitude,
        n_steps_total    = 26,
        max_pseudo_steps = 500,
        htail_z_m        = htail_z,
    )
    name = f"{phase}_fine_ad_parent"
    case = proj.run_case(
        params=params, name=name, run_async=True,
        use_beta_mesher=True,
        tags=["fine_ad_mesh", fam, phase, "parent_probe"],
    )
    print(f"  {fam:22s} {phase:8s}  {case.id}")


if __name__ == "__main__":
    print(f"PROP_REFINE_M = {C.PROP_REFINE_M:.5f} m (= "
          f"{C.PROP_REFINE_M / P.WING_MAC_M:.3f} c_w)")
    for fam, phase, get_surf, htail_z in JOBS:
        try:
            submit_one(fam, phase, get_surf, htail_z)
        except Exception as e:
            print(f"  {fam}/{phase}: FAILED — {type(e).__name__}: {e}")
