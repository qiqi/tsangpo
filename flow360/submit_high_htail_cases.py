"""
Launch SOLVER cases on the six high-htail mesh-only projects created by
`submit_high_htail_meshes.py`.

The mesh-only run already produced a volume mesh per project; this script
calls `project.run_case(...)` so Flow360 picks up the existing mesh and
runs the unsteady solver at the v2 BO baseline for each phase, with
`htail_z_m=P.Z_TAIL_HIGH_M` so the htail rotation cylinder sits where
the htail geometry actually is.

Usage:
    python3 flow360/submit_high_htail_cases.py            # all 6
    python3 flow360/submit_high_htail_cases.py continuous # one config
    python3 flow360/submit_high_htail_cases.py gapped landing
"""
from __future__ import annotations
import sys
from math import radians
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
import cfd_setup as C
import params as P
import flow360 as fl

# Project IDs from submit_high_htail_meshes.py (Summary printed at submit time).
PROJECTS = {
    ("continuous", "cruise"):  "prj-21d5e737-a1d9-4dba-ad7c-2155e647ede1",
    ("continuous", "takeoff"): "prj-3505c35f-0a58-4aa2-9b0e-75af2f2143f6",
    ("continuous", "landing"): "prj-121d08b0-626c-475b-8c53-9d2ab54765d7",
    ("gapped",     "cruise"):  "prj-d5d18139-3f53-4bf5-b224-7d3a5f3feb2a",
    ("gapped",     "takeoff"): "prj-fadaacba-ac09-4b2f-a0f2-364c46f5cb02",
    ("gapped",     "landing"): "prj-1315636c-f6d9-4076-ba32-0ec82c272430",
}

PHASES = {
    "cruise":  dict(alpha=+7.0, theta_ht=0.0,  T=+1.0,
                    V=P.V_CRUISE_M_S,  alt=P.ALT_CRUISE_M),
    "takeoff": dict(alpha=+8.0, theta_ht=-5.0, T=+16.0,
                    V=P.V_TAKEOFF_M_S, alt=0.0),
    "landing": dict(alpha=+8.0, theta_ht=-6.0, T=+12.0,
                    V=P.V_LANDING_M_S, alt=0.0),
}

GET_SURFS = {
    "continuous": C.get_geometry_surfaces,
    "gapped":     C.get_gapped_geometry_surfaces,
}


def submit(cfg: str, phase: str):
    pid = PROJECTS[(cfg, phase)]
    spec = PHASES[phase]
    project = fl.Project.from_cloud(project_id=pid)
    surfaces = GET_SURFS[cfg](project)
    name = f"v2_{phase}_{cfg}_high_htail_parent"
    print(f"  {cfg:11s} {phase:8s}  project {pid[:18]}  → {name}")
    case = project.run_case(
        params=C.build_params(
            surfaces,
            theta_ac_rad     = +radians(spec["alpha"]),
            theta_ht_rad     = +radians(spec["theta_ht"]),
            thrust_mult      = spec["T"],
            velocity_m_s     = spec["V"],
            altitude_m       = spec["alt"],
            n_steps_total    = 20,
            max_pseudo_steps = 1000,
            htail_z_m        = P.Z_TAIL_HIGH_M,
        ),
        name=name,
        run_async=True,
        tags=["SI", "v2", phase, cfg, "high_htail", "parent"],
        use_beta_mesher=True,
    )
    print(f"    case = {case.id}")


if __name__ == "__main__":
    args = [a.lower() for a in sys.argv[1:]]
    configs = [c for c in ("continuous", "gapped") if not args or c in args]
    phases  = [p for p in ("cruise", "takeoff", "landing") if not args or p in args]
    print(f"Submitting cases for configs={configs}, phases={phases}")
    for cfg in configs:
        for ph in phases:
            submit(cfg, ph)
