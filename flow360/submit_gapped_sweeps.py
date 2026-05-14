"""
Fork α / θ_htail / T_mult sweeps off the three gapped-flap parents
launched by submit_gapped_flap.py, mirroring the v2 continuous-flap
sweep recipes (post/v2_continuous TRIM_RESULTS.md).

Each phase gets 30 forks (10 per dimension).  Same sweep ranges and
convergence settings as the continuous-flap v2 campaigns so the two
geometries are directly comparable.

The gapped parents are:
  cruise  : case-3a4859d9-5298-4a35-8d68-b98bcb26b580  (prj-59c27343)
  takeoff : case-fe40382d-13d5-4343-8626-453fd3872993  (prj-e0e11ed5)
  landing : case-3fe6a528-9151-4337-8ece-8f0862fee1bf  (prj-0cd29981)

Usage:
    python3 flow360/submit_gapped_sweeps.py                # all three
    python3 flow360/submit_gapped_sweeps.py cruise         # one phase
"""
from __future__ import annotations

import os
import sys
from math import radians
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
import cfd_setup as C
import params as P
import flow360 as fl

N_PARENT_STEPS = 20
N_FORK_NEW     = 6
N_FORK_TOTAL   = N_PARENT_STEPS + N_FORK_NEW
PSEUDO_FORK    = 500

# BO baselines + sweep ranges per phase (copied from the v2 continuous campaigns).
SPECS = {
    "cruise": dict(
        parent_case_id = "case-3a4859d9-5298-4a35-8d68-b98bcb26b580",
        alpha_b = +7.0, theta_ht_b = 0.0,  T_b = +1.0,
        velocity_m_s = P.V_CRUISE_M_S, altitude_m = P.ALT_CRUISE_M,
        alpha_sweep  = (-3.0, -1.0, +1.0, +3.0, +5.0, +7.0, +9.0, +11.0, +13.0, +15.0),
        htail_sweep  = (-12.0, -9.0, -6.0, -3.0, +3.0, +6.0, +9.0, +12.0, +15.0),
        thrust_sweep = (0.0, 0.25, 0.50, 0.75, 1.25, 1.50, 2.00, 2.50, 3.00),
        name_prefix  = "gap40_cruise",
        tags         = ["v2", "gapped40", "cruise", "phase0"],
    ),
    "takeoff": dict(
        parent_case_id = "case-fe40382d-13d5-4343-8626-453fd3872993",
        alpha_b = +8.0, theta_ht_b = -5.0, T_b = +16.0,
        velocity_m_s = P.V_TAKEOFF_M_S, altitude_m = 0.0,
        alpha_sweep  = (-2.0, +2.0, +5.0, +11.0, +14.0, +17.0, +20.0, +25.0, +30.0),
        htail_sweep  = (-15.0, -10.0, 0.0, +5.0, +10.0, +15.0, +20.0, +25.0, +30.0),
        thrust_sweep = (6.0, 9.0, 12.0, 14.0, 18.0, 20.0, 22.0, 25.0, 30.0),
        name_prefix  = "gap40_TO",
        tags         = ["v2", "gapped40", "takeoff", "phase1"],
    ),
    "landing": dict(
        parent_case_id = "case-3fe6a528-9151-4337-8ece-8f0862fee1bf",
        alpha_b = +8.0, theta_ht_b = -6.0, T_b = +12.0,
        velocity_m_s = P.V_LANDING_M_S, altitude_m = 0.0,
        alpha_sweep  = (-2.0, +2.0, +5.0, +11.0, +14.0, +17.0, +20.0, +25.0, +30.0),
        htail_sweep  = (-10.0, 0.0, +10.0, +15.0, +20.0, +25.0, +30.0, +35.0, +40.0, +50.0),
        thrust_sweep = (4.0, 7.0, 10.0, 14.0, 16.0, 18.0, 21.0, 25.0, 30.0),
        name_prefix  = "gap40_LD",
        tags         = ["v2", "gapped40", "landing", "phase2"],
    ),
}


def sweep_one(phase_key: str):
    spec = SPECS[phase_key]
    parent = fl.Case.from_cloud(case_id=spec["parent_case_id"])
    project = fl.Project.from_cloud(project_id=parent.project_id)
    surfaces = C.get_gapped_geometry_surfaces(project)
    print(f"\n=== {phase_key} — parent {parent.id[:18]} project {project.id[:18]} ===")

    def submit(vary: str, values):
        out = []
        for v in values:
            ta = radians(v) if vary == "alpha"  else radians(spec["alpha_b"])
            th = radians(v) if vary == "htail"  else radians(spec["theta_ht_b"])
            tm = v          if vary == "thrust" else spec["T_b"]
            name = (f"{spec['name_prefix']}_{vary}_{v:+.2f}"
                    .replace("+", "p").replace("-", "m").replace(".", "p"))
            case = project.run_case(
                params=C.build_params(
                    surfaces, ta, th, tm,
                    velocity_m_s   = spec["velocity_m_s"],
                    altitude_m     = spec["altitude_m"],
                    n_steps_total  = N_FORK_TOTAL,
                    max_pseudo_steps = PSEUDO_FORK,
                ),
                name=name, run_async=True, fork_from=parent,
                tags=["SI", "fork", f"{vary}_sweep", *spec["tags"]],
                use_beta_mesher=True,
            )
            out.append((v, case.id))
            print(f"  {vary:6s} {v:+6.2f}  →  {case.id}")
        return out

    return {
        "phase":  phase_key,
        "project": project.id,
        "parent":  parent.id,
        "alpha":   submit("alpha",  spec["alpha_sweep"]),
        "htail":   submit("htail",  spec["htail_sweep"]),
        "thrust":  submit("thrust", spec["thrust_sweep"]),
    }


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] in SPECS:
        keys = [sys.argv[1]]
    elif len(sys.argv) > 1:
        sys.exit(f"unknown phase {sys.argv[1]!r}; choices: {list(SPECS)}")
    else:
        keys = list(SPECS)

    results = [sweep_one(k) for k in keys]

    print("\n=== Summary ===")
    for r in results:
        print(f"\n{r['phase']}: project {r['project']}  parent {r['parent']}")
        for swp in ("alpha", "htail", "thrust"):
            print(f"  {swp}:")
            for v, cid in r[swp]:
                print(f"    {v:+7.2f}  {cid}")
