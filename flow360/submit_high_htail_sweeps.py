"""
Fork α / θ_htail / T_mult sweeps off the cont-high and gap-high parents
in folders 2 and 4.  Mirrors submit_gapped_sweeps.py but applies the
appropriate get_*_surfaces helper per geometry AND passes
htail_z_m=P.Z_TAIL_HIGH_M so the htail rotation cylinder follows the
raised htail.

Usage:
    python3 flow360/submit_high_htail_sweeps.py                       # all 6
    python3 flow360/submit_high_htail_sweeps.py continuous            # both cfgs of one config
    python3 flow360/submit_high_htail_sweeps.py continuous cruise     # one combo
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

N_PARENT_STEPS = 20
N_FORK_NEW     = 6
N_FORK_TOTAL   = N_PARENT_STEPS + N_FORK_NEW
PSEUDO_FORK    = 500

# Parents are looked up by name pattern within the project — Flow360
# re-issues the trailing case-id segment when a draft case promotes to
# COMPLETED, so a static-id dict goes stale (verified empirically).
PROJECT_IDS = {
    ("continuous", "cruise"):  "prj-21d5e737-a1d9-4dba-ad7c-2155e647ede1",
    ("continuous", "takeoff"): "prj-3505c35f-0a58-4aa2-9b0e-75af2f2143f6",
    ("continuous", "landing"): "prj-121d08b0-626c-475b-8c53-9d2ab54765d7",
    ("gapped",     "cruise"):  "prj-d5d18139-3f53-4bf5-b224-7d3a5f3feb2a",
    ("gapped",     "takeoff"): "prj-fadaacba-ac09-4b2f-a0f2-364c46f5cb02",
    ("gapped",     "landing"): "prj-1315636c-f6d9-4076-ba32-0ec82c272430",
}


def find_parent(cfg: str, phase: str) -> fl.Case:
    pid = PROJECT_IDS[(cfg, phase)]
    project = fl.Project.from_cloud(project_id=pid)
    for cid in project.get_case_ids():
        c = fl.Case.from_cloud(case_id=cid)
        if c.name.endswith("_parent"):
            return c
    raise RuntimeError(f"no parent case in project {pid}")

GET_SURFS = {
    "continuous": C.get_geometry_surfaces,
    "gapped":     C.get_gapped_geometry_surfaces,
}

SPECS = {
    "cruise": dict(
        alpha_b=+7.0, theta_ht_b=0.0, T_b=+1.0,
        velocity_m_s=P.V_CRUISE_M_S, altitude_m=P.ALT_CRUISE_M,
        alpha_sweep =(-3.0, -1.0, +1.0, +3.0, +5.0, +9.0, +11.0, +13.0, +15.0),
        htail_sweep =(-12.0, -9.0, -6.0, -3.0, +3.0, +6.0, +9.0, +12.0, +15.0),
        thrust_sweep=(0.0, 0.25, 0.50, 0.75, 1.25, 1.50, 2.00, 2.50, 3.00),
    ),
    "takeoff": dict(
        alpha_b=+8.0, theta_ht_b=-5.0, T_b=+16.0,
        velocity_m_s=P.V_TAKEOFF_M_S, altitude_m=0.0,
        alpha_sweep =(-2.0, +2.0, +5.0, +11.0, +14.0, +17.0, +20.0, +25.0, +30.0),
        htail_sweep =(-15.0, -10.0, 0.0, +5.0, +10.0, +15.0, +20.0, +25.0, +30.0),
        thrust_sweep=(6.0, 9.0, 12.0, 14.0, 18.0, 20.0, 22.0, 25.0, 30.0),
    ),
    "landing": dict(
        alpha_b=+8.0, theta_ht_b=-6.0, T_b=+12.0,
        velocity_m_s=P.V_LANDING_M_S, altitude_m=0.0,
        alpha_sweep =(-2.0, +2.0, +5.0, +11.0, +14.0, +17.0, +20.0, +25.0, +30.0),
        htail_sweep =(-10.0, 0.0, +10.0, +15.0, +20.0, +25.0, +30.0, +35.0, +40.0, +50.0),
        thrust_sweep=(4.0, 7.0, 10.0, 14.0, 16.0, 18.0, 21.0, 25.0, 30.0),
    ),
}


def sweep_one(cfg: str, phase: str):
    spec = SPECS[phase]
    parent = find_parent(cfg, phase)
    project = fl.Project.from_cloud(project_id=parent.project_id)
    surfaces = GET_SURFS[cfg](project)
    tag_cfg = "cont" if cfg == "continuous" else "gap40"
    prefix = f"{tag_cfg}_{phase}_high"
    print(f"\n=== {cfg} {phase} — parent {parent.id[:18]} project {project.id[:18]} ===")

    def submit(vary: str, values):
        out = []
        for v in values:
            ta = radians(v) if vary == "alpha"  else radians(spec["alpha_b"])
            th = radians(v) if vary == "htail"  else radians(spec["theta_ht_b"])
            tm = v          if vary == "thrust" else spec["T_b"]
            name = (f"{prefix}_{vary}_{v:+.2f}"
                    .replace("+", "p").replace("-", "m").replace(".", "p"))
            case = project.run_case(
                params=C.build_params(
                    surfaces, ta, th, tm,
                    velocity_m_s     = spec["velocity_m_s"],
                    altitude_m       = spec["altitude_m"],
                    n_steps_total    = N_FORK_TOTAL,
                    max_pseudo_steps = PSEUDO_FORK,
                    htail_z_m        = P.Z_TAIL_HIGH_M,
                ),
                name=name, run_async=True, fork_from=parent,
                tags=["SI", "fork", f"{vary}_sweep", "v2", "high_htail",
                      tag_cfg, phase],
                use_beta_mesher=True,
            )
            out.append((v, case.id))
            print(f"  {vary:6s} {v:+6.2f}  →  {case.id}")
        return out

    return {
        "cfg": cfg, "phase": phase,
        "project": project.id, "parent": parent.id,
        "alpha":  submit("alpha",  spec["alpha_sweep"]),
        "htail":  submit("htail",  spec["htail_sweep"]),
        "thrust": submit("thrust", spec["thrust_sweep"]),
    }


if __name__ == "__main__":
    CONFIGS_LIST = ("continuous", "gapped")
    PHASES_LIST  = ("cruise", "takeoff", "landing")
    args = [a.lower() for a in sys.argv[1:]]
    cfgs = [a for a in args if a in CONFIGS_LIST] or list(CONFIGS_LIST)
    phs  = [a for a in args if a in PHASES_LIST]  or list(PHASES_LIST)
    print(f"Submitting sweeps for cfgs={cfgs}, phases={phs}")
    results = []
    for c in cfgs:
        for p in phs:
            parent = find_parent(c, p)
            if "COMPLETED" not in str(parent.status):
                print(f"\n=== {c} {p} parent {parent.id[:18]} not COMPLETED "
                      f"({parent.status}) — skipping ===")
                continue
            results.append(sweep_one(c, p))
    print("\n=== Summary ===")
    for r in results:
        n = len(r["alpha"]) + len(r["htail"]) + len(r["thrust"])
        print(f"  {r['cfg']:10s} {r['phase']:8s}  parent {r['parent']}  +{n} forks")
