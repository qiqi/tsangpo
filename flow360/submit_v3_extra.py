"""
Submit *additional* v3 sweep cases on top of the existing
takeoff/landing parents.

  * α extension (2× coarser grid than the original 2° steps): pushes
    down to α = 0° for takeoff and landing.
       takeoff:  α ∈ {7, 3, 0}°            (BO = +15°, was bottom = +11°)
       landing:  α ∈ {17, 13, 9, 5, 0}°    (BO = +25°, was bottom = +21°)

  * htail extension toward stall on the negative side (2 at a time):
       takeoff:  θ_ht ∈ {−25, −30}°        (was bottom = −20°)
       landing:  θ_ht ∈ {−15, −20}°        (was bottom = −10°)

Each new case is forked from the existing `_parent` case in the same
project (project IDs read from post/v3/project_ids.json).  Run after
the parents have COMPLETED; sweep cases inherit the parent's volume mesh.

Usage:
    python3 flow360/submit_v3_extra.py                  # both phases
    python3 flow360/submit_v3_extra.py takeoff          # one phase
"""
from __future__ import annotations

import json
import sys
import traceback
from math import radians
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
import cfd_setup as C
import params as P
import flow360 as fl

# Same fork-step counts as submit_v3.py (matches the existing sweep cases).
N_PARENT_STEPS = 20
N_FORK_NEW     = 6
N_FORK_TOTAL   = N_PARENT_STEPS + N_FORK_NEW
PSEUDO_FORK    = 500

PROJECT_IDS_PATH = REPO / "post" / "v3" / "project_ids.json"


# v3 BO trim points + new sweep extensions.
#
# Note on resubmission: this same script has been used for two rounds.
# Round 1 (committed earlier): α extension to 0° + first htail-negative
#   batch (takeoff −25°/−30°, landing −15°/−20°).
# Round 2 (current): just push htail further negative — round-1 data
#   still showed linear dCm/dθ_ht ≈ −0.10/° at the previous min, so we
#   step out another 10° each.  α_extend is empty this round.
SPECS = {
    "takeoff": dict(
        alpha_b         = +15.0,
        theta_ht_b      =  -5.0,
        T_b             = +15.21,
        velocity_m_s    = P.V_TAKEOFF_M_S,
        altitude_m      = 0.0,
        alpha_extend    = (),
        htail_extend    = ( -35.0, -40.0 ),
        tags_phase      = ["v3", "gapped40", "short_boom", "takeoff", "gamma_p30"],
    ),
    "landing": dict(
        alpha_b         = +25.0,
        theta_ht_b      =  +5.0,
        T_b             =  +8.69,
        velocity_m_s    = P.V_LANDING_M_S,
        altitude_m      = 0.0,
        alpha_extend    = (),
        htail_extend    = ( -25.0, -30.0 ),
        tags_phase      = ["v3", "gapped40", "short_boom", "landing", "gamma_m30"],
    ),
}


def _name_token(value: float) -> str:
    """Encode a signed float as 'p07p00' / 'm04p00' — matches submit_v3.py."""
    sign = "p" if value >= 0 else "m"
    return f"{sign}{abs(value):05.2f}".replace(".", "p")


def _project_id(phase_key: str) -> str:
    d = json.loads(PROJECT_IDS_PATH.read_text())
    if phase_key not in d:
        raise SystemExit(f"no project_id for {phase_key} in {PROJECT_IDS_PATH}")
    return d[phase_key]


def _find_parent(project: fl.Project, phase_key: str) -> fl.Case:
    """Locate the v3 parent case by name.  Flow360 rotates the trailing
    case-id segment when a draft promotes to COMPLETED, so we look it up
    fresh on every run instead of caching the id."""
    target = f"v3_{phase_key}_parent"
    for cid in project.get_case_ids():
        c = fl.Case.from_cloud(case_id=cid)
        if c.name == target:
            return c
    raise RuntimeError(f"no case named {target!r} in project {project.id}")


def submit_one(phase_key: str) -> dict:
    spec = SPECS[phase_key]
    alpha_b_deg = spec["alpha_b"]
    theta_ht_b_deg = spec["theta_ht_b"]
    T_b = spec["T_b"]
    V   = spec["velocity_m_s"]
    H   = spec["altitude_m"]

    print(f"\n=== {phase_key.upper()} extension ===")
    print(f"  BO   α={alpha_b_deg:+.2f}°  θ_ht={theta_ht_b_deg:+.2f}°  T_mult={T_b:+.3f}")
    print(f"  α  extend: {spec['alpha_extend']}")
    print(f"  θ_ht extend: {spec['htail_extend']}")

    pid = _project_id(phase_key)
    project = fl.Project.from_cloud(project_id=pid)
    parent  = _find_parent(project, phase_key)
    print(f"  project {project.id}")
    print(f"  parent  {parent.id}   ({parent.name})")

    surfaces = C.get_gapped_geometry_surfaces(project)

    forks = {"alpha": [], "htail": []}

    def _submit_fork(sweep: str, value: float):
        if sweep == "alpha":
            ta, th, tm = radians(value), radians(theta_ht_b_deg), T_b
        elif sweep == "htail":
            ta, th, tm = radians(alpha_b_deg), radians(value), T_b
        else:
            raise ValueError(sweep)
        name = f"v3_{phase_key}_{sweep}_{_name_token(value)}"
        case = project.run_case(
            params=C.build_params(
                surfaces,
                theta_ac_rad     = ta,
                theta_ht_rad     = th,
                thrust_mult      = tm,
                velocity_m_s     = V,
                altitude_m       = H,
                n_steps_total    = N_FORK_TOTAL,
                max_pseudo_steps = PSEUDO_FORK,
                htail_x_m        = P.X_TAIL_LE_V3_M + 0.25 * P.HTAIL_CHORD_M,
                ht_radius_m      = 1.0 * P.HTAIL_CHORD_M,
            ),
            name=name,
            run_async=True,
            fork_from=parent,
            tags=["SI", "fork", "sweep", f"{sweep}_sweep", "extend",
                   *spec["tags_phase"]],
            use_beta_mesher=True,
        )
        forks[sweep].append((value, case.id))
        print(f"  fork  {sweep:6s} {value:+7.2f}  →  {case.id}   ({name})")

    for v in spec["alpha_extend"]:
        try:
            _submit_fork("alpha", v)
        except Exception as exc:  # noqa: BLE001
            print(f"  fork  alpha  {v:+7.2f}  FAILED: {exc!r}")
            traceback.print_exc()

    for v in spec["htail_extend"]:
        try:
            _submit_fork("htail", v)
        except Exception as exc:  # noqa: BLE001
            print(f"  fork  htail  {v:+7.2f}  FAILED: {exc!r}")
            traceback.print_exc()

    return {"phase": phase_key, "project": project.id, "parent": parent.id, "forks": forks}


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] in SPECS:
        keys = [sys.argv[1]]
    elif len(sys.argv) > 1:
        sys.exit(f"unknown phase {sys.argv[1]!r}; choices: {list(SPECS)}")
    else:
        keys = list(SPECS)

    results = []
    for k in keys:
        try:
            results.append(submit_one(k))
        except Exception as exc:  # noqa: BLE001
            print(f"\n!!! {k} FAILED: {exc!r}")
            traceback.print_exc()

    print("\n\n=== v3 extra-sweep submission summary ===")
    for r in results:
        print(f"\n{r['phase']}: project {r['project']}  parent {r['parent']}")
        for sweep in ("alpha", "htail"):
            print(f"  {sweep}:")
            for v, cid in r["forks"][sweep]:
                print(f"    {v:+7.2f}  {cid}")
