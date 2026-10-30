"""α-sweep RERUN at higher pinned θ_ht for the two continuous-flap
configs (cont_low, cont_high), takeoff + landing.

Original v2 α-sweeps were pinned at the BO θ_ht (-5° takeoff, -6° landing).
At high α in those configs the downwash field pushes the htail into stall,
which contaminates the dCm/dα slope used for static-margin extraction.
This script reruns the α-sweep at:
    takeoff:  θ_ht = +10°
    landing:  θ_ht = +20°
which keeps the htail in its linear regime across the swept α range.

Forks each project's `*_parent` case at every α in the sweep list.

Run:
    python3 flow360/submit_alpha_resweep_high_htail.py
"""
from __future__ import annotations

import json
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

ALPHA_VALUES = (-2.0, +2.0, +5.0, +8.0, +11.0, +14.0, +17.0, +20.0, +25.0, +30.0)

THETA_HT_PIN = {"takeoff": +10.0, "landing": +20.0}
T_MULT_BO    = {"takeoff": +16.0, "landing": +12.0}
VELOCITY_M_S = {"takeoff": P.V_TAKEOFF_M_S, "landing": P.V_LANDING_M_S}

PROJECTS = {
    ("cont_low",  "takeoff"): "prj-9cd3ad10-ea47-41d9-9e33-0096fc30d6c1",
    ("cont_low",  "landing"): "prj-e7dc7d6d-4baf-4101-8818-1173da5a359a",
    ("cont_high", "takeoff"): "prj-3505c35f-0a58-4aa2-9b0e-75af2f2143f6",
    ("cont_high", "landing"): "prj-121d08b0-626c-475b-8c53-9d2ab54765d7",
}

HTAIL_Z = {
    "cont_low":  P.Z_TAIL_LOW_M,
    "cont_high": P.Z_TAIL_HIGH_M,
}


def find_parent(project: fl.Project) -> fl.Case:
    for cid in project.get_case_ids():
        c = fl.Case.from_cloud(case_id=cid)
        if c.name.endswith("_parent") and "fine_ad" in c.name:
            return c
    raise RuntimeError(f"no fine_ad parent in project {project.id}")


def sweep_one(cfg: str, phase: str):
    pid = PROJECTS[(cfg, phase)]
    project = fl.Project.from_cloud(project_id=pid)
    parent = find_parent(project)
    if "COMPLETED" not in str(parent.status):
        print(f"  skip {cfg} {phase}: parent {parent.id} not COMPLETED ({parent.status})")
        return None

    surfaces = C.get_geometry_surfaces(project)
    htail_z = HTAIL_Z[cfg]
    theta_ht_deg = THETA_HT_PIN[phase]
    T_b = T_MULT_BO[phase]
    v_m_s = VELOCITY_M_S[phase]
    tag_cfg = cfg  # cont_low or cont_high

    prefix = f"{cfg}_{phase}_alphaResweep_thtP{int(theta_ht_deg):02d}"
    print(f"\n=== {cfg} {phase}  parent={parent.id}  θ_ht_pin=+{theta_ht_deg:.0f}° ===")

    out = []
    for a in ALPHA_VALUES:
        name = (f"{prefix}_alpha{a:+.1f}"
                .replace("+", "p").replace("-", "m").replace(".", "p"))
        case = project.run_case(
            params=C.build_params(
                surfaces,
                theta_ac_rad=radians(a),
                theta_ht_rad=radians(theta_ht_deg),
                thrust_mult=T_b,
                velocity_m_s=v_m_s,
                altitude_m=0.0,
                n_steps_total=N_FORK_TOTAL,
                max_pseudo_steps=PSEUDO_FORK,
                htail_z_m=htail_z,
            ),
            name=name, run_async=True, fork_from=parent,
            tags=["SI", "fork", "alpha_sweep", "v2", "alpha_resweep_high_htail",
                  tag_cfg, phase, f"th{int(theta_ht_deg):+03d}"],
            use_beta_mesher=True,
        )
        out.append((a, case.id))
        print(f"  α={a:+5.1f}°  →  {case.id}")
    return {"cfg": cfg, "phase": phase, "project": pid,
            "parent": parent.id, "theta_ht_pin": theta_ht_deg, "alpha": out}


def main():
    OUT = REPO / "flow360" / "alpha_resweep_high_htail.json"
    records = json.loads(OUT.read_text()) if OUT.exists() else []
    keys_done = {(r["cfg"], r["phase"]) for r in records}

    for cfg in ("cont_low", "cont_high"):
        for phase in ("takeoff", "landing"):
            if (cfg, phase) in keys_done:
                print(f"  {cfg} {phase}: already submitted, skip")
                continue
            r = sweep_one(cfg, phase)
            if r is None:
                continue
            records.append(r)
            OUT.write_text(json.dumps(records, indent=2))

    print(f"\n=== Submitted ({len(records)} sweeps) ===")
    for r in records:
        print(f"  {r['cfg']:10s} {r['phase']:8s}  +{len(r['alpha'])} α-forks  "
              f"@ θ_ht=+{r['theta_ht_pin']:.0f}°")


if __name__ == "__main__":
    main()
