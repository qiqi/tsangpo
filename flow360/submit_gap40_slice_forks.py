"""
Fork every gap40 (gapped_low_htail) case with `SliceOutput` added on a set
of y-planes (symmetry plane through wing tip, hitting prop centers and
between-prop midpoints).

Strategy: pull each completed case's converged `SimulationParams` from
the cloud (`case.params`), deepcopy it, mutate `time_stepping` to a
single more unsteady step, and append a `SliceOutput`.  Submit as a
fork (`fork_from=parent_case`) so the new run inherits the parent's
mesh and warm-starts from its converged flow.

Skips cases that have already been forked (name ending `_slice`) so
the script is idempotent.

Run:
    python3 flow360/submit_gap40_slice_forks.py [cruise|takeoff|landing]
With no arg, submits forks across all three gap40 phases.
"""
from __future__ import annotations

import argparse
import sys
from copy import deepcopy
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
import flow360 as fl
import params as P


# 12 y-planes: symmetry, between-symmetry-and-prop-1, each prop center,
# midpoints between consecutive props, wing tip.
def slice_y_positions() -> list[float]:
    props = list(P.PROP_Y_M)
    ys = [0.0, props[0] / 2.0]
    for i, p in enumerate(props):
        ys.append(p)
        if i < len(props) - 1:
            ys.append((p + props[i + 1]) / 2.0)
    ys.append(P.WING_SEMI_SPAN_M)
    return ys


SLICE_YS = slice_y_positions()
print(f"Slice planes ({len(SLICE_YS)}):  " +
      "  ".join(f"y={y:.3f}" for y in SLICE_YS))


GAP40_PROJECTS = {
    "cruise":  "prj-59c27343-3c39-43a4-ab19-18863acf02c4",
    "takeoff": "prj-e0e11ed5-2f3a-4e7d-8af7-c79baaf3ed22",
    "landing": "prj-0cd29981-d281-442a-984f-06562abc1f39",
}


def build_slice_output() -> fl.SliceOutput:
    slices = [
        fl.Slice(name=f"y={y:+.3f}",
                 normal=(0.0, 1.0, 0.0),
                 origin=(0.0, y, 0.0) * fl.u.m)
        for y in SLICE_YS
    ]
    return fl.SliceOutput(
        name="y_slices",
        entities=slices,
        # primitiveVars gives rho, u, v, w, p — enough to derive
        # total pressure (p + ½ρ|u|²) and velocity magnitude post-hoc.
        output_fields=["velocity", "Mach", "Cp", "primitiveVars"],
    )


def mutate_params(parent_params: "fl.SimulationParams") -> "fl.SimulationParams":
    new = deepcopy(parent_params)
    new.outputs = list(new.outputs) + [build_slice_output()]
    # Single more unsteady step from the warm-started flow; small
    # pseudo-step budget since the parent was already converged.
    new.time_stepping.steps = 1
    new.time_stepping.max_pseudo_steps = 80
    return new


def submit_phase(phase: str, dry_run: bool = False, limit: int | None = None):
    pid = GAP40_PROJECTS[phase]
    proj = fl.Project.from_cloud(project_id=pid)
    print(f"\n=== {phase}  (project {pid[:18]}) ===")
    submitted = 0
    for cid in proj.get_case_ids():
        c = fl.Case.from_cloud(case_id=cid)
        if "COMPLETED" not in str(c.status):
            continue
        if c.name.endswith("_slice"):
            continue
        new_name = c.name + "_slice"
        # idempotency — skip if the _slice fork already exists
        if any(fl.Case.from_cloud(case_id=x).name == new_name
               for x in proj.get_case_ids()):
            continue
        if dry_run:
            print(f"  [dry-run] would fork  {c.name}  →  {new_name}")
            submitted += 1
            if limit and submitted >= limit:
                break
            continue
        new_params = mutate_params(c.params)
        new_case = proj.run_case(
            params=new_params, fork_from=c,
            name=new_name, run_async=True,
            use_beta_mesher=True,
            tags=(list(c.tags) if c.tags else []) + ["slice"],
        )
        print(f"  forked  {c.name:38s}  →  {new_case.id}")
        submitted += 1
        if limit and submitted >= limit:
            break
    print(f"  ({submitted} cases forked)")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("phases", nargs="*", default=["cruise", "takeoff", "landing"])
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--limit", type=int, default=None,
                    help="Limit fork count per phase (for testing).")
    args = ap.parse_args()
    for ph in args.phases:
        if ph not in GAP40_PROJECTS:
            print(f"unknown phase {ph!r}")
            continue
        submit_phase(ph, dry_run=args.dry_run, limit=args.limit)
