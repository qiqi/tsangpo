"""Fork 6 landing-sweep cases (3 configs × 2 alphas) with an X-slice output
at the htail-LE x-station.  The fork inherits the converged steady flow,
adds the X-slice, runs ONE more unsteady step, and writes out velocity +
primitive variables on the slice plane so we can compute downwash angle.

Configs / alphas selected:
  v2_continuous (cont-low)         α=5°, α=17°
  v2_continuous_high (cont-high)   α=5°, α=17°
  v2_gapped_high (gap-high)        α=5°, α=17°

Cut x-station:
  All three v2 configs share htail LE at x = +4.50 m
  (htail_c4_x_m = +5.19 m, chord = 1.385 m → LE = 5.19 − 0.346 = +4.844 m,
   to be safe we cut 0.10 m upstream of LE).

Output: post/v3/downwash_slice_forks.json with the new case IDs.
"""
from __future__ import annotations

import sys
import json
from copy import deepcopy
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
import flow360 as fl


PARENT_CASES = {
    # (config, alpha) → parent case id
    ("cont_low",  5):  "case-cd6044ac-26d1-4136-94dc-d3c380c3cbda",
    ("cont_low",  17): "case-65daeabe-7e44-4d37-804c-88f2a5837538",
    ("cont_high", 5):  "case-54f70d97-7224-4c03-909a-0d718a41a6b6",
    ("cont_high", 17): "case-e1b337c0-755c-4fee-8740-76dd9b1e463d",
    ("gap_high",  5):  "case-34741da9-7f2a-429f-9615-52b6994000e2",
    ("gap_high",  17): "case-df5ec973-caed-44b7-ada5-ddb71d9570a6",
}

# htail LE at +4.50 m for all v2 configs; cut 0.10 m upstream so the slice
# samples incoming flow without intersecting the htail itself.
HTAIL_LE_X = 4.50
SLICE_X    = HTAIL_LE_X - 0.10   # = 4.40 m


def build_x_slice() -> "fl.SliceOutput":
    """One slice at x = SLICE_X with normal +x, covering full yz span."""
    return fl.SliceOutput(
        name="x_slice_htail_upstream",
        entities=[
            fl.Slice(name=f"x={SLICE_X:+.2f}",
                     normal=(1.0, 0.0, 0.0),
                     origin=(SLICE_X, 0.0, 0.0) * fl.u.m),
        ],
        output_fields=["velocity", "Mach", "Cp", "primitiveVars"],
    )


def mutate_params(parent_params: "fl.SimulationParams") -> "fl.SimulationParams":
    new = deepcopy(parent_params)
    new.outputs = list(new.outputs) + [build_x_slice()]
    # Short unsteady step on warm-started flow; converged-flow already
    # gives the right field, we just need one step to write the slice.
    new.time_stepping.steps = 1
    new.time_stepping.max_pseudo_steps = 80
    return new


def main():
    OUT = REPO / "post/v3/downwash_slice_forks.json"
    submitted = {}
    if OUT.exists():
        submitted = json.loads(OUT.read_text())
    for (config, alpha), parent_id in PARENT_CASES.items():
        key = f"{config}_alpha{alpha}"
        if key in submitted:
            print(f"  {key}: already submitted as {submitted[key]} — skipping")
            continue
        parent = fl.Case.from_cloud(case_id=parent_id)
        proj   = fl.Project.from_cloud(project_id=parent.project_id)
        print(f"\n  {key}: parent {parent.name}  ({parent.status})  in project {parent.project_id[:18]}")
        new_params = mutate_params(parent.params)
        new_case = proj.run_case(
            params=new_params, fork_from=parent,
            name=f"{parent.name}_xslice_htail",
            run_async=True, use_beta_mesher=True,
            tags=(list(parent.tags) if parent.tags else []) + ["xslice", "downwash"],
        )
        print(f"  forked → {new_case.id}")
        submitted[key] = new_case.id
        OUT.write_text(json.dumps(submitted, indent=2))


if __name__ == "__main__":
    main()
