"""Body-frame variant of submit_downwash_slice_forks.py.

For each (config, α) pair, fork the parent landing-sweep case with an
X-slice whose normal follows the airframe's body x-axis (= world x
rotated by +α around +y, since `ac_pitch` rotates by exactly α).

In Flow360's RH convention R_y(α)·(1,0,0) = (cos α, 0, −sin α), so the
slice normal in world is (cos α, 0, −sin α) and the slice passes
through R_y(α)·(x_body_station, 0, 0) = (x_body·cos α, 0, −x_body·sin α).

The body-frame x-station is x_body = +4.40 m (same as world-frame
version — just upstream of htail LE).  In world that's a slightly tilted
plane that follows the airframe's pitch.
"""
from __future__ import annotations

import math
import sys
import json
from copy import deepcopy
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
import flow360 as fl

PARENT_CASES = {
    ("cont_low",  5):  "case-cd6044ac-26d1-4136-94dc-d3c380c3cbda",
    ("cont_low",  17): "case-65daeabe-7e44-4d37-804c-88f2a5837538",
    ("cont_high", 5):  "case-54f70d97-7224-4c03-909a-0d718a41a6b6",
    ("cont_high", 17): "case-e1b337c0-755c-4fee-8740-76dd9b1e463d",
    ("gap_high",  5):  "case-34741da9-7f2a-429f-9615-52b6994000e2",
    ("gap_high",  17): "case-df5ec973-caed-44b7-ada5-ddb71d9570a6",
}
X_BODY_STATION_M = 4.40   # body-x of the cut


def build_body_x_slice(alpha_deg: float) -> "fl.SliceOutput":
    a = math.radians(alpha_deg)
    n = (math.cos(a), 0.0, -math.sin(a))
    o = (X_BODY_STATION_M * math.cos(a),
         0.0,
         -X_BODY_STATION_M * math.sin(a))
    return fl.SliceOutput(
        name="x_slice_body",
        entities=[
            fl.Slice(name=f"x_body=+{X_BODY_STATION_M:.2f}",
                     normal=n,
                     origin=o * fl.u.m),
        ],
        output_fields=["velocity", "Mach", "Cp", "primitiveVars"],
    )


def mutate_params(parent_params, alpha_deg: float):
    new = deepcopy(parent_params)
    new.outputs = list(new.outputs) + [build_body_x_slice(alpha_deg)]
    new.time_stepping.steps = 1
    new.time_stepping.max_pseudo_steps = 80
    return new


def main():
    OUT = REPO / "post/v3/downwash_slice_forks_body.json"
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
        print(f"\n  {key}: parent {parent.name}  ({parent.status})")
        new_params = mutate_params(parent.params, alpha)
        new_case = proj.run_case(
            params=new_params, fork_from=parent,
            name=f"{parent.name}_xbodyslice",
            run_async=True, use_beta_mesher=True,
            tags=(list(parent.tags) if parent.tags else []) + ["xslice_body", "downwash"],
        )
        print(f"  forked → {new_case.id}")
        submitted[key] = new_case.id
        OUT.write_text(json.dumps(submitted, indent=2))


if __name__ == "__main__":
    main()
