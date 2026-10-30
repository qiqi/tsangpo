"""Submit body-frame X-slice forks for the TWO additional landing configs
(gap-low and v3) so the downwash figure can be split per-config in the
paper.

Cuts:
  v2 configs (cont_low, cont_high, gap_low, gap_high): htail LE at +4.847 m
    → cut at body-x = +4.40 m (0.45 m upstream of LE).
  v3 (short boom): htail LE at +2.077 m
    → cut at body-x = +1.85 m (0.23 m upstream of LE).

Slice normal follows airframe pitch: (cos α, 0, -sin α) in world.
"""
from __future__ import annotations

import math, sys, json
from copy import deepcopy
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
import flow360 as fl

PARENTS = {
    # config, α : (parent_case_id, x_body_station_m)
    ("gap_low", 5):  ("case-55f630c7-4cc3-4821-83bd-80e5b25562d4", 4.40),
    ("gap_low", 17): ("case-7959cdcb-391c-4cb4-a996-5587f3b2a456", 4.40),
    ("v3",      5):  ("case-c4698f56-dcc3-4f51-90f4-fa900a13058d", 1.85),
    ("v3",      17): ("case-850d0b4d-7d06-42c6-8ede-d4b3a2dd8d03", 1.85),
}


def build_body_x_slice(alpha_deg: float, x_body_m: float):
    a = math.radians(alpha_deg)
    n = (math.cos(a), 0.0, -math.sin(a))
    o = (x_body_m * math.cos(a), 0.0, -x_body_m * math.sin(a))
    return fl.SliceOutput(
        name="x_slice_body",
        entities=[fl.Slice(name=f"x_body=+{x_body_m:.2f}",
                            normal=n, origin=o * fl.u.m)],
        output_fields=["velocity", "Mach", "Cp", "primitiveVars"],
    )


def main():
    OUT = REPO / "post/v3/downwash_slice_forks_body_more.json"
    submitted = {}
    if OUT.exists():
        submitted = json.loads(OUT.read_text())
    for (config, alpha), (parent_id, x_body) in PARENTS.items():
        key = f"{config}_alpha{alpha}"
        if key in submitted:
            print(f"  {key}: already submitted ({submitted[key]})"); continue
        parent = fl.Case.from_cloud(case_id=parent_id)
        proj   = fl.Project.from_cloud(project_id=parent.project_id)
        new_params = deepcopy(parent.params)
        new_params.outputs = list(new_params.outputs) + [build_body_x_slice(alpha, x_body)]
        new_params.time_stepping.steps = 1
        new_params.time_stepping.max_pseudo_steps = 80
        new_case = proj.run_case(
            params=new_params, fork_from=parent,
            name=f"{parent.name}_xbodyslice",
            run_async=True, use_beta_mesher=True,
            tags=(list(parent.tags) if parent.tags else []) + ["xslice_body", "downwash"],
        )
        print(f"  {key}: forked → {new_case.id} (x_body={x_body:.2f})")
        submitted[key] = new_case.id
        OUT.write_text(json.dumps(submitted, indent=2))


if __name__ == "__main__":
    main()
