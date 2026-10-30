"""Fork the cruise warmup case with a single pseudo-step and a y=0 slice
output, so we can inspect the mesh resolution at the cyl_2 ⟷ cyl_3
interface.

Background: Flow360 disputes the nested-rotation bug report and
suggests the mesh between the elbow (cyl_2) and the airframe-pitch
cylinder (cyl_3) is too coarse for the freestream signal to propagate
through. To check, we slice the warmup mesh at y=0 (the longitudinal
symmetry plane) and visualise mesh edges on the −x (nose) side of the
body where the cyl_2 outer (x ≈ −3 c_w) and cyl_3 outer (x ≈ −2.9 c_w)
meet.
"""
from __future__ import annotations

import json
import sys
from copy import deepcopy
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

import flow360 as fl


PARENT_KEY = "cruise_warmup_widegap_case"
PROJECT_KEY = "cruise_warmup_widegap"


def build_y0_slice() -> fl.SliceOutput:
    return fl.SliceOutput(
        name="y0_slice",
        entities=[fl.Slice(name="y=0",
                            normal=(0.0, 1.0, 0.0),
                            origin=(0.0, 0.0, 0.0) * fl.u.m)],
        output_fields=["velocity", "Mach", "Cp", "primitiveVars"],
    )


def mutate_params(parent_params: "fl.SimulationParams") -> "fl.SimulationParams":
    new = deepcopy(parent_params)
    new.outputs = list(new.outputs) + [build_y0_slice()]
    # Single pseudo-step is enough: we only need the cell topology in
    # the slice file, not converged flow.
    if isinstance(new.time_stepping, fl.Unsteady):
        new.time_stepping.steps = 1
        new.time_stepping.max_pseudo_steps = 20
    else:
        # Parent is Steady — convert to a 1-step Unsteady so we get a
        # snapshot.
        new.time_stepping = fl.Unsteady(
            step_size=0.01 * fl.u.s,
            steps=1,
            max_pseudo_steps=20,
        )
    return new


def main():
    ids_path = REPO/"post"/"v3"/"unsteady_project_ids.json"
    d = json.loads(ids_path.read_text())
    project = fl.Project.from_cloud(project_id=d[PROJECT_KEY])
    parent = fl.Case.from_cloud(case_id=d[PARENT_KEY])
    print(f"  parent: {parent.id}  name={parent.name}  status={parent.status}")

    new_params = mutate_params(parent.params)
    new_name = parent.name + "_y0_meshslice"
    case = project.run_case(
        params=new_params, fork_from=parent,
        name=new_name, run_async=True,
        use_beta_mesher=True,
        tags=(list(parent.tags) if parent.tags else []) + ["mesh-slice", "y0"],
    )
    print(f"  submitted: {case.id}  name={new_name}")
    d["mesh_slice_y0_widegap"] = case.id
    ids_path.write_text(json.dumps(d, indent=2))
    return case.id


if __name__ == "__main__":
    main()
