"""Trajectory + attitude viz for the LANDING-RETRIM chain (θ_ht=−2°).

Parametric variant of viz_nokick_landing.py — uses the retrim case
keys (phugoid_landing_thtN2_chunk1, phugoid_landing_thtN2_chunk2..6)
and UDD name prefix `phugoid_6DOF_landing_thtN2_<tag>` (since the
new phase is "landing_thtN2").

Outputs:
  post/v3/phugoid_runs/landing_thtN2/nokick_landing_thtN2_trajectory.png
  post/v3/phugoid_runs/landing_thtN2/nokick_landing_thtN2_attitude.mp4
"""
from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO/'post'/'v3'))

# Reuse the parser + plotter from the baseline landing viz, but override
# the UDD phase prefix, case-key prefix, and output directory.
import viz_nokick_landing as VZ

VZ.UDD_PHASE        = "landing_thtN2"   # ← matches phase= in build_unsteady_params
VZ.OUT_DIR          = REPO / "post" / "v3" / "phugoid_runs" / "landing_thtN2"
VZ.OUT_DIR.mkdir(parents=True, exist_ok=True)

# Patch collect_chunks() to use the retrim case keys.
_orig_collect = VZ.collect_chunks
def _collect_thtN2():
    import json
    import flow360 as fl
    ids = json.loads((REPO / "post/v3/unsteady_project_ids.json").read_text())
    ordered = [("chunk1", ids.get("phugoid_landing_thtN2_chunk1"))]
    for n in range(2, 7):
        k = f"phugoid_landing_thtN2_chunk{n}"
        if ids.get(k):
            ordered.append((f"chunk{n}", ids[k]))
    out = []
    for label, cid in ordered:
        if cid is None:
            print(f"  {label}: not submitted yet — stop")
            break
        st = str(fl.Case.from_cloud(case_id=cid).status).split(".")[-1]
        if st not in ("COMPLETED", "COMPLETED_WITH_WARNINGS"):
            print(f"  {label} ({cid[:13]}): {st} — stop including here")
            break
        out.append((label, cid))
        print(f"  including {label} ({cid[:13]})")
    return out

VZ.collect_chunks = _collect_thtN2

# Patch the output-file paths inside main() by overriding OUT_DIR is enough
# (main() builds out_png/out_mp4 from OUT_DIR).  Run main directly.
if __name__ == "__main__":
    VZ.main()
