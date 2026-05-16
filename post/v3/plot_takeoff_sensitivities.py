"""v3 short-boom takeoff sensitivities + trim.  Uses post/_phase_plot.run()."""
from __future__ import annotations
import argparse, json, sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "post"))
import params as P
from _phase_plot import PhaseSpec, run


def _project_id(phase: str) -> str:
    p = Path(__file__).parent / "project_ids.json"
    if p.exists():
        try:
            return json.loads(p.read_text()).get(phase, "TBD")
        except Exception:
            return "TBD"
    return "TBD"


SPEC = PhaseSpec(
    label       = "v3 short-boom takeoff",
    project_id  = _project_id("takeoff"),
    out_dir     = REPO / "post" / "out" / "v3",
    phase_name  = "takeoff",
    alpha_b     = +15.0,
    theta_ht_b  = -5.0,
    T_b         = +15.21,
    velocity    = P.V_TAKEOFF_M_S,
    rho         = P.RHO_TAKEOFF_KG_M3,
    gamma_deg   = P.CLIMB_ANGLE_DEG,
    # v3 short-boom takeoff: copying v2_gapped masks until v3 data lands.
    mask_alpha  = lambda a: a <= 11.0,
    mask_htail  = lambda h: h <= 12.0,
    mask_thrust = lambda t: np.ones_like(t, dtype=bool),
    title       = "Tsangpo v3 TAKEOFF (short-boom, gap40) — BO α=+15°, "
                  "θ_ht=-5°, T_mult=15.21, V=18 m/s, γ=+30°",
)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--refresh", action="store_true")
    run(SPEC, refresh=ap.parse_args().refresh)
