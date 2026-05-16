"""v3 short-boom cruise sensitivities + trim.  Uses post/_phase_plot.run()."""
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
    label       = "v3 short-boom cruise",
    project_id  = _project_id("cruise"),
    out_dir     = REPO / "post" / "out" / "v3",
    phase_name  = "cruise",
    alpha_b     = +7.0,
    theta_ht_b  = +2.0,
    T_b         = +2.17,
    velocity    = P.V_CRUISE_M_S,
    rho         = P.RHO_CRUISE_KG_M3,
    gamma_deg   = 0.0,
    mask_alpha  = lambda a: a <= 11.0,
    mask_htail  = lambda h: np.ones_like(h, dtype=bool),
    mask_thrust = lambda t: np.ones_like(t, dtype=bool),
    title       = "Tsangpo v3 CRUISE (short-boom, gap40) — α=+7°, θ_ht=+2°, "
                  "T_mult=2.17, V=45.72 m/s, level",
)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--refresh", action="store_true")
    run(SPEC, refresh=ap.parse_args().refresh)
