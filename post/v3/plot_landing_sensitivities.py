"""v3 short-boom landing sensitivities + trim.  Uses post/_phase_plot.run()."""
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
    label       = "v3 short-boom landing",
    project_id  = _project_id("landing"),
    out_dir     = REPO / "post" / "out" / "v3",
    phase_name  = "landing",
    alpha_b     = +25.0,
    theta_ht_b  = +5.0,
    T_b         = +8.69,
    velocity    = P.V_LANDING_M_S,
    rho         = P.RHO_LANDING_KG_M3,
    a_sound     = P.A_SOUND_LANDING_M_S,
    gamma_deg   = P.DESCENT_ANGLE_DEG,
    # v3 short-boom landing: copying v2_gapped masks until v3 data lands.
    mask_alpha  = lambda a: a <= 30.0,
    mask_htail  = lambda h: h <= 20.0,
    mask_thrust = lambda t: np.ones_like(t, dtype=bool),
    title       = "Tsangpo v3 LANDING (short-boom, gap40) — BO α=+25°, "
                  "θ_ht=+5°, T_mult=8.69, V=12.86 m/s, γ=-30°",
)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--refresh", action="store_true")
    run(SPEC, refresh=ap.parse_args().refresh)
