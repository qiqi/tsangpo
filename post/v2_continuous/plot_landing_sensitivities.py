"""v2 continuous-flap landing sensitivities + trim.  Uses post/_phase_plot.run()."""
from __future__ import annotations
import argparse, sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "post"))
import params as P
from _phase_plot import PhaseSpec, run


SPEC = PhaseSpec(
    label       = "v2 continuous landing",
    project_id  = "prj-e7dc7d6d-4baf-4101-8818-1173da5a359a",
    out_dir     = REPO / "post" / "out" / "v2_continuous",
    phase_name  = "landing",
    alpha_b     = +8.0,
    theta_ht_b  = -6.0,
    T_b         = +12.0,
    velocity    = P.V_LANDING_M_S,
    rho         = P.RHO_LANDING_KG_M3,
    gamma_deg   = P.DESCENT_ANGLE_DEG,
    # Wing CL peaks at α=+11°; htail in heavy downwash blanket for θ_ht ≤ 0;
    # over-the-top stall past +40°; thrust CMy flattens past +25.
    mask_alpha  = lambda a: a <= 8.0,
    mask_htail  = lambda h: (h >= 10.0) & (h <= 40.0),
    mask_thrust = lambda t: np.ones_like(t, dtype=bool),
    title       = "Tsangpo v2 LANDING (continuous flap) — BO α=+8°, θ_ht=-6°, T=+12, "
                  "V=12.86 m/s, γ=-30°",
)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--refresh", action="store_true")
    run(SPEC, refresh=ap.parse_args().refresh)
