"""v2 continuous-flap takeoff sensitivities + trim.  Uses post/_phase_plot.run()."""
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
    label       = "v2 continuous takeoff",
    project_id  = "prj-9cd3ad10-ea47-41d9-9e33-0096fc30d6c1",
    out_dir     = REPO / "post" / "out" / "v2_continuous",
    phase_name  = "takeoff",
    alpha_b     = +8.0,
    theta_ht_b  = -5.0,
    T_b         = +16.0,
    velocity    = P.V_TAKEOFF_M_S,
    rho         = P.RHO_TAKEOFF_KG_M3,
    a_sound     = P.A_SOUND_TAKEOFF_M_S,
    alpha_sweep_theta_ht = +10.0,
    gamma_deg   = P.CLIMB_ANGLE_DEG,
    # Wing α ≤ +11° pre-stall; htail blanketed by downwash for θ ≤ -5°;
    # thrust CMy slope reverses past T_mult > +22.
    mask_alpha  = lambda a: a <= 11.0,
    mask_htail  = lambda h: (h >= 0.0) & (h <= 25.0),
    mask_thrust = lambda t: np.ones_like(t, dtype=bool),
    title       = "Tsangpo v2 TAKEOFF (continuous flap) — BO α=+8°, θ_ht=-5°, T=+16, "
                  "V=18 m/s, γ=+30°",
)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--refresh", action="store_true")
    run(SPEC, refresh=ap.parse_args().refresh)
