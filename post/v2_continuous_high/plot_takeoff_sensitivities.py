"""v2 continuous high-htail takeoff sensitivities + trim."""
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
    label       = "v2 continuous-high takeoff",
    project_id  = "prj-3505c35f-0a58-4aa2-9b0e-75af2f2143f6",
    out_dir     = REPO / "post" / "out" / "v2_continuous_high",
    phase_name  = "takeoff",
    alpha_b     = +8.0,
    theta_ht_b  = -5.0,
    T_b         = +16.0,
    velocity    = P.V_TAKEOFF_M_S,
    rho         = P.RHO_TAKEOFF_KG_M3,
    gamma_deg   = P.CLIMB_ANGLE_DEG,
    # Inherit low-htail masks; relax if high htail clears the downwash blanket.
    mask_alpha  = lambda a: a <= 11.0,
    mask_htail  = lambda h: np.ones_like(h, dtype=bool),
    mask_thrust = lambda t: t <= 22.0,
    title       = "Tsangpo v2 TAKEOFF (continuous flap, HIGH htail) — BO α=+8°, θ_ht=-5°, "
                  "T=+16, V=18 m/s, γ=+30°",
)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--refresh", action="store_true")
    run(SPEC, refresh=ap.parse_args().refresh)
