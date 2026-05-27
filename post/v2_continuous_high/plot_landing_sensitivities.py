"""v2 continuous high-htail landing sensitivities + trim."""
from __future__ import annotations
import argparse, sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "post"))
import params as P
from _phase_plot import PhaseSpec, run


SPEC = PhaseSpec(
    label       = "v2 continuous-high landing",
    project_id  = "prj-121d08b0-626c-475b-8c53-9d2ab54765d7",
    out_dir     = REPO / "post" / "out" / "v2_continuous_high",
    phase_name  = "landing",
    alpha_b     = +8.0,
    theta_ht_b  = -6.0,
    T_b         = +12.0,
    velocity    = P.V_LANDING_M_S,
    rho         = P.RHO_LANDING_KG_M3,
    a_sound     = P.A_SOUND_LANDING_M_S,
    alpha_sweep_theta_ht = +20.0,
    gamma_deg   = P.DESCENT_ANGLE_DEG,
    # Inherit low-htail landing masks; refine when data lands.
    mask_alpha  = lambda a: a <= 8.0,
    mask_htail  = lambda h: (h >= 0.0) & (h <= 30.0),
    mask_thrust = lambda t: t >= 7.0,
    title       = "Tsangpo v2 LANDING (continuous flap, HIGH htail) — BO α=+8°, θ_ht=-6°, "
                  "T=+12, V=12.86 m/s, γ=-30°",
)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--refresh", action="store_true")
    run(SPEC, refresh=ap.parse_args().refresh)
