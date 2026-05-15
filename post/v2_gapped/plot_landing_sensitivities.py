"""v2 gapped-flap landing sensitivities + trim.  Uses post/_phase_plot.run()."""
from __future__ import annotations
import argparse, sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "post"))
import params as P
from _phase_plot import PhaseSpec, run


SPEC = PhaseSpec(
    label       = "v2 gap40 landing",
    project_id  = "prj-0cd29981-d281-442a-984f-06562abc1f39",
    out_dir     = REPO / "post" / "out" / "v2_gapped",
    phase_name  = "landing",
    alpha_b     = +8.0,
    theta_ht_b  = -6.0,
    T_b         = +12.0,
    velocity    = P.V_LANDING_M_S,
    rho         = P.RHO_LANDING_KG_M3,
    gamma_deg   = P.DESCENT_ANGLE_DEG,
    # gap40 landing: α unstalled out to +20°.  Htail is unstalled across
    # the entire negative range up through +20°; above +20° the htail
    # stalls (over-the-top).
    mask_alpha  = lambda a: a <= 20.0,
    mask_htail  = lambda h: h <= 20.0,
    mask_thrust = lambda t: t <= 25.0,
    title       = "Tsangpo v2 LANDING (gapped-flap) — BO α=+8°, θ_ht=-6°, T=+12, "
                  "V=12.86 m/s, γ=-30°",
)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--refresh", action="store_true")
    run(SPEC, refresh=ap.parse_args().refresh)
