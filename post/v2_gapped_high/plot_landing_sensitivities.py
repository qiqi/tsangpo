"""v2 gapped high-htail landing sensitivities + trim."""
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
    label       = "v2 gap40-high landing",
    project_id  = "prj-1315636c-f6d9-4076-ba32-0ec82c272430",
    out_dir     = REPO / "post" / "out" / "v2_gapped_high",
    phase_name  = "landing",
    alpha_b     = 8.0,
    theta_ht_b  = -6.0,
    T_b         = 12.0,
    velocity    = P.V_LANDING_M_S,
    rho         = P.RHO_LANDING_KG_M3,
    a_sound     = P.A_SOUND_LANDING_M_S,
    gamma_deg   = -30.0,
    # Masks inherited from the gap40 low-htail siblings; refine post hoc.
    mask_alpha  = lambda a: a <= 20.0,
    mask_htail  = lambda h: h <= 20.0,
    mask_thrust = lambda t: t <= 25.0,
    title       = "Tsangpo v2 LANDING (gapped-flap, HIGH htail) — BO α=8.0°, θ_ht=-6.0°, T=12.0",
)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--refresh", action="store_true")
    run(SPEC, refresh=ap.parse_args().refresh)
