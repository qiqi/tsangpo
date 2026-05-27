"""v2 gapped high-htail takeoff sensitivities + trim."""
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
    label       = "v2 gap40-high takeoff",
    project_id  = "prj-fadaacba-ac09-4b2f-a0f2-364c46f5cb02",
    out_dir     = REPO / "post" / "out" / "v2_gapped_high",
    phase_name  = "takeoff",
    alpha_b     = 8.0,
    theta_ht_b  = -5.0,
    T_b         = 16.0,
    velocity    = P.V_TAKEOFF_M_S,
    rho         = P.RHO_TAKEOFF_KG_M3,
    a_sound     = P.A_SOUND_TAKEOFF_M_S,
    gamma_deg   = 30.0,
    # Masks inherited from the gap40 low-htail siblings; refine post hoc.
    mask_alpha  = lambda a: a <= 11.0,
    mask_htail  = lambda h: np.ones_like(h, dtype=bool),
    mask_thrust = lambda t: t <= 22.0,
    title       = "Tsangpo v2 TAKEOFF (gapped-flap, HIGH htail) — BO α=8.0°, θ_ht=-5.0°, T=16.0",
)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--refresh", action="store_true")
    run(SPEC, refresh=ap.parse_args().refresh)
