"""v2 gapped high-htail cruise sensitivities + trim."""
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
    label       = "v2 gap40-high cruise",
    project_id  = "prj-d5d18139-3f53-4bf5-b224-7d3a5f3feb2a",
    out_dir     = REPO / "post" / "out" / "v2_gapped_high",
    phase_name  = "cruise",
    alpha_b     = 7.0,
    theta_ht_b  = 0.0,
    T_b         = 1.0,
    velocity    = P.V_CRUISE_M_S,
    rho         = P.RHO_CRUISE_KG_M3,
    gamma_deg   = 0.0,
    # Masks inherited from the gap40 low-htail siblings; refine post hoc.
    mask_alpha  = lambda a: a <= 9.0,
    mask_htail  = lambda h: np.ones_like(h, dtype=bool),
    mask_thrust = lambda t: np.ones_like(t, dtype=bool),
    title       = "Tsangpo v2 CRUISE (gapped-flap, HIGH htail) — BO α=7.0°, θ_ht=0.0°, T=1.0",
)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--refresh", action="store_true")
    run(SPEC, refresh=ap.parse_args().refresh)
