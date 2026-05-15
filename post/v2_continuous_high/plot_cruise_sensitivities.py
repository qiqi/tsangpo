"""v2 continuous high-htail cruise sensitivities + trim."""
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
    label       = "v2 continuous-high cruise",
    project_id  = "prj-21d5e737-a1d9-4dba-ad7c-2155e647ede1",
    out_dir     = REPO / "post" / "out" / "v2_continuous_high",
    phase_name  = "cruise",
    alpha_b     = +7.0,
    theta_ht_b  = 0.0,
    T_b         = +1.0,
    velocity    = P.V_CRUISE_M_S,
    rho         = P.RHO_CRUISE_KG_M3,
    gamma_deg   = 0.0,
    mask_alpha  = lambda a: a <= 9.0,
    mask_htail  = lambda h: np.ones_like(h, dtype=bool),
    mask_thrust = lambda t: np.ones_like(t, dtype=bool),
    title       = "Tsangpo v2 CRUISE (continuous flap, HIGH htail) — α=+7°, V=45.72 m/s, level",
)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--refresh", action="store_true")
    run(SPEC, refresh=ap.parse_args().refresh)
