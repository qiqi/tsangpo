"""Kinematics for the Tsangpo 4-cylinder robot-arm topology.

Mirrors the math the UDD applies on the cloud side.  Post-processing /
rendering scripts use this module so every caller agrees on L1, L2,
the rotation sign convention, and the forward/inverse kinematics.

Sign convention: Flow360 R_y(+θ) right-hand rule.  Body axes per
params.py: +X aft, +Y starboard, +Z up.  body+x → world (cos θ, -sin θ).
"""
from __future__ import annotations
import importlib.util
import sys
from pathlib import Path

import numpy as np


class ArmKinematics:
    """4-link arm: shoulder + elbow + airframe-pitch + htail-pitch."""

    def __init__(self, L1: float, L2: float, MAC: float):
        self.L1 = L1
        self.L2 = L2
        self.MAC = MAC

    @staticmethod
    def Ry(theta, x, z):
        """R_y(+θ) in the (x, z) plane.  Vectorised over arrays."""
        c, s = np.cos(theta), np.sin(theta)
        return x * c + z * s, -x * s + z * c

    def cg_world(self, theta_1, theta_2):
        """Body CG world position from shoulder/elbow angles.
        Forward kinematics for the cyl_1 ∘ cyl_2 chain that the mesh applies.
        Generalised for L1 != L2."""
        cx, cz = self.Ry(theta_1, self.L1, 0.0)
        # Second link length is L2, not L1 (previously hardcoded -L1 — a
        # silent isosceles-arm assumption that would desync from the CFD
        # mesh if L1 != L2).
        rx, rz = self.Ry(theta_1 + theta_2, -self.L2, 0.0)
        return cx + rx, cz + rz

    def ik(self, x_cg, z_cg):
        """Arm angles (θ_1, θ_2) for a target CG.

        Closed form (acos / atan, not atan2) — matches the UDD's IK
        formulas exactly so post-processing and the CFD mesh stay in sync.

        Safeguards:
        - Generalised Law of Cosines for L1 != L2.
        - np.clip on the acos argument to absorb floating-point overshoot
          at the arm's reachability boundary (r = L1 + L2).
        - np.where on the atan denominators to avoid NaN at L1 == L2·cos θ₂
          (= straight arm) and at x_cg == 0 (= shoulder-directly-above-CG).
        - Quadrant caveat: standard atan is bounded to (-π/2, π/2).  For
          x_cg > 0 (the operational envelope of our Tsangpo arm), this is
          fine; if x_cg ever goes negative the result will be wrong by π.
        """
        L1, L2 = self.L1, self.L2
        r2 = x_cg * x_cg + z_cg * z_cg
        # Generalised Law of Cosines: r² = L1² + L2² − 2·L1·L2·cos θ₂
        cos_t2 = (L1 * L1 + L2 * L2 - r2) / (2 * L1 * L2)
        # Floating-point clipping (handles r infinitesimally above L1+L2)
        cos_t2 = np.clip(cos_t2, -1.0, 1.0)
        t2 = np.arccos(cos_t2)
        # Atan singularity guards: produce 0 where the denominator is 0
        # (the geometric singularity = straight arm or shoulder-above-CG).
        EPS = 1e-16
        den1 = L1 - L2 * np.cos(t2)
        den1_safe = np.where(den1 == 0, EPS, den1)
        x_safe = np.where(x_cg == 0, EPS, x_cg)
        t1 = (np.arctan(L2 * np.sin(t2) / den1_safe)
              - np.arctan(z_cg / x_safe))
        return t1, t2


def _build_canonical() -> ArmKinematics:
    """Load L1, L2, MAC from params + unsteady_setup so every caller agrees."""
    repo = Path(__file__).resolve().parent
    sys.path.insert(0, str(repo))
    import params as P
    spec = importlib.util.spec_from_file_location(
        "_unsteady_setup", repo / "flow360/unsteady_setup.py")
    U = importlib.util.module_from_spec(spec); spec.loader.exec_module(U)
    return ArmKinematics(L1=U.L1_M, L2=U.L2_M, MAC=P.WING_MAC_M)


arm = _build_canonical()
