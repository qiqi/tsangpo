"""
flow360/unsteady_setup.py
─────────────────────────

Builds the Flow360 SimulationParams for the v3 phugoid unsteady-CFD-coupled
case.  Adds the robot-arm cylinders (cyl 1 shoulder + cyl 2 elbow) on top of
the existing cyl 3 (airframe pitch) + cyl 4 (htail) static-CFD topology, and
authors four lockstep User-Defined-Dynamics blocks (one per cylinder, since
Flow360 allows one output_target per UDD) that each integrate the same
longitudinal 3-DOF dynamics and drive their own cylinder's rotation angle.

TOPOLOGY (per the design dialog, "Topology X — 4 rotating cylinders"):

    static farfield
    └── cyl 1 (shoulder, rotates θ_link1 around its center = origin)
        └── cyl 2 (elbow, rotates θ_link2 rel. cyl 1; centered at (L1, 0, 0)
                    in cyl 1 frame)
            └── cyl 3 (airframe pitch, rotates θ_body−θ_link1−θ_link2 rel.
                       cyl 2; centered at (L1+L2, 0, 0) in cyl 1 frame at CSM
                       construction, which is the airframe CG location)
                └── cyl 4 (htail, rotates θ_ht_rel rel. cyl 3)

  All cylinder rotation axes are (0, +1, 0).  L1 = L2 = R_arm = 750·c_w
  (R_ARM_C below; was 250 in the first build).

INERTIAL FRAME = the CSM construction frame.  The freestream BC at the
farfield is a fixed velocity vector in this frame (uniform direction +x at
speed V_∞), per the user's assumption that alphaAngle and Mach are never
modified.  Airplane motion in the inertial frame happens via the cyl-1 and
cyl-2 rotations (the 2-link arm), so the CFD sees the airplane physically
translating through the freestream.  Forces from CFD are reported in cyl
1's rotating frame and rotated to inertial in the UDD update law.

STATE (6 dynamics + 2 IK-cache, all in radians/SI/inertial-frame):
    state[0] = x_cg      (m)
    state[1] = z_cg      (m)
    state[2] = Vx        (m/s, inertial)
    state[3] = Vz        (m/s, inertial)
    state[4] = theta_body (rad, absolute body pitch in inertial)
    state[5] = q         (rad/s, body pitch rate)
    state[6] = theta_link2 (rad, IK auxiliary)
    state[7] = theta_link1 (rad, IK auxiliary)

INITIAL CONDITIONS for the v3-cruise phugoid γ₀ = +10° kick:
    state[0] = -L = -750·c_w     (L-pose, Jacobian-safe)
    state[1] = -L = -750·c_w
    state[2] = V_∞·(1 − cos γ₀)  ≈ +0.69 m/s     (small +x drift)
    state[3] = V_∞·sin γ₀        ≈ +7.94 m/s     (climb in inertial)
    state[4] = α_BO + γ₀ (in rad) ≈ +0.272 rad   (body pitched up to maintain α)
    state[5] = 0
    state[6] = θ_link2_0 = π/2
    state[7] = θ_link1_0 = π/2

WHY THIS IS "OPTION A" PHYSICALLY CORRECT (cf. earlier dialog):
    The airframe physically translates through the freestream via the
    rotation chain.  V variation in the airspeed is captured by the CFD
    naturally (the moving mesh at the wall changes the relative flow).
    No V²-scaling kludge is needed.  Dimensional forces from CFD go
    directly into Newton's law.

USAGE:
    from flow360.unsteady_setup import build_unsteady_params
    params = build_unsteady_params(surfaces, phase='cruise', mode='warmup')
    # ... or mode='phugoid' to fork an unsteady run from the warmup case.

Author: Qiqi Wang (with Claude Opus 4.7)
"""
from __future__ import annotations

import sys
from math import radians
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path[:] = [p for p in sys.path if str(REPO / "flow360") not in p]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

import flow360 as fl
import params as P
import cfd_setup as C


# =============================================================================
#  1. Geometry constants for the robot arm
# =============================================================================

R_ARM_C  = 750.0                                          # link length in c_w units
                                                            # (3× original 250; workspace
                                                            # radius 2L = 1500 c_w ≈ 2078 m,
                                                            # 9× area for γ=10° kicks)
# Gaps between consecutive rotation-zone outer boundaries.  Sized so the
# surface-mesh resolution at each cylinder's outer boundary matches the
# local volume-mesh resolution driven by the next-inner refined surface
# (h_vol = min_y(0.1·|x-y| + surf(y)) ; match condition at cyl_N outer:
#  0.1·gap + surf_{N-1} ≈ surf_N).
#
# Sizes:
#   surf_cyl3 ≈ 0.11 c_w ; surf_cyl2 ≈ 11 c_w ; surf_cyl1 ≈ 21.7 c_w
#   gap_2_3 = (11 − 0.11)/0.1 ≈ 110 c_w
#   gap_1_2 = (21.7 − 11)/0.1 ≈ 110 c_w  (used to be 100 c_w; bumped for symmetry)
R_GAP_2_3_C = 110.0      # cyl_3 (around CG) <-> cyl_2 (around elbow)
R_GAP_1_2_C = 110.0      # cyl_2 (elbow)     <-> cyl_1 (around shoulder)
L1_M     = R_ARM_C * P.WING_MAC_M                          # ≈ 1038.2 m (link 1)
L2_M     = R_ARM_C * P.WING_MAC_M                          # ≈ 1038.2 m (link 2)
R_CYL3_C = 2.90                                            # airframe pitch cyl radius
R_CYL4_C = 0.85                                            # htail cyl radius (= 0.75 c TE
                                                            # offset from htail c/4 + 0.1 c gap)
R_CYL3_M = R_CYL3_C * P.WING_MAC_M                         # ≈ 4.01 m
R_CYL4_M = R_CYL4_C * P.WING_MAC_M                         # ≈ 1.18 m

# Outer cylinders: contain the inner one + above-specified gap.
R_CYL2_M = L2_M + R_CYL3_M + R_GAP_2_3_C * P.WING_MAC_M    # ≈ 1056.2 m
R_CYL1_M = L1_M + R_CYL2_M + R_GAP_1_2_C * P.WING_MAC_M    # ≈ 2232.8 m

# Cylinder heights (extruded length along the rotation axis y):
HEIGHT_CYL1_M = 2.0 * R_CYL1_M                             # generous
HEIGHT_CYL2_M = 2.0 * R_CYL2_M
HEIGHT_CYL3_M = 1.25 * P.WING_SPAN_M
HEIGHT_CYL4_M = 1.30 * P.HTAIL_SPAN_M


# =============================================================================
#  2. Per-phase trim points and initial conditions
# =============================================================================
# From geometry/airframe_v3.yaml refined-trim values.

PHASE_TRIM = {
    "cruise": dict(
        V_inf_m_s    = 80.0 / 1.94384449,    # 80 kt freestream — slower than the
                                              # airframe's V_init so the airplane
                                              # drifts mildly in −X (forward) over
                                              # the 60-s run and stays centered in
                                              # the (now 9×-area) workspace.
        V_init_m_s   = 100.0 / 1.94384449,   # 100 kt initial AIRSPEED of the
                                              # airframe relative to the freestream.
                                              # This is the "the airplane is flying
                                              # at 100 kt" spec; sets the kinetic
                                              # energy of the phugoid.
        altitude_m   = 3658.0,
        alpha_BO_deg = +5.57,
        theta_ht_deg = +1.5,                # 2° nose-up-bias from the
                                              # level-cruise trim of +4.88°.
                                              # Compensates for residual
                                              # nose-down tendency observed in
                                              # the earlier plunge runs;
                                              # tested in parallel with the
                                              # disk-thrust UDD patch.
        T_mult       = +1.35,
        gamma_trim_deg = 0.0,
    ),
    "takeoff": dict(
        V_inf_m_s    = 18.00,
        altitude_m   = 0.0,
        alpha_BO_deg = +11.31,
        theta_ht_deg = -3.21,
        T_mult       = +11.47,
        gamma_trim_deg = +30.0,
    ),
    "landing": dict(
        V_inf_m_s    = 12.86,
        altitude_m   = 0.0,
        alpha_BO_deg = +27.63,
        theta_ht_deg = -3.67,
        T_mult       = +14.17,
        gamma_trim_deg = -30.0,
    ),
}


def _ik_initial(x_cg: float, z_cg: float, L1: float, L2: float
                 ) -> tuple[float, float]:
    """Compute (θ_link1, θ_link2) for an initial-condition CG position
    under **Option E** geometry: cyl 2 center at (+L1, 0, 0) in cyl 1 frame,
    cyl 3 center at (−L2, 0, 0) in cyl 2 frame.

    Flow360 convention: axis (0, +1, 0), R(θ)·(1,0,0) = (cosθ, 0, −sinθ).

    Forward kinematics:
        cyl_3 in cyl_1 frame = (L1 − L2·cos θ_link2,  0,  L2·sin θ_link2)
        cyl_3 in world       = R(θ_link1) · (above)

    Inverse:
        r²            = x_cg² + z_cg²
        cos θ_link2   = (L1² + L2² − r²) / (2·L1·L2)        ← Option E sign
        θ_link2       = +acos(...)                            (elbow-up branch)
        θ_link1       = atan2(L2·sin θ_link2, L1 − L2·cos θ_link2) − atan2(z_cg, x_cg)

    Verification at trim L-pose (x_cg, z_cg) = (−L, −L) with L1 = L2 = L:
        cos θ_link2 = 0 → θ_link2 = π/2
        a = L1 − L2·cos(π/2) = L1 = L,   b = L2·sin(π/2) = L
        atan2(L, L) = π/4
        atan2(−L, −L) = −3π/4
        θ_link1 = π/4 − (−3π/4) = π   ✓
    """
    r2 = x_cg * x_cg + z_cg * z_cg
    cos_t2 = max(-1.0, min(1.0, (L1 * L1 + L2 * L2 - r2) / (2.0 * L1 * L2)))
    t2 = np.arccos(cos_t2)
    half = np.arctan2(L2 * np.sin(t2), L1 - L2 * np.cos(t2))
    psi  = np.arctan2(z_cg, x_cg)
    t1 = half - psi
    return float(t1), float(t2)


# =============================================================================
#  3. Master UDD construction (the heart of the file)
# =============================================================================

def _build_per_zone_udds(
    *,
    cyl_shoulder, cyl_elbow, cyl_airframe, cyl_htail,
    input_patches,          # list of wall surfaces for force integration
    phase: str,
    gamma_kick_deg: float = 10.0,
    initial_x_cg_m: float = None,
    initial_z_cg_m: float = None,
    use_analytic_omegaDot: bool = False,
    max_pseudo_steps: int = 200,    # accepted for signature compatibility; the
                                     # state update is gated at pseudoStep == 0
                                     # (see POSTMORTEM §4), NOT at the last step.
    diagnostics: bool = False,      # if True: repurpose state[8,9] to cache
                                     # M_about_CG_nd and Q_DOT_nd; expose all state
                                     # slots and these via custom output_vars so the
                                     # user log emits them per physical step.
):
    """Author 4 UDDs (one per rotating cylinder), each running an identical
    8-state forward-Euler integration of the longitudinal 6-DOF + 2 IK
    dynamics, and each publishing only the rotation angle for its own
    `output_target`.

    Why four UDDs, not one
    ----------------------
    The Flow360 solver's output-var registry is keyed by NUMERIC zone IDs
    (`zone_<id>_theta`) which are only known after meshing.  The supported
    pattern for setting a zone's rotation theta from a UDD is to set
    `output_target=<cylinder>` and use the alias key `"theta"` — the solver
    then resolves it to `zone_<target's id>_theta`.  Only one output_target
    is permitted per UDD instance.

    Because forward Euler is bit-deterministic and all four UDDs receive
    the same `forceX`, `forceZ`, `momentY` (same input_boundary_patches)
    plus identical constants and initial state, the four state vectors
    evolve in lockstep.  Each UDD publishes a different `theta` expression
    derived from its own (identical) state.

    `input_patches` is the list of airframe wall surfaces (wing main, vanes,
    flaps, htail, fuselage) whose pressure+viscous force will be summed and
    fed into each UDD as `forceX`, `forceZ`, `momentY`.
    """
    trim = PHASE_TRIM[phase]
    V_inf = trim["V_inf_m_s"]
    alpha_BO_rad = radians(trim["alpha_BO_deg"])
    gamma_rad    = radians(gamma_kick_deg)
    theta_ht_rad = radians(trim["theta_ht_deg"])

    # L-pose default initial position (Jacobian-safe).  (+L, −L) maps to
    # (θ_link1, θ_link2) = (+π/2, +π/2) via the Option E IK.
    if initial_x_cg_m is None:
        initial_x_cg_m = +L1_M
    if initial_z_cg_m is None:
        initial_z_cg_m = -L2_M

    # ---- Non-dim conversion -------------------------------------------------
    # Flow360 evaluates UDD update_law in INTERNAL non-dim units.  Empirically
    # confirmed by submitting case-6c0185ce: with `step_size = 0.05 s` the
    # JSON's `timeStepSize` was 16.298, a factor of ~326 = a_inf at altitude.
    # Forces and moments in the UDD are likewise non-dim (verified at trim:
    # forceZ ≈ 0.155 ≈ CL·qS/(rho_inf·a_inf²) ≈ 0.158).  We therefore work
    # entirely in solver non-dim, with L_ref = 1 m (Flow360 SI default).
    #
    #     t_nd      = t_si  × a_inf / L_ref      (L_ref = 1 m)
    #     v_nd      = v_si  / a_inf
    #     a_nd      = a_si  / a_inf²
    #     m_nd      = m_si  / (rho_inf · L_ref³) = m_si / rho_inf
    #     I_yy_nd   = I_si  / (rho_inf · L_ref⁵) = I_si / rho_inf
    #     g_nd      = g_si  / a_inf²
    #     F_nd, M_nd = (already non-dim from Flow360)
    #     x_nd      = x_si   (since L_ref = 1 m, numerically identical)
    # Single source of truth for the freestream reference (params.isa_atmosphere
    # matches Flow360 from_standard_atmosphere); never hard-code rho/a per altitude.
    rho_inf, a_inf = P.isa_atmosphere(trim["altitude_m"])
    inv_a_inf = 1.0 / a_inf

    # Initial airframe velocity in WORLD frame for the γ-kick (non-dim).
    # The airframe's AIRSPEED (= |V_airplane - V_air|) is V_init, distinct
    # from the freestream V_inf:
    #   V_apparent = V_init·(-cos γ, 0, +sin γ)   (in airplane-from-air frame:
    #                                              air comes from "forward and below"
    #                                              for γ>0 climbing)
    #   V_air      = (V_inf, 0, 0)                (freestream BC in world)
    #   V_airplane = V_air - V_apparent
    #              = (V_inf - V_init·cos γ, 0, +V_init·sin γ)
    # For V_inf < V_init·cos γ, V_airplane_x < 0 = airplane translating
    # FORWARD (in -X) in world — consistent with "the airplane is flying
    # at V_init while the freestream BC sits at a different speed."
    V_init = trim["V_init_m_s"]
    Vx_0_nd = (V_inf - V_init * np.cos(gamma_rad)) * inv_a_inf
    Vz_0_nd = (+V_init * np.sin(gamma_rad)) * inv_a_inf
    theta_body_0 = alpha_BO_rad + gamma_rad

    # Initial IK auxiliary slots — angles are unitless (radians).
    theta_link1_0, theta_link2_0 = _ik_initial(
        initial_x_cg_m, initial_z_cg_m, L1_M, L2_M)

    # ---- State-integration gate: pseudoStep == 0 (POSTMORTEM §4) -------------
    # Flow360 auto-inserts a `<udd>_previousTheta_recorder` that snapshots
    # theta at pseudoStep 0; rotateGrid() then rotates by theta - previousTheta.
    # Gating the update at the LAST pseudo step (an earlier version of this
    # file did) makes the recorder copy the already-advanced theta, so
    # deltaTheta == 0 and the mesh never rotates (wing CL frozen at trim).
    # At pseudoStep 0 the force registry still holds the PREVIOUS physical
    # step's converged values, so integrating there is correct, with one
    # physical step of lag that the lever arm below honours.

    # ---- Actuator-disk reaction thrust ---------------------------------------
    # The UDD's `forceX` input is a surface integral on `input_boundary_patches`
    # (airframe walls only) and does NOT include the actuator-disk volume body
    # force.  At v3 cruise BO trim the surface CFx ≈ +0.08 (drag-direction,
    # not zero) — confirming the surface integrate misses the ~1.1× commanded
    # thrust.  Without this correction the UDD-driven aircraft sees only drag,
    # decelerates by ~10 m/s over 10 s, loses lift, and plunges.
    #
    # 2026-05-21 FIX: the disks ROTATE WITH the airframe (they sit inside cyl_3
    # and auto-nest spatially under cyl_3's rotation).  So the disk-reaction
    # thrust on the airplane is BODY-FIXED in the nose direction (body -X),
    # not world-fixed in +X.  The previous code added +T to forceX (= +X world)
    # which is the wrong sign — it accelerated the airplane in the AFT (+X)
    # direction instead of the nose (-X) direction.
    #
    # Thrust on airplane in body frame:   (-T_disk_nd, 0, 0)
    # In world after R_y(+θ_body):
    #   T_x_world = -T_disk_nd · cos(θ_body)
    #   T_z_world = +T_disk_nd · sin(θ_body)
    # Disk-reaction moment about body CG (= world CG, since rotation is about y
    # which preserves the y-component of any (x, z)-plane cross product):
    #   M_y_disk_about_CG = (r_disk × F_disk)_y_body
    #                    = (prop_z_body)·F_x_body  -  (prop_x_body)·F_z_body
    #                    = (+PROP_Z_M)·(-T_disk_nd)  -  (PROP_X_M)·0
    #                    = -PROP_Z_M · T_disk_nd
    # (Constant in body frame, hence constant in world too.)
    T_disk_total_N = (P.N_PROPS * trim["T_mult"]
                       * P.T_CRUISE_PER_PROP_N)        # commanded thrust at this T_mult
    T_disk_nd      = T_disk_total_N / (rho_inf * a_inf * a_inf)  # solver non-dim

    constants = {
        # Geometry (L_ref = 1 m, so non-dim values are numerically same as SI).
        "L1":        L1_M,
        "L2":        L2_M,
        "two_L_sq":  2.0 * L1_M * L1_M,
        # Mass properties (non-dim).
        "m_nd":      1179.0 / rho_inf,
        "I_yy_nd":   3000.0 / rho_inf,
        "g_nd":      9.81 / (a_inf * a_inf),
        # Htail held at trim deflection (relative to body).
        "theta_ht_rel": theta_ht_rad,
        # Actuator-disk total reaction thrust magnitude (positive scalar).
        # Direction is BODY-FIXED (body -X = nose direction), so the world-
        # frame components depend on state[4] = θ_body.  See state[2]/[3]/[5]
        # update_law expressions below for the body-orientation-aware decomposition.
        "T_disk_nd": T_disk_nd,
        # Disk z-offset in body frame (= PROP_Z_M, the perpendicular distance
        # from the body CG to the actuator-disk centroid plane).  Used to
        # compute the constant disk-reaction moment about CG (= -prop_z · T).
        "prop_z_m":  P.PROP_Z_M,
        # moment_center coordinates in world (FIXED) — set to initial CG so
        # the lever arm starts at zero and the moment-translation cancellation
        # error is minimized.  Must match ReferenceGeometry.moment_center below.
        "mc_x":      initial_x_cg_m,
        "mc_z":      initial_z_cg_m,
        # Initial conditions (positions in m == non-dim with L_ref = 1).
        "x_cg_0":    initial_x_cg_m,
        "z_cg_0":    initial_z_cg_m,
        "Vx_0":      Vx_0_nd,
        "Vz_0":      Vz_0_nd,
        "theta_body_0":  theta_body_0,
        "theta_link1_0": theta_link1_0,
        "theta_link2_0": theta_link2_0,
    }

    # state[6] = theta_link2 (IK from x_cg, z_cg); state[7] = theta_link1
    # (references state[6]).  update_law is evaluated SEQUENTIALLY in slot
    # order, in place (POSTMORTEM §5.1): slot k sees the NEW value of state[j]
    # for j < k and the OLD value for j >= k.  So slots 6,7 read the freshly
    # updated state[0,1] of THIS step, while slots 0-5 (which reference only
    # higher slots) see OLD state[6,7] = the pose at which the forces being
    # integrated were evaluated.  Slot order is load-bearing: do not reorder.

    # FWD-IK lever-arm expressions for M_about_CG (temporal-consistency fix):
    # In update_law, references to state[6], state[7] resolve to OLD values
    # (= published at step N-1).  Forward IK on those recovers state[0,1]
    # from the END of step N-2 -- which is exactly the body position where
    # the wall forces (read at step N's pseudoStep==0 = end of step N-1's
    # converged values) were evaluated.  Using these for the lever arm makes
    # the moment-translation temporally consistent with the forces.
    X_CG_IK = ("(cos(state[7])*(L1 - L2*cos(state[6])) "
                "+ sin(state[7])*L2*sin(state[6]))")
    Z_CG_IK = ("(-sin(state[7])*(L1 - L2*cos(state[6])) "
                "+ cos(state[7])*L2*sin(state[6]))")
    state_vars_initial_value = [
        "x_cg_0",            # state[0]
        "z_cg_0",            # state[1]
        "Vx_0",              # state[2]
        "Vz_0",              # state[3]
        "theta_body_0",      # state[4]
        "0.0",               # state[5] = q
        "theta_link2_0",     # state[6]
        "theta_link1_0",     # state[7]
        # state[8..9] cache the analytic ω̇ for the two actively-rotating
        # cylinders with the largest r (the ones whose Euler-force
        # contribution is most consequential).  Flow360 enforces a hard
        # limit of ≤10 state components, so cyl_2 (elbow) ω̇ stays 0 — its
        # FD magnitude is ~3e-8 in non-dim units which is small enough to
        # accept.  cyl_4 (htail) is non-rotating in cruise.
        "0.0",               # state[8]  = ω̇_airframe_rel (cyl_3)
        "0.0",               # state[9]  = ω̇_shoulder    (cyl_1)
    ]

    # All expressions gated by `pseudoStep == 0` so we only advance state
    # once per physical step (Flow360 calls the UDD on every pseudo step).
    #
    # UNITS:
    #   Everything is in Flow360 internal non-dim (see "Non-dim conversion"
    #   above).  timeStepSize is non-dim; forceX/Y/Z and momentY are non-dim;
    #   positions are numerically equal to SI metres (L_ref = 1 m); velocities
    #   are SI/a_inf; mass and I_yy are SI/rho_inf; g is SI/a_inf².
    #
    # FRAME of CFD-reported forces / moments — **WORLD (inertial)**:
    #   Empirically verified against the warmup case at trim L-pose
    #   (case-6c0185ce step 20):
    #     forceX_log  =  0.0408
    #     forceZ_log  =  0.1549
    #     momentY_log = -67.71
    #     state[0]   = +346.07,  state[1] = -346.07
    #     M_about_CG = momentY + state[0]·forceZ - state[1]·forceX
    #                = -67.71 + 346·0.1549 - (-346)·0.0408
    #                = -67.71 + 53.60 + 14.12 ≈ 0   ✓ (trim)
    #   With the WORLD-frame interpretation, the dynamics integrate trivially
    #   in world coordinates — no cyl-frame rotation of forces is needed.
    #   state[6] and state[7] are output-only: pure functions of state[0],
    #   state[1] used to drive the cylinder rotations that physically move
    #   the airframe through the (fixed) freestream.

    # ---- Analytic ω̇ expressions (defined here so update_law can cache them
    # in state[8..10]; same definitions are also referenced by per-cyl
    # output_vars further below).  All use forceX/Z/momentY which is only
    # allowed inside update_law, hence the caching pattern.
    AX_EXPR    = "(forceX/m_nd)"
    AZ_EXPR    = "(forceZ/m_nd - g_nd)"
    N_EXPR     = "(state[0]*state[2] + state[1]*state[3])"
    A_EXPR     = "(state[0]*state[3] - state[1]*state[2])"
    R2_EXPR    = "(state[0]*state[0] + state[1]*state[1])"
    NDOT_EXPR  = (f"(state[2]*state[2] + state[3]*state[3] "
                   f"+ state[0]*{AX_EXPR} + state[1]*{AZ_EXPR})")
    ADOT_EXPR  = f"(state[0]*{AZ_EXPR} - state[1]*{AX_EXPR})"
    OMEGA_ELBOW_LOCAL = ("((state[0]*state[2] + state[1]*state[3]) "
                          "/ (L1*L2*sin(state[6])))")
    OMEGA_DOT_ELBOW = (f"({NDOT_EXPR} / (L1*L2*sin(state[6])) "
                        f"- {OMEGA_ELBOW_LOCAL} * {OMEGA_ELBOW_LOCAL} "
                        f"* cos(state[6])/sin(state[6]))")
    OMEGA_DOT_SHOULDER = (f"(-0.5*{OMEGA_DOT_ELBOW} "
                          f"- {ADOT_EXPR}/{R2_EXPR} "
                          f"+ 2.0*{A_EXPR}*{N_EXPR}/({R2_EXPR}*{R2_EXPR}))")

    update_law = [
        # state[0]: x_cg += Vx · dt          (m, m/s → all non-dim)
        "if (pseudoStep == 0) state[0] + state[2] * timeStepSize; else state[0];",
        # state[1]: z_cg += Vz · dt
        "if (pseudoStep == 0) state[1] + state[3] * timeStepSize; else state[1];",
        # state[2]: Vx += ((Fx_surface + T_x_world) / m_nd) · dt
        # T_x_world = -T_disk_nd · cos(state[4])  -- thrust in body -X direction
        # (nose), which in world frame at body pitch θ has x-component -T·cos θ.
        # The CFD-reported `forceX` is the airframe-wall surface integral
        # only and does NOT include the disk reaction; we add it here.
        "if (pseudoStep == 0) state[2] + "
        "( (forceX - T_disk_nd * cos(state[4])) / m_nd ) * timeStepSize; else state[2];",
        # state[3]: Vz += ((Fz_surface + T_z_world) / m_nd - g_nd) · dt
        # T_z_world = +T_disk_nd · sin(state[4])  -- z-component of body-fixed
        # thrust as the body pitches.  At trim θ_body ≈ +5.57° this is ~+0.10·T
        # (small but non-zero; matters during phugoid excursions where θ can
        # reach ±30°).
        "if (pseudoStep == 0) state[3] + "
        "( (forceZ + T_disk_nd * sin(state[4])) / m_nd - g_nd ) * timeStepSize; else state[3];",
        # state[4]: theta_body += q · dt
        "if (pseudoStep == 0) state[4] + state[5] * timeStepSize; else state[4];",
        # state[5]: q += (M_y_about_CG / I_yy_nd) · dt
        # Moment translation (world frame) of WALL-SURFACE forces+moments:
        #   r_CG  = (state[0], 0, state[1])      (moves with airframe)
        #   r_mc  = (mc_x, 0, mc_z)               (FIXED in world)
        # M_wall_about_CG = momentY + (r_mc − r_CG) × F
        # (cross)_y       = (mc_z − state[1])·forceX − (mc_x − state[0])·forceZ
        # We set mc = initial CG = (+L1, 0, −L2) so the lever arm starts
        # at zero (eliminates catastrophic cancellation when subtracting
        # large M_origin terms) and only grows with airframe displacement.
        #
        # PLUS the disk-reaction moment about CG (which is NOT in the wall
        # forces — disk is a volume body force, not a surface):
        #   M_disk_about_CG = -prop_z_m · T_disk_nd  (constant, since
        #     rotation axis = +y preserves the y-moment of any disk-attached
        #     force pair).  Disk in body frame at (PROP_X_M, 0, PROP_Z_M)
        #     with force (-T, 0, 0); (r × F)_y = z·F_x - x·F_z =
        #     PROP_Z_M·(-T) - PROP_X_M·0 = -PROP_Z_M·T.
        f"if (pseudoStep == 0) state[5] + "
        f"( ( momentY + (mc_z - {Z_CG_IK}) * forceX - (mc_x - {X_CG_IK}) * forceZ "
        f"    - prop_z_m * T_disk_nd ) "
        f"/ I_yy_nd ) * timeStepSize; else state[5];",
        # state[6]: theta_link2 (IK output) = acos((2L² − r²) / 2L²)
        "if (pseudoStep == 0) "
        "acos( ( two_L_sq - state[0] * state[0] - state[1] * state[1] ) / two_L_sq ); "
        "else state[6];",
        # state[7]: theta_link1 (IK output)
        # = atan(L2·sin θ_link2 / (L1 − L2·cos θ_link2)) − atan(z / x)
        # atan2 is not supported by the UDD expression evaluator; atan(y/x)
        # is exact here because both denominators stay strictly positive in
        # the trim-near pose: L1 − L2·cos(π/2) = L1 > 0; state[0] starts at
        # +L > 0 and phugoid excursions keep it positive.
        "if (pseudoStep == 0) "
        "atan( L2 * sin(state[6]) / (L1 - L2 * cos(state[6])) ) "
        "- atan(state[1] / state[0]); "
        "else state[7];",
        # state[8]: ω̇_airframe_rel = q̇ - 0.5·ω̇_elbow + Ȧ/r² - 2·A·N/r⁴
        # Derived by substituting ω̇_shoulder = -0.5·ω̇_elbow - Ȧ/r² + 2·A·N/r⁴
        # into ω̇_airframe_rel = q̇ - ω̇_shoulder - ω̇_elbow.
        # Variables (all in update_law, where forces are allowed):
        #   q̇  = (momentY + (mc_z − state[1])·forceX − (mc_x − state[0])·forceZ
        #         - prop_z_m·T_disk_nd) / I_yy_nd
        #   ax = (forceX - T_disk_nd·cos(state[4])) / m_nd      (BODY-fixed thrust)
        #   az = (forceZ + T_disk_nd·sin(state[4])) / m_nd − g_nd
        #   ω̇_elbow = (Ṅ - ω_elbow²·L²·cos θ₂) / (L²·sin θ₂),
        #     where Ṅ = Vx² + Vz² + x·ax + z·az
        #   Ȧ = x·az − z·ax,  A = x·Vz − z·Vx,  N = x·Vx + z·Vz, r² = x² + z²
        f"if (pseudoStep == 0) "
        f"((momentY + (mc_z - {Z_CG_IK})*forceX - (mc_x - {X_CG_IK})*forceZ "
        f"  - prop_z_m * T_disk_nd)/I_yy_nd "
        " - 0.5*((state[2]*state[2] + state[3]*state[3] "
        "         + state[0]*((forceX - T_disk_nd*cos(state[4]))/m_nd) "
        "         + state[1]*((forceZ + T_disk_nd*sin(state[4]))/m_nd - g_nd)) "
        "        /(L1*L2*sin(state[6])) "
        "        - ((state[0]*state[2] + state[1]*state[3])/(L1*L2*sin(state[6]))) "
        "         *((state[0]*state[2] + state[1]*state[3])/(L1*L2*sin(state[6]))) "
        "         *cos(state[6])/sin(state[6])) "
        " + (state[0]*((forceZ + T_disk_nd*sin(state[4]))/m_nd - g_nd) "
        "    - state[1]*((forceX - T_disk_nd*cos(state[4]))/m_nd)) "
        "  /(state[0]*state[0] + state[1]*state[1]) "
        " - 2.0*(state[0]*state[3] - state[1]*state[2]) "
        "       *(state[0]*state[2] + state[1]*state[3]) "
        "  /((state[0]*state[0] + state[1]*state[1]) "
        "    *(state[0]*state[0] + state[1]*state[1]))); "
        "else state[8];",
        # state[9]: ω̇_shoulder = -0.5·ω̇_elbow - Ȧ/r² + 2·A·N/r⁴
        # (closed-form analytic; uses forces, so must be cached here).
        f"if (pseudoStep == 0) {OMEGA_DOT_SHOULDER}; "
        "else state[9];",
    ]

    if diagnostics:
        # Diagnostic mode: repurpose state[8] and state[9] to cache the
        # in-UDD computed M_about_CG_nd and Q_DOT_nd so we can see exactly
        # what the UDD integrates per step (vs externally reconstructed).
        # use_analytic_omegaDot is forced off below so these slots are free.
        M_CG_EXPR = (f"(momentY + (mc_z - {Z_CG_IK})*forceX "
                      f"- (mc_x - {X_CG_IK})*forceZ "
                      f"- prop_z_m*T_disk_nd)")
        update_law[8] = (f"if (pseudoStep == 0) {M_CG_EXPR}; "
                          "else state[8];")
        update_law[9] = (f"if (pseudoStep == 0) ({M_CG_EXPR}) / I_yy_nd; "
                          "else state[9];")
        use_analytic_omegaDot = False

    # Per-cylinder published angles AND rates.
    #
    # Each rotating cylinder needs BOTH theta and omega (and omegaDot) for
    # the Flow360 solver to set the rigid-body wall BC correctly.  Without
    # omega, the wall velocity defaults to zero — the mesh teleports to
    # the new angle each step but the CFD never sees the body MOVING
    # through the freestream, so the apparent γ-correction is lost.
    #
    # Analytic ω derivations (the IK gives state[6], state[7] as closed-
    # form functions of state[0..3] + L1=L2=L, so their derivatives are
    # also closed-form):
    #
    #   ω_elbow      = (x·Vx + z·Vz) / (L1·L2·sin(state[6]))
    #   ω_shoulder   = -ω_elbow/2  -  (x·Vz - z·Vx) / r²
    #   ω_airframe_r = state[5] - ω_elbow/2  +  (x·Vz - z·Vx) / r²
    #                = state[5] - ω_shoulder - ω_elbow   (Δ from absolute pitch
    #                  rate minus parent-chain rates)
    #   ω_htail_r    = 0   (theta_ht_rel constant)
    #
    # where x = state[0], z = state[1], Vx = state[2], Vz = state[3],
    # r² = state[0]² + state[1]².  All quantities are in Flow360 internal
    # non-dim (timeStepSize is non-dim, so ω is rad / non-dim-time, which
    # is what the solver expects).
    #
    # omegaDot (angular acceleration) is set to 0 as a first cut.  For
    # phugoid-rate evolution at our dt, omegaDot is ~10⁻⁷ rad/(nd-time)²
    # — negligible vs the omega we're putting in, but if the solver uses
    # it for a 2nd-order moving-mesh scheme, we may want to derive it.

    # ---- ω expressions (already validated to give CFD agreement vs simplistic) -----
    # Common subexpressions re-used across UDDs.
    OMEGA_ELBOW = ("(state[0]*state[2] + state[1]*state[3]) "
                    "/ (L1*L2*sin(state[6]))")
    PSI_DOT     = ("(state[0]*state[3] - state[1]*state[2]) "
                    "/ (state[0]*state[0] + state[1]*state[1])")

    omega_shoulder   = f"-0.5 * {OMEGA_ELBOW} - {PSI_DOT}"
    omega_elbow      = OMEGA_ELBOW
    omega_airframe_r = f"state[5] - 0.5 * {OMEGA_ELBOW} + {PSI_DOT}"
    omega_htail_r    = "0.0"

    # ---- ω̇ analytic expressions -------------------------------------------------
    # Derived by differentiating ω w.r.t. time using the dynamics state.
    # Let:
    #   N   = x·Vx + z·Vz       (numerator of ω_elbow)
    #   A   = x·Vz − z·Vx       (numerator of ψ̇ for shoulder)
    #   r²  = x² + z²
    #   Ṅ   = Vx² + Vz² + x·ax + z·az      with ax = forceX/m_nd,
    #                                            az = forceZ/m_nd − g_nd
    #   Ȧ   = x·az − z·ax
    # Then:
    #   ω̇_elbow      = Ṅ/(L²·sin θ₂)   −  ω_elbow² · cot θ₂
    #   ω̇_shoulder   = −½·ω̇_elbow      −  Ȧ/r²  +  2·A·N/r⁴
    #   ω̇_airframe_r = q̇ − ω̇_shoulder − ω̇_elbow,
    #                  q̇ = (momentY + (mc_z − z)·forceX − (mc_x − x)·forceZ) / I_yy
    #   ω̇_htail_r    = 0
    # (AX_EXPR, AZ_EXPR, N_EXPR, A_EXPR, R2_EXPR, NDOT_EXPR, ADOT_EXPR,
    #  OMEGA_DOT_ELBOW, OMEGA_DOT_SHOULDER are defined earlier — before
    #  update_law — so they can be cached in state[9], state[10].)

    # Publish ω̇ for each rotating cylinder from its state-slot cache.
    # Inlining the analytic expression in output_vars is rejected by Flow360
    # ("Use of forceX requires a proper source"); update_law is allowed to
    # use forces, so update_law populates state[8..10] and output_vars
    # references those slots.  cyl_4 (htail) is non-rotating in cruise so
    # its ω̇ is genuinely 0.
    if use_analytic_omegaDot:
        omegaDot_shoulder   = "state[9]"
        omegaDot_elbow      = "0.0"         # cyl_2 ω̇ uncached (10-slot cap)
        omegaDot_airframe_r = "state[8]"
        omegaDot_htail_r    = "0.0"
    else:
        omegaDot_shoulder   = "0.0"
        omegaDot_elbow      = "0.0"
        omegaDot_airframe_r = "0.0"
        omegaDot_htail_r    = "0.0"

    per_cyl_specs = [
        ("shoulder", cyl_shoulder, "state[7]",
            omega_shoulder, omegaDot_shoulder),
        ("elbow",    cyl_elbow,    "state[6]",
            omega_elbow, omegaDot_elbow),
        ("airframe", cyl_airframe, "state[4] - state[7] - state[6]",
            omega_airframe_r, omegaDot_airframe_r),
        ("htail",    cyl_htail,    "theta_ht_rel",
            omega_htail_r, omegaDot_htail_r),
    ]

    common = dict(
        input_vars      = ["forceX", "forceZ", "momentY"],
        constants       = constants,
        state_vars_initial_value = state_vars_initial_value,
        update_law      = update_law,
        input_boundary_patches = input_patches,
    )

    # Diagnostic channels via omegaDot.  Solver rejects custom output_var
    # names (only the whitelist: theta/omega/omegaDot/alphaAngle/...), and
    # only omegaDot is safe to hijack (theta and omega drive the cylinder).
    # We allocate one diagnostic per UDD:
    #   shoulder.omegaDot  ← state[4]  (theta_body, sanity)
    #   elbow.omegaDot     ← state[5]  (q, body pitch rate)
    #   airframe.omegaDot  ← state[8]  (M_about_CG_nd, repurposed cache)
    #   htail.omegaDot     ← state[9]  (Q_DOT_nd, repurposed cache)
    diag_omegaDot = {}
    if diagnostics:
        diag_omegaDot = {
            "shoulder": "state[4]",
            "elbow":    "state[5]",
            "airframe": "state[8]",
            "htail":    "state[9]",
        }

    return [
        fl.UserDefinedDynamic(
            name        = f"phugoid_6DOF_{phase}_{tag}",
            output_vars = {
                "theta":    theta_expr,
                "omega":    omega_expr,
                "omegaDot": diag_omegaDot.get(tag, omegaDot_expr),
            },
            output_target = cyl,
            **common,
        )
        for tag, cyl, theta_expr, omega_expr, omegaDot_expr in per_cyl_specs
    ]


# =============================================================================
#  4. SimulationParams builder
# =============================================================================

def build_unsteady_params(
    surfaces,                       # SurfaceBundle from cfd_setup
    *,
    phase:           str = "cruise",
    mode:            str = "phugoid",       # "warmup" or "phugoid"
    n_physical_steps: int = 1500,
    timestep_size_s:  float = 0.01,
    max_pseudo_steps: int = 100,
    gamma_kick_deg:   float = 10.0,
    use_analytic_omegaDot: bool = False,
    diagnostics: bool = False,               # phugoid mode only: instrument the 4 UDDs
                                              # with custom diag_* output_vars exposing all
                                              # state slots + M_about_CG_nd + Q_DOT_nd.
    slice_frequency: int = -1,               # y=0 SliceOutput write frequency in physical
                                              # steps (-1 = last step only; 1 = every step
                                              # for a flow-field animation).
):
    """Build SimulationParams for the v3 phugoid 4-cylinder unsteady setup.

    `mode='warmup'`  → all cylinders frozen at the trim L-pose (no UDD).
                        Run to steady-state convergence on the multi-zone
                        mesh; fork the phugoid case from this.

    `mode='phugoid'` → four lockstep UDDs active.  Initial state seeded
                        with the γ_0 kick.  Fork this from the warmup case so
                        the flow starts converged.

    The prescribed-kinematics diagnostic modes used while chasing the two
    root-cause bugs (pitch_ramp, pitch_step*) were removed on 2026-10-30;
    their conclusions are recorded in PHUGOID_UDD_POSTMORTEM.md §2 and
    their case IDs in post/v3/unsteady_project_ids.json.  Recover the code
    from git history (commit d1dacc8) if ever needed.
    """
    if phase not in PHASE_TRIM:
        raise ValueError(f"phase must be one of {list(PHASE_TRIM)}, got {phase!r}")
    if mode not in ("warmup", "phugoid"):
        raise ValueError(f"mode must be 'warmup' or 'phugoid', got {mode!r}")
    trim = PHASE_TRIM[phase]

    # Unpack surfaces.
    if isinstance(surfaces, C.SurfaceBundle):
        wing_system_surfs = surfaces.wing_system_surfs
        htail_surf = surfaces.htail_surf
        all_surfs  = surfaces.all_surfs
    else:
        main_wing_surf, vane_surf, aft_flap_surf, htail_surf = surfaces
        wing_system_surfs = [main_wing_surf, vane_surf, aft_flap_surf]
        all_surfs = list(surfaces)

    # Initial conditions (L-pose for IK conditioning).  (+L, −L) → (π/2, π/2).
    initial_x_cg_m = +L1_M
    initial_z_cg_m = -L2_M
    theta_link1_0, theta_link2_0 = _ik_initial(
        initial_x_cg_m, initial_z_cg_m, L1_M, L2_M)
    alpha_BO_rad   = radians(trim["alpha_BO_deg"])
    gamma_trim_rad = radians(trim["gamma_trim_deg"])
    theta_body_trim = alpha_BO_rad + gamma_trim_rad
    theta_ht_rel    = radians(trim["theta_ht_deg"])

    # Thrust setting (held fixed in this implementation).
    mult   = max(trim["T_mult"], 1e-4)
    fpa    = mult * C.FPA_CRUISE_PA
    swirl  = mult * C.SWIRL_CRUISE
    V_inf  = trim["V_inf_m_s"]

    farfield = fl.AutomatedFarfield()

    with fl.SI_unit_system:
        # ---- Define the four nested cylinders ----------------------------
        # Option E layout — all cyl centers in CSM coords (= the frame
        # tsangpo_v3.csm is constructed in, with airframe at origin):
        #   cyl 1 center  (0,    0, 0)   ← coincides with airframe CG in CSM
        #   cyl 2 center  (+L1,  0, 0)   ← elbow point, offset +L1 in +x
        #   cyl 3 center  (0,    0, 0)   ← airframe location (= cyl 1 center
        #                                  geometrically, but very different
        #                                  radii — cyl 3 is small)
        #   cyl 4 center  htail's CSM position (unchanged from steady CFD)
        #
        # cyl 3 offset = (−L2, 0, 0) from cyl 2's center (in cyl 2 local frame)
        # — that's what makes Option E reduce to Option E and not the +L2
        # variant.  At trim L-pose, θ_link2 = π/2 swings this -L2 offset
        # toward +z (in cyl 2's frame), then cyl 1 rotates by π to land
        # the airframe at world (−L, 0, −L).

        cyl_shoulder = fl.Cylinder(
            name="cyl1_shoulder",
            center=(0, 0, 0) * fl.u.m,
            axis=(0, 1, 0),
            height=HEIGHT_CYL1_M * fl.u.m,
            outer_radius=R_CYL1_M * fl.u.m,
        )

        cyl_elbow = fl.Cylinder(
            name="cyl2_elbow",
            center=(L1_M, 0, 0) * fl.u.m,
            axis=(0, 1, 0),
            height=HEIGHT_CYL2_M * fl.u.m,
            outer_radius=R_CYL2_M * fl.u.m,
        )

        cyl_airframe = fl.Cylinder(
            name="cyl3_airframe_pitch",
            center=(0, 0, 0) * fl.u.m,
            axis=(0, 1, 0),
            height=HEIGHT_CYL3_M * fl.u.m,
            outer_radius=R_CYL3_M * fl.u.m,
        )

        cyl_htail = fl.Cylinder(
            name="cyl4_htail_pitch",
            center=(P.X_TAIL_LE_V3_M + 0.25 * P.HTAIL_CHORD_M,
                     0,
                     P.Z_TAIL_LOW_M) * fl.u.m,
            axis=(0, 1, 0),
            height=HEIGHT_CYL4_M * fl.u.m,
            outer_radius=R_CYL4_M * fl.u.m,
        )

        # ---- Prop / actuator disks (CSM-aligned, no L1+L2 offset) -------
        prop_cyls = [
            fl.Cylinder(
                name=f"disk_{side}{i + 1}",
                center=(P.PROP_X_M,
                         side_sign * eta * P.WING_SEMI_SPAN_M,
                         P.PROP_Z_M) * fl.u.m,
                axis=(-1, 0, 0),
                height=P.PROP_HEIGHT_M * fl.u.m,
                outer_radius=P.PROP_RADIUS_M * fl.u.m,
            )
            for side, side_sign in (("R", +1), ("L", -1))
            for i, eta in enumerate(P.PROP_Y_NONDIM)
        ]
        # Counter-rotating prop pairs: starboard (R) and port (L) sides spin
        # in opposite senses so their reaction torques on the airframe cancel.
        ad_models = [
            fl.ActuatorDisk(
                name=cyl.name.replace("disk_", "prop_"),
                entities=cyl,
                force_per_area=fl.ForcePerArea(
                    radius=np.array([0.15 * P.PROP_RADIUS_M, P.PROP_RADIUS_M]) * fl.u.m,
                    thrust=np.array([fpa, fpa]) * fl.u.N / fl.u.m ** 2,
                    circumferential=(
                        (+1 if "_R" in cyl.name else -1)
                        * np.array([swirl, swirl]) * fl.u.N / fl.u.m ** 2
                    ),
                ),
            )
            for cyl in prop_cyls
        ]

        # ---- Rotation specs ---------------------------------------------
        # Airframe-cylinder angle that puts the body at trim pitch in the L-pose.
        cyl3_theta_trim = theta_body_trim - theta_link1_0 - theta_link2_0
        if mode == "warmup":
            # All cylinders frozen at the trim L-pose; no UDD.  This gives a
            # static-flow steady-state case on the multi-zone mesh that we
            # can fork the phugoid case from.
            rot_shoulder = fl.Rotation(
                name="cyl1_rotation", volumes=[cyl_shoulder],
                spec=fl.AngleExpression(f"{theta_link1_0:.10f}"),
            )
            rot_elbow = fl.Rotation(
                name="cyl2_rotation", volumes=[cyl_elbow],
                spec=fl.AngleExpression(f"{theta_link2_0:.10f}"),
                parent_volume=cyl_shoulder,
            )
            rot_airframe = fl.Rotation(
                name="cyl3_rotation", volumes=[cyl_airframe],
                spec=fl.AngleExpression(f"{cyl3_theta_trim:.10f}"),
                parent_volume=cyl_elbow,
            )
            rot_htail = fl.Rotation(
                name="cyl4_rotation", volumes=[cyl_htail],
                spec=fl.AngleExpression(f"{theta_ht_rel:.10f}"),
                parent_volume=cyl_airframe,
            )
            udds = None
        else:  # mode == "phugoid": all four cylinders driven by the UDDs
            rot_shoulder = fl.Rotation(
                name="cyl1_rotation", volumes=[cyl_shoulder],
                spec=fl.FromUserDefinedDynamics(),
            )
            rot_elbow = fl.Rotation(
                name="cyl2_rotation", volumes=[cyl_elbow],
                spec=fl.FromUserDefinedDynamics(),
                parent_volume=cyl_shoulder,
            )
            rot_airframe = fl.Rotation(
                name="cyl3_rotation", volumes=[cyl_airframe],
                spec=fl.FromUserDefinedDynamics(),
                parent_volume=cyl_elbow,
            )
            rot_htail = fl.Rotation(
                name="cyl4_rotation", volumes=[cyl_htail],
                spec=fl.FromUserDefinedDynamics(),
                parent_volume=cyl_airframe,
            )
            udds = _build_per_zone_udds(
                cyl_shoulder=cyl_shoulder,
                cyl_elbow=cyl_elbow,
                cyl_airframe=cyl_airframe,
                cyl_htail=cyl_htail,
                input_patches=all_surfs,
                phase=phase,
                gamma_kick_deg=gamma_kick_deg,
                use_analytic_omegaDot=use_analytic_omegaDot,
                max_pseudo_steps=max_pseudo_steps,
                diagnostics=diagnostics,
            )

        # ---- SimulationParams assembly ----------------------------------
        return fl.SimulationParams(
            meshing=fl.MeshingParams(
                volume_zones=[
                    farfield,
                    fl.RotationVolume(
                        name="cyl1_zone", entities=cyl_shoulder,
                        enclosed_entities=[cyl_elbow],
                        # 3× coarsening vs original 10 m to keep cell count
                        # constant after R_arm 3× → R_cyl1 3× (volume 27×,
                        # cells = volume / h³ = 27 / 27 = 1×).
                        spacing_axial=30.0 * fl.u.m,
                        spacing_radial=30.0 * fl.u.m,
                        spacing_circumferential=30.0 * fl.u.m,
                    ),
                    fl.RotationVolume(
                        name="cyl2_zone", entities=cyl_elbow,
                        enclosed_entities=[cyl_airframe],
                        spacing_axial=15.0 * fl.u.m,
                        spacing_radial=15.0 * fl.u.m,
                        spacing_circumferential=15.0 * fl.u.m,
                    ),
                    fl.RotationVolume(
                        name="cyl3_zone", entities=cyl_airframe,
                        # prop_cyls enclosed so the actuator-disk slipstream
                        # rotates with the airframe (otherwise disk axes stay
                        # horizontal as the body pitches, blowing past the wing).
                        enclosed_entities=(
                            wing_system_surfs + [cyl_htail] + prop_cyls
                        ),
                        spacing_axial=0.30 * fl.u.m,
                        spacing_radial=0.15 * fl.u.m,
                        spacing_circumferential=0.15 * fl.u.m,
                    ),
                    fl.RotationVolume(
                        name="cyl4_zone", entities=cyl_htail,
                        enclosed_entities=[htail_surf],
                        spacing_axial=0.15 * fl.u.m,
                        spacing_radial=0.06 * fl.u.m,
                        spacing_circumferential=0.06 * fl.u.m,
                    ),
                ],
                refinements=[
                    fl.UniformRefinement(entities=prop_cyls,
                                         spacing=C.PROP_REFINE_M * fl.u.m),
                ],
                defaults=fl.MeshingDefaults(
                    surface_max_edge_length=0.075 * fl.u.m,
                    curvature_resolution_angle=15 * fl.u.deg,
                    boundary_layer_first_layer_thickness=7.62e-6 * fl.u.m,
                    boundary_layer_growth_rate=1.3,
                ),
                gap_treatment_strength=0.5,
            ),
            reference_geometry=fl.ReferenceGeometry(
                area=P.WING_AREA_M2 * fl.u.m ** 2,
                # moment_center at the airframe's INITIAL CG position
                # (+L1, 0, -L2).  This minimizes the lever arm in the per-panel
                # moment integration: at t=0 the lever is exactly zero, so
                # momentY ≈ M_about_CG directly — no catastrophic cancellation.
                # As the airframe moves over the phugoid, the lever grows only
                # with the (small) displacement, not with the absolute CG
                # position.  Without this fix, force noise × huge-lever swamps
                # the small M_CG signal and the dynamics integrates garbage.
                moment_center=(+L1_M, 0, -L2_M) * fl.u.m,
                moment_length=(P.WING_SPAN_M, P.WING_MAC_M,
                               P.WING_SPAN_M) * fl.u.m,
            ),
            operating_condition=fl.AerospaceCondition(
                velocity_magnitude=V_inf * fl.u.m / fl.u.s,
                alpha=0 * fl.u.deg,                  # FIXED at 0 (per Topology X)
                thermal_state=fl.ThermalState.from_standard_atmosphere(
                    altitude=trim["altitude_m"] * fl.u.m,
                ),
            ),
            models=[
                fl.Fluid(
                    navier_stokes_solver=fl.NavierStokesSolver(
                        # 1e-8 (relaxed from 1e-10) — tighter is over-spec for the
                        # phugoid dynamic-coupling case where each physical step is
                        # a small perturbation off the previous step's converged
                        # state.  Per the warmup analysis, residuals start each
                        # physical step at ~1e-10 (carried from the previous step)
                        # and we only need them to stay <1e-8 after the per-step
                        # perturbation.  Saves ~50-100 wasted pseudo iters/step.
                        absolute_tolerance=1e-8,
                        linear_solver=fl.LinearSolver(max_iterations=35),
                        low_mach_preconditioner=True,
                    ),
                    turbulence_model_solver=fl.SpalartAllmaras(
                        absolute_tolerance=1e-8,
                        rotation_correction=True,
                    ),
                ),
                fl.Wall(name="aircraft", entities=all_surfs),
                fl.Freestream(name="freestream", entities=[farfield.farfield]),
                rot_shoulder, rot_elbow, rot_airframe, rot_htail,
                *ad_models,
            ],
            time_stepping=fl.Unsteady(
                step_size=timestep_size_s * fl.u.s,
                steps=n_physical_steps,
                max_pseudo_steps=max_pseudo_steps,
                CFL=fl.AdaptiveCFL(max=1e4, convergence_limiting_factor=0.25),
            ),
            user_defined_dynamics=udds,
            outputs=[
                fl.SurfaceOutput(name="surface", entities=all_surfs,
                                 output_fields=["Cp", "Cf", "yPlus", "CfVec"]),
                fl.VolumeOutput(output_fields=["Mach", "qcriterion", "Cp"]),
                # Mandated y=0 symmetry-plane slice (CLAUDE.md convention).
                # The body only translates/pitches in the x-z plane, so y=0
                # always cuts the centerline as the airframe oscillates.
                # frequency=slice_frequency physical steps (default -1 = last
                # step only; set =1 for a per-step flow-field animation).
                fl.SliceOutput(
                    name="y0_slice",
                    entities=[fl.Slice(name="y=0",
                                       normal=(0.0, 1.0, 0.0),
                                       origin=(0.0, 0.0, 0.0) * fl.u.m)],
                    output_fields=["velocity", "Mach", "Cp", "primitiveVars"],
                    frequency=slice_frequency,
                ),
            ],
        )
