"""
Fork the converged cruise case and run a UDD-driven trim search.

Three user-defined-dynamics controllers, each a slow proportional
integrator on a single state variable:

  • ac_alpha_trim   — rotates the aircraft pitch volume so that
                       wall C_L reaches W / (q·S) (level-flight lift).
  • htail_trim      — rotates the H-tail pitch volume so that the
                       total momentY (about the CG, = origin) → 0.
  • thrust_trim     — scales every actuator-disk thrust uniformly
                       (via actuatorDisk_<name>_thrustMultiplier) so
                       that the total forceX → 0.

Run:
    TSANGPO_PARENT_CASE_ID=case-... python flow360/submit_trim.py
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
import params as P

sys.path = [p for p in sys.path if str(REPO / "flow360") not in p]
import flow360 as fl

PARENT_CASE_ID = os.environ["TSANGPO_PARENT_CASE_ID"]
parent_case = fl.Case.from_cloud(PARENT_CASE_ID)
project = fl.Project.from_cloud(parent_case.project_id)
print(f"Forking from {PARENT_CASE_ID} on {project.id}")

# Re-attach the cap-Group named surfaces on the geometry.
geo = project.geometry
geo.group_faces_by_tag("capsGroup")
main_wing_surf = geo["main_wing"]
vane_surf      = geo["vane"]
aft_flap_surf  = geo["aft_flap"]
htail_surf     = geo["htail"]
all_surfs = [main_wing_surf, vane_surf, aft_flap_surf, htail_surf]

# Targets for the controllers.
q_inf       = 0.5 * P.RHO_12K_SLUG_FT3 * P.V_CRUISE_FT_S ** 2
CL_target   = P.W_GROSS_LBF / (q_inf * P.WING_AREA_FT2)
print(f"  q∞·S = {q_inf*P.WING_AREA_FT2:.0f} lbf,  CL_target = W/(q·S) = {CL_target:.4f}")

FPA_CRUISE   = P.T_CRUISE_PER_PROP_LBF / P.A_DISK_PER_PROP_FT2
SWIRL_CRUISE = 0.012 * FPA_CRUISE

AC_ZONE_HEIGHT       = 1.25 * P.WING_SPAN_FT
AC_ZONE_OUTER_RADIUS = 1.10 * (P.X_TAIL_DEFAULT_FT + 1.5 * P.HTAIL_CHORD_FT)
HTAIL_ZONE_HEIGHT    = 1.3  * P.HTAIL_SPAN_FT
HTAIL_ZONE_RADIUS    = 1.5  * P.HTAIL_CHORD_FT
PROP_REFINE_SPACING  = 0.05 * P.WING_MAC_FT

farfield = fl.AutomatedFarfield()

with fl.imperial_unit_system:

    # Same cylinders as submit_cruise.py — Flow360 matches by name into
    # the parent's existing volume mesh on fork.
    ac_pitch_cyl = fl.Cylinder(
        name="ac_pitch_zone",
        center=(0, 0, 0) * fl.u.ft,
        axis=(0, 1, 0),
        height=AC_ZONE_HEIGHT * fl.u.ft,
        outer_radius=AC_ZONE_OUTER_RADIUS * fl.u.ft,
    )
    htail_pitch_cyl = fl.Cylinder(
        name="htail_pitch_zone",
        center=(P.X_TAIL_DEFAULT_FT, 0.0,
                P.Z_TAIL_LOW_CHORDS * P.WING_MAC_FT) * fl.u.ft,
        axis=(0, 1, 0),
        height=HTAIL_ZONE_HEIGHT * fl.u.ft,
        outer_radius=HTAIL_ZONE_RADIUS * fl.u.ft,
    )

    prop_cyls = []
    for side, side_sign in (("R", +1), ("L", -1)):
        for i, eta in enumerate(P.PROP_Y_NONDIM, start=1):
            y = side_sign * eta * P.WING_SEMI_SPAN_FT
            prop_cyls.append(fl.Cylinder(
                name=f"disk_{side}{i}",
                center=(P.PROP_X_FT, y, P.PROP_Z_FT) * fl.u.ft,
                axis=(-1, 0, 0),
                height=P.PROP_HEIGHT_FT * fl.u.ft,
                outer_radius=P.PROP_RADIUS_FT * fl.u.ft,
            ))

    ad_models = [
        fl.ActuatorDisk(
            name=cyl.name.replace("disk_", "prop_"),
            entities=cyl,
            force_per_area=fl.ForcePerArea(
                radius=np.array([0.15 * P.PROP_RADIUS_FT, P.PROP_RADIUS_FT])
                       * fl.u.ft,
                thrust=np.array([FPA_CRUISE, FPA_CRUISE]) * fl.u.lbf / fl.u.ft ** 2,
                circumferential=np.array([SWIRL_CRUISE, SWIRL_CRUISE])
                                * fl.u.lbf / fl.u.ft ** 2,
            ),
        )
        for cyl in prop_cyls
    ]

    # Both rotation models now driven by UDD.
    ac_rotation = fl.Rotation(
        name="ac_pitch",
        volumes=[ac_pitch_cyl],
        spec=fl.FromUserDefinedDynamics(),
    )
    htail_rotation = fl.Rotation(
        name="htail_pitch",
        volumes=[htail_pitch_cyl],
        spec=fl.FromUserDefinedDynamics(),
        parent_volume=ac_pitch_cyl,
    )

    # ── UDD #1: aircraft pitch → CL = CL_target ────────────────────────
    # state[0] = theta_ac (rad).  Simple Euler integrator on (CL_target − CL).
    # +theta = aircraft nose up (LE of wing rises), which increases effective α.
    ac_alpha_udd = fl.UserDefinedDynamic(
        name="ac_alpha_trim",
        input_vars=["CL"],
        constants={"CL_target": float(CL_target), "gain": 4e-4},
        output_vars={"theta": "state[0];"},
        state_vars_initial_value=["0.0"],
        update_law=[
            "state[0] + timeStepSize * gain * (CL_target - CL);"
        ],
        output_target=ac_pitch_cyl,
    )

    # ── UDD #2: H-tail pitch → momentY = 0 ─────────────────────────────
    # state[0] = theta_htail (rad, relative to aircraft pitch).
    # +theta_htail = htail nose up → inverted-camber tail makes less
    # downforce → less nose-up moment about CG.
    htail_udd = fl.UserDefinedDynamic(
        name="htail_trim",
        input_vars=["momentY"],
        constants={"gain": 1e-3},
        output_vars={"theta": "state[0];"},
        state_vars_initial_value=["0.0"],
        update_law=[
            "state[0] + timeStepSize * gain * momentY;"
        ],
        output_target=htail_pitch_cyl,
    )

    # ── UDD #3: thrust scaling → forceX = 0 ─────────────────────────────
    # state[0] = uniform thrustMultiplier (starts at 1.0).
    # forceX > 0 → net aft force (drag > thrust) → mult should rise.
    thrust_outputs = {
        f"actuatorDisk_{cyl.name}_thrustMultiplier": "state[0];"
        for cyl in prop_cyls
    }
    thrust_udd = fl.UserDefinedDynamic(
        name="thrust_trim",
        input_vars=["forceX"],
        constants={"gain": 5e-2},
        output_vars=thrust_outputs,
        state_vars_initial_value=["1.0"],
        update_law=[
            "state[0] + timeStepSize * gain * forceX;"
        ],
    )

    params = fl.SimulationParams(
        meshing=fl.MeshingParams(
            volume_zones=[
                farfield,
                fl.RotationVolume(
                    name="ac_rotation",
                    entities=ac_pitch_cyl,
                    enclosed_entities=[main_wing_surf, vane_surf, aft_flap_surf,
                                       htail_surf, htail_pitch_cyl, *prop_cyls],
                    spacing_axial=1.0 * fl.u.ft,
                    spacing_radial=0.5 * fl.u.ft,
                    spacing_circumferential=0.5 * fl.u.ft,
                ),
                fl.RotationVolume(
                    name="htail_rotation",
                    entities=htail_pitch_cyl,
                    enclosed_entities=[htail_surf],
                    spacing_axial=0.5 * fl.u.ft,
                    spacing_radial=0.2 * fl.u.ft,
                    spacing_circumferential=0.2 * fl.u.ft,
                ),
            ],
            refinements=[
                fl.UniformRefinement(
                    entities=prop_cyls,
                    spacing=PROP_REFINE_SPACING * fl.u.ft,
                ),
            ],
            defaults=fl.MeshingDefaults(
                surface_max_edge_length=0.25 * fl.u.ft,
                curvature_resolution_angle=15 * fl.u.deg,
                boundary_layer_first_layer_thickness=5e-5 * fl.u.ft,
                boundary_layer_growth_rate=1.3,
            ),
            gap_treatment_strength=0.5,
        ),
        reference_geometry=fl.ReferenceGeometry(
            area=P.WING_AREA_FT2 * fl.u.ft ** 2,
            moment_center=(0, 0, 0) * fl.u.ft,
            moment_length=P.WING_MAC_FT * fl.u.ft,
        ),
        operating_condition=fl.AerospaceCondition(
            velocity_magnitude=P.V_CRUISE_FT_S * fl.u.ft / fl.u.s,
            alpha=0 * fl.u.deg,
            thermal_state=fl.ThermalState.from_standard_atmosphere(
                altitude=P.ALT_CRUISE_FT * fl.u.ft,
            ),
        ),
        models=[
            fl.Fluid(
                navier_stokes_solver=fl.NavierStokesSolver(
                    absolute_tolerance=1e-10,
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
            ac_rotation,
            htail_rotation,
            *ad_models,
        ],
        time_stepping=fl.Steady(
            max_steps=5000,
            CFL=fl.AdaptiveCFL(max=1e4, convergence_limiting_factor=0.25),
        ),
        user_defined_dynamics=[ac_alpha_udd, htail_udd, thrust_udd],
        outputs=[
            fl.SurfaceOutput(
                name="surface",
                entities=all_surfs,
                output_fields=["Cp", "Cf", "yPlus", "CfVec"],
            ),
            fl.VolumeOutput(
                output_fields=["Mach", "qcriterion", "Cp"],
            ),
        ],
    )

print("Forking with UDD trim controllers …")
draft = parent_case.fork(
    name="trim_v1",
    params=params,
    tags=["cruise", "trim_search", "UDD"],
)
case = draft.submit()
print(f"Forked case: {case.id}")
print(f"Project:     {project.id}")
