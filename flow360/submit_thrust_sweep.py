"""
Actuator-disk thrust calibration sweep — 10 forks of the cruise parent,
each with a DIFFERENT commanded `force_per_area` on the 10 disks (no
UDD; thrust is hardcoded per fork).

Replaces the prior single-fork-UDD design (which Flow360 silently
rejected — no output files were produced for case-72dc517a).  Each
fork pins constant rotations (α_eff=+7°, θ_htail=0) and a constant
thrust multiplier, running `steps=20` so the fork gets ~10 new
physical steps to settle.

Run:
    TSANGPO_PARENT_CASE_ID=case-... python3 flow360/submit_thrust_sweep.py
"""
from __future__ import annotations

import os
import sys
from math import radians
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
print(f"Forking thrust sweep from {PARENT_CASE_ID} on {project.id} ({project.metadata.name})")

THETA_AC_RAD       = +radians(7.0)
THETA_HT_RAD       = 0.0
THRUST_MULTIPLIERS = (0.0, 0.25, 0.50, 0.75, 1.00, 1.25, 1.50, 2.00, 2.50, 3.00)
N_STEPS_TOTAL      = 20
STEP_SIZE_S        = 1.0

geo = project.geometry
geo.group_faces_by_tag("capsGroup")
main_wing_surf = geo["main_wing"]
vane_surf      = geo["vane"]
aft_flap_surf  = geo["aft_flap"]
htail_surf     = geo["htail"]
all_surfs = [main_wing_surf, vane_surf, aft_flap_surf, htail_surf]

FPA_CRUISE_PA        = P.T_CRUISE_PER_PROP_N / P.A_DISK_PER_PROP_M2
SWIRL_CRUISE         = 0.012 * FPA_CRUISE_PA
AC_ZONE_HEIGHT       = 1.25 * P.WING_SPAN_M
AC_ZONE_OUTER_RADIUS = 1.10 * (P.X_TAIL_DEFAULT_M + 1.5 * P.HTAIL_CHORD_M)
HTAIL_ZONE_HEIGHT    = 1.30 * P.HTAIL_SPAN_M
HTAIL_ZONE_RADIUS    = 1.50 * P.HTAIL_CHORD_M
PROP_REFINE_SPACING  = 0.05 * P.WING_MAC_M


def build_params(thrust_multiplier: float) -> fl.SimulationParams:
    farfield = fl.AutomatedFarfield()
    fpa_scaled   = FPA_CRUISE_PA * thrust_multiplier
    swirl_scaled = SWIRL_CRUISE * thrust_multiplier
    with fl.SI_unit_system:
        ac_pitch_cyl = fl.Cylinder(
            name="ac_pitch_zone",
            center=(0, 0, 0) * fl.u.m, axis=(0, 1, 0),
            height=AC_ZONE_HEIGHT * fl.u.m,
            outer_radius=AC_ZONE_OUTER_RADIUS * fl.u.m,
        )
        htail_pitch_cyl = fl.Cylinder(
            name="htail_pitch_zone",
            center=(P.X_TAIL_DEFAULT_M, 0,
                    P.Z_TAIL_LOW_CHORDS * P.WING_MAC_M) * fl.u.m,
            axis=(0, 1, 0),
            height=HTAIL_ZONE_HEIGHT * fl.u.m,
            outer_radius=HTAIL_ZONE_RADIUS * fl.u.m,
        )
        prop_cyls = []
        for side, side_sign in (("R", +1), ("L", -1)):
            for i, eta in enumerate(P.PROP_Y_NONDIM, start=1):
                y = side_sign * eta * P.WING_SEMI_SPAN_M
                prop_cyls.append(fl.Cylinder(
                    name=f"disk_{side}{i}",
                    center=(P.PROP_X_M, y, P.PROP_Z_M) * fl.u.m,
                    axis=(-1, 0, 0),
                    height=P.PROP_HEIGHT_M * fl.u.m,
                    outer_radius=P.PROP_RADIUS_M * fl.u.m,
                ))
        # ForcePerArea must be strictly positive; for the T=0 fork, use a
        # tiny non-zero value (0.01 % of cruise) so the AD model stays
        # active but contributes negligible thrust.
        fpa_eff   = max(fpa_scaled,   1e-4 * FPA_CRUISE_PA)
        swirl_eff = max(swirl_scaled, 1e-4 * SWIRL_CRUISE)
        ad_models = [
            fl.ActuatorDisk(
                name=cyl.name.replace("disk_", "prop_"),
                entities=cyl,
                force_per_area=fl.ForcePerArea(
                    radius=np.array([0.15 * P.PROP_RADIUS_M, P.PROP_RADIUS_M]) * fl.u.m,
                    thrust=np.array([fpa_eff, fpa_eff]) * fl.u.N / fl.u.m ** 2,
                    circumferential=np.array([swirl_eff, swirl_eff]) * fl.u.N / fl.u.m ** 2,
                ),
            )
            for cyl in prop_cyls
        ]
        ac_rotation = fl.Rotation(
            name="ac_pitch", volumes=[ac_pitch_cyl],
            spec=fl.AngleExpression(f"{THETA_AC_RAD:.10f}"),
        )
        htail_rotation = fl.Rotation(
            name="htail_pitch", volumes=[htail_pitch_cyl],
            spec=fl.AngleExpression(f"{THETA_HT_RAD:.10f}"),
            parent_volume=ac_pitch_cyl,
        )
        return fl.SimulationParams(
            meshing=fl.MeshingParams(
                volume_zones=[
                    farfield,
                    fl.RotationVolume(
                        name="ac_rotation", entities=ac_pitch_cyl,
                        enclosed_entities=[main_wing_surf, vane_surf, aft_flap_surf,
                                           htail_pitch_cyl],
                        spacing_axial=0.30 * fl.u.m,
                        spacing_radial=0.15 * fl.u.m,
                        spacing_circumferential=0.15 * fl.u.m,
                    ),
                    fl.RotationVolume(
                        name="htail_rotation", entities=htail_pitch_cyl,
                        enclosed_entities=[htail_surf],
                        spacing_axial=0.15 * fl.u.m,
                        spacing_radial=0.06 * fl.u.m,
                        spacing_circumferential=0.06 * fl.u.m,
                    ),
                ],
                refinements=[
                    fl.UniformRefinement(entities=prop_cyls,
                                         spacing=PROP_REFINE_SPACING * fl.u.m),
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
                moment_center=(0, 0, 0) * fl.u.m,
                moment_length=(P.WING_SEMI_SPAN_M, P.WING_MAC_M,
                               P.WING_SEMI_SPAN_M) * fl.u.m,
            ),
            operating_condition=fl.AerospaceCondition(
                velocity_magnitude=P.V_CRUISE_M_S * fl.u.m / fl.u.s,
                alpha=0 * fl.u.deg,
                thermal_state=fl.ThermalState.from_standard_atmosphere(
                    altitude=P.ALT_CRUISE_M * fl.u.m,
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
            time_stepping=fl.Unsteady(
                step_size=STEP_SIZE_S * fl.u.s,
                steps=N_STEPS_TOTAL,
                max_pseudo_steps=1000,
                CFL=fl.AdaptiveCFL(max=1e4, convergence_limiting_factor=0.25),
            ),
            outputs=[
                fl.SurfaceOutput(name="surface", entities=all_surfs,
                                 output_fields=["Cp", "Cf", "yPlus", "CfVec"]),
                fl.VolumeOutput(output_fields=["Mach", "qcriterion", "Cp"]),
            ],
        )


submitted = []
print(f"  α_eff (pinned) = 7.0°  θ_htail (pinned) = 0°")
print(f"  Cruise thrust per prop = {P.T_CRUISE_PER_PROP_N:.1f} N "
      f"(commanded FPA = {FPA_CRUISE_PA:.2f} Pa)")
for mult in THRUST_MULTIPLIERS:
    name = f"thrust_x{mult:.2f}".replace(".", "p")
    params = build_params(mult)
    case = project.run_case(
        params=params, name=name, run_async=True,
        fork_from=parent_case,
        tags=["SI", "alpha7", "htail0", "thrust_sweep",
              f"mult{mult:.2f}", "hardcoded_FPA_fork"],
        use_beta_mesher=True,
    )
    submitted.append((mult, case.id))
    print(f"  thrust × {mult:.2f}  (FPA = {FPA_CRUISE_PA*mult:.2f} Pa)  →  {case.id}")

print()
print("Submitted thrust-sweep forks:")
for m, cid in submitted:
    print(f"  ×{m:.2f}  {cid}")
