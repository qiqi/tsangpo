"""
Submit the Tsangpo cruise case to Flow360.

Uploads the stowed-phase airframe (4 bodies: main_wing, vane, aft_flap,
htail) as a fresh geometry project, configures steady RANS at the cruise
operating point with 10 actuator-disk propellers along the LE, and wraps
the H-tail in a rotational zone whose angle will be driven by user-defined
dynamics in a follow-up case to find the trim point.

    python flow360/submit_cruise.py
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
import params as P

# Delayed import so local flow360/ dir doesn't shadow the SDK.
sys.path = [p for p in sys.path if str(REPO / "flow360") not in p]
import flow360 as fl

STOWED = REPO / "geometry" / "out" / "stowed"

# Reuse an existing Flow360 project by exporting TSANGPO_PROJECT_ID; otherwise
# upload the four stowed-phase STEPs into a fresh project.
project_id = os.environ.get("TSANGPO_PROJECT_ID")
if project_id:
    print(f"Reusing Flow360 project {project_id} …")
    project = fl.Project.from_cloud(project_id)
else:
    # Upload order fixes the body00001..body00004 mapping.
    STEPS = [STOWED / f"{n}.step" for n in ("main_wing", "vane", "aft_flap", "htail")]
    for s in STEPS:
        assert s.exists(), f"{s} not found; run serveCSM tsangpo.csm first"
    print("Uploading 4 per-body STEPs as a new Flow360 project …")
    project = fl.Project.from_geometry(
        [str(s) for s in STEPS],
        name="tsangpo_cruise",
        length_unit="ft",
        tags=["tsangpo", "cruise", "stowed"],
    )

geo = project.geometry
geo.group_faces_by_tag("groupByBodyId")
geo.rename_surfaces("body00001", "main_wing")
geo.rename_surfaces("body00002", "vane")
geo.rename_surfaces("body00003", "aft_flap")
geo.rename_surfaces("body00004", "htail")
main_wing_surf = geo["main_wing"]
vane_surf      = geo["vane"]
aft_flap_surf  = geo["aft_flap"]
htail_surf     = geo["htail"]
all_surfs = [main_wing_surf, vane_surf, aft_flap_surf, htail_surf]
print(f"  Surfaces: {[s.name for s in all_surfs]}")

# ── Actuator-disk loading (cruise; uniform across each disk) ───────────
FPA_CRUISE   = P.T_CRUISE_PER_PROP_LBF / P.A_DISK_PER_PROP_FT2
SWIRL_CRUISE = 0.012 * FPA_CRUISE
print(f"  Cruise thrust:  {P.T_CRUISE_PER_PROP_LBF:.2f} lbf/prop, "
      f"force per area = {FPA_CRUISE:.3f} psf")

# ── H-tail rotation zone (centered at htail quarter-chord) ─────────────
HTAIL_QC_X = P.X_TAIL_DEFAULT_FT          # despmtr X_tail in .csm; that IS the quarter-chord
HTAIL_QC_Z = P.Z_TAIL_LOW_CHORDS * P.WING_MAC_FT
HTAIL_ZONE_HEIGHT = 1.3 * P.HTAIL_SPAN_FT
HTAIL_ZONE_RADIUS = 1.5 * P.HTAIL_CHORD_FT

farfield = fl.AutomatedFarfield()

with fl.imperial_unit_system:

    # Cylinder shared by the mesher (RotationVolume) and the solver (Rotation
    # model). Centered on the H-tail quarter-chord; height & radius leave
    # clearance for the sliding-interface mesh on both sides of the surface.
    htail_rot_cylinder = fl.Cylinder(
        name="htail_pitch_zone",
        center=(HTAIL_QC_X, 0.0, HTAIL_QC_Z) * fl.u.ft,
        axis=(0, 1, 0),
        height=HTAIL_ZONE_HEIGHT * fl.u.ft,
        outer_radius=HTAIL_ZONE_RADIUS * fl.u.ft,
    )

    ad_models = []
    for side, side_sign in (("R", +1), ("L", -1)):
        for i, eta in enumerate(P.PROP_Y_NONDIM, start=1):
            y = side_sign * eta * P.WING_SEMI_SPAN_FT
            ad_models.append(fl.ActuatorDisk(
                name=f"prop_{side}{i}",
                entities=fl.Cylinder(
                    name=f"disk_{side}{i}",
                    center=(P.PROP_X_FT, y, P.PROP_Z_FT) * fl.u.ft,
                    axis=(-1, 0, 0),
                    height=(0.05 * P.PROP_RADIUS_FT) * fl.u.ft,
                    outer_radius=P.PROP_RADIUS_FT * fl.u.ft,
                ),
                force_per_area=fl.ForcePerArea(
                    radius=np.array([0.15 * P.PROP_RADIUS_FT, P.PROP_RADIUS_FT])
                           * fl.u.ft,
                    thrust=np.array([FPA_CRUISE, FPA_CRUISE]) * fl.u.lbf / fl.u.ft ** 2,
                    circumferential=np.array([SWIRL_CRUISE, SWIRL_CRUISE])
                                    * fl.u.lbf / fl.u.ft ** 2,
                ),
            ))

    htail_rotation = fl.Rotation(
        name="htail_pitch_for_trim",
        volumes=[htail_rot_cylinder],
        spec=fl.FromUserDefinedDynamics(),
    )

    params = fl.SimulationParams(
        meshing=fl.MeshingParams(
            volume_zones=[
                farfield,
                fl.RotationVolume(
                    name="htail_rotation_volume",
                    entities=htail_rot_cylinder,
                    enclosed_entities=[htail_surf],
                    spacing_axial=0.5 * fl.u.ft,
                    spacing_radial=0.2 * fl.u.ft,
                    spacing_circumferential=0.2 * fl.u.ft,
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
            alpha=P.ALPHA_CRUISE_DEG * fl.u.deg,
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
            htail_rotation,
            *ad_models,
        ],
        time_stepping=fl.Steady(
            max_steps=3000,
            CFL=fl.AdaptiveCFL(max=1e4, convergence_limiting_factor=0.25),
        ),
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

print("Submitting cruise case (mesh → solve) …")
case = project.run_case(
    params=params,
    name="cruise_v1",
    run_async=True,
    tags=["cruise", "stowed", "htail_rotational_zone"],
)
print(f"Case submitted: {case.id}")
print(f"Project:        {project.id}")
