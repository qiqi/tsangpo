"""
Minimal reproducer for the Flow360 actuator-disk delivered-vs-commanded
discrepancy.  Submits ONE case at the gap-low landing BO trim point with
the same `force_per_area` AD model we use across the campaign, then
prints the commanded vs delivered thrust.

Run:
    python3 submit_repro.py            # submits + prints case ID
    python3 submit_repro.py <case-id>  # skip submission, just measure
                                        # delivered force on an existing case

Geometry:
    tsangpo_gapped.csm               (this folder; gap-low planform)

Expected output (fine-mesh delivery, release-25.9):
    commanded thrust per disk  = 638.3 N      (730.31 N/m² × 0.874 m²)
    delivered thrust per disk  ≈ 415 N        (~ 65 % of commanded)
    delivered/commanded ratio  ≈ 0.65         (vs expected 1.0)

See FLOW360_AD_BUG_REPORT.md (paper/figures/) for the full report
including 12 production cases mapped vs mesh refinement and the 1.10×
camp split observed on coarse meshes.
"""
from __future__ import annotations
import argparse, sys, tempfile
from math import pi, radians
from pathlib import Path

import numpy as np
import flow360 as fl

HERE = Path(__file__).resolve().parent
CSM  = HERE / "tsangpo_gapped.csm"

# Trim point — gap-low landing, T_mult = 12  (commanded T/W ≈ 0.55).
ALPHA_DEG       = +8.0
THETA_HT_DEG    = -6.0
THRUST_MULT     = +12.0
V_INF_M_S       = 12.86
ALT_M           =  0.0

# Aircraft + atmosphere constants
WING_AREA_M2    = 15.33
WING_SPAN_M     = 11.07
WING_CHORD_M    = WING_AREA_M2 / WING_SPAN_M             # ≈ 1.385 m
WING_LE_X_M     = -0.5 * WING_CHORD_M
WING_Z_M        = +0.4 * WING_CHORD_M
HTAIL_CHORD_M   = WING_CHORD_M
HTAIL_SPAN_M    = 0.40 * WING_SPAN_M
X_TAIL_C4_M     = WING_LE_X_M + 4.0 * WING_CHORD_M + 0.25 * HTAIL_CHORD_M
Z_TAIL_LOW_M    = WING_Z_M
RHO_INF_KG_M3   = 0.849                                  # ISA 12 000 ft
A_INF_M_S       = 326.0
RHO_A2          = RHO_INF_KG_M3 * A_INF_M_S ** 2         # 90 250 N/m²

# 10 prop array
PROP_R_M        = 0.5335
PROP_R_INNER_M  = 0.15 * PROP_R_M                        # 0.080 m
PROP_HEIGHT_M   = 0.10 * WING_CHORD_M                    # 0.139 m  axial thickness
PROP_X_M        = WING_LE_X_M - 0.20 * WING_CHORD_M
PROP_Z_M        = WING_Z_M - 0.30 * WING_CHORD_M
PROP_Y_M        = tuple((0.1, 0.3, 0.5, 0.7, 0.9))       # nondim semi-span
WING_SEMI_SPAN  = WING_SPAN_M / 2

# AD model — uniform constant-pressure-jump.  fpa(N/m²) = T_mult * cruise pressure.
T_CRUISE_TOTAL_N      = 0.5 * RHO_INF_KG_M3 * 45.72**2 * WING_AREA_M2 * 0.04   # ≈ 543 N
T_CRUISE_PER_PROP_N   = T_CRUISE_TOTAL_N / 10                                   # ≈ 54.3 N
A_DISK_PER_PROP_M2    = pi * PROP_R_M ** 2                                      # ≈ 0.894 m²
FPA_CRUISE_PA         = T_CRUISE_PER_PROP_N / A_DISK_PER_PROP_M2                # ≈ 60.86 N/m²

# Annular area used in the AD `force_per_area` spec (radius ∈ [0.15R, R])
ANNULAR_AREA_M2 = pi * (PROP_R_M ** 2 - PROP_R_INNER_M ** 2)


def commanded_thrust_per_disk(t_mult: float) -> float:
    fpa = t_mult * FPA_CRUISE_PA                           # N/m²
    return fpa * ANNULAR_AREA_M2                           # N


def submit_one() -> str:
    print(f"Uploading {CSM.name} as a Flow360 project …")
    project = fl.Project.from_geometry(
        str(CSM), name="ad_repro", length_unit="m",
        tags=["ad_repro", "v2_gapped"],
    )
    print(f"  project: {project.id}")
    geo = project.geometry; geo.group_faces_by_tag("capsGroup")
    main = geo["main_wing"]; htail = geo["htail"]
    vane_l = geo["vane_left"]; vane_r = geo["vane_right"]
    flap_l = geo["aft_flap_left"]; flap_r = geo["aft_flap_right"]
    wing_surfs = [main, vane_l, vane_r, flap_l, flap_r]
    all_surfs  = wing_surfs + [htail]

    fpa = THRUST_MULT * FPA_CRUISE_PA
    swirl = 0.012 * fpa

    with fl.SI_unit_system:
        farfield = fl.AutomatedFarfield()
        ac_cyl = fl.Cylinder(name="ac_pitch_zone",
            center=(0, 0, 0) * fl.u.m, axis=(0, 1, 0),
            height=1.25 * WING_SPAN_M * fl.u.m,
            outer_radius=1.10 * (X_TAIL_C4_M + 1.5 * HTAIL_CHORD_M) * fl.u.m,
        )
        ht_cyl = fl.Cylinder(name="htail_pitch_zone",
            center=(X_TAIL_C4_M, 0, Z_TAIL_LOW_M) * fl.u.m, axis=(0, 1, 0),
            height=1.30 * HTAIL_SPAN_M * fl.u.m,
            outer_radius=1.50 * HTAIL_CHORD_M * fl.u.m,
        )
        prop_cyls = [
            fl.Cylinder(
                name=f"disk_{side}{i+1}",
                center=(PROP_X_M, ss * eta * WING_SEMI_SPAN, PROP_Z_M) * fl.u.m,
                axis=(1, 0, 0),
                height=PROP_HEIGHT_M * fl.u.m, outer_radius=PROP_R_M * fl.u.m,
            )
            for ss, side in ((+1, "R"), (-1, "L"))
            for i, eta in enumerate(PROP_Y_M)
        ]
        ad_models = [
            fl.ActuatorDisk(
                name=cyl.name.replace("disk_", "prop_"),
                entities=cyl,
                force_per_area=fl.ForcePerArea(
                    radius=np.array([PROP_R_INNER_M, PROP_R_M]) * fl.u.m,
                    thrust=np.array([fpa, fpa]) * fl.u.N / fl.u.m ** 2,
                    circumferential=np.array([swirl, swirl]) * fl.u.N / fl.u.m ** 2,
                ),
            ) for cyl in prop_cyls
        ]
        ac_rotation = fl.Rotation(name="ac_pitch", volumes=[ac_cyl],
                                   spec=fl.AngleExpression(f"{radians(ALPHA_DEG):.10f}"))
        ht_rotation = fl.Rotation(name="htail_pitch", volumes=[ht_cyl],
                                   spec=fl.AngleExpression(f"{radians(THETA_HT_DEG):.10f}"),
                                   parent_volume=ac_cyl)
        params = fl.SimulationParams(
            meshing=fl.MeshingParams(
                volume_zones=[
                    fl.AutomatedFarfield(),
                    fl.RotationVolume(name=ac_cyl.name, entities=ac_cyl,
                                       enclosed_entities=wing_surfs),
                    fl.RotationVolume(name=ht_cyl.name, entities=ht_cyl,
                                       parent_volume=ac_cyl, enclosed_entities=[htail]),
                ],
                refinements=[
                    fl.UniformRefinement(entities=prop_cyls,
                                          spacing=0.025 * WING_CHORD_M * fl.u.m),
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
                area=WING_AREA_M2 * fl.u.m ** 2,
                moment_center=(0, 0, 0) * fl.u.m,
                moment_length=(WING_SPAN_M/2, WING_CHORD_M, WING_SPAN_M/2) * fl.u.m,
            ),
            operating_condition=fl.AerospaceCondition(
                velocity_magnitude=V_INF_M_S * fl.u.m / fl.u.s,
                alpha=0 * fl.u.deg,
                thermal_state=fl.ThermalState.from_standard_atmosphere(altitude=ALT_M * fl.u.m),
            ),
            models=[
                fl.Fluid(
                    navier_stokes_solver=fl.NavierStokesSolver(
                        absolute_tolerance=1e-10,
                        linear_solver=fl.LinearSolver(max_iterations=35),
                        low_mach_preconditioner=True,
                    ),
                    turbulence_model_solver=fl.SpalartAllmaras(
                        absolute_tolerance=1e-8, rotation_correction=True),
                ),
                fl.Wall(name="aircraft", entities=all_surfs),
                fl.Freestream(name="freestream", entities=[farfield.farfield]),
                ac_rotation, ht_rotation, *ad_models,
            ],
            time_stepping=fl.Unsteady(
                step_size=1.0 * fl.u.s, steps=26, max_pseudo_steps=500,
                CFL=fl.AdaptiveCFL(max=1e4, convergence_limiting_factor=0.25),
            ),
            outputs=[
                fl.SurfaceOutput(name="surface", entities=all_surfs,
                                  output_fields=["Cp", "Cf", "yPlus", "CfVec"]),
                fl.VolumeOutput(output_fields=["Mach", "qcriterion", "Cp"]),
            ],
        )
    case = project.run_case(params=params, name="ad_repro_case",
                             run_async=True, use_beta_mesher=True,
                             tags=["ad_repro"])
    print(f"  case: {case.id}")
    return case.id


def measure_delivered(case_id: str):
    c = fl.Case.from_cloud(case_id=case_id)
    print(f"case: {c.name}  status={c.status}")
    if "COMPLETED" not in str(c.status):
        print("  case not COMPLETED yet — re-run when it is.")
        return
    ad = c.results.actuator_disks; ad.load_from_remote()
    av = ad.values
    F_solver_total = sum(float(np.array(av[f"Disk{i}_Force"])[-1])
                         for i in range(10))
    F_N_total = F_solver_total * RHO_A2
    F_N_per_disk = F_N_total / 10
    cmd_per_disk = commanded_thrust_per_disk(THRUST_MULT)
    print()
    print(f"  commanded thrust per disk = {cmd_per_disk:7.1f} N")
    print(f"                       total= {cmd_per_disk*10:7.1f} N")
    print(f"  delivered thrust per disk = {F_N_per_disk:7.1f} N")
    print(f"                       total= {F_N_total:7.1f} N")
    print(f"  delivered / commanded     = {F_N_total / (cmd_per_disk*10):.3f}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("case_id", nargs="?", default=None,
                    help="Existing case-id; if omitted, submit a fresh one.")
    a = ap.parse_args()
    cid = a.case_id or submit_one()
    print()
    measure_delivered(cid)
