"""
Minimal reproducer for the Flow360 actuator-disk delivered-vs-commanded
discrepancy.  Submits TWO cases at the SAME trim point with the SAME
constant-pressure-jump AD spec — only the prop-cylinder UniformRefinement
spacing differs:

    CASE A  PROP_REFINE = 0.05 c_wing  (≈ 0.0692 m → octree-cast 0.0625 m,
                                         ~17 cells across disk diameter)
    CASE B  PROP_REFINE = 0.025 c_wing (≈ 0.0346 m → octree-cast 0.03125 m,
                                         ~34 cells across diameter)

Observed (release-25.9, our gap-low / cont-low campaign):
    CASE A  delivered ≈ 1.10 × commanded   (e.g. cont-low takeoff)
    CASE B  delivered ≈ 0.65 × commanded   (same trim, same geometry, finer mesh)

i.e. the integrated `Disk_i_Force` swings 1.10x → 0.65x with mesh
refinement, even though the constant-pressure-jump model should
integrate to (jump × annular area) exactly.

Run:
    # download the geometry once (see README for the URL)
    wget -O tsangpo.csm \\
        https://raw.githubusercontent.com/qiqi/tsangpo/main/geometry/tsangpo.csm

    pip install flow360 numpy

    python3 submit_repro.py                          # submits BOTH cases
    python3 submit_repro.py <case-a> <case-b>        # skip submission,
                                                     # just measure two
                                                     # existing cases
"""
from __future__ import annotations
import argparse, sys
from math import pi, radians
from pathlib import Path

import numpy as np
import flow360 as fl

HERE = Path(__file__).resolve().parent
CSM  = HERE / "tsangpo.csm"

# Trim point — continuous-flap (cont-low) takeoff BO.
ALPHA_DEG     = +8.0
THETA_HT_DEG  = -5.0
THRUST_MULT   = +16.0
V_INF_M_S     = 18.0
ALT_M         =  0.0

# Aircraft + atmosphere constants
WING_AREA_M2  = 15.33
WING_SPAN_M   = 11.07
WING_CHORD_M  = WING_AREA_M2 / WING_SPAN_M
WING_LE_X_M   = -0.5 * WING_CHORD_M
WING_Z_M      = +0.4 * WING_CHORD_M
HTAIL_CHORD_M = WING_CHORD_M
HTAIL_SPAN_M  = 0.40 * WING_SPAN_M
X_TAIL_C4_M   = WING_LE_X_M + 4.0 * WING_CHORD_M + 0.25 * HTAIL_CHORD_M
Z_TAIL_LOW_M  = WING_Z_M
RHO_INF       = 0.849
A_INF         = 326.0
RHO_A2        = RHO_INF * A_INF ** 2                         # 90,250 N/m²

# 10-prop array
PROP_R_M       = 0.5335
PROP_R_INNER_M = 0.15 * PROP_R_M                             # 0.080 m
PROP_HEIGHT_M  = 0.10 * WING_CHORD_M                         # 0.139 m
PROP_X_M       = WING_LE_X_M - 0.20 * WING_CHORD_M
PROP_Z_M       = WING_Z_M - 0.30 * WING_CHORD_M
PROP_ETA       = (0.1, 0.3, 0.5, 0.7, 0.9)
WING_SEMI_SPAN = WING_SPAN_M / 2

# Cruise reference for the actuator-disk pressure scaling
T_CRUISE_TOTAL_N    = 0.5 * RHO_INF * 45.72 ** 2 * WING_AREA_M2 * 0.04
T_CRUISE_PER_PROP_N = T_CRUISE_TOTAL_N / 10
A_DISK_PER_PROP_M2  = pi * PROP_R_M ** 2
FPA_CRUISE_PA       = T_CRUISE_PER_PROP_N / A_DISK_PER_PROP_M2     # 60.86 N/m²
ANNULAR_AREA_M2     = pi * (PROP_R_M ** 2 - PROP_R_INNER_M ** 2)   # 0.874 m²

COMMANDED_PER_DISK_N  = THRUST_MULT * FPA_CRUISE_PA * ANNULAR_AREA_M2
COMMANDED_TOTAL_N     = 10 * COMMANDED_PER_DISK_N

# Two mesh refinements
PROP_REFINE_COARSE = 0.05  * WING_CHORD_M    # 0.0692 m (octree 0.0625 m)
PROP_REFINE_FINE   = 0.025 * WING_CHORD_M    # 0.0346 m (octree 0.03125 m)


def _build_params(project, prop_refine_m: float):
    geo = project.geometry; geo.group_faces_by_tag("capsGroup")
    main  = geo["main_wing"]; htail = geo["htail"]
    vane  = geo["vane"];      flap  = geo["aft_flap"]
    wing_surfs = [main, vane, flap]
    all_surfs  = wing_surfs + [htail]
    fpa   = THRUST_MULT * FPA_CRUISE_PA
    swirl = 0.012 * fpa

    with fl.SI_unit_system:
        farfield = fl.AutomatedFarfield()
        ac_cyl = fl.Cylinder(name="ac_pitch_zone",
            center=(0, 0, 0) * fl.u.m, axis=(0, 1, 0),
            height=1.25 * WING_SPAN_M * fl.u.m,
            outer_radius=1.10 * (X_TAIL_C4_M + 1.5 * HTAIL_CHORD_M) * fl.u.m)
        ht_cyl = fl.Cylinder(name="htail_pitch_zone",
            center=(X_TAIL_C4_M, 0, Z_TAIL_LOW_M) * fl.u.m, axis=(0, 1, 0),
            height=1.30 * HTAIL_SPAN_M * fl.u.m,
            outer_radius=1.50 * HTAIL_CHORD_M * fl.u.m)
        prop_cyls = [
            fl.Cylinder(name=f"disk_{side}{i+1}",
                center=(PROP_X_M, ss * eta * WING_SEMI_SPAN, PROP_Z_M) * fl.u.m,
                axis=(1, 0, 0),
                height=PROP_HEIGHT_M * fl.u.m, outer_radius=PROP_R_M * fl.u.m)
            for ss, side in ((+1, "R"), (-1, "L"))
            for i, eta in enumerate(PROP_ETA)
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
                    farfield,
                    fl.RotationVolume(name=ac_cyl.name, entities=ac_cyl,
                                       enclosed_entities=wing_surfs),
                    fl.RotationVolume(name=ht_cyl.name, entities=ht_cyl,
                                       parent_volume=ac_cyl, enclosed_entities=[htail]),
                ],
                refinements=[
                    fl.UniformRefinement(entities=prop_cyls,
                                          spacing=prop_refine_m * fl.u.m),
                ],
                defaults=fl.MeshingDefaults(
                    surface_max_edge_length=0.075 * fl.u.m,
                    curvature_resolution_angle=15 * fl.u.deg,
                    boundary_layer_first_layer_thickness=7.62e-6 * fl.u.m,
                    boundary_layer_growth_rate=1.3),
                gap_treatment_strength=0.5),
            reference_geometry=fl.ReferenceGeometry(
                area=WING_AREA_M2 * fl.u.m ** 2,
                moment_center=(0, 0, 0) * fl.u.m,
                moment_length=(WING_SPAN_M/2, WING_CHORD_M, WING_SPAN_M/2) * fl.u.m),
            operating_condition=fl.AerospaceCondition(
                velocity_magnitude=V_INF_M_S * fl.u.m / fl.u.s,
                alpha=0 * fl.u.deg,
                thermal_state=fl.ThermalState.from_standard_atmosphere(altitude=ALT_M * fl.u.m)),
            models=[
                fl.Fluid(
                    navier_stokes_solver=fl.NavierStokesSolver(
                        absolute_tolerance=1e-10,
                        linear_solver=fl.LinearSolver(max_iterations=35),
                        low_mach_preconditioner=True),
                    turbulence_model_solver=fl.SpalartAllmaras(
                        absolute_tolerance=1e-8, rotation_correction=True)),
                fl.Wall(name="aircraft", entities=all_surfs),
                fl.Freestream(name="freestream", entities=[farfield.farfield]),
                ac_rotation, ht_rotation, *ad_models,
            ],
            time_stepping=fl.Unsteady(
                step_size=1.0 * fl.u.s, steps=26, max_pseudo_steps=500,
                CFL=fl.AdaptiveCFL(max=1e4, convergence_limiting_factor=0.25)),
            outputs=[
                fl.SurfaceOutput(name="surface", entities=all_surfs,
                                  output_fields=["Cp", "Cf", "yPlus", "CfVec"]),
                fl.VolumeOutput(output_fields=["Mach", "qcriterion", "Cp"])])
    return params


def submit_pair() -> tuple[str, str]:
    if not CSM.exists():
        sys.exit(
            f"missing geometry: {CSM}\n"
            "Download once:\n"
            "  wget -O tsangpo.csm \\\n"
            "    https://raw.githubusercontent.com/qiqi/tsangpo/main/geometry/tsangpo.csm")
    print(f"Uploading {CSM.name} (this happens twice — Flow360 deduplicates the geometry).")

    proj_coarse = fl.Project.from_geometry(str(CSM), name="ad_repro_coarse",
                                            length_unit="m",
                                            tags=["ad_repro", "coarse_mesh"])
    print(f"  coarse project: {proj_coarse.id}")
    case_coarse = proj_coarse.run_case(
        params=_build_params(proj_coarse, PROP_REFINE_COARSE),
        name="ad_repro_coarse_case", run_async=True,
        use_beta_mesher=True, tags=["ad_repro", "coarse_mesh"])
    print(f"  coarse case   : {case_coarse.id}")

    proj_fine = fl.Project.from_geometry(str(CSM), name="ad_repro_fine",
                                          length_unit="m",
                                          tags=["ad_repro", "fine_mesh"])
    print(f"  fine project: {proj_fine.id}")
    case_fine = proj_fine.run_case(
        params=_build_params(proj_fine, PROP_REFINE_FINE),
        name="ad_repro_fine_case", run_async=True,
        use_beta_mesher=True, tags=["ad_repro", "fine_mesh"])
    print(f"  fine case   : {case_fine.id}")

    return case_coarse.id, case_fine.id


def measure(case_id: str, label: str):
    c = fl.Case.from_cloud(case_id=case_id)
    print(f"\n  --- {label}: {c.name}  status={c.status} ---")
    if "COMPLETED" not in str(c.status):
        print("    not COMPLETED yet — re-run when it is.")
        return None
    ad = c.results.actuator_disks; ad.load_from_remote()
    av = ad.values
    F_solver_total = sum(float(np.array(av[f"Disk{i}_Force"])[-1]) for i in range(10))
    F_total_N = F_solver_total * RHO_A2
    ratio = F_total_N / COMMANDED_TOTAL_N
    print(f"    commanded total = {COMMANDED_TOTAL_N:7.1f} N")
    print(f"    delivered total = {F_total_N:7.1f} N")
    print(f"    delivered / commanded = {ratio:.3f}")
    return ratio


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("case_a", nargs="?", help="Existing COARSE case ID")
    ap.add_argument("case_b", nargs="?", help="Existing FINE   case ID")
    a = ap.parse_args()
    if a.case_a and a.case_b:
        case_a, case_b = a.case_a, a.case_b
    elif a.case_a or a.case_b:
        sys.exit("provide both case IDs or neither")
    else:
        case_a, case_b = submit_pair()
    print(f"\nReference: commanded thrust per disk = "
          f"{COMMANDED_PER_DISK_N:.1f} N → total {COMMANDED_TOTAL_N:.0f} N")
    measure(case_a, "COARSE (PROP_REFINE = 0.05 c_w)")
    measure(case_b, "FINE   (PROP_REFINE = 0.025 c_w)")
