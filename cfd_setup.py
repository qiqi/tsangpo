"""
Shared SimulationParams builder for every Tsangpo Flow360 submission.

All cruise / takeoff / sweep / GAI scripts call `build_params(...)` to
construct a SimulationParams for one (θ_ac, θ_htail, T_mult) point.
`inline_udcs()` inlines airfoil UDC contours into a .csm string for
upload.  `set_csm_phase()` swaps the despmtr phase value (stowed →
takeoff → landing).  `get_geometry_surfaces()` looks up the four
lifting-surface tagged groups on a project geometry.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent
# Strip the local flow360/ dir from sys.path so `import flow360` resolves
# to the cloud SDK rather than this repo's flow360/ directory.
sys.path[:] = [p for p in sys.path if str(REPO / "flow360") not in p]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

import numpy as np
import params as P
import flow360 as fl


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def inline_udcs(csm_path: Path, airfoils_dir: Path) -> str:
    """Replace `udprim $/airfoils/<name>` with the matching .udc body."""
    pat = re.compile(r"^\s*udprim\s+\$/airfoils/(\w+)\s*$")
    out = []
    for line in csm_path.read_text().splitlines():
        m = pat.match(line)
        if not m:
            out.append(line)
            continue
        udc = (airfoils_dir / f"{m.group(1)}.udc").read_text().splitlines()
        udc = [l for l in udc if l.strip() != "end"]
        while udc and udc[-1].strip() == "":
            udc.pop()
        out.append(f"# inlined from airfoils/{m.group(1)}.udc")
        out.extend(udc)
    return "\n".join(out) + "\n"


def set_csm_phase(inlined: str, phase: int) -> str:
    """Swap the value of `despmtr phase <N>` in an inlined .csm string."""
    new, n = re.subn(r"(despmtr\s+phase\s+)\d+\b",
                     rf"\g<1>{phase}", inlined, count=1)
    assert n == 1, f"expected exactly 1 phase substitution, got {n}"
    return new


def get_geometry_surfaces(project):
    """Return (main_wing, vane, aft_flap, htail) Surface objects."""
    geo = project.geometry
    geo.group_faces_by_tag("capsGroup")
    return geo["main_wing"], geo["vane"], geo["aft_flap"], geo["htail"]


# Tsangpo folder hierarchy on Flow360 (`Tsangpo/<config_num>_<descr>/`).
# Configurations:
#   1  continuous flap, low htail            (v1 geometry — cruise + takeoff + landing v1 campaigns)
#   2  continuous flap, high htail           (Electra-style; future)
#   3  gapped flap,     low htail            (future)
#   4  gapped flap,     high htail           (future)
#   5  v2 low_htail with bigger tail         (post-revision: CG-origin, half-chord-over-CG, c_h=c_w,
#                                             span 0.40 b_w, htail LE 4 c aft of wing LE)
TSANGPO_FOLDER_IDS = {
    "Tsangpo":                       "folder-4ba7bf56-581f-4b41-b8ee-af8d80c27d11",
    "1_continuous_flap_low_htail":   "folder-8375dac1-f190-48af-82f2-1cfa0dd054f2",
    "2_continuous_flap_high_htail":  "folder-503c0720-2d4b-4a26-859d-9aa949264944",
    "3_gapped_flap_low_htail":       "folder-df67b6aa-c96a-4512-be09-ef8618b26dc3",
    "4_gapped_flap_high_htail":      "folder-b5b13549-2e59-4bf1-9114-edb38ebe5e29",
    "5_v2_low_htail_bigger_tail":    "folder-2c582dd7-918a-430f-a0f8-e28e036451ad",
}


def move_project_to_folder(project, folder_name: str):
    """Move a Flow360 Project into the named Tsangpo subfolder."""
    from flow360.cloud.http_util import http
    folder_id = TSANGPO_FOLDER_IDS[folder_name]
    http.patch(f"v2/projects/{project.id}",
               json={"parentFolderId": folder_id})
    return folder_id


# ---------------------------------------------------------------------------
# SimulationParams construction
# ---------------------------------------------------------------------------

FPA_CRUISE_PA = P.T_CRUISE_PER_PROP_N / P.A_DISK_PER_PROP_M2
SWIRL_CRUISE  = 0.012 * FPA_CRUISE_PA

AC_HEIGHT_M   = 1.25 * P.WING_SPAN_M
AC_RADIUS_M   = 1.10 * (P.X_TAIL_DEFAULT_M + 1.5 * P.HTAIL_CHORD_M)
HT_HEIGHT_M   = 1.30 * P.HTAIL_SPAN_M
HT_RADIUS_M   = 1.50 * P.HTAIL_CHORD_M
PROP_REFINE_M = 0.05 * P.WING_MAC_M


def build_params(
    surfaces,
    theta_ac_rad:              float,
    theta_ht_rad:              float,
    thrust_mult:               float,
    velocity_m_s:              float = P.V_CRUISE_M_S,
    altitude_m:                float = P.ALT_CRUISE_M,
    n_steps_total:             int   = 10,
    step_size_s:               float = 1.0,
    max_pseudo_steps:          int   = 1000,
    geometry_accuracy_m:       float | None = None,
    surface_max_edge_length_m: float = 0.075,
    curvature_resolution_deg:  float | None = None,
):
    """Build a SimulationParams for one (α_body, θ_htail, T_mult) point.

    `surfaces` = result of `get_geometry_surfaces(project)`.
    `geometry_accuracy_m=None`  → regular beta mesher.
    `geometry_accuracy_m=value` → GAI mesh at that resolution with
    preserve_thin_geometry + resolve_face_boundaries.

    Thrust is clamped to a tiny positive value when `thrust_mult=0`,
    because `fl.ForcePerArea.thrust` rejects exact zero.
    """
    main_wing_surf, vane_surf, aft_flap_surf, htail_surf = surfaces
    all_surfs = list(surfaces)

    mult  = max(thrust_mult, 1e-4)
    fpa   = mult * FPA_CRUISE_PA
    swirl = mult * SWIRL_CRUISE
    use_gai  = geometry_accuracy_m is not None
    farfield = fl.AutomatedFarfield()

    with fl.SI_unit_system:
        ac_cyl = fl.Cylinder(
            name="ac_pitch_zone",
            center=(0, 0, 0) * fl.u.m, axis=(0, 1, 0),
            height=AC_HEIGHT_M * fl.u.m, outer_radius=AC_RADIUS_M * fl.u.m,
        )
        # In v2 geometry the origin is the CG and the htail's absolute z is
        # P.WING_Z_M + P.Z_TAIL_LOW_CHORDS · MAC (= +0.4 c for low, +2.9 c for high).
        ht_cyl = fl.Cylinder(
            name="htail_pitch_zone",
            center=(P.X_TAIL_DEFAULT_M, 0, P.Z_TAIL_LOW_M) * fl.u.m,
            axis=(0, 1, 0),
            height=HT_HEIGHT_M * fl.u.m, outer_radius=HT_RADIUS_M * fl.u.m,
        )
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
        ad_models = [
            fl.ActuatorDisk(
                name=cyl.name.replace("disk_", "prop_"),
                entities=cyl,
                force_per_area=fl.ForcePerArea(
                    radius=np.array([0.15 * P.PROP_RADIUS_M, P.PROP_RADIUS_M]) * fl.u.m,
                    thrust=np.array([fpa, fpa]) * fl.u.N / fl.u.m ** 2,
                    circumferential=np.array([swirl, swirl]) * fl.u.N / fl.u.m ** 2,
                ),
            )
            for cyl in prop_cyls
        ]
        ac_rotation = fl.Rotation(
            name="ac_pitch", volumes=[ac_cyl],
            spec=fl.AngleExpression(f"{theta_ac_rad:.10f}"),
        )
        ht_rotation = fl.Rotation(
            name="htail_pitch", volumes=[ht_cyl],
            spec=fl.AngleExpression(f"{theta_ht_rad:.10f}"),
            parent_volume=ac_cyl,
        )

        crv_res = curvature_resolution_deg
        if crv_res is None:
            crv_res = 12 if use_gai else 15
        defaults_kwargs = dict(
            surface_max_edge_length=surface_max_edge_length_m * fl.u.m,
            curvature_resolution_angle=crv_res * fl.u.deg,
            boundary_layer_first_layer_thickness=7.62e-6 * fl.u.m,
            boundary_layer_growth_rate=1.3,
        )
        if use_gai:
            defaults_kwargs |= dict(
                geometry_accuracy=geometry_accuracy_m * fl.u.m,
                preserve_thin_geometry=True,
                resolve_face_boundaries=True,
            )

        return fl.SimulationParams(
            meshing=fl.MeshingParams(
                volume_zones=[
                    farfield,
                    fl.RotationVolume(
                        name="ac_rotation", entities=ac_cyl,
                        enclosed_entities=[main_wing_surf, vane_surf,
                                           aft_flap_surf, ht_cyl],
                        spacing_axial=0.30 * fl.u.m,
                        spacing_radial=0.15 * fl.u.m,
                        spacing_circumferential=0.15 * fl.u.m,
                    ),
                    fl.RotationVolume(
                        name="htail_rotation", entities=ht_cyl,
                        # The legacy beta mesher NEEDS `enclosed_entities`
                        # to map the htail capsGroup-tagged surface into
                        # the htail_pitch_zone rotation volume.  Without
                        # it, the mesher leaves the htail attached to
                        # `farfield/htail` (verified on v2 vm-a55869c0:
                        # geometry correct but htail surface missing from
                        # the htail_pitch_zone).  This is the inverse of
                        # the GAI workaround discussed in
                        # post/FLOW360_GAI_BUG_REPORT.md — GAI rejects
                        # this listing but legacy requires it.  Since
                        # GAI is blocked on this geometry anyway, list
                        # the htail surface explicitly.
                        enclosed_entities=[htail_surf],
                        spacing_axial=0.15 * fl.u.m,
                        spacing_radial=0.06 * fl.u.m,
                        spacing_circumferential=0.06 * fl.u.m,
                    ),
                ],
                refinements=[
                    fl.UniformRefinement(entities=prop_cyls,
                                         spacing=PROP_REFINE_M * fl.u.m),
                ],
                defaults=fl.MeshingDefaults(**defaults_kwargs),
                gap_treatment_strength=0.5,
            ),
            reference_geometry=fl.ReferenceGeometry(
                area=P.WING_AREA_M2 * fl.u.m ** 2,
                moment_center=(0, 0, 0) * fl.u.m,
                moment_length=(P.WING_SEMI_SPAN_M, P.WING_MAC_M,
                               P.WING_SEMI_SPAN_M) * fl.u.m,
            ),
            operating_condition=fl.AerospaceCondition(
                velocity_magnitude=velocity_m_s * fl.u.m / fl.u.s,
                alpha=0 * fl.u.deg,
                thermal_state=fl.ThermalState.from_standard_atmosphere(
                    altitude=altitude_m * fl.u.m,
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
                ac_rotation, ht_rotation, *ad_models,
            ],
            time_stepping=fl.Unsteady(
                step_size=step_size_s * fl.u.s,
                steps=n_steps_total,
                max_pseudo_steps=max_pseudo_steps,
                CFL=fl.AdaptiveCFL(max=1e4, convergence_limiting_factor=0.25),
            ),
            outputs=[
                fl.SurfaceOutput(name="surface", entities=all_surfs,
                                 output_fields=["Cp", "Cf", "yPlus", "CfVec"]),
                fl.VolumeOutput(output_fields=["Mach", "qcriterion", "Cp"]),
            ],
        )
