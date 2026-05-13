"""
Submit the Tsangpo cruise case to Flow360.

Steady RANS at α_freestream = 0; effective AoA is controlled by an
aircraft-wide rotation volume centered at the wing-root quarter-chord
(taken as the CG). The H-tail sits inside a smaller nested rotation
volume so it can pitch relative to the aircraft. Both rotations are
held fixed at 0° in this script — a follow-up `submit_trim.py` will
fork from this run with `FromUserDefinedDynamics()` driving them to
L = W and ΣM = 0.

Run:
    python flow360/submit_cruise.py
"""
from __future__ import annotations

import os
import re
import sys
import tempfile
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
import params as P

# Delayed import so local flow360/ dir doesn't shadow the SDK.
sys.path = [p for p in sys.path if str(REPO / "flow360") not in p]
import flow360 as fl

CSM      = REPO / "geometry" / "tsangpo.csm"
AIRFOILS = REPO / "geometry" / "airfoils"


def inline_udcs(csm_path: Path, airfoils_dir: Path) -> str:
    """Return tsangpo.csm with each `udprim $/airfoils/<name>` replaced by
    the body of <name>.udc (minus the trailing `end`)."""
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


project_id = os.environ.get("TSANGPO_PROJECT_ID")
if project_id:
    print(f"Reusing Flow360 project {project_id} …")
    project = fl.Project.from_cloud(project_id)
else:
    inlined = inline_udcs(CSM, AIRFOILS)
    with tempfile.NamedTemporaryFile("w", suffix=".csm", delete=False) as f:
        f.write(inlined)
        tmp_csm = f.name
    print(f"Uploading inlined tsangpo.csm ({len(inlined.splitlines())} lines) …")
    project = fl.Project.from_geometry(
        tmp_csm,
        name="tsangpo_cruise",
        length_unit="ft",
        tags=["tsangpo", "cruise", "stowed"],
    )

geo = project.geometry
geo.group_faces_by_tag("capsGroup")
main_wing_surf = geo["main_wing"]
vane_surf      = geo["vane"]
aft_flap_surf  = geo["aft_flap"]
htail_surf     = geo["htail"]
all_surfs = [main_wing_surf, vane_surf, aft_flap_surf, htail_surf]
print(f"  Surfaces: {[s.name for s in all_surfs]}")

# ── Disk loading (cruise, uniform across each disk) ────────────────────
FPA_CRUISE   = P.T_CRUISE_PER_PROP_LBF / P.A_DISK_PER_PROP_FT2
SWIRL_CRUISE = 0.012 * FPA_CRUISE
print(f"  Cruise thrust:  {P.T_CRUISE_PER_PROP_LBF:.2f} lbf/prop  "
      f"(FPA = {FPA_CRUISE:.3f} psf)")

# ── Rotation-volume geometry ───────────────────────────────────────────
# Aircraft volume: centered at CG (= wing-root quarter-chord = origin),
# axis = Y (pitch). Big enough to enclose every wall + the H-tail
# rotation cylinder + all 10 prop disks. Cylinder radius is measured in
# the X-Z plane, so we just need to cover max(|x|, |z|) of every
# enclosed entity. H-tail aft tip sits ~18.3 ft from origin in X;
# prop cylinders dip to z ≈ −2.1 ft; 20 ft outer radius leaves margin.
AC_ZONE_HEIGHT       = 1.25 * P.WING_SPAN_FT          # span + 12.5% margin
AC_ZONE_OUTER_RADIUS = 1.10 * (P.X_TAIL_DEFAULT_FT
                               + 1.5 * P.HTAIL_CHORD_FT)
HTAIL_ZONE_HEIGHT    = 1.3  * P.HTAIL_SPAN_FT
HTAIL_ZONE_RADIUS    = 1.5  * P.HTAIL_CHORD_FT
PROP_REFINE_SPACING  = 0.05 * P.WING_MAC_FT

farfield = fl.AutomatedFarfield()

with fl.imperial_unit_system:

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

    # Per-prop cylinders (reused as ActuatorDisk volumes, UniformRefinement
    # entities, and enclosed entities of the aircraft rotation volume).
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

    # Both rotations held at 0° in this run — fork with FromUserDefinedDynamics
    # for the trim search.
    ac_rotation = fl.Rotation(
        name="ac_pitch",
        volumes=[ac_pitch_cyl],
        spec=fl.AngleExpression("0"),
    )
    htail_rotation = fl.Rotation(
        name="htail_pitch",
        volumes=[htail_pitch_cyl],
        spec=fl.AngleExpression("0"),
        parent_volume=ac_pitch_cyl,
    )

    params = fl.SimulationParams(
        meshing=fl.MeshingParams(
            volume_zones=[
                farfield,
                fl.RotationVolume(
                    name="ac_rotation",
                    entities=ac_pitch_cyl,
                    # Walls + nested htail rotation volume. Prop cylinders are
                    # NOT enclosed (they're not RotationVolumes themselves, and
                    # listing them here makes Flow360 mis-classify them as
                    # nested sliding interfaces); they stay in the outer zone
                    # as UniformRefinement regions. The thrust axis stays
                    # fixed in inertial X — acceptable for the small (<5°)
                    # aircraft pitch range UDD will explore.
                    enclosed_entities=[main_wing_surf, vane_surf, aft_flap_surf,
                                       htail_surf, htail_pitch_cyl],
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
            moment_center=(0, 0, 0) * fl.u.ft,           # CG = wing-root c/4
            moment_length=P.WING_MAC_FT * fl.u.ft,
        ),
        operating_condition=fl.AerospaceCondition(
            velocity_magnitude=P.V_CRUISE_FT_S * fl.u.ft / fl.u.s,
            alpha=0 * fl.u.deg,                          # α controlled by ac_pitch
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
    name="cruise_v2_refined",
    run_async=True,
    tags=["cruise", "stowed", "ac_rotation_zone", "refined_props"],
    use_beta_mesher=True,
)
print(f"Case submitted: {case.id}")
print(f"Project:        {project.id}")
