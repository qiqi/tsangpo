"""
Submit a static cruise baseline to Flow360 in SI units.

  • Geometry uploaded from the inlined SI .csm (length_unit = "m").
  • α_effective set by a fixed `AngleExpression` on the aircraft pitch
    rotation volume (no UDD).  Default α = 7° (passive-rotation
    convention: θ_ac = −α).
  • H-tail rotation also set by a fixed `AngleExpression` — default 0°
    in this parent run.  `submit_htail_sweep.py` forks from this case
    and varies θ_htail through a series of fixed values.
  • Reference area = wing area; moment_length = (b/2, c, b/2):
    semi-span for roll & yaw, chord for pitch.
  • First-layer thickness halved vs. the imperial run (5e-5 ft →
    7.62e-6 m) to pull y+ closer to 1.

    python flow360/submit_cruise.py
"""
from __future__ import annotations

import os
import re
import sys
import tempfile
from math import radians
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
import params as P

sys.path = [p for p in sys.path if str(REPO / "flow360") not in p]
import flow360 as fl

CSM      = REPO / "geometry" / "tsangpo.csm"
AIRFOILS = REPO / "geometry" / "airfoils"

# Effective AoA via ac_pitch passive rotation:
#   freestream is held at α = 0 in the operating condition; the aircraft
#   rotation volume is rotated by θ_ac so that the body sees the
#   freestream tilted by θ_ac, i.e. effective AoA = −θ_ac.  α = +7° ⇒
#   θ_ac = −7° = −0.12217 rad.
ALPHA_EFF_DEG = 7.0
THETA_AC_RAD  = -radians(ALPHA_EFF_DEG)
THETA_HT_RAD  = 0.0


def inline_udcs(csm_path: Path, airfoils_dir: Path) -> str:
    """Replace every `udprim $/airfoils/<name>` in tsangpo.csm with the
    body of the matching .udc file (minus the trailing `end`)."""
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


# Re-use an existing project by exporting TSANGPO_PROJECT_ID; otherwise
# upload a fresh SI project.
project_id = os.environ.get("TSANGPO_PROJECT_ID")
if project_id:
    print(f"Reusing Flow360 project {project_id} …")
    project = fl.Project.from_cloud(project_id)
else:
    inlined = inline_udcs(CSM, AIRFOILS)
    with tempfile.NamedTemporaryFile("w", suffix=".csm", delete=False) as f:
        f.write(inlined)
        tmp_csm = f.name
    print(f"Uploading SI tsangpo.csm ({len(inlined.splitlines())} lines) …")
    project = fl.Project.from_geometry(
        tmp_csm,
        name="tsangpo_cruise_SI",
        length_unit="m",
        tags=["tsangpo", "cruise", "SI", "static"],
    )

geo = project.geometry
geo.group_faces_by_tag("capsGroup")
main_wing_surf = geo["main_wing"]
vane_surf      = geo["vane"]
aft_flap_surf  = geo["aft_flap"]
htail_surf     = geo["htail"]
all_surfs = [main_wing_surf, vane_surf, aft_flap_surf, htail_surf]
print(f"  Surfaces: {[s.name for s in all_surfs]}")

# Disk loading — baseline cruise FPA (uniform, no scaling).
FPA_CRUISE_PA = P.T_CRUISE_PER_PROP_N / P.A_DISK_PER_PROP_M2
SWIRL_CRUISE  = 0.012 * FPA_CRUISE_PA
print(f"  Cruise thrust:  {P.T_CRUISE_PER_PROP_N:.1f} N/prop  "
      f"(FPA = {FPA_CRUISE_PA:.2f} Pa)")

# Rotation-volume geometry (SI lengths).
AC_ZONE_HEIGHT       = 1.25 * P.WING_SPAN_M
AC_ZONE_OUTER_RADIUS = 1.10 * (P.X_TAIL_DEFAULT_M + 1.5 * P.HTAIL_CHORD_M)
HTAIL_ZONE_HEIGHT    = 1.30 * P.HTAIL_SPAN_M
HTAIL_ZONE_RADIUS    = 1.50 * P.HTAIL_CHORD_M
PROP_REFINE_SPACING  = 0.05 * P.WING_MAC_M

farfield = fl.AutomatedFarfield()

with fl.SI_unit_system:

    ac_pitch_cyl = fl.Cylinder(
        name="ac_pitch_zone",
        center=(0, 0, 0) * fl.u.m,
        axis=(0, 1, 0),
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

    ad_models = [
        fl.ActuatorDisk(
            name=cyl.name.replace("disk_", "prop_"),
            entities=cyl,
            force_per_area=fl.ForcePerArea(
                radius=np.array([0.15 * P.PROP_RADIUS_M, P.PROP_RADIUS_M]) * fl.u.m,
                thrust=np.array([FPA_CRUISE_PA, FPA_CRUISE_PA]) * fl.u.N / fl.u.m ** 2,
                circumferential=np.array([SWIRL_CRUISE, SWIRL_CRUISE])
                                * fl.u.N / fl.u.m ** 2,
            ),
        )
        for cyl in prop_cyls
    ]

    ac_rotation = fl.Rotation(
        name="ac_pitch",
        volumes=[ac_pitch_cyl],
        spec=fl.AngleExpression(f"{THETA_AC_RAD}"),
    )
    htail_rotation = fl.Rotation(
        name="htail_pitch",
        volumes=[htail_pitch_cyl],
        spec=fl.AngleExpression(f"{THETA_HT_RAD}"),
        parent_volume=ac_pitch_cyl,
    )

    params = fl.SimulationParams(
        meshing=fl.MeshingParams(
            volume_zones=[
                farfield,
                fl.RotationVolume(
                    name="ac_rotation",
                    entities=ac_pitch_cyl,
                    enclosed_entities=[main_wing_surf, vane_surf, aft_flap_surf,
                                       htail_pitch_cyl],
                    spacing_axial=0.30 * fl.u.m,
                    spacing_radial=0.15 * fl.u.m,
                    spacing_circumferential=0.15 * fl.u.m,
                ),
                fl.RotationVolume(
                    name="htail_rotation",
                    entities=htail_pitch_cyl,
                    enclosed_entities=[htail_surf],
                    spacing_axial=0.15 * fl.u.m,
                    spacing_radial=0.06 * fl.u.m,
                    spacing_circumferential=0.06 * fl.u.m,
                ),
            ],
            refinements=[
                fl.UniformRefinement(
                    entities=prop_cyls,
                    spacing=PROP_REFINE_SPACING * fl.u.m,
                ),
            ],
            defaults=fl.MeshingDefaults(
                surface_max_edge_length=0.075 * fl.u.m,        # ≈ 0.25 ft
                curvature_resolution_angle=15 * fl.u.deg,
                boundary_layer_first_layer_thickness=7.62e-6 * fl.u.m,  # half of 5e-5 ft
                boundary_layer_growth_rate=1.3,
            ),
            gap_treatment_strength=0.5,
        ),
        reference_geometry=fl.ReferenceGeometry(
            area=P.WING_AREA_M2 * fl.u.m ** 2,
            moment_center=(0, 0, 0) * fl.u.m,
            # (lx, ly, lz) for (Mx, My, Mz): semi-span for roll/yaw,
            # chord for pitch.
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

print(f"  α_eff = {ALPHA_EFF_DEG:.1f}°  (θ_ac = {THETA_AC_RAD:+.4f} rad)")
print(f"  θ_htail = {THETA_HT_RAD:+.4f} rad")
print("Submitting parent cruise case (mesh → solve) …")
case = project.run_case(
    params=params,
    name="cruise_SI_baseline",
    run_async=True,
    tags=["SI", "cruise", f"alpha{int(ALPHA_EFF_DEG)}", "htail0"],
    use_beta_mesher=True,
)
print(f"Case submitted: {case.id}")
print(f"Project:        {project.id}")
