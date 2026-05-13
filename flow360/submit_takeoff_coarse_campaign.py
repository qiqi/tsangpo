"""
Takeoff (flap phase 1) coarse-mesh campaign.

Uploads a NEW Flow360 project with `despmtr phase 1` (takeoff flap
deflection) baked into the geometry, then submits:

  • Parent case at the back-of-envelope takeoff trim
        α=+8°, θ_ht=-5°, T_mult=+16
    (see post/TAKEOFF_PLAN.md for the derivation).
  • Three 10-fork sweeps around it:
        α     ∈ {-2, +2, +5, +8, +11, +14, +17, +20, +25, +30}°
        θ_ht  ∈ {-12, -9, -6, -4, -2, 0, +2, +4, +6, +9}°
        T_mult∈ {6, 9, 12, 14, 16, 18, 20, 22, 25, 30}

Coarse beta mesher (no GAI) for fast turnaround.  Once results land,
analyse → refine trim → relaunch on GAI mesh
(`submit_takeoff_gai_campaign.py`).

Run:
    python3 flow360/submit_takeoff_coarse_campaign.py
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

# === Back-of-envelope initial trim guess (post/TAKEOFF_PLAN.md) ===
ALPHA_GUESS_DEG    = +8.0
THETA_HT_GUESS_DEG = -5.0
T_MULT_GUESS       = +16.0

# === Sweep ranges centred on the guess ===
ALPHA_SWEEP_DEG = (-2.0, +2.0, +5.0, +8.0, +11.0, +14.0, +17.0, +20.0, +25.0, +30.0)
HTAIL_SWEEP_DEG = (-12.0, -9.0, -6.0, -4.0, -2.0, 0.0, +2.0, +4.0, +6.0, +9.0)
THRUST_SWEEP    = (6.0, 9.0, 12.0, 14.0, 16.0, 18.0, 20.0, 22.0, 25.0, 30.0)

N_PARENT_STEPS = 20
N_FORK_NEW     = 10
N_FORK_TOTAL   = N_PARENT_STEPS + N_FORK_NEW
STEP_SIZE_S    = 1.0


# ---------------------------------------------------------------------------
# Geometry — inline UDCs into the .csm and set phase = 1 (takeoff)
# ---------------------------------------------------------------------------

def inline_udcs(csm_path: Path, airfoils_dir: Path) -> str:
    """Replace `udprim $/airfoils/<name>` with inlined .udc body."""
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


inlined = inline_udcs(CSM, AIRFOILS)
inlined_with_phase = re.sub(
    r"^(\s*despmtr\s+phase\s+)0\s*$",
    r"\g<1>1",
    inlined,
    count=1,
    flags=re.MULTILINE,
)
assert "despmtr   phase            1" in inlined_with_phase or \
       re.search(r"despmtr\s+phase\s+1\b", inlined_with_phase), \
       "Failed to set phase = 1 in the inlined CSM."

# Reuse project if env var set, otherwise upload fresh
project_id = os.environ.get("TSANGPO_TAKEOFF_PROJECT_ID")
if project_id:
    print(f"Reusing Flow360 project {project_id} …")
    project = fl.Project.from_cloud(project_id)
else:
    with tempfile.NamedTemporaryFile("w", suffix=".csm", delete=False) as f:
        f.write(inlined_with_phase)
        tmp_csm = f.name
    print(f"Uploading takeoff geometry (phase=1, {len(inlined_with_phase.splitlines())} lines)…")
    project = fl.Project.from_geometry(
        tmp_csm,
        name="tsangpo_takeoff_coarse",
        length_unit="m",
        tags=["tsangpo", "takeoff", "SI", "phase1", "coarse"],
    )
print(f"Project: {project.id}")

geo = project.geometry
geo.group_faces_by_tag("capsGroup")
main_wing_surf = geo["main_wing"]
vane_surf      = geo["vane"]
aft_flap_surf  = geo["aft_flap"]
htail_surf     = geo["htail"]
all_surfs = [main_wing_surf, vane_surf, aft_flap_surf, htail_surf]
print(f"  surfaces: {[s.name for s in all_surfs]}")

# Disk loading (cruise FPA × T_mult per case)
FPA_CRUISE_PA        = P.T_CRUISE_PER_PROP_N / P.A_DISK_PER_PROP_M2
SWIRL_CRUISE         = 0.012 * FPA_CRUISE_PA
AC_ZONE_HEIGHT       = 1.25 * P.WING_SPAN_M
AC_ZONE_OUTER_RADIUS = 1.10 * (P.X_TAIL_DEFAULT_M + 1.5 * P.HTAIL_CHORD_M)
HTAIL_ZONE_HEIGHT    = 1.30 * P.HTAIL_SPAN_M
HTAIL_ZONE_RADIUS    = 1.50 * P.HTAIL_CHORD_M
PROP_REFINE_SPACING  = 0.05 * P.WING_MAC_M

print(f"  V_takeoff   = {P.V_TAKEOFF_M_S:.2f} m/s ({P.V_TAKEOFF_M_S/0.5144:.1f} kt)")
print(f"  q_takeoff   = {P.Q_TAKEOFF_PA:.1f} Pa,   q·S = {P.Q_TAKEOFF_PA * P.WING_AREA_M2:.1f} N")
print(f"  cruise FPA  = {FPA_CRUISE_PA:.2f} Pa  (×T_mult per case)")


def build_params(theta_ac_rad: float,
                 theta_ht_rad: float,
                 thrust_mult:  float,
                 n_steps_total: int) -> fl.SimulationParams:
    farfield = fl.AutomatedFarfield()
    fpa   = FPA_CRUISE_PA * thrust_mult
    swirl = SWIRL_CRUISE  * thrust_mult
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
            name="ac_pitch", volumes=[ac_pitch_cyl],
            spec=fl.AngleExpression(f"{theta_ac_rad:.10f}"),
        )
        htail_rotation = fl.Rotation(
            name="htail_pitch", volumes=[htail_pitch_cyl],
            spec=fl.AngleExpression(f"{theta_ht_rad:.10f}"),
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
                velocity_magnitude=P.V_TAKEOFF_M_S * fl.u.m / fl.u.s,
                alpha=0 * fl.u.deg,
                thermal_state=fl.ThermalState.from_standard_atmosphere(
                    altitude=P.ALT_TAKEOFF_M * fl.u.m,
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
                steps=n_steps_total,
                max_pseudo_steps=1000,
                CFL=fl.AdaptiveCFL(max=1e4, convergence_limiting_factor=0.25),
            ),
            outputs=[
                fl.SurfaceOutput(name="surface", entities=all_surfs,
                                 output_fields=["Cp", "Cf", "yPlus", "CfVec"]),
                fl.VolumeOutput(output_fields=["Mach", "qcriterion", "Cp"]),
            ],
        )


# ---------------------------------------------------------------------------
# Parent at the BO-envelope estimate
# ---------------------------------------------------------------------------
print(f"\n=== Parent: α={ALPHA_GUESS_DEG:+.1f}°, θ_ht={THETA_HT_GUESS_DEG:+.1f}°, "
      f"T_mult={T_MULT_GUESS:.1f}× (= {FPA_CRUISE_PA*T_MULT_GUESS:.1f} Pa FPA) ===")
parent_params = build_params(
    radians(ALPHA_GUESS_DEG),
    radians(THETA_HT_GUESS_DEG),
    T_MULT_GUESS,
    N_PARENT_STEPS,
)
parent_case = project.run_case(
    params=parent_params,
    name="takeoff_coarse_BO_estimate",
    run_async=True,
    tags=["SI", "takeoff", "phase1", "coarse", "parent", "BO_estimate"],
    use_beta_mesher=True,
)
print(f"  parent case = {parent_case.id}")

# ---------------------------------------------------------------------------
# Sweeps
# ---------------------------------------------------------------------------
submitted = {"alpha": [], "htail": [], "thrust": []}

print("\n=== α sweep (θ_ht=guess, T_mult=guess) ===")
for alpha_deg in ALPHA_SWEEP_DEG:
    params = build_params(radians(alpha_deg), radians(THETA_HT_GUESS_DEG),
                          T_MULT_GUESS, N_FORK_TOTAL)
    name = f"TO_coarse_alpha_{alpha_deg:+.1f}deg".replace("+", "p").replace("-", "m").replace(".", "p")
    case = project.run_case(
        params=params, name=name, run_async=True,
        fork_from=parent_case,
        tags=["SI", "takeoff", "coarse", "alpha_sweep", f"alpha{alpha_deg:+.1f}"],
        use_beta_mesher=True,
    )
    submitted["alpha"].append((alpha_deg, case.id))
    print(f"  α = {alpha_deg:+5.1f}°  →  {case.id}")

print("\n=== θ_htail sweep (α=guess, T_mult=guess) ===")
for theta_deg in HTAIL_SWEEP_DEG:
    params = build_params(radians(ALPHA_GUESS_DEG), radians(theta_deg),
                          T_MULT_GUESS, N_FORK_TOTAL)
    name = f"TO_coarse_htail_{theta_deg:+.1f}deg".replace("+", "p").replace("-", "m").replace(".", "p")
    case = project.run_case(
        params=params, name=name, run_async=True,
        fork_from=parent_case,
        tags=["SI", "takeoff", "coarse", "htail_sweep", f"htail{theta_deg:+.1f}"],
        use_beta_mesher=True,
    )
    submitted["htail"].append((theta_deg, case.id))
    print(f"  θ_ht = {theta_deg:+5.1f}°  →  {case.id}")

print("\n=== Thrust sweep (α=guess, θ_ht=guess) ===")
for mult in THRUST_SWEEP:
    params = build_params(radians(ALPHA_GUESS_DEG), radians(THETA_HT_GUESS_DEG),
                          mult, N_FORK_TOTAL)
    name = f"TO_coarse_thrust_x{mult:.1f}".replace(".", "p")
    case = project.run_case(
        params=params, name=name, run_async=True,
        fork_from=parent_case,
        tags=["SI", "takeoff", "coarse", "thrust_sweep", f"mult{mult:.1f}"],
        use_beta_mesher=True,
    )
    submitted["thrust"].append((mult, case.id))
    print(f"  T_mult = {mult:.1f}  →  {case.id}")

print("\n=== Summary ===")
print(f"project      : {project.id}  ({project.metadata.name})")
print(f"parent (guess): {parent_case.id}")
for k, lst in submitted.items():
    print(f"{k:6s} sweep ({len(lst)}):")
    for v, cid in lst:
        print(f"  {v:+7.2f}  {cid}")
