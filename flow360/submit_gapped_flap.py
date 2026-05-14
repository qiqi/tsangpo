"""
Submit a Tsangpo v2 cruise parent on the GAPPED-FLAP configuration
(`gap_fraction=0.40`, inboard 40% of each semispan has NO flap; the
main wing in that region is replaced by a complete LS(1)-0417 cross-
section made by ESP-joining main_wing.udc with main_wing_tail.udc).

Geometry comes from `geometry/tsangpo_gapped.csm` (sister file of the
continuous-flap `tsangpo.csm`; defaults already set to the 40 %-gapped
configuration).  The only despmtr this script overrides is `phase=0`
for the stowed cruise flap; if you want to vary the gap geometry,
override gap_fraction / mid_outer_semispan / flap_panel_frac /
side_span_frac (keeping them mutually consistent per the file header).
The derived fractions are separate despmtrs because OpenCSM's `set`
parser silently mis-evaluates binary +/- between two named variables
(see flow360/LESSONS.md).

This script submits ONE parent case at the v2 cruise BO estimate point
(α=+7°, θ_ht=0°, T_mult=+1).  No sweeps yet — once the parent's mesh
is healthy and the per-surface CL breakdown is sensible, fork sweeps
can be added (mirroring submit_alpha_sweep / _htail_sweep / _thrust_sweep
but pointing at this project's parent).

    python3 flow360/submit_gapped_flap.py

Lands in: Tsangpo/3_gapped_flap_low_htail/
"""
from __future__ import annotations

import os
import sys
import tempfile
from math import radians
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
import cfd_setup as C
import params as P
import flow360 as fl

CSM      = REPO / "geometry" / "tsangpo_gapped.csm"
AIRFOILS = REPO / "geometry" / "airfoils"

# v2 cruise BO estimate point (same as case-e30a9610 parent on the
# continuous-flap project).  Once the gapped-flap parent has healthy
# forces, switch to the v2 trim solution when it's available.
ALPHA_EFF_DEG  = +7.0
THETA_HT_DEG   = 0.0
T_MULT         = +1.0

THETA_AC_RAD = +radians(ALPHA_EFF_DEG)
THETA_HT_RAD = +radians(THETA_HT_DEG)

GAP_FRACTION         = 0.40
MID_OUTER_SEMISPAN   = 0.39    # 0.5%-of-span air gap on each side
FLAP_PANEL_FRAC      = 0.55    # = flap_outboard_eta(0.95) - 0.40
SIDE_SPAN_FRAC       = 0.60    # = 1 - 0.40

print("=== submit_gapped_flap.py — v2 cruise, gap_fraction=0.40 ===")
print(f"  α        = {ALPHA_EFF_DEG:+.2f}°   θ_ac = {THETA_AC_RAD:+.4f} rad")
print(f"  θ_htail  = {THETA_HT_DEG:+.2f}°    θ_ht = {THETA_HT_RAD:+.4f} rad")
print(f"  T_mult   = {T_MULT:+.2f}")
print(f"  CSM despmtrs: phase=0, gap_fraction={GAP_FRACTION}, "
      f"mid_outer={MID_OUTER_SEMISPAN}, flap_panel={FLAP_PANEL_FRAC}, "
      f"side_span={SIDE_SPAN_FRAC}")

# Reuse an existing project via TSANGPO_PROJECT_ID, otherwise upload.
project_id = os.environ.get("TSANGPO_PROJECT_ID")
if project_id:
    print(f"\nReusing project {project_id} …")
    project = fl.Project.from_cloud(project_id)
else:
    inlined = C.inline_udcs(CSM, AIRFOILS)
    inlined = C.set_csm_despmtrs(
        inlined,
        phase=0,
        gap_fraction=GAP_FRACTION,
        mid_outer_semispan=MID_OUTER_SEMISPAN,
        flap_panel_frac=FLAP_PANEL_FRAC,
        side_span_frac=SIDE_SPAN_FRAC,
    )
    with tempfile.NamedTemporaryFile("w", suffix=".csm", delete=False) as f:
        f.write(inlined)
        tmp_csm = f.name
    print(f"\nUploading inlined {CSM.name} ({len(inlined.splitlines())} lines) …")
    project = fl.Project.from_geometry(
        tmp_csm,
        name="tsangpo_v2_cruise_gapped40",
        length_unit="m",
        tags=["tsangpo", "v2", "cruise", "gapped40", "low_htail"],
    )
    print(f"  → project {project.id}")
    folder_id = C.move_project_to_folder(project, "3_gapped_flap_low_htail")
    print(f"  Moved into folder 3_gapped_flap_low_htail ({folder_id})")

# `get_gapped_geometry_surfaces` returns a SurfaceBundle.  build_params
# accepts it via the SurfaceBundle isinstance branch.
surfaces = C.get_gapped_geometry_surfaces(project)
print(f"\nGapped-flap surfaces:")
print(f"  main_wing: {[s.name for s in surfaces.wing_main_surfs]}")
print(f"  vane     : {[s.name for s in surfaces.wing_vane_surfs]}")
print(f"  aft_flap : {[s.name for s in surfaces.wing_flap_surfs]}")
print(f"  htail    : {surfaces.htail_surf.name}")

print("\nSubmitting parent case …")
case = project.run_case(
    params=C.build_params(
        surfaces,
        theta_ac_rad=THETA_AC_RAD,
        theta_ht_rad=THETA_HT_RAD,
        thrust_mult=T_MULT,
        velocity_m_s=P.V_CRUISE_M_S,
        altitude_m=P.ALT_CRUISE_M,
        n_steps_total=20,
        max_pseudo_steps=1000,
    ),
    name="v2_cruise_gapped40_parent",
    run_async=True,
    tags=["SI", "v2", "cruise", "gapped40", "parent"],
    use_beta_mesher=True,
)
print(f"Case submitted: {case.id}")
print(f"Project:        {project.id}")
