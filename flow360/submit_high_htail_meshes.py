"""
Mesh-only submission for the two high-htail v2 geometries.

For each (config, phase) combination, uploads the inlined .csm to a new
Flow360 project, runs the surface + volume mesher under the beta mesher,
and parks the project in the appropriate Tsangpo folder.  NO solver run
is submitted — the user wants to inspect meshes first.

Configs
  * continuous_high_htail → folder `2_v2_continuous_high_htail`
       CSM: geometry/tsangpo_high_htail.csm
  * gapped_high_htail     → folder `4_v2_gapped_high_htail`
       CSM: geometry/tsangpo_gapped_high_htail.csm

For each config, three phases (0=stowed/cruise, 1=takeoff, 2=landing).
6 projects total.  In each project we generate ONE volume mesh.

The MeshingParams used here come from `cfd_setup.build_params(...)` with
placeholder operating + thrust values — only the meshing section matters
for `generate_volume_mesh`.  The script supplies `htail_z_m=Z_TAIL_HIGH_M`
so the htail rotation cylinder sits up at the high-htail z.

Usage:
    python3 flow360/submit_high_htail_meshes.py             # all 6
    python3 flow360/submit_high_htail_meshes.py continuous  # both phases of one config
    python3 flow360/submit_high_htail_meshes.py gapped takeoff   # one combo
"""
from __future__ import annotations

import os, sys, tempfile
from math import radians
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
import cfd_setup as C
import params as P
import flow360 as fl

# Gapped despmtrs (must match tsangpo_gapped_high_htail.csm header defaults).
GAP_FRACTION         = 0.40
MID_OUTER_SEMISPAN   = 0.39
FLAP_PANEL_FRAC      = 0.55
WING_SIDE_FRAC       = 0.61

CONFIGS = {
    "continuous_high_htail": dict(
        csm        = REPO / "geometry" / "tsangpo_high_htail.csm",
        folder     = "2_v2_continuous_high_htail",
        get_surfs  = C.get_geometry_surfaces,
        project_tag= "continuous",
        despmtr_override = lambda inlined, phase: C.set_csm_despmtrs(inlined, phase=phase),
    ),
    "gapped_high_htail": dict(
        csm        = REPO / "geometry" / "tsangpo_gapped_high_htail.csm",
        folder     = "4_v2_gapped_high_htail",
        get_surfs  = C.get_gapped_geometry_surfaces,
        project_tag= "gapped40",
        despmtr_override = lambda inlined, phase: C.set_csm_despmtrs(
            inlined, phase=phase,
            gap_fraction=GAP_FRACTION, mid_outer_semispan=MID_OUTER_SEMISPAN,
            flap_panel_frac=FLAP_PANEL_FRAC, wing_side_frac=WING_SIDE_FRAC,
        ),
    ),
}

PHASES = {
    "stowed":  dict(phase=0, name="cruise",  alpha=+7.0, theta_ht=0.0,  T=+1.0,
                    V_m_s=P.V_CRUISE_M_S,  alt_m=P.ALT_CRUISE_M),
    "takeoff": dict(phase=1, name="takeoff", alpha=+8.0, theta_ht=-5.0, T=+16.0,
                    V_m_s=P.V_TAKEOFF_M_S, alt_m=0.0),
    "landing": dict(phase=2, name="landing", alpha=+8.0, theta_ht=-6.0, T=+12.0,
                    V_m_s=P.V_LANDING_M_S, alt_m=0.0),
}


def upload_and_mesh(cfg_key: str, phase_key: str):
    cfg   = CONFIGS[cfg_key]
    phase = PHASES[phase_key]
    print(f"\n=== {cfg_key} / {phase_key} (phase {phase['phase']}) ===")

    inlined = C.inline_udcs(cfg["csm"], REPO / "geometry" / "airfoils")
    inlined = cfg["despmtr_override"](inlined, phase["phase"])
    with tempfile.NamedTemporaryFile("w", suffix=".csm", delete=False) as f:
        f.write(inlined); tmp_csm = f.name
    print(f"  uploading {cfg['csm'].name} (phase={phase['phase']}, "
          f"{len(inlined.splitlines())} lines) …")
    name = f"tsangpo_v2_{phase['name']}_{cfg['project_tag']}_high_htail"
    project = fl.Project.from_geometry(
        tmp_csm, name=name, length_unit="m",
        tags=["tsangpo", "v2", phase["name"], cfg["project_tag"], "high_htail", "mesh_only"],
    )
    print(f"  → project {project.id}")
    folder_id = C.move_project_to_folder(project, cfg["folder"])
    print(f"  moved into {cfg['folder']} ({folder_id[:24]}…)")

    surfaces = cfg["get_surfs"](project)
    params = C.build_params(
        surfaces,
        theta_ac_rad   = +radians(phase["alpha"]),
        theta_ht_rad   = +radians(phase["theta_ht"]),
        thrust_mult    = phase["T"],
        velocity_m_s   = phase["V_m_s"],
        altitude_m     = phase["alt_m"],
        n_steps_total  = 1,           # placeholder — meshing only
        max_pseudo_steps = 1,
        htail_z_m      = P.Z_TAIL_HIGH_M,
    )
    vm = project.generate_volume_mesh(
        params=params,
        name=f"{name}_volume_mesh",
        run_async=True,
        tags=["tsangpo", "v2", phase["name"], cfg["project_tag"], "high_htail", "mesh_only"],
        use_beta_mesher=True,
    )
    print(f"  volume-mesh submitted: {vm.id}")
    return cfg_key, phase_key, project.id, vm.id


if __name__ == "__main__":
    args = [a.lower() for a in sys.argv[1:]]
    cfg_keys = [k for k in CONFIGS if any(a in k for a in args)] if args else list(CONFIGS)
    phase_keys = [k for k in PHASES if any(a in k for a in args)] if args else list(PHASES)
    if not cfg_keys:   cfg_keys = list(CONFIGS)
    if not phase_keys: phase_keys = list(PHASES)
    print(f"Submitting mesh-only for configs={cfg_keys}, phases={phase_keys}")

    results = []
    for ck in cfg_keys:
        for ph in phase_keys:
            results.append(upload_and_mesh(ck, ph))

    print("\n=== Summary ===")
    for ck, ph, pid, vmid in results:
        print(f"  {ck:25s} {ph:8s}  project {pid}  vm {vmid}")
