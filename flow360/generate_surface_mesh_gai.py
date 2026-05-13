"""
Generate a surface mesh on the Tsangpo geometry using Flow360's
Geometry-AI (GAI) beta mesher.

Geometry accuracy is set to (thinniest TE) / 5, where the thinniest TE
on the 3-element wing is the blunt TE at 0.003 c (the htail's LS(1)-0417
TE is sharp and doesn't enter the accuracy budget):

    TE absolute thickness = 0.003 × wing_chord
                          = 0.003 × 1.3843 m
                          ≈ 4.153 mm
    geometry_accuracy     = TE / 5 ≈ 0.831 mm

`preserve_thin_geometry=True` keeps the blunt TE faces from being
collapsed away — necessary since the TE thickness sits within an order
of magnitude of `geometry_accuracy`.

This runs against the project created by `submit_cruise.py`.  Pass the
project ID via `TSANGPO_PROJECT_ID`, otherwise we use the most-recent
cruise project.

    TSANGPO_PROJECT_ID=prj-... python3 flow360/generate_surface_mesh_gai.py
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
import params as P

sys.path = [p for p in sys.path if str(REPO / "flow360") not in p]
import flow360 as fl

TE_NORMALIZED       = 0.003            # 0.003 c blunt TE from estol_config.yaml
TE_THICKNESS_M      = TE_NORMALIZED * P.WING_CHORD_M
GEOM_ACCURACY_M     = TE_THICKNESS_M / 5.0
SURF_MAX_EDGE_M     = 0.075            # match cruise mesh
CURV_RES_DEG        = 12               # GAI default; finer than cruise's 15° to use the AI resolution

DEFAULT_PROJECT_ID = "prj-90cebbd0-80a7-4bb1-9a09-cbad8e7c5bc7"
project_id = os.environ.get("TSANGPO_PROJECT_ID", DEFAULT_PROJECT_ID)
project = fl.Project.from_cloud(project_id)
print(f"Project: {project.id}")
print(f"  wing chord            = {P.WING_CHORD_M:.4f} m")
print(f"  thinniest TE (0.003 c) = {TE_THICKNESS_M*1000:.3f} mm")
print(f"  geometry_accuracy     = TE/5 = {GEOM_ACCURACY_M*1000:.3f} mm "
      f"({GEOM_ACCURACY_M:.6f} m)")

geo = project.geometry
geo.group_faces_by_tag("capsGroup")
main_wing_surf = geo["main_wing"]
vane_surf      = geo["vane"]
aft_flap_surf  = geo["aft_flap"]
htail_surf     = geo["htail"]
all_surfs = [main_wing_surf, vane_surf, aft_flap_surf, htail_surf]

farfield = fl.AutomatedFarfield()

with fl.SI_unit_system:
    params = fl.SimulationParams(
        meshing=fl.MeshingParams(
            volume_zones=[farfield],
            defaults=fl.MeshingDefaults(
                geometry_accuracy=GEOM_ACCURACY_M * fl.u.m,
                preserve_thin_geometry=True,
                surface_max_edge_length=SURF_MAX_EDGE_M * fl.u.m,
                curvature_resolution_angle=CURV_RES_DEG * fl.u.deg,
                resolve_face_boundaries=True,
                boundary_layer_first_layer_thickness=7.62e-6 * fl.u.m,
                boundary_layer_growth_rate=1.3,
            ),
        ),
        # Surface meshing doesn't need an operating condition or models,
        # but SimulationParams requires *some* reference_geometry — give
        # it the wing reference values for completeness.
        reference_geometry=fl.ReferenceGeometry(
            area=P.WING_AREA_M2 * fl.u.m ** 2,
            moment_center=(0, 0, 0) * fl.u.m,
            moment_length=(P.WING_SEMI_SPAN_M, P.WING_MAC_M,
                           P.WING_SEMI_SPAN_M) * fl.u.m,
        ),
    )

print("Submitting GAI surface-mesh job …")
surface_mesh = project.generate_surface_mesh(
    params=params,
    name="tsangpo_gai_surface_mesh",
    run_async=True,
    use_beta_mesher=True,
    use_geometry_AI=True,
    tags=["SI", "GAI", "surface_mesh",
          f"geom_acc_{int(GEOM_ACCURACY_M*1e6)}um"],
)
print(f"Surface mesh submitted: {surface_mesh.id}")
