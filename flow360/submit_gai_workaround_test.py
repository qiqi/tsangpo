"""
Single-case test of workaround A for the Flow360 GAI volume-mesher bug
(ERROR 7221) documented in post/FLOW360_GAI_BUG_REPORT.md.

Workaround A: omit `enclosed_entities` from the inner RotationVolume
(htail_rotation).  Geometric enclosure inside the rotation cylinder
should be sufficient; the explicit named-surface hint was what tripped
GAI's lookup.  Patched in cfd_setup.build_params().

This script submits ONE GAI parent at the cruise trim point
(α=+6.78°, θ_ht=-0.54°, T_mult=2.13) and reports its case+vm id so
the volume-mesh build can be checked.  If it builds successfully,
relaunch the full GAI trim campaign and the takeoff Phase C with GAI.

    python3 flow360/submit_gai_workaround_test.py
"""
from __future__ import annotations

import os
import sys
from math import radians
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
import cfd_setup as C
import flow360 as fl

ALPHA_TRIM_DEG    = +6.78
THETA_HT_TRIM_DEG = -0.54
T_MULT_TRIM       = +2.13
GEOM_ACCURACY_M   = 0.000831     # = 0.003 c_wing / 5

PROJECT_ID = os.environ.get(
    "TSANGPO_PROJECT_ID",
    "prj-3e8b1ed8-0109-4f1f-8738-fef0c55a573b",
)
project  = fl.Project.from_cloud(PROJECT_ID)
surfaces = C.get_geometry_surfaces(project)
print(f"Project: {project.id} ({project.metadata.name})")
print(f"Trim point: α=+{ALPHA_TRIM_DEG:.2f}°, θ_ht={THETA_HT_TRIM_DEG:+.2f}°, "
      f"T_mult={T_MULT_TRIM:.2f}")
print(f"GAI geometry_accuracy = {GEOM_ACCURACY_M*1000:.3f} mm  "
      f"(htail_rotation.enclosed_entities omitted)")

params = C.build_params(
    surfaces,
    theta_ac_rad=radians(ALPHA_TRIM_DEG),
    theta_ht_rad=radians(THETA_HT_TRIM_DEG),
    thrust_mult=T_MULT_TRIM,
    n_steps_total=20,
    geometry_accuracy_m=GEOM_ACCURACY_M,
)
case = project.run_case(
    params=params,
    name="GAI_workaround_A_test",
    run_async=True,
    tags=["SI", "GAI", "trim_centered", "workaround_A_test",
          "enclosed_entities_dropped"],
    use_beta_mesher=True,
    use_geometry_AI=True,
)
print(f"\ncase: {case.id}")
print(f"  status: {case.status}")
