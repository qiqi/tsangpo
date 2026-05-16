"""
Single-axes overlay of the three high-lift configurations
(stowed / takeoff / landing), each shifted downward by 0.5 c, no axes.

Output: paper/figures/airfoil_phases_stacked.png
"""
from __future__ import annotations
import importlib.util, sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
AIRFOIL_DIR = REPO / "geometry" / "airfoils"

# Load build_estol_geometry as a module to reuse its primitives.
sys.path.insert(0, str(AIRFOIL_DIR))
sys.path.insert(0, str(REPO))
spec = importlib.util.spec_from_file_location("besg", AIRFOIL_DIR / "build_estol_geometry.py")
besg = importlib.util.module_from_spec(spec)
spec.loader.exec_module(besg)

cfg = yaml.safe_load((AIRFOIL_DIR / "estol_config.yaml").read_text())
blunt = cfg["trailing_edge"]["blunt_thickness"]
vane_cfg, flap_cfg = cfg["vane"], cfg["aft_flap"]
vane_chord = np.linalg.norm(np.array(vane_cfg["stowed_te"]) - np.array(vane_cfg["stowed_le"]))
flap_chord = np.linalg.norm(np.array(flap_cfg["stowed_te"]) - np.array(flap_cfg["stowed_le"]))

_, main_flat = besg.build_main_wing(cfg["airfoils"]["ls1_0417"], cfg["main_wing"]["cutouts"], blunt)
_, vane_flat = besg.build_naca_element(vane_cfg["naca"], blunt / vane_chord,
                                        vane_cfg["stowed_le"], vane_cfg["stowed_te"])
_, flap_flat = besg.build_naca_element(flap_cfg["naca"], blunt / flap_chord,
                                        flap_cfg["stowed_le"], flap_cfg["stowed_te"])

pivot = cfg["kinematics"]["pivot_point"]
phases = ["stowed", "takeoff", "landing"]
elements = {ph: {"vane": besg.apply_kinematics(vane_flat, pivot,
                                                cfg["kinematics"]["phases"][ph]["dx"],
                                                cfg["kinematics"]["phases"][ph]["dy"],
                                                cfg["kinematics"]["phases"][ph]["rotation_deg"]),
                  "flap": besg.apply_kinematics(flap_flat, pivot,
                                                cfg["kinematics"]["phases"][ph]["dx"],
                                                cfg["kinematics"]["phases"][ph]["dy"],
                                                cfg["kinematics"]["phases"][ph]["rotation_deg"])}
            for ph in phases}

# Actuator-disk side-view rectangle (axial thickness x vertical
# diameter).  In airfoil-chord coordinates: prop center is at
# x/c = -0.20 (i.e. 0.2 c ahead of wing LE), y/c = -0.30 (i.e. 0.3 c
# below the wing chord plane).  Disk thickness = 0.10 c axially,
# diameter = 2 * 0.385 c = 0.77 c vertically.
DISK_X_C  = -0.20   # center
DISK_Y_C  = -0.30
DISK_TH_C = 0.10
DISK_D_C  = 0.77
DISK_X_LO = DISK_X_C - DISK_TH_C / 2
DISK_Y_LO = DISK_Y_C - DISK_D_C / 2
fig, ax = plt.subplots(figsize=(7.0, 7.5))
DY = 0.3
DX = 0.2   # horizontal shift between phases so the disks don't overlap
for i, ph in enumerate(phases):
    y_off = -i * DY
    x_off = +i * DX
    # actuator-disk rectangle (drawn first so airfoils overlay it cleanly)
    ax.fill([DISK_X_LO + x_off,                DISK_X_LO + DISK_TH_C + x_off,
             DISK_X_LO + DISK_TH_C + x_off,    DISK_X_LO + x_off,
             DISK_X_LO + x_off],
            [DISK_Y_LO + y_off,                DISK_Y_LO + y_off,
             DISK_Y_LO + DISK_D_C + y_off,     DISK_Y_LO + DISK_D_C + y_off,
             DISK_Y_LO + y_off],
            facecolor="#d8d8d8", edgecolor="#4a4a4a", lw=1.0, alpha=0.85, zorder=0)
    # airfoils
    ax.fill(main_flat[:, 0]              + x_off, main_flat[:, 1]              + y_off,
            facecolor="#cfd8e6", edgecolor="#1f3a68", lw=1.3, zorder=1)
    ax.fill(elements[ph]["vane"][:, 0]   + x_off, elements[ph]["vane"][:, 1]   + y_off,
            facecolor="#fbd4b4", edgecolor="#9c3d1a", lw=1.2, zorder=1)
    ax.fill(elements[ph]["flap"][:, 0]   + x_off, elements[ph]["flap"][:, 1]   + y_off,
            facecolor="#cfe9c8", edgecolor="#2c6b2c", lw=1.2, zorder=1)

ax.set_aspect("equal", adjustable="box")
ax.set_axis_off()
fig.subplots_adjust(left=0, right=1, top=1, bottom=0)
out = HERE / "airfoil_phases_stacked.png"
fig.savefig(out, dpi=200, bbox_inches="tight", pad_inches=0.05)
print(f"wrote {out}")
