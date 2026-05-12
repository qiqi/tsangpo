#!/usr/bin/env python3
"""
Build a 3-element STOL airfoil (coved main wing + NACA vane + NACA aft flap)
from estol_config.yaml, simulate rigid Fowler kinematics for the stowed,
takeoff, and landing phases, export Selig .dat files for the stowed state,
and produce a 3-panel matplotlib validation figure.
"""
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import yaml


# ---------------------------------------------------------------------------
# Geometric primitives
# ---------------------------------------------------------------------------

def rotate(points: np.ndarray, angle_rad: float, center=(0.0, 0.0)) -> np.ndarray:
    """Rotate Nx2 points by angle_rad (CCW positive) around `center`."""
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    R = np.array([[c, -s], [s, c]])
    return (points - np.asarray(center)) @ R.T + np.asarray(center)


def _unit(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    return v / n if n > 0 else v


# ---------------------------------------------------------------------------
# NACA 4-digit airfoil (unit chord, LE at origin, TE at (1,0))
# ---------------------------------------------------------------------------

def naca4(code: str, n_panels: int = 160,
          te_thickness: float = 0.0) -> np.ndarray:
    """Selig-ordered coordinates for a NACA 4-digit airfoil with unit chord.

    If `te_thickness > 0`, the airfoil is truncated so that the straight-line
    gap between the upper and lower TE points equals `te_thickness` (in local
    unit-chord units), giving a flat blunt trailing edge."""
    m = int(code[0]) / 100.0
    p = int(code[1]) / 10.0
    t = int(code[2:4]) / 100.0
    p_safe = max(p, 1e-9)
    q_safe = max(1.0 - p, 1e-9)

    def _surfaces(x):
        yt = 5 * t * (0.2969 * np.sqrt(x) - 0.1260 * x
                      - 0.3516 * x ** 2 + 0.2843 * x ** 3 - 0.1015 * x ** 4)
        yc = np.where(x < p,
                      m / p_safe ** 2 * (2 * p * x - x ** 2),
                      m / q_safe ** 2 * (1.0 - 2 * p + 2 * p * x - x ** 2))
        dyc = np.where(x < p,
                       2 * m / p_safe ** 2 * (p - x),
                       2 * m / q_safe ** 2 * (p - x))
        theta = np.arctan(dyc)
        return (x - yt * np.sin(theta), yc + yt * np.cos(theta),
                x + yt * np.sin(theta), yc - yt * np.cos(theta))

    x_cut = 1.0
    if te_thickness > 0.0:
        x_scan = np.linspace(0.5, 1.0, 5001)
        xu_s, yu_s, xl_s, yl_s = _surfaces(x_scan)
        gap = np.hypot(xu_s - xl_s, yu_s - yl_s)
        below = gap < te_thickness
        if below.any():
            i = int(np.argmax(below))
            if 0 < i < len(x_scan):
                x1, x2 = x_scan[i - 1], x_scan[i]
                g1, g2 = gap[i - 1], gap[i]
                x_cut = float(x1 + (te_thickness - g1) * (x2 - x1) / (g2 - g1))

    beta = np.linspace(0.0, math.pi, n_panels // 2 + 1)
    x = 0.5 * (1.0 - np.cos(beta)) * x_cut  # cosine clustering on [0, x_cut]
    xu, yu, xl, yl = _surfaces(x)

    upper = np.column_stack([xu[::-1], yu[::-1]])   # upper TE corner -> LE
    lower = np.column_stack([xl[1:], yl[1:]])       # LE -> lower TE corner
    return np.vstack([upper, lower])


# ---------------------------------------------------------------------------
# Anchor a unit-chord airfoil between explicit LE/TE coordinates
# ---------------------------------------------------------------------------

def anchor_airfoil(raw: np.ndarray, target_le, target_te) -> np.ndarray:
    """Map a raw airfoil so its (LE, TE-midpoint) coincide with target_le and
    target_te. Handles blunt TEs: the raw TE is taken as the midpoint of the
    first and last contour points (which are the two TE corners in Selig
    order)."""
    target_le = np.asarray(target_le, dtype=float)
    target_te = np.asarray(target_te, dtype=float)
    raw_le = raw[int(np.argmin(raw[:, 0]))]
    raw_te = 0.5 * (raw[0] + raw[-1])
    raw_vec = raw_te - raw_le
    tgt_vec = target_te - target_le
    scale = float(np.linalg.norm(tgt_vec) / np.linalg.norm(raw_vec))
    angle = math.atan2(tgt_vec[1], tgt_vec[0]) - math.atan2(raw_vec[1], raw_vec[0])
    return rotate(raw - raw_le, angle) * scale + target_le


# ---------------------------------------------------------------------------
# Cove fillet
# ---------------------------------------------------------------------------

def fillet_corner(p_in: np.ndarray, vertex: np.ndarray, p_out: np.ndarray,
                  radius: float, n_arc: int = 40):
    """Return (tangent_in, arc_pts, tangent_out) replacing the sharp corner at
    `vertex` with a tangent circular arc of given `radius`. Arc points go from
    tangent_in to tangent_out along the shorter sweep."""
    v1 = _unit(p_in - vertex)
    v2 = _unit(p_out - vertex)
    cos_alpha = float(np.clip(np.dot(v1, v2), -1.0, 1.0))
    alpha = math.acos(cos_alpha)
    if alpha <= 1e-9 or alpha >= math.pi - 1e-9:
        raise ValueError("fillet_corner: degenerate vertex angle")

    half = alpha / 2.0
    d_tan = radius / math.tan(half)
    d_ctr = radius / math.sin(half)

    t_in = vertex + d_tan * v1
    t_out = vertex + d_tan * v2
    center = vertex + d_ctr * _unit(v1 + v2)

    a_in = math.atan2(t_in[1] - center[1], t_in[0] - center[0])
    a_out = math.atan2(t_out[1] - center[1], t_out[0] - center[0])
    delta = a_out - a_in
    while delta > math.pi:
        delta -= 2 * math.pi
    while delta < -math.pi:
        delta += 2 * math.pi
    angles = a_in + np.linspace(0.0, delta, n_arc)
    arc = np.column_stack([center[0] + radius * np.cos(angles),
                           center[1] + radius * np.sin(angles)])
    return t_in, arc, t_out


# ---------------------------------------------------------------------------
# Main wing (LS(1)-0417 with cove cutout)
# ---------------------------------------------------------------------------

def build_main_wing(airfoil_cfg: dict, cutouts: dict,
                    blunt_thickness: float = 0.0) -> np.ndarray:
    upper = np.asarray(airfoil_cfg['upper'], dtype=float)
    lower = np.asarray(airfoil_cfg['lower'], dtype=float)

    x_lo_cut = float(cutouts['lower_cut_x'])
    x_up_cut = float(cutouts['upper_lip_cut_x'])
    vertex = np.array([cutouts['cove_vertex_x'], cutouts['cove_vertex_y']], dtype=float)
    r_fillet = float(cutouts['cove_fillet_radius'])

    y_up_at_cut = float(np.interp(x_up_cut, upper[:, 0], upper[:, 1]))
    y_lo_at_cut = float(np.interp(x_lo_cut, lower[:, 0], lower[:, 1]))

    up_kept = np.vstack([upper[upper[:, 0] < x_up_cut],
                         [[x_up_cut, y_up_at_cut]]])
    lo_kept = np.vstack([lower[lower[:, 0] < x_lo_cut],
                         [[x_lo_cut, y_lo_at_cut]]])

    upper_lip_tip = np.array([x_up_cut, y_up_at_cut])
    lip_bottom = np.array([x_up_cut, y_up_at_cut - blunt_thickness])
    lower_cut_pt = np.array([x_lo_cut, y_lo_at_cut])

    # The upper cove edge now departs from the BOTTOM of the blunt lip face,
    # so the fillet is computed off lip_bottom -> vertex -> lower_cut_pt.
    _t_lip, arc, _t_lower = fillet_corner(
        lip_bottom, vertex, lower_cut_pt, r_fillet)

    contour = [
        up_kept[::-1],   # upper_lip_tip -> LE
        lo_kept[1:],     # LE -> lower_cut (drop duplicate LE)
        arc[::-1],       # t_lower -> t_lip
    ]
    if blunt_thickness > 0.0:
        contour.append(lip_bottom[None, :])
    return np.vstack(contour)


# ---------------------------------------------------------------------------
# H-tail: inverted LS(1)-0417
# ---------------------------------------------------------------------------

def build_tail(airfoil_cfg: dict) -> np.ndarray:
    """Return the LS(1)-0417 closed contour with y inverted, Selig-ordered.

    Inverted (negative camber) so the tail produces downforce at AoA = 0.
    """
    upper = np.asarray(airfoil_cfg['upper'], dtype=float)
    lower = np.asarray(airfoil_cfg['lower'], dtype=float)
    new_upper_LE_to_TE = np.column_stack([lower[:, 0], -lower[:, 1]])
    new_lower_LE_to_TE = np.column_stack([upper[:, 0], -upper[:, 1]])
    return np.vstack([new_upper_LE_to_TE[::-1], new_lower_LE_to_TE[1:]])


# ---------------------------------------------------------------------------
# Fowler kinematics
# ---------------------------------------------------------------------------

def apply_kinematics(points: np.ndarray, pivot, dx: float, dy: float,
                     rotation_deg: float) -> np.ndarray:
    """Rotate about `pivot` (aero convention: negative rotation_deg = TE down),
    then translate by (dx, dy)."""
    rotated = rotate(points, math.radians(rotation_deg), center=pivot)
    return rotated + np.array([dx, dy])


# ---------------------------------------------------------------------------
# I/O
# ---------------------------------------------------------------------------

def write_selig(path: Path, name: str, points: np.ndarray) -> None:
    with path.open('w') as f:
        f.write(f"{name}\n")
        for x, y in points:
            f.write(f"{x: .6f} {y: .6f}\n")


def write_udc(path: Path, points: np.ndarray, description: str) -> None:
    """Write an OpenCSM sketch UDC that builds a closed planar airfoil
    contour in the XY plane. Consumed from a .csm via `udprim <stem>`.

    The sketch starts at points[0], adds a linseg for every subsequent
    point, and closes with one final linseg back to points[0] (required
    so the sketch becomes a SheetBody, not a WireBody)."""
    with path.open('w') as f:
        f.write(f"# {path.name} — {description}\n")
        f.write("# Generated by build_estol_geometry.py.  Closed planar\n")
        f.write("# sketch in the XY plane (z = 0); SheetBody ready for\n")
        f.write("# `extrude`. DO NOT EDIT BY HAND — regenerate the .dat\n")
        f.write("# and re-run the builder.\n\n")
        x0, y0 = points[0]
        f.write(f"skbeg     {x0: .6f}  {y0: .6f}   0\n")
        for x, y in points[1:]:
            f.write(f"   linseg {x: .6f}  {y: .6f}   0\n")
        f.write(f"   linseg {x0: .6f}  {y0: .6f}   0\n")
        f.write("skend\n\n")
        f.write("end\n")


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_configurations(main_wing: np.ndarray,
                        elements_by_phase: dict,
                        out_path: Path,
                        xlim=(0.0, 1.4),
                        ylim=(-0.4, 0.3)) -> None:
    phases = list(elements_by_phase.keys())
    fig, axes = plt.subplots(len(phases), 1, figsize=(10, 10),
                             sharex=True, sharey=True)

    for ax, phase in zip(axes, phases):
        ax.fill(main_wing[:, 0], main_wing[:, 1],
                facecolor='#cfd8e6', edgecolor='#1f3a68', lw=1.3, label='Main wing')

        vane = elements_by_phase[phase]['vane']
        flap = elements_by_phase[phase]['flap']
        ax.fill(vane[:, 0], vane[:, 1],
                facecolor='#fbd4b4', edgecolor='#9c3d1a', lw=1.2, label='Vane')
        ax.fill(flap[:, 0], flap[:, 1],
                facecolor='#cfe9c8', edgecolor='#2c6b2c', lw=1.2, label='Aft flap')

        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)
        ax.set_aspect('equal', adjustable='box')
        ax.grid(True, alpha=0.3)
        ax.set_title(f"{phase.capitalize()} configuration",
                     loc='left', fontsize=11)
        ax.set_ylabel('y / c')
        ax.legend(loc='lower right', fontsize=8, framealpha=0.9)

    axes[-1].set_xlabel('x / c')
    fig.suptitle('3-element STOL airfoil: Fowler kinematics validation',
                 fontsize=13)
    fig.subplots_adjust(left=0.08, right=0.97, top=0.94, bottom=0.06,
                        hspace=0.25)
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------

def main(cfg_path: Path) -> None:
    cfg = yaml.safe_load(cfg_path.read_text())
    here = cfg_path.parent

    # --- Task 1: stowed base geometry ---
    # Uniform blunt TE thickness in overall-chord units (e.g. 0.005 = 0.5% c).
    # Each NACA element is truncated so its blunt face has the same ABSOLUTE
    # thickness, which means a different fraction of each element's own chord.
    blunt_global = float(cfg.get('trailing_edge', {}).get('blunt_thickness', 0.0))

    def _local_blunt(le, te):
        chord = float(np.linalg.norm(np.asarray(te) - np.asarray(le)))
        return blunt_global / chord if chord > 0 else 0.0

    main_wing = build_main_wing(cfg['airfoils']['ls1_0417'],
                                cfg['main_wing']['cutouts'],
                                blunt_thickness=blunt_global)

    vane_raw = naca4(cfg['vane']['naca'],
                     te_thickness=_local_blunt(cfg['vane']['stowed_le'],
                                               cfg['vane']['stowed_te']))
    flap_raw = naca4(cfg['aft_flap']['naca'],
                     te_thickness=_local_blunt(cfg['aft_flap']['stowed_le'],
                                               cfg['aft_flap']['stowed_te']))
    vane_stowed = anchor_airfoil(vane_raw,
                                 cfg['vane']['stowed_le'],
                                 cfg['vane']['stowed_te'])
    flap_stowed = anchor_airfoil(flap_raw,
                                 cfg['aft_flap']['stowed_le'],
                                 cfg['aft_flap']['stowed_te'])

    tail = build_tail(cfg['airfoils']['ls1_0417'])

    # Selig .dat exports (stowed for the wing elements; inverted for tail).
    write_selig(here / 'main_wing.dat', 'main_wing_ls1_0417_coved', main_wing)
    write_selig(here / 'vane_stowed.dat',
                f"vane_naca{cfg['vane']['naca']}_stowed", vane_stowed)
    write_selig(here / 'flap_stowed.dat',
                f"flap_naca{cfg['aft_flap']['naca']}_stowed", flap_stowed)
    write_selig(here / 'tail.dat', 'tail_ls1_0417_inverted', tail)

    # OpenCSM sketch UDCs (closed planar SheetBody sources). Vane & flap
    # are emitted in stowed pose; Fowler kinematics are applied per-phase
    # inside each tsangpo_<phase>.csm before chord scaling.
    write_udc(here / 'main_wing.udc', main_wing,
              "coved LS(1)-0417 main wing")
    write_udc(here / 'vane.udc', vane_stowed,
              f"NACA {cfg['vane']['naca']} vane (stowed pose)")
    write_udc(here / 'flap.udc', flap_stowed,
              f"NACA {cfg['aft_flap']['naca']} aft flap (stowed pose)")
    write_udc(here / 'tail.udc', tail,
              "inverted LS(1)-0417 H-tail")

    # --- Task 2: kinematic phases ---
    kin = cfg['kinematics']
    pivot = kin['pivot_point']
    vane_te_stowed = np.asarray(cfg['vane']['stowed_te'], dtype=float)
    flap_le_stowed = np.asarray(cfg['aft_flap']['stowed_le'], dtype=float)

    elements_by_phase: dict = {}
    slot_endpoints: dict = {}
    for phase, params in kin['phases'].items():
        v = apply_kinematics(vane_stowed, pivot,
                             params['dx'], params['dy'], params['rotation_deg'])
        f = apply_kinematics(flap_stowed, pivot,
                             params['dx'], params['dy'], params['rotation_deg'])
        elements_by_phase[phase] = {'vane': v, 'flap': f}
        vte = apply_kinematics(vane_te_stowed[None, :], pivot,
                               params['dx'], params['dy'], params['rotation_deg'])[0]
        fle = apply_kinematics(flap_le_stowed[None, :], pivot,
                               params['dx'], params['dy'], params['rotation_deg'])[0]
        slot_endpoints[phase] = (vte, fle)

    # --- Task 3: validation plot ---
    plot_path = here / 'estol_configurations.png'
    plot_configurations(main_wing, elements_by_phase, plot_path)

    for name in ('main_wing.dat', 'vane_stowed.dat', 'flap_stowed.dat',
                 'tail.dat', 'main_wing.udc', 'vane.udc', 'flap.udc',
                 'tail.udc', plot_path.name):
        print(f"Wrote {here / name}")
    for phase, (vte, fle) in slot_endpoints.items():
        gap = math.hypot(fle[0] - vte[0], fle[1] - vte[1])
        print(f"  {phase:>8s}: vane_TE={tuple(np.round(vte, 4))}  "
              f"flap_LE={tuple(np.round(fle, 4))}  slot={gap:.4f} c")


if __name__ == '__main__':
    main(Path(__file__).resolve().parent / 'estol_config.yaml')
