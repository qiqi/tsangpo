#!/usr/bin/env python3
"""
Build a 3-element STOL airfoil (coved main wing + NACA vane + NACA aft flap)
from estol_config.yaml, simulate rigid Fowler kinematics for the stowed,
takeoff, and landing phases, export Selig .dat files for the stowed state,
emit OpenCSM sketch UDCs (consumed by tsangpo.csm) with smooth splines for
the curved sections, and produce a 3-panel matplotlib validation figure.

UDC topology:
    main_wing — 5 segments: spline (main contour, upper-lip-tip → LE →
                lower-cut-pt), linseg (lower cove edge), spline (filleted
                arc), linseg (upper cove edge), linseg (blunt TE close).
    vane/flap — 2 splines (upper TE corner → LE, LE → lower TE corner)
                joined by a zero-length linseg break, plus a linseg blunt
                TE close.
    tail     — 2 splines (sharp TE on LS(1)-0417), joined by linseg break;
                no blunt TE since both upper/lower meet at (1, 0).
"""
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import yaml


# ---------------------------------------------------------------------------
# Geometric primitives
# ---------------------------------------------------------------------------

def rotate(points, angle_rad, center=(0.0, 0.0)):
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    R = np.array([[c, -s], [s, c]])
    return (points - np.asarray(center)) @ R.T + np.asarray(center)


def unit(v):
    return v / np.linalg.norm(v)


# ---------------------------------------------------------------------------
# NACA 4-digit airfoil (unit chord, LE at origin, TE near (1,0))
# ---------------------------------------------------------------------------

def naca4(code, te_thickness=0.0, n_panels=160):
    """Selig-ordered coordinates with optional blunt TE."""
    m = int(code[0]) / 100.0
    p = int(code[1]) / 10.0
    t = int(code[2:4]) / 100.0

    def surfaces(x):
        yt = 5 * t * (0.2969 * np.sqrt(x) - 0.1260 * x
                      - 0.3516 * x ** 2 + 0.2843 * x ** 3 - 0.1015 * x ** 4)
        yc = np.where(x < p,
                      m / p ** 2 * (2 * p * x - x ** 2),
                      m / (1 - p) ** 2 * (1 - 2 * p + 2 * p * x - x ** 2))
        dyc = np.where(x < p,
                       2 * m / p ** 2 * (p - x),
                       2 * m / (1 - p) ** 2 * (p - x))
        theta = np.arctan(dyc)
        return (x - yt * np.sin(theta), yc + yt * np.cos(theta),
                x + yt * np.sin(theta), yc - yt * np.cos(theta))

    x_scan = np.linspace(0.5, 1.0, 5001)
    xu, yu, xl, yl = surfaces(x_scan)
    gap = np.hypot(xu - xl, yu - yl)
    x_cut = np.interp(te_thickness, gap[::-1], x_scan[::-1])

    beta = np.linspace(0.0, math.pi, n_panels // 2 + 1)
    x = 0.5 * (1.0 - np.cos(beta)) * x_cut
    xu, yu, xl, yl = surfaces(x)
    upper = np.column_stack([xu[::-1], yu[::-1]])
    lower = np.column_stack([xl[1:], yl[1:]])
    return np.vstack([upper, lower])


# ---------------------------------------------------------------------------
# Anchor a unit-chord airfoil between explicit LE/TE coordinates
# ---------------------------------------------------------------------------

def anchor_airfoil(raw, target_le, target_te):
    target_le = np.asarray(target_le, dtype=float)
    target_te = np.asarray(target_te, dtype=float)
    raw_le = raw[np.argmin(raw[:, 0])]
    raw_te = 0.5 * (raw[0] + raw[-1])
    raw_vec = raw_te - raw_le
    tgt_vec = target_te - target_le
    scale = np.linalg.norm(tgt_vec) / np.linalg.norm(raw_vec)
    angle = math.atan2(tgt_vec[1], tgt_vec[0]) - math.atan2(raw_vec[1], raw_vec[0])
    return rotate(raw - raw_le, angle) * scale + target_le


# ---------------------------------------------------------------------------
# Cove fillet
# ---------------------------------------------------------------------------

def fillet_corner(p_in, vertex, p_out, radius, n_arc=40):
    v1 = unit(p_in - vertex)
    v2 = unit(p_out - vertex)
    alpha = math.acos(np.clip(np.dot(v1, v2), -1.0, 1.0))
    half = alpha / 2.0
    t_in = vertex + radius / math.tan(half) * v1
    t_out = vertex + radius / math.tan(half) * v2
    center = vertex + radius / math.sin(half) * unit(v1 + v2)

    a_in = math.atan2(t_in[1] - center[1], t_in[0] - center[0])
    a_out = math.atan2(t_out[1] - center[1], t_out[0] - center[0])
    delta = (a_out - a_in + math.pi) % (2 * math.pi) - math.pi
    angles = a_in + np.linspace(0.0, delta, n_arc)
    arc = np.column_stack([center[0] + radius * np.cos(angles),
                           center[1] + radius * np.sin(angles)])
    return t_in, arc, t_out


# ---------------------------------------------------------------------------
# Element builders — each returns (segments, flat_contour).
#
# `segments` is a list of (kind, points) tuples consumed by write_udc.
# The first point of the first segment is the sketch start (skbeg). Each
# `linseg` adds one segment from current to the next point; consecutive
# `spline` calls merge into one B-spline through their control points.
#
# `flat_contour` is the Selig-ordered closed polyline for plotting and
# the .dat export.
# ---------------------------------------------------------------------------

def build_main_wing(airfoil_cfg, cutouts, blunt_thickness):
    upper = np.asarray(airfoil_cfg['upper'], dtype=float)
    lower = np.asarray(airfoil_cfg['lower'], dtype=float)
    x_lo_cut = cutouts['lower_cut_x']
    x_up_cut = cutouts['upper_lip_cut_x']
    vertex = np.array([cutouts['cove_vertex_x'], cutouts['cove_vertex_y']])
    r_fillet = cutouts['cove_fillet_radius']

    y_up_at_cut = np.interp(x_up_cut, upper[:, 0], upper[:, 1])
    y_lo_at_cut = np.interp(x_lo_cut, lower[:, 0], lower[:, 1])
    up_kept = np.vstack([upper[upper[:, 0] < x_up_cut], [[x_up_cut, y_up_at_cut]]])
    lo_kept = np.vstack([lower[lower[:, 0] < x_lo_cut], [[x_lo_cut, y_lo_at_cut]]])
    lip_bottom = np.array([x_up_cut, y_up_at_cut - blunt_thickness])
    lower_cut_pt = np.array([x_lo_cut, y_lo_at_cut])

    # fillet_corner returns tangent points on each leg in the order
    # (toward p_in, arc, toward p_out). Here p_in = lip_bottom (upper side)
    # and p_out = lower_cut_pt (lower side); arc traces lip-side → lower-side.
    t_upper, arc, t_lower = fillet_corner(lip_bottom, vertex, lower_cut_pt, r_fillet)
    main_contour = np.vstack([up_kept[::-1], lo_kept[1:]])  # upper-lip-tip → LE → lower-cut-pt
    arc_lower_to_upper = arc[::-1]                          # lower-side → upper-side

    segments = [
        ('spline', main_contour),                  # main outer surface
        ('linseg', np.array([t_lower])),           # flat lower cove edge
        ('spline', arc_lower_to_upper[1:]),        # filleted cove vertex
        ('linseg', np.array([lip_bottom])),        # flat upper cove edge
    ]                                              # (implicit) blunt-TE close
    flat = np.vstack([main_contour, [t_lower], arc_lower_to_upper[1:], [lip_bottom]])
    return segments, flat


def build_naca_element(code, te_thickness, target_le, target_te, n_panels=160):
    """Vane / aft flap: anchored NACA airfoil split into upper-surface and
    lower-surface splines, joined by a zero-length linseg break at the LE,
    closed by a linseg across the blunt TE."""
    raw = naca4(code, te_thickness, n_panels)
    anchored = anchor_airfoil(raw, target_le, target_te)
    le_idx = n_panels // 2  # exact LE index by construction (cosine-clustered)

    segments = [
        ('spline', anchored[:le_idx + 1]),              # upper TE corner → LE
        ('linseg', anchored[le_idx:le_idx + 1]),        # zero-length break (LE → LE)
        ('spline', anchored[le_idx + 1:]),              # next pt → lower TE corner
    ]
    return segments, anchored


def build_tail(airfoil_cfg):
    """Inverted LS(1)-0417 with negative camber (downforce at AoA = 0)."""
    upper = np.asarray(airfoil_cfg['upper'], dtype=float)
    lower = np.asarray(airfoil_cfg['lower'], dtype=float)
    new_upper = np.column_stack([lower[:, 0], -lower[:, 1]])
    new_lower = np.column_stack([upper[:, 0], -upper[:, 1]])
    contour = np.vstack([new_upper[::-1], new_lower[1:]])
    le_idx = np.argmin(contour[:, 0])

    segments = [
        ('spline', contour[:le_idx + 1]),
        ('linseg', contour[le_idx:le_idx + 1]),
        ('spline', contour[le_idx + 1:]),
    ]
    return segments, contour


# ---------------------------------------------------------------------------
# Fowler kinematics
# ---------------------------------------------------------------------------

def apply_kinematics(points, pivot, dx, dy, rotation_deg):
    return rotate(points, math.radians(rotation_deg), center=pivot) + np.array([dx, dy])


# ---------------------------------------------------------------------------
# I/O
# ---------------------------------------------------------------------------

def write_selig(path, name, points):
    with path.open('w') as f:
        f.write(f"{name}\n")
        for x, y in points:
            f.write(f"{x: .6f} {y: .6f}\n")


def write_udc(path, segments, description):
    """Closed planar sketch UDC. `segments` is a list of (kind, points)
    tuples; the first point of the first segment is the sketch start.
    Sketch is auto-closed by linseg back to that start."""
    head_kind, head_pts = segments[0]
    x0, y0 = head_pts[0]
    with path.open('w') as f:
        f.write(f"# {path.name} — {description}\n")
        f.write("# Generated by build_estol_geometry.py — DO NOT EDIT BY HAND.\n\n")
        f.write(f"skbeg     {x0: .6f}  {y0: .6f}   0\n")
        for x, y in head_pts[1:]:
            f.write(f"   {head_kind} {x: .6f}  {y: .6f}   0\n")
        for kind, pts in segments[1:]:
            for x, y in pts:
                f.write(f"   {kind} {x: .6f}  {y: .6f}   0\n")
        f.write(f"   linseg {x0: .6f}  {y0: .6f}   0\n")
        f.write("skend\n\nend\n")


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_configurations(main_wing, elements_by_phase, out_path,
                        xlim=(0.0, 1.4), ylim=(-0.4, 0.3)):
    fig, axes = plt.subplots(len(elements_by_phase), 1, figsize=(10, 10),
                             sharex=True, sharey=True)
    for ax, (phase, elements) in zip(axes, elements_by_phase.items()):
        ax.fill(main_wing[:, 0], main_wing[:, 1],
                facecolor='#cfd8e6', edgecolor='#1f3a68', lw=1.3, label='Main wing')
        ax.fill(elements['vane'][:, 0], elements['vane'][:, 1],
                facecolor='#fbd4b4', edgecolor='#9c3d1a', lw=1.2, label='Vane')
        ax.fill(elements['flap'][:, 0], elements['flap'][:, 1],
                facecolor='#cfe9c8', edgecolor='#2c6b2c', lw=1.2, label='Aft flap')
        ax.set(xlim=xlim, ylim=ylim, ylabel='y / c')
        ax.set_aspect('equal', adjustable='box')
        ax.grid(True, alpha=0.3)
        ax.set_title(f"{phase.capitalize()} configuration", loc='left', fontsize=11)
        ax.legend(loc='lower right', fontsize=8, framealpha=0.9)
    axes[-1].set_xlabel('x / c')
    fig.suptitle('3-element STOL airfoil: Fowler kinematics validation', fontsize=13)
    fig.subplots_adjust(left=0.08, right=0.97, top=0.94, bottom=0.06, hspace=0.25)
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------

def main(cfg_path):
    cfg = yaml.safe_load(cfg_path.read_text())
    here = cfg_path.parent
    blunt = cfg['trailing_edge']['blunt_thickness']
    vane_cfg, flap_cfg = cfg['vane'], cfg['aft_flap']
    vane_chord = np.linalg.norm(np.array(vane_cfg['stowed_te']) - np.array(vane_cfg['stowed_le']))
    flap_chord = np.linalg.norm(np.array(flap_cfg['stowed_te']) - np.array(flap_cfg['stowed_le']))

    main_segs, main_flat = build_main_wing(cfg['airfoils']['ls1_0417'],
                                            cfg['main_wing']['cutouts'], blunt)
    vane_segs, vane_flat = build_naca_element(vane_cfg['naca'], blunt / vane_chord,
                                               vane_cfg['stowed_le'], vane_cfg['stowed_te'])
    flap_segs, flap_flat = build_naca_element(flap_cfg['naca'], blunt / flap_chord,
                                               flap_cfg['stowed_le'], flap_cfg['stowed_te'])
    tail_segs, tail_flat = build_tail(cfg['airfoils']['ls1_0417'])

    write_selig(here / 'main_wing.dat',   'main_wing_ls1_0417_coved',           main_flat)
    write_selig(here / 'vane_stowed.dat', f"vane_naca{vane_cfg['naca']}_stowed", vane_flat)
    write_selig(here / 'flap_stowed.dat', f"flap_naca{flap_cfg['naca']}_stowed", flap_flat)
    write_selig(here / 'tail.dat',        'tail_ls1_0417_inverted',              tail_flat)

    write_udc(here / 'main_wing.udc', main_segs, "coved LS(1)-0417 main wing")
    write_udc(here / 'vane.udc',      vane_segs, f"NACA {vane_cfg['naca']} vane (stowed)")
    write_udc(here / 'flap.udc',      flap_segs, f"NACA {flap_cfg['naca']} aft flap (stowed)")
    write_udc(here / 'tail.udc',      tail_segs, "inverted LS(1)-0417 H-tail")

    pivot = cfg['kinematics']['pivot_point']
    elements_by_phase = {
        phase: {'vane': apply_kinematics(vane_flat, pivot, p['dx'], p['dy'], p['rotation_deg']),
                'flap': apply_kinematics(flap_flat, pivot, p['dx'], p['dy'], p['rotation_deg'])}
        for phase, p in cfg['kinematics']['phases'].items()
    }
    plot_path = here / 'estol_configurations.png'
    plot_configurations(main_flat, elements_by_phase, plot_path)

    slot_gap = np.linalg.norm(np.array(flap_cfg['stowed_le']) - np.array(vane_cfg['stowed_te']))
    print(f"Wrote {here / '*.dat'}, *.udc, {plot_path.name}")
    print(f"  slot gap (vane TE → flap LE): {slot_gap:.4f} c (invariant under rigid kinematics)")


if __name__ == '__main__':
    main(Path(__file__).resolve().parent / 'estol_config.yaml')
