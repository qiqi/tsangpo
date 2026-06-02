"""Per-case diagnostic plots for the pitch_eq V × θ_ht sweep.

For every chunk1 and chunk2 case in the grid, build a multi-pane figure
showing every per-pseudo-step quantity we can measure: airframe UDD state,
forces and moments, per-surface CL breakdown, actuator-disk forces,
residuals, CFL, and min/max state.

The x-axis is the cumulative pseudo-iteration counter from case start —
not physical time — so the resolution captures within-step convergence.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import flow360 as fl
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

REPO = Path(__file__).resolve().parents[2]
IDS  = json.loads((REPO / "post" / "v3" / "unsteady_project_ids.json").read_text())
OUT  = REPO / "post" / "v3" / "phugoid_runs" / "landing" / "pitch_eq_per_case_diags"
OUT.mkdir(parents=True, exist_ok=True)

# ---- Case catalog -----------------------------------------------------------
# Only the chunk2_qs_D30k forks — the production recipe.  Earlier integrator
# variants (chunk1 implicit, chunk2 with dt=5, chunk2 with dt=2, QS at D=10k)
# are no longer kept on disk; their cases still live in Flow360 if needed.
CASES: list[tuple[str, float, float, str, str]] = []
for th_lbl, th in [('thN3p67', -3.67), ('thN2p00', -2.0), ('thP0p00', 0.0),
                   ('thP1p00',  1.0),  ('thP3p00',  3.0)]:
    for V_lbl, V in [('V10p00', 10.0), ('V12p86', 12.86), ('V16p00', 16.0)]:
        key = f"{th_lbl}_{V_lbl}"
        suffix = 'chunk2_qs_D30k_case'
        if f"pitch_eq_{key}_{suffix}" in IDS:
            CASES.append((key, th, V, 'chunk2_qs_D30k', suffix))

print(f"Cases to plot: {len(CASES)}")

# ---- File loaders -----------------------------------------------------------
RESULTS = [
    'udd_pitch_cyl3int_airframe_v2.csv',
    'total_forces_v2.csv',
    'surface_forces_v2.csv',
    'actuatorDisk_force_coefficients_v2.csv',
    'linear_residual_v2.csv',
    'nonlinear_residual_v2.csv',
    'minmax_state_v2.csv',
    'cfl_v2.csv',
]
def load_case(case_id: str) -> dict:
    target = Path(f"/tmp/diag_{case_id[:13]}/")
    target.mkdir(exist_ok=True)
    data = {}
    for fn in RESULTS:
        local = target / fn
        if not local.exists():
            try:
                c = fl.Case.from_cloud(case_id=case_id)
                c._download_file(f"results/{fn}", to_folder=str(target), overwrite=True)
            except Exception as e:
                print(f"  miss {fn}: {e}")
                data[fn] = None
                continue
        data[fn] = pd.read_csv(local, sep=r"\s*,\s*", engine="python")
    # also pull flow360.json once per case so we can read airframe UDD constants
    if not (target / "flow360.json").exists():
        try:
            c = fl.Case.from_cloud(case_id=case_id)
            c._download_file("flow360.json", to_folder=str(target), overwrite=True)
        except Exception:
            pass
    try:
        data["constants"] = next(
            u["constants"] for u in json.loads((target/"flow360.json").read_text())
            ["userDefinedDynamics"] if u["dynamicsName"] == "pitch_cyl3int_airframe")
    except Exception:
        data["constants"] = {}
    return data


def make_iter(df: pd.DataFrame) -> np.ndarray:
    """Cumulative pseudo-iteration counter (one per row)."""
    return np.arange(len(df))


# ---- Plot helpers -----------------------------------------------------------
def safe_get(df: pd.DataFrame | None, col: str) -> np.ndarray | None:
    if df is None: return None
    return df[col].to_numpy() if col in df.columns else None

def plot_case(key: str, th: float, V: float, chunk_lbl: str, case_id: str) -> Path:
    data = load_case(case_id)
    udd  = data['udd_pitch_cyl3int_airframe_v2.csv']
    tot  = data['total_forces_v2.csv']
    surf = data['surface_forces_v2.csv']
    ad   = data['actuatorDisk_force_coefficients_v2.csv']
    lin  = data['linear_residual_v2.csv']
    nl   = data['nonlinear_residual_v2.csv']
    mm   = data['minmax_state_v2.csv']
    cfl  = data['cfl_v2.csv']
    consts = data.get('constants', {})
    # default in case constants are missing
    ratio = 1.0
    cmy_ad_coef = 0.0
    if tot is not None and udd is not None and consts:
        end_tot = tot.groupby('physical_step').tail(1).reset_index(drop=True)
        end_udd = udd.groupby('physical_step').tail(1).reset_index(drop=True)
        with np.errstate(divide='ignore', invalid='ignore'):
            r = end_tot['CMy'].to_numpy() / end_udd['momentY'].to_numpy()
            r = r[np.isfinite(r) & (np.abs(r) < 1000)]
            ratio = float(np.median(r)) if len(r) else 1.0
        cmy_ad_coef = -consts['prop_z_m'] * consts['T_disk_nd'] * ratio

    # Use UDD as the "primary" length reference; other CSVs should match row count
    n_udd = len(udd) if udd is not None else 0
    x_udd = make_iter(udd) if udd is not None else None
    x_tot = make_iter(tot) if tot is not None else None
    x_surf = make_iter(surf) if surf is not None else None

    fig, axes = plt.subplots(4, 3, figsize=(17, 12))
    fig.suptitle(
        f"pitch_eq diag: θ_ht={th:+.2f}°  V={V:.2f} m/s  [{chunk_lbl}]   "
        f"case {case_id[:13]}",
        fontsize=12, y=0.995,
    )

    # 1. θ_body from UDD state[4]
    ax = axes[0,0]
    if udd is not None:
        ax.plot(x_udd, np.degrees(udd['state[4]']), lw=0.7)
        # mark physical-step boundaries
        for ps in udd['physical_step'].unique()[1:]:
            idx = udd.index[udd['physical_step']==ps][0]
            ax.axvline(idx, color='gray', alpha=0.2, lw=0.5)
    ax.set_title('θ_body  (deg)  [UDD state[4]]'); ax.grid(alpha=0.3)

    # 2. q (state[5]) — should hover near 0 for quasi-static
    ax = axes[0,1]
    if udd is not None:
        ax.plot(x_udd, udd['state[5]'], lw=0.7)
    ax.set_title('q  (state[5], solver nondim)'); ax.grid(alpha=0.3)
    ax.axhline(0, color='k', alpha=0.3)

    # 3. CL, CD total (overlaid)
    ax = axes[0,2]
    if tot is not None:
        ax.plot(x_tot, tot['CL'], lw=0.6, label='CL', color='C0')
        ax.plot(x_tot, tot['CD'], lw=0.6, label='CD', color='C3')
    ax.set_title('CL, CD  (total)'); ax.legend(fontsize=8); ax.grid(alpha=0.3)

    # 4. CMy_wall  +  AD reaction (both in airframe-coef)  +  Q_TORQUE×I_yy
    # All in the SAME unit so the moment balance is visible.  The sign of
    # the green (wall + AD) curve drives θ — note how it can flip even
    # while the wall-only CMy stays one-signed.
    ax = axes[1,0]
    if tot is not None:
        ax.plot(x_tot, tot['CMy'].to_numpy(), lw=0.6, color='C0',
                label='CMy_wall (total)')
        if consts:
            ax.axhline(cmy_ad_coef, color='C3', ls='--', lw=1.0,
                        label=f'CMy_AD = −prop_z·T_disk·{ratio:.1f} = {cmy_ad_coef:+.3f}')
            qt_coef = tot['CMy'].to_numpy() + cmy_ad_coef
            ax.plot(x_tot, qt_coef, lw=0.7, color='C2', alpha=0.85,
                    label='Q_TORQUE·I_yy  =  wall + AD')
    ax.set_title('Q_TORQUE in airframe-coef\n(wall + AD reaction; sign of total drives θ)')
    ax.legend(fontsize=7, loc='best'); ax.grid(alpha=0.3)
    ax.axhline(0, color='k', alpha=0.4)

    # 5. Per-surface CL: main_wing, htail, vanes, aft_flap, fuselage
    ax = axes[1,1]
    if surf is not None:
        for label, col_match in [
            ('main_wing', 'main_wing_CL'),
            ('htail',     'htail_CL'),
            ('vane_R',    'vane_right_CL'),
            ('vane_L',    'vane_left_CL'),
            ('flap_R',    'aft_flap_right_CL'),
            ('flap_L',    'aft_flap_left_CL'),
        ]:
            cols = [c for c in surf.columns if c.endswith(col_match)]
            if cols:
                ax.plot(x_surf, surf[cols[0]], lw=0.6, label=label, alpha=0.8)
    ax.set_title('Per-surface CL'); ax.legend(fontsize=7, ncol=2); ax.grid(alpha=0.3)

    # 6. Per-surface CMy
    ax = axes[1,2]
    if surf is not None:
        for label, col_match in [
            ('main_wing', 'main_wing_CMy'),
            ('htail',     'htail_CMy'),
            ('flap_R',    'aft_flap_right_CMy'),
            ('flap_L',    'aft_flap_left_CMy'),
        ]:
            cols = [c for c in surf.columns if c.endswith(col_match)]
            if cols:
                ax.plot(x_surf, surf[cols[0]], lw=0.6, label=label, alpha=0.8)
    ax.set_title('Per-surface CMy'); ax.legend(fontsize=7); ax.grid(alpha=0.3)
    ax.axhline(0, color='k', alpha=0.3)

    # 7. AD axial thrust per disk (sum of all 10 disks for total) — CFx is body-axis x
    ax = axes[2,0]
    if ad is not None:
        x_ad = make_iter(ad)
        tot_cfx = sum(ad[f'Disk{i}_CFx'] for i in range(10) if f'Disk{i}_CFx' in ad.columns)
        ax.plot(x_ad, tot_cfx, lw=0.7, color='C2', label='Σ Disk_CFx')
    ax.set_title('Actuator disk total CFx'); ax.legend(fontsize=8); ax.grid(alpha=0.3)

    # 8. AD total CMy: raw rotor-units (left) vs UDD effective coef (right)
    ax = axes[2,1]
    if ad is not None:
        x_ad = make_iter(ad)
        tot_cmy_raw = sum(ad[f'Disk{i}_CMy'] for i in range(10)
                          if f'Disk{i}_CMy' in ad.columns)
        ax.plot(x_ad, tot_cmy_raw, lw=0.7, color='C2', alpha=0.6,
                label='Σ Disk_CMy raw (rotor units)')
        ax.legend(fontsize=7, loc='upper right')
        if consts:
            ax2 = ax.twinx()
            ax2.axhline(cmy_ad_coef, color='C3', ls='--', lw=1.2,
                        label=f'UDD effective (airframe coef) = {cmy_ad_coef:+.3f}')
            ax2.set_ylabel('airframe coef', color='C3', fontsize=8)
            ax2.tick_params(axis='y', labelcolor='C3', labelsize=7)
            ax2.legend(fontsize=7, loc='lower right')
    ax.set_title('Actuator disk total CMy  (two normalizations)')
    ax.grid(alpha=0.3); ax.axhline(0, color='k', alpha=0.3)

    # 9. CG world position (state[0], state[1])
    ax = axes[2,2]
    if udd is not None:
        ax.plot(x_udd, udd['state[0]'], lw=0.6, label='CG_x = state[0]')
        ax.plot(x_udd, udd['state[1]'], lw=0.6, label='CG_z = state[1]')
        ax.plot(x_udd, np.degrees(udd['state[6]']), lw=0.5, label='link2 (deg)',
                color='C3', alpha=0.6)
        ax.plot(x_udd, np.degrees(udd['state[7]']), lw=0.5, label='link1 (deg)',
                color='C4', alpha=0.6)
    ax.set_title('CG world (m) + link angles (deg)'); ax.legend(fontsize=7); ax.grid(alpha=0.3)

    # 10. Nonlinear residuals (log scale)
    ax = axes[3,0]
    if nl is not None:
        x_nl = make_iter(nl)
        for col, lbl in [('0_cont','cont'), ('1_momx','momx'),
                         ('2_momy','momy'), ('3_momz','momz'),
                         ('4_energ','energ'), ('5_nuHat','nuHat')]:
            if col in nl.columns:
                ax.semilogy(x_nl, nl[col].clip(lower=1e-20), lw=0.5, label=lbl, alpha=0.8)
    ax.set_title('Nonlinear residuals (log)'); ax.legend(fontsize=7, ncol=2); ax.grid(alpha=0.3)

    # 11. CFL number
    ax = axes[3,1]
    if cfl is not None:
        x_cfl = make_iter(cfl)
        if '0_NavierStokes_cfl' in cfl.columns:
            ax.plot(x_cfl, cfl['0_NavierStokes_cfl'], lw=0.7, label='NS CFL')
        if '1_SpalartAllmaras_cfl' in cfl.columns:
            ax.plot(x_cfl, cfl['1_SpalartAllmaras_cfl'], lw=0.7, label='SA CFL')
    ax.set_title('CFL number'); ax.legend(fontsize=8); ax.grid(alpha=0.3)

    # 12. min ρ, min p, max |u| (sanity for physical realizability)
    ax = axes[3,2]
    if mm is not None:
        x_mm = make_iter(mm)
        ax.plot(x_mm, mm['min_rho'], lw=0.6, label='min ρ', color='C0')
        ax.plot(x_mm, mm['min_p'],   lw=0.6, label='min p', color='C3')
        ax.plot(x_mm, mm['max_umag'],lw=0.6, label='max |u|', color='C2')
    ax.set_title('min ρ, min p, max |u|'); ax.legend(fontsize=8); ax.grid(alpha=0.3)

    # common x label
    for ax in axes[-1, :]:
        ax.set_xlabel('cumulative pseudo-iteration')

    plt.tight_layout(rect=(0, 0, 1, 0.985))
    fn = OUT / f"pitch_eq_{key}_{chunk_lbl}.png"
    fig.savefig(fn, dpi=100)
    plt.close(fig)
    return fn


# ---- Run --------------------------------------------------------------------
if __name__ == "__main__":
    for key, th, V, chunk_lbl, suffix in CASES:
        case_id = IDS[f"pitch_eq_{key}_{suffix}"]
        try:
            fn = plot_case(key, th, V, chunk_lbl, case_id)
            print(f"  ✓ {fn.name}")
        except Exception as e:
            print(f"  ✗ {key} {chunk_lbl}: {e}")
    print(f"\nAll plots in {OUT}")
