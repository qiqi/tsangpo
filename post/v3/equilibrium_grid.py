"""Equilibrium grid for the v3 short-boom design.

Mirrors post/v2_gapped/equilibrium_grid.py exactly (same trim solver),
but loads SPECs from post/v3/plot_*_sensitivities.py.  Grids shifted to
bracket v3 BO trim points (α=+15° takeoff, α=+25° landing).

Writes:
    post/out/v3/equilibrium_grid.csv
    post/out/v3/equilibrium_grid.md
"""
from __future__ import annotations

import csv, importlib.util, sys
from math import atan2, cos, degrees, isnan, radians, sin, sqrt
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "post"))
import params as P
from _phase_plot import (PhaseSpec, by_sweep, baseline_of, discover_sweeps,
                         fit_linear, load_or_fetch)


def load_spec(phase: str) -> PhaseSpec:
    pp = REPO / "post" / "v3" / f"plot_{phase}_sensitivities.py"
    s  = importlib.util.spec_from_file_location(f"spec_{phase}", pp)
    m  = importlib.util.module_from_spec(s); s.loader.exec_module(m)
    return m.SPEC


# v3 BO trim points: cruise α=+7°, takeoff α=+15°, landing α=+25°.
# Grids span ±5° around BO and a 3-point thrust ladder bracketing BO T_mult.
GRIDS = {
    "cruise":  dict(alphas=( 5.0,  7.0,  9.0), Ts=( 1.0,  2.0,  3.0)),
    "takeoff": dict(alphas=( 9.0, 12.0, 15.0), Ts=( 8.0, 15.0, 22.0)),
    "landing": dict(alphas=(15.0, 20.0, 25.0), Ts=( 5.0,  9.0, 13.0)),
}

KNOTS_PER_M_S = 1.0 / 0.5144


def compute_sensitivities(spec: PhaseSpec) -> tuple[dict, dict]:
    csv_path = spec.out_dir / f"{spec.phase_name}_sweep_data.csv"
    rows = load_or_fetch(
        csv_path, refresh=False,
        sweeps_factory=lambda: discover_sweeps(
            spec.project_id, spec.alpha_b, spec.theta_ht_b, spec.T_b),
        rho_a2_L2=spec.rho_a2_L2,
    )
    a, da = by_sweep(rows, "alpha",  spec.qS)
    h, dh = by_sweep(rows, "htail",  spec.qS)
    t, dt = by_sweep(rows, "thrust", spec.qS)
    Ma, Mh, Mt = spec.mask_alpha(a), spec.mask_htail(h), spec.mask_thrust(t)
    slopes = {
        "dCL_da":  fit_linear(a, da["CL"],     Ma)[0],
        "dCD_da":  fit_linear(a, da["CD"],     Ma)[0],
        "dCMy_da": fit_linear(a, da["CMy_CG"], Ma)[0],
        "dCL_dh":  fit_linear(h, dh["CL"],     Mh)[0],
        "dCD_dh":  fit_linear(h, dh["CD"],     Mh)[0],
        "dCMy_dh": fit_linear(h, dh["CMy_CG"], Mh)[0],
        "dCL_dT":  fit_linear(t, dt["CL"],     Mt)[0],
        "dCD_dT":  fit_linear(t, dt["CD"],     Mt)[0],
        "dCMy_dT": fit_linear(t, dt["CMy_CG"], Mt)[0],
        "dCT_dT":  fit_linear(t, dt["CT_delivered"], Mt)[0],
    }
    baselines = {
        "CL_b":  baseline_of(da["CL"],     a, spec.alpha_b),
        "CD_b":  baseline_of(da["CD"],     a, spec.alpha_b),
        "CMy_b": baseline_of(da["CMy_CG"], a, spec.alpha_b),
        "CT_b":  baseline_of(dt["CT_delivered"], t, spec.T_b),
    }
    return slopes, baselines


def solve_equilibrium(spec: PhaseSpec, slopes: dict, baselines: dict,
                       alpha_deg: float, T_mult: float) -> dict:
    da = alpha_deg - spec.alpha_b
    dT = T_mult    - spec.T_b
    theta_ht_deg = spec.theta_ht_b - (
        baselines["CMy_b"] + slopes["dCMy_da"] * da + slopes["dCMy_dT"] * dT
    ) / slopes["dCMy_dh"]
    dh = theta_ht_deg - spec.theta_ht_b
    CL = baselines["CL_b"] + slopes["dCL_da"]*da + slopes["dCL_dh"]*dh + slopes["dCL_dT"]*dT
    CD = baselines["CD_b"] + slopes["dCD_da"]*da + slopes["dCD_dh"]*dh + slopes["dCD_dT"]*dT
    CT = baselines["CT_b"] + slopes["dCT_dT"]*dT
    a_rad = radians(alpha_deg)
    K = spec.qS / P.W_GROSS_N
    A = CL**2 + CD**2
    B = 2.0 * CT * (CL * sin(a_rad) - CD * cos(a_rad))
    C = CT**2 - 1.0 / (K**2)
    disc = B*B - 4*A*C
    if disc < 0:
        return dict(alpha=alpha_deg, T_mult=T_mult, theta_ht=theta_ht_deg,
                    CL=CL, CD=CD, CT_del=CT,
                    V_m_s=float("nan"), V_kt=float("nan"),
                    gamma_deg=float("nan"),
                    F_thrust_N=float("nan"), T_over_L=float("nan"),
                    note="no real positive root for V — design infeasible")
    x = (-B + sqrt(disc)) / (2.0 * A)
    if x <= 0:
        x = (-B - sqrt(disc)) / (2.0 * A)
    V = spec.velocity * sqrt(x) if x > 0 else float("nan")
    cos_g = K * (CL * x + CT * sin(a_rad))
    sin_g = K * (CT * cos(a_rad) - CD * x)
    gamma_deg = degrees(atan2(sin_g, cos_g))
    F_thrust_N = CT * spec.qS
    CL_total   = CL + CT * sin(radians(alpha_deg))
    q_trim   = 0.5 * spec.rho * V * V if V > 0 else float("nan")
    L_total_N = CL_total * q_trim * P.WING_AREA_M2
    T_over_L  = F_thrust_N / L_total_N if L_total_N > 0 else float("nan")
    return dict(
        alpha=alpha_deg, T_mult=T_mult, theta_ht=theta_ht_deg,
        CL=CL, CD=CD, CT_del=CT,
        V_m_s=V, V_kt=V * KNOTS_PER_M_S,
        gamma_deg=gamma_deg,
        F_thrust_N=F_thrust_N, T_over_L=T_over_L,
        note="",
    )


def main():
    out_csv = REPO / "post" / "out" / "v3" / "equilibrium_grid.csv"
    out_md  = REPO / "post" / "out" / "v3" / "equilibrium_grid.md"
    out_csv.parent.mkdir(parents=True, exist_ok=True)

    rows: list[dict] = []
    per_phase_summary: dict = {}
    for phase in ("cruise", "takeoff", "landing"):
        spec = load_spec(phase)
        slopes, baselines = compute_sensitivities(spec)
        per_phase_summary[phase] = (spec, slopes, baselines)
        grid = GRIDS[phase]
        for a in grid["alphas"]:
            for T in grid["Ts"]:
                r = solve_equilibrium(spec, slopes, baselines, a, T)
                r["phase"] = phase
                r["V_b"]   = spec.velocity
                r["alpha_b"] = spec.alpha_b
                r["theta_ht_b"] = spec.theta_ht_b
                r["T_b"] = spec.T_b
                rows.append(r)

    fields = ["phase", "alpha", "T_mult", "theta_ht", "CL", "CD", "CT_del",
              "V_m_s", "V_kt", "gamma_deg", "F_thrust_N", "T_over_L",
              "V_b", "alpha_b", "theta_ht_b", "T_b", "note"]
    with out_csv.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields); w.writeheader()
        for r in rows: w.writerow({k: r.get(k, "") for k in fields})
    print(f"wrote {out_csv}")

    lines = [
        "# v3 short-boom equilibrium grid",
        "",
        "Same trim solver as `post/v2_gapped/equilibrium_grid.py`, but slopes",
        "and baselines are taken from the v3 short-boom CFD sweeps.  Grids",
        "shifted to bracket v3 BO trim points (α=+15° takeoff, α=+25° landing).",
        "",
    ]
    for phase in ("cruise", "takeoff", "landing"):
        spec, slopes, baselines = per_phase_summary[phase]
        lines += [
            f"## {phase} (BO α={spec.alpha_b:+.0f}°, θ_ht={spec.theta_ht_b:+.0f}°, "
            f"T_mult={spec.T_b:+.2f}, V_b={spec.velocity:.2f} m/s)",
            "",
            "| slope     | value      |",
            "|-----------|------------|",
        ]
        for k, v in slopes.items():
            lines.append(f"| `{k}` | {v:+.5f} |")
        lines += [
            "",
            "| α [deg] | T_mult | θ_htail [deg] | CL | CD | CT_del | V [kt] | γ [deg] | F_thrust [N] | T/L | note |",
            "|---------|--------|---------------|----|----|--------|--------|---------|--------------|-----|------|",
        ]
        for r in [r for r in rows if r["phase"] == phase]:
            vkt = "{:6.2f}".format(r["V_kt"])     if not isnan(r["V_kt"])     else "  nan "
            g   = "{:+6.2f}".format(r["gamma_deg"]) if not isnan(r["gamma_deg"]) else " nan  "
            F   = "{:7.1f}".format(r["F_thrust_N"]) if not isnan(r["F_thrust_N"]) else " nan   "
            tw  = "{:5.3f}".format(r["T_over_L"]) if not isnan(r["T_over_L"]) else " nan "
            lines.append(
                f"| {r['alpha']:+5.1f} | {r['T_mult']:+5.1f} | {r['theta_ht']:+6.2f}        |"
                f" {r['CL']:+.3f} | {r['CD']:+.3f} | {r['CT_del']:+.3f} |"
                f" {vkt} | {g} | {F} | {tw} | {r['note']} |"
            )
        lines.append("")
    out_md.write_text("\n".join(lines))
    print(f"wrote {out_md}")


if __name__ == "__main__":
    main()
