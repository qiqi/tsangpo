"""
Refined-equilibrium follow-up for the gapped LOW-htail (gap40) v2
campaign.  Per phase (cruise / takeoff / landing):

  1. Verify all sweep forks in the phase project are no longer pending —
     COMPLETED or STOPPED-dedup is fine.  Skip if any are still running.
  2. Skip if the refined nominal case already exists in the project
     (idempotent — safe to re-fire from a cron).
  3. Fit sensitivities (using the masks the per-phase plotter uses) and
     solve the 3×3 trim system → equilibrium (α*, θ_ht*, T*).
  4. Submit 7 cases at the equilibrium:
       1 nominal,
       2 α perturbations  (±Δα),
       2 θ_ht perturbations (±Δθ_ht),
       2 T_mult perturbations (±ΔT).
     All as forks of the phase's parent case so they reuse its mesh.

Perturbation sizes are chosen ≈ 25 % of the masked-fit range in each
sweep — small enough to stay inside the unstalled linear regime in the
existing data, large enough that signal/noise is good.

Run from cron:
    python3 flow360/submit_gapped_low_htail_followup.py
Or per phase:
    python3 flow360/submit_gapped_low_htail_followup.py cruise
"""
from __future__ import annotations
import importlib.util
import sys
from math import radians
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "post"))
import cfd_setup as C
import params as P
import flow360 as fl
from _phase_plot import (PhaseSpec, by_sweep, baseline_of, fit_linear,
                         discover_sweeps, fetch_rows)

# Perturbation sizes (same magnitudes across phases — Wu Wei: don't over-
# parametrise).  Justified by the linear-regime widths in the v2-cont
# masks (α-window ≈ 14°, θ_ht-window 25°-30°, thrust-window 22+).
D_ALPHA   = 2.0     # deg
D_THETAH  = 3.0     # deg
D_T_MULT  = 2.0     # multiplier units

# Load each phase's PhaseSpec from its existing plotter module so we
# inherit the masks / BO baselines without duplication.
def load_spec(phase: str) -> PhaseSpec:
    pp = REPO / "post" / "v2_gapped" / f"plot_{phase}_sensitivities.py"
    spec_name = f"v2_gapped_{phase}_spec"
    s = importlib.util.spec_from_file_location(spec_name, pp)
    mod = importlib.util.module_from_spec(s); s.loader.exec_module(mod)
    return mod.SPEC


def all_sweeps_done(project: fl.Project) -> bool:
    for cid in project.get_case_ids():
        c = fl.Case.from_cloud(case_id=cid)
        s = str(c.status).replace("Flow360Status.", "")
        # COMPLETED is the happy path; STOPPED forks are Flow360 dedups
        # (BO-point duplicate of parent) — also OK.  Anything else means
        # the sweep is still in flight.
        if s not in ("COMPLETED", "STOPPED"):
            return False
    return True


def refined_already(project: fl.Project, phase: str) -> bool:
    nominal_name = f"gap40_{phase}_refined_nominal"
    for cid in project.get_case_ids():
        c = fl.Case.from_cloud(case_id=cid)
        if c.name == nominal_name:
            return True
    return False


def find_parent(project: fl.Project) -> fl.Case:
    for cid in project.get_case_ids():
        c = fl.Case.from_cloud(case_id=cid)
        if "_parent" in c.name or "BO_estimate" in c.name:
            return c
    raise RuntimeError(f"no parent case in {project.id}")


def solve_trim(spec: PhaseSpec, rows: list[dict]) -> tuple[float, float, float, dict]:
    """Returns (α*, θ_ht*, T*) and the slope dict."""
    a, da = by_sweep(rows, "alpha",  spec.qS)
    h, dh = by_sweep(rows, "htail",  spec.qS)
    t, dt = by_sweep(rows, "thrust", spec.qS)
    Ma, Mh, Mt = spec.mask_alpha(a), spec.mask_htail(h), spec.mask_thrust(t)
    sens = {
        "dCL_da":  fit_linear(a, da["CL"],     Ma)[0],
        "dCFx_da": fit_linear(a, da["CFx"],    Ma)[0],
        "dCMy_da": fit_linear(a, da["CMy_CG"], Ma)[0],
        "dCL_dh":  fit_linear(h, dh["CL"],     Mh)[0],
        "dCFx_dh": fit_linear(h, dh["CFx"],    Mh)[0],
        "dCMy_dh": fit_linear(h, dh["CMy_CG"], Mh)[0],
        "dCL_dT":  fit_linear(t, dt["CL"],     Mt)[0],
        "dCFx_dT": fit_linear(t, dt["CFx"],    Mt)[0],
        "dCMy_dT": fit_linear(t, dt["CMy_CG"], Mt)[0],
        "dCT_dT":  fit_linear(t, dt["CT_delivered"], Mt)[0],
    }
    if not all(np.isfinite(v) for v in sens.values()):
        raise RuntimeError(f"non-finite slope(s) — sparse data: {sens}")

    CL_b  = baseline_of(da["CL"],     a, spec.alpha_b)
    CMy_b = baseline_of(da["CMy_CG"], a, spec.alpha_b)
    CFx_b = baseline_of(da["CFx"],    a, spec.alpha_b)
    CT_b  = baseline_of(dt["CT_delivered"], t, spec.T_b)
    from math import cos, sin
    ab = radians(spec.alpha_b)
    res = np.array([(spec.W_cos_g - CT_b * spec.qS * sin(ab)) / spec.qS - CL_b,
                    -CMy_b,
                    (spec.W_sin_g / spec.qS) - (CT_b * cos(ab) - CFx_b)])
    A = np.array([
        [sens["dCL_da"],  sens["dCL_dh"],  sens["dCL_dT"]],
        [sens["dCMy_da"], sens["dCMy_dh"], sens["dCMy_dT"]],
        [-CT_b * sin(ab) - sens["dCFx_da"], -sens["dCFx_dh"],
         sens["dCT_dT"] * cos(ab) - sens["dCFx_dT"]],
    ])
    dx = np.linalg.solve(A, res)
    return spec.alpha_b + dx[0], spec.theta_ht_b + dx[1], spec.T_b + dx[2], sens


def submit_seven(project: fl.Project, parent: fl.Case, spec: PhaseSpec,
                 alpha_star: float, theta_h_star: float, T_star: float):
    points = [
        ("nominal",  alpha_star,             theta_h_star,             T_star),
        ("a_minus",  alpha_star - D_ALPHA,   theta_h_star,             T_star),
        ("a_plus",   alpha_star + D_ALPHA,   theta_h_star,             T_star),
        ("h_minus",  alpha_star,             theta_h_star - D_THETAH,  T_star),
        ("h_plus",   alpha_star,             theta_h_star + D_THETAH,  T_star),
        ("t_minus",  alpha_star,             theta_h_star,             T_star - D_T_MULT),
        ("t_plus",   alpha_star,             theta_h_star,             T_star + D_T_MULT),
    ]
    surfaces = C.get_gapped_geometry_surfaces(project)
    submitted = []
    for tag, alpha, theta_h, T in points:
        name = f"gap40_{spec.phase_name}_refined_{tag}"
        case = project.run_case(
            params=C.build_params(
                surfaces,
                theta_ac_rad     = +radians(alpha),
                theta_ht_rad     = +radians(theta_h),
                thrust_mult      = T,
                velocity_m_s     = spec.velocity,
                altitude_m       = 0.0 if spec.phase_name != "cruise" else P.ALT_CRUISE_M,
                n_steps_total    = 26,
                max_pseudo_steps = 500,
                htail_z_m        = P.Z_TAIL_LOW_M,
            ),
            name=name, run_async=True, fork_from=parent,
            tags=["SI", "v2", "gapped40", "low_htail", "refined", spec.phase_name, tag],
            use_beta_mesher=True,
        )
        print(f"  {name:38s} α={alpha:+.2f}°  θ_ht={theta_h:+.2f}°  T={T:+.2f}  → {case.id}")
        submitted.append((tag, case.id))
    return submitted


def process_phase(phase: str):
    spec = load_spec(phase)
    project = fl.Project.from_cloud(project_id=spec.project_id)
    print(f"\n=== {phase} (project {project.id[:18]}) ===")
    if not all_sweeps_done(project):
        print("  sweeps not all done — skipping")
        return
    if refined_already(project, phase):
        print("  refined nominal already submitted — skipping")
        return
    rows = fetch_rows(
        discover_sweeps(spec.project_id, spec.alpha_b, spec.theta_ht_b, spec.T_b),
        spec.rho_a2_L2,
    )
    alpha_star, theta_h_star, T_star, sens = solve_trim(spec, rows)
    print(f"  Trim equilibrium:  α*={alpha_star:+.3f}°  θ_ht*={theta_h_star:+.3f}°  "
          f"T*={T_star:+.3f}")
    parent = find_parent(project)
    print(f"  Forking 7 refined cases from parent {parent.id[:18]} …")
    submit_seven(project, parent, spec, alpha_star, theta_h_star, T_star)


if __name__ == "__main__":
    phases = [a for a in sys.argv[1:] if a in ("cruise", "takeoff", "landing")]
    if not phases: phases = ["cruise", "takeoff", "landing"]
    for ph in phases:
        process_phase(ph)
