"""
Phase-plot library used by the six per-phase scripts in
`post/v2_continuous/` and `post/v2_gapped/`.

Each phase script supplies a `PhaseSpec` (project id, BO baseline,
flight constants, sweep masks, output paths, title) and calls
`run(spec)`.  Everything else — sweep discovery, force fetch, slope
fit, 3×3 trim solve, plot generation — lives here.
"""
from __future__ import annotations

import csv
import re
import sys
from dataclasses import dataclass
from math import cos, radians, sin
from pathlib import Path
from typing import Callable

import numpy as np
import matplotlib.pyplot as plt

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
import params as P

SWEEP_RE = re.compile(r"_(alpha|htail|thrust)_([pmx][\d.p]+)(?:deg)?(?:$|_)")
PARENT_SUBSTR_DEFAULT = "_parent"


# ---------------------------------------------------------------------------
# Sweep-case discovery from Flow360 project (by name pattern)
# ---------------------------------------------------------------------------

def _parse_tok(tok: str) -> float:
    sign = +1
    if tok.startswith("m"):
        sign = -1
        tok = tok[1:]
    elif tok[0] in "px":
        tok = tok[1:]
    return sign * float(tok.replace("p", "."))


def discover_sweeps(project_id: str, alpha_b: float, theta_ht_b: float, T_b: float,
                    parent_substr: str = PARENT_SUBSTR_DEFAULT) -> dict:
    """Walk the project's cases, parse sweep names, return dict of sweep
    lists + the parent case id.  The parent is also injected at the BO
    point of each sweep so the fitter sees a complete range."""
    from flow360 import Project, Case
    p = Project.from_cloud(project_id=project_id)
    out: dict = {"alpha": [], "htail": [], "thrust": [], "parent": None}
    for cid in p.get_case_ids():
        c = Case.from_cloud(case_id=cid)
        if parent_substr in c.name or "BO_estimate" in c.name:
            out["parent"] = cid
            continue
        m = SWEEP_RE.search(c.name)
        if not m:
            continue
        out[m.group(1)].append((_parse_tok(m.group(2)), cid))
    if out["parent"]:
        out["alpha"].append((alpha_b, out["parent"]))
        out["htail"].append((theta_ht_b, out["parent"]))
        out["thrust"].append((T_b, out["parent"]))
    for k in ("alpha", "htail", "thrust"):
        out[k] = sorted(out[k])
    return out


# ---------------------------------------------------------------------------
# Force fetch (skips cases with no result yet)
# ---------------------------------------------------------------------------

def fetch_rows(sweeps: dict, rho_a2_L2: float) -> list[dict]:
    """Pull final-step total_forces + actuator_disks for each case in the
    sweep dict.  Skips cases whose Flow360 status isn't COMPLETED (still
    running, stopped, errored).  Any unexpected download failure on a
    COMPLETED case propagates — that's a real bug worth surfacing."""
    import flow360 as fl
    rows: list[dict] = []
    for sweep in ("alpha", "htail", "thrust"):
        for val, cid in sweeps[sweep]:
            c = fl.Case.from_cloud(case_id=cid)
            if "COMPLETED" not in str(c.status):
                print(f"  {sweep:6s} {val:+6.2f}  {cid[:18]}  SKIP — "
                      f"{str(c.status).replace('Flow360Status.', '')}")
                continue
            tf = c.results.total_forces; tf.load_from_remote(); v = tf.values
            ps = np.array(v["physical_step"])
            last = int(np.where(ps == ps.max())[0][-1])
            ad = c.results.actuator_disks; ad.load_from_remote()
            av = ad.values
            F_AD = sum(np.array(av[f"Disk{i}_Force"])[-1] for i in range(10)) * rho_a2_L2
            rows.append(dict(
                sweep=sweep, value=val, case_id=cid,
                CL=float(v["CL"][last]),  CD=float(v["CD"][last]),
                CMy=float(v["CMy"][last]), CFx=float(v["CFx"][last]),
                F_AD_delivered_N=F_AD,
                physical_step=int(ps[last]),
                pseudo_step=int(v["pseudo_step"][last]),
            ))
            print(f"  {sweep:6s} {val:+6.2f}  {cid[:18]}  "
                  f"CL={rows[-1]['CL']:+.4f}  CMy={rows[-1]['CMy']:+.4f}  F_AD={F_AD:+.0f}")
    return rows


# ---------------------------------------------------------------------------
# CSV cache + post-processing
# ---------------------------------------------------------------------------

FLOAT_KEYS = {"value", "CL", "CD", "CMy", "CFx", "F_AD_delivered_N"}


def load_or_fetch(csv_path: Path, refresh: bool, sweeps_factory: Callable,
                  rho_a2_L2: float) -> list[dict]:
    if csv_path.exists() and not refresh:
        with csv_path.open() as f:
            return [{k: (float(v) if k in FLOAT_KEYS else v) for k, v in r.items()}
                    for r in csv.DictReader(f)]
    print("Fetching from Flow360 …")
    rows = fetch_rows(sweeps_factory(), rho_a2_L2)
    if rows:
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        with csv_path.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader()
            for r in rows: w.writerow(r)
        print(f"Wrote {csv_path}")
    return rows


def by_sweep(rows: list[dict], name: str, qS: float):
    sel = sorted([r for r in rows if r["sweep"] == name], key=lambda r: r["value"])
    x = np.array([r["value"] for r in sel])
    data = {k: np.array([r[k] for r in sel])
            for k in ("CL", "CD", "CMy", "CFx", "F_AD_delivered_N")}
    data["CT_delivered"] = data["F_AD_delivered_N"] / qS
    data["CMy_CG"]       = data["CMy"] - 0.1 * data["CT_delivered"]
    return x, data


def fit_linear(x, y, mask):
    slope, intercept = np.polyfit(x[mask], y[mask], 1)
    return float(slope), float(intercept)


def baseline_of(arr, x, x0):
    return float(arr[int(np.argmin(np.abs(x - x0)))])


# ---------------------------------------------------------------------------
# Phase config dataclass + driver
# ---------------------------------------------------------------------------

@dataclass
class PhaseSpec:
    label:       str            # "Cruise gap40" / "Takeoff v2 cont"
    project_id:  str
    out_dir:     Path
    phase_name:  str            # "cruise" | "takeoff" | "landing"
    alpha_b:     float
    theta_ht_b:  float
    T_b:         float
    velocity:    float          # m/s
    rho:         float          # kg/m^3
    gamma_deg:   float          # 0 for cruise, +30 climb, -30 descent
    mask_alpha:  Callable[[np.ndarray], np.ndarray]
    mask_htail:  Callable[[np.ndarray], np.ndarray]
    mask_thrust: Callable[[np.ndarray], np.ndarray]
    title:       str            # plot suptitle

    @property
    def qS(self):        return 0.5 * self.rho * self.velocity ** 2 * P.WING_AREA_M2
    @property
    def rho_a2_L2(self): return self.rho * P.A_SOUND_CRUISE_M_S ** 2
    @property
    def W_cos_g(self):   return P.W_GROSS_N * cos(radians(self.gamma_deg))
    @property
    def W_sin_g(self):   return P.W_GROSS_N * sin(radians(self.gamma_deg))


# ---------------------------------------------------------------------------
# Plotting + trim
# ---------------------------------------------------------------------------

def _plot_sensitivities(spec: PhaseSpec, a, da, h, dh, t, dt,
                        mask_a, mask_h, mask_t, out_png: Path):
    fig, axes = plt.subplots(3, 3, figsize=(13, 11))
    plt.subplots_adjust(left=0.08, right=0.97, top=0.93, bottom=0.07,
                        hspace=0.34, wspace=0.30)
    sweeps_grid = [
        ("alpha",  a, da, r"$\alpha$ [deg]",
         f"α sweep (θ_htail={spec.theta_ht_b:+.0f}°, T_mult={spec.T_b:.1f})"),
        ("htail",  h, dh, r"$\theta_{\rm htail}$ [deg]",
         f"H-tail sweep (α={spec.alpha_b:+.0f}°, T_mult={spec.T_b:.1f})"),
        ("thrust", t, dt, "T_mult",
         f"Thrust sweep (α={spec.alpha_b:+.0f}°, θ_htail={spec.theta_ht_b:+.0f}°)"),
    ]
    cols = [("CL", r"$C_L$"), ("CD", r"$C_D$"),
            ("CMy_CG", r"$C_{m,y}$  (CG, incl. thrust)")]
    masks = {"alpha": mask_a, "htail": mask_h, "thrust": mask_t}
    for row, (name, x, data, xlabel, title) in enumerate(sweeps_grid):
        for col, (key, ylabel) in enumerate(cols):
            ax = axes[row, col]; y = data[key]; mask = masks[name]
            ax.plot(x[mask], y[mask], "o-", color="C0", lw=1.4, mfc="white",
                    ms=6, label="fit pts")
            if (~mask).any():
                ax.plot(x[~mask], y[~mask], "x", color="C7", ms=7,
                        label="stalled/saturated")
            ax.set(xlabel=xlabel, ylabel=ylabel); ax.grid(True, alpha=0.3)
            if mask.sum() >= 2:
                slope, icpt = fit_linear(x, y, mask)
                xx = np.linspace(x[mask].min(), x[mask].max(), 50)
                ax.plot(xx, icpt + slope * xx, "--", color="C3", lw=0.9,
                        alpha=0.7, label=f"slope = {slope:+.4f}")
                ax.legend(fontsize=7, loc="best", framealpha=0.85)
            if col == 0:
                ax.set_title(title, loc="left", fontsize=9, pad=8)
    fig.suptitle(spec.title, fontsize=11, y=0.995)
    fig.savefig(out_png, dpi=160); plt.close(fig)
    print(f"Wrote {out_png}")


def _plot_thrust_balance(spec: PhaseSpec, t, dt, out_png: Path):
    fig, ax = plt.subplots(1, 1, figsize=(7, 5))
    drag_N        = dt["CFx"] * spec.qS
    cmd_thrust_N  = t * 10 * P.T_CRUISE_PER_PROP_N
    delivered_N   = dt["F_AD_delivered_N"]
    if abs(spec.gamma_deg) > 1e-3:
        drag_plus_g_N = drag_N + spec.W_sin_g
        ax.plot(t, drag_plus_g_N, "x-", lw=1.0, alpha=0.7,
                label="drag + W·sin(γ)")
    ax.plot(t, drag_N,       "o-", lw=1.5, mfc="white", label="aircraft drag")
    ax.plot(t, cmd_thrust_N, "s--", lw=1.0, alpha=0.7,    label="commanded AD")
    if np.isfinite(delivered_N).all():
        ax.plot(t, delivered_N, "^-", lw=1.5, mfc="white", label="AD delivered")
    ax.set(xlabel="T_mult", ylabel="force [N]")
    ax.set_title(f"{spec.label} thrust vs drag — γ={spec.gamma_deg:+.0f}°, "
                 f"V={spec.velocity:.1f} m/s", fontsize=11)
    ax.grid(True, alpha=0.3); ax.legend()
    fig.tight_layout(); fig.savefig(out_png, dpi=160); plt.close(fig)
    print(f"Wrote {out_png}")


def _solve_trim(spec: PhaseSpec, a, da, t, dt, sens: dict):
    CL_b  = baseline_of(da["CL"],     a, spec.alpha_b)
    CMy_b = baseline_of(da["CMy_CG"], a, spec.alpha_b)
    CFx_b = baseline_of(da["CFx"],    a, spec.alpha_b)
    CT_b  = baseline_of(dt["CT_delivered"], t, spec.T_b)
    a_b   = radians(spec.alpha_b)
    print(f"\nBaseline: CL={CL_b:+.4f} CMy_CG={CMy_b:+.4f} "
          f"CFx={CFx_b:+.4f} CT_del={CT_b:+.4f}")
    res = np.array([(spec.W_cos_g - CT_b * spec.qS * sin(a_b)) / spec.qS - CL_b,
                    -CMy_b,
                    (spec.W_sin_g / spec.qS) - (CT_b * cos(a_b) - CFx_b)])
    dCL  = [sens["dCL/dα"],  sens["dCL/dθh"],  sens["dCL/dT"]]
    dCMy = [sens["dCMy/dα"], sens["dCMy/dθh"], sens["dCMy/dT"]]
    dCFx = [sens["dCD/dα"],  sens["dCD/dθh"],  sens["dCD/dT"]]
    dCT_T = sens["dCT_del/dT"]
    dAx  = [-CT_b * sin(a_b) - dCFx[0], -dCFx[1], dCT_T * cos(a_b) - dCFx[2]]
    A = np.array([dCL, dCMy, dAx])
    dx = np.linalg.solve(A, res)
    sm = -dCMy[0] / dCL[0]
    print(f"\nRefined trim: α={spec.alpha_b + dx[0]:+.2f}°, "
          f"θ_ht={spec.theta_ht_b + dx[1]:+.2f}°, T_mult={spec.T_b + dx[2]:+.2f}")
    print(f"static margin = {sm:+.4f}  ({sm * 100:+.1f}% MAC)")
    return sm, dx


def run(spec: PhaseSpec, refresh: bool = False):
    spec.out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = spec.out_dir / f"{spec.phase_name}_sweep_data.csv"
    rows = load_or_fetch(
        csv_path, refresh,
        sweeps_factory=lambda: discover_sweeps(
            spec.project_id, spec.alpha_b, spec.theta_ht_b, spec.T_b),
        rho_a2_L2=spec.rho_a2_L2,
    )
    if not rows:
        print("No data — sweep cases not yet complete.  Re-run with --refresh later.")
        return

    a, da = by_sweep(rows, "alpha",  spec.qS)
    h, dh = by_sweep(rows, "htail",  spec.qS)
    t, dt = by_sweep(rows, "thrust", spec.qS)
    Ma, Mh, Mt = spec.mask_alpha(a), spec.mask_htail(h), spec.mask_thrust(t)
    for label, mask in (("alpha", Ma), ("htail", Mh), ("thrust", Mt)):
        if mask.sum() < 2:
            print(f"Insufficient {label} data in fit range "
                  f"({mask.sum()} point{'s' if mask.sum()!=1 else ''} after mask). "
                  f"Re-run with --refresh once more sweep cases complete.")
            return

    sens = {
        "dCL/dα":     fit_linear(a, da["CL"],     Ma)[0],
        "dCD/dα":     fit_linear(a, da["CD"],     Ma)[0],
        "dCMy/dα":    fit_linear(a, da["CMy_CG"], Ma)[0],
        "dCL/dθh":    fit_linear(h, dh["CL"],     Mh)[0],
        "dCD/dθh":    fit_linear(h, dh["CD"],     Mh)[0],
        "dCMy/dθh":   fit_linear(h, dh["CMy_CG"], Mh)[0],
        "dCL/dT":     fit_linear(t, dt["CL"],     Mt)[0],
        "dCD/dT":     fit_linear(t, dt["CD"],     Mt)[0],
        "dCMy/dT":    fit_linear(t, dt["CMy_CG"], Mt)[0],
        "dCT_del/dT": fit_linear(t, dt["CT_delivered"], Mt)[0],
    }
    print(f"\n=== {spec.label} sensitivities "
          f"(about α={spec.alpha_b:+.0f}°, θ_ht={spec.theta_ht_b:+.0f}°, "
          f"T_mult={spec.T_b:+.1f}) ===")
    for k, v in sens.items():
        print(f"  {k:12s} = {v:+.5f}")

    _plot_sensitivities(spec, a, da, h, dh, t, dt, Ma, Mh, Mt,
                        spec.out_dir / f"{spec.phase_name}_sensitivities.png")
    _plot_thrust_balance(spec, t, dt,
                         spec.out_dir / f"{spec.phase_name}_thrust_balance.png")
    _solve_trim(spec, a, da, t, dt, sens)
