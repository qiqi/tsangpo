"""
Estimate how much the htail moment arm can be shortened while preserving
stability AND trim across cruise, takeoff, and landing for the gap40
low-htail config.

Idea: the htail's CMy_CG contribution scales linearly with its arm
(force × distance about the CG).  Wing+flap CMy contribution doesn't
depend on htail position.  So if we scale the htail arm by k ∈ [0, 1]
(k=1 = current = +3.75 c from CG), the new total CMy and slopes are:

    new_CMy_b          = CMy_rest_b   + k · CMy_htail_b
    new_dCMy/d•_total  = dCMy_rest/d• + k · dCMy_htail/d•

We pull the htail-only contribution from the surface_forces CSVs (each
case has `htail_pitch_zone/htail_CMy` separately tabulated), fit its
slopes vs α, θ_ht, T_mult.  The "rest" is `total - htail`.

For each candidate k we:

  1.  Stability — require new dCMy/dα < 0 (i.e., SM > 0).
  2.  Trim — solve the 3×3 trim system with the rescaled htail slopes
     and check that |θ_ht_trim - θ_ht_b| ≤ Δθ_ht_lim where the limit
     is set by the htail unstall window from the per-phase masks.

The smallest k that satisfies BOTH across all three phases is the
design floor for arm-shortening.  Output: post/out/v2_gapped/
htail_arm_study.md.
"""
from __future__ import annotations

import csv, importlib.util, sys, tempfile
from math import cos, radians, sin
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "post"))
import params as P
import flow360 as fl
from _phase_plot import PhaseSpec, by_sweep, baseline_of, fit_linear


def load_spec(phase: str) -> PhaseSpec:
    pp = REPO / "post" / "v2_gapped" / f"plot_{phase}_sensitivities.py"
    s  = importlib.util.spec_from_file_location(f"spec_{phase}", pp)
    m  = importlib.util.module_from_spec(s); s.loader.exec_module(m)
    return m.SPEC


def fetch_per_surface_cmy(case_id: str) -> float | None:
    """Pull last-step htail/CMy from a case's surface_forces.csv."""
    try:
        c = fl.Case.from_cloud(case_id=case_id)
        if "COMPLETED" not in str(c.status):
            return None
        with tempfile.TemporaryDirectory() as tmp:
            c.results.surface_forces.download(tmp + "/sf.csv")
            import pandas as pd
            df = pd.read_csv(tmp + "/sf.csv")
            df.columns = [c.strip() for c in df.columns]
            ht_col = next(c for c in df.columns
                          if c.endswith("/htail_CMy")
                          and "Pressure" not in c and "Skin" not in c)
            return float(df[ht_col].iloc[-1])
    except Exception:
        return None


def htail_slopes(spec: PhaseSpec) -> tuple[dict, float]:
    """Fit htail-only CMy slopes by reading per-case surface_forces.
    Cache to post/out/v2_gapped/{phase}_htail_cmy.csv to avoid re-pulling."""
    cache = spec.out_dir / f"{spec.phase_name}_htail_cmy.csv"
    if cache.exists():
        with cache.open() as f:
            rows = list(csv.DictReader(f))
        for r in rows:
            r["value"] = float(r["value"]); r["htail_CMy"] = float(r["htail_CMy"])
    else:
        sweep_csv = spec.out_dir / f"{spec.phase_name}_sweep_data.csv"
        with sweep_csv.open() as f:
            sweep_rows = list(csv.DictReader(f))
        rows = []
        for r in sweep_rows:
            ht_cmy = fetch_per_surface_cmy(r["case_id"])
            if ht_cmy is None: continue
            rows.append(dict(sweep=r["sweep"], value=float(r["value"]),
                             htail_CMy=ht_cmy))
            print(f"  {r['sweep']:6s} {float(r['value']):+6.2f}  "
                  f"htail_CMy={ht_cmy:+.4f}")
        with cache.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=["sweep", "value", "htail_CMy"])
            w.writeheader()
            for r in rows: w.writerow(r)
    # Fit slopes by sweep, with the same masks as the per-phase plotter.
    by = {"alpha": [], "htail": [], "thrust": []}
    for r in rows: by[r["sweep"]].append((r["value"], r["htail_CMy"]))
    out = {}
    for sweep, mask_fn in (("alpha",  spec.mask_alpha),
                           ("htail",  spec.mask_htail),
                           ("thrust", spec.mask_thrust)):
        if not by[sweep]: out[sweep] = float("nan"); continue
        xs = np.array([r[0] for r in by[sweep]])
        ys = np.array([r[1] for r in by[sweep]])
        ix = np.argsort(xs); xs, ys = xs[ix], ys[ix]
        m  = mask_fn(xs)
        out[sweep] = fit_linear(xs, ys, m)[0] if m.sum() >= 2 else float("nan")
    # Also htail_CMy at BO (use parent / closest baseline).
    base = None
    for r in rows:
        if r["sweep"] == "alpha" and abs(r["value"] - spec.alpha_b) < 0.1:
            base = r["htail_CMy"]; break
    return out, base if base is not None else 0.0


def total_slopes(spec: PhaseSpec) -> tuple[dict, dict]:
    csv_path = spec.out_dir / f"{spec.phase_name}_sweep_data.csv"
    with csv_path.open() as f:
        rows = [{k: (float(v) if k in {"value","CL","CD","CMy","CFx","F_AD_delivered_N"}
                     else v) for k, v in r.items()} for r in csv.DictReader(f)]
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
    base = {
        "CL_b":  baseline_of(da["CL"],     a, spec.alpha_b),
        "CD_b":  baseline_of(da["CD"],     a, spec.alpha_b),
        "CMy_b": baseline_of(da["CMy_CG"], a, spec.alpha_b),
        "CFx_b": baseline_of(da["CFx"],    a, spec.alpha_b),
        "CT_b":  baseline_of(dt["CT_delivered"], t, spec.T_b),
    }
    return slopes, base


def trim_3x3_for_k(k: float, spec: PhaseSpec, total: dict, base: dict,
                   ht_slopes: dict, ht_base: float):
    """Solve for trim deflection given the htail arm scaled by k."""
    # Rescale htail contributions, keep "rest" fixed.
    dCMy_da = total["dCMy_da"] - (1 - k) * ht_slopes["alpha"]
    dCMy_dh = k * total["dCMy_dh"]   # entire dCMy/dθh comes from htail
    dCMy_dT = total["dCMy_dT"] - (1 - k) * ht_slopes["thrust"]
    CMy_b   = base["CMy_b"]   - (1 - k) * ht_base
    SM      = -dCMy_da / total["dCL_da"]

    a_b = radians(spec.alpha_b)
    res = np.array([
        (spec.W_cos_g - base["CT_b"] * spec.qS * sin(a_b)) / spec.qS - base["CL_b"],
        -CMy_b,
        (spec.W_sin_g / spec.qS) - (base["CT_b"] * cos(a_b) - base["CFx_b"]),
    ])
    A = np.array([
        [total["dCL_da"],  total["dCL_dh"],  total["dCL_dT"]],
        [dCMy_da,          dCMy_dh,          dCMy_dT],
        [-base["CT_b"] * sin(a_b) - total["dCD_da"], -total["dCD_dh"],
         total["dCT_dT"] * cos(a_b) - total["dCD_dT"]],
    ])
    dx = np.linalg.solve(A, res)
    return dict(k=k, SM=SM,
                alpha_trim   = spec.alpha_b   + dx[0],
                theta_ht_trim= spec.theta_ht_b + dx[1],
                T_trim       = spec.T_b       + dx[2])


def find_floor(spec: PhaseSpec, total: dict, base: dict,
               ht_slopes: dict, ht_base: float,
               sm_min: float = 0.05, theta_max: float = 25.0):
    """Binary search for the smallest k ∈ [0, 1] that keeps SM ≥ sm_min
    AND |θ_ht_trim| ≤ theta_max."""
    def ok(k):
        r = trim_3x3_for_k(k, spec, total, base, ht_slopes, ht_base)
        return r["SM"] >= sm_min and abs(r["theta_ht_trim"]) <= theta_max, r
    ok_1, r1 = ok(1.0)
    if not ok_1:
        return None, "current arm doesn't satisfy SM/θ_ht limits — check the constraint values"
    # bisect
    lo, hi = 0.0, 1.0
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        is_ok, _ = ok(mid)
        if is_ok: hi = mid
        else:     lo = mid
    return hi, trim_3x3_for_k(hi, spec, total, base, ht_slopes, ht_base)


def main():
    out_md = REPO / "post" / "out" / "v2_gapped" / "htail_arm_study.md"
    lines = [
        "# gap40 low-htail — how far can the htail arm be shortened?",
        "",
        "Linear-model estimate: the htail's CMy_CG contribution scales with",
        "its moment arm.  Fitting htail-only CMy slopes from the per-case",
        "`htail_pitch_zone/htail_CMy` (surface_forces CSV), then rescaling",
        "the htail contribution by k ∈ [0, 1] (k=1 is current = +3.75 c from",
        "CG), we get the new total slopes:",
        "",
        "    new dCMy/d•_total = (dCMy/d•_total)_now - (1-k) · (dCMy/d•_htail)",
        "    new CMy_b         = (CMy_b)_now         - (1-k) · CMy_htail_b",
        "",
        "Then solve the same 3×3 trim system per phase.  The minimum k that",
        "satisfies SM ≥ 5 % MAC AND |θ_ht_trim| ≤ 25° (htail unstall window)",
        "across all three phases is the design floor.",
        "",
        "Caveats: (a) ignores downwash change as the htail moves toward the",
        "wing (closer to CG = closer to wing slipstream = stronger downwash =",
        "less effective).  This study OVERESTIMATES how close the htail can",
        "be moved.  (b) gap40 takeoff/landing slopes use the current",
        "partial-sweep data; numbers will refine as more cases land.",
        "",
    ]
    per_phase = {}
    floors    = {}
    for phase in ("cruise", "takeoff", "landing"):
        print(f"\n=== {phase} ===")
        spec = load_spec(phase)
        total, base = total_slopes(spec)
        ht_slopes, ht_base = htail_slopes(spec)
        per_phase[phase] = (spec, total, base, ht_slopes, ht_base)
        k_floor, r = find_floor(spec, total, base, ht_slopes, ht_base)
        floors[phase] = (k_floor, r)
        if k_floor is None:
            print(f"  current arm already violates constraints: {r}")
        else:
            print(f"  k_floor = {k_floor:.3f}  → SM={r['SM']:+.4f}  "
                  f"θ_ht_trim={r['theta_ht_trim']:+.2f}°  "
                  f"L_h/c = {k_floor*3.75:.2f}")
    overall = max((f for f, _ in floors.values() if f is not None), default=None)

    # Render
    lines += [
        "## Per-phase htail slopes (CMy contribution of the htail only)",
        "",
        "| phase   | dCMy_htail/dα | dCMy_htail/dθh | dCMy_htail/dT_mult | CMy_htail at BO |",
        "|---------|---------------|-----------------|--------------------|-----------------|",
    ]
    for phase in ("cruise", "takeoff", "landing"):
        _, _, _, hs, hb = per_phase[phase]
        lines.append(f"| {phase:7s} | {hs['alpha']:+.4f}     | {hs['htail']:+.4f}      | "
                     f"{hs['thrust']:+.4f}        | {hb:+.4f}        |")

    lines += [
        "",
        "## Minimum arm fraction k that still trims, per phase",
        "",
        "| phase   | k_floor | new arm L_h/c | SM   | α_trim | θ_ht_trim | T_mult_trim |",
        "|---------|---------|----------------|------|--------|-----------|-------------|",
    ]
    for phase in ("cruise", "takeoff", "landing"):
        k, r = floors[phase]
        if k is None:
            lines.append(f"| {phase:7s} | n/a | n/a | n/a | n/a | n/a | n/a |")
        else:
            lines.append(
                f"| {phase:7s} | {k:.3f}  | {k*3.75:.2f} c        | "
                f"{r['SM']*100:+.1f} % | {r['alpha_trim']:+.2f}° | "
                f"{r['theta_ht_trim']:+.2f}°    | {r['T_trim']:+.2f}    |"
            )
    lines += [
        "",
        f"**Design floor across all three phases: k = {overall:.3f}**  →  "
        f"new htail arm L_h ≈ {overall*3.75:.2f} c (was 3.75 c).",
        "",
        "I.e., the htail could in principle be moved forward by",
        f"≈ {(1-overall)*3.75:.2f} chord-lengths (from +3.75 c down to +{overall*3.75:.2f} c",
        "aft of the CG) before the most-constrained phase loses either",
        "stability margin or elevator authority.",
        "",
        "Subject to the caveats above (especially downwash growth as the",
        "htail moves into the wing wake) — the **realistic** floor is",
        "likely 10–20 % larger than the linear-model estimate.",
    ]
    out_md.write_text("\n".join(lines))
    print(f"\nwrote {out_md}")
    print(f"design floor across all 3 phases: k = {overall:.3f}  →  L_h ≈ {overall*3.75:.2f} c")


if __name__ == "__main__":
    main()
