"""
Pull CL/Cm from each completed Flow360 run, compute the pitching-moment
slope C_m_alpha, write a CSV, and print a console summary that calls out
the Case 2 (Downwash Failure, positive C_m_alpha) vs Case 4 (Proposed
Synthesis, negative C_m_alpha) contrast - the headline of the SciTech
non-linear coupling story.

    python post/extract_stability.py            # use forces.csv if present, else synth
    python post/extract_stability.py --demo     # force synthesis

Per case it expects flow360/results/<case>/forces.csv with columns
alpha, CL, CD, Cm (one row per alpha in a small micro-sweep around 10 deg).
"""

from __future__ import annotations

import argparse, csv, math, sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
import params as P

RESULTS = REPO / "flow360" / "results"
OUT     = REPO / "post" / "out"


# Synthesised C_L_a, C_m_a targets at alpha_0 = 10 deg for each case. These
# encode the physical story: at the low H-tail station (Cases 2 & 4), the
# tail response is governed by what hits it - continuous-flap downwash
# (Case 2: destabilising) or gap-fed upwash (Case 4: amplified stability).
# The high-tail cases (1 & 3) sit in clean flow and are nearly identical.
TARGET_AT_ALPHA0 = {
    "industry_baseline":   {"CL_a": 5.70, "Cm_a": -1.00, "CL0": 1.40},
    "downwash_failure":    {"CL_a": 5.70, "Cm_a":  0.30, "CL0": 1.40},
    "bad_tradeoff":        {"CL_a": 5.40, "Cm_a": -1.00, "CL0": 1.26},
    "proposed_synthesis":  {"CL_a": 5.40, "Cm_a": -1.80, "CL0": 1.26},
}


def _synth_rows(case_name: str) -> list[dict]:
    t = TARGET_AT_ALPHA0[case_name]
    a0_rad = math.radians(P.ALPHA_DESIGN_DEG)
    Cm0 = -t["Cm_a"] * a0_rad + 0.02   # so that Cm crosses near zero around alpha=10
    rows = []
    for a in (P.ALPHA_DESIGN_DEG - 1.0, P.ALPHA_DESIGN_DEG, P.ALPHA_DESIGN_DEG + 1.0):
        a_rad = math.radians(a)
        rows.append({
            "alpha": a,
            "CL":    t["CL0"] - t["CL_a"] * (a0_rad - a_rad),
            "CD":    0.06 + 0.05 * a_rad ** 2,
            "Cm":    Cm0 + t["Cm_a"] * a_rad,
        })
    return rows


def _read_rows(case_name: str, demo: bool) -> tuple[list[dict], str]:
    csv_path = RESULTS / case_name / "forces.csv"
    if demo or not csv_path.exists():
        return _synth_rows(case_name), "synth"
    with csv_path.open() as f:
        return [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)], "flow360"


def _slope(xs: list[float], ys: list[float]) -> float:
    return (ys[-1] - ys[0]) / (math.radians(xs[-1]) - math.radians(xs[0]))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--demo", action="store_true", help="Force synthesis.")
    args = ap.parse_args()

    cases = list(P.study1_matrix())
    table = []
    sources = set()
    for idx, c in enumerate(cases, start=1):
        rows, src = _read_rows(c.name, args.demo)
        sources.add(src)
        a    = [r["alpha"] for r in rows]
        CL_a = _slope(a, [r["CL"] for r in rows])
        Cm_a = _slope(a, [r["Cm"] for r in rows])
        mid  = rows[len(rows) // 2]
        table.append({
            "case_id":      f"C{idx}",
            "case":         c.name,
            "label":        c.label,
            "gap_fraction": c.gap_fraction,
            "z_tail_c":     c.z_tail_chords,
            "alpha_deg":    mid["alpha"],
            "CL":           mid["CL"],
            "CD":           mid["CD"],
            "Cm":           mid["Cm"],
            "CL_alpha":     CL_a,
            "Cm_alpha":     Cm_a,
            "xNP_over_MAC": 0.25 - Cm_a / CL_a,
            "stable":       Cm_a < 0,
        })

    OUT.mkdir(parents=True, exist_ok=True)
    csv_path = OUT / "stability_table.csv"
    with csv_path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(table[0].keys()))
        w.writeheader()
        w.writerows(table)
    print(f"wrote {csv_path.relative_to(REPO)}  ({'synth' if 'synth' in sources else 'flow360'} data)\n")

    print(f"{'ID':3s} {'case':22s} {'gap':>5s} {'z_t':>5s} {'CL':>6s} "
          f"{'Cm':>7s} {'CL_a':>6s} {'Cm_a':>7s} {'xNP/MAC':>9s}  status")
    for r in table:
        flag = "stable" if r["stable"] else ">>UNSTABLE<<"
        print(f"{r['case_id']:3s} {r['case']:22s} {r['gap_fraction']:5.2f} "
              f"{r['z_tail_c']:+5.2f} {r['CL']:6.3f} {r['Cm']:+7.3f} "
              f"{r['CL_alpha']:6.2f} {r['Cm_alpha']:+7.2f} "
              f"{r['xNP_over_MAC']:9.3f}  {flag}")

    by_id = {r["case_id"]: r for r in table}
    c1, c2, c3, c4 = by_id["C1"], by_id["C2"], by_id["C3"], by_id["C4"]

    print("\n--- Headline contrast ---------------------------------------------------")
    print(f"  C2 Downwash Failure :   Cm_alpha = {c2['Cm_alpha']:+.3f}  (positive: UNSTABLE)")
    print(f"  C4 Proposed Synthesis:  Cm_alpha = {c4['Cm_alpha']:+.3f}  (negative: STABLE)")
    print(f"  Delta Cm_alpha (C2 - C4) = {c2['Cm_alpha'] - c4['Cm_alpha']:+.3f}  "
          "(single H-tail station flip)")

    print("\n--- Non-linear coupling -------------------------------------------------")
    eff_z   = c2["Cm_alpha"] - c1["Cm_alpha"]   # Z_tail alone, at gap=0
    eff_gap = c3["Cm_alpha"] - c1["Cm_alpha"]   # gap alone, at high tail
    linear  = c1["Cm_alpha"] + eff_z + eff_gap
    nonlin  = c4["Cm_alpha"] - linear
    print(f"  effect of Z_tail alone   (C1 -> C2):  Delta Cm_a = {eff_z:+.3f}")
    print(f"  effect of gap alone      (C1 -> C3):  Delta Cm_a = {eff_gap:+.3f}")
    print(f"  linear superposition     -> C4 pred:  Cm_a       = {linear:+.3f}")
    print(f"  actual C4                            Cm_a       = {c4['Cm_alpha']:+.3f}")
    print(f"  non-linear coupling                  Delta       = {nonlin:+.3f}   "
          f"({'synergy' if nonlin < 0 else 'anti-synergy'})")

    print("\n--- Lift penalty --------------------------------------------------------")
    dCL = (c4["CL"] - c1["CL"]) / c1["CL"] * 100
    print(f"  CL loss (C1 -> C4): {dCL:+.1f} %    (the 'price' of the gap)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
