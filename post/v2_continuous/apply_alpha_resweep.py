"""Replace the α-sweep rows in the cached sweep CSVs for cont_low and
cont_high with the new α-resweep at higher θ_ht (where each case is
COMPLETED).  htail-sweep and thrust-sweep rows are preserved.

Rationale: the original α-sweeps pinned θ_ht at the BO point (-5° takeoff,
-6° landing), which put the htail deep into negative-α stall when the
wing-flap downwash piles on at high α.  The contaminated slope dCMy/dα
makes the static margin look much worse than it really is.  The resweep
pins θ_ht at +10° takeoff / +20° landing — htail in its linear regime
across the full α range.

For each (config, phase):
  - Filter the existing CSV to keep htail-sweep and thrust-sweep rows
  - Add the resweep α-rows (only completed cases)
  - Write the CSV back

Then plot_*_sensitivities.py can be re-run (without --refresh) to refit
slopes and regenerate plots.
"""
from __future__ import annotations
import csv, json, sys
from pathlib import Path
import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "post"))
import params as P
from _phase_plot import _case_altitude_m, _isa_rho_a
import flow360 as fl

# CSV target paths per (config, phase).
CSV_PATHS = {
    ("cont_low",  "takeoff"): REPO / "post/out/v2_continuous/takeoff_sweep_data.csv",
    ("cont_low",  "landing"): REPO / "post/out/v2_continuous/landing_sweep_data.csv",
    ("cont_high", "takeoff"): REPO / "post/out/v2_continuous_high/takeoff_sweep_data.csv",
    ("cont_high", "landing"): REPO / "post/out/v2_continuous_high/landing_sweep_data.csv",
}


def fetch_resweep_alpha_rows(alpha_ids: list) -> list[dict]:
    """For each (α, case_id) in alpha_ids, if the case is COMPLETED,
    return a CSV-row dict.  Uses per-case altitude for F_AD."""
    rows = []
    for a, cid in alpha_ids:
        c = fl.Case.from_cloud(case_id=cid)
        if "COMPLETED" not in str(c.status):
            print(f"  α={a:+5.1f}  {cid[:18]}  SKIP — {c.status}")
            continue
        tf = c.results.total_forces; tf.load_from_remote(); v = tf.values
        ps = np.array(v["physical_step"])
        last = int(np.where(ps == ps.max())[0][-1])
        ad = c.results.actuator_disks; ad.load_from_remote(); av = ad.values
        alt = _case_altitude_m(c)
        rho_c, a_c = _isa_rho_a(alt or 0.0)
        rho_a2 = rho_c * a_c * a_c
        F_AD = sum(np.array(av[f"Disk{i}_Force"])[-1] for i in range(10)) * rho_a2
        rows.append(dict(
            sweep="alpha", value=a, case_id=cid,
            CL=float(v["CL"][last]), CD=float(v["CD"][last]),
            CMy=float(v["CMy"][last]), CFx=float(v["CFx"][last]),
            F_AD_delivered_N=F_AD,
            physical_step=int(ps[last]),
            pseudo_step=int(v["pseudo_step"][last]),
        ))
        print(f"  α={a:+5.1f}  {cid[:18]}  alt={alt:5.0f}m  CL={rows[-1]['CL']:+.3f}  CMy={rows[-1]['CMy']:+.3f}")
    return rows


def main():
    data = json.loads((REPO / "flow360/alpha_resweep_high_htail.json").read_text())
    for r in data:
        cfg, phase = r["cfg"], r["phase"]
        if (cfg, phase) not in CSV_PATHS:
            continue
        csv_path = CSV_PATHS[(cfg, phase)]
        print(f"\n=== {cfg} {phase}  →  {csv_path.name} ===")

        with csv_path.open() as f:
            existing = list(csv.DictReader(f))
        non_alpha = [row for row in existing if row["sweep"] != "alpha"]

        new_alpha = fetch_resweep_alpha_rows(r["alpha"])
        if not new_alpha:
            print(f"  no completed resweep cases yet — leaving CSV unchanged")
            continue

        out_rows = new_alpha + non_alpha
        fieldnames = list(existing[0].keys())
        with csv_path.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=fieldnames); w.writeheader()
            for row in out_rows:
                w.writerow({k: row.get(k, "") for k in fieldnames})
        print(f"  wrote {len(new_alpha)} α-rows + {len(non_alpha)} non-α rows → {csv_path}")


if __name__ == "__main__":
    main()
