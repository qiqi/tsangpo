"""α-sweep comparison: original (θ_ht at BO, possibly htail-stalled in
landing) vs new resweep at higher θ_ht (htail unstalled).

For each (config, phase) in [cont_low, cont_high] × [takeoff, landing]:
  - Fetch all-α CFD from the original sweep CSV (post/out/...)
  - Fetch the resweep cases (case IDs from flow360/alpha_resweep_high_htail.json)
  - Overlay CL/CD/CMy vs α
  - Annotate dCMy/dα slope for each curve

Output: post/out/v2_continuous/alpha_resweep_vs_orig.png  (one 2x3 figure
per config; two configs side-by-side rows for cont_low and cont_high gives
4x3 total).
"""
from __future__ import annotations
import csv, json, math, sys
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import flow360 as fl

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "post"))
import params as P
from _phase_plot import _case_altitude_m, _isa_rho_a

ALPHA_THETA_PIN = {"takeoff": +10.0, "landing": +20.0}
ORIG_THETA = {"takeoff": -5.0, "landing": -6.0}

ROWS = [
    ("cont_low",  "takeoff", "v2_continuous",       "takeoff_sweep_data.csv"),
    ("cont_low",  "landing", "v2_continuous",       "landing_sweep_data.csv"),
    ("cont_high", "takeoff", "v2_continuous_high",  "takeoff_sweep_data.csv"),
    ("cont_high", "landing", "v2_continuous_high",  "landing_sweep_data.csv"),
]


def load_orig(fam, fname):
    """α-sweep rows from the cached CSV."""
    p = REPO / "post" / "out" / fam / fname
    out = []
    for r in csv.DictReader(p.open()):
        if r["sweep"] != "alpha":
            continue
        out.append((float(r["value"]),
                     float(r["CL"]), float(r["CD"]), float(r["CMy"])))
    out.sort()
    a = np.array([r[0] for r in out])
    CL = np.array([r[1] for r in out])
    CD = np.array([r[2] for r in out])
    CMy = np.array([r[3] for r in out])
    # Subtract moment-center shift to get CMy_CG (matches _phase_plot.by_sweep
    # which computes CMy_CG = CMy - 0.1 * CT_delivered for v2 geometry).
    # For overlay purposes we just plot raw CMy vs α — the SLOPE is what we
    # care about, and the constant offset cancels.
    return a, CL, CD, CMy


def load_resweep(cfg, phase, ids):
    rows = []
    for a, cid in ids:
        c = fl.Case.from_cloud(case_id=cid)
        if "COMPLETED" not in str(c.status):
            continue
        tf = c.results.total_forces; tf.load_from_remote(); v = tf.values
        ps = np.array(v["physical_step"]); last = int(np.where(ps == ps.max())[0][-1])
        rows.append((a, float(v["CL"][last]), float(v["CD"][last]), float(v["CMy"][last])))
    rows.sort()
    if not rows:
        return None
    a = np.array([r[0] for r in rows])
    CL = np.array([r[1] for r in rows])
    CD = np.array([r[2] for r in rows])
    CMy = np.array([r[3] for r in rows])
    return a, CL, CD, CMy


def linfit(a, y, mask):
    if mask.sum() < 2: return None, None
    p = np.polyfit(a[mask], y[mask], 1)
    return p[0], p[1]


def main():
    ids_by_key = {}
    data = json.loads((REPO / "flow360/alpha_resweep_high_htail.json").read_text())
    for r in data:
        ids_by_key[(r["cfg"], r["phase"])] = r["alpha"]

    fig, axes = plt.subplots(4, 3, figsize=(13, 13),
                              constrained_layout=True, sharex=True)
    for row_i, (cfg, phase, fam, fname) in enumerate(ROWS):
        a_o, CL_o, CD_o, CMy_o = load_orig(fam, fname)
        res = load_resweep(cfg, phase, ids_by_key.get((cfg, phase), []))
        if res is None:
            a_n = CL_n = CD_n = CMy_n = None
        else:
            a_n, CL_n, CD_n, CMy_n = res

        # Pre-stall mask for slope fit (matches the spec mask:
        # takeoff α ≤ +11°, landing α ≤ +17°)
        mask_cut_o = (a_o <= (11.0 if phase == "takeoff" else 17.0))
        if a_n is not None:
            mask_cut_n = (a_n <= (11.0 if phase == "takeoff" else 17.0))

        col_data = [("CL", CL_o, CL_n), ("CD", CD_o, CD_n), ("CMy", CMy_o, CMy_n)]
        for col, (name, y_o, y_n) in enumerate(col_data):
            ax = axes[row_i, col]
            # original
            ax.plot(a_o, y_o, "o-", color="#1f77b4", ms=5, lw=1.2,
                     label=f"orig θ_ht={ORIG_THETA[phase]:+.0f}°")
            slope_o, _ = linfit(a_o, y_o, mask_cut_o)
            if slope_o is not None and name == "CMy":
                ax.text(0.02, 0.93,
                         f"d{name}/dα = {slope_o:+.4f}",
                         transform=ax.transAxes, fontsize=9, color="#1f77b4")
            # new resweep
            if a_n is not None:
                ax.plot(a_n, y_n, "s-", color="#d62728", ms=6, lw=1.4,
                         label=f"resweep θ_ht={ALPHA_THETA_PIN[phase]:+.0f}°")
                slope_n, _ = linfit(a_n, y_n, mask_cut_n)
                if slope_n is not None and name == "CMy":
                    ax.text(0.02, 0.85,
                             f"d{name}/dα = {slope_n:+.4f}",
                             transform=ax.transAxes, fontsize=9, color="#d62728")
            ax.set_title(f"{cfg} {phase}: {name}(α)", fontsize=10)
            ax.set_xlabel("α [deg]"); ax.set_ylabel(name)
            ax.grid(alpha=0.3)
            ax.axvline(11 if phase == "takeoff" else 17, color="gray",
                       ls=":", lw=0.7)
            if row_i == 0 and col == 2:
                ax.legend(fontsize=8)

    fig.suptitle("α-sweep comparison: original BO-θ_ht (possibly htail-stalled in landing) "
                 "vs resweep at higher θ_ht.  Dotted vertical line = pre-stall α cutoff used in linear fit.",
                 fontsize=11)
    out = REPO / "post/out/v2_continuous/alpha_resweep_vs_orig.png"
    fig.savefig(out, dpi=140)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
