"""
Pull the final-step total forces from each fork in the three calibration
sweeps on the `tsangpo_blunt_htail_active` Flow360 project, plot the
three coefficients (CL, CD, CMy) vs the swept parameter, fit linear
sensitivities, and write everything (CSV, plots, summary markdown) to
post/out/.

This script is the single source-of-truth tying each datapoint back to
its Flow360 caseId — re-running it from scratch will refetch every case
and regenerate the artefacts.  The cached CSV lets re-plots be fast.

    python3 post/plot_sweep_sensitivities.py            # use cache if present
    python3 post/plot_sweep_sensitivities.py --refresh  # force refetch
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
import params as P

OUT = REPO / "post" / "out" / "v1"
OUT.mkdir(exist_ok=True)

PROJECT_ID     = "prj-3e8b1ed8-0109-4f1f-8738-fef0c55a573b"  # tsangpo_blunt_htail_active
PARENT_CASE_ID = "case-9520355d-22ed-4f2a-bad0-eece1946594b" # cruise_SI_baseline (α=+7°, htail=0°)

ALPHA_CASES = [
    (-3.0, "case-62103e52-7956-4c52-851f-bc8b1d055dd5"),
    (-1.0, "case-8e472bfc-088d-4bb3-814b-d27e06c16b37"),
    (+1.0, "case-ffb614ea-6fca-4c4e-9b8a-fa0a51a0c3be"),
    (+3.0, "case-18446df1-d5e4-4286-bccd-05a6db158d72"),
    (+5.0, "case-6a5e5853-8240-44b9-9209-ef9cc5d9d4c0"),
    (+7.0, "case-7d14b0ab-9b80-443c-894a-037b4a77aec2"),
    (+9.0, "case-fe9b2fbc-db6b-49f4-b1e9-c1e825c3a756"),
    (+11.0, "case-2e247116-c66e-407f-8605-d1e6d75d68ad"),
    (+13.0, "case-a6fc399f-de8d-419a-b98c-8589b3d69a50"),
    (+15.0, "case-0376a381-07bb-4b52-8497-ccc2130f1364"),
]
HTAIL_CASES = [
    (-12.0, "case-c09282c9-caa2-4e95-a651-efa1f324c813"),
    ( -9.0, "case-7dbd5a59-59a2-4138-adcb-b99f06c08998"),
    ( -6.0, "case-7f77be58-8680-4fee-b412-a9cff1bed42a"),
    ( -3.0, "case-c1320127-c38b-42f8-b699-d313bbf90041"),
    (  0.0, "case-70e450cd-5e5f-4b82-9f68-f3fedbc22bff"),
    ( +3.0, "case-55dcf454-de8c-4c4d-9a16-47ca6609f8f8"),
    ( +6.0, "case-0aeb2041-21cc-478b-a003-5923478d04f2"),
    ( +9.0, "case-8eefccb9-8608-48a8-8039-e177fca0e5a3"),
    (+12.0, "case-c704699b-b68d-40f4-8f58-1a28e826862c"),
    (+15.0, "case-ad90944f-f90b-49d7-8940-03292de13ed6"),
]
THRUST_CASES = [
    (0.00, "case-d820f45a-3d8f-40ae-970c-3d50f24d89be"),
    (0.25, "case-59db849a-1bb5-4275-92c0-226ca2a6a12d"),
    (0.50, "case-c8d05bc2-fdb6-4662-84b1-bbae3a4a8f03"),
    (0.75, "case-b06cd35b-5284-4f4b-bec4-001b718eb63c"),
    (1.00, "case-70e450cd-5e5f-4b82-9f68-f3fedbc22bff"),  # = htail θ_ht=0 (dedup)
    (1.25, "case-9f8fcda4-1a44-4301-9ad0-b83064a0150d"),
    (1.50, "case-096eb33a-3482-4c1a-8253-6f5ce771209d"),
    (2.00, "case-5d110a48-b757-4916-95e7-d0b49680fed4"),
    (2.50, "case-c24fcad9-d5ec-47da-902a-8d66de570bd0"),
    (3.00, "case-2f5c1360-8755-4b45-97c4-852d79930a82"),
]

CSV_PATH = OUT / "sweep_data.csv"


def fetch_all() -> list[dict]:
    import flow360 as fl
    rows = []
    for sweep_name, cases in [("alpha",  ALPHA_CASES),
                              ("htail",  HTAIL_CASES),
                              ("thrust", THRUST_CASES)]:
        for val, cid in cases:
            c = fl.Case.from_cloud(cid)
            tf = c.results.total_forces
            tf.load_from_remote()
            v = tf.values
            ps = np.array(v["physical_step"])
            last = int(np.where(ps == ps.max())[0][-1])
            # Pull AD-delivered thrust (Mach-based non-dim ⇒ ρ a² L²
            # for force; integrate over all 10 disks).
            try:
                ad = c.results.actuator_disks
                ad.load_from_remote()
                av = ad.values
                F_nondim = sum(np.array(av[f"Disk{i}_Force"])[-1] for i in range(10))
                rho_a2_L2 = P.RHO_CRUISE_KG_M3 * P.A_SOUND_CRUISE_M_S ** 2 * 1.0 ** 2
                F_AD_N = F_nondim * rho_a2_L2
            except Exception:
                F_AD_N = float("nan")
            rows.append(dict(
                sweep=sweep_name, value=val, case_id=cid,
                CL=float(v["CL"][last]),
                CD=float(v["CD"][last]),
                CMy=float(v["CMy"][last]),
                CFx=float(v["CFx"][last]),
                F_AD_delivered_N=F_AD_N,
                physical_step=int(ps[last]),
                pseudo_step=int(v["pseudo_step"][last]),
            ))
            print(f"  {sweep_name:6s} {val:+6.2f}  {cid[:18]}  "
                  f"CL={rows[-1]['CL']:+.4f}  CD={rows[-1]['CD']:+.4f}  "
                  f"CMy={rows[-1]['CMy']:+.4f}")
    return rows


def load_or_fetch(refresh: bool) -> list[dict]:
    if CSV_PATH.exists() and not refresh:
        with CSV_PATH.open() as f:
            return [
                {k: (float(v) if k in {"value", "CL", "CD", "CMy", "CFx",
                                       "F_AD_delivered_N"} else v)
                 for k, v in r.items()}
                for r in csv.DictReader(f)
            ]
    print("Fetching from Flow360 …")
    rows = fetch_all()
    with CSV_PATH.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        for r in rows:
            w.writerow(r)
    print(f"Wrote {CSV_PATH}")
    return rows


def by_sweep(rows: list[dict], name: str) -> tuple[np.ndarray, dict]:
    sel = [r for r in rows if r["sweep"] == name]
    sel.sort(key=lambda r: r["value"])
    x = np.array([r["value"] for r in sel])
    data = {k: np.array([r[k] for r in sel])
            for k in ("CL", "CD", "CMy", "CFx", "F_AD_delivered_N")}
    # === Re-reference moment to a chosen CG and add thrust contribution. ===
    # The CFD moment_center was (0,0,0) (wing-root quarter-chord, z=0 chord
    # plane).  We re-reference to the CG at z_CG = -0.4 c (0.4 chords
    # beneath the wing LE, equivalently 0.1 chords below the thrust line
    # at z=-0.3 c).  Shifting the reference point DOWN by 0.4 c adds
    #   +0.4 · CFx   to CMy   (drag at the wing plane is now above CG ⇒
    #                           rearward force above CG ⇒ nose-up moment).
    # The thrust applied at the disks (z=-0.3 c) is 0.1 c above the CG and
    # acts in the −X (forward) direction with magnitude F_AD; it
    # contributes
    #   -0.1 · (F_AD / qS) = -0.1 · CT_delivered   to CMy  (nose-down).
    # Wall-integral CFx already EXCLUDES the AD body force, so the two
    # contributions are independent.
    qS = 0.5 * P.RHO_CRUISE_KG_M3 * P.V_CRUISE_M_S ** 2 * P.WING_AREA_M2
    CT_delivered = data["F_AD_delivered_N"] / qS
    data["CMy_CG"] = data["CMy"] + 0.4 * data["CFx"] - 0.1 * CT_delivered
    return x, data


def fit_linear(x, y, mask=None):
    if mask is None:
        mask = np.ones_like(x, dtype=bool)
    slope, intercept = np.polyfit(x[mask], y[mask], 1)
    return float(slope), float(intercept)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--refresh", action="store_true")
    args = ap.parse_args()

    rows = load_or_fetch(args.refresh)

    a, da = by_sweep(rows, "alpha")
    h, dh = by_sweep(rows, "htail")
    t, dt = by_sweep(rows, "thrust")

    # Sensitivities (linear fits, pre-stall region for α-sweep).  Moment
    # sensitivities use the CG-referenced + thrust-inclusive CMy_CG.
    mask_lin = a <= 9.0
    sens = {
        "dCL/dα      [/deg]":  fit_linear(a, da["CL"],     mask_lin)[0],
        "dCD/dα      [/deg]":  fit_linear(a, da["CD"],     mask_lin)[0],
        "dCMy/dα     [/deg]":  fit_linear(a, da["CMy_CG"], mask_lin)[0],
        "dCL/dθ_h    [/deg]":  fit_linear(h, dh["CL"])[0],
        "dCD/dθ_h    [/deg]":  fit_linear(h, dh["CD"])[0],
        "dCMy/dθ_h   [/deg]":  fit_linear(h, dh["CMy_CG"])[0],
        "dCL/dT_m   [/unit]":  fit_linear(t, dt["CL"])[0],
        "dCD/dT_m   [/unit]":  fit_linear(t, dt["CD"])[0],
        "dCMy/dT_m  [/unit]":  fit_linear(t, dt["CMy_CG"])[0],
    }
    for k, v in sens.items():
        print(f"  {k:24s} = {v:+.5f}")

    # === Plot: 3 rows × 3 cols ===========================================
    fig, axes = plt.subplots(3, 3, figsize=(13, 11), sharey=False)
    plt.subplots_adjust(left=0.08, right=0.97, top=0.94, bottom=0.07,
                        hspace=0.34, wspace=0.30)

    sweeps = [
        ("alpha",  a, da, r"$\alpha$ [deg]",
         f"α sweep (θ_htail=0°, T=1.0× cruise)\nparent={PARENT_CASE_ID[:13]}"),
        ("htail",  h, dh, r"$\theta_{\rm htail}$ [deg]",
         f"H-tail sweep (α=+7°, T=1.0× cruise)\nparent={PARENT_CASE_ID[:13]}"),
        ("thrust", t, dt, "thrust multiplier",
         f"Thrust sweep (α=+7°, θ_htail=0°)\nparent={PARENT_CASE_ID[:13]}"),
    ]
    cols = [
        ("CL",     r"$C_L$"),
        ("CD",     r"$C_D$"),
        ("CMy_CG", r"$C_{m,y}$  (about CG at $z=-0.4c$, incl. thrust)"),
    ]

    for row, (name, x, data, xlabel, title) in enumerate(sweeps):
        for col, (key, ylabel) in enumerate(cols):
            ax = axes[row, col]
            y = data[key]
            ax.plot(x, y, "o-", color="C0", lw=1.4, mfc="white", ms=6)
            ax.set_xlabel(xlabel)
            ax.set_ylabel(ylabel)
            ax.grid(True, alpha=0.3)
            # Linear-fit overlay for sensitivity panels (excluding post-stall)
            if name == "alpha":
                mask = x <= 9.0
            else:
                mask = np.ones_like(x, dtype=bool)
            if mask.sum() >= 2:
                slope, intercept = fit_linear(x, y, mask)
                xx = np.linspace(x.min(), x.max(), 50)
                ax.plot(xx, intercept + slope * xx, "--", color="C3", lw=0.9,
                        alpha=0.7,
                        label=f"slope = {slope:+.4f}/{xlabel.split()[0].strip('$')}")
                ax.legend(fontsize=7, loc="best", framealpha=0.85)
            if col == 0:
                # Put sweep title on left-most panel of each row
                ax.set_title(title, loc="left", fontsize=9, pad=8)

    fig.suptitle("Tsangpo cruise calibration sweeps  "
                 "(blunt-htail geometry, project prj-3e8b1ed8)  —  "
                 "moments re-referenced to CG (0.4c below LE) with "
                 "thrust contribution included",
                 fontsize=11, y=0.995)
    plot_path = OUT / "sensitivities.png"
    fig.savefig(plot_path, dpi=160)
    plt.close(fig)
    print(f"Wrote {plot_path}")

    # === Thrust-balance plot — drag (aircraft only) vs commanded thrust ==
    fig2, ax = plt.subplots(1, 1, figsize=(7, 5))
    qS = 0.5 * P.RHO_CRUISE_KG_M3 * P.V_CRUISE_M_S ** 2 * P.WING_AREA_M2
    drag_N = dt["CFx"] * qS
    cmd_thrust_N = t * 10 * P.T_CRUISE_PER_PROP_N
    delivered_N = dt["F_AD_delivered_N"]
    ax.plot(t, drag_N, "o-", lw=1.5, mfc="white", label="aircraft drag (wall integral)")
    ax.plot(t, cmd_thrust_N, "s--", lw=1.0, alpha=0.7, label="commanded AD thrust")
    if np.isfinite(delivered_N).all():
        ax.plot(t, delivered_N, "^-", lw=1.5, mfc="white",
                label="AD-delivered thrust (×ρ·a²·L²)")
    ax.set_xlabel("thrust multiplier  (×T_CRUISE_PER_PROP_N)")
    ax.set_ylabel("force  [N]")
    ax.set_title("Thrust vs drag balance at α=+7°, θ_htail=0°", fontsize=11)
    ax.grid(True, alpha=0.3)
    ax.legend()
    # find balance point: drag = delivered
    if np.isfinite(delivered_N).all():
        f = drag_N - delivered_N
        idx = np.where(np.diff(np.sign(f)))[0]
        if len(idx):
            i = idx[0]
            # linear interp
            x1, x2 = t[i], t[i + 1]
            f1, f2 = f[i], f[i + 1]
            t_bal = x1 - f1 * (x2 - x1) / (f2 - f1)
            ax.axvline(t_bal, color="C2", ls=":", alpha=0.6)
            ax.annotate(f"balance ≈ ×{t_bal:.2f}", (t_bal, drag_N.mean()),
                        xytext=(8, 0), textcoords="offset points",
                        fontsize=9, color="C2")
    plot2_path = OUT / "thrust_balance.png"
    fig2.tight_layout()
    fig2.savefig(plot2_path, dpi=160)
    plt.close(fig2)
    print(f"Wrote {plot2_path}")

    # === Summary printout =================================================
    print("\n=== Sensitivities (linear fits) ===")
    sm_alpha = -sens["dCMy/dα     [/deg]"] / sens["dCL/dα      [/deg]"]
    print(f"  static margin (-dCMy/dα / dCL/dα) = {sm_alpha:.3f}  (= {sm_alpha*100:.1f}% MAC)")
    cl_cruise = da["CL"][a == 7][0]
    cmy_cg_cruise = da["CMy_CG"][a == 7][0]
    print(f"  cruise α=+7° CL = {cl_cruise:.4f}  → L/W = "
          f"{cl_cruise * qS / P.W_GROSS_N:.3f}")
    print(f"  cruise α=+7° CMy_CG (incl. thrust at ×1.0) = {cmy_cg_cruise:+.4f}")
    print(f"  trim Δθ_htail (CMy_CG → 0 at α=+7°, T=cruise) = "
          f"{-cmy_cg_cruise / sens['dCMy/dθ_h   [/deg]']:+.2f}°")
    print(f"  CL_max (in sweep) ≈ {da['CL'].max():.3f} at α=+{a[np.argmax(da['CL'])]:.0f}°  "
          f"(stall onset ≈ +11–13°)")


if __name__ == "__main__":
    main()
