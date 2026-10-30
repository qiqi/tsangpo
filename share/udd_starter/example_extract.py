"""Extract θ_eq with 95 % CI from a damped pitch-only chunk.

Reads IDs from /tmp/udd_starter_ids.json (written by example_submit.py),
downloads the UDD CSV, fits Δθ_k = A·exp(-r·k) to the LAST 4 deltas,
applies two t-tests, and reports θ_eq + CI.

This is the same algorithm used by the production grid sweep — copy
this function directly into your own pipeline if it suits.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import flow360 as fl
import numpy as np
import pandas as pd
from scipy.optimize import curve_fit
from scipy import stats as ss


def load_trajectory(chunk_case_id: str, step_offset: int = 3) -> np.ndarray:
    """Download the airframe UDD CSV and return θ_body (deg) at end-of-step
    for every physical step in the chunk (i.e. step ≥ step_offset)."""
    c = fl.Case.from_cloud(case_id=chunk_case_id)
    out = Path(f"/tmp/udd_starter_{chunk_case_id[:13]}/")
    out.mkdir(parents=True, exist_ok=True)
    c._download_file("results/udd_pitch_cyl3int_airframe_v2.csv",
                     to_folder=str(out), overwrite=True)
    df = pd.read_csv(out / "udd_pitch_cyl3int_airframe_v2.csv",
                     sep=r"\s*,\s*", engine="python")
    end = df.groupby("physical_step").tail(1).reset_index(drop=True)
    chunk = end[end["physical_step"] >= step_offset]
    return chunk["state[4]"].apply(math.degrees).to_numpy()


def estimate_theta_eq(theta_traj: np.ndarray, alpha: float = 0.05
                      ) -> tuple[float, float, str, dict]:
    """Return (θ_eq, 95% CI half-width, status, diagnostics).

    Status is one of:
      'converged'  — mean of last 4 Δθ indistinguishable from 0
                     (body settling around equilibrium with noise);
                     θ_eq = mean of last 4 θ, CI = half-range
      'drift'      — Δθ significantly nonzero but no significant decay;
                     no point estimate, CI = ∞
      'exp_OK'    — Δθ ≠ 0 AND r > 0 both significant;
                     extrapolate the geometric tail past k=3
      'too_short' — fewer than 5 θ samples; can't compute deltas
      'fit_fail'  — curve_fit raised
    """
    if len(theta_traj) < 5:
        return float(theta_traj[-1]), float("inf"), "too_short", {}

    deltas = np.diff(theta_traj)
    last4_d  = deltas[-4:]
    last4_th = theta_traj[-4:]
    n        = 4

    # ── Test 1: is the mean of the last 4 Δθ significantly nonzero?
    # operational meaning of "deltas are just noise around zero" → converged
    mean_d  = float(last4_d.mean())
    std_d   = float(last4_d.std(ddof=1))
    se_mean = std_d / math.sqrt(n)
    t_mean  = mean_d / max(se_mean, 1e-12)
    t_crit1 = ss.t.ppf(1 - alpha/2, n-1)         # df=3, two-sided → 3.18
    if abs(t_mean) < t_crit1:
        theq = float(last4_th.mean())
        ci   = max(0.5 * (last4_th.max() - last4_th.min()), 0.02)
        return theq, ci, "converged", {"t_mean": t_mean}

    # ── Test 2: fit Δ_k = A·exp(-r·k); is r > 0 significant?
    try:
        A0 = last4_d[0] if abs(last4_d[0]) > 1e-6 else mean_d
        p, pcov = curve_fit(
            lambda k, A, r: A * np.exp(-r * k),
            np.arange(4, dtype=float), last4_d,
            p0=[A0, 0.1], maxfev=10000)
        A_hat, r_hat = float(p[0]), float(p[1])
        sA = float(math.sqrt(max(pcov[0,0], 0)))
        sR = float(math.sqrt(max(pcov[1,1], 0)))
        cov_Ar = float(pcov[0,1])
    except Exception as ex:
        return float(theta_traj[-1]), float("inf"), "fit_fail",  {"err": str(ex)}

    t_r     = r_hat / max(sR, 1e-12)
    t_crit2 = ss.t.ppf(1 - alpha, n-2)            # df=2, one-sided → 2.92
    if t_r < t_crit2:
        return float(theta_traj[-1]), float("inf"), "drift", {
            "t_mean": t_mean, "A": A_hat, "r": r_hat, "t_r": t_r}

    # ── Extrapolate the geometric tail past k=3
    er = math.exp(-r_hat)
    g  = A_hat * math.exp(-4*r_hat) / (1 - er)
    theq = float(theta_traj[-1] + g)

    # Propagate CI via analytic gradient of g(A, r)
    dg_dA = math.exp(-4*r_hat) / (1 - er)
    dg_dr = -A_hat * math.exp(-4*r_hat) * (4 - 3*er) / (1 - er)**2
    sigma_g_sq = (dg_dA**2 * sA**2
                  + dg_dr**2 * sR**2
                  + 2 * dg_dA * dg_dr * cov_Ar)
    sigma_g = math.sqrt(max(sigma_g_sq, 0))
    # df=2 two-sided 95% t-critical = 4.30 — honest small-sample correction
    ci = float(ss.t.ppf(1 - alpha/2, n-2) * sigma_g)
    diag = {"t_mean": t_mean, "A": A_hat, "r": r_hat, "t_r": t_r,
            "sigma_g": sigma_g}
    return theq, ci, "exp_OK", diag


if __name__ == "__main__":
    ids = json.loads(Path("/tmp/udd_starter_ids.json").read_text())
    print(f"Cell θ_ht={ids['theta_ht_deg']:+.2f}°  V={ids['V_inf_m_s']} m/s")

    theta_traj = load_trajectory(ids["chunk"])
    print(f"\nθ trajectory ({len(theta_traj)} points):")
    for k, th in enumerate(theta_traj):
        print(f"  k={k}  θ={th:+.4f}°")
    if len(theta_traj) >= 2:
        deltas = np.diff(theta_traj)
        print(f"\n  Δθ: {', '.join(f'{d:+.4f}' for d in deltas)}")

    theq, ci, status, diag = estimate_theta_eq(theta_traj)
    ci_str = f"±{ci:.3f}°" if math.isfinite(ci) else "±∞"
    print(f"\n→ θ_eq = {theq:+.4f}°  95% CI = {ci_str}    [status: {status}]")
    for k, v in diag.items():
        if isinstance(v, float):
            print(f"    {k:8s} = {v:+.4f}")
