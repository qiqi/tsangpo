# Landing-phugoid session summary (2026-05-29)

## Deliverables

| What | File | Commit |
|---|---|---|
| 20-s slice MP4 (absolute Mach, baseline) | `post/v3/phugoid_runs/landing/nokick_landing_slices.mp4` | 9b8a776 |
| 20-s slice MP4 (LIC + Mach_rel, **relative to ambient air**) | `post/v3/phugoid_runs/landing/nokick_landing_slices_lic.mp4` | 2eb56b3 |
| Stitched trajectory + attitude (4-chunk, baseline) | `post/v3/phugoid_runs/landing/nokick_landing_trajectory.png` | 293c854/37290ad |
| Baseline-vs-retrim chunk1 comparison plot | `post/v3/phugoid_runs/landing_thtN2/baseline_vs_retrim_chunk1.png` | 37290ad |
| Documentation | `post/v3/phugoid_runs/landing/OPEN_LOOP_LANDING.md` | b76d8a2 |

## Retrim sweep at landing trim (V=12.86 m/s, α_BO=+27.63°, T_mult=14.17, γ_trim=−30°)

| θ_ht | CMy_warmup (4-cyl mesh) |
|---|---|
| −3.67° (steady 2-cyl baseline) | +1.241 |
| −2.00° | +1.011 |
| 0.00° | +0.759 |
| +3.00° | +0.345 |

Slope locked at **−0.134 ΔCMy per degree of θ_ht** across all 4 points.
Linear extrapolation: θ_ht ≈ **+5.6°** zeroes CMy.

## Open-loop dynamics (baseline θ_ht=−3.67° chunks 1-4)

* Trim is open-loop unstable; ∂CMy/∂θ > 0 in [−5°, +30°].
* No-kick perturbation drives half-loop divergence:
  * t=5 s: θ_body = +52.7°
  * t=9 s: **θ_body = +98° (half-loop apex)**
  * t=10 s: θ starts returning (CMy crosses zero at t=9.49 s)
  * t=15 s: solver fidelity exceeded; integrator continues into tumble

## Retrim chunk1 (θ_ht=−2°, case-484c926a)

Compared to baseline at t=5s:
* baseline θ=+52.7°  ⟷  retrim θ=**+36.9°** (30% slower divergence)
* CSV CMy values misleading: integrator uses moment about CURRENT CG;
  CSV reports about FIXED initial CG; lever-arm correction differs.
* CMy(θ) phase portrait shows **both curves cross zero at θ≈0°** —
  the natural trim of the open-loop airframe in this regime.

## Pending at end of session

* **#98** +3° chunk1 (case-f2507832): still queued behind its parent's
  remaining warmup steps.  At next session resume — check & compare
  trajectory to baseline.  Critical diagnostic for "htail trim was
  the lever (would slow ≥70%) vs intrinsic instability (similar
  divergence)."
* **#92** Re-run with corrected warmup pose (drop +γ_trim from
  theta_body_trim).  One-line fix to put the warmup at α=+27.6°
  instead of −2.4° (canonical CFD convention).  Independent of the
  retrim sweep.
* **#84** Landing-maneuver controller in CFD — project goal.

