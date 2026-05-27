# Flow360 actuator-disk "delivered vs commanded thrust" — investigated & RESOLVED (not a solver bug)

**Original report:** Qiqi Wang, 2026-05-16 (solver release-25.9, beta mesher).
**Resolution:** Feilin Jia (Flexcompute, assisted by Claude), 2026-05-19 —
verified across all 8 takeoff cases (4 coarse + 4 fine).
**Status:** **CLOSED — measurement-script unit error, not a solver defect.**

> This file previously asserted a dramatic actuator-disk under-delivery
> (a 1.10 / 0.70 split across configs, decaying to a 0.65 "fine-mesh
> asymptote"). That conclusion was wrong. The split was an artifact of a
> hard-coded reference-pressure constant in the measurement script. The
> solver delivers the commanded thrust to ~10 % (coarse) / ~2 % (fine).
> The corrected analysis is below; the original (incorrect) write-up is
> retained in git history (this file was renamed from `FLOW360_AD_BUG_REPORT.md`).

## Root cause — wrong `ρ∞·a∞²` in the measurement script

The solver writes `Disk_i_Force` non-dimensionalized by **each case's own**
`ρ∞·a∞²`. The reproducer script `submit_repro.py` re-dimensionalized with a
**hard-coded** `ρ∞·a∞² = 90,250 N/m²` (ISA 12,000 ft: ρ=0.849, a=326), while
**only 1 of the 12 cases actually ran at 12,000 ft** — the other 11 were
submitted at sea level, where `ρ∞·a∞² = 141,829 N/m²`. The mismatch under-counts
the dimensional thrust by exactly

```
141,829 / 90,250 = 1.5717
```

which reproduces every "discrepant" number:

```
correct 1.10 / 1.5717 = 0.700   ← the apparent "0.70 camp"
correct 1.02 / 1.5717 = 0.649   ← the apparent "0.65 fine asymptote"
```

The fix is to compute `ρ∞·a∞²` from each case's actual altitude (read it back
from `case.params`, or derive ρ and a together via
`from_standard_atmosphere(altitude=...)`), never a hard-coded constant.

## Corrected delivery ratios (each case's own freestream)

`commanded_total = 10 × 973.74 N/m² × π·(0.5335² − 0.080025²) = 8,511.0 N`.

| mesh | config | case id | alt [m] | ρa² [N/m²] | Σ Disk_i_Force (nd) | delivered [N] | ratio |
|------|--------|---------|--------:|-----------:|--------------------:|--------------:|------:|
| coarse | v2_continuous      | case-118aced8 | 3658 |  90,220 | 0.103873 | 9,371.5 | **1.101** |
| coarse | v2_continuous_high | case-ab08db57 |    0 | 141,829 | 0.066062 | 9,369.5 | **1.101** |
| coarse | v2_gapped          | case-fe40382d |    0 | 141,829 | 0.066061 | 9,369.3 | **1.101** |
| coarse | v2_gapped_high     | case-a1bf0628 |    0 | 141,829 | 0.066071 | 9,370.7 | **1.101** |
| fine   | v2_continuous      | case-3d97afde |    0 | 141,829 | 0.060974 | 8,647.9 | **1.016** |
| fine   | v2_continuous_high | case-ad854db9 |    0 | 141,829 | 0.060974 | 8,647.9 | **1.016** |
| fine   | v2_gapped          | case-decca8e7 |    0 | 141,829 | 0.060974 | 8,647.9 | **1.016** |
| fine   | v2_gapped_high     | case-0ec31ee6 |    0 | 141,829 | 0.060974 | 8,647.9 | **1.016** |

All four configs agree to ≤0.04 % at each mesh. **There is no
geometry-far-from-the-disk effect** — that apparent spread was the unit error.

## The small, real residuals (after the unit error is removed)

1. **Coarse ~+8 % over-delivery, fine ~+2 %.** Coarse-mesh artifact from binary
   cell-membership at the prop-cylinder boundary plus the 2-axial-cell
   quadrature of the cosine weight in the AD kernel
   (`ActuatorDisk.h`; `NavierStokesSolver.cpp:1611-1640`). Real but mild;
   fixing it would require partial-volume weighting instead of binary
   cell-membership. Low priority.
2. **Fine ~1.016, not 1.000** — `linearInterp`
   (`Flow360Math.h:309-320`) clamps to `ySample[0]` for `r < radius[0]`, so the
   body force is applied **through the hub region**, not just the annulus
   `[r_inner, R]`. With `radius = [0.080025, 0.5335]` the exact prediction is
   `1 / (1 − (r_inner/R)²) = 1 / (1 − 0.15²) = 1.0231`, matching the measured
   1.016 within discretization error. If a pure annular force (zero for
   `r < r_inner`) is wanted, that clamp behaviour is a `force_per_area`
   semantics question for the solver team — separate from this (closed)
   discrepancy.

## Implication for downstream work

The disk delivers commanded thrust to within ~2 % on a converged (fine) mesh,
so treating `T_disk` as the commanded total in the phugoid UDD's disk-reaction
moment (`−prop_z·T_disk`) is accurate to a couple of percent; the coarse-mesh
~8 % is the only caveat if a coarse AD refinement is used. No solver-side
correction is needed.

## Lessons

- **Never hard-code `ρ∞·a∞²` (or any freestream reference) in a post-processing
  script that spans cases at different altitudes.** Read each case's own
  freestream from `simulation.json` / `case.params`.
- A clean "constant ratio across unrelated geometry changes" is a red flag for
  a per-case normalization error, not a physical effect — here it was the same
  1.5717 factor masquerading as a 1.10-vs-0.70 split.
