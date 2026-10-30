# SciTech 2027 paper

`extended_abstract.tex` is the AIAA SciTech 2027 extended abstract
(`\documentclass[conf]{new-aiaa}`; `new-aiaa.cls` is committed). The
PDF in this folder is the last build; rebuild with the script below.

## Deadlines

- Abstract / extended abstract: 21 May 2026 (submitted).
- Author notification: 24 Aug 2026. **Outcome not recorded here; add it.**
- **Final manuscript: 1 Dec 2026, 8:00 PM ET.**

## Build

```
bash paper/build.sh
```

The script refits every per-phase sensitivity from the cached sweep CSVs
under `post/out/`, rebuilds the gap-low and v3 equilibrium grids,
re-renders the generated figures, writes `gitinfo.tex` with the source
commit (quoted in the reproducibility section), and runs pdflatex and
bibtex. No Flow360 access is needed. The downwash slice figures
(`figures/downwash_*.png`) and the phugoid setup / trace figures are
committed outputs of `post/v3/render_downwash*.py` and
`figures/_build_phugoid_{setup,traces}.py`; the first needs the slice
caches that `figures/_retrieve_slices.py` downloads, the second only
`flow360/unsteady_setup.py` and the committed UDD logs.

## Structure (as written)

1. Introduction.
2. Configurational evolution: Step 0 cont-low → Step 1 cont-high →
   Step 2 gap-high → Step 3 gap-low → Step 4 v3 short boom.
3. Analysis of v3: trimmable envelope, slipstream coverage, flare
   geometry, cautions.
4. CFD-based dynamic maneuver simulation: nested rotation zones + UDD
   3-DOF, the cruise phugoid (T = 17.4 s, ζ ≈ 0.026), two implementation
   lessons.
5. Expected full-paper contributions.
6. Geometry reference.

## Open items for the final manuscript

- Record the acceptance decision above.
- The landing open-loop result (statically unstable trim, half-loop
  divergence; `post/v3/phugoid_runs/landing/OPEN_LOOP_LANDING.md`) and
  the θ_eq(θ_ht, V_inf) grid are not in the abstract and belong in the
  dynamics section of the full paper.
- The integrator has since changed to symplectic Euler–Cromer
  (`flow360/unsteady_setup.py`); the abstract's phugoid number was produced
  with explicit Euler. Say so if the number is kept.
- `paper/sections/phugoid_dynamics_plan.md` and
  `adaptive_sampling_paradox_map_plan.md` are the full-paper plans; the
  first predates the UDD method and is marked superseded.
