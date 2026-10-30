# Adaptive-sampling map of the stability paradox (full paper, §7.3)

## Goal

Map the boundary of the gap-low / v3 stability paradox region in
the continuous 5-D design space
$(f_\text{gap},\,z_{ht},\,x_{ht},\,\alpha,\,T/L)$ at the landing
trim point, replacing the four-corner $2\times 2$ sample of the
extended abstract with a published boundary surface.

The headline question the full paper must answer: is the paradox
a finite-volume basin that survives perturbation along each design
axis, or a knife-edge that the $2\times 2$ happens to straddle?
The honest answer determines how strongly the v3 configuration can
be recommended outside the exact operating point used in the
extended abstract.

## Method

Adaptive sampling with the following architecture (the same
architecture validated on the NASA X-57 Maxwell at a 2,333-case
BET-RANS oracle):

1. **AVL linear prior.** A vortex-lattice model of each
   configuration provides a cheap, gradient-aware prior for the
   static-aero coefficients at any design-state vector. AVL does
   not know slipstream blowing, separation, or upwash-column
   physics; that is precisely the residual the GP learns.
2. **AVL-residual Gaussian-process surrogate.** Each new Flow360
   point teaches the GP correction layer; the AVL prior + GP
   posterior is the working surface and the source of
   reconstructed sensitivity slopes.
3. **Reversal-risk acquisition.** Candidate points are scored by
   a combination of posterior uncertainty, prior–posterior
   disagreement, and a reversal-risk channel that up-weights
   regions where the surrogate's
   $\partial C_{m_\text{CG}}/\partial\alpha$ is close to zero
   (i.e. the paradox boundary itself). Acquisition mechanics are
   proprietary to Flexcompute and are not reproduced here; the
   full paper cites the X-57 result and reports outcomes.
4. **Learned next-point policy (NN augmentation).** The
   hand-crafted acquisition is supplemented by a compact
   convolutional encoder/decoder trained on a large corpus of
   synthetic aerodynamic databases where the oracle surface is
   known. The network takes the current state of the campaign
   (sparse acquired samples, the AVL prior field, the current
   GP correction, prior gradient/saliency planes, and a
   metric-conditioning channel) and outputs a next-point utility
   map. The learned policy is then mixed with the mathematical
   acquisition arms so that the sampler can trade across
   value-RMSE, gradient-RMSE, feature-object identity, and
   Sobolev-style smooth-field error as the operating-relevance
   weighting demands.
5. **Sobolev-style success metric.** Reconstruction quality is
   judged in a Sobolev-type norm $\|\cdot\|_{H^1}$ (or $H^2$) on
   $C_{m_\text{CG}}$, weighted by engineering tolerances on
   value, $\partial C_{m_\text{CG}}/\partial\alpha$, and its
   curvature. This is the honest metric for a stability-margin
   problem — broad value-RMSE understates a sampler that targets
   slope features. The X-57 study showed gradient-field RMSE
   $\sim 52\%$ lower than LHS at small budgets; Sobolev-norm
   improvement on $C_{m_\text{CG}}$ is the headline figure for
   the Tsangpo full-paper result.
6. **Termination.** Reseed-stability: rerun the campaign with
   $k$ independent random seeds; declare convergence when the
   recovered boundary surface is reproducible to within the
   chosen $H^1$ tolerance.

## Design-space axes

| Axis           | Range                          | Notes                                |
|----------------|--------------------------------|--------------------------------------|
| $f_\text{gap}$ | $[0.00,\,0.60]$                | continuous; currently fixed at 0.40  |
| $z_{ht}$       | $[-0.2,\,+2.0]\,c_w$           | continuous                           |
| $x_{ht}$       | $[+1.5,\,+4.5]\,c_w$ aft of CG | continuous                           |
| $\alpha$       | $[+5,\,+30]^\circ$             | continuous                           |
| $T/L$          | $[0.0,\,1.5]$                  | continuous; delivered, not commanded |

Phase fixed at landing for the abstract-deadline contribution.
Takeoff and cruise are obvious follow-on extensions.

## Deliverables

- **Figure (primary):** 2-D projections of the recovered
  paradox-region boundary at fixed $f_\text{gap}=0.40$ — e.g.
  $(z_{ht}, x_{ht})$ at landing trim — with the AVL prior-only
  baseline overlaid as the "what the cheap model would predict"
  reference.
- **Figure (convergence):** recovered-boundary $L^2$ error vs
  sample budget, adaptive vs LHS+GP baseline. The reseed envelope
  is shown as a band, not a single trace.
- **Headline number:** $N_\text{adaptive}$ to converge vs
  $N_\text{uniform}$ for matching boundary resolution. Ratio is
  the experimental result, not a prediction; X-57 ratios provide
  the order-of-magnitude precedent.
- **Optional:** a single out-of-sample CFD verification point
  where the surrogate predicted the boundary location, run as a
  blind test, to demonstrate generalisation beyond the training
  set.

## CFD tasks needed

1. **AVL models** of cont-low, cont-high, gap-high, gap-low, and
   v3 — parameterised so the same model can be re-trimmed at any
   $(f_\text{gap}, z_{ht}, x_{ht}, \alpha, T/L)$ in the box,
   including a slipstream approximation for the htail station.
2. **Parametric CSM generation** for the 5-D box. The existing
   `geometry/tsangpo.csm` is phase-parameterised in $\alpha$ via
   `-despmtrs`; we extend the same machinery to expose
   $(z_{ht}, x_{ht}, f_\text{gap})$ as design variables. Geometry
   guards are needed at extreme $x_{ht}$ to avoid tail-boom /
   empennage collisions.
3. **Submission harness.** The adaptive sampler drives Flow360 in
   a loop: it passes a 5-tuple, the harness generates the CSM,
   submits via `cfd_setup.py` with `use_beta_mesher=True`, polls
   for completion, and returns $(C_L, C_D, C_{m_\text{CG}})$ plus
   the surface integrals needed for sensitivity reconstruction.
4. **Case budget (order-of-magnitude).** Scaling from the X-57
   result and the 5-D dimensionality suggests $\mathcal{O}(10^2)$
   adaptive Flow360 cases at the landing trim point for a
   converged boundary, against $\mathcal{O}(10^4)$ for an
   equivalent-resolution uniform grid. The exact ratio is the
   experimental result.

## Risks and cautions

- **Parametric CSM coverage.** OpenCSM must produce valid
  geometries across the full 5-D box, not just the four existing
  $2\times 2$ corners. Boom-empennage collisions at extreme
  $x_{ht}$ and gap-fraction ranges above 0.5 are the obvious
  failure modes.
- **Mesh consistency.** Every candidate point must run with the
  beta mesher and a fixed refinement budget; mesh noise leaking
  into the GP residual would manifest as spurious "features" and
  drive the sampler to waste budget on numerical artefacts.
- **AVL slipstream approximation.** AVL has no native model of
  the prop wake; the prior's usefulness depends on how the
  10-prop slipstream column is represented (effective-AoA
  perturbation across the htail strip is the leading candidate).
  A sensitivity check on prior choice should precede the
  full campaign.
- **NN training-distribution transfer.** The next-point network
  was trained on synthetic aerodynamic databases with a
  particular distribution of feature objects (ridges, breaks,
  reversals). The Tsangpo $C_{m_\text{CG}}$ surface has
  feature topology shaped by upwash–slipstream interaction,
  which may sit outside the training distribution. Watch for
  the learned policy underperforming the mathematical arms; if
  so, fall back to the hand-crafted acquisition or retrain the
  policy on a Tsangpo-specific synthetic corpus.
- **Single-phase scope.** Restricting this contribution to the
  landing trim point is a deliberate scoping decision; extending
  to all three phases multiplies the CFD budget linearly but
  adds no new methodology. The full paper may take that
  extension if cycles permit.

## Dependencies

- Completion of the v3 Flow360 sensitivity sweep currently in
  flight at $(f_\text{gap}=0.40,\,z_{ht}=+0.4\,c_w,\,
  x_{ht}=+2.0\,c_w)$ — this is the seed point of the
  adaptive campaign and the GP needs it to anchor the residual.
- Parametric CSM generation across the 5-D box.
- AVL setup for the Tsangpo planform with the chosen slipstream
  approximation.

## Out of scope for this contribution

- Phugoid dynamics (covered by §7.1 / `phugoid_dynamics_plan.md`).
- Ground-effect landing maneuver (covered by §7.2 /
  `ground_effect_landing_plan.md`).
- Disclosure of acquisition-function internals (proprietary
  to Flexcompute).
