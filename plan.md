# Tsangpo eSTOL — Project Plan & Technical Nuggets

This file is the running ledger of the **physical phenomena** the CFD campaign
must capture and the **engineering claims** the SciTech paper must defend.
Every nugget below should map to (a) a specific Flow360 case in
`params.study1_matrix()` and (b) a specific figure or table in
`scitech_outline.md`.

---

## Status Dashboard

| Phase | Artifact                                            | Status |
|------:|-----------------------------------------------------|--------|
| 1     | `plan.md` / `params.py` / `scitech_outline.md`      | locked |
| 2     | `geometry/tsangpo.csm` (Hershey wing + H-tail + deflected flap; ESP STL+STEP per case) | locked, builds clean for all 4 cases |
| 2     | `geometry/render.py` (top + side + zoomed-airfoil PNG) | locked |
| 3     | `flow360/run_matrix.py` + `case_template.json` (4 cases, 10 actuator disks each) | configured, not yet submitted |
| 4     | `post/extract_stability.py` (Cm slope + NP shift + non-linear-coupling decomposition) | runs against synthetic table; swaps to CFD forces.csv when populated |

The remaining unblockable step is the Flow360 submission itself — every
upstream artifact (parameters, geometry, case JSONs, post script) is in
place and re-derivable from `params.study1_matrix()`.

---

## Technical Nugget #1 — Vortex Upwash Through the Gap

**Claim.** The inboard flap gap is *not* a lift leak; it is an **energy
concentrator**. The two inboard propellers (Props 1 & 2) wash a high-
dynamic-pressure column rearward, and the suppressed inboard flap segments
allow that column to remain coherent until it reaches the H-tail, where it
produces a strong **upwash** on the lower surface of the stab.

**Physics to resolve in CFD**

- Streamwise coherence of the prop-wake "tube" from disk plane → tail LE.
- Vortex pair shed from the inboard edge of the *outboard* flap segment
  (the edge of the gap), which co-rotates with prop swirl on one side and
  counter-rotates on the other → expect asymmetric tail loading per side
  that cancels globally.
- Upwash angle $\Delta\alpha_\text{tail}$ at the H-tail quarter-chord.

**CFD requirements**

- Refinement box from each inboard disk, swept aft past
  $X_\text{tail} + 0.5\,$MAC.
- y+ ≤ 1 on H-tail upper and lower surfaces.
- Streamline seeds on a vertical rake just aft of the inboard disks.

**Pass/fail signature**

- $C_p$ on H-tail lower surface in C4 (Proposed Synthesis) shows a
  localized suction lobe centered on the gap streamtube, absent in C1
  (Industry Baseline).
- $\Delta C_m / \Delta\alpha$ increases (more negative) under power → larger
  static margin under power.

---

## Technical Nugget #2 — Mach / Tip-Speed Limits on the Distributed Array

**Claim.** With 10 small props doing the work of two big ones, the per-prop
tip Mach number governs noise and acoustic-fatigue margin more than it
governs performance. We must show **$M_\text{tip} \le 0.55$** at the design
point so the SciTech reviewer cannot dismiss the array on acoustics.

**Physics to resolve**

- Per-prop disk loading $T/A$ at $T/W = 0.5$, 2,600 lb, 12,000 ft.
- Tip helical Mach from RPM × R combined with forward speed at climb-out.

**Where defended**

- `params.py` carries `RHO_12K_SLUG_FT3`, `A_DISK_PER_PROP_FT2`, and a
  derived `DISK_LOADING_PSF`.
- Acoustic limit case is a **deferred** Phase 5 noise study; in Phase 3 we
  only need to confirm the actuator-disk thrust setting respects the limit.

**Pass/fail signature**

- Reported $M_\text{tip}$ < 0.55 at all 4 matrix points.

---

## Technical Nugget #3 — Slope-to-Sky Transition

**Claim.** The vehicle is sized for a **ramp takeoff** (rolling start
uphill, rotation initiated by tail-lift not by elevator) and a **whip-stall
landing** (short flare with deliberate tail-stall recovery into wing-borne
descent). Both maneuvers require **low-speed pitch authority well in
excess of CS-23 minima** — that is the *engineering reason* the gap exists.

**Physics to resolve**

- Pitch authority $\Delta C_m / \Delta\delta_e$ at $1.2\,V_s$ with the
  inboard props at static $T$.
- Neutral-point shift between Baseline and Proposed configurations.
- Tail effectiveness $\eta_t = q_\text{tail}/q_\infty$ — expected > 1 in
  the upwash column.

**Where defended**

- Phase 4 script `post/extract_stability.py` computes NP shift from a
  finite-difference $C_m(\alpha)$ sweep at fixed throttle.
- Figure: "$\eta_t$ map on the H-tail surface", C1 vs C4.

**Pass/fail signature**

- $\Delta\text{NP}_\text{aft} \ge 0.05\,\text{MAC}$ (C4 vs C1) at
  $T/W = 0.5$, $\alpha = 10°$.
- $\eta_t \ge 1.2$ over the gap-aligned strip of the H-tail.

---

## Open Questions / Risks

1. **Actuator-disk fidelity.** Does Flow360's AD model carry enough swirl
   to reproduce the upwash twist? If not, fall back to BET-disk.
2. **Trim drag bookkeeping.** A more powerful tail means we trim with less
   download → must report $L/D$ *trimmed*, not just $C_{L_\max}$.
3. **Gap edge separation.** The outboard flap's inboard edge sees a sharp
   spanwise gradient. Confirm no premature separation that would mask the
   upwash benefit.
4. **Mesh independence.** At minimum, one grid-refinement triple on C4
   for the SciTech defensibility appendix.

---

## Run-Order of Operations

1. Lock `params.py` (done).
2. Build `geometry/tsangpo.csm` STL/STEP for all 4 matrix cases via
   `flow360/build_geometry.py` (done).
3. Phase 3 mesh & run the 2x2: baseline (C1) first as sanity, then C2,
   C3, C4. Submit via `python flow360/run_matrix.py --submit`.
4. Phase 4 post: stability table → headline contrast (C2 vs. C4) →
   non-linear-coupling decomposition → paper section drafts.
