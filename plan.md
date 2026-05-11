# Himalayan eSTOL — Project Plan & Technical Nuggets

This file is the running ledger of the **physical phenomena** the CFD campaign
must capture and the **engineering claims** the SciTech paper must defend.
Every nugget below should map to (a) a specific Flow360 case and (b) a
specific figure or table in `scitech_outline.md`.

---

## Status Dashboard

| Phase | Artifact                       | Owner   | Status |
|------:|--------------------------------|---------|--------|
| 1     | `plan.md` / `params.py` / `scitech_outline.md` | Cursor  | scaffolded |
| 2     | `geometry/himalaya.csm` (ESP)  | ESP     | parametric stub |
| 3     | `flow360/run_matrix.json` + driver | Flow360 | configured, not yet submitted |
| 4     | `post/extract_*.py`            | Cursor  | stubs in place |

---

## Technical Nugget #1 — Vortex Upwash Through the Gap

**Claim.** The inboard flap gap is *not* a lift leak; it is an **energy
concentrator**. The two inboard propellers (Props 1 & 2) wash a high–dynamic-
pressure column rearward, and the suppressed inboard flap segments allow that
column to remain coherent until it reaches the H-tail, where it produces a
strong **upwash** on the lower surface of the stab.

**Physics to resolve in CFD**
- Streamwise coherence of the prop-wake "tube" from disk plane → tail LE.
- Vortex pair shed from the inboard edge of the *outboard* flap segment (the
  edge of the gap), which co-rotates with prop swirl on one side and counter-
  rotates on the other → expect asymmetric tail loading per side that cancels
  globally.
- Upwash angle Δα_tail at the H-tail quarter-chord.

**CFD requirements**
- Refinement box from each inboard disk, swept aft past `X_tail + 0.5·MAC`.
- y+ ≤ 1 on H-tail upper and lower surfaces.
- Streamline seeds on a vertical rake just aft of the inboard disks.

**Pass/fail signature**
- `Cp` on H-tail lower surface in the *Proposed* case shows a localized suction
  lobe centered on the gap streamtube, absent in *Baseline*.
- ΔCm / Δα increases (more negative) → larger static margin under power.

---

## Technical Nugget #2 — Mach / Tip-Speed Limits on the Distributed Array

**Claim.** With 10 small props doing the work of two big ones, the per-prop
tip Mach number governs noise and acoustic-fatigue margin more than it governs
performance. We must show **M_tip ≤ 0.55** at the design point so the
Scitech reviewer cannot dismiss the array on acoustics.

**Physics to resolve**
- Per-prop disk loading T/A at T/W = 0.5, 2,600 lb, 12,000 ft.
- Tip helical Mach from RPM × R combined with forward speed at climb-out.

**Where defended**
- `params.py` carries `RHO_12K`, `A_DISK_PER_PROP`, and a derived
  `disk_loading_psf`.
- Acoustic limit case is a **deferred** Phase 5 noise study; in Phase 3 we
  only need to confirm the actuator-disk thrust setting respects the limit.

**Pass/fail signature**
- Reported M_tip in `post/derived_quantities.csv` < 0.55 at all run-matrix
  points.

---

## Technical Nugget #3 — Slope-to-Sky Transition

**Claim.** The vehicle is sized for a **ramp takeoff** (rolling start uphill,
rotation initiated by tail-lift not by elevator) and a **whip-stall landing**
(short flare with deliberate tail-stall recovery into wing-borne descent).
Both maneuvers require **low-speed pitch authority well in excess of CS-23
minima** — that is the *engineering reason* the gap exists.

**Physics to resolve**
- Pitch authority ΔCm / Δδ_e at 1.2·Vs with the inboard props at static T.
- Neutral-point shift between Baseline and Proposed configurations.
- Tail effectiveness η_t = (q_tail / q_∞) — expected > 1 in the upwash column.

**Where defended**
- Phase 4 script `post/extract_stability.py` computes NP shift from a
  finite-difference Cm(α) sweep at fixed throttle.
- Figure: "η_t map on the H-tail surface", Baseline vs Proposed.

**Pass/fail signature**
- ΔNP_aft ≥ 0.05·MAC (Proposed vs Baseline) at T/W = 0.5, α = 10°.
- η_t ≥ 1.2 over the gap-aligned strip of the H-tail.

---

## Open Questions / Risks

1. **Actuator-disk fidelity.** Does Flow360's AD model carry enough swirl to
   reproduce the upwash twist? If not, fall back to BET-disk.
2. **Trim drag bookkeeping.** A more powerful tail means we trim with less
   download → must report L/D *trimmed*, not just CL_max.
3. **Gap edge separation.** The outboard flap's inboard edge sees a sharp
   spanwise gradient. Confirm no premature separation that would mask the
   upwash benefit.
4. **Mesh independence.** At minimum, one Proposed-case grid refinement study
   for the SciTech defensibility appendix.

---

## Run-Order of Operations

1. Lock `params.py` (this commit).
2. Build `geometry/himalaya.csm`, generate STEP for `gap_fraction ∈ {0, 0.20}`.
3. Phase 3 mesh & run matrix: Baseline first (sanity), then Proposed, then
   the `Z_tail` sweep.
4. Phase 4 post: stability table → figures → paper section drafts.
