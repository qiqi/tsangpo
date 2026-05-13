# Tsangpo eSTOL — Project Plan & Technical Nuggets

This file is the running ledger of the **physical phenomena** the CFD campaign
must capture and the **engineering claims** the SciTech paper must defend.
Every nugget below should map to (a) one of the live Flow360 campaigns and
(b) a specific figure or table in `scitech_outline.md`.

---

## Status Dashboard

| Phase | Artifact                                            | Status |
|------:|-----------------------------------------------------|--------|
| 1     | `plan.md` / `params.py` (SI) / `scitech_outline.md` | locked |
| 2     | `geometry/airfoils/build_estol_geometry.py` + `estol_config.yaml` (coved LS(1)-0417 main + NACA 9621 vane + NACA 6311 aft-flap + inverted-LS(1)-0417 tail; UDC splines + Selig `.dat` exports) | locked |
| 2     | `geometry/tsangpo.csm` (phase 0/1/2 → stowed/takeoff/landing; rigid Fowler vane+flap; despmtr `gap_fraction`, `Z_tail_chords`, `X_tail_mac`) | locked; per-phase STL+STEP in `geometry/out/{stowed,takeoff,landing}/` |
| 2     | `geometry/render.py` (2 × 3 STL → planform + side-section per phase) | locked |
| 3a    | Cruise calibration sweep (α, θ_ht, T_mult; 30 forks on `prj-3e8b1ed8`) | complete — see `post/SENSITIVITIES.md` |
| 3b    | GAI-meshed cruise trim campaign (`flow360/submit_gai_trim_campaign.py`) | queued at trim ≈ (α=+6.78°, θ_ht=−0.54°, T_mult=+2.13) |
| 3c    | Takeoff coarse-mesh campaign at phase-1 flap (`flow360/submit_takeoff_coarse_campaign.py`) | spec'd — see `post/TAKEOFF_PLAN.md` |
| 3d    | Study-1 2×2 matrix (`gap_fraction` × `Z_tail_chords`; `params.study1_matrix()`) | **not yet run** — depends on a single-script SI driver replacing the deleted `run_matrix.py` |
| 4     | `post/plot_sweep_sensitivities.py` → `post/SENSITIVITIES.md` | runs against live forces; sensitivities + thrust balance plots done |
| 4     | `post/extract_stability.py` (4-case `C_m_α` + neutral-point shift + linear/non-linear decomposition) | runs against synth table; swaps to CFD forces.csv when Study 1 lands |

Live driver scripts (SI, SDK-based, all inline the UDCs into `tsangpo.csm`
before upload to Flow360):

```
flow360/submit_cruise.py                  # parent cruise case
flow360/submit_alpha_sweep.py             # α fork sweep
flow360/submit_htail_sweep.py             # θ_htail fork sweep
flow360/submit_thrust_sweep.py            # T_mult fork sweep
flow360/submit_gai_trim_campaign.py       # GAI-meshed trim + 3-sweep refinement
flow360/submit_takeoff_coarse_campaign.py # phase-1 takeoff campaign (Phase A)
flow360/submit_rotation_test.py           # rotation-volume sanity check
flow360/generate_surface_mesh_gai.py      # GAI surface-mesh probe
flow360/LESSONS.md                        # Flow360 quirks paid for in credits
```

---

## Technical Nugget #1 — Vortex Upwash Through the Gap

**Claim.** The inboard flap gap is *not* a lift leak; it is an **energy
concentrator**. The two inboard propellers (Props 1 & 2) wash a high-
dynamic-pressure column rearward, and the suppressed inboard flap segments
allow that column to remain coherent until it reaches the H-tail, where it
produces a strong **upwash** on the lower surface of the stab.

**Physics to resolve in CFD**

- Streamwise coherence of the prop-wake "tube" from disk plane → tail LE.
- Vortex pair shed from the inboard edge of the *outboard* vane+flap
  segment (the edge of the gap), which co-rotates with prop swirl on one
  side and counter-rotates on the other → expect asymmetric tail loading
  per side that cancels globally.
- Upwash angle $\Delta\alpha_\text{tail}$ at the H-tail quarter-chord.

**CFD requirements**

- `fl.UniformRefinement` boxes from each inboard disk swept aft past the
  H-tail (`PROP_REFINE_SPACING ≈ 0.05·MAC`, as set in `submit_cruise.py`).
- y+ ≤ 1 on H-tail upper and lower surfaces
  (`boundary_layer_first_layer_thickness = 7.62e-6 m`).
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

- Per-prop disk loading $T/A$ at $T/W = 0.5$, 11,565 N, 12,000 ft.
- Tip helical Mach from RPM × R combined with forward speed at climb-out.

**Where defended**

- `params.py` carries `RHO_CRUISE_KG_M3`, `A_DISK_PER_PROP_M2`, and
  `DISK_LOADING_NM2`.
- Acoustic limit case is a **deferred** Phase 5 noise study; in Phase 3 we
  only need to confirm the actuator-disk thrust setting respects the limit.

**Pass/fail signature**

- Reported $M_\text{tip}$ < 0.55 at the cruise design point and at all
  Study-1 matrix points.

---

## Technical Nugget #3 — Slope-to-Sky Transition

**Claim.** The vehicle is sized for a **ramp takeoff** (rolling start
uphill, rotation initiated by tail-lift not by elevator) and a **whip-stall
landing** (short flare with deliberate tail-stall recovery into wing-borne
descent). Both maneuvers require **low-speed pitch authority well in
excess of CS-23 minima** — that is the *engineering reason* the gap exists.

**Physics to resolve**

- Pitch authority $\Delta C_m / \Delta\theta_\text{ht}$ at $1.2\,V_s$ with the
  inboard props at static $T$.
- Neutral-point shift between Baseline and Proposed configurations.
- Tail effectiveness $\eta_t = q_\text{tail}/q_\infty$ — expected > 1 in
  the upwash column.

**Where defended**

- Cruise sensitivities already pin $dC_{m_y}/d\theta_\text{ht}$ and the
  static margin (see `post/SENSITIVITIES.md` — `dC_{m_y}/d\theta_\text{ht}
  = −0.0480$ /deg, static margin 35.8 % MAC at cruise).
- Phase 4 script `post/extract_stability.py` will compute the NP shift
  across the Study-1 matrix from a finite-difference $C_m(\alpha)$ sweep
  at fixed throttle, once the matrix has run.
- Figure: "$\eta_t$ map on the H-tail surface", C1 vs C4.

**Pass/fail signature**

- $\Delta\text{NP}_\text{aft} \ge 0.05\,\text{MAC}$ (C4 vs C1) at
  $T/W = 0.5$, $\alpha = 10°$.
- $\eta_t \ge 1.2$ over the gap-aligned strip of the H-tail.

---

## Open Questions / Risks

1. **Actuator-disk fidelity.** Does Flow360's AD model carry enough swirl
   to reproduce the upwash twist? If not, fall back to BET-disk. The
   cruise sweep already shows ~95 % of commanded thrust is delivered
   (`post/SENSITIVITIES.md`), so the AD calibration is acceptable;
   the open question is swirl-induced tail upwash.
2. **Cruise drag is 2× the design assumption.** `CFx` at cruise is 0.077
   vs the `CD = 0.04` assumed in `params.T_CRUISE_PER_PROP_N`. The trim
   solve in `post/TAKEOFF_PLAN.md` puts cruise thrust at ~119 N/prop, not
   54 N/prop. **Action**: update `T_CRUISE_PER_PROP_N` in `params.py`
   once the GAI-meshed trim campaign confirms the higher drag at finer
   resolution.
3. **Trim drag bookkeeping.** A more powerful tail means we trim with less
   download → must report $L/D$ *trimmed*, not just $C_{L_\max}$.
4. **Gap edge separation.** The outboard vane+flap's inboard edge sees a
   sharp spanwise gradient when `gap_fraction > 0`. Confirm no premature
   separation that would mask the upwash benefit.
5. **Mesh independence.** GAI campaign (`submit_gai_trim_campaign.py`)
   gives the fine reference; the coarse beta-meshed sweeps are the
   sensitivity backbone. A formal grid-refinement triple on C4 is the
   SciTech defensibility appendix.

---

## Run-Order of Operations

1. Lock `params.py` (done).
2. Generate UDCs + `.dat` files: `python geometry/airfoils/build_estol_geometry.py` (done).
3. Build per-phase geometry via direct serveCSM invocation
   (`serveCSM geometry/tsangpo.csm -batch -despmtrs phase.despmtrs`,
   one per phase 0/1/2 — done; outputs in `geometry/out/{stowed,takeoff,landing}/`).
4. **Phase 3a — cruise calibration** via `submit_cruise.py` + the three
   fork sweeps (done; `post/SENSITIVITIES.md`).
5. **Phase 3b — GAI cruise trim** via `submit_gai_trim_campaign.py`
   (queued).
6. **Phase 3c — takeoff coarse campaign** via
   `submit_takeoff_coarse_campaign.py` (spec in `post/TAKEOFF_PLAN.md`).
7. **Phase 3d — Study-1 2×2 matrix** (`params.study1_matrix()`):
   four geometry rebuilds (one serveCSM run per gap_fraction × Z_tail_chords
   combination, phase 0), then four SI-units `submit_*` driver
   invocations. Driver script TBD — modeled on `submit_cruise.py`.
8. **Phase 4 — post**: cruise sensitivities (`plot_sweep_sensitivities.py`)
   feeding the trim solve; Study-1 stability table
   (`extract_stability.py`) → headline contrast (C2 vs. C4) → non-linear-
   coupling decomposition → paper section drafts.
