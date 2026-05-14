# Landing back-of-envelope and campaign plan

Companion to `post/TAKEOFF_PLAN.md` and `post/SENSITIVITIES.md`.

## Target

| | value |
|---|---:|
| airspeed | 25 kt = **12.86 m/s** |
| altitude | 3,658 m (same as cruise / takeoff) |
| ρ_∞ | 0.8491 kg/m³ |
| q_∞ | 0.5·ρ·V² = **70.2 Pa** |
| q·S | 70.2 × 15.33 = **1,076 N** |
| flight-path γ | **−30°** (descending) |
| weight W | 11,565 N |
| flap setting | **phase 2** (kinematics from `estol_config.yaml`: dx=+0.300 c, dy=+0.050 c, rotate −65° about pivot — much steeper deflection than takeoff's −40°) |

## Force balance (steady descent at γ = −30°)

Same equations as takeoff but with γ negative, so the weight component
*along* the flight path now *helps* forward motion:

```
Z_flight: L + T·sin(α) − W·cos(γ) = 0
X_flight: T·cos(α) − D − W·sin(γ) = 0      (W·sin(γ) < 0 ⇒ gravity assists)
Pitch:    CMy_CG = 0
```

Numbers:
- `W·cos(γ) = 11,565 · cos(30°) = 10,015 N` — same as takeoff (|γ| same)
- `W·sin(γ) = −5,782 N` — gravity now pulls forward, opposite sign to takeoff

Aero CL needed (without thrust assist): **CL = 10,015 / 1,076 ≈ 9.31**.
With T·sin(α) help at α≈+5–8°, CL_aero drops to ~8.5–8.7.  Either way
**roughly 2× the takeoff CL_target**, because q is half (V is 71 % of
takeoff and we square it).

Drag estimate: CDi = CL² / (π·AR·e) ≈ 9.3² / (π·8·0.7) = **4.9**, plus
~0.2 parasite/flap → **CD ≈ 5**. So D ≈ 5 × 1,076 = **5,400 N**.

Thrust:
```
T = (D + W·sin(γ)) / cos(α) = (5,400 − 5,782) / cos(α) ≈ −400 N
```

i.e., **almost idle thrust** — gravity essentially balances the drag.
At first glance this contradicts "we need high T_mult to generate
slipstream lift," because the slipstream is what makes CL=9.3 possible.

### Resolving the contradiction

To blow the wing enough to hit CL=9.3, the disks have to push a
substantial momentum flux through the slipstream. But the drag of the
blown wing scales with both the lift and the slipstream q, so D
grows roughly as fast as T_thrust. The **net axial** is
T_thrust − D ≈ −5,800 N regardless of disk loading, once the
slipstream is strong enough to support the lift.

So at landing, we expect:
- **T_mult ≈ 10–14** (just slightly less than takeoff's ~16 — the
  blown-wing CL has to be even higher than takeoff, but the speed is
  lower so the absolute thrust per disk is smaller).
- Net (T − D) settles to ≈ −5.8 kN once CL is on the manifold.

## Initial guess for the landing trim

Mirroring the takeoff back-of-envelope but accounting for the higher
required CL and the negative net axial:

| | guess |
|---|---:|
| α | **+8°** (same starting point as takeoff; the sweep will move us) |
| θ_ht | **−6°** (more negative than takeoff to counter the heavier flap nose-down moment of phase 2's −65° deflection) |
| T_mult | **+12** (blown-wing demand similar to takeoff, slightly lower per propeller because q·A_disk is lower at 12.86 m/s) |

## Sweep ranges (coarse, Phase A)

| sweep | values | rationale |
|---|---|---|
| α [°] | −2, +2, +5, +8, +11, +14, +17, +20, +25, +30 | same as takeoff; stall behaviour likely different but pre/post stall both interesting |
| θ_ht [°] | −15, −12, −9, −6, −3, 0, +3, +6, +9, +12 | shifted slightly more negative than the takeoff sweep, to bracket the guess of −6° |
| T_mult | 4, 7, 10, 12, 14, 16, 18, 21, 25, 30 | bracket 12 with finer steps in 10–14; keep upper end for stall-related cross-checks |

Time-stepping uses the same lessons we just paid for in the takeoff
campaign: parent gets full 1000 pseudo iters (one bootstrap solve from
freestream), forks get **6 new physical steps × 500 pseudo iters**
(forces settle by step 5; residual floors at ~1e-8 and won't budge).

## Plan

1. **Phase A — coarse landing campaign** (legacy beta mesher,
   `tsangpo_landing_coarse_phaseA` project, fresh phase-2 geometry
   upload).  Same structure as takeoff Phase A: parent at BO + three
   10-fork sweeps.
2. **Phase B — refine.** Same as takeoff Phase B: pull sensitivities,
   solve the 3×3 trim system, get sharper (α, θ_ht, T_mult).
3. **Phase C — refined sweep at the new trim** (still legacy mesh —
   GAI volume mesher has the nested-rotation bug documented in
   `flow360/LESSONS.md`; optionally with tightened mesh settings).

Submission deferred until the takeoff Phase A completes + Phase B
plots land in `post/out/`, per the user's pipeline order.

## Case-id manifest

### Phase A — coarse landing sweep (submitted 2026-05-13)

- **Project**: `prj-17a37c9d-fe5d-4454-88b6-b8c6c45f4721` (`tsangpo_landing_coarse_phaseA`, in `Tsangpo/1_v1_historical/`)
- **Parent (BO estimate, α=+8°, θ_ht=−6°, T_mult=+12)**: `case-9f8147fa-2872-4f0c-b41a-7aa1fe7318bb`
- Forks use tight-iter settings (`N_FORK_NEW=6`, `PSEUDO_FORK=500`) inherited from the takeoff post-mortem.

| sweep | values | case IDs |
|---|---|---|
| α [°] | −2, +2, +5, +8, +11, +14, +17, +20, +25, +30 | `b8b89b25`, `c77ec4e3`, `cc5b367c`, `41ee6b25`★, `01f7cab6`, `387f40df`, `c7f2b811`, `c14dd229`, `cd448854`, `514e40e7` |
| θ_ht [°] | −15, −12, −9, −6, −3, 0, +3, +6, +9, +12 | `84033a94`, `235c14e1`, `3b2a4c9f`, `41ee6b25`★, `b591e117`, `3b065be7`, `4b830f23`, `645343d1`, `050490df`, `49a6fe19` |
| T_mult | 4, 7, 10, 12, 14, 16, 18, 21, 25, 30 | `1d6d8c8b`, `08d16ca6`, `eb7ad909`, `41ee6b25`★, `0742f620`, `2fd3e0e9`, `8d2c2bdb`, `a64b58a1`, `96451c6d`, `60c3b34d` |

★ Dedup: `case-41ee6b25` is the BO baseline (α=+8°, θ_ht=−6°, T_mult=+12) shared by all three sweeps.

### Phase A — landing parent snapshot

The parent finished first (~30 min mesh+solve, similar to takeoff). At
the BO point (α=+8°, θ_ht=−6°, T_mult=+12):

| | value |
|---|---:|
| CL | +13.53 |
| L | 14,559 N |
| target L (= W·cos 30°) | 10,015 N |
| over-lift factor | **1.45×** (vs takeoff's 1.85×) |
| CD | +7.36 |
| D | 7,920 N |
| T_delivered ≈ | 6,230 N |
| (T_delivered − D) | **−1,690 N** |
| target (W·sin γ at γ=−30°) | **−5,782 N** |
| CMy (about origin) | −6.33 |
| CMy_CG (with thrust) | ≈ −3.97 |

So at the BO point landing is:
- Over-lifting by 1.45× → need lower α and/or T_mult
- Decelerating *less* than gravity assists → need *less* thrust still
- Big nose-down CMy_CG, same htail-authority-in-slipstream issue
  expected as in takeoff

Refined trim will move T_mult down hard (probably to ~3–5) and α
down slightly. Same caveat as takeoff Phase B: htail in heavy
slipstream wake may have too little authority to drive CMy_CG to
zero — the design study question becomes whether CG location or
high-htail (config 2) recovers it.

### Phase B — landing trim solve

Sensitivities from all 30 forks about the BO baseline (α=+8°,
θ_ht=−6°, T_mult=+12, V=12.86 m/s, γ=−30°):

| | value | note |
|---|---:|---|
| dCL/dα | +0.131 /deg | lower than takeoff (+0.20) — already past stall in heavy slipstream |
| dCL/dT_mult | +0.493 /unit | **2.4× takeoff** — blown-lift dependence is much stronger at landing q |
| dCD/dα | +0.264 /deg | rising fast — high-AoA stall drag |
| dCD/dT_mult | +0.386 /unit | strong |
| **dCMy/dθ_ht** | **−0.00015 /deg** | **near-zero** — htail completely wake-shielded at landing |
| **dCL/dθ_ht** | **+0.00042 /deg** | near-zero |
| dCMy/dα | +0.066 /deg | weak (also wake-influenced) |
| dCMy/dT_mult | −0.212 /unit | strong nose-up effect with reduced thrust |

**The 3×3 trim system is effectively singular** — htail row is
~10⁻³ × the others, the linear solver returns Δθ_ht ≈ −70,000°.
Dropping htail and solving the 2×2 system in (α, T_mult) only:

| | refined |
|---|---:|
| Δα | +2.6° → α ≈ **+10.6°** |
| ΔT_mult | −10.9 → T_mult ≈ **+1.1** (near idle) |
| Δθ_ht | indeterminate (htail authority too low to converge) |

Reality check: at T_mult ≈ 1.1, the slipstream is nearly absent
and the blown-lift mechanism that made CL=13.5 disappears.
Predicted CL ≈ 8.5 (close to but below the target 9.3). So at
near-idle thrust the wing+flap on its own might not generate
enough lift to hold the descent — the airplane would simply not
sustain γ=−30° at 25 kt. This is a real **design conclusion**: the
current geometry needs the slipstream to make CL_target at this
slow speed.

### Design conclusion (config 1: continuous flap, low htail)

Both takeoff (γ=+30°, V=18 m/s) and landing (γ=−30°, V=12.86 m/s)
trim solves on this geometry fail on **pitch trim**: the low htail
sits in the heavy wing+flap slipstream wake and has effectively
zero aerodynamic authority. The wing+flap nose-down moment is
real (CMy_CG ≈ −1.7 at takeoff BO, ≈ −4.0 at landing BO) and the
htail cannot balance it. The implications:

- **The current low-htail design is not pitch-trim feasible for
  either steep takeoff or steep landing at the assumed CG**
  (z = −0.4c, x = 0 = wing-root quarter-chord).
- **Path forward**: configurations 2 (continuous flap + high htail,
  Electra-style) and/or 4 (gapped flap + high htail) — the high
  htail sits out of the wing/flap wake and should restore the
  elevator authority needed to trim. A CG shift aft by ~0.2c
  could also help structurally, but the htail-shielding problem
  remains physical: in heavy slipstream the htail is just out of
  the loop.

Phase C launches at refined trim points on config 1 are still
useful as benchmarks against which configs 2/3/4 can be compared,
but they will not converge to trimmed cruise/climb/descend.
