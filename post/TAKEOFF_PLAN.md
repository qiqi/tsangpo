# Takeoff back-of-envelope and campaign plan

## Cruise findings (one-paragraph recap)

Three constant-condition fork sweeps on the blunt-htail geometry
(`prj-3e8b1ed8`, `tsangpo_blunt_htail_active`, see
`post/SENSITIVITIES.md` for full case-id thread) pin the cruise
sensitivities at α=+7°, θ_ht=0°, T_mult=1.0× design point:

| | value |
|---|---:|
| dCL/dα | +0.0886 /deg (= +5.08 /rad, matches AR=8 lifting-line) |
| dCMy/dα | −0.0318 /deg ⇒ static margin = **35.8 % MAC** |
| dCMy/dθ_ht | −0.0480 /deg = −2.75 /rad |
| dCL/dθ_ht | +0.0146 /deg |
| dCFx/dT_mult | +0.0054 /unit (slipstream raises wall drag) |
| dCT_delivered/dT_mult | +0.0382 /unit (~95.5 % of commanded) |

Solving the 3×3 linear system for `CL=W/qS`, `CMy_CG=0`,
`CFx_wall=CT_delivered` (moment about CG at z=−0.4c, thrust contribution
included) gave a cruise trim of:
**α=+6.78°, θ_ht=−0.54°, T_mult=2.13** (drag came out ≈ 2× what
`params.py`'s CD=0.04 assumption produced; required cruise thrust is
~119 N/prop, not 54 N/prop).

A GAI-meshed campaign at this trim is queued (parent
`case-a8634829-…` on `prj-3e8b1ed8`); when it lands, the same plot
script (`post/plot_sweep_sensitivities.py`) will refine the sensitivities.

## Takeoff target

| | value |
|---|---:|
| airspeed | 35 kt = **18.0 m/s** |
| altitude | 3,658 m (same as cruise — high-altitude STOL) |
| ρ_∞ | 0.8491 kg/m³ |
| q_∞ | 0.5·ρ·V² = **137.6 Pa** |
| q·S | 137.6 × 15.33 = **2,109 N** |
| climb-out γ | 30° |
| weight W | 11,565 N |
| flap setting | phase 1 (kinematics from `estol_config.yaml`: dx=+0.280c, dy=+0.045c, rotate −40° about pivot) |

## Force balance in steady climb at γ = 30°

In the flight-path frame, with thrust acting along the body forward
axis (at angle α relative to the velocity vector):

```
Z_flight: L + T·sin(α) − W·cos(γ) = 0
X_flight: T·cos(α) − D − W·sin(γ) = 0
Pitch:    CMy_CG = 0
```

Numbers:
- `W·cos(γ) = 11,565 × cos(30°) = 10,015 N`
- `W·sin(γ) = 11,565 × sin(30°) =  5,782 N`

So:
- Lift needed (without the thrust assist): **L = 10,015 N ⇒ CL ≈ 4.75** (with T·sin(α) help, CL_aero ≈ 4.2 at α≈+8°).
- Thrust = (D + 5,782 N) / cos(α).

Drag estimate (induced + parasite + flap):
- CDi = CL² / (π·AR·e) ≈ (4.5)² / (π·8·0.7) ≈ **1.15**
- CD_parasite + flap ≈ 0.10
- **CD ≈ 1.25** ⇒ D ≈ CD · q·S ≈ 2,640 N.

So required total thrust **T ≈ (2,640 + 5,782) / cos(8°) ≈ 8,510 N**, i.e.
**~850 N/prop**. Compared to `T_CRUISE_PER_PROP_N = 54.4 N/prop`,
this is a multiplier of:

> **T_mult ≈ 8510 / (10 × 54.4) ≈ 15.6**

## Why this CL is achievable: slipstream augmentation

Bare-wing CL_max with takeoff flap (−40°) is probably ~2.5. To hit
CL=4.5 we need the props to roughly double the effective dynamic
pressure over the wing.

Static-thrust induced velocity per disk at T_prop = 850 N:
`v_i = √(T/(2ρA)) = √(850 / (2·0.85·0.894)) ≈ 23.6 m/s`.

Slipstream velocity at the disk:
`V + v_i ≈ 18 + 24 = 42 m/s ⇒ q_slipstream ≈ 750 Pa`,
or **~5.4× freestream q**.

The blown portion of the wing therefore operates at much higher
effective q, and CL referenced to freestream q can exceed the bare
airfoil's intrinsic CL_max. The CFD will resolve the actual
partition.

## Pitch trim (very rough — cruise sensitivities don't transfer)

Flap deflection adds a large nose-down moment from the wing. To
counter it the htail must contribute *more* nose-up moment. With
`dCMy/dθ_ht < 0` (i.e., positive θ_ht produces *negative* CMy via
the inverted-camber tail), we want **θ_ht negative** (LE-down,
makes the inverted htail see a more negative effective α ⇒ more
downforce ⇒ tail-down ⇒ nose-up).

Rough cruise scaling says a flap nose-down contribution of order
−0.2 would require **Δθ_ht ≈ −5°**, but the slipstream over the
htail will change dCMy/dθ_ht itself — that's exactly what the
coarse sweep will measure.

## Initial guess for the takeoff trim

| | guess |
|---|---:|
| α | **+8°** |
| θ_ht | **−5°** |
| T_mult | **+16** (~850 N/prop) |

## Plan

1. **Phase A — coarse takeoff campaign** (no GAI; regular beta mesher,
   takeoff-flap geometry uploaded to a fresh project): parent at the
   back-of-envelope estimate above, then three 10-fork sweeps
   centred on it:
   - α ∈ {−2, +2, +5, +8, +11, +14, +17, +20, +25, +30}° (wide range to capture stall under blown lift)
   - θ_ht ∈ {−15, −12, −9, −6, −3, 0, +3, +6, +9, +12}° (centred near −5°, but generous)
   - T_mult ∈ {6, 9, 12, 14, 16, 18, 20, 22, 25, 30} (centred near 16)
2. **Phase B — refine.** Pull sensitivities from the coarse sweep,
   solve the 3×3 trim system, get a sharper (α, θ_ht, T_mult) estimate.
3. **Phase C — GAI takeoff campaign** at the refined trim, same
   three-sweep structure but on the finer GAI mesh
   (`geometry_accuracy = 0.831 mm`, `preserve_thin_geometry=True`).

See `flow360/submit_takeoff_coarse_campaign.py` for the Phase A
launcher; case-id manifest will be appended below as each phase
completes.

## Case-id manifest

### Phase A — coarse takeoff sweep (submitted 2026-05-13)

- **Project**: `prj-25133de4-f045-4901-895a-5d3f241dc675` (`tsangpo_takeoff_coarse`)
- **Parent (BO estimate, α=+8°, θ_ht=−5°, T_mult=+16)**: `case-95963eff-6cbe-4605-be95-4f36aaeb6345`

| sweep | values | case IDs |
|---|---|---|
| α [°] | −2, +2, +5, +8, +11, +14, +17, +20, +25, +30 | `ad72eaa5`, `2437b407`, `844f2587`, `ce8cb292`★, `05dfe478`, `d2bfe85f`, `4aa30ad5`, `c65ec5b0`, `9e4c0cc2`, `f1954357` |
| θ_ht [°] | −12, −9, −6, −4, −2, 0, +2, +4, +6, +9 | `bdcfa85a`, `8fbf8263`, `cef967d5`, `20a2c443`, `127f52b5`, `41304844`, `3a1d8fad`, `fb4d6486`, `fa364f8d`, `778c11c6` |
| T_mult | 6, 9, 12, 14, 16, 18, 20, 22, 25, 30 | `375d0b9d`, `55101927`, `f4c2197e`, `330c5fd1`, `ce8cb292`★, `843e3a30`, `5ab3641c`, `26ecca60`, `fd572e56`, `1dea911c` |

★ Dedup: the α=+8° fork and the T_mult=16 fork have identical params (both at the BO baseline), so Flow360 returned the same case ID.

### Phase A — coarse takeoff sweep, **v2 forks** (tight iter settings)

Initial v1 forks (1000 pseudo iters × 10 new steps) were cancelled after
inspecting the parent's convergence — forces stabilised by physical
step 5 and within-step CL is flat to 0.008 % across 1000 pseudo iters.
Resubmitted with **N_FORK_NEW=6, PSEUDO_FORK=500** (≈70 % compute
saving per fork), forked from the now-completed parent
`case-95963eff` (parent solver wall time 63 min, realFlexUnit 23.5).

| sweep | values | case IDs |
|---|---|---|
| α [°] | −2, +2, +5, +8, +11, +14, +17, +20, +25, +30 | `611dc481`, `e2740270`, `af473707`, `f50417a9`★, `56a02a76`, `86973474`, `a44322f9`, `06ff58fc`, `07d360f5`, `5e7b3963` |
| θ_ht [°] | −12, −9, −6, −4, −2, 0, +2, +4, +6, +9 | `db98b4d4`, `1c8aa00a`, `8cf4a3ff`, `e390dc48`, `af6eac32`, `3da44e22`, `c7c3ab88`, `e1fe6661`, `3901f098`, `b4ea25cc` |
| T_mult | 6, 9, 12, 14, 16, 18, 20, 22, 25, 30 | `96ca3665`, `623c3792`, `c40be1d2`, `f4fc5718`, `f50417a9`★, `d5a429d0`, `1a8b9776`, `bd92aedd`, `44923768`, `134b5899` |

★ Dedup: α=+8° and T_mult=16 share `case-f50417a9` (identical
SimulationParams — both at the BO baseline).

### Phase B — refined trim solve from Phase A v2 sensitivities

Sensitivities about the BO baseline (α=+8°, θ_ht=−5°, T_mult=+16),
using 20/29 of the v2 forks (alpha sweep complete, half of htail and
half of thrust):

| | value |
|---|---:|
| dCL/dα | +0.203 /deg (pre-stall fit, α ≤ +11°) |
| dCL/dθ_ht | +0.008 /deg (very weak) |
| dCL/dT_mult | +0.203 /unit (huge — blown-lift dominates) |
| dCMy/dα | +0.017 /deg |
| **dCMy/dθ_ht** | **−0.022 /deg** ← half of cruise's −0.048, slipstream wake diminishes htail authority |
| dCMy/dT_mult | −0.098 /unit |
| dCD/dα | +0.145 /deg |
| dCD/dT_mult | +0.114 /unit |
| dCT_delivered/dT_mult | +0.246 /unit (matches Mach-based AD scaling) |

Baseline at BO: CL=+7.82, CMy_CG=−1.71, CFx=+2.84, CT_delivered=+3.93
(referenced to qS=2109 N). Solving the linearised 3×3 system for
`CL = (W·cos γ − T·sin α)/qS`, `CMy_CG = 0`, `CT_del·cos α − CFx =
W·sin γ /qS`:

> Δα = **−4.71°**, Δθ_ht = **−27.88°**, ΔT_mult = **−12.02**

i.e., raw refined trim α=+3.3°, θ_ht=−32.9°, T_mult=+4.0.

**Trim feasibility caveat.** The α and T_mult numbers are credible
(BO over-lifts ~1.9× → both knobs need to come way down). But the
Δθ_ht is unphysical — well outside the sweep range (−12 to +9°) and
beyond any reasonable elevator deflection. The htail in heavy
slipstream wake from the wing+flap has too little authority to
balance the wing/flap nose-down moment (CMy_CG_base = −1.71 vs
dCMy/dθ_ht ≈ −0.022 /deg ⇒ need ≈ −78° of htail input — clearly
not achievable). This is a real finding: **the current low-htail
phase-1-flap geometry can't pitch-trim aerodynamically in a
30°-climb takeoff at the assumed CG (z = −0.4c, x = 0 = wing-root
c/4)**. Likely fixes:

1. Move the CG aft by ≈ 0.2 c (puts wing CL arm closer to neutral
   point — would shift CMy_CG by +0.2 × CL ≈ +1.6, enough to trim).
2. High-htail (Electra config #2) — htail out of the wing/flap wake
   should restore much of the dCMy/dθ_ht authority.
3. Reduce flap deflection from −40° (phase 1) to something milder.

For Phase C we proceed with a **damped, envelope-clipped trim**:

| | Phase C target |
|---|---:|
| α | +5° (between BO=+8° and linear solve=+3.3°, just past CL_max stall onset) |
| θ_ht | −12° (lower limit of sweep envelope; maximum nose-up htail authority) |
| T_mult | +8 (between BO=16 and linear solve=4, clipped to keep blown lift) |

This isn't a *trimmed* point — CMy_CG will still be negative — but
it brings CL into the target zone (~4.5) and lets us cross-check
the sensitivities at a much better operating point than the BO.

### Phase C — refined sweep (legacy mesher, tightened mesh)

Submitted parent at the damped trim above with tightened mesh
settings (legacy beta mesher; GAI still blocked per
`post/FLOW360_GAI_BUG_REPORT.md`):

| | Phase A | Phase C |
|---|---:|---:|
| surface_max_edge_length | 0.075 m | **0.04 m** (≈ 2× finer surface) |
| curvature_resolution_angle | 15° | **10°** |
| first-layer thickness | 7.62 × 10⁻⁶ m | unchanged |

- Project: `prj-25133de4-…` (same as Phase A)
- **Phase C parent**: `case-16cebe64-db52-4b86-a470-3b2e530b428b`
  (α=+5°, θ_ht=−12°, T_mult=+8)

Fork sweeps around this point are a follow-on if the Phase C parent
delivers an answer in the right ballpark (CL near target, residual
CMy small enough to be a sensible operating point).

### Phase C parent — result

`case-16cebe64` completed in ~25 min (mesh + solve, all 20 physical
steps).  At (α=+5°, θ_ht=−12°, T_mult=+8) on the **tightened mesh**:

| | Phase A linear extrap | Phase C actual | error |
|---|---:|---:|---:|
| CL | +5.53 | **+5.95** | +7.5% |
| CFx (= CD) | +1.50 | **+1.64** | +9% |
| CT_delivered | +1.97 | **+1.99** | +1% |
| CMy_CG (with thrust) | −0.82 | **−1.96** | **+138%** |

So:
- CL is still 25% over target (~4.5–4.8 needed; we have 5.95) →
  α and/or T_mult need to come down further.
- Net axial (T_delivered − D) ≈ 4,206 − 3,462 = **+744 N**, target
  W·sin(30°) = +5,782 N — still ~5 kN of *missing thrust* to hold
  the climb.  This pulls T_mult back up, not down.
- **CMy is dramatically more negative than the linear extrap
  predicted** — the wing+flap nose-down moment grows nonlinearly
  with reduced lift, and the htail (in slipstream wake) gives no
  authority to fix it.  The 138 % CMy prediction error is the
  clearest signal that the linear trim is unreliable for any
  large step in this regime.

So Phase C at this point is **CL-too-high, thrust-too-low,
moment-uncorrected**.  The takeoff condition simply doesn't trim
in this geometry — same conclusion as landing, with the same
underlying physics.  See `post/LANDING_PLAN.md` "Design
conclusion" section.
