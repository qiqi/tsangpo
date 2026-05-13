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

### Phase C — GAI takeoff sweep (refined trim)

Pending Phase A completion + sensitivity refinement.
