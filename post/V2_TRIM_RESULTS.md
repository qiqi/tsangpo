# v2 trim results — 3×3 equilibrium solve

`(α, θ_htail, T_mult)` linear-trim solution from each phase's 9-slope
Jacobian (`dCL/d•`, `dCMy_CG/d•`, `d(net axial)/d•`).  All slopes are
fit to the UNSTALLED-regime points only (per-sweep masks listed
below).  CMy convention is `CMy_CG = CMy − 0.1·CT_delivered` —
moment_center is at the CG in v2 geometry, so the v1 `+0.4·CFx` shift
is dropped.

Reproduce:
    python3 post/plot_cruise_v2_sensitivities.py
    python3 post/plot_takeoff_v2_sensitivities.py
    python3 post/plot_landing_v2_sensitivities.py

Source data: `post/out/{cruise,takeoff,landing}_v2_sweep_data.csv`.

## Cruise v2 (V=45.72 m/s, level, ρ=cruise)

Linear-regime masks:  α ≤ +9°  ;  htail full range −12 … +15°  ;  thrust full range 0…3

Sensitivities about BO baseline (α=+7°, θ_ht=0°, T_mult=+1):
    dCL/dα   = +0.0948 /deg     dCMy/dα   = -0.0447 /deg     dCD/dα   = +0.0038 /deg
    dCL/dθh  = +0.0249 /deg     dCMy/dθh  = -0.0897 /deg     dCD/dθh  = +0.0020 /deg
    dCL/dT   = +0.0135 /unit    dCMy/dT   = +0.0090 /unit    dCD/dT   = +0.0067 /unit
    dCT_del/dT = +0.0431 /unit

Baseline at BO point:  CL=+0.870,  CMy_CG=+0.159,  CFx=+0.083,  CT_del=+0.043

**3×3 trim solution:  α = +6.03°,  θ_htail = +2.35°,  T_mult = +2.01**

Static margin (−dCMy_CG/dα / dCL/dα) = **+47.1 % MAC**   (stable)

## Takeoff v2 (phase-1 flap, V=18 m/s, γ=+30°)

Linear-regime masks:  α ≤ +11°  ;  htail θ_ht ∈ [0°, +25°] (wing+flap downwash
puts htail into stall for θ_ht ≤ −5°; over-the-top stall past +25°)  ;
thrust T_mult ≤ 22 (CMy slope reverses past +22).

Sensitivities about BO baseline (α=+8°, θ_ht=−5°, T_mult=+16):
    dCL/dα   = +0.2004 /deg     dCMy/dα   = +0.0250 /deg     dCD/dα   = +0.1492 /deg
    dCL/dθh  = +0.0268 /deg     dCMy/dθh  = -0.0899 /deg     dCD/dθh  = +0.0050 /deg
    dCL/dT   = +0.2305 /unit    dCMy/dT   = -0.0473 /unit    dCD/dT   = +0.1428 /unit
    dCT_del/dT = +0.2777 /unit

Baseline at BO point:  CL=+8.005,  CMy_CG=+0.797,  CFx=+3.061,  CT_del=+4.444

**3×3 trim solution:  α = +3.31°,  θ_htail = +10.19°,  T_mult = +1.51**

Static margin (−dCMy_CG/dα / dCL/dα) = **−12.5 % MAC**   (unstable)

Caveat: T_mult shifting from BO=16 to refined=1.5 (≈11× drop) means the
linearization point is far from the trim solution — needs a nonlinear
2-step refine (re-linearize at the refined point, solve again) before
acting on the numbers.  The qualitative finding (large pitch-up reserve
via positive elevator + greatly reduced thrust at moderate α) is real;
the exact magnitudes are linear-extrapolation artifacts.

## Landing v2 (phase-2 flap, V=12.86 m/s, γ=−30°)

Linear-regime masks:  α ≤ +8° (CL peaks at +11°)  ;  htail θ_ht ∈ [+10°, +40°]
(massive wing+flap downwash blankets htail for θ_ht ≤ 0°; over-the-top
stall past +40°)  ;  thrust T_mult ≤ 25 (CMy slope flattens past +25).

Sensitivities about BO baseline (α=+8°, θ_ht=−6°, T_mult=+12):
    dCL/dα   = +0.1480 /deg     dCMy/dα   = +0.0425 /deg     dCD/dα   = +0.2754 /deg
    dCL/dθh  = +0.0249 /deg     dCMy/dθh  = -0.0858 /deg     dCD/dθh  = +0.0138 /deg
    dCL/dT   = +0.5511 /unit    dCMy/dT   = -0.1147 /unit    dCD/dT   = +0.4416 /unit
    dCT_del/dT = +0.5442 /unit

Baseline at BO point:  CL=+13.644,  CMy_CG=+0.598,  CFx=+7.833,  CT_del=+6.530

**3×3 trim solution:  α = +10.20°,  θ_htail = +16.97°,  T_mult = +0.85**

Static margin (−dCMy_CG/dα / dCL/dα) = **−28.7 % MAC**   (very unstable)

Same large-shift caveat as takeoff (T_mult from 12 → 0.85, factor ~14).
Trim Jacobian condition number ≈ 15 (non-singular — v1 landing was
near-singular because the htail had ~zero elevator authority; the bigger
v2 htail at the longer arm restored authority once you push past
θ_ht = +10° to get out of the downwash blanket).

## Three-phase comparison

| phase    | static margin   | α       | θ_htail | T_mult |
|----------|-----------------|---------|---------|--------|
| cruise   | **+47.1 %**     | +6.03°  | +2.35°  | +2.01  |
| takeoff  | **−12.5 %**     | +3.31°  | +10.19° | +1.51  |
| landing  | **−28.7 %**     | +10.20° | +16.97° | +0.85  |

v2 cruise is comfortably stable.  v2 takeoff and landing are
aerodynamically unstable (the wing+flap forward-shift outpaces even the
bigger htail's restoring moment in those configurations) — the airframe
would require active pitch stabilization, or the gapped-flap config
(`tsangpo_gapped.csm`, cases launching) may shift the balance back.

The linear T_mult shifts in takeoff and landing are unphysically large
because the BO baseline thrust setting (T_mult=16 / 12) far over-thrusts
the actually-required level/climb thrust at those slow flapped flight
speeds; a 2-step nonlinear refine would tighten the numbers but won't
change the static-margin verdict.

## CG-shift study — aft CG to bring cruise SM down to 10 % MAC

Question: what happens to all three phase trims if we move the CG AFT
(only Δx_CG > 0, no z change) until cruise static margin drops to a
"normal" 10 % MAC?

Transfer to the new CG (CMy_CG is about the new moment_center;
parallel-axis with CFz ≈ CL valid for the small body-rotations here):

    CMy_CG_new  = CMy_CG_old  + (Δx/c) · CFz_at_origin
    dCMy_new/d• = dCMy_old/d• + (Δx/c) · dCL/d•

So SM_new = SM_old − Δx/c.  Target SM_cruise = +0.10 with SM_cruise_old
= +0.4715 requires:

**Δx/c = +0.3715**, i.e., move CG aft by 0.514 m ≈ 37 % MAC.

Reproduce: `python3 post/cg_shift_trim.py`.

Re-solving the 3×3 with the same Δx/c applied to all three phases:

| phase    | SM old → new        | α trim   | θ_htail trim | T_mult trim |
|----------|----------------------|----------|--------------|-------------|
| cruise   | +47.1 % → **+10.0 %** | +4.92°   | +6.40°       | +1.96       |
| takeoff  | −12.5 % → **−49.6 %** | +2.81°   | +27.99°      | **−0.12**   |
| landing  | −28.7 % → **−65.9 %** | +9.63°   | +55.16°      | **−0.73**   |

Cruise: SM lands at the target +10 % and the trim deflections are
reasonable (α down 1.1°, more positive elevator to balance the larger
nose-up CMy from CG aft, T_mult barely changes).

**Takeoff and landing become massively more unstable.**  The reason is
mechanical: the aft CG shift's effect on dCMy/dα is (Δx/c)·dCL/dα.  In
cruise that's 0.371 × 0.095 = +0.035 — small enough to tip a +47 % SM
down to +10 %.  In takeoff the wing+flap is making CL ≈ 8 and
dCL/dα = 0.20, so the same shift adds 0.371 × 0.20 = +0.074 to dCMy/dα
— roughly doubling the destabilizing aerodynamic term.  Landing is
worse still: dCL/dα = 0.148 and CL_base = 13.6, so the BASELINE
CMy_CG balloons from +0.60 to +5.67, demanding huge htail deflection
to zero it out.

The negative T_mult values (−0.12 in takeoff, −0.73 in landing) and
55° elevator on landing are physically infeasible — the linearized
trim is just telling us that the trim point doesn't exist at all in
the realistic flight envelope.  In practice an aft CG that buys
cruise neutrality at the cost of catastrophic flapped-flight stability
is not a viable design — the wing+flap CMy.CG-vs-CG sensitivity in
those configurations is dominated by the high CL × dx-shift product.

Implication for the design: the gapped-flap configuration is the more
promising lever, since it reduces flap chord-wise loading inboard and
should weaken the wing+flap forward-CP shift that creates the high
CMy.CG slopes in takeoff and landing.  Numbers TBD from the launched
gap40 parents (`prj-59c27343`, `prj-e0e11ed5`, `prj-0cd29981`).
