# v2 GAPPED-FLAP trim results

Sister doc of `post/v2_continuous/TRIM_RESULTS.md`.  Filled 2026-10-30
from the cached sweep CSVs in `post/out/v2_gapped/` (sweeps submitted by
`flow360/submit_gapped_sweeps.py`); the numbers below are what the three
phase plotters print.  The (α, T_mult) equilibrium grid built on these
slopes is `post/out/v2_gapped/equilibrium_grid.md`.

Reproduce:

    python3 post/v2_gapped/plot_cruise_sensitivities.py  --refresh
    python3 post/v2_gapped/plot_takeoff_sensitivities.py --refresh
    python3 post/v2_gapped/plot_landing_sensitivities.py --refresh

## BO-point snapshots (parents only — pre-sweep)

| metric (at BO baseline)  | continuous v2 | **gap40**   |
|--------------------------|---------------|-------------|
| **cruise CL**            | +0.870        | +0.862      |
| cruise CMy_CG            | +0.159        | +0.151      |
| cruise CFx               | +0.083        | +0.072      |
| **takeoff CL**           | +8.005        | +4.136      |
| takeoff CMy_CG           | +0.797        | +0.233      |
| takeoff CFx              | +3.061        | +1.521      |
| **landing CL**           | +13.644       | +5.96       |
| landing CMy_CG           | +0.598        | +1.83       |
| landing htail CL         | ≈ 0 (stalled) | **−0.59 (working)** |

Stand-out observations from the parents:
  * Cruise — gap40 ≈ continuous (flap is stowed; gap is in flap-only).
  * Takeoff — gap40 cuts main+flap nose-up CMy by ~71 % at the same point.
  * Landing — htail is no longer stalled; bigger CMy_CG is the elevator
    finally producing real downforce that needs less negative trim θ_ht.

## Cruise gap40 (V=45.72 m/s, level)

Linear-regime mask: α ≤ +11°; htail and thrust full range.

Sensitivities about BO baseline (α=+7°, θ_ht=0°, T_mult=+1):
    dCL/dα   = +0.0972 /deg     dCMy/dα   = -0.0473 /deg     dCD/dα   = +0.0049 /deg
    dCL/dθh  = +0.0249 /deg     dCMy/dθh  = -0.0897 /deg     dCD/dθh  = +0.0021 /deg
    dCL/dT   = +0.0131 /unit    dCMy/dT   = +0.0096 /unit    dCD/dT   = +0.0060 /unit
    dCT_del/dT = +0.0430 /unit

Baseline at BO point:  CL=+0.862,  CMy_CG=+0.151,  CFx=+0.072,  CT_del=+0.043

**3×3 trim solution:  α = +6.17°,  θ_htail = +2.19°,  T_mult = +1.69**

Static margin (−dCMy_CG/dα / dCL/dα) = **+48.6 % MAC**   (stable; continuous: +47.1 %)

## Takeoff gap40 (phase-1 flap, V=18 m/s, γ=+30°)

Sensitivities about BO baseline (α=+8°, θ_ht=−5°, T_mult=+16):
    dCL/dα   = +0.2320 /deg     dCMy/dα   = -0.1696 /deg     dCD/dα   = +0.0722 /deg
    dCL/dθh  = +0.0689 /deg     dCMy/dθh  = -0.2351 /deg     dCD/dθh  = +0.0032 /deg
    dCL/dT   = +0.0741 /unit    dCMy/dT   = +0.0289 /unit    dCD/dT   = +0.0572 /unit
    dCT_del/dT = +0.2777 /unit

Baseline at BO point:  CL=+4.136,  CMy_CG=+0.071,  CFx=+1.521,  CT_del=+4.444

**3×3 trim solution:  α = +8.06°,  θ_htail = −4.80°,  T_mult = +15.56**

Static margin = **+73.1 % MAC**   (stable; continuous: −12.5 %).  The trim
lands within 0.1° / 0.2° / 3 % of the BO point, so unlike the continuous
case the linearization is trustworthy.

## Landing gap40 (phase-2 flap, V=12.86 m/s, γ=−30°)

Sensitivities about BO baseline (α=+8°, θ_ht=−6°, T_mult=+12):
    dCL/dα   = +0.1185 /deg     dCMy/dα   = -0.1085 /deg     dCD/dα   = +0.1215 /deg
    dCL/dθh  = +0.0715 /deg     dCMy/dθh  = -0.2460 /deg     dCD/dθh  = +0.0108 /deg
    dCL/dT   = +0.1684 /unit    dCMy/dT   = +0.0929 /unit    dCD/dT   = +0.1819 /unit
    dCT_del/dT = +0.5439 /unit

Baseline at BO point:  CL=+5.953,  CMy_CG=+1.591,  CFx=+3.596,  CT_del=+6.527

**3×3 trim solution:  α = +17.94°,  θ_htail = −1.75°,  T_mult = +17.72**

Static margin = **+91.6 % MAC**   (stable; continuous: −28.7 %).  The α
shift from the BO point (+8° → +17.9°) is large, so treat the trim
values as a first iterate; the sign of the static margin is not in doubt
(dCMy/dα = −0.108 /deg against dCL/dα = +0.118 /deg).

## Three-phase comparison

| phase    | SM continuous | **SM gap40**  | α gap40  | θ_htail gap40 | T_mult gap40 |
|----------|---------------|---------------|----------|---------------|--------------|
| cruise   | +47.1 %       | **+48.6 %**   | +6.17°   | +2.19°        | +1.69        |
| takeoff  | −12.5 %       | **+73.1 %**   | +8.06°   | −4.80°        | +15.56       |
| landing  | −28.7 %       | **+91.6 %**   | +17.94°  | −1.75°        | +17.72       |

The flap gap turns the flapped phases from statically unstable to
strongly stable at essentially the same cruise margin, and the htail
elevator slope roughly doubles to triples (−0.235 and −0.246 /deg vs
−0.090 and −0.086 /deg) because the htail is out of the stalled downwash
blanket.  This is the headline of `paper/extended_abstract.tex`
(Steps 2–3) and the reason v3 kept the gapped planform.
