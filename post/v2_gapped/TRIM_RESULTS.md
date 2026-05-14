# v2 GAPPED-FLAP trim results

Sister doc of `post/v2_continuous/TRIM_RESULTS.md`.  Once the gap40
sweeps land (submitted by `flow360/submit_gapped_sweeps.py`), run the
three phase plotters and paste the (sensitivities + 3×3 trim) output
into the sections below for direct comparison with the continuous-flap
table.

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

## Cruise gap40 (sweeps pending — fill once forks land)

    TBD — paste plot_cruise_sensitivities.py output here.
    Expected: SM similar to continuous (~+47% MAC) since cruise is
    nearly flap-independent.

## Takeoff gap40 (sweeps pending)

    TBD.  Expected static margin to be much less negative than continuous
    (−12.5%), because dCMy/dα should be smaller after the wing+flap
    forward CP shift is reduced.

## Landing gap40 (sweeps pending)

    TBD.  Expected static margin to be much less negative than continuous
    (−28.7%), and the elevator slope dCMy/dθ_h to be even larger than the
    masked continuous value (−0.086 /deg).
