# Per-surface aerodynamic breakdown at BO trim points

Comparing cont-low (Step 0; low-htail + continuous flap) vs
cont-high (Step 1; T-tail + continuous flap) at the *same* BO
trim point, surface-by-surface.  CL, CD, CMy are integrated
coefficients on each surface (wind axes, referenced to
$S_w$, $c_w$, CG).

## cruise (BO α / θ_ht / T_mult per phase plotter)

| surface | cont-low CL | cont-high CL | ΔCL | cont-low CD | cont-high CD | ΔCD | cont-low CMy | cont-high CMy | ΔCMy |
|---|---|---|---|---|---|---|---|---|---|
| main_wing | +0.929 | +0.927 | -0.002 | +0.030 | +0.030 | +0.000 | +0.116 | +0.115 | -0.000 |
| htail     | +0.000 | +0.020 | +0.019 | +0.007 | +0.008 | +0.000 | +0.029 | -0.037 | -0.066 |
| TOTAL     | +0.930 | +0.947 | +0.017 | +0.038 | +0.038 | +0.001 | +0.144 | +0.078 | -0.066 |

## takeoff (BO α / θ_ht / T_mult per phase plotter)

| surface | cont-low CL | cont-high CL | ΔCL | cont-low CD | cont-high CD | ΔCD | cont-low CMy | cont-high CMy | ΔCMy |
|---|---|---|---|---|---|---|---|---|---|
| main_wing | +5.082 | +4.650 | -0.432 | +0.646 | +0.500 | -0.146 | +0.501 | +0.471 | -0.031 |
| htail     | -0.517 | -0.418 | +0.098 | -0.120 | -0.070 | +0.050 | +1.995 | +1.597 | -0.398 |
| TOTAL     | +4.565 | +4.232 | -0.333 | +0.526 | +0.430 | -0.096 | +2.496 | +2.068 | -0.428 |

## landing (BO α / θ_ht / T_mult per phase plotter)

| surface | cont-low CL | cont-high CL | ΔCL | cont-low CD | cont-high CD | ΔCD | cont-low CMy | cont-high CMy | ΔCMy |
|---|---|---|---|---|---|---|---|---|---|
| main_wing | +8.542 | +7.554 | -0.988 | +1.000 | +0.775 | -0.225 | +0.751 | +0.695 | -0.056 |
| htail     | -0.476 | -0.516 | -0.040 | -0.099 | -0.109 | -0.010 | +1.860 | +1.942 | +0.082 |
| TOTAL     | +8.066 | +7.038 | -1.029 | +0.900 | +0.666 | -0.235 | +2.611 | +2.637 | +0.026 |
