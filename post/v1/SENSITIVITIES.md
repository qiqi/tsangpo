# Tsangpo cruise sensitivities

Calibration sweeps of the cruise condition (α = +7°, θ_htail = 0°,
T = 1.0× design) on the **blunt-htail** geometry. Each sweep varies a
single parameter while pinning the other two at the cruise baseline.

- **Project**: `prj-3e8b1ed8-0109-4f1f-8738-fef0c55a573b` (`tsangpo_blunt_htail_active`)
- **Parent case**: `case-9520355d-22ed-4f2a-bad0-eece1946594b` (`cruise_SI_baseline`, α=+7°, θ_ht=0°, T=cruise → CL = 0.868, L/W = 1.02)
- **30 forks** (10 per sweep). All sweep forks pin a CONSTANT
  `AngleExpression` and (where varied) a hardcoded `ForcePerArea`;
  each runs ~19 unsteady steps after the parent's 10 to settle on its
  new condition. See `flow360/submit_alpha_sweep.py`,
  `flow360/submit_htail_sweep.py`, `flow360/submit_thrust_sweep.py`.

## Moment reference convention

Flow360 reports moments about the geometric origin (wing-root
quarter-chord, z = 0). For reporting we re-reference to a CG at
**z = −0.4 c** (0.4 chords below the wing LE; 0.1 chords below the
thrust line, which sits at z = −0.3 c):

```
CMy_CG = CMy_origin + 0.4 · CFx − 0.1 · CT_delivered
            └─ aerodynamic ─┘    └─── thrust ───┘
```

- `+0.4 · CFx`: drag acts above CG ⇒ nose-up moment about CG.
- `−0.1 · CT_delivered`: forward thrust above CG ⇒ nose-down moment.
  `CT_delivered = F_AD / (q · S)`, where `F_AD = Disk*_Force × ρ · a² · L²`
  (Flow360 reports `Disk*_Force` non-dimensionally with a Mach-based
  reference, NOT `q · S`; using the wrong reference gave us a fictitious
  "14% effectiveness" earlier — the disks actually deliver ≈ 95% of
  commanded).

CFD wall-integral `CFx` excludes the AD body force, so the two terms
add cleanly.

## Plots

- `out/sensitivities.png` — 3×3 grid: rows = sweeps (α, θ_ht, T_mult),
  cols = `C_L`, `C_D`, `C_my` (about CG, with thrust). Red dashed line
  is a linear fit (over α ≤ +9° for the alpha sweep, full range for
  the other two).
- `out/thrust_balance.png` — aircraft drag, commanded thrust, AD-delivered
  thrust vs the thrust multiplier; balance at multiplier ≈ **2.18**.

## Sensitivities (linear fits)

All slopes computed by the linear regression in `plot_sweep_sensitivities.py`.

| Quantity | Slope | Notes |
|---|---:|---|
| `dCL/dα` | **+0.0886 /deg = +5.08 /rad** | Pre-stall fit, α ≤ +9°. Matches lifting-line for AR=8 (5.03 /rad). |
| `dCD/dα` | +0.0040 /deg | Pre-stall induced-drag rise. |
| `dCMy/dα` | **−0.0318 /deg** | About CG w/ thrust. Static margin = −(dCMy/dα)/(dCL/dα) = **35.8 % MAC** (very stable). |
| `dCL/dθ_ht` | +0.0146 /deg | Modest lift change with htail rotation. |
| `dCD/dθ_ht` | +0.0012 /deg | Asymmetric — htail drag rises faster on the positive side. |
| `dCMy/dθ_ht` | **−0.0480 /deg = −2.75 /rad** | Elevator effectiveness. Matches V_H · CL_α_ht prediction of −2.99 /rad. |
| `dCL/dT_mult` | +0.0089 /unit | Slipstream-augmented lift. |
| `dCD/dT_mult` | +0.0054 /unit | Wall drag rises slightly with disk loading (more energy in slipstream). |
| `dCMy/dT_mult` | +0.0016 /unit | Drag-moment shift (≈+0.4·dCD/dT) almost cancels thrust-moment shift (−0.1·dCT/dT). |

## Cruise-point implications

- **Lift / weight** at α=+7°: CL = 0.868, q·S = 13,604 N ⇒ L = 11,808 N vs W = 11,565 N ⇒ **L/W = 1.02** (within 2 % of trim).
- **Trim incidence** (drive `CMy_CG → 0` at α=+7°, T = ×1.0): `Δθ_ht = −CMy_CG / (dCMy/dθ_ht) = −(−0.0347)/(−0.0480) = −0.72°`. Htail needs ≈ −0.7° incidence (LE down by 0.7°) for trimmed cruise.
- **CL_max ≈ 1.43 at α = +15°** in the clean (stowed) configuration. Slope rolls off between +11° and +15° → stall onset around +11°–13°.
- **Cruise drag ≈ 2× the design assumption.** Aircraft `CFx` at cruise is 0.0768, vs the `CD = 0.04` assumed in `params.py`. AD-delivered thrust at multiplier × 1 is ~520 N; required for thrust = drag is ~1100 N ⇒ **operate at thrust ≈ ×2.18 of `T_CRUISE_PER_PROP_N`**. Update `T_CRUISE_PER_PROP_N` in `params.py` accordingly (current 54.4 N/prop → ~119 N/prop).

## Traceability — case IDs

### α sweep (θ_ht = 0°, T = ×1.0)

| α [°] | case ID |
|---:|---|
| −3 | `case-62103e52-7956-4c52-851f-bc8b1d055dd5` |
| −1 | `case-8e472bfc-088d-4bb3-814b-d27e06c16b37` |
| +1 | `case-ffb614ea-6fca-4c4e-9b8a-fa0a51a0c3be` |
| +3 | `case-18446df1-d5e4-4286-bccd-05a6db158d72` |
| +5 | `case-6a5e5853-8240-44b9-9209-ef9cc5d9d4c0` |
| +7 | `case-7d14b0ab-9b80-443c-894a-037b4a77aec2` |
| +9 | `case-fe9b2fbc-db6b-49f4-b1e9-c1e825c3a756` |
| +11 | `case-2e247116-c66e-407f-8605-d1e6d75d68ad` |
| +13 | `case-a6fc399f-de8d-419a-b98c-8589b3d69a50` |
| +15 | `case-0376a381-07bb-4b52-8497-ccc2130f1364` |

### θ_htail sweep (α = +7°, T = ×1.0)

| θ_ht [°] | case ID |
|---:|---|
| −12 | `case-c09282c9-caa2-4e95-a651-efa1f324c813` |
| −9 | `case-7dbd5a59-59a2-4138-adcb-b99f06c08998` |
| −6 | `case-7f77be58-8680-4fee-b412-a9cff1bed42a` |
| −3 | `case-c1320127-c38b-42f8-b699-d313bbf90041` |
| 0 | `case-70e450cd-5e5f-4b82-9f68-f3fedbc22bff` ★ |
| +3 | `case-55dcf454-de8c-4c4d-9a16-47ca6609f8f8` |
| +6 | `case-0aeb2041-21cc-478b-a003-5923478d04f2` |
| +9 | `case-8eefccb9-8608-48a8-8039-e177fca0e5a3` |
| +12 | `case-c704699b-b68d-40f4-8f58-1a28e826862c` |
| +15 | `case-ad90944f-f90b-49d7-8940-03292de13ed6` |

### Thrust sweep (α = +7°, θ_ht = 0°)

| mult | case ID |
|---:|---|
| 0.00 | `case-d820f45a-3d8f-40ae-970c-3d50f24d89be` |
| 0.25 | `case-59db849a-1bb5-4275-92c0-226ca2a6a12d` |
| 0.50 | `case-c8d05bc2-fdb6-4662-84b1-bbae3a4a8f03` |
| 0.75 | `case-b06cd35b-5284-4f4b-bec4-001b718eb63c` |
| 1.00 | `case-70e450cd-5e5f-4b82-9f68-f3fedbc22bff` ★ |
| 1.25 | `case-9f8fcda4-1a44-4301-9ad0-b83064a0150d` |
| 1.50 | `case-096eb33a-3482-4c1a-8253-6f5ce771209d` |
| 2.00 | `case-5d110a48-b757-4916-95e7-d0b49680fed4` |
| 2.50 | `case-c24fcad9-d5ec-47da-902a-8d66de570bd0` |
| 3.00 | `case-2f5c1360-8755-4b45-97c4-852d79930a82` |

★ `case-70e450cd` appears in both htail and thrust sweeps — it's the
shared θ_ht=0° / T=×1.0 baseline (Flow360 deduplicated the two
identical fork submissions).

## Reproduce

```bash
python3 post/plot_sweep_sensitivities.py            # use cached out/sweep_data.csv
python3 post/plot_sweep_sensitivities.py --refresh  # refetch all 30 cases from cloud
```

Outputs land in `post/out/`:
- `sweep_data.csv` — final-step CL/CD/CMy/CFx/F_AD_delivered_N per case
- `sensitivities.png` — the 3×3 grid above
- `thrust_balance.png` — aircraft drag vs commanded vs delivered thrust
