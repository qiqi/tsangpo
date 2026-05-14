# v2 geometry — campaign manifest

This is the case-id manifest for the v2 campaigns submitted on the
revised geometry (`Tsangpo/5_v2_low_htail_bigger_tail/`).  v2 is
defined by **commit `a3b1d86`** (params.py, tsangpo.csm, airframe.yaml,
cfd_setup.py).  v1 reference commit is `e10892f`.

Three campaigns submitted at the v1 BO trim points (so we have a clean
v1↔v2 comparison at the same target conditions).  Each is on its own
Flow360 project (cruise/takeoff/landing — flap phase 0/1/2), with one
parent at the BO point and 30 forks (10 each for α, θ_htail, T_mult).

Tight-iter settings inherited from v1: parent gets 1000 pseudo iters
× 20 steps; forks get 500 pseudo iters × 6 new steps.

## Cruise v2

- **Project**: `prj-ee96bbf8-0d42-442b-ab25-c16e64a618c0` (`tsangpo_v2_cruise`)
- **Parent** (α=+7°, θ_ht=0°, T_mult=+1, V=45.72 m/s, level flight): `case-14183aea-be39-4982-ac36-507a1eeb6c39`

| sweep | values | case IDs |
|---|---|---|
| α [°] | −3, −1, +1, +3, +5, +7, +9, +11, +13, +15 | `19f5eff5`, `e2cb6bd2`, `f80d9ec1`, `bf4c8d51`, `5504bbb0`, `4355b365`★, `ae6f4657`, `cc4dadf6`, `d13feb6f`, `bed86e5b` |
| θ_ht [°] | −12, −9, −6, −3, 0, +3, +6, +9, +12, +15 | `0741f50b`, `5af97e97`, `7bb6b60c`, `6082a894`, `4355b365`★, `d18bff39`, `b60cea02`, `027e7b70`, `6792d541`, `adbd01f4` |
| T_mult | 0, 0.25, 0.50, 0.75, 1.00, 1.25, 1.50, 2.00, 2.50, 3.00 | `23b29afb`, `b587eb9c`, `9ca2b04a`, `5c9133bb`, `4355b365`★, `35f694d4`, `0a2db14b`, `7cf40b93`, `e6a1a692`, `b0ba9825` |

★ `case-4355b365` is the BO baseline (α=+7°, θ_ht=0, T_mult=1) shared
by all three sweeps via Flow360 dedup.

## Takeoff v2

- **Project**: `prj-fd35217f-4135-4062-84d7-9bf55b8d6222` (`tsangpo_v2_takeoff_coarse`)
- **Parent** (α=+8°, θ_ht=−5°, T_mult=+16, V=18 m/s, γ=+30°, phase-1 flap): `case-b196ff81-630c-4d2d-aab7-ce6a2753b1ec`
- **Note**: htail sweep extended from v1's (−12..+9) to **(−15..+30)** to bracket positive-deflection unstall territory.

| sweep | values | case IDs |
|---|---|---|
| α [°] | −2, +2, +5, +8, +11, +14, +17, +20, +25, +30 | `c727fa14`, `cad5799a`, `b0d014b3`, `e22dcb08`★, `d7f25bff`, `20334b3a`, `ab85ec53`, `e8a41188`, `4e6be124`, `4d4dfdc7` |
| θ_ht [°] | **−15, −10, −5, 0, +5, +10, +15, +20, +25, +30** | `c262e960`, `99f13ced`, `e22dcb08`★, `2794af3f`, … (full list when sweep completes) |
| T_mult | 6, 9, 12, 14, 16, 18, 20, 22, 25, 30 | (queued; see submit log) |

★ `case-e22dcb08` is the BO baseline shared between α=+8° and the dedup'd θ_ht=−5° / T_mult=16 points.

## Landing v2

- **Project**: `prj-c5e371dc-fc34-448e-8953-9785f3d218f4` (`tsangpo_v2_landing_coarse`)
- **Parent** (α=+8°, θ_ht=−6°, T_mult=+12, V=12.86 m/s, γ=−30°, phase-2 flap): `case-bc31e4f2-890b-4827-88eb-08d142c76d9a`
- **Note**: htail sweep extended dramatically from v1's (−15..+12) to **(−10..+50)** to find the unstalled region; v1 analysis showed the entire sweep range was inside the stalled regime due to ~−45° downwash from wing+flap.

| sweep | values | case IDs |
|---|---|---|
| α [°] | −2, +2, +5, +8, +11, +14, +17, +20, +25, +30 | `68f8165a`, `5cea79fe`, `f62c29bf`, `c47f78ee`★, `ec395796`, `e08deb06`, `8600decf`, `0f65995c`, `f89daf59`, `9a65cca5` |
| θ_ht [°] | **−10, 0, +10, +15, +20, +25, +30, +35, +40, +50** | `5e9fc1b0`, `77d12901`, `85c7a12e`, `da6ea5dd`, `8570c4db`, `297bcff7`, `a17f7ac1`, … |
| T_mult | 4, 7, 10, 12, 14, 16, 18, 21, 25, 30 | (queued) |

## Next steps

1. Wait for parents + forks to land.
2. Adapt `plot_takeoff_sensitivities.py` / `plot_landing_sensitivities.py`
   to point at v2 case IDs, drop the v1-only `+0.4·CFx` CMy_CG shift
   (CFD's moment_center is now AT the CG), keep the `−0.1·CT_delivered`
   thrust contribution.
3. Solve the 3×3 trim system again; with V_H ≈ 2.1× larger and the
   bigger htail less wake-shielded, we should see far better trim
   feasibility — that's the design hypothesis we're testing.
4. If trim closes for cruise/takeoff/landing on v2 → success.  If
   landing still doesn't trim because even the bigger low-htail is in
   too-heavy slipstream wake, we'd then look at configs 2/4 (high htail).
