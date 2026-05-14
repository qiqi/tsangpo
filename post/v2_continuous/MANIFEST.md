# v2 geometry — campaign manifest

This is the case-id manifest for the v2 campaigns submitted on the
revised geometry (`Tsangpo/5_v2_continuous_low_htail/`).  v2 is
defined by **commit `a3b1d86`** (params.py, tsangpo.csm, airframe.yaml,
cfd_setup.py).  v1 reference commit is `e10892f`.

Three campaigns submitted at the v1 BO trim points (so we have a clean
v1↔v2 comparison at the same target conditions).  Each is on its own
Flow360 project (cruise/takeoff/landing — flap phase 0/1/2), with one
parent at the BO point and 30 forks (10 each for α, θ_htail, T_mult).

Tight-iter settings inherited from v1: parent gets 1000 pseudo iters
× 20 steps; forks get 500 pseudo iters × 6 new steps.

## Cruise v2

- **Project**: `prj-16082511-6d6a-447e-8c54-828d476c0a85` (`tsangpo_v2_cruise`) — LIVE.
  (`prj-ee96bbf8…` was BROKEN_CSM (binary-+ in CSM `set`).  `prj-d90a26cf…` was
  BROKEN2_plus_op (renaming attempt).  `prj-29352296…` was BROKEN3_enclosed_missing
  (legacy mesher needed `enclosed_entities=[htail_surf]`). All three are tagged
  `*_BROKEN*` in the 5_v2 folder and kept for forensics.)
- **Parent** (α=+7°, θ_ht=0°, T_mult=+1, V=45.72 m/s, level flight): `case-e30a9610-248f-40af-aa9f-9f30846c419d`
  - Verified: htail surface = `htail_pitch_zone/htail` (not `farfield/htail`).
    main_wing CL=+0.929, htail CL=+0.0004, htail CMy_CG=+0.029 (θ_ht=0).

_⚠ STALE — case IDs below were from the BROKEN_CSM run; the live project has different IDs. Refresh after live forks complete._

| sweep | values | case IDs |
|---|---|---|
| α [°] | −3, −1, +1, +3, +5, +7, +9, +11, +13, +15 | `19f5eff5`, `e2cb6bd2`, `f80d9ec1`, `bf4c8d51`, `5504bbb0`, `4355b365`★, `ae6f4657`, `cc4dadf6`, `d13feb6f`, `bed86e5b` |
| θ_ht [°] | −12, −9, −6, −3, 0, +3, +6, +9, +12, +15 | `0741f50b`, `5af97e97`, `7bb6b60c`, `6082a894`, `4355b365`★, `d18bff39`, `b60cea02`, `027e7b70`, `6792d541`, `adbd01f4` |
| T_mult | 0, 0.25, 0.50, 0.75, 1.00, 1.25, 1.50, 2.00, 2.50, 3.00 | `23b29afb`, `b587eb9c`, `9ca2b04a`, `5c9133bb`, `4355b365`★, `35f694d4`, `0a2db14b`, `7cf40b93`, `e6a1a692`, `b0ba9825` |

★ `case-4355b365` is the BO baseline (α=+7°, θ_ht=0, T_mult=1) shared
by all three sweeps via Flow360 dedup.

## Takeoff v2

- **Project**: `prj-9cd3ad10-ea47-41d9-9e33-0096fc30d6c1` (`tsangpo_v2_takeoff_coarse`) — LIVE
  (BROKEN siblings: `prj-fd35217f…`, `prj-2e8dff30…`, `prj-efe82f5a…`).
- **Parent** (α=+8°, θ_ht=−5°, T_mult=+16, V=18 m/s, γ=+30°, phase-1 flap): `case-118aced8-e380-45a5-b374-04a303f1802a`
  - Verified: htail CL=−0.517, htail CMy_CG=+1.995 (big pitch-up at θ_ht=−5°).
- **Note**: htail sweep extended from v1's (−12..+9) to **(−15..+30)** to bracket positive-deflection unstall territory.

_⚠ STALE — case IDs below were from the BROKEN_CSM run; the live project has different IDs. Refresh after live forks complete._

| sweep | values | case IDs |
|---|---|---|
| α [°] | −2, +2, +5, +8, +11, +14, +17, +20, +25, +30 | `c727fa14`, `cad5799a`, `b0d014b3`, `e22dcb08`★, `d7f25bff`, `20334b3a`, `ab85ec53`, `e8a41188`, `4e6be124`, `4d4dfdc7` |
| θ_ht [°] | **−15, −10, −5, 0, +5, +10, +15, +20, +25, +30** | `c262e960`, `99f13ced`, `e22dcb08`★, `2794af3f`, … (full list when sweep completes) |
| T_mult | 6, 9, 12, 14, 16, 18, 20, 22, 25, 30 | (queued; see submit log) |

★ `case-e22dcb08` is the BO baseline shared between α=+8° and the dedup'd θ_ht=−5° / T_mult=16 points.

## Landing v2

- **Project**: `prj-e7dc7d6d-4baf-4101-8818-1173da5a359a` (`tsangpo_v2_landing_coarse`) — LIVE
  (BROKEN siblings: `prj-c5e371dc…`, `prj-7db5be94…`, `prj-619b0dbf…`).
- **Parent** (α=+8°, θ_ht=−6°, T_mult=+12, V=12.86 m/s, γ=−30°, phase-2 flap): `case-fed9edf1-681b…` (RUNNING as of 2026-05-14)
- **Note**: htail sweep extended dramatically from v1's (−15..+12) to **(−10..+50)** to find the unstalled region; v1 analysis showed the entire sweep range was inside the stalled regime due to ~−45° downwash from wing+flap.

_⚠ STALE — case IDs below were from the BROKEN_CSM run; the live project has different IDs. Refresh after live forks complete._

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
