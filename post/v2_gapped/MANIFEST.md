# v2 GAPPED-FLAP geometry — campaign manifest

Sister doc of `post/v2_continuous/MANIFEST.md`.  Same v2 airframe
(CG-origin, bigger htail at longer arm), but the main wing is one
continuous solid with the cove FILLED in the middle 39 % of span by
the tail-cap (built via `build_estol_geometry.build_main_wing_tail()`
and `JOIN`'d in ESP).  Geometry is in `geometry/tsangpo_gapped.csm`
(commit `6cbff27` — continuous main wing + 0.5 %-of-span flap gap).

Geometry parameters (mid_outer_semispan = 0.39, gap_fraction = 0.40):
  * Main wing: continuous full-span solid, complete LS(1)-0417 cross-
    section in y ∈ [−0.39 s, +0.39 s]; coved elsewhere.
  * Flap+vane: left/right panels, y ∈ [±0.40 s, ±0.95 s], with a
    0.5 %-of-span air gap inboard.

## Cruise gap40

- **Project**: `prj-59c27343-3c39-43a4-ab19-18863acf02c4` (`tsangpo_v2_cruise_gapped40`)
- **Parent**: `case-3a4859d9-5298-4a35-8d68-b98bcb26b580` (α=+7°, θ_ht=0°, T_mult=+1, V=45.72 m/s)
- BO forces: CL=+0.862, CMy_CG=+0.151, CFx=+0.072, CT_del=+0.043
- Sweeps: 9 × 3 forks queued by `submit_gapped_sweeps.py` (alpha/htail/thrust).

## Takeoff gap40

- **Project**: `prj-e0e11ed5-2f3a-4e7d-8af7-c79baaf3ed22` (`tsangpo_v2_takeoff_gapped40`)
- **Parent**: `case-fe40382d-13d5-4343-8626-453fd3872993` (α=+8°, θ_ht=−5°, T_mult=+16, V=18 m/s, γ=+30°)
- BO forces: CL=+4.14, CMy_CG=+0.23, CFx=+1.52, CT_del=+2.83
- Sweeps: 9 × 3 forks queued (htail extended +25 → +30 deg).

## Landing gap40

- **Project**: `prj-0cd29981-d281-442a-984f-06562abc1f39` (`tsangpo_v2_landing_gapped40`)
- **Parent**: `case-3fe6a528-9151-4337-8ece-8f0862fee1bf` (α=+8°, θ_ht=−6°, T_mult=+12, V=12.86 m/s, γ=−30°)
- BO forces: CL=+5.96, CMy_CG=+1.83, CFx=+3.60, CT_del=+4.15;
  htail CL=−0.59 (htail working — vs ≈ 0 stalled in continuous-flap landing).
- Sweeps: 9 × 3 forks queued (htail extended −10 → +50 deg).

## Folder layout

    post/v2_gapped/
      ├── MANIFEST.md                          (this file)
      ├── TRIM_RESULTS.md                      (filled in once sweeps land)
      ├── plot_cruise_sensitivities.py
      ├── plot_takeoff_sensitivities.py
      ├── plot_landing_sensitivities.py
      └── _common.py                           (sweep-case discovery helper)

    post/out/v2_gapped/
      ├── cruise_sweep_data.csv
      ├── cruise_sensitivities.png
      ├── cruise_thrust_balance.png
      └── …(takeoff / landing analogues)

The plot scripts use `_common.discover_sweep_cases()` to pull sweep case
IDs from the project by name pattern (`gap40_{cruise,TO,LD}_{sweep}_{tok}`),
so they keep working as forks land — no hardcoded case-ID list to update.
