# Flow360 Meshing Strategy — Tsangpo eSTOL

## Goal

Resolve, without numerical dissipation:

1. The **vortex/streamtube column** from each inboard propeller (Props 1 & 2)
   from the disk plane back to and beyond the H-tail leading edge.
2. The **gap-edge vortex pair** shed from the inboard edge of the outboard
   flap segment when `gap_fraction > 0`.
3. The **upwash field** on the H-tail lower surface.

Everything else is secondary; mesh budget is spent on (1)-(3).

## Zonal layout

| Zone                | Origin                          | Extent                                                 | Target cell size |
|---------------------|---------------------------------|--------------------------------------------------------|------------------|
| Farfield            | airframe centroid               | 30 · MAC sphere                                        | 50 · MAC         |
| Wing BL             | wing surface                    | y+ ≤ 1, 25 prismatic layers, growth 1.20               | first cell 1.5e-5 ft |
| H-tail BL           | H-tail surface                  | y+ ≤ 1, 25 prismatic layers, growth 1.20               | first cell 1.5e-5 ft |
| **Prop streamtubes**| each prop disk (10 total)       | cylindrical box, R = 1.2 · R_disk, length = X_tail + 0.5·MAC | 0.025 · R_disk   |
| **Gap shear layer** | inboard edge of outboard flap   | thin slab, thickness 0.1 · MAC, length to H-tail TE    | 0.020 · MAC      |
| Wing wake           | wing TE                         | wedge widening 5°, length 5 · MAC                      | 0.05 · MAC       |
| Tail wake           | H-tail TE                       | wedge widening 5°, length 3 · MAC_ht                   | 0.04 · MAC_ht    |

The 10 propeller streamtube refinement boxes are placed by
`flow360/run_matrix.py` from `params.PROP_Y_NONDIM` and `params.PROP_X_FT`
— the CSM does *not* carry prop geometry (Flow360 models them directly as
actuator disks, see `params.PROP_*` and `T_PER_PROP_LBF`).

The gap shear-layer slab is keyed off `gap_fraction × wing_semispan`; it
collapses to a no-op refinement when `gap_fraction = 0`.

## Cell count target

| Case                       | gap  | $Z_t/c$ | Approx cells |
|----------------------------|------|---------|--------------|
| C1 Industry Baseline       | 0.00 | +2.50   | 35 M         |
| C2 Downwash Failure        | 0.00 |  0.00   | 35 M         |
| C3 Bad Trade-off           | 0.35 | +2.50   | 42 M (extra refinement in the gap shear layer) |
| C4 Proposed Synthesis      | 0.35 |  0.00   | 42 M         |

## Verification

One mesh-independence triple on **C4 Proposed Synthesis** at the design
point — that's the load-bearing case for the paper:

- Coarse: 50 % cells (~21 M)
- Medium: nominal (~42 M)
- Fine:   200 % cells (~84 M)

Acceptance: `|ΔC_m| < 0.002` and `|ΔC_L| < 0.01` between Medium and Fine.
