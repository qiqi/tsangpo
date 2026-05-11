# Flow360 Meshing Strategy — Himalayan eSTOL

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

Streamtube zones are tagged by attribute `propIndex` carried over from the
CSM file, so they re-build automatically when the geometry is regenerated
for a different `Z_tail` station.

## Cell count target

| Configuration | Approx cells |
|---------------|--------------|
| Baseline      | 35 M         |
| Proposed      | 42 M (extra refinement in the gap shear layer) |
| Sweep cases   | 35-40 M each |

## Verification

One mesh-independence triple on the Proposed case at the design point:

- Coarse: 50 % cells (~21 M)
- Medium: nominal (~42 M)
- Fine:   200 % cells (~84 M)

Acceptance: `|ΔC_m| < 0.002` and `|ΔC_L| < 0.01` between Medium and Fine.
