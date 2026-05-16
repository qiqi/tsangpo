# Ground-effect landing maneuver in CFD (full paper, §7.2)

## Goal

Demonstrate the v3 (gap-low, short-tail) configuration executing a
representative landing maneuver in unsteady CFD with ground proximity.
The maneuver couples:

- evolving ground effect on wing and htail
- finite pitch dynamics during flare
- thrust-mult ramp-down on touchdown approach
- htail in slipstream throughout

Single high-resolution CFD run, treated as a publication-grade
verification of the static aero conclusions of §3–§6 of the
extended abstract.

## Maneuver definition

1. **Initial state.** Trimmed at landing BO (α≈+8°, θ_ht≈-6°,
   T_mult≈+12, V≈12.86 m/s) at h = 30 ft AGL on a -3° glideslope.
   No ground effect yet.
2. **Descent.** Hold attitude and trim, descend along glideslope.
   Ground effect builds smoothly through h ≈ 1 c_w.
3. **Flare initiation** at h ≈ 2 ft AGL. Prescribed nose-up pitch
   schedule (e.g., 5°/s) to land at α_touchdown ≈ +17°.
4. **Throttle chop** at flare initiation: T_mult ramped from +12 to
   +6 over 1.5 s.
5. **Touchdown.** Main-gear contact when wing-tip extension hits
   z_ground.

## CFD setup

- Time-accurate RANS, Δt small enough to resolve flare pitch rate
  (≈ 0.01 s).
- Move the entire aircraft body in a translating + rotating
  frame (existing rotation-volume / translation-volume Flow360
  primitives).
- Ground plane as a slip wall (or no-slip, sensitivity to be
  studied) added to the farfield template.
- Actuator-disk thrust ramped via the same machinery as the
  existing sweeps.
- Outputs: time-series of CL, CD, Cm, htail CL, ground-effect
  pressure gradient on belly, slipstream attachment to the
  ground plane.

## Key questions to answer

1. Does the v3 htail (now closer to wing and lower) keep slipstream
   coverage during ground effect? Or does the ground push the
   slipstream upward into / past the htail?
2. How much does ground effect amplify the wing CL? Does it
   exacerbate the static margin (ground effect typically reduces
   downwash → larger destabilising wing contribution)?
3. Does the htail trim deflection at touchdown stay within
   the +20° landing stall ceiling under the dynamic flare?
4. Is the throttle-chop pitch transient recoverable on hand-flown
   stick, or does it require FCS assist?

## Deliverables

- Time history plot: CL, Cm, α, θ_ht, h_AGL, T_mult.
- Y-slice movie frames of velocity and total pressure during flare.
- Pressure-on-ground footprint visualisation.
- Comparison of trimmed-static and dynamic-flare CL at the same
  α (the "ground-effect headroom" credit).

## Mesh / cost estimate

- Add a flat ground patch ≈ 5b × 5b in extent.
- Refinement zone in the wake corridor + below the htail.
- Estimated mesh size ≈ 1.3–1.5× existing landing case.
- Wall-clock ≈ 24 h on a single GPU node for the full maneuver.
