# Phugoid dynamics simulation plan (full paper, §7.1)

## Goal

Compare the **longitudinal dynamic** response of three configurations
under the same disturbances, to convert the static-margin headline
of the extended abstract into a handling-quality story:

1. **cont-high (v2)** — baseline T-tail, continuous flap.
2. **gap-low (v2)** — flap gap + low htail (the static-aero winner).
3. **gap-low (v3)** — same as 2 with the 2 c-aft-of-wing shortened
   tail boom.

For each configuration we exercise the **phugoid mode** at the
landing trim point (the most-constrained phase) and compare:

- damping ratio ζ_phugoid
- natural frequency ω_n
- time-to-half (or time-to-double) amplitude
- coupling into short-period (frequency separation)
- response to elevator and thrust step disturbances

## Method options

### Option A — linearised 4-state longitudinal model

State: (u, w, q, θ). Coefficients from the existing
sensitivity sweeps (see `post/v2_continuous_high/*` and
`post/v2_gapped/*`) plus a one-time CFD measurement of:

- `dCmy/dq` (pitch damping) — needs a forced pitch oscillation
- `dCL/dq` — small but non-zero with htail in slipstream
- `dCm/du, dCL/du` — velocity sensitivities at trim (slow but
  matters for phugoid).

The phugoid is dominated by (u, θ) coupling so these
velocity-sensitivities are essential. Today's sweep doesn't
include them — we need a small u-sweep (e.g., V_b ± 5 m/s at
fixed α, θ_ht, T_mult) in each phase.

Linearised model → eigenvalues of A-matrix → phugoid root.
Cheap. Suitable for the abstract / cross-config compare.

### Option B — direct CFD time integration

Run the unsteady RANS with prescribed CG translation /
rotation following the dynamic equations of motion, fed back
through the actuator-disk thrust. Resolves coupling we'd
otherwise miss (slipstream evolution as V changes, time-lag of
htail wake) but ~50–100× more expensive.

Recommend B for **one** ground-effect demonstrator case
(§7.2 of the abstract) and A for the cross-config phugoid
comparison.

## Deliverables

- Table: phugoid (ζ, ω_n, t½) for cont-high / gap-low-v2 / gap-low-v3.
- Plot: pole locations in s-plane across the three configs.
- Plot: time histories of (V, θ, α) following a 5°/1 s elevator
  pulse, three configs overlaid.
- Headline: which configuration gives the pilot the most
  forgiving handling, and how much margin v3 retains relative
  to v2-gap-low.

## CFD tasks needed

1. Per-phase velocity sweep (V_b ± 5 m/s at BO), three configs ×
   three phases ≈ 18 cases. Reuse existing fork machinery.
2. (Optional, only if Option B is selected) Forced pitch
   oscillation at f = ω_n/(2π) for one case per config to
   measure pitch damping coefficient. ≈ 3 cases.
