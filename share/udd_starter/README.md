# UDD-driven rigid-body dynamics in Flow360 — starter pack

A how-to for running coupled rigid-body / CFD simulations in Flow360 using
its **User-Defined Dynamics** (UDD) facility.  The pattern here was developed
for the Tsangpo eSTOL project and used to map static-stability and free-flight
phugoid behavior of a tilt-arm aircraft; the underlying recipe is general.

This pack assumes you already have a working Flow360 setup (Python SDK,
geometry pipeline, an actuator-disk or BET model if needed).  Everything
specific to Tsangpo lives in the parent repo (`flow360/unsteady_setup.py`,
`params.py`, `cfd_setup.py`); this directory has minimal examples you can
adapt.

## What a "UDD simulation" buys you

Flow360 normally treats rotating volume zones as kinematically prescribed
— the user supplies angle(t) for each cylinder.  UDD lets you make the
angle(t) a function of forces/moments integrated from the CFD wall plus
some state vector you maintain across pseudo iterations.  That turns the
solver into a one-way-coupled rigid-body integrator: the flow drives the
body, the body's pose drives the mesh.

Typical use cases:

- **Static-stability scans**.  Hold most degrees of freedom (linkage
  angles, translation) fixed, free up one (e.g. body pitch).  Sweep an
  input parameter (htail incidence, V_inf).  The body relaxes to its
  equilibrium angle for each cell — this pack documents that workflow
  end-to-end.
- **Free-flight dynamics**.  All 3 longitudinal DOFs (x, z, θ).  Integrate
  Newton's equations from CFD forces.  Gives phugoid / short-period /
  flare dynamics directly out of CFD.
- **Closed-loop control**.  The UDD also has access to the simulation
  time and physical-step count, so a controller can compute control
  inputs from state and feed them into prescribed-motion UDDs for control
  surfaces.

## Mesh topology

The pack assumes a **nested rotating-cylinder** topology with N cylinders.
For the Tsangpo arrangement (3-DOF longitudinal):

```
                 freestream (stationary outer zone)
                          │
                ┌──────── cyl_1 ────────┐       link1: shoulder pitch
                │                       │       (whole airplane swings around outer pivot)
                │  ┌────  cyl_2  ─────┐ │
                │  │                  │ │       link2: elbow pitch
                │  │  ┌──  cyl_3  ─┐  │ │       (arm folds at elbow)
                │  │  │            │  │ │
                │  │  │   airframe │  │ │       θ_body: airframe pitch
                │  │  │            │  │ │       around its own CG
                │  │  │  ┌─ cyl_4 ┐│  │ │
                │  │  │  │ htail  ││  │ │       θ_ht: horizontal stabilizer incidence
                │  │  │  └────────┘│  │ │
                │  │  └────────────┘  │ │
                │  └──────────────────┘ │
                └───────────────────────┘
```

Each cylinder's rotation is driven by one UDD.  The UDDs share a common
state vector via Flow360's "recorder" mechanism (each UDD can read
others' previous-step state via output_vars).

For your problem you may have fewer cylinders (e.g. 1 if all you need
is body pitch in fixed translation) or different ones (yaw axis, rotor
pitch axis, etc.).  The integrator recipe in this pack adapts directly.

## State vector convention

Across the UDDs we maintain a shared state vector.  The Tsangpo project
uses:

| slot | meaning | units |
|---|---|---|
| state[0] | CG world x | grid units (m) |
| state[1] | CG world z | grid units (m) |
| state[2] | spare or pitch rate q | rad/s_nd |
| state[3] | spare or CG vel | m/s_nd |
| state[4] | θ_body (airframe pitch) | rad |
| state[5] | q (pitch rate) | rad / time_nd |
| state[6] | link2 angle (elbow) | rad |
| state[7] | link1 angle (shoulder) | rad |

You can repurpose slots freely; what matters is that the integrator
update_law inside each UDD references them consistently.  **Critical**:
Flow360 evaluates the update_law expressions **sequentially** by slot
index per pseudo step, so state[i+1]'s expression sees the NEW value of
state[i] if state[i] is updated first.  Use this for symplectic
integrators; abuse it and you get one-step lag bugs.

## The integrator (production recipe)

For static-stability work we use a damped 1-DOF integrator on θ_body:

$$ \dot q = \frac{M(\theta)}{I_{yy}} - \frac{D}{I_{yy}} q, \quad \dot\theta = q $$

With Flow360's pseudo-step gate `if (pseudoStep == 0) … else state[5]`,
the update happens once per physical step using forces from the END of the
previous step.  Two variants exist:

### Implicit-damping (semi-implicit Euler-Cromer)

```
q_new = (q_old + Q_TORQUE · dt) / (1 + (D/I) · dt)
θ_new = θ_old + q_new · dt
```

Stability: unconditionally stable in q.  The (1 + D·dt/I) denominator
keeps q bounded even when D is large.  Useful when you want to track
real dynamics with q-inertia (e.g. phugoid).

### Quasi-static (no q memory) — for fast equilibrium seeking

```
q_new = Q_TORQUE / D                ← no q_old, no dt in q
θ_new = θ_old + q_new · dt
```

Equivalent to dropping the inertial term entirely (I → 0).  Body relaxes
on the timescale D/k where k = -∂M/∂θ.  Stability condition is just
forward-Euler on the slow θ ODE: **dt < 2·D/k**.  Pick D so that this
holds at the worst-case k in your operating range.

For the Tsangpo landing config, V=12.86 m/s, the local k ranges from
~1.7 kN·m/rad (linear regime) to ~13 kN·m/rad (stalled regime, V=16).
We use **D = 30 kN·m·s/rad** and dt = 2 s — gives dt·k/D ∈ [0.11, 0.87]
across all operating points.

## Time-stepping pattern

Each equilibrium cell runs in two stages:

1. **Warmup**: a few physical steps with **all** cylinders frozen at the
   target initial pose.  This brings the flow from a uniform initial
   condition to a converged steady solution at the trim pose, gives the
   actuator disk a chance to ramp up, and clears transients.

   Recipe: `n_physical_steps = 3`, `timestep_size_s = 1.0`,
   `max_pseudo_steps = 1500`.  Wall time ~15 min per cell (queue
   plus mesh build dominate).

2. **Chunk** (forked from warmup): free up the DOF of interest, run the
   integrator.

   Recipe (quasi-static D=30 kN, dt=2 s, 8 steps):
   `n_physical_steps = 8`, `timestep_size_s = 2.0`, `max_pseudo_steps = 1500`,
   `pitch_damping_si = 30000.0`, `quasi_static_q = True`.
   Wall time ~15-20 min per cell.

You can extend with chunk2, chunk3… by forking from the previous chunk's
end state if τ is large at your operating point and 8 steps isn't enough.

## Post-processing: extracting θ_eq with CI

We fit Δθ_k = A·exp(-r·k) to the LAST 4 Δθ values (k=0..3) and apply
two tests:

- **Test 1 ("converged"):** t-test (df=3) on the mean of the last 4 Δθ
  against zero.  If not rejected at 95%, fluctuations are noise around
  the equilibrium — report θ_eq = mean(last 4 θ), CI = half-range.

- **Test 2 ("drift"):** one-sided t-test (df=2) on the fitted r vs 0.
  If r > 0 is not significant at 95%, the body isn't decaying — report
  drift (no point estimate).

- **Else ("exp_OK"):** extrapolate the geometric tail past k=3:

  $$ \theta_{eq} = \theta_{\text{last}} + \frac{A \, e^{-4r}}{1 - e^{-r}} $$

  Propagate CI through the joint covariance cov(A,r) using the analytic
  gradient.  Use t-critical with df=2 (≈4.30 for 95% two-sided) — the
  honest small-sample correction.

See `example_extract.py` for the implementation.

## Pitfalls we hit

Things that took us many iterations to figure out — written down so the
next person doesn't repeat them.

1. **Always use `use_beta_mesher=True`**.  The legacy mesher silently
   ignores `MeshingDefaults.curvature_resolution_angle`, gives poor LE
   refinement on smooth spline airfoils.

2. **Always emit a y=0 SliceOutput**.  Cheap (~10 MB per case) and saves
   you from having to fork a 1-step case later just to look at the
   symmetry plane.  Pattern:

   ```python
   fl.SliceOutput(
       name="y0_slice",
       entities=[fl.Slice(name="y=0",
                          normal=(0.0, 1.0, 0.0),
                          origin=(0.0, 0.0, 0.0) * fl.u.m)],
       output_fields=["velocity", "Mach", "Cp", "primitiveVars"],
   )
   ```

3. **The `pseudoStep == 0` gate is mandatory** for state updates that
   drive grid rotation.  Flow360 auto-inserts a `previousTheta_recorder`
   UDD that snapshots θ at pseudoStep == 0 and computes
   `deltaTheta = theta_new - previousTheta_old` for the grid rotation.
   If you update state[θ] at pseudoStep == LAST, the snapshot still
   reads the OLD θ at pseudoStep == 0 of the next step → deltaTheta = 0
   → mesh never rotates.  This is the "plateASI gate convention".

4. **publish q AND ω̇ alongside θ in each UDD** rather than letting
   Flow360 finite-difference them from θ.  The grid rotation engine
   needs angular velocity and acceleration; finite-difference gives
   sluggish/lagged responses.

5. **UDD inputs are in SOLVER NONDIM units**, not coefficient form.
   Force coefficients on the `total_forces_v2.csv` differ from the
   UDD's `forceX, forceZ, momentY` inputs by the factor
   `(q_inf · S_ref · L_ref) / (ρ_∞ · c_∞² · L_grid³)`, which is
   roughly `2/Mach² · (S·L/L_grid³)` — easily ~60–100× for low-Mach
   problems with body-chord references.

6. **Actuator-disk contributions to the body moment**.  AD's are
   volume body-forces, not wall integrations — Flow360 reports their
   force/moment in `actuatorDisk_force_coefficients_v2.csv` with
   ROTOR-specific normalization (different from airframe coefficients).
   The UDD's `momentY` input is wall-only.  Add the disk reaction
   manually as `-prop_z · T_disk_nd` (a constant if thrust is fixed)
   in your Q_TORQUE expression.

7. **Lever-arm correction**.  If wall moments are integrated about a
   fixed reference point (Flow360's default) but the body's CG has
   moved, translate the moment to the actual CG:

   ```
   M_about_CG = momentY + (mc_z - CG_z) · forceX − (mc_x - CG_x) · forceZ
   ```

   For 1-DOF tests where the CG is held fixed (cyl_1, cyl_2 frozen),
   this term is identically zero — confirmed empirically to within FK
   trig noise (~1e-9 rad).

8. **Explicit-damping stability**:
   `q_new = q_old + (M/I − D·q_old/I)·dt` is **only stable for
   dt < 2·I/D**.  For us (I=3000, D=10k) that's dt < 0.6 s.  Going to
   dt=2 s with explicit damping caused divergent overshoot in q (gain
   factor −5.67), the body rotated to −63° in two steps, mesh
   interfaces broke.  Use the implicit form (denominator 1 + D·dt/I) —
   unconditionally stable for any dt.

9. **Quasi-static is the limit of implicit**, but with the dt·D/I → ∞
   limit it has no q-memory.  For our problem, at dt=2 s, D=30 kN·m·s/rad,
   I=3000 kg·m²: dt·D/I = 20 → q-memory factor 1/21 = 4.7%.  Already
   negligible.  The pure quasi-static form gets you that for free at
   any dt.

10. **Forward-Euler θ stability** under quasi-static damping:
    `dt < 2·D/k_local`.  At Tsangpo high-V stall, k_local ≈ 13 000
    N·m/rad → D = 10 kN gives dt·k/D = 2.6 (divergent overshoot
    in θ).  Bumping D to 30 kN restores stability everywhere.

11. **Cloud queue throughput** can be the wall-time bottleneck for a
    grid sweep.  Don't assume 15 cells run in parallel — we saw a
    fairly common throttle of 1-2 concurrent cases.  Build your
    orchestrator with that in mind (phase 1 submit all warmups in
    parallel, phase 2/3 poll and fork chunks as warmups finish — the
    classic producer-consumer pattern).

12. **CSM face naming**: name every element's faces with
    `attribute capsGroup $<name>` in the .csm (right after `extrude`).
    Flow360's geometry processor exposes `capsGroup` as a face
    attribute tag; `geo.group_faces_by_tag("capsGroup")` retrieves them.
    Don't rely on body-dump order or `body0000N` renaming — fragile
    across configs.

13. **Inline UDCs into a single .csm before upload**.  Multi-file STEP
    uploads only surface-mesh body00001 (Flow360 limitation as of
    25.9.x).

## Repo files to look at

In the parent repo, the most relevant files:

| file | what it has |
|---|---|
| `flow360/unsteady_setup.py` | `build_unsteady_params()` — full UDD construction for all modes. The "mode = 'pitch_step_cyl3_integrated'" branch is the production 1-DOF integrator. |
| `flow360/pitch_eq_grid.py` | orchestrator for the θ_eq(θ_ht, V_inf) sweep. Phase 1/2/3 producer/consumer. |
| `params.py` | trim points, geometry constants, atmosphere helpers. |
| `cfd_setup.py` | the steady SimulationParams template + surface-tag lookup. |
| `post/v3/plot_pitch_eq_case_diags.py` | the 12-pane diagnostic plotter — useful to inspect each case in detail. |
| `flow360/PHUGOID_UDD_POSTMORTEM.md` | the 3-DOF (free-flight) UDD framework lessons. |
| `flow360/UDD_DYNAMICS_GUIDE.md` | UDD slot-order conventions for sequential evaluation. |
| `flow360/LESSONS.md` | Flow360 gotchas accumulated over the project. |

## In this pack

- `README.md` — what you're reading.
- `example_submit.py` — minimal submission script showing one cell's
  warmup + quasi-static chunk forking pattern.
- `example_extract.py` — the θ_eq + CI extraction algorithm (the same
  one used by the production sweep, copy-paste reusable).
- `example_plot.py` — the 12-pane diagnostic plot generator.

To run an end-to-end test:

```bash
python3 example_submit.py    # submits one warmup + chunk pair
                              # writes case IDs to /tmp/udd_starter_ids.json
python3 example_extract.py    # reads CSV, fits, prints θ_eq ± CI
python3 example_plot.py       # writes a 12-pane diagnostic PNG
```
