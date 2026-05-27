# Free-flight phugoid via embedded-dynamics UDD — postmortem, field guide & vendor feedback

**Project:** Tsangpo eSTOL longitudinal free-flight (phugoid) in Flow360
(SDK v25.9.x, solver release-25.9, beta in-house mesher), driven by
User-Defined-Dynamics (UDD) that integrate 6-DOF rigid-body motion from the
CFD-reported aerodynamic loads and feed it back to a 4-cylinder sliding-mesh
"robot-arm" that pitches and translates the airframe through a fixed freestream.

**Outcome (2026-05-27):** a clean, physically-correct phugoid — period
**17.4 s** (classic estimate π√2·V/g = 18.6 s), damping ratio **ζ ≈ 0.026**
(lightly damped, stable). Peaks decay +44.6°→+38.7°→+33.4°→+28.9°. The airframe
is dynamically stable; what looked for weeks like a Flow360 solver defect was, in
the end, **two subtle bugs in how our UDD interacted with Flow360's execution
model**, on top of several genuine-but-undocumented API constraints.

This single document is self-contained and serves three audiences:
- **§1–§2** — cost and the debugging journey (for managers / future-self).
- **§3–§8** — a field guide for **engineers and coding agents** building
  embedded-dynamics cases (the part to read before writing any UDD).
- **§9** — concrete, ranked **feedback for the Flexcompute Flow360 team**.

Visual results referenced here: `post/v3/phugoid_runs/htail_p1p5deg/nokick_lagfix_trajectory.png`
(0-60 s body motion), `nokick_lagfix_attitude.mp4` (attitude along the flight
path), `nokick_lagfix_slices.mp4` (t=30-60 s y=0 Mach flow animation),
`nokick_diag_panels.png` (per-step UDD state diagnostics).

---

## 1. Cost of the debugging

| metric | value |
|---|---|
| Calendar span of UDD/phugoid work | **~15 days** (2026-05-12 → 05-27) |
| of which chasing the two root-cause bugs | ~2 weeks (the "wing-CL-frozen" misdiagnosis dominated) |
| Distinct CFD cases in the debug effort | **~100+** (66 in the unsteady registry + ~40 reproducer/warmup/pitch-step cases) |
| Typical unsteady multi-zone chunk | 200 physical steps × ≤200 pseudo-steps, **40–77 min wall** each |
| Reproducer families built (mostly dead ends) | 3-cyl concentric, eccentric 3-cyl, NACA0012 single-wing, pitch-ramp, pitch-step (×4 variants), actuator-disk delivery — ~10 families |

**FlexCredit:** the SDK exposes no per-case billing (`CaseMeta` carries
status/version/timestamps only) — **read the exact spend from the billing
dashboard.** Order-of-magnitude, this consumed the bulk of a multi-week solver
budget, and **almost all of it was avoidable** (see §9). The expensive lesson:
~2 weeks went into ever-more-elaborate reproducers chasing a "multi-cylinder
nested-rotation solver bug" **that did not exist.** The real causes were two
conventions in how a force-integrating UDD must be written, both invisible
without instrumentation Flow360 does not make easy.

---

## 2. The journey: misdiagnoses → root causes

```
Symptom A: "wing CL frozen — doesn't respond to body AoA"  (dCL/dα ≈ 0.001/°)
  ├─ misdiagnosis: nested rotating zones freeze inner-zone α-response
  │     → 3-cyl, eccentric, NACA0012 reproducers — all WORKED, ruled it out
  ├─ misdiagnosis: multi-zone mesh-resolution jump at sliding interfaces
  │     → gap110-vs-widegap comparison — ruled out
  ├─ real fix (necessary, not sufficient): must publish omega (not just theta) ... §5.5
  └─ ROOT CAUSE 1: UDD state-update gate convention (pseudoStep==0) ............... §4

Symptom B (after fix 1): "no-kick body dives monotonically from trim"
  ├─ misdiagnosis: catastrophic cancellation in moment translation
  │     → real arithmetic hazard, but NOT why it dived
  └─ ROOT CAUSE 2: sequential update_law eval + one-step lever-arm lag .......... §5(below)
```

Both root causes are fixed in `flow360/unsteady_setup.py`. The two sections that
matter most, if you read nothing else, are §4 and the lever-arm part of §3.

---

## 3. What a UDD is, and the setup in one paragraph

A `fl.UserDefinedDynamic` is a per-physical-step state machine the solver
evaluates inside the pseudo-step loop. Its `update_law` is a list of
expressions (one per `state[i]`) that read aero inputs (force/moment integrals
over chosen boundary patches) and write rotation angles back to one cylinder.

```python
fl.UserDefinedDynamic(
    name        = "phugoid_6DOF_cruise_shoulder",
    input_vars  = ["forceX", "forceZ", "momentY"],   # world frame, about moment_center
    constants   = {...},                              # named scalars
    state_vars_initial_value = [...],                 # state[0..N-1]
    update_law  = [...],                              # one expr per slot; SEQUENTIAL eval (§5)
    output_vars = {"theta": "...", "omega": "...", "omegaDot": "..."},
    output_target = cyl_shoulder,                     # exactly ONE Cylinder per UDD
    input_boundary_patches = [...],                   # wall surfaces summed for the inputs
)
```

Our airframe rides a chain of four `Cylinder` sliding-mesh zones
(shoulder→elbow→airframe→htail). Because **Flow360 allows one `output_target`
per UDD**, we author **four identical UDDs** (one per cylinder); each runs the
same 8–10-slot forward-Euler integrator of the longitudinal 6-DOF plus a 2-link
inverse kinematics (IK) that turns CG translation into shoulder/elbow rotations,
and each publishes only its own cylinder's `theta/omega/omegaDot`. Cylinders use
`spec=fl.FromUserDefinedDynamics()` in their `fl.Rotation`. State is in Flow360
solver non-dim (L_ref = 1 m). The freestream is fixed at +X; the airframe flies
−X nose-first (CSM geometry convention: +X = aft).

State layout we settled on (slot order is **load-bearing** — see §5):
`[0]x_cg [1]z_cg [2]Vx [3]Vz [4]θ_body [5]q [6]θ_link2 [7]θ_link1
[8,9]=ω̇ caches / diagnostics`.

---

## 4. Root cause #1 — the state-update gate convention (`pseudoStep == 0`)

**Bug:** state integration gated `if (pseudoStep == last_pseudo_step)`. Symptom:
wing CL completely frozen vs body α (`dCL_wing/dα ≈ 0.001/°` vs the healthy
~0.05–0.1/°). Cost ~2 weeks, misattributed to nested rotating zones.

**Mechanism:** Flow360 **auto-inserts a `<udd>_previousTheta_recorder` UDD** for
any UDD that publishes `theta`. That recorder snapshots `theta` at
**`pseudoStep == 0`** to populate `previousTheta`, and `rotateGrid()` rotates the
mesh by `deltaTheta = theta − previousTheta`. If your main UDD advances `theta`
at the *last* pseudo step, the recorder (firing at pseudoStep 0 of the next step)
copies the already-advanced `theta` as `previousTheta`, so `deltaTheta = 0`
forever — **the mesh never rotates**, body attitude never changes, wall CL is
pinned at trim.

**Fix:** gate every state-integration expression at **`pseudoStep == 0`**:
```
"if (pseudoStep == 0) <new value>; else state[i];"
```
The forces read at pseudoStep 0 are the *previous* physical step's converged
residuals (still in the registry) — correct to integrate, with one physical step
of lag that §5 then forces us to honor in the lever arm. This matches Flow360's
own PlateASI example convention.

**Verified:** dCL_wing/dα recovered to +0.07/° (case-c8cabf45) vs ~0.001/° buggy.

---

## 5. Root cause #2 — `update_law` is evaluated SEQUENTIALLY, and the lever arm must honor the force lag

After fix #1 the mesh rotated and CL responded — but a **no-kick run (start
exactly at trim, γ_kick = 0) still dived monotonically** +5.6°→−35° in 10 s. We
nearly concluded the airframe was statically unstable. Two intertwined,
undocumented facts:

**5.1 `update_law` is evaluated SEQUENTIALLY, slot 0→N, in place — NOT in
batch.** When `update_law[k]` references `state[j]`: it sees the **NEW**
(already-updated this step) value if `j < k`, and the **OLD** value if `j ≥ k`.
We proved this directly: two diagnostic slots holding *byte-identical* moment
expressions, one at index 5 and one at index 8, returned **opposite-sign**
results, because index 5 saw OLD `state[6,7]` while index 8 saw the `state[6,7]`
updated at indices 6,7. Our own comments had asserted "batch semantics" — wrong,
and load-bearing. **Order your slots so each integrator reads its inputs at the
timing you intend.** Our forward-Euler integrators (slots 0-5) reference only
*higher* slots, so they all read OLD values = clean explicit Euler.

**5.2 The moment's lever arm must match the body position where the forces were
evaluated.** Forces read at pseudoStep 0 of step N were converged on the mesh at
step N-1's position. The pitch-rate integrator `state[5]` originally translated
the moment with `state[0,1]` (the world CG). But `state[0,1]` sit at slots 0,1 —
updated *before* slot 5 — so they were already the NEW (one-step-ahead) position.
The lever arm led the forces by one step (~1 m at cruise), injecting a small
systematic moment error that integrated over 200 steps into the **wrong sign** —
the artificial divergent dive.

**Fix:** compute the lever-arm CG by **forward IK of `state[6], state[7]`** (the
link angles) inside `update_law`. Those slots are at 6,7 > 5, so they're still
OLD when `state[5]` integrates, and OLD link angles = the body position where the
forces were evaluated. This is **not** an algebraic no-op vs using `state[0,1]` —
it shifts the lever arm back exactly one step.

```
# lever-arm CG via forward IK on OLD link-angle slots (L1=L2=L):
x_cg = cos(state[7])*(L1 - L2*cos(state[6])) + sin(state[7])*L2*sin(state[6])
z_cg = -sin(state[7])*(L1 - L2*cos(state[6])) + cos(state[7])*L2*sin(state[6])
M_y_about_CG = momentY + (mc_z - z_cg)*forceX - (mc_x - x_cg)*forceZ - prop_z*T_disk
```

**Verified:** identical no-kick start, dive → clean phugoid (case-84ed883f). The
applied moment `Δstate5/dt·I_yy` matches an independent wing+htail+disk
reconstruction (with the OLD-state[6,7] lever arm) to 4 decimals; the 60-s chain
gives T = 17.4 s, ζ = 0.026.

**5.3 Diagnostics trap from the same fact.** A moment *cache* in a HIGH slot
(8,9) reads the already-updated (NEW, lag-wrong) `state[6,7]` and disagrees with
what the dynamics in slot 5 actually applied. The reliable measure of the applied
angular acceleration is `(state5[N] − state5[N-1]) / dt`, recovered in post from
the published `state[5]` trajectory — never trust a high-slot moment cache.

**5.4 Catastrophic cancellation (real hazard, mitigate it).**
`M_about_CG = momentY + (mc_z−z)·forceX − (mc_x−x)·forceZ`. Once the body
translates far from a fixed `moment_center`, `momentY` and the translation term
each grow large and cancel to a small residual (≈99.9% cancellation in our case),
which then sits near the force-integration precision floor. Mitigations: set
`moment_center` to the **initial CG** (lever arm starts at zero); and read the
dynamics from `Δstate5/dt`, not from an externally re-derived moment (both
re-derivations are equally cancellation-limited and will mislead you — they did).

**5.5 You MUST publish `omega` (and `omegaDot`) alongside `theta`.** A separate,
earlier-found, still-valid constraint. The three outputs are independent; the
solver does **not** finite-difference `theta` to get `omega`. Publish only
`theta` and `omega` defaults to 0 → the rigid-body wall BC has zero velocity →
the mesh teleports to each new angle but the CFD never "sees" the airframe moving
through the freestream → forces are silently, disastrously wrong. A/B test
(2 s, 40 steps): RMS error vs the simplistic-aero prototype was **0.0815 (theta
only)** vs **0.0096 (theta+omega+omegaDot)** — 8.5× better; at step 25,
no-omega CL=1.49 vs with-omega CL=0.713 (simplistic 0.710). Derive `omega`
analytically by chain rule from the same state your `update_law` integrates; for
our 2-link arm (`x,z,Vx,Vz = state[0..3]`, `r²=x²+z²`):
```
ω_elbow       = (x·Vx + z·Vz) / (L1·L2·sin(state[6]))
ω_shoulder    = -ω_elbow/2  - (x·Vz - z·Vx)/r²
ω_airframe_rel = state[5]    - ω_elbow/2 + (x·Vz - z·Vx)/r²
ω_htail_rel   = 0
```
`omegaDot = 0` is acceptable for first-order schemes (informational; the solver
runs fine with it zero). Do **not** route a nonzero diagnostic into a cylinder's
`omegaDot` if you care about the moving-mesh BC.

---

## 6. Other genuine constraints we hit (read before authoring)

- **All UDD I/O is solver NON-DIM, not SI, not coefficients.** With `L_ref=1 m`,
  `V_ref=a_inf`, `ρ_ref=ρ_inf`: `timeStepSize_nd = dt_si·a_inf`;
  `F_nd = F_si/(ρ_inf·a_inf²·L_ref²)`; `M_nd = M_si/(ρ_inf·a_inf²·L_ref³)`;
  `x_nd = x_si` (L=1); `v_nd = v_si/a_inf`; `m_nd = m_si/ρ_inf`;
  `I_nd = I_si/ρ_inf`; `g_nd = 9.81/a_inf²`. Newton's law is then just
  `state_dot = F_nd/m_nd` — the qS factor is already baked into the non-dim force.
  Putting SI numbers next to `timeStepSize` makes every Euler step ~a_inf× too
  long → blow-up in ~20 steps. (Empirically: dt=0.05 s at h=3658 m gave
  `timeStepSize=16.298 = 0.05·a_inf`.)
- **Forces/moments are WORLD (inertial) frame**, at `ReferenceGeometry.moment_center`
  — NOT the outermost rotating cylinder's frame. Verified at the frozen trim pose:
  `momentY + x_cg·forceZ − z_cg·forceX ≈ 0` (trim) with the world-frame CG; the
  cyl-frame formula gave a wildly off-trim residual. (But translate to the CG with
  the lag-correct lever arm — see §5.2.)
- **One UDD ↔ one `output_target`.** Zone-NAME output keys
  (`zone_cyl1_shoulder_theta`) are rejected ("not defined as a scalar"); the
  registry uses numeric zone IDs assigned at mesh time. Pattern: N identical UDDs,
  one per zone, lockstep. **Verify lockstep**: the per-cylinder ω expressions
  should sum to the body rate to ~1e-5.
- **`atan2` is unsupported** ("atan2 is not defined as a scalar"). Use
  `atan(y/x)` with a denominator guaranteed positive over the trajectory (our
  L-pose IK keeps `L1−L2·cosθ₂ > 0` and `x_cg > 0`).
- **Expression DSL, learned empirically:** `atan, acos, sin, cos, sqrt` (one-arg)
  work; the `if (c) … ; else … ;` form is accepted (translated to a ternary
  `c ? a : b`); multi-arg comma functions appear unsupported. Custom
  `output_vars` names are rejected by the solver — only the whitelist
  (`theta/omega/omegaDot/alphaAngle/betaAngle/bet_*_omega/actuatorDisk_*`) is
  honored, so the only safe diagnostic channel is `omegaDot` (one per UDD).
- **Actuator-disk reaction is a volume body force, NOT in the wall integral.** Add
  the disk reaction moment by hand: a disk at body `(PROP_X,0,PROP_Z)` with thrust
  `(−T,0,0)` contributes `(r×F)_y = −PROP_Z·T` about the CG. Verify the disk
  model's *actual* integrated thrust equals the constant you assume.
- **Output frequency:** `SliceOutput`/`VolumeOutput` default `frequency = -1`
  (last step only). For a flow animation set `frequency = 1` (per step). A
  single-frame VolumeOutput is ~2 GB and useless for animation; the y=0 slice is
  ~10 MB/frame. The y=0 plane is the body symmetry plane (motion is confined to
  x-z), so it always cuts the centerline.

---

## 7. Recipe and validation pattern

**Authoring recipe:**
1. Derive the dynamics in solver non-dim up front (convert m, I, g, v).
2. Confirm forces are world-frame at a frozen-trim warmup
   (`momentY + r_CG_world × F_world ≈ 0` at trim).
3. Author one UDD per rotation zone, each publishing `theta`+`omega`+`omegaDot`,
   gated at `pseudoStep == 0`.
4. Put the moment's lever arm on *higher-index, still-OLD* link-angle slots
   (forward IK), not the integrated CG.
5. Rewrite `atan2(y,x)` → `atan(y/x)` where the denominator stays positive.
6. Fork unsteady chunks from a converged frozen-pose warmup.

**Validation (cheap, decisive):**
- **Step 0:** forward-Euler a Python prototype of the dynamics with the same ICs,
  dt, and a simplistic aero model — baseline trajectory.
- **2 s A/B:** fork two 40-step cases from the warmup (with/without the change);
  compare per-step `forceZ/forceX/momentY` and back-derived `θ_body/x_cg/z_cg`
  against the prototype. A factor-N drop in RMS error confirms the change.
- **First real test: γ_kick = 0 (start at trim).** A correct rig sits still or
  executes a clean damped phugoid. A monotonic divergence from trim is almost
  always a timing/sign bug, not physics.
- **Instrument via `omegaDot`** (one diagnostic per UDD); recover applied moment
  as `Δstate5/dt·I_yy`. Read the full per-step trace with
  `case.logs.set_remote_log_file_name("logs/flow360_case.user.log"); case.logs.print()`,
  bracketing on `"Physical step ="`.

---

## 8. Agent / engineer checklist (the blunt version)

1. Gate state updates at `pseudoStep == 0`, never `last_pseudo_step`. (§4)
2. `update_law` is sequential — order slots so references read OLD/NEW as
   intended; lag-correct quantities must live in higher slots. (§5.1)
3. Lever arm = forward IK of OLD link-angle slots, matching the force lag, not the
   integrated CG. (§5.2)
4. Publish `omega` and `omegaDot`, not just `theta`. (§5.5)
5. Everything non-dim; `moment_center` at the initial CG; read dynamics from
   `Δstate5/dt`. (§5.4, §6)
6. One UDD per cylinder; verify lockstep; `atan2`→`atan(y/x)`. (§6)
7. Add the actuator-disk reaction moment by hand. (§6)
8. y=0 `SliceOutput` with `frequency=1` if you want a flow movie. (§6)
9. First test at γ_kick=0; divergence-from-trim ⇒ suspect a bug, not the airframe. (§7)

---

## 9. Feedback for the Flexcompute Flow360 team (ranked by cost saved)

**A. Document the `update_law` evaluation model (highest impact, ~1 week saved).**
That `update_law` is evaluated *sequentially in slot order, in place* — with
OLD/NEW visibility set by slot index — is the single most expensive unknown and is
nowhere in the docs. Our code assumed batch semantics. State the order, the
OLD-vs-NEW rule, and its stability across solver versions.

**B. Document the `previousTheta_recorder` + `pseudoStep==0` contract.** Publishing
`theta` silently spawns a recorder UDD that snapshots at pseudoStep 0; a state
update gated at the last pseudo step therefore zeroes `deltaTheta` and freezes the
mesh. Either document the required gate or emit a validation warning when a
theta-publishing UDD updates its theta-state at a gate ≠ `pseudoStep==0`.

**C. Validate / warn when a Cylinder-targeted UDD writes `theta` without `omega`.**
The most expensive *silent* failure: case runs clean, trajectory looks plausible,
forces are wrong because the wall velocity defaulted to 0. Add a pydantic
validator (require `omega`, or an explicit `assume_quasi_static=True`) and a
one-line solver `[USERDBG]` warning.

**D. Document the force/moment FRAME and UNIT system for UDD I/O.** State
explicitly that inputs are world-frame at `moment_center` and in solver non-dim,
with an SI↔non-dim table. (A prior source-verified Q&A told us "outermost cylinder
frame" and "all `<cmath>` available" — both wrong against the deployed solver;
please sanity-test such answers against the current build before sending them.)

**E. First-class read-only custom output variables.** Today the only way to
inspect internal `state[i]` / computed moments is to hijack `omegaDot` (and even
that reads post-update values for high slots). Allow arbitrary *named scalar*
log-only outputs that don't have to be in the rotation whitelist (the solver
already evaluates the expression — just don't reject the name).

**F. A moving / body-frame `moment_center`.** Free-flight inevitably translates the
body far from any fixed center, turning the moment-about-CG into a tiny residual of
two large cancelling terms at the precision floor. A `moment_center` bindable to a
moving frame (e.g. a cylinder's local origin) eliminates this whole error class.

**G. Native 6-DOF / free-flight rigid-body motion (the big one).** We hand-rolled
6-DOF + a 2-link IK + four lockstep UDDs because there is no native free-flight
option. A supported "rigid body with mass/inertia, integrate from aero loads"
motion type — with the gate, lever-arm, force-lag, and omega all handled
internally and correctly — would turn this multi-week effort into a 1-day task.

**H. Smaller asks:** support `atan2`; publish the expression-DSL grammar /
supported-builtins table; differentiate the overloaded "X is not defined as a
scalar" error (unknown function vs wrong output key vs typo); add a
pre-submission translated-JSON dump (`dump_translated(params)`) to avoid the
submit→fail→read-`flow360.json` loop; cache `pseudoStep==0`-gated expressions to
cut the 100+ MB user logs and wall time; allow drop of the one-`output_target`
restriction (fan out internally); ship a multi-zone closed-loop UDD example
notebook; expose per-case compute/FlexCredit in the SDK (`CaseMeta` has none);
and report worst-cell location/zone for high mesh-aspect-ratio warnings.

---

## 10. Appendix — decisive cases & fixed source

- **Fixed source:** `flow360/unsteady_setup.py` — `pseudoStep==0` gate on all
  state integrators; forward-IK lever arm in the `state[5]` / `state[8,9]` moment
  terms; mandated y=0 `SliceOutput` with a `slice_frequency` knob; `diagnostics`
  flag routing `state[*]` to the four `omegaDot` channels. Shared geometry/trim in
  `flow360/cfd_setup.py`, `params.py`. Final phugoid submitters:
  `submit_unsteady_warmup.py`, `submit_gap110_nokick_diag.py`,
  `submit_gap110_chunkN_nokick_diag.py`. Post: `post/v3/plot_running_case.py`,
  `viz_nokick_lagfix.py`, `render_phugoid_slices.py`.
- **Case ledger:**
  - `case-c8cabf45` — gate fix proven (dCL/dα ≈ +0.07/° vs ~0.001/° buggy).
  - `case-36e2b351` — buggy lever arm (state[0,1]); no-kick dives +5.6→−35°. Artifact.
  - `case-84ed883f` — lag-fixed lever arm (forward-IK state[6,7]); no-kick climbs and
    recovers. Correct. Parent of the 60-s chain (chunks 2-6: case-6e0a729f,
    -d38c0480, -…, -8f1d6f90).
- **Result:** phugoid period **17.4 s**, damping ratio **ζ ≈ 0.026**, peaks
  +44.6°→+38.7°→+33.4°→+28.9°. See the trajectory PNG and the three MP4s listed at
  the top.
