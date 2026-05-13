# Flow360 LESSONS — Tsangpo

Running log of Flow360 quirks paid for in case-credit on this project.
Each lesson cites the caseId(s) that proved or disproved it.
Trim entries when Flow360 ships a fix; do not leave "no longer needed" comments.

Last reviewed: 2026-05-13 — SDK v25.9.x.

## Gotchas (silently wrong)

- **`fl.Steady` + `Rotation(spec=AngleExpression)` is a no-op.** Verified by
  three forks at θ_ac ∈ {−7, −1, +0.5} (cases-0b6a7162, -2647042c,
  -83caa967) all returning identical CL = 0.2428. Use `fl.Unsteady`
  whenever a `Rotation` model is in play. Step size ≫ chord/V is fine
  (we use dt = 1 s with chord/V ≈ 0.03 s — each step converges to
  quasi-steady).

- **Active vs passive rotation convention.** In unsteady, the body is
  actually rotated; freestream stays at α = 0. **+θ_ac = nose UP.**
  Initial (wrong) assumption was passive convention (−θ_ac = nose up).
  Case-f4a71962 with θ_ac = −0.122 gave main_wing CL = −0.18 (downforce)
  — fix: flip sign.

- **`AngleExpression` parser is restricted.** Accepts `t` (physical
  time) and standard math/trig functions of `t`. Does **not** accept
  `physicalStep`, `pseudoStep`, or solver-state variables. For
  piecewise-in-step behaviour use a smooth function of `t` (sin, tanh,
  or a linear ramp evaluated at integer step times).

- **`ActuatorDisk` / `ForcePerArea` does not accept expressions.** Only
  fixed numpy arrays. Time-varying thrust must go through a
  `UserDefinedDynamic` writing `actuatorDisk_<name>_thrustMultiplier`.

- **`UserDefinedDynamic` without `input_boundary_patches=` fails with
  "CL requires a proper source".** Pass the wall surfaces even if the
  UDD doesn't read CL — the validator wants the binding.

- **`timeStepSize=inf` (the `fl.Steady` default) propagates into UDD
  gain expressions** and makes any `state[0] += gain · err` term blow
  up to ±∞ silently. Drop `timeStepSize` from the update law in
  steady runs, or run unsteady.

- **Double-enclosure zeroes wall forces.** If a surface appears in
  `enclosed_entities` of *two* nested `RotationVolume`s, the solver
  returns wall forces exactly 0.0000 on it. Keep each surface only in
  the innermost rotation volume.

- **`enclosed_entities` rejects sliding-interface children.** Putting
  prop cylinders into `ac_rotation.enclosed_entities` triggered
  sliding-interface error 7221 (case-e768de32). Only put STATIC
  geometry in `enclosed_entities`; nested rotating volumes go in via
  `Rotation(..., parent_volume=outer_cyl)`.

- **Cylinder `name` has an 18-char limit.** `"htail_rotation_zone"` (19)
  is rejected. Trim to ≤ 18.

- **Body-dump order is NOT a stable face-naming hook.** Reordering
  `dump` statements in the .csm renames `body0000N` and breaks every
  surface lookup downstream. Always use
  `select face / attribute capsGroup $<name>` per face, then
  `geo.group_faces_by_tag("capsGroup")` on the Flow360 side.

- **ESP body-level `_name` does not propagate to faces.** The face
  tag must be on faces (via `select face / attribute capsGroup`), not
  on the body via `attribute _name`. Body `_name` is for ESP display
  only.

- **Combined-STEP upload meshes only `body00001`.** Case-baa2eb43:
  upload of `airframe.step` (merged via `from_geometry([list])`) gave
  a mesh of just the wing. Workaround: **inline UDCs into a single
  `.csm` file and upload the .csm** (see `submit_cruise.py` :
  `inline_udcs`).

- **`MeshingDefaults.curvature_resolution_angle` is silently ignored
  by the legacy mesher.** Only the beta mesher honors it; the legacy
  mesher also produces a poor LE on spline contours. **Always pass
  `use_beta_mesher=True`** (codified in CLAUDE.md).

- **`Case.fork()` rejects v2 `SimulationParams`.** Use
  `project.run_case(fork_from=parent_case, params=v2params, ...)`
  instead of the `case.fork()` method.

- **`Rotation`-model volume must also be in `meshing.volume_zones`.**
  Otherwise the validator reports "Volume zone X does not exist" even
  though the same `Cylinder` instance is referenced from the Rotation
  model. The Cylinder defines the geometry; the `RotationVolume`
  entry in `meshing.volume_zones` defines the mesh refinement.

## Mandatory settings (codified in CLAUDE.md)

- `use_beta_mesher=True` on every `run_case`.
- `attribute capsGroup $<name>` on every face in the .csm.
- One STEP per body in the .csm (or inline UDCs and ship one .csm).
- Inline UDCs into the .csm before upload — multi-file uploads are
  unreliable.
- For our scale (chord ≈ 1.4 m, V ≈ 46 m/s): first-layer thickness
  ≈ 7.62 × 10⁻⁶ m to keep y⁺ near 1.

## Validated patterns

- **Nested rotation volumes**: outer `ac_pitch_zone` (whole-aircraft
  pitch), inner `htail_pitch_zone` (htail-only pitch) linked via
  `fl.Rotation(name="htail_pitch", spec=..., parent_volume=ac_pitch_cyl)`.
  `enclosed_entities` only at the innermost rotation; outer rotation
  encloses the static wing surfaces plus the inner cylinder geometry.

- **Actuator disk thickening + refinement.** Disk thickness = 0.10 c,
  with `fl.UniformRefinement(entities=prop_cyls, spacing=0.05*MAC)` —
  fixes the ~60 % over-thrust caused by under-resolved actuator-disk
  zones.

- **Reference geometry vector lengths.** `moment_length = (b/2, c, b/2)`
  so roll & yaw normalize by semi-span and pitch normalizes by chord.

- **Unsteady sweep as quasi-steady ladder.** `fl.Unsteady(step_size=1
  s, steps=N, max_pseudo_steps=1000, CFL=AdaptiveCFL(max=1e4,
  convergence_limiting_factor=0.25))`. dt ≫ chord/V means each
  physical step is a steady-state solve at whatever boundary
  condition is active during that step — pack a parameter sweep into
  one case by varying the BC step-by-step (linear `AngleExpression`
  ramp for rotation, UDD multiplier piecewise on `physicalStep` for
  thrust).

- **`Unsteady` forks won't run more than 1 new physical step in
  practice.**  Both `steps=N_parent` and `steps=N_parent + N_new`
  produced exactly one new physical step in our experiments
  (cases-0cd529ae, -5d32e34a, -1a8bc5cc, -c84507d1).  The
  case is then marked ERROR.  Whether this is because `steps` has
  a non-obvious semantic in forks, or because the body's instant
  rotation jump between parent's last angle and the fork's first
  AngleExpression value blew up the mesh, was never pinned down.
  **Practical pattern**: one fork per swept value, with a CONSTANT
  `AngleExpression`/`force_per_area` (no `t`-dependence).  Each fork
  gets a few new physical steps to settle on the new condition.
  This is what `submit_alpha_sweep.py` / `_htail_sweep.py` /
  `_thrust_sweep.py` currently do.

- **AngleExpression(t) in forks is unreliable.**  When the fork's
  `AngleExpression` depends on `t`, the body's actual rotation
  doesn't match what the formula predicts at any plausible `t`
  (neither cumulative-from-parent nor reset-to-zero made the
  observed CL/CD self-consistent across v1 and v2 of the alpha
  fork).  The numbers come out as if the body landed in deep stall
  — `±15°+` regardless of intended angle.  Until Flow360 clarifies
  the semantics, stick to constant `AngleExpression` per fork.

- **UDD `update_law` strings are pure expressions, NOT assignment
  statements.** Each list entry evaluates to the new value of
  `state[i]`; writing `"... state[0] = <expr>; else state[0];"` is a
  syntax error that errors the case *silently* before any output
  files are written (verified by case-c8308ae4: ERROR status, no
  total_forces / nonlinear_residuals CSV).  Correct form:
  `"if (pseudoStep == 0) <expr>; else state[0];"` — the `if/else`
  branches are themselves expressions, not statements.

- **UDD targeting `actuatorDisk_*_thrustMultiplier` rejected in
  forks even with corrected syntax.** Case-72dc517a (a fork that
  declared `output_vars={"actuatorDisk_prop_R1_thrustMultiplier":
  "state[0];", …}` for all 10 disks with a syntactically valid
  update law) still errored before producing any output. Fallback:
  one fork per swept thrust value with `force_per_area` hardcoded
  per case — no UDD needed.

- **UDD `physicalStep` and `pseudoStep` ARE accessible inside
  `update_law`** (unlike inside `AngleExpression`). So piecewise
  control of an actuator-disk thrust multiplier can be expressed as
  a ternary on `physicalStep` directly — no `t`-conversion needed.

- **GAI volume mesher rejects `enclosed_entities` referenced by their
  original capsGroup name when the surface lives inside nested
  rotation zones.** Volume mesh `vm-2d091b4e` (parent
  `case-a8634829`, GAI cruise trim campaign) errored with
  `(ERROR 7221) Encountered object name: htail in enclosed objects
  for sliding interface: slidingInterface-htail_pitch_zone, which is
  not a known volume entity.` The GAI surface mesher had renamed the
  patch to `htail__rotating_ac_pitch_zone__rotating_htail_pitch_zone`
  (zone-hierarchy suffix); the volume mesher then can't match
  `htail` from `Rotation(...).enclosed_entities`. The identical
  `SimulationParams` builds cleanly under the legacy beta mesher (which
  apparently keeps an internal alias).  All 30 forks downstream of
  the failed parent also errored.

  Workaround: skip `use_geometry_AI=True` for cases that have nested
  rotation volumes referencing surfaces inside them. The trim-centered
  cruise campaign was re-submitted as a non-GAI run
  (`submit_cruise_trim_campaign.py`).

## Open / unverified

- Does the AngleExpression parser accept C-style ternary `?:`? Not
  empirically tested — the linear-ramp pattern dodges the question.
