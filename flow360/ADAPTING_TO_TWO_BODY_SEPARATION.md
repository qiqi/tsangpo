# Adapting the Tsangpo phugoid rig to a two-body stage separation

*Written 2026-10-30 for engineers who have read `PHUGOID_UDD_POSTMORTEM.md`
once. Prose with pointers; nothing here has been run on two bodies. Where we
say "verify", we mean it: Tsangpo is one body, planar, release-25.9.*

## 1. What carries over unchanged

Every rule in POSTMORTEM §8 applies verbatim. Tick them off, do not reread
them: gate at `pseudoStep == 0`; `update_law` is sequential, slot order is
load-bearing; lever arm from forward IK of the OLD link-angle slots; publish
`theta` + `omega` + `omegaDot`; all UDD I/O in solver non-dim with
`L_ref = 1 m`; forces are world-frame totals at `moment_center`; one
`output_target` per UDD, so N zones means N identical lockstep UDDs; at most
10 state slots; no `atan2`. The implementation to copy is
`_build_per_zone_udds` in `unsteady_setup.py`. Use the slot numbers from that
file's docstring, not from the postmortem: since 2026-05-28 the integrator is
symplectic Euler–Cromer with velocities in slots 0–2 and positions in 3–5, so
positions read the new velocities and the published `omega` equals the
discrete mesh displacement per step.

## 2. Topology: where the second body can live

Flow360 rotation zones (`fl.Rotation` on a `RotationVolume`) are right
circular cylinders, nested strictly, never intersecting, and they only rotate.
As of release-25.9 there is no translating zone and no overset; check the
25.12 notes before assuming otherwise. Translation therefore comes from the
two-link arm (`unsteady_setup.py` module docstring), and the arm's
shoulder cylinder encloses everything within about `L1 + L2` of the shoulder.

That single fact rules out the layout you would draw first, both bodies each
on their own arm, or one body on an arm with the other body sitting next to it
in the farfield: the fixed body would be inside the moving body's shoulder
cylinder. The one geometry that works with the bodies initially adjacent is:

```
farfield (fixed body A lives here, in the freestream frame)
└── cyl 1  shoulder, centre on the separation axis, AFT of body B
    └── cyl 2  elbow
        └── cyl 3  body B (the one that moves relative to A)
```

with the arm nearly straight along the separation axis at t = 0, body B at
its tip, and body A just forward of the shoulder cylinder's rim. Folding the
arm pulls B aft along the axis, which is the separation motion. The arm
cylinders protrude forward of B's nose by roughly `L·φ₀² + 2·gap + g`, where
`φ₀` is the initial bend of each link from the axis, `gap` is the interface
gap of §4 and `g` the clearance of the innermost cylinder around B. So the
simulation cannot start at the instant of release; it starts with A and B
already that far apart. For `L = 5` body lengths and `φ₀ = 10°` that is about
0.15 body lengths plus the gaps.

Two consequences to accept up front:

- **The outer sliding interface cuts through the interstage region.** The
  flow between the two bodies crosses cyl 1's boundary exactly where the
  plume and wake are. The gap rule of §4 then sets cyl 1's rim spacing from
  body A's local surface spacing, around the whole circumference.
- **The initial pose is near a kinematic singularity.** The Jacobian scales
  with `sin(2φ)`; at `φ₀ = 10°` the link rates are about three times the CG
  velocity over L. Tolerable, but the closed-form IK in `_ik_initial` and the
  `X_CG_IK` / `Z_CG_IK` strings assume the Tsangpo L-pose (`θ_link2 = π/2`).
  Re-derive them for the near-straight pose and make sure every `atan`
  denominator stays strictly positive over the full trajectory (POSTMORTEM §6).

Body A can be wrapped in its own pitch-only cylinder in the farfield, outside
cyl 1, at the cost of one more gap in the standoff. It cannot translate.

## 3. Sizing the arm

The arm radius sets the workspace, not the straightness; the IK is exact.
Pick `L1 = L2 = L` so that the CG of B stays inside the annulus
`|L1 − L2| < r < L1 + L2` with margin at both ends for the whole separation,
and so the elbow interior angle stays away from 0 and π. Tsangpo used
`L = 750 c_w` so a 10° kick phugoid fits with a 9× area margin
(`R_ARM_C` in `unsteady_setup.py` §1); a separation of a few body lengths
needs an L of a few body lengths, which is three orders of magnitude cheaper
than ours.

Mesh cost scales with the shoulder volume, `L³`. We tripled L and kept the
cell count fixed by coarsening the outer two zones 3× (the
`spacing_axial/radial/circumferential` values on `cyl1_zone` and `cyl2_zone`,
with the arithmetic in the comment). Do the same bookkeeping.

## 4. The interface-gap rule

Match condition at each cylinder's outer boundary, from `unsteady_setup.py`
§1: the volume mesh size there is `min over inner surfaces of
(0.1·distance + surface spacing)`, so with surface spacing `s_N` on the
cylinder's boundary and `s_{N−1}` on the next-inner one,
`gap ≈ (s_N − s_{N−1}) / 0.1`. Tsangpo: 0.11 → 11 → 21.7 chords gives two
gaps of 110 chords. Skipping this produces a resolution jump at the sliding
interface that looks like a physics problem and is not (POSTMORTEM §2,
Symptom A, second misdiagnosis). Do the arithmetic for your three spacings
before you build the geometry, because the gaps set the cylinder radii.

## 5. Forces the UDD does not see

Two separate issues.

**Body forces are not in the wall integral.** Actuator disks, and anything
you model as a momentum source (an engine plume, a separation motor), must be
added by hand in the update law in body axes and rotated to world. The
pattern is the `T_disk_nd` terms in the phugoid `update_law`
(`unsteady_setup.py`, search `T_disk_nd`), including the hand-added reaction
moment `−prop_z · T` about the CG (POSTMORTEM §6).

**The force inputs are totals over every wall in the case.** With two bodies
in one case `forceX`, `forceZ`, `momentY` are `F_A + F_B`. We never had to
split them. Before writing a single expression, ask the Flow360 team whether
25.12 exposes per-boundary force inputs to UDDs. If not, the workaround is to
treat A's motion as prescribed (it is the thrusting stage, so its aero is
small next to thrust) and subtract a precomputed `F_A` from a steady case of A
alone. The relative frame is then non-inertial, so subtract A's acceleration
in B's equations of motion.

## 6. Moment centre as the bodies separate

POSTMORTEM §5.4 applies with more force. `ReferenceGeometry` has one
`moment_center` per case. Put it at B's initial CG so the lever arm starts at
zero and grows with the separation distance; body A's moment then carries the
full cancellation error, which is acceptable if A's attitude is prescribed
(§5). Read the pitch dynamics from `Δstate/dt`, never from a re-derived
moment.

## 7. Solver version

See `LESSONS.md`, "Solver version & rotation-axis caveat". Every Tsangpo axis
is `(0, 1, 0)`. The moment you nest a roll or yaw rotation inside the pitch
chain, which a spin-stabilised stage or any out-of-plane separation does,
FXC-11747 applies: run on Flow360 25.12 or later and pin the version in the
project. Do not start on 25.9.

## 8. Validation ladder

Same ladder as POSTMORTEM §7, with the zero-kick test replaced, since a
separation has no trim state.

1. **Frozen-pose warmup** (`mode='warmup'`, `submit_unsteady_warmup.py`) at
   the initial standoff. This is also where you confirm the forces are
   world-frame at `moment_center`, as in §7 step 2 of the postmortem.
2. **Python prototype.** Forward-Euler the rigid-body equations with the same
   initial conditions, dt and a crude aero model. Ours was a few dozen lines
   and is not in this repo.
3. **Two-second A/B** forked from the warmup, against the prototype, per step.
4. **Infinite-mass hold.** Set `m_nd` and `I_nd` absurdly large. B must not
   move. Any drift is an IK or slot-order bug.
5. **Ballistic check.** Multiply the aero inputs by zero. B must follow the
   analytic relative parabola. This exercises the gate, the non-dim
   conversions and the arm kinematics with no CFD in the loop.
6. Only then release with real forces, in 200-step chunks forked from the
   previous chunk (`submit_gap110_nokick_diag.py`, then
   `submit_gap110_chunkN_nokick_diag.py`), reading the per-step trace from
   the user log as described in POSTMORTEM §7.

## 9. What this repo does not cover

Contact, latch or release logic; per-body force integration; two moving
bodies in one case; compressible or high-Mach conditions; anything about
25.12 behaviour. All our data are subsonic, one body, release-25.9. We state
this so you do not go looking.
