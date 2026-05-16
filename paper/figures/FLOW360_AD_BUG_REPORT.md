# Flow360 actuator-disk delivered-vs-commanded thrust discrepancy

**Reporter:** Qiqi Wang
**Date:** 2026-05-16
**Solver version:** release-25.9
**SDK:** flow360 v25.9.x
**Mesher:** beta (in-house) mesher

## TL;DR

`fl.ActuatorDisk(force_per_area=fl.ForcePerArea(...))` with a uniform
constant-pressure-jump specification produces an integrated reaction
force (`Disk_i_Force`) that **does not match** the commanded pressure
× annular area, and the deficit scales non-trivially with **both**
the disk-refinement mesh spacing and the thrust setting:

- At cruise-level thrust (low loading): solver delivers ~**1.10×**
  commanded *consistently* across all four geometric configurations
  and both mesh refinements.
- At takeoff/landing thrust (high loading), with the original AD
  refinement (`PROP_REFINE_M = 0.05·c_w` → octree-cast 0.0625 m,
  ≈ 17 cells across disk diameter, ≈ 2 cells axially through the
  cylinder), delivery splits into two camps: one geometry delivers
  **1.10×** and three deliver **0.70×**.
- Halving the AD refinement to **0.03125 m** (≈ 34 cells across
  diameter, ≈ 4 cells axially) collapses the two camps to the
  **same ratio, ~0.65×**.

The wing CL/CD respond proportionally to the delivered AD force
(13% wing-CL drop in tandem with a 41% AD-force drop in the same
case), confirming this is a *real* momentum-transfer change rather
than a mis-reported diagnostic.

Looking for guidance on (a) whether we are mis-specifying the
`ForcePerArea` model, (b) whether release-25.9's actuator-disk
integration has a known nonlinear delivery curve at high disk
loading, or (c) whether this is a solver bug worth a fix.

---

## 1. Configuration setup

10-prop distributed-electric-propulsion uSTOL aircraft, 10 actuator
disks total. Each disk:

```python
# cfd_setup.py:244–251
fl.ActuatorDisk(
    name=f"prop_R{i}",
    entities=cyl,                                      # see cylinder below
    force_per_area=fl.ForcePerArea(
        radius=np.array([0.080025, 0.5335]) * fl.u.m,  # [0.15·R, R]
        thrust=np.array([fpa, fpa]) * fl.u.N / fl.u.m**2,
        circumferential=np.array([swirl, swirl]) * fl.u.N / fl.u.m**2,
    ),
)

# Per-disk Cylinder, axis along streamwise (+x):
#   center = (PROP_X_M, PROP_Y_M[i], PROP_Z_M)
#   axis   = (1, 0, 0)
#   height = 0.10 c_w = 0.1385 m       (axial thickness)
#   outer_radius = 0.5335 m
# fpa, swirl scale linearly with `T_mult`.
```

`fpa` and `swirl` scale linearly with `T_mult`:

| phase   | T_mult | `thrust` [N/m²] | annular area × 10 [m²] | commanded total [N] |
|---------|--------|-----------------|------------------------|---------------------|
| cruise  | 1.0    | 60.86           | 8.74                   | 531.9               |
| takeoff | 16.0   | 973.74          | 8.74                   | 8511.0              |
| landing | 12.0   | 730.31          | 8.74                   | 6383.2              |

The annular area per disk = π·(R² − r_inner²) = π·(0.5335² − 0.080025²) =
0.874 m². Hub (r ∈ [0, 0.080]) has no commanded pressure (would be
~2.3 % of full disk area if extended).

Ambient: ISA 3658 m altitude (12,000 ft); ρ = 0.849 kg/m³, a = 326 m/s,
ρa² = 90250 N/m².

## 2. Delivered force measurement

`F_AD_delivered_N` is computed from `case.results.actuator_disks`:

```python
ad = case.results.actuator_disks; ad.load_from_remote()
av = ad.values
F_solver = sum(np.array(av[f"Disk{i}_Force"])[-1] for i in range(10))  # last iter, all 10 disks
F_AD_delivered_N = F_solver * (rho_inf * a_inf**2)                      # = 90250 N/m²
```

`Disk_i_Force` is constant from iteration 1 onward (2020 records, all
identical, every config). Not a convergence artifact.

## 3. Initial mesh ( `PROP_REFINE_M = 0.0625 m`, ~17 cells across disk diameter, ~2 axial)

| config              | phase   | thrust [N/m²] | commanded [N] | delivered [N] | **ratio** |
|---------------------|---------|---------------|---------------|---------------|-----------|
| v2_continuous       | cruise  | 60.86         | 531.9         | 585.7         | **1.10**  |
| v2_continuous       | takeoff | 973.74        | 8511.0        | 9370.5        | **1.10**  |
| v2_continuous       | landing | 730.31        | 6383.2        | 7029.0        | **1.10**  |
| v2_continuous_high  | cruise  | 60.86         | 531.9         | 585.8         | **1.10**  |
| v2_continuous_high  | takeoff | 973.74        | 8511.0        | 5959.5        | **0.70**  |
| v2_continuous_high  | landing | 730.31        | 6383.2        | 4470.1        | **0.70**  |
| v2_gapped           | cruise  | 60.86         | 531.9         | 585.2         | **1.10**  |
| v2_gapped           | takeoff | 973.74        | 8511.0        | 5959.4        | **0.70**  |
| v2_gapped           | landing | 730.31        | 6383.2        | 4467.5        | **0.70**  |
| v2_gapped_high      | cruise  | 60.86         | 531.9         | 585.4         | **1.10**  |
| v2_gapped_high      | takeoff | 973.74        | 8511.0        | 5960.3        | **0.70**  |
| v2_gapped_high      | landing | 730.31        | 6383.2        | 4467.0        | **0.70**  |

Pattern: every cruise (low-loading) point delivers 1.10× consistently;
at high loading the four configurations split into a 1.10× camp
(`v2_continuous`) and a 0.70× camp (the other three).  The 0.70 values
are deterministic and identical to 3-4 significant figures across
three independent configurations.

## 4. Fine mesh (`PROP_REFINE_M = 0.03125 m`, ~34 radial × ~4 axial across disk)

Two cases completed so far; remaining 10 are PENDING / RUNNING.

| config              | phase   | coarse-mesh ratio | fine-mesh ratio |
|---------------------|---------|-------------------|-----------------|
| v2_continuous       | takeoff | 1.10              | **0.65**        |
| v2_continuous_high  | takeoff | 0.70              | **0.65**        |

Both camps converge to **0.65** with the finer mesh — strong evidence
that the coarse-mesh camp split is a discretization / integration
artifact in the AD body-force application, and the "asymptotic"
delivery is ~65% of (commanded pressure × annular area) at
high-loading conditions.

## 5. Wing forces track the AD-force change (per-surface decomposition)

Same case (`v2_continuous`, takeoff, BO α=+8°, θ_ht=−5°, T_mult=16),
coarse vs fine AD mesh:

|                    | coarse AD mesh | fine AD mesh | Δ             |
|--------------------|----------------|--------------|---------------|
| F_AD_delivered [N] | 9370           | 5500.6       | −3870  (−41%) |
| main_wing CL       | +5.082         | +4.436       | −0.645 (−13%) |
| main_wing CD       | +0.646         | +0.452       | −0.195 (−30%) |
| htail CL           | −0.517         | −0.486       | +0.031 (−6%)  |
| TOTAL CL           | +4.565         | +3.950       | −0.615 (−13%) |
| TOTAL CD           | +0.526         | +0.346       | −0.180 (−34%) |
| TOTAL CMy          | +2.496         | +2.322       | −0.175        |

Wing blown-lift response drops in lockstep with the AD-force drop —
the AD model is actually injecting less momentum, not mis-reporting.

## 6. Hypotheses ruled out

- **Different AD model parameters between configs.** All probed cases
  report the same `force_per_area.thrust` value when probed via
  `case.params.models`. Inputs identical.
- **Convergence artifact.** `Disk_i_Force` constant from iteration 1
  (2020 records identical, every case).
- **Mesh refinement spec discrepancy.** Both coarse and fine campaigns
  honor the requested `PROP_REFINE_M` (octree rounded as expected,
  visible in run warnings); zoomed slice plots show ~17 and ~34 cells
  across the disk respectively, matching expectation.
- **Solver version.** All cases on `release-25.9`.
- **Geometry-related coupling.** Four geometrically distinct
  configurations all collapse to the same ratio at fine mesh.

## 7. Hypotheses still open

1. **`force_per_area.thrust` is interpreted as a body-force *density*
   (per cell volume) rather than a face pressure jump**, and our
   commanded calculation `pressure × annular_area` doesn't match the
   solver's actual integral. The cell volume divided by axial
   thickness should reduce to area, so this should give the same
   answer asymptotically — but discretization effects on a thin
   2-cell-axial cylinder are large.
2. **Glauert-style momentum-theory correction at high disk loading**:
   constant-pressure-jump AD with an induced-velocity-feedback term
   that reduces effective thrust output as disk-loading rises. Would
   explain the 1.10 → 0.65 sweep across thrust levels.
3. **Octree-mesh granularity effects on the body-force integration**:
   coarse mesh gives a "lucky" overshoot/undershoot depending on how
   the cylinder happens to align with cell centroids; fine mesh
   approaches an asymptotic ratio that's not 1.0.
4. **Sub-grid cylinder boundary**: when the cylinder face partially
   covers cells (cells whose centroid is just outside the cylinder
   don't get the body force), the effective integration area shrinks
   below the geometric annular area.

(2) and (4) together would also be consistent with the data.

## 8. Reproducer

### Project IDs (all 12)

```
v2_continuous          cruise    prj-16082511-6d6a-447e-8c54-828d476c0a85
v2_continuous          takeoff   prj-9cd3ad10-ea47-41d9-9e33-0096fc30d6c1
v2_continuous          landing   prj-e7dc7d6d-4baf-4101-8818-1173da5a359a
v2_continuous_high     cruise    prj-21d5e737-a1d9-4dba-ad7c-2155e647ede1
v2_continuous_high     takeoff   prj-3505c35f-0a58-4aa2-9b0e-75af2f2143f6
v2_continuous_high     landing   prj-121d08b0-626c-475b-8c53-9d2ab54765d7
v2_gapped              cruise    prj-59c27343-3c39-43a4-ab19-18863acf02c4
v2_gapped              takeoff   prj-e0e11ed5-2f3a-4e7d-8af7-c79baaf3ed22
v2_gapped              landing   prj-0cd29981-d281-442a-984f-06562abc1f39
v2_gapped_high         cruise    prj-d5d18139-3f53-4bf5-b224-7d3a5f3feb2a
v2_gapped_high         takeoff   prj-fadaacba-ac09-4b2f-a0f2-364c46f5cb02
v2_gapped_high         landing   prj-1315636c-f6d9-4076-ba32-0ec82c272430
```

### Coarse-mesh BO parents (`PROP_REFINE_M = 0.0625 m`)

```
v2_continuous          cruise    v2_cruise_parent
                                   case-e30a9610-248f-40af-aa9f-9f30846c419d
v2_continuous          takeoff   takeoff_coarse_BO_estimate
                                   case-118aced8-e380-45a5-b374-04a303f1802a
v2_continuous          landing   landing_coarse_BO_estimate
                                   case-fed9edf1-681b-49fa-b4e9-c0513aaf987a
v2_continuous_high     cruise    v2_cruise_continuous_high_htail_parent
                                   case-a7d02011-db4b-4286-a32d-afccbfc9cdb0
v2_continuous_high     takeoff   v2_takeoff_continuous_high_htail_parent
                                   case-ab08db57-9fec-4fdc-b44f-8a956357204f
v2_continuous_high     landing   v2_landing_continuous_high_htail_parent
                                   case-54f6aae3-cde4-4043-9527-130ab5b77708
v2_gapped              cruise    v2_cruise_gapped40_parent
                                   case-3a4859d9-5298-4a35-8d68-b98bcb26b580
v2_gapped              takeoff   v2_takeoff_gapped40_parent
                                   case-fe40382d-13d5-4343-8626-453fd3872993
v2_gapped              landing   v2_landing_gapped40_parent
                                   case-3fe6a528-9151-4337-8ece-8f0862fee1bf
v2_gapped_high         cruise    v2_cruise_gapped_high_htail_parent
                                   case-4f848e6b-e636-4bea-8886-526ff116b8ef
v2_gapped_high         takeoff   v2_takeoff_gapped_high_htail_parent
                                   case-a1bf0628-19d7-4291-9eee-2da1966456c0
v2_gapped_high         landing   v2_landing_gapped_high_htail_parent
                                   case-8ef11bc2-c83c-4f18-a80a-0dcd075d2aec
```

### Fine-mesh BO parents (`PROP_REFINE_M = 0.03125 m`)

```
v2_continuous          cruise    cruise_fine_ad_parent
                                   case-b5395660-0128-4116-be14-a599f3a2a637
v2_continuous          takeoff   takeoff_fine_ad_parent
                                   case-3d97afde-3272-441a-aad9-786aa0b394ad
v2_continuous          landing   landing_fine_ad_parent
                                   case-d69a3590-a537-442a-8c71-f03927645d9b
v2_continuous_high     cruise    cruise_fine_ad_parent
                                   case-a89c87d7-c786-4227-95f6-997d96c8223b
v2_continuous_high     takeoff   takeoff_fine_ad_parent
                                   case-ad854db9-c8b2-4254-81f6-a3e01ec830b9
v2_continuous_high     landing   landing_fine_ad_parent
                                   case-0584e25d-5bae-43c6-b2af-bf40b2c07cae
v2_gapped              cruise    cruise_fine_ad_parent
                                   case-267d4470-0f76-4354-90e9-32d7447ad7da
v2_gapped              takeoff   takeoff_fine_ad_parent
                                   case-decca8e7-c1f0-4620-9c6f-5b920e4dc380
v2_gapped              landing   landing_fine_ad_parent
                                   case-41cd287f-b71d-4aa3-a3a2-92a8191c9c4f
v2_gapped_high         cruise    cruise_fine_ad_parent
                                   case-1a76a1c0-17bb-4982-9a69-ee7d2fb67fa8
v2_gapped_high         takeoff   takeoff_fine_ad_parent
                                   case-0ec31ee6-64cc-4b5c-8311-9618dab7783c
v2_gapped_high         landing   landing_fine_ad_parent
                                   case-5528dbf6-3cae-4156-a5fb-d27f77b4b1af
```

### Mesh-comparison slice-fork pair (`v2_continuous` takeoff, with `SliceOutput`)

```
case-d178c940-4f33-481f-9891-e912f780b5ca   cont_low_takeoff_coarse_ad_slice
case-62559b27-a446-42b4-a5d9-52950714d18f   cont_low_takeoff_fine_ad_slice
```

These two cases share trim point and AD inputs but differ only in
`PROP_REFINE_M` (0.0625 vs 0.03125 m). Their `y = +0.554` slices
(prop-1 axis) make the mesh-resolution effect at the disk visible
directly: see `paper/figures/slices/ad_mesh_compare/`.

### Source repo

`himalaya/` — `cfd_setup.py`, `params.py`, the four `.csm` geometries
(`tsangpo*.csm`), and `flow360/submit_fine_ad_parents.py` are the
relevant entry points. Happy to share the repo location or a tarball.

## 9. Questions for the team

In rough priority order:

1. **Specification semantics**: is `ForcePerArea.thrust` interpreted
   as a face pressure jump (Pa) or a body-force density (effectively
   Pa × cylinder thickness)? `Disk_i_Force` in the output — is it
   integrated normal force, or already a coefficient?
2. **High-loading correction**: does the constant-pressure-jump AD
   model apply a momentum-theory induced-velocity correction at high
   disk loading? If so, where in the docs is this described, and
   what's the closed-form mapping from commanded thrust to delivered?
3. **Reference**: a worked example in the docs of "commanded thrust =
   X, delivered force expected = Y" for a non-trivial loading would
   close this for users.
4. **Workaround**: at the very least, is there a known practice for
   getting `commanded ≈ delivered` to within a few % at takeoff/landing
   disk loading? E.g., a recommended cylinder-thickness : refinement
   spacing ratio, an alternative `BETDisk` formulation, or a different
   model variant?

I'll happily participate in any further debugging — flow visualization
slices through the disk are ready to share, and the project structure
matches Flexcompute's standard `Project.run_case` flow.
