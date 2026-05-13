# Flow360 GAI volume mesher: ERROR 7221 on `enclosed_entities` under nested rotation volumes

## TL;DR

The Geometry-AI volume mesher rejects a `RotationVolume.enclosed_entities`
reference to a surface tagged by its `capsGroup` name when that surface
lives inside *nested* rotation zones. The same `SimulationParams` builds
without complaint on the legacy beta mesher. Symmetric behaviour
between the two meshers seems to be the intended contract; this looks
like a missing alias step in the GAI volume mesher's entity-lookup.

Same project, same geometry, same SDK params. Legacy builds; GAI errors
with `(ERROR 7221) Encountered object name: htail in enclosed objects
for sliding interface: slidingInterface-htail_pitch_zone, which is not
a known volume entity.`

## Environment

- Flow360 Python SDK: **release-25.9** (client version 25.9.8 per error
  payload).
- Solver version on the failing run: `release-25.9` (per VolumeMeshV2's
  `solver_version`).
- User: `AIDAXWCOWJGJMSYAG54AU`.
- Project: `prj-3e8b1ed8-0109-4f1f-8738-fef0c55a573b`
  (`tsangpo_blunt_htail_active`).
- Geometry: 3-element wing (main_wing + vane + aft_flap) plus an
  inverted LS(1)-0417 H-tail, all tagged via `attribute capsGroup
  $<name>` in `tsangpo.csm`. The .csm has been used to build dozens
  of successful runs on this project under the legacy mesher.

## Reproduction

### Failing case / mesh / surface mesh

| | id | status |
|---|---|---|
| Parent case (failing) | `case-a8634829-82ed-44fa-af92-2a64a767531e` | ERROR |
| Volume mesh built from it | `vm-2d091b4e-d015-4072-9faf-b25e7252399e` | error |
| Upstream GAI surface mesh | `sm-192138ea-8c75-4b1f-9582-aa70133df0c0` | **PROCESSED** (485 MB, 776 286 nodes, 1 552 556 triangles) |
| Geometry | `geo-4618ed7e-5a71-4632-9b4c-8c1c8342eabd` | uploaded |

The case was submitted via `project.run_case(..., use_beta_mesher=True,
use_geometry_AI=True)`. Surface mesh built fine; volume mesh build
errored before solver start. All 30 downstream forks of this parent
also errored because they depend on the volume mesh.

### Code that triggers it

The relevant fragment of the `SimulationParams` construction (full
builder is in our repo at `cfd_setup.py` :: `build_params`):

```python
ac_cyl = fl.Cylinder(
    name="ac_pitch_zone",
    center=(0,0,0)*fl.u.m, axis=(0,1,0),
    height=AC_HEIGHT_M*fl.u.m, outer_radius=AC_RADIUS_M*fl.u.m,
)
ht_cyl = fl.Cylinder(
    name="htail_pitch_zone",
    center=(X_TAIL, 0, 0)*fl.u.m, axis=(0,1,0),
    height=HT_HEIGHT_M*fl.u.m, outer_radius=HT_RADIUS_M*fl.u.m,
)

# nested rotations: htail rotates inside the aircraft rotation
ac_rotation = fl.Rotation(
    name="ac_pitch", volumes=[ac_cyl],
    spec=fl.AngleExpression(f"{theta_ac:.10f}"),
)
ht_rotation = fl.Rotation(
    name="htail_pitch", volumes=[ht_cyl],
    spec=fl.AngleExpression(f"{theta_ht:.10f}"),
    parent_volume=ac_cyl,
)

# Failing entity reference is in this inner RotationVolume:
fl.MeshingParams(volume_zones=[
    farfield,
    fl.RotationVolume(
        name="ac_rotation", entities=ac_cyl,
        enclosed_entities=[main_wing_surf, vane_surf, aft_flap_surf, ht_cyl],
        ...
    ),
    fl.RotationVolume(
        name="htail_rotation", entities=ht_cyl,
        enclosed_entities=[htail_surf],     # ← htail_surf = geo["htail"], a Surface
        ...
    ),
])
```

`htail_surf` is obtained from
`project.geometry.group_faces_by_tag("capsGroup"); geo["htail"]`, i.e.
referenced by the capsGroup tag set in the .csm. The cylinders are not
in `parent_id` form of each other; `ht_cyl` is passed to the inner
RotationVolume's `entities` and to the outer one's `enclosed_entities`,
which is how we express "the htail volume sits inside the aircraft
volume". The outer Rotation also gets `parent_volume=ac_cyl` to declare
the hierarchy.

## The error

### Volume-mesher user log

`logs/flow360_volume_mesh.user.log` from `vm-2d091b4e` (downloaded from
S3 via the project's userCredentials):

```
[26-05-13 19:02:28.366][USER]: Started Flynn360
[26-05-13 19:02:28.366][USER]: Volume mesh parameters:
[26-05-13 19:02:28.366][USER]: - planarFaceTolerance: 1.000e-06
[26-05-13 19:02:28.366][USER]: - slidingInterfaceTolerance: 1.000e-02
[26-05-13 19:02:28.366][USER]: Boundary layer mesh parameters:
[26-05-13 19:02:28.366][USER]: - firstLayerThickness: 7.620e-06
[26-05-13 19:02:28.366][USER]: - growthRate: 1.3000
[26-05-13 19:02:28.366][USER]: - gapTreatmentStrength: 0.5
[26-05-13 19:02:28.366][USER]: Read 10 refinement zone(s)
[26-05-13 19:02:28.366][USER]: Read 2 sliding interfaces
[26-05-13 19:02:28.368][USER]: Started reading surfaceMesh.lb8.ugrid
[26-05-13 19:02:28.396][USER]: Number of boundary patches found in mesh file: 9
[26-05-13 19:02:28.804][USER]: Number of boundary patches read: 4
[26-05-13 19:02:28.804][USER]:  - Boundary patch 5 (name: main_wing__rotating_ac_pitch_zone) contains 581,928 triangles, 0 quads
[26-05-13 19:02:28.804][USER]:  - Boundary patch 9 (name: htail__rotating_ac_pitch_zone__rotating_htail_pitch_zone) contains 168,886 triangles, 0 quads
[26-05-13 19:02:28.804][USER]:  - Boundary patch 7 (name: aft_flap__rotating_ac_pitch_zone) contains 489,746 triangles, 0 quads
[26-05-13 19:02:28.804][USER]:  - Boundary patch 6 (name: vane__rotating_ac_pitch_zone) contains 311,996 triangles, 0 quads
[26-05-13 19:02:28.870][USER]: Finished reading surfaceMesh.lb8.ugrid (776286 points, 1552556 triangles, 0 quads)
[26-05-13 19:02:28.902][USER]: - Mesh file has 4 boundaries
[26-05-13 19:02:33.585][USER]: (ERROR 7221) Encountered object name: htail in enclosed objects for sliding interface: slidingInterface-htail_pitch_zone, which is not a known volume entity.
```

Three things to notice in the log:

1. The GAI surface mesher has already **renamed** the htail boundary
   patch by appending the rotation-zone hierarchy:
   `htail` → `htail__rotating_ac_pitch_zone__rotating_htail_pitch_zone`.
   The wing/vane/flap also got `__rotating_ac_pitch_zone` appended,
   but only one level since they're inside a single rotation.
2. The volume mesher reads the surface mesh and acknowledges these
   suffixed names verbatim — it does not maintain an alias from
   `htail` → the suffixed name.
3. When it then walks the `enclosed_entities` of
   `slidingInterface-htail_pitch_zone`, it looks for an entity named
   `htail` (the original capsGroup tag, which is what the SDK
   serialized). No match → 7221.

### REST endpoint to retrieve the log

```python
from flow360.cloud.http_util import http
resp = http.get(
    "v2/volume-meshes/vm-2d091b4e-d015-4072-9faf-b25e7252399e/file"
    "?filename=logs/flow360_volume_mesh.user.log"
)
# resp['userCredentials'] + resp['cloudpath'] gives an STS-scoped S3 URL.
```

The file is 2 431 bytes; the snippet above is the full content (minus a
few echo lines I trimmed).

## What the legacy mesher does differently

The exact same `SimulationParams` (same project, same geometry,
identical `enclosed_entities`, identical capsGroup tags) builds without
error on the legacy beta mesher (`use_geometry_AI=False`). We have
~60 successful cases on this same project under the legacy mesher,
e.g. cases-`9520355d` (parent), `c09282c9..ad90944f` (10-fork htail
sweep at α=+7°), `62103e52..0376a381` (10-fork α sweep), `d820f45a..2f5c1360`
(10-fork thrust sweep).

Some of those completed cases produced `surface_forces_v2.csv` with
keys like `ac_pitch_zone/main_wing_CL` and
`htail_pitch_zone/htail_CL` — which **indicates the legacy mesher also
renames patches with zone-hierarchy prefixes/suffixes internally**, but
keeps an alias from the original capsGroup name back to the renamed
patch. The GAI volume mesher appears to drop that alias step.

So the asymmetry is *not* "GAI renames, legacy doesn't" — both rename.
It's "legacy resolves SDK references to renamed patches; GAI does not."

## Suggested investigation paths for whoever has source-tree access

1. **Where the SDK serializes the entity reference.** What does
   `RotationVolume.enclosed_entities = [Surface(name="htail")]` look
   like in the JSON that goes to the volume mesher? Probably a string
   `"htail"`. If so, what's responsible on the receiving end for
   resolving that string to a mesh patch when the patch has been
   renamed by the surface mesher? In the legacy path, find the lookup
   table (alias map). In the GAI path, the same map likely isn't being
   populated — that's the fix point.

2. **Where the GAI surface mesher emits the rename.** Greppable
   targets: `__rotating_`, `rotating_<name>`,
   `slidingInterface-<name>`. The renaming convention is
   `<orig>__rotating_<outer_zone>__rotating_<inner_zone>` — one or
   more `__rotating_<zone>` suffixes depending on nesting depth.
   Whatever code produces that should also (a) record the alias
   back to `<orig>` for downstream lookups, OR (b) the downstream
   lookup should strip `__rotating_...` to recover the original tag.

3. **`Encountered object name: ... is not a known volume entity`.**
   String-search the volume mesher's source for this exact message;
   that's the throw site. The fix is almost certainly two lines above
   it — accept the original name even if the actual patch was
   renamed.

4. **Surface vs volume distinction.** The error says "not a known
   volume entity" but `htail` was always a surface. This may indicate
   the lookup is hitting the wrong table type, or the message is just
   inaccurate. Worth checking which table it's searching.

5. **Reproducibility test.** A minimal reproducer is: any two-level
   nested rotation (`Rotation(parent_volume=outer_cyl)`) with the
   inner `RotationVolume.enclosed_entities` referencing a surface by
   its capsGroup name. Flat single-rotation setups (only one
   rotation cylinder, surfaces directly under `ac_pitch_zone`) should
   *not* trip this because the rename gives only one
   `__rotating_<name>` suffix and the lookup map may handle that
   single case correctly. If you have a regression test for
   `actuatorDisk_*_thrustMultiplier` UDD on GAI, the same project
   layout should reproduce.

## Workarounds we've tried / will try

- **Submit non-GAI at the same trim point.** Works (legacy mesher
  doesn't have the bug) but doesn't give us GAI's leading-edge /
  thin-geometry refinement, which was the whole point.

- **Drop `enclosed_entities=[htail_surf]` from the inner
  RotationVolume.** Tested via `case-f4d78aa5` (cruise trim point,
  vm `vm-9d784174`):

  - ✅ **Volume mesh build succeeded** — `vm-9d784174.status` =
    `processed`/COMPLETED. So whatever entity-lookup the volume
    mesher does for `enclosed_entities` was the *first* place
    the alias problem surfaced; removing the explicit hint clears
    that check, and spatial enclosure inside the cylinder is
    sufficient for the mesher to assemble the sliding interface.
  - ❌ **Case JSON validation still fails immediately downstream.**
    The case errored at solver startup with:

    ```
    (WARNING) htail, htail is/are not found in the volume mesh file.
              It/they may have been deleted during mesh generation
              process or not generated by mesher.
    (ERROR 0010) Case json failed validation check
        validationError: ["Boundary
            farfield/htail__rotating_ac_pitch_zone__rotating_htail_pitch_zone
            in the mesh file is not defined in the Flow360 case JSON."]
    ```

    The Wall model `fl.Wall(entities=[main_wing_surf, vane_surf,
    aft_flap_surf, htail_surf])` references `htail_surf` by its
    capsGroup tag `htail`; the mesh exposes the patch as
    `htail__rotating_ac_pitch_zone__rotating_htail_pitch_zone`.
    The validator looks for `htail` (or a Wall mapping for the
    suffixed name) and finds neither.

  Implication: the alias-resolution gap isn't just in the volume
  mesher — it's a project-wide GAI invariant. **At least two
  separate code paths need the alias map**: the volume mesher's
  `enclosed_entities` resolver, and the case JSON's boundary-set
  validator. Probably more.

- **Workaround B: flatten the rotation hierarchy.** Not yet tested,
  but likely *won't* fix this on its own — the htail surface would
  still get a zone suffix (`htail__rotating_htail_pitch_zone`,
  single-level), and the case JSON validator still expects bare
  `htail`. Worth trying for completeness if Flow360 has a partial
  fix in the works for single-level renames.

- **Workaround C (SDK-side):** construct the `Wall(...)` model
  using a `Surface(name="htail__rotating_..._rotating_...")` built
  directly rather than via `geo["htail"]`. Not tested; the SDK's
  Surface lookup is typed to `geo["..."]` only.

## Implication for the project

For now, **GAI is fully blocked for any geometry with rotation
zones that enclose named surfaces** — and any aircraft case with a
sliding-interface mesh strategy has that. We've confirmed this
applies to both:

- Cruise trim campaign (nested ac_pitch_zone + htail_pitch_zone):
  parent `case-a8634829` + `case-f4d78aa5` both blocked.
- Takeoff Phase C planned: same nested-rotation params, would hit
  the same chain.

Sticking with the legacy beta mesher for cruise / takeoff / landing
Phase C, and optionally tightening the legacy mesh settings
(`surface_max_edge_length`=0.04 m, `curvature_resolution_angle`=10°)
for a finer answer.

## Why this matters for our project

This geometry is `eSTOL` with distributed propulsion and an htail in
the slipstream — we want the htail in its own rotation volume so we
can sweep its incidence independently of aircraft α. That requires
the nested-rotation structure that triggers the bug. The geometry
also has fine features (a blunt TE of 4.15 mm on a 1.4 m chord), so
GAI is genuinely the right tool for the surface mesh. The non-GAI
fallback gives us roughly 1/3 the surface triangle count and visibly
coarser leading-edge resolution.
