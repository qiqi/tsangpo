# Flow360 agent-usability report v2

**Reporter:** Qiqi Wang (qiqi@flexcompute.com)
**Date:** 2026-05-16
**SDK:** flow360 v25.9.x · Solver: release-25.9 · Mesher: beta in-house

A running log of Flow360 quirks that made it harder for an LLM agent
to drive the SDK during the Tsangpo eSTOL CFD campaign.  These are
beyond the GAI / nested-rotation issues already filed in
`post/FLOW360_GAI_BUG_REPORT.md` (which is largely resolved).

Each entry: **what bit us**, **why an agent stumbles on it**,
**recommended fix**.

---

## 1. Case IDs mutate on draft → COMPLETED promotion

**Bit us:** `submit_high_htail_sweeps.py` cached parent case IDs from
the submission step, then re-used them later to `fork_from=parent`.
Half the forks 409'd because the parent's case ID changed during the
draft → COMPLETED promotion (e.g., `case-a7d02011-db4b-4f99-… →
case-a7d02011-db4b-4286-…`). Same SHA prefix, different middle.

**Agent angle:** an obvious correctness pattern for an agent is
"cache the ID and use it later." That assumption is silently false.

**Recommended fix:** keep case IDs invariant across status promotions,
or expose a stable `external_id` field that the SDK consumer chooses.
Failing that, document this prominently — multiple downstream agents
will write the same buggy caching pattern.

---

## 2. `download(...)` returns `None`, not the path

**Bit us:** All `case.results.{slices,surface_forces,...}.download(to_folder=tmp)`
calls return `None`. The downloaded file lands at
`<tmp>/<some_filename>.csv` but the SDK doesn't tell us what the
filename ended up being. We worked around it with `next(Path(tmp).rglob("*.csv"))`,
which is brittle.

**Agent angle:** an agent writes the obvious code `p = case.results.X.download(...)`
and then `pd.read_csv(p)` — that crashes with "Invalid file path or
buffer object type: <class 'NoneType'>".

**Recommended fix:** every `download` method should return the local
file path (preferred) or write to a known canonical name (e.g.,
exactly `surface_forces.csv` regardless of solver-internal naming).

---

## 3. `case.params` introspection spams refinement WARNINGs

**Bit us:** every `from_cloud(case_id=...)` + access to `case.params`
prints, for every UniformRefinement on the case:

```
WARNING: The spacing of 0.06921 m specified in UniformRefinement
will be cast to the first lower refinement in the octree series
(0.0625 m).
```

A status-check script iterating 120 cases produced 240+ identical
WARNING blocks. Status output buried.

**Agent angle:** an agent piping output through `tail` loses real
content; an agent searching for ERROR/FAIL keywords gets false-positive
WARNINGs first.

**Recommended fix:** emit the warning once at submission time (when
the spacing actually gets cast), not on every params read. Or expose
a `flow360.set_log_level()` hook the SDK consumer can quiet for
introspection.

---

## 4. No bulk-fetch of project case metadata

**Bit us:** to find which cases in a project are COMPLETED vs RUNNING
we have to call `fl.Case.from_cloud(case_id=cid)` for each. With ~80
cases per project, an idempotency check that iterates "for every case,
fetch every other case" becomes O(N²) network calls.

**Agent angle:** the obvious "check if X already exists in project"
becomes 6000-call O(N²) on a 80-case project, taking many minutes.
Agent reaches for parallelism and then has to discover that the SDK
isn't thread-safe.

**Recommended fix:** `project.list_cases()` should return all cases'
`{id, name, status, tags, created_at}` in a single request. This is
already implicit on the project page in the web UI.

---

## 5. Sweep discovery: no first-class "sweep axis" metadata

**Bit us:** in `_phase_plot.py:27` we use a regex
`r"(?:^|_)(alpha|htail|thrust)_([pmx][\d.p]+)(?:deg)?(?:$|_)"` to parse
sweep type and value out of the case name. Older cont-low cases were
named `alpha_p5p0deg` (no `_` prefix); the older regex `r"_(alpha|...)"`
silently matched zero cases and the plotter produced a
single-data-point figure with no warning.

**Agent angle:** name-based metadata is a fragile contract between
the submission script and the post-processing script. The agent
writes a regex, ships it, and N submissions later finds the regex
wasn't broad enough.

**Recommended fix:** allow cases to carry typed metadata in `case.params`
(or `case.tags`) — e.g.,
`fl.SweepAxis(name="alpha", value=+5.0)` — and let
the SDK surface it via `case.sweep_axis`.

---

## 6. Mesh-stage failures give no log on the case object

**Bit us:** when a case's volume mesh fails, the case status goes to
`ERROR` but `case.logs.errors()` returns "No log files available for
this resource. The job may not have started or produced any logs
yet." To actually find the error we had to:

1. Note `case.info.case_mesh_id` (works only because the property
   exists on the model — easy to miss).
2. `fl.VolumeMesh.from_cloud(id=...)` (note: `id=`, not `case_id=` or
   `mesh_id=`).
3. `vm._webapi._download_file('logs/flow360_volume_mesh.user.log',
   to_folder=tmp)` (private API, returns `None`).
4. `(Path(tmp).rglob('*.log'))[0].read_text()` and grep for `ERROR`.

**Agent angle:** the agent's first instinct, `case.logs.tail()`,
returns a confidently-wrong message. Five additional steps to find
the actual error.

**Recommended fix:** when a case fails at the mesh stage, surface the
mesher log via `case.logs` directly with a small "(from mesh stage)"
prefix. Make the path obvious.

---

## 7. Surface-mesh intersection error is a giant log dump, no summary

**Bit us:** Error 4081 "Surface mesh contains self-intersecting
triangles!" was preceded by *1775 individual triangle coordinate
dumps* (~5000 log lines). The actual root-cause statement is at the
very end:

```
[USER]: Found 1775 self-intersecting triangles in surface mesh!
[USER]: (ERROR 4081) Surface mesh contains self-intersecting triangles!
```

with no summary of WHICH surfaces intersect or AT WHAT spanwise/chord-
wise station. We had to manually scan the triangle coords (`+y =
±2.879`) and back-correlate with the geometry to figure out that the
htail rotation cylinder was cutting through the wing TE.

**Agent angle:** ~5000-line log dump exceeds an agent's context easily;
no `egrep` query short of "(ERROR" tells you anything useful, and
once it does, there's no spatial context.

**Recommended fix:** at the end of the triangle-dump, print a SUMMARY:
"Intersections concentrated at y ≈ ±2.88 m (matching the
ht_cyl rotation-zone boundary). Suspect surfaces: main_wing,
htail_pitch_zone." A nearest-named-surface lookup would close most
debug loops in one line.

---

## 8. `fl.VolumeMesh.from_cloud` parameter is `id=`, not `case_id=` or `mesh_id=`

**Bit us:** other `from_cloud` constructors take typed keywords:
`fl.Case.from_cloud(case_id=...)`, `fl.Project.from_cloud(project_id=...)`,
`fl.SurfaceMesh.from_cloud(id=...)`, `fl.VolumeMesh.from_cloud(id=...)`.
The naming inconsistency was a 3-iteration error loop for us.

**Recommended fix:** standardize on either `id=` everywhere (deprecate
the typed keywords) or `{resource}_id=` everywhere. Document the
chosen convention.

---

## 9. `download()` is a private method on some result types

**Bit us:** `case.logs` is a `RemoteResourceLogs` object — has no
`download()` method (`AttributeError`), only `head/tail/errors/print/
to_file`. `case.results.slices` is a `ResultTarGZModel` — has
`download(to_folder=...)`. `case.results.volumes` is the same type as
slices. `vm.logs` is something else again (the mesh log isn't even
exposed as a public attribute; we used `vm._webapi._download_file(...)`).

**Recommended fix:** unify the resource-download surface. Every "thing
on cloud storage" should respond to `.download(to_folder=...) -> Path`
or `.read_text() -> str`. Currently the consumer has to remember a
different incantation per resource type.

---

## 10. `SliceOutput` is silently optional

**Bit us:** we assumed `case.results.slices` would be auto-populated
with default slice planes. It's not — slices only exist if
`SliceOutput` was in the submission `outputs` list. With no
`SliceOutput`, `case.results.slices.download(...)` returns a 404. The
SDK presents a `slices` attribute either way, with no indication that
no slices were generated.

**Recommended fix:** at minimum, document conspicuously that slices
require explicit configuration. Better: make `case.results.slices`
raise a typed exception like
`Flow360NoSuchOutputConfigured("SliceOutput")` rather than a 404 from
the S3 client.

---

## 11. `force_per_area.thrust` semantics aren't fully documented

**Bit us:** see `paper/figures/FLOW360_AD_BUG_REPORT.md` (the
parallel bug report we'll send). Short version: we don't actually
know whether `force_per_area.thrust` is a face pressure jump, a body-
force density, or a coefficient — and the integrated reaction force
(0.65 ×, 1.10 ×, or who-knows-× the commanded depending on disk
resolution and inflow loading) varies more than enough to matter.

**Agent angle:** an agent reads the field name `force_per_area.thrust`
and unit (`N/m²`), assumes "uniform pressure jump", and computes
`commanded = pressure × annular_area`. The actual delivered force is
0.65× of that at takeoff/landing with the fine mesh. The
discrepancy is large enough to flip the cross-config CD ranking in
our paper figures.

**Recommended fix:** add a numerical worked example to the
`ActuatorDisk` docs: "for a uniform `thrust=730.31 N/m²` over annulus
r∈[0.08, 0.534] m, with axial cylinder height = 0.139 m and
sufficient mesh resolution (≥ N cells radial, ≥ M cells axial),
the integrated `Disk_i_Force` is expected to be …". Then we'll know
when our results are model-correct vs mesh-artifact.

---

## 12. Submission scripts can't auto-detect campaign-level params drift

**Bit us:** cont-low submissions used an older `cfd_setup.py` version
where `force_per_area.thrust` reflected pre-v2 values. Even though
the printed `force_per_area.thrust = 730.31 N/m²` looked identical to
the new v2 cases, the *delivered* solver force differed. We couldn't
detect this from the case introspection alone.

**Agent angle:** an agent maintaining a campaign over time has no
way to know that "two cases with identical-looking inputs delivered
different outputs because the cfd_setup.py changed between
submissions." We had to find this empirically by comparing per-disk
forces.

**Recommended fix:** record the submission-time SDK + solver +
mesher version triple in `case.info` as separate fields, and a
content hash of the params. The hash lets us cluster cases that were
actually built from identical inputs.

---

## 13. No public API for the volume-mesh log

**Bit us:** see #6 — `vm._webapi._download_file(...)` is a private
method (leading underscore). The only "public" way to read the
mesh log is to download all mesh files via the web UI and grep the
log out by hand.

**Recommended fix:** `mesh.logs.tail(N)` mirroring `case.logs.tail(N)`.

---

## 14. `PENDING` vs `RUNNING` distinction is invisible in summaries

**Bit us:** our hourly progress-check counted `PENDING` and `RUNNING`
together as "in-flight." Cases sitting in the GPU queue look the
same as cases actively solving. We wrote a "v3 cases not progressing"
alarm that fired three times before we noticed the cases were just
queued.

**Agent angle:** in a busy fleet, a *queue-time* metric would tell
the agent "wait longer, no need to debug." Without it, we look for
problems that aren't there.

**Recommended fix:** expose `case.info.queue_time_seconds`,
`case.info.run_time_seconds`, or a richer status with sub-stages
(`QUEUED`, `MESHING`, `SOLVING`, `POSTPROCESSING`).

---

## 15. Constant per-iteration printout in `actuator_disks` is wasteful

**Bit us:** `case.results.actuator_disks.load_from_remote()` then
inspect `av["Disk0_Force"]` — the array has 2020 entries, all
*identical*. The disk force was set at iteration 0 and never moved
(constant-pressure-jump model). 2020× the data we needed.

**Recommended fix:** RLE-compress constant time-series in the
solver output. Or expose a `case.results.actuator_disks.final` that
returns the last-iteration scalar directly.

---

## Aggregate fixes that would help most

In rough priority for an agent-automation use case:

1. **#4 bulk case metadata** — eliminates the O(N²) blocker.
2. **#3 quiet WARNING spam** — clears the agent's stdout.
3. **#6 surface mesh failure logs on the case object** — closes the
   biggest debugging loop.
4. **#11 worked AD example in docs** — eliminates the biggest
   unknown-physics question hanging over our paper.
5. **#9 unified resource download** — eliminates a constant
   tax on script reliability.

These five together would probably halve the time required for the
next agent-driven campaign of this size.

---

We'll happily participate in a fix-design call for any of the above
— project repo state and per-case reproducers available.
