# Tsangpo eSTOL — project conventions

## Flow360

**Always submit cases with `use_beta_mesher=True`.** The in-house beta
mesher does curvature-based surface refinement at leading edges without
requiring an explicit split line — the legacy mesher produces a poor
LE surface on the LS(1)-0417 / NACA spline contours we use here.

```python
project.run_case(params=params, ..., use_beta_mesher=True)
```

`MeshingDefaults.curvature_resolution_angle` is also only honored by
the beta mesher; with the legacy mesher it silently does nothing.

## Geometry

- Airfoil sketches are emitted by `geometry/airfoils/build_estol_geometry.py`
  as OpenCSM splines (not piecewise linsegs) — see UDC topology in that
  script's docstring.
- `geometry/tsangpo.csm` is phase-parameterized: pass `phase 0/1/2` via
  `-despmtrs` for stowed / takeoff / landing.
- **Name every element's faces with `attribute capsGroup $<name>`**
  (right after `extrude`, via `select face / attribute capsGroup …`).
  Flow360's geometry processor exposes `capsGroup` as a face attribute
  tag; `submit_cruise.py` calls `geo.group_faces_by_tag("capsGroup")`
  and looks up surfaces by their .csm-given names. Do NOT rely on body
  dump order or `body0000N` renaming — it's fragile across configs.
- Upload to Flow360 by inlining UDCs into `tsangpo.csm` and passing a
  single `.csm` file to `from_geometry`. Multi-file STEP uploads only
  surface-mesh body00001 (Flow360 limitation as of 25.9.x).
