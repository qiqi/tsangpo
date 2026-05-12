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
- One `.step` per body (not a combined airframe.step): Flow360 merges a
  combined STEP into a single body, which defeats the per-surface
  boundary naming.
