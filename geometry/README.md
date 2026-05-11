# Geometry — ESP / Engineering Sketch Pad

## Files

- `himalaya.csm` — top-level parametric model.

## Parameters that drive Study 1

| CSM symbol         | Default | Sweep range / values     | Meaning                              |
|--------------------|---------|--------------------------|--------------------------------------|
| `gap_fraction`     | 0.0     | {0.0, 0.35}              | inboard fraction of semi-span with flap suppressed |
| `Z_tail_chords`    | 0.0     | -0.5 → +2.0 (6 stations) | H-tail vertical position, in MAC     |
| `X_tail_mac`       | 3.5     | {3.0, 3.5, 4.0}          | H-tail longitudinal arm, in MAC      |

## Driving the model from the CLI

Inside ESP / `serveCSM`:

```text
serveCSM -batch himalaya.csm -despmtr gap_fraction 0.35 \
                             -despmtr Z_tail_chords 0.0 \
                             -dump   airframe.step
```

A Python driver that loops `params.study1_matrix()` and emits one STEP file
per case lives at `../flow360/build_geometry.py`.

## Numbering & conventions

- Origin at wing root quarter-chord on the symmetry plane.
- +X aft, +Y starboard, +Z up.
- Prop index runs 1 (inboard) → 5 (outboard) per semi-span.
- Each prop disk solid carries attributes `propIndex`, `side`,
  `overTail` so the mesher can target refinement boxes by attribute name.
