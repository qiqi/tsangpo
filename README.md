# Tsangpo eSTOL — CFD Campaign & SciTech Paper Repo

A 10-prop, 2,600 lb, 165 ft² wing eSTOL configuration whose Phase-1 design
question is: *what happens when you remove the inboard flaps and lower the
H-tail into the inboard propellers' wake?*

The answer this project is built to defend, with Flow360 RANS at the
SciTech-paper level of rigor, is the **Stability Paradox**:

> A gap in the flap is not a leak in lift. It is the channel through which
> the inboard propulsors feed the empennage. Sacrificing ~10 % of total $C_L$
> buys a large gain in low-speed pitch authority — enough to enable the
> Ramp takeoff and Whip-Stall landing profiles.

The full plot, parameter list, and paper outline live in the three
source-of-truth files at the repo root.

---

## Repo layout

```
.
├── plan.md                 Technical Nuggets ledger
├── params.py               Central dimension repository (the only source of truth)
├── scitech_outline.md      Paper outline w/ Form / Formless mapping
├── wu-wei.mdc              Rule: optimize for the minimum required for correctness
├── geometry/
│   ├── tsangpo.csm        ESP parametric model: 30P30N slat + main + outboard flap, Hershey H-tail
│   ├── airfoils/           30P30N coordinate files (Slat / Main / Flap, normalized)
│   ├── render.py           STL -> 2x2 matrix PNG (top + side + zoomed 30P30N contour per case)
│   ├── README.md           ESP parameter table + invocation cheat-sheet
│   └── out/<case>/         per-case STL bodies + airframe.step + airfoils symlink (generated)
├── flow360/
│   ├── mesh_strategy.md    Vortex-refinement + actuator-disk meshing strategy
│   ├── case_template.json  Flow360 RANS case JSON template
│   ├── run_matrix.py       Expands study1_matrix() -> per-case Flow360 JSONs (--submit to send)
│   └── build_geometry.py   Drives serveCSM -batch for each case
└── post/
    └── extract_stability.py  C_m(alpha), neutral-point shift + headline contrast table
```

---

## Quick start

```bash
# Parameter set + the 4-case run matrix summary:
python params.py

# Emit Flow360 case JSONs for all 4 cases:
python flow360/run_matrix.py

# Preview the stability table + non-linear-coupling decomposition
# (synthetic until forces.csv is populated by the real CFD runs):
python post/extract_stability.py
```

When ESP is sourced and the Flow360 SDK is installed:

```bash
source ~/esp/ESP129/EngSketchPad/ESPenv.sh

# Build per-case STL bodies + airframe.step:
python flow360/build_geometry.py

# Submit the run matrix to Flow360:
python flow360/run_matrix.py --submit

# Render the 2x2 geometry PNG. ESPenv.sh exports a PYTHONPATH pointing at
# ESP's Python 3.12 site-packages; we have to strip it so the system Python
# 3.10 sees its own numpy/scipy/trimesh:
env -u PYTHONPATH python3 geometry/render.py
```

---

## Study 1 — the 2x2 matrix

Steady-state RANS at $\alpha = 10^\circ$, $T/W = 0.5$, 12,000 ft density
altitude. The matrix isolates the **non-linear coupling** between two
configuration knobs: the inboard flap gap and the H-tail vertical position.

|                       | continuous flap (gap = 0.00)         | inboard gap (gap = 0.35)              |
|-----------------------|--------------------------------------|---------------------------------------|
| **high T-tail**       | C1 **Industry Baseline**             | C3 **Bad Trade-off**                  |
| ($Z_{tail} = +2.5\,c$)| stable, heavy                        | stable, heavy, lift penalty           |
| **low H-tail**        | C2 **Downwash Failure**              | C4 **Proposed Synthesis**             |
| ($Z_{tail} = 0\,c$)   | unstable (low tail in downwash)      | stable, lightweight, agile            |

The headline test of the paper is C2 vs. C4: switching the inboard flap to
a gap takes the low H-tail from *unstable* to *more strongly stable than
the heavy T-tail baseline.* Linear superposition of the two effects (Z and
gap, measured alone) cannot reproduce this — the non-linearity is the
result.

---

## Conventions

- Units: ft / slug / lbf / s.
- Origin at wing root c/4 on the symmetry plane; +X aft, +Y starboard, +Z up.
- Numbers live in `params.py`. Hard-coded dimensions anywhere else are a bug.

### Wing section — 30P30N validation airfoil

The wing's chordwise section is the McDonnell-Douglas **30P30N** 3-element
high-lift airfoil (30° slat, 30° flap), the canonical RANS validation case
for high-lift CFD. The three element contours live in
`geometry/airfoils/30P30N_{Slat,Main,Flap}.dat3` and are fit by ESP's
`udpFitcurve` into B-spline faces, then ruled root-to-tip into Hershey-bar
3D bodies. The `gap_fraction` design parameter suppresses the inboard
portion of the **flap** only — slat and main element remain continuous
across the full span.

### ESP / OpenCSM gotchas (discovered the hard way)

These bite when reading or editing `geometry/tsangpo.csm`:

- **NACA UDP plane.** `udprim naca Series NNNN` (used for the H-tail)
  draws the airfoil in the XY plane with thickness in Y. Wings need
  thickness in Z, so every NACA sketch is followed by `rotatex 90 0 0`
  before `scale` / `translate`. The same convention is used for the
  `udpFitcurve` sheet bodies that build the 30P30N elements.
- **`udpFitcurve` needs a `split` index** for closed contours, otherwise
  it errors with "wraparound geometry with only one Edge". We split each
  30P30N element at its leading-edge (min-x) point; the indices are
  baked into `tsangpo.csm` (`slat_LE_idx`, `main_LE_idx`, `flap_LE_idx`).
- **`udpFitcurve` filename arg** is a string, addressed as
  `$airfoils/30P30N_*.dat3` in CSM. `flow360/build_geometry.py` drops a
  symlink `case_dir/airfoils -> geometry/airfoils` so the relative path
  resolves from the per-case build cwd.
- **ROTATEY pivot args.** The signature is `rotatey angDeg zaxis xaxis`
  (z-pivot first, x-pivot second), per `OpenCSM.c`. Bites you the moment
  you try to rotate something about an axis that is not the y-axis.
- **MARK / RULE.** `MARK` blocks are closed by the subsequent `RULE`,
  `BLEND`, or `LOFT`; do not put a stray `END` inside one.
- **`-despmtrs`, not `-despmtr`.** `serveCSM -batch` takes a single file
  argument `-despmtrs FILE` with `param value` pairs. `flow360/build_geometry.py`
  writes a tiny tempfile per case.
