# Himalayan eSTOL — CFD Campaign & SciTech Paper Repo

A 10-prop, 2,600 lb, 165 ft² wing eSTOL configuration whose Phase-1 design
question is: *what happens when you remove the inboard flaps and lower the
H-tail into the inboard propellers' wake?*

The answer this project is built to defend, with Flow360 RANS at the
SciTech-paper level of rigor, is the **Stability Paradox**:

> A gap in the flap is not a leak in lift. It is the channel through which
> the inboard propulsors feed the empennage. Sacrificing ~10% of total $C_L$
> buys a large gain in low-speed pitch authority — enough to enable the
> Ramp takeoff and Whip-Stall landing profiles.

The full plot, parameter list, and paper outline live in the three
source-of-truth files at the repo root.

---

## Repo layout

```
.
├── plan.md                 Technical Nuggets ledger
├── params.py               Central dimension repository
├── scitech_outline.md      Paper outline w/ Form/Formless mapping
├── wu-wei.mdc              Rule: optimize for the minimum required for correctness
├── geometry/
│   ├── himalaya.csm        ESP parametric model (Hershey-bar wing + H-tail + flap)
│   ├── render.py           STL -> Baseline-vs-Proposed PNG
│   └── README.md
├── flow360/
│   ├── mesh_strategy.md    Vortex-refinement + AD model strategy
│   ├── case_template.json  Flow360 case JSON template
│   ├── run_matrix.py       Expands study1_matrix() -> case JSONs (--submit to send)
│   └── build_geometry.py   Drives serveCSM -batch per case
└── post/
    └── extract_stability.py   C_m(alpha), NP shift (synthesises until forces.csv exists)
```

---

## Quick start

```bash
# Parameter set + run matrix:
python params.py

# Emit Flow360 case JSONs:
python flow360/run_matrix.py

# Preview the stability table (synthetic until CFD runs):
python post/extract_stability.py
```

When ESP is sourced and the Flow360 SDK is installed:

```bash
source ~/esp/ESP129/EngSketchPad/ESPenv.sh

# Build one STEP per case:
python flow360/build_geometry.py

# Submit the run matrix:
python flow360/run_matrix.py --submit

# Render the geometry PNG (ESPenv pollutes PYTHONPATH for system Python):
env -u PYTHONPATH python3 geometry/render.py
```

---

## Study 1 run matrix

Steady-state RANS at $\alpha = 10^\circ$, $T/W = 0.5$, 12,000 ft.

| Case                              | gap_fraction | Z_tail (chords) | Role             |
|-----------------------------------|--------------|-----------------|------------------|
| `baseline_continuous_highT`       | 0.00         | +2.00           | Form / Named     |
| `proposed_gap_lowH`               | 0.35         |  0.00           | Formless / Nameless |
| `sweep_z{m0.50…p2.00}` (6 cases)  | 0.35         | −0.5 → +2.0     | Z_tail sweep     |

---

## Conventions

- Units: ft / slug / lbf / s.
- Origin at wing root c/4 on the symmetry plane; +X aft, +Y starboard, +Z up.
- Numbers live in `params.py`. Hard-coded dimensions elsewhere are a bug.
