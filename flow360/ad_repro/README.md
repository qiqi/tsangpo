# Flow360 actuator-disk delivery — minimal reproducer

A single script that submits **two** Flow360 cases at the same trim
point with the same `force_per_area` AD specification — the only
difference is the `UniformRefinement` spacing on the prop cylinders.
Pulls `Disk_i_Force` from each completed case and prints
commanded vs delivered thrust.

## What the two cases do

| case | `PROP_REFINE` | octree-cast | ~ cells across disk Ø | observed `F_AD/commanded` |
|------|---------------|-------------|------------------------|---------------------------|
| A (coarse) | `0.05 c_wing` ≈ 0.069 m | **0.0625 m** | ~17 | **1.10** |
| B (fine)   | `0.025 c_wing` ≈ 0.035 m | **0.03125 m** | ~34 | **0.65** |

Same trim ($\alpha=+8^\circ$, $\theta_{ht}=-5^\circ$,
$T_{\mathrm{mult}}=16$, $V_\infty=18\,$m/s, ISA 12,000 ft), same
geometry (10-prop / 11.07 m / 165 ft² distributed-electric uSTOL),
same constant-pressure-jump `force_per_area.thrust = 973.74 N/m²` over
annular radius `[0.080, 0.534] m`.  Commanded thrust =
`fpa × π·(R² − r²) = 851.1 N/disk → 8511 N total`.

## Get the geometry

The .csm lives in our public-ish repo:

```
https://github.com/qiqi/tsangpo
```

The single file you need is at:

```
https://raw.githubusercontent.com/qiqi/tsangpo/main/geometry/tsangpo.csm
```

```bash
wget -O tsangpo.csm \
   https://raw.githubusercontent.com/qiqi/tsangpo/main/geometry/tsangpo.csm
```

(If that 404s because the repo is private, ping Qiqi and we'll either
flip it public or paste a Drive link.)

## Run

```bash
pip install flow360 numpy
python3 submit_repro.py                       # submits BOTH cases
# (~30 min meshing + solver per case; can run in parallel on cloud)
```

Once both cases are COMPLETED, re-invoke with the two case IDs to
print just the comparison:

```bash
python3 submit_repro.py <case-coarse> <case-fine>
```

## Expected output

```
Reference: commanded thrust per disk = 851.1 N → total 8511 N

  --- COARSE (PROP_REFINE = 0.05 c_w): ad_repro_coarse_case  status=COMPLETED ---
    commanded total =  8511.0 N
    delivered total =  9370.0 N
    delivered / commanded = 1.100

  --- FINE   (PROP_REFINE = 0.025 c_w): ad_repro_fine_case  status=COMPLETED ---
    commanded total =  8511.0 N
    delivered total =  5500.0 N
    delivered / commanded = 0.646
```

So with the same constant-pressure-jump spec, refining the prop mesh
swings the integrated `Disk_i_Force` from 110 % to 65 % of the
commanded $p \times A_{\mathrm{annular}}$.

## Companion documents

- `paper/figures/FLOW360_AD_BUG_REPORT.md` — full 12-case (4 configs × 3
  phases) delivered-vs-commanded matrix on coarse vs fine mesh,
  including all project + case IDs, hypotheses ruled in / out, and
  the specific questions we'd like guidance on.
- `flow360/AGENT_USABILITY_REPORT_v2.md` — Flow360 SDK pain points that
  bit us while investigating this (case-ID mutation, no public mesh-
  log accessor, etc.).

— Qiqi Wang
