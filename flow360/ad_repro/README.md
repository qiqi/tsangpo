# Flow360 actuator-disk delivery — minimal reproducer

Self-contained reproducer for the discrepancy described in
`paper/figures/FLOW360_AD_BUG_REPORT.md`.

## What's in this folder

- `tsangpo_gapped.csm` — exact OpenCSM geometry used in the gap-low
  campaign.  Inlining of the airfoil UDCs has already been done.
- `submit_repro.py` — minimal Python script that:
  1. uploads the .csm as a Flow360 project,
  2. submits ONE case at the gap-low landing BO trim point
     ($\alpha=+8^\circ$, $\theta_{ht}=-6^\circ$, $T_{\mathrm{mult}}=12$,
     $V_\infty=12.86\,$m/s, ISA 12,000 ft),
  3. once the case completes, pulls `Disk_i_Force` from the actuator-
     disk output, multiplies by $\rho_\infty a_\infty^2 \approx 90250\,$N/m²,
     and prints commanded vs delivered thrust.

## Run

```bash
pip install flow360 numpy
python3 submit_repro.py                # submit + measure once complete
python3 submit_repro.py <case-id>      # skip submission, just measure
                                       # an existing case
```

Each case takes ~30 min on a single GPU node (release-25.9, beta
mesher).  The mesh refinement at the prop cylinders is
`PROP_REFINE_M = 0.025 c_wing = 0.0346 m` → octree-cast to 0.03125 m
(~34 cells across the disk diameter, ~4 cells axially through the
0.139 m cylinder thickness).

## Expected output

For the gap-low landing BO trim ($T_{\mathrm{mult}}=12$, fine mesh):

```
  commanded thrust per disk =   638.3 N       (730.31 N/m² × 0.874 m² annular)
                       total=  6383.2 N
  delivered thrust per disk ≈   415   N       (~ 65 % of commanded)
                       total≈  4150   N
  delivered / commanded     ≈ 0.65
```

The full delivered-vs-commanded matrix across all 4 configs × 3 phases
(both coarse 0.0625 m and fine 0.03125 m AD refinement) is in
`../../paper/figures/FLOW360_AD_BUG_REPORT.md` together with the case
IDs the original observations were taken from.

## What we're asking

In short:

1. Is `force_per_area.thrust` a face pressure jump (Pa)?  A body-force
   density?  A normalised coefficient?
2. What is `Disk_i_Force` (in the `actuator_disks` output) — integrated
   normal force, normalised force, or something else?
3. Does the solver apply a momentum-theory induced-velocity correction
   at high disk loading?  If so, what's the closed-form mapping from
   commanded to delivered?
4. Recommended practice to make delivered ≈ commanded for blown-lift
   applications.  Should we use `BETDisk` instead?  A different
   cylinder thickness : refinement ratio?

Happy to participate in a screen-share / debug call.

— Qiqi Wang
