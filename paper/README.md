# SciTech 2027 paper

## Deadlines

- **Abstract / extended abstract**: 21 May 2026, 8:00 PM ET
- Author notifications: 24 Aug 2026
- **Final manuscript**: 1 Dec 2026, 8:00 PM ET

(Confirm at https://scitech.aiaa.org/call-for-content/call-for-papers/ — dates are subject to change.)

## Build

```
cd paper
pdflatex extended_abstract.tex
bibtex extended_abstract
pdflatex extended_abstract.tex
pdflatex extended_abstract.tex
```

Current template is plain `article`. To switch to the AIAA template, drop
`new-aiaa.cls` into the folder and change the `\documentclass` line.

## TODOs for the extended abstract

- [ ] Fill the `\todo{...}` placeholders with the actual SM / sensitivity numbers from `post/out/v2_continuous_high/*` once those sweeps finish. cont-high cruise is mostly done; takeoff and landing still finishing as of 2026-05-15.
- [ ] Replace `figures/upwash_mechanism_placeholder.png` with the actual y-slice CFD figure once the gap40 slice forks land (slice forks submitted 2026-05-15; ~97 cases).
- [ ] Confirm the Electra reference — looking for the most-citable SciTech / Aviation paper on EL2/EL9 blown-lift flight test.
- [ ] Drop in `new-aiaa.cls` and re-render.
- [ ] Author list / affiliations / co-author confirmations.

## Story arc (one paragraph each, mapped to sections)

1. **Intro.** uSTOL + DEP; Electra; the empennage problem.
2. **Method.** 4 configs × 3 phases; Flow360 RANS + actuator disks.
3. **cont-high.** Necessary but not sufficient — landing still unstable; htail stall = nose-diver.
4. **Flap gap.** Stability fix via flap-edge vortices → centerline upwash. Costs ~10% $C_{L,\max}$, tiny cruise drag win.
5. **gap-low.** Htail back in slipstream — both stability and authority roughly double.
6. **v3.** Shortened tail boom: aggressive flare, shorter landing, easier go-around.
7. **Full paper.** Phugoid dynamics (3 configs) + ground-effect landing CFD.
