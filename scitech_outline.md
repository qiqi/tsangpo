# AIAA SciTech Paper Outline — Himalayan eSTOL

> **Working title.** *The Useful Emptiness: A Distributed-Propulsion eSTOL
> Configuration Where an Inboard Flap Gap Concentrates Energy Onto the
> Horizontal Tail.*

> **Framing.**
> The 11th chapter of the *Tao Te Ching*:
>
> > *Thirty spokes share one hub. It is the empty space that makes the wheel
> > useful. Clay is shaped into a vessel; it is the hollow that makes it
> > useful.*
>
> Conventional STOL design is *Form* — the **Named, Rigid** path:
> continuous flaps to maximize $C_L$, an out-of-the-way T-tail to escape
> downwash. This paper argues that for a distributed-prop eSTOL the
> productive design is *Formless* — the **Nameless, Fluid** path: a
> deliberate inboard gap, a low H-tail bathed in upwash, and pitch authority
> bought by sacrificing a small fraction of total lift. The gap is the
> hollow that makes the configuration useful.

---

## 0. Mapping: Form vs. Formless → Configuration vs. CFD

| Aspect              | Form / Named / Rigid (Baseline)      | Formless / Nameless / Fluid (Proposed) |
|---------------------|--------------------------------------|----------------------------------------|
| Flap                | Continuous root → tip                | Inboard gap, `gap_fraction = 0.35`     |
| H-tail location     | High T-tail, `Z_tail = +2.0 c`        | Low H-tail, `Z_tail = 0.0 c`            |
| Inboard prop wake   | Wasted onto fuselage / lost           | Threaded through gap, hits tail        |
| Pitch authority     | $C_{m_\alpha}$ from tail in clean flow | $C_{m_\alpha}$ amplified by $\eta_t > 1$ |
| Engineering claim   | Maximize $C_{L_{\max}}$                | Maximize **usable** low-speed control  |
| Risk                | Tail unresponsive at high $\alpha$    | Loss of ~10 % $C_L$ from gap            |

This table is the spine of the paper. Every section maps to one row.

---

## 1. Introduction

1.1 The eSTOL design problem at 12,000 ft density altitude.
1.2 Why a distributed array of 10 small propellers beats two large ones for
    blown-surface control authority (and why noise/Mach favors it too).
1.3 **Thesis statement.** *Configuration choices that look like geometric
    losses — a flap gap, a low tail — are net wins once the energetic field
    of the prop wake is treated as part of the airframe.*
1.4 Contribution list:
    - (a) A 10-prop / 165 ft² / 2,600 lb reference design point.
    - (b) A high-fidelity CFD study (Flow360 RANS + actuator disks) of the
      gap–tail interaction.
    - (c) A quantified Neutral-Point shift versus tail vertical position.
    - (d) A control-authority "Slope-to-Sky" envelope enabling ramp takeoff
      and whip-stall landing.

---

## 2. Configuration & Parameter Set

2.1 Wing planform (S = 165 ft², AR = 8.0, MAC, taper, twist).
2.2 The 10-prop array. Spanwise stations, disk loading, T/W = 0.5.
2.3 The H-tail family: span = 0.35 b, swept $Z_{tail}$ and $X_{tail}$.
2.4 The `gap_fraction` parameter and its physical interpretation.
2.5 All numbers cross-referenced to `params.py`.

*Figure 2.1.* Planform with prop disks and H-tail footprint colored by which
prop wake threads the tail.

---

## 3. Method

3.1 ESP parametric geometry (`geometry/himalaya.csm`).
3.2 Mesh: vortex-refinement streamtubes from each prop disk to past the
    H-tail; y+ ≤ 1 on the H-tail.
3.3 Flow360 setup: steady-state RANS (SA), actuator-disk models per prop
    sized for $T/W = 0.5$ at 12 kft.
3.4 The Study 1 matrix (see `params.py::study1_matrix`):
    - **Baseline**: continuous flaps + high T-tail.
    - **Proposed**: inboard gap + low H-tail.
    - **Z_tail sweep**: 6 stations from −0.5 c to +2.0 c at the Proposed
      flap configuration.
3.5 Verification: one mesh-refinement triple on the Proposed case.

---

## 4. Results — *The Form*: The Baseline Configuration

4.1 Global coefficients ($C_L, C_D, C_m$) at $\alpha = 10°$, $T/W = 0.5$.
4.2 Surface $C_p$ on wing and H-tail.
4.3 Streamlines: inboard prop wakes deflect over and around the T-tail,
    transmitting energy to no useful surface.
4.4 **Observation**: the T-tail rides in the wing+flap downwash, η_t < 1.
    Pitch authority is "rigidly named" but energetically empty.

---

## 5. Results — *The Formless*: The Proposed Configuration

5.1 Global coefficients at the same flight condition. ~10 % $C_L$ loss
    versus Baseline (the cost).
5.2 Streamlines: a coherent **upwash column** from Props 1 & 2 threads the
    gap and impinges on the H-tail lower surface.
5.3 Surface $C_p$ on the H-tail shows a high-q lobe co-located with the gap
    streamtube.
5.4 Local dynamic-pressure ratio $\eta_t = q_{tail}/q_\infty$ is mapped
    across the H-tail. Strip-averaged $\eta_t > 1.2$ over the gap.
5.5 **Observation**: the gap is the *hollow that makes the tail useful*.

*Figure 5.1.* Side-by-side streamline visualization, Baseline vs Proposed.
*Figure 5.2.* $C_p$ contours on the H-tail lower surface, Baseline vs Proposed.

---

## 6. Results — Tail Vertical Position Sweep

6.1 Plots of $C_{m_\alpha}$, $C_{L_\alpha}$, $C_{D_0}$ versus $Z_{tail}$.
6.2 Neutral-point location versus $Z_{tail}$.
6.3 Identify the **sweet-spot** vertical position where the upwash column
    peaks on the H-tail. Compare it to the geometric centerline of the
    inboard prop disk.
6.4 Discuss the trade against ground-clearance / handling at flare.

---

## 7. The Stability Paradox (the Headline Nugget)

> *In traditional aero, a gap in the flap is a leak in lift. In our
> Himalayan eSTOL, the gap is an energy concentrator. By sacrificing ~10 %
> of total $C_L$ we gain ~300 % in low-speed pitch authority, enabling the
> "Ramp" takeoff and "Whip-Stall" landing profiles.*

7.1 Quantify the trade: $\Delta C_L / \Delta C_{m_\delta_e}$ ratio.
7.2 The neutral-point shift $\Delta x_{np}/\text{MAC}$ between Baseline and
    Proposed at fixed $X_{tail}$.
7.3 Implication for usable CG envelope.

---

## 8. Maneuver Envelope: Slope-to-Sky

8.1 **Ramp takeoff**: rolling start uphill, rotation by tail-lift not
    elevator deflection; required $C_{m_\delta_e}$ vs ground reaction.
8.2 **Whip-stall landing**: deliberate tail stall as flare-stop mechanism;
    requires tail recovery margin that the gap-fed upwash provides.
8.3 Map both maneuvers onto the $(C_L, C_m)$ envelope from § 6.

---

## 9. Discussion

9.1 Why this result is invisible to lifting-line / VLM with prescribed wake
    (the upwash column is a viscous-streamtube phenomenon).
9.2 Limits of the actuator-disk treatment; expectations from a future
    unsteady BET-disk run.
9.3 Acoustic margin: M_tip ≤ 0.55 at design point (deferred to Phase 5).
9.4 Where the *Form/Formless* metaphor stops being helpful (trim drag
    bookkeeping, structural attach).

---

## 10. Conclusion

The eSTOL configuration whose flaps and tail look *wrong* on a clean-aero
drawing is the one whose **energetic field** is correctly organized. The
Himalayan vehicle's inboard flap gap is not absence; it is the channel
through which the inboard propulsors feed the empennage. The clay vessel
is useful because of its hollow.

---

## Appendices

A. Full Run Matrix and Convergence Histories
B. Mesh Independence Study
C. Coordinate System and Sign Conventions
D. Reproducibility: commit hashes, container digests, and Flow360 case IDs.

---

## Cross-Reference Map (filled as we go)

| Paper Item            | Backing Artifact                             |
|-----------------------|----------------------------------------------|
| Fig 2.1 (planform)    | `geometry/himalaya.csm` → `post/plot_planform.py` |
| Fig 5.1 (streamlines) | `post/extract_streamlines.py`                |
| Fig 5.2 (tail Cp)     | `post/extract_tail_cp.py`                    |
| Fig 6.1 (NP shift)    | `post/extract_stability.py`                  |
| Table 4/5 (globals)   | `post/extract_global_coeffs.py`              |
