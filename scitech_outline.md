# AIAA SciTech Paper Outline — Tsangpo eSTOL

> **Working title.** *The Useful Emptiness: A Distributed-Propulsion eSTOL
> Configuration Where an Inboard Flap Gap Concentrates Energy Onto the
> Horizontal Tail.*

> **Framing.**
> The 11th chapter of the *Tao Te Ching*:
>
> > *Thirty spokes share one hub. It is the empty space that makes the
> > wheel useful. Clay is shaped into a vessel; it is the hollow that
> > makes it useful.*
>
> Conventional STOL design is *Form* — the **Named, Rigid** path:
> continuous flaps to maximize $C_L$, an out-of-the-way T-tail to escape
> downwash. This paper argues that for a distributed-prop eSTOL the
> productive design is *Formless* — the **Nameless, Fluid** path: a
> deliberate inboard gap, a low H-tail bathed in upwash, and pitch
> authority bought by sacrificing a small fraction of total lift. The
> gap is the hollow that makes the configuration useful.

---

## 0. Mapping: Form vs. Formless → Configuration vs. CFD

| Aspect              | Form / Named / Rigid (C1)             | Formless / Nameless / Fluid (C4)        |
|---------------------|---------------------------------------|-----------------------------------------|
| Flap                | Continuous root → tip                 | Inboard gap, `gap_fraction = 0.35`      |
| H-tail location     | High T-tail, $Z_\text{tail} = +2.5\,c$ | Low H-tail, $Z_\text{tail} = 0\,c$       |
| Inboard prop wake   | Wasted onto fuselage / lost           | Threaded through gap, hits tail         |
| Pitch authority     | $C_{m_\alpha}$ from tail in clean flow | $C_{m_\alpha}$ amplified by $\eta_t > 1$ |
| Engineering claim   | Maximize $C_{L_{\max}}$                | Maximize **usable** low-speed control   |
| Risk                | Tail unresponsive at high $\alpha$    | Loss of ~10 % $C_L$ from gap            |

This table is the spine of the paper. Every section maps to one row.

---

## 1. Introduction

1.1 The eSTOL design problem at 12,000 ft density altitude.
1.2 Why a distributed array of 10 small propellers beats two large ones
    for blown-surface control authority (and why noise/Mach favors it too).
1.3 **Thesis statement.** *Configuration choices that look like geometric
    losses — a flap gap, a low tail — are net wins once the energetic
    field of the prop wake is treated as part of the airframe.*
1.4 Contribution list:
    - (a) A 10-prop / 165 ft² / 2,600 lb reference design point.
    - (b) A high-fidelity CFD study (Flow360 RANS + actuator disks) of the
      gap–tail interaction.
    - (c) A quantified Neutral-Point shift across the 2x2 design matrix.
    - (d) A control-authority "Slope-to-Sky" envelope enabling ramp takeoff
      and whip-stall landing.

---

## 2. Configuration & Parameter Set

2.1 Wing planform — Hershey bar, $S = 15.33$ m² ($\approx 165$ ft²),
    $\text{AR} = 8.0$, chord $c = 1.385$ m. Chordwise section is a
    3-element high-lift system: a **coved LS(1)-0417 main element**
    (Selig contour with a lower-cove cutout and filleted vertex), a
    **NACA 9621 vane**, and a **NACA 6311 aft flap**. The vane and
    aft flap move together as a rigid **Fowler assembly** about a
    pivot at $(0.55, -0.048)\,c$ (vane LE in stowed coordinates) — see
    `geometry/airfoils/estol_config.yaml`.
2.2 The 10-prop array. Spanwise stations at $y/b/2 \in \{0.1, 0.3, 0.5,
    0.7, 0.9\}$ per semi-span, disk loading at $T/W = 0.5$.
2.3 The H-tail — Hershey bar, $b_\text{ht} = 0.35\,b$,
    $\text{AR}_\text{ht} = 4.5$, **inverted LS(1)-0417** (negative
    camber gives down-force at $\alpha = 0$ without baking in an
    incidence angle), swept $Z_\text{tail}$.
2.4 The `gap_fraction` and $Z_\text{tail}$ parameters and their physical
    interpretation. `gap_fraction` is applied to the **vane + aft-flap
    assembly only**; the main element is continuous across the span.
2.5 All numbers cross-referenced to `params.py` (SI: m, kg, N, s, rad).

*Figure 2.1.* The per-phase geometry montage (planform + side section
× stowed / takeoff / landing), as emitted by `geometry/render.py`.

---

## 3. Method

3.1 Geometry pipeline. `geometry/airfoils/build_estol_geometry.py`
    reads `estol_config.yaml` and emits four OpenCSM UDC sketches
    (`main_wing`, `vane`, `flap`, `tail`) as smooth-spline closed
    planar contours, plus Selig `.dat` exports for plotting. ESP's
    `tsangpo.csm` ingests each UDC via `udprim $/airfoils/<name>`,
    swings it into the XZ plane (`rotatex 90`), applies the Fowler
    kinematics (`rotatez flap_rot_deg pivot_x pivot_y` then
    `translate flap_dx flap_dy`) in the normalized chord frame
    *before* chord scaling, then extrudes spanwise. Phase is a
    despmtr (`phase 0/1/2` = stowed / takeoff / landing).
3.2 Mesh: the live SI mesh strategy is encoded directly in the
    `submit_*.py` drivers, not in a separate document. Each prop
    disk gets a `fl.UniformRefinement` cylinder, the whole-aircraft
    pitch volume gets a `fl.RotationVolume` so $\alpha_\text{eff}$ is
    set by an active body rotation, and the H-tail gets a nested
    rotation zone for $\theta_\text{ht}$. y+ ≤ 1 on all walls
    (`boundary_layer_first_layer_thickness = 7.62 \times 10^{-6}$ m at
    cruise scale). The beta mesher is mandatory because the legacy
    mesher silently ignores `curvature_resolution_angle` and produces
    a poor LE on spline contours (see `flow360/LESSONS.md` and
    `CLAUDE.md`).
3.3 Flow360 setup: unsteady RANS with $\Delta t \gg c/V$ so each
    physical step is a quasi-steady solve at whatever rotation /
    thrust state is active (`fl.Steady` no-ops `Rotation` models —
    verified empirically; see `flow360/LESSONS.md`). 10 actuator-disk
    models per case sized for $T/W = 0.5$ at 12,000 ft. Parent case
    setup is in `flow360/submit_cruise.py`; the Study-1 2×2 driver
    will be modelled on it.
3.4 The Study 1 matrix (`params.study1_matrix`) — four cases:

    | Tag | label              | gap_fraction | $Z_\text{tail}/c$ | Role                                  |
    |-----|--------------------|--------------|-------------------|---------------------------------------|
    | C1  | Industry Baseline  | 0.00         | +2.50             | stable, heavy                         |
    | C2  | Downwash Failure   | 0.00         |  0.00             | unstable — low tail in downwash       |
    | C3  | Bad Trade-off      | 0.35         | +2.50             | stable, heavy, lift penalty           |
    | C4  | Proposed Synthesis | 0.35         |  0.00             | stable, lightweight, agile            |

3.5 Verification: one mesh-refinement triple on C4.

---

## 4. Results — *The Form*: C1 Industry Baseline

4.1 Global coefficients ($C_L, C_D, C_m$) at $\alpha = 10°$, $T/W = 0.5$.
4.2 Surface $C_p$ on wing and H-tail.
4.3 Streamlines: inboard prop wakes deflect over and around the T-tail,
    transmitting energy to no useful surface.
4.4 **Observation**: the T-tail rides in the wing+flap downwash,
    $\eta_t < 1$. Pitch authority is "rigidly named" but energetically
    empty.

---

## 5. Results — *The Formless*: C4 Proposed Synthesis

5.1 Global coefficients at the same flight condition. ~10 % $C_L$ loss
    versus C1 (the cost).
5.2 Streamlines: a coherent **upwash column** from Props 1 & 2 threads the
    gap and impinges on the H-tail lower surface.
5.3 Surface $C_p$ on the H-tail shows a high-q lobe co-located with the
    gap streamtube.
5.4 Local dynamic-pressure ratio $\eta_t = q_\text{tail}/q_\infty$ is
    mapped across the H-tail. Strip-averaged $\eta_t > 1.2$ over the gap.
5.5 **Observation**: the gap is the *hollow that makes the tail useful*.

*Figure 5.1.* Streamline visualization, C1 vs C4.
*Figure 5.2.* $C_p$ contours on the H-tail lower surface, C1 vs C4.

---

## 6. Results — The Non-Linear Coupling (C1 ↔ C2 ↔ C3 ↔ C4)

6.1 Headline: $C_{m_\alpha}$ for each of the four cases at the design
    point. C2 (gap = 0, low tail) is unstable; C4 (gap = 0.35, low tail)
    is *more strongly stable* than the heavy-T-tail baseline C1.
6.2 The **linearity check**: compute "$Z$ alone" from C1 → C2 and "gap
    alone" from C1 → C3, predict C4 by superposition, and compare to the
    measured C4. The residual is the energy-concentrator effect.
6.3 Neutral-point location vs. the (gap, $Z_\text{tail}$) cell. Identify
    the **sweet-spot** for the upwash-fed configuration.
6.4 Discuss the trade against ground-clearance / handling at flare.

This section is the engine of the paper; the post-processing module
`post/extract_stability.py` already prints the headline contrast and the
linear/non-linear decomposition.

---

## 7. The Stability Paradox (the Headline Nugget)

> *In traditional aero, a gap in the flap is a leak in lift. In our
> Tsangpo eSTOL, the gap is an energy concentrator. By sacrificing
> ~10 % of total $C_L$ we gain ~300 % in low-speed pitch authority,
> enabling the "Ramp" takeoff and "Whip-Stall" landing profiles.*

7.1 Quantify the trade: $\Delta C_L / \Delta C_{m_{\delta_e}}$ ratio.
7.2 The neutral-point shift $\Delta x_\text{np}/\text{MAC}$ between C1 and
    C4 at fixed $X_\text{tail}$.
7.3 Implication for usable CG envelope.

---

## 8. Maneuver Envelope: Slope-to-Sky

8.1 **Ramp takeoff**: rolling start uphill, rotation by tail-lift not
    elevator deflection; required $C_{m_{\delta_e}}$ vs. ground reaction.
8.2 **Whip-stall landing**: deliberate tail stall as flare-stop mechanism;
    requires tail recovery margin that the gap-fed upwash provides.
8.3 Map both maneuvers onto the $(C_L, C_m)$ envelope from § 6.

---

## 9. Discussion

9.1 Why this result is invisible to lifting-line / VLM with prescribed
    wake (the upwash column is a viscous-streamtube phenomenon).
9.2 Limits of the actuator-disk treatment; expectations from a future
    unsteady BET-disk run.
9.3 Acoustic margin: $M_\text{tip} \le 0.55$ at design point (deferred to
    Phase 5).
9.4 Where the *Form/Formless* metaphor stops being helpful (trim drag
    bookkeeping, structural attach).

---

## 10. Conclusion

The eSTOL configuration whose flaps and tail look *wrong* on a clean-aero
drawing is the one whose **energetic field** is correctly organized. The
Tsangpo vehicle's inboard flap gap is not absence; it is the channel
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

| Paper Item            | Backing Artifact                                        |
|-----------------------|---------------------------------------------------------|
| Fig 2.1 (geometry 2x2)| `geometry/render.py` → `geometry/out/tsangpo_geometry.png` |
| Fig 5.1 (streamlines) | Flow360 visualization → `post/` (TBD)                   |
| Fig 5.2 (tail Cp)     | Flow360 surface field → `post/` (TBD)                   |
| Fig 6.1 (NP shift)    | `post/extract_stability.py`                             |
| Table 6.1 (C_m_α 2x2) | `post/extract_stability.py` (stability.csv)             |
