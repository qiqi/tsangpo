# གཙང་པོ་ eSTOL — CFD ལས་གཞི་དང་ SciTech ཡིག་ཚགས་ཀྱི་མཛོད།

སྒྱུར་ཆས་ 10, ལྗིད་ཚད་ 11,565 N (≈ 2,600 lb), འདབ་མའི་ཁྱོན་ 15.33 m²
(≈ 165 ft²) ཡོད་པའི་ eSTOL བཀོད་པ་ཞིག་ཡིན། སྐབས་དང་པོའི་བཀོད་པའི་
གནད་དོན་ནི། *ནང་ཕྱོགས་ཀྱི་ཅིན་ཡི་ (flap) རྣམས་ཕྱིར་བཤིག་ནས་ H-མཇུག་དེ་
ནང་ཕྱོགས་སྒྱུར་ཆས་ཀྱི་འགྲིམ་རླབས་ནང་དུ་ཕབ་ཚེ་ཅི་ཞིག་འབྱུང་ངམ་ཞེས་པ་
འདི་ཡིན།*

ལས་གཞི་འདིས་ Flow360 RANS གྱིས་ SciTech ཡིག་ཚགས་ཀྱི་གཟབ་ཚད་ལ་སྲུང་
སྐྱོབ་བྱེད་འདོད་པའི་ལན་ནི། **བརྟན་ལྷིང་གི་འགལ་གཏམ་** (Stability Paradox)
ཞེས་པ་འདི་ཡིན།

> ཅིན་ཡི་ནང་གི་བར་སྟོང་ཞིག་ནི་ ཤེང་ལི་ (lift) ཡི་འཛག་ས་མིན། དེ་ནི་
> ནང་ཕྱོགས་སྒྱུར་ཆས་ཀྱིས་མཇུག་གི་ཡན་ལག་རྣམས་ལ་གསོ་སྐྱོང་གཏོང་སའི་
> ལམ་ཁ་ཡིན། $C_L$ ཡོངས་ཀྱི་ ~10 % བཏང་སྙོམས་སུ་སྤྱད་ནས། དལ་འགྲོས་ངང་
> ཁ་ཕྱོགས་འཛིན་པའི་སྟོབས་ཤུགས་ཆེན་པོ་ཐོབ་ཐུབ་ལ། འདི་ནི་ ཤེ་ཕོ་ (Ramp)
> ཡར་འཕུར་དང་ པན་ཤི་ཤི་སུ་ (Whip-Stall) མར་འབབ་ཀྱི་ཡུལ་སྐོར་གཉིས་
> ཀའི་སྒྲུབ་ཐུབ་པར་འདང་གི་ཡོད།

---

## ལས་གཞིའི་འཕེལ་རིམ།

ལས་གཞི་འདི་རིམ་པ་གསུམ་གྱིས་འཕེལ་ཡོད།

1. **བརྟན་པོའི་ aero ལས་གཞི།** — cruise / takeoff / landing གི་ སའོ་མེའོ་
   (sweep) རྣམས། SciTech ཡིག་ཚགས་ཀྱི་ཆེད་དུ། *ལེགས་གྲུབ།* འབྲས་བུ་ནི་
   `post/v2_gapped/TRIM_RESULTS.md` དང་ `paper/` ནང་ཡོད།
2. **རང་འཕུར་ phugoid།** — UDD (User-Defined Dynamics) ཡིས་ CFD ཡི་
   ཤུགས་ནས་ longitudinal 3-DOF (x, z, pitch) ཀྱི་འགུལ་སྐྱོད་སྒྲིག་ནས།
   sliding-mesh སྐྱོར་སྒྱུར་ཁྱམས་ (rotation zone) བཞི་ལ་ཕྱིར་སྤྲོད་བྱེད།
   *2026-05-27 ལེགས་གྲུབ་ཅིང་ར་སྤྲོད་ཟིན།* phugoid གི་དུས་ཡུན་ T = 17.4 s
   (ཚོད་དཔག་ 18.6 s), ζ ≈ 0.026.
3. **མར་འབབ་ཀྱི་ controller།** — དམིགས་ཡུལ། UDD ཞིག་གིས་གནས་སྟངས་ལས་
   ཚོད་འཛིན་གྱི་ནང་འཇུག་སྤྲོད་ནས་ CFD ནང་དུ་མར་འབབ་འཕུར་སྐྱོད་བྱེད་པ།
   *སྔོན་འགྲོ་ལེགས་གྲུབ། controller མ་འགོ་བཙུགས།* 2026-05-29 ལ་མར་འབབ་ཀྱི་
   trim གྱི་ open-loop འགུལ་སྐྱོད་བརྟག་དཔྱད་བྱས་ཟིན། trim དེ་བརྟན་པོ་མིན་
   (∂CMy/∂θ > 0, θ ∈ [−5°, +30°])། kick མེད་པར་ 9 s ནང་ θ = +98° བར་
   མགོ་ཡར་འཁོར། θ_ht retrim (−3.67° ནས་ +3°) གིས་སྐྱོན་སེལ་མི་ཐུབ།
   ཞིབ་ཕྲ་ `post/v3/phugoid_runs/landing/OPEN_LOOP_LANDING.md` དང་
   θ_eq(θ_ht, V_inf) གི་དྲ་མིག་ `pitch_eq_grid.csv` ལ་གཟིགས།

## ཀློག་པའི་གོ་རིམ།

གནས་མཛོད་འདི་ཐོག་མར་ཀློག་མཁན་ (མི་དང་ agent གཉིས་ཀ) གིས་གོ་རིམ་འདི་
ལྟར་ཀློག་དགོས།

1. `CLAUDE.md` — ལས་གཞིའི་ཚུལ་ལུགས་དང་ཡིག་ཆ་གང་དུ་ཡོད་པའི་ས་བཀྲ།
2. `flow360/PHUGOID_UDD_POSTMORTEM.md` — **ཡིག་ཆ་གལ་ཆེ་ཤོས།** UDD
   ཡིས་འགུལ་སྐྱོད་སྒྲིག་པའི་ལམ་ལུགས། རྩ་བའི་སྐྱོན་གཉིས་ (§4, §5) དང་
   ལག་ལེན་གྱི་ཐོ་གཞུང་ (§8)།
3. `flow360/LESSONS.md` — Flow360 གི་གྱོང་གུད་ཀྱི་ལོ་རྒྱུས། (case ID
   དང་བཅས།)
4. `flow360/unsteady_setup.py` — སྐྱོར་སྒྱུར་ཁྱམས་བཞིའི་ topology དང་
   UDD བཞིའི་སྦྱོར་སྒྲིག (`mode='warmup'` དང་ `mode='phugoid'` གཉིས་
   དང་ 1-DOF `pitch_step_cyl3_integrated` ལྷག་ཡོད། སྔོན་གྱི་ `pitch_*`
   modes གཞན་བཞི་ commit c263a1d ནས་ཕྱིར་ལེན་ཐུབ།)
5. `cfd_setup.py` དང་ `params.py` — བརྟན་པོའི་ SimulationParams དང་
   ཚད་འཇོག་གི་མཛོད་ཁང་།
6. `flow360/ADAPTING_TO_TWO_BODY_SEPARATION.md` — ལུས་གཉིས་ཁ་འབྲལ་
   (stage separation) ལ་བཀོལ་སྤྱོད་བྱེད་པའི་ལམ་སྟོན། ཕྱི་མིས་ཀློག་
   རྒྱུར་དམིགས།

---

## གནས་མཛོད་ཀྱི་བཀོད་པ།

```
.
├── CLAUDE.md               ལས་གཞིའི་ཚུལ་ལུགས་དང་ས་བཀྲ། (ཐོག་མར་ཀློག)
├── params.py               ཚད་འཇོག་གི་མཛོད་ཁང་གཙོ་བོ (SI: m, kg, N, s, rad; ISA; non-dim → SI)
├── cfd_setup.py            བརྟན་པོའི་ SimulationParams སྦྱོར་བྱེད། (actuator disk, སྐྱོར་སྒྱུར་ཁྱམས་ 2)
├── wu-wei.mdc              གནས་ལུགས། ཡང་དག་པར་བྱེད་པའི་ཆ་ཉུང་ཤོས་ལ་སྦྱོར་བ།
├── geometry/
│   ├── airfoils/            3-ཡན་ལག་མཁའ་འདབ། estol_config.yaml + build_estol_geometry.py
│   ├── tsangpo.csm          v2 རྒྱུན་མཐུད་ཅིན་ཡི། phase 0/1/2 → ཡུ་འདུག/ཡར་འཕུར/མར་འབབ།
│   ├── tsangpo_gapped.csm   v2 བར་སྟོང་ཅིན་ཡི།
│   ├── tsangpo_*_high_htail.csm   H-མཇུག་མཐོན་པོའི་ཁྱད་པར།
│   ├── tsangpo_v3.csm       v3 boom ཐུང་བ། (phugoid ལ་སྤྱད་པ)
│   ├── render*.py           STL → PNG
│   └── README.md            ESP ཚད་འཇོག་གི་ཐོ་གཞུང་།
├── flow360/
│   ├── PHUGOID_UDD_POSTMORTEM.md   UDD འགུལ་སྐྱོད་ཀྱི་ལམ་སྟོན། (ཡིག་ཆ་གལ་ཆེ་ཤོས)
│   ├── LESSONS.md                  Flow360 གི་གྱོང་གུད་ཀྱི་ལོ་རྒྱུས།
│   ├── ADAPTING_TO_TWO_BODY_SEPARATION.md  ལུས་གཉིས་ཁ་འབྲལ་ལ་བཀོལ་སྤྱོད་ཀྱི་ལམ་སྟོན།
│   ├── AGENT_USABILITY_REPORT_v2.md  SDK ལ་ agent གིས་འཕྲད་པའི་དཀའ་ངལ།
│   ├── unsteady_setup.py           སྐྱོར་སྒྱུར་ཁྱམས་ 4 + UDD 4 (phugoid གི་རྨང་གཞི)
│   ├── submit_unsteady_warmup.py   phugoid གི་ warmup (དྲ་མིག་སྦྱོར་བ)
│   ├── submit_gap110_nokick_diag.py       phugoid chunk 1 (warmup ནས་ fork)
│   ├── submit_gap110_chunkN_nokick_diag.py phugoid chunk N (chunk N-1 ནས་ fork)
│   ├── submit_landing_*.py         མར་འབབ་ཀྱི་ warmup / chunk (θ_ht retrim ལྔ)
│   ├── run_landing_phugoid_chain.py  མར་འབབ་ chunk འབྲེལ་མཐུད་ཀྱི་ orchestrator
│   ├── pitch_eq_grid.py            1-DOF pitch-only θ_eq(θ_ht, V_inf) དྲ་མིག
│   ├── ADAPTING_TO_TWO_BODY_SEPARATION.md  ལུས་གཉིས་ཁ་འབྲལ་ལ་བཀོལ་སྤྱོད་ཀྱི་ལམ་སྟོན།
│   ├── submit_cruise.py / submit_v3.py    བརྟན་པོའི་ parent གནད་དོན།
│   ├── submit_*_sweep*.py / submit_*_campaign.py   བརྟན་པོའི་ སའོ་མེའོ།
│   └── submit_fine_ad_parents.py   actuator-disk དྲ་མིག་ཞིབ་མོའི་ཚོད་ལྟ།
├── post/
│   ├── _phase_plot.py, cg_shift_trim.py
│   ├── v1/                  v1 (2026-05) ཁོ་ནའི་ལོ་རྒྱུས། PLAN_v1.md, SCITECH_OUTLINE_v1.md, SENSITIVITIES.md
│   ├── v2_gapped/           v2 བར་སྟོང་ཅིན་ཡིའི་ཐིག་གཞི་དང་ TRIM_RESULTS.md
│   ├── v3/                  phugoid གི་ཐིག་གཞི་ (plot_running_case, viz_nokick_lagfix,
│   │                        render_phugoid_slices, render_phugoid_3d_{lic,native})
│   │                        + unsteady_project_ids.json + mp4/png
│   │   └── phugoid_runs/landing/   OPEN_LOOP_LANDING.md, pitch_eq_grid.csv, mp4/png
│   └── FLOW360_GAI_BUG_REPORT.md
├── share/udd_starter/       UDD rigid-body ལམ་སྟོན་སྤྱིར་བཏང་ + དཔེ་མཚོན་ script གསུམ།
└── paper/                   SciTech extended abstract + figures (paper/build.sh)
```

**2026-10-30 ནས་ GitHub ཆ་ཚང་ཡིན།** 2026-05-28 ནས་ 06-12 བར་གྱི་ commit 52
(མར་འབབ་ཀྱི་ open-loop ལས་གཞི། Euler–Cromer integrator། `share/udd_starter/`།
3D animation) 014-v100-dev ནས་ GitHub ལ་ཕུལ་ཟིན། animation གི་ frame PNG
(1.25 GB) ཁོ་ན་བསྐྱུར། mp4 རྣམས་ཡོད།

---

## མགྱོགས་པོར་འགོ་འཛུགས་པ།

```bash
# ཚད་འཇོག་ཡོངས་ཀྱི་མདོར་བསྡུས་སྔོན་ལྟ
python params.py

# UDC / .dat / sketch PNG སྦྱོར་བ
python geometry/airfoils/build_estol_geometry.py
```

ESP ཁུངས་སུ་བཀོད་ཟིན་ལ་ Flow360 SDK (v25.9.x ཡན) བཙུགས་ཟིན་པའི་སྐབས།

```bash
source ~/esp/ESP129/EngSketchPad/ESPenv.sh

# phase 0/1/2 རེ་རེའི་གཟུགས་སྦྱོར་བ (ས་གནས་སུ་ལྟ་བའི་ཆེད། Flow360 ལ་ .csm ཐད་ཀར་ཕུལ)
for p in 0 1 2; do
    name=$(printf "stowed\ntakeoff\nlanding\n" | sed -n "$((p+1))p")
    mkdir -p geometry/out/$name
    echo "phase $p" > geometry/out/$name/phase.despmtrs
    ln -snf "$PWD/geometry/airfoils" geometry/out/$name/airfoils
    (cd geometry/out/$name && \
     serveCSM ../../tsangpo.csm -batch -despmtrs phase.despmtrs -outLevel 0)
done

# ---- བརྟན་པོའི་ལས་གཞི ----
python flow360/submit_cruise.py                 # parent cruise གནད་དོན
TSANGPO_PARENT_CASE_ID=case-... python flow360/submit_alpha_sweep.py
TSANGPO_PARENT_CASE_ID=case-... python flow360/submit_htail_sweep.py
TSANGPO_PARENT_CASE_ID=case-... python flow360/submit_thrust_sweep.py

# ---- phugoid (unsteady, UDD) ----
python flow360/submit_unsteady_warmup.py        # ཁྱམས་བཞི་ trim ལ་བརྟན་པོར་བཞག་ནས་དྲ་མིག་ + བརྟན་པོའི་འཁོར་སྐྱོད
python flow360/submit_gap110_nokick_diag.py [--project-id prj-... --parent-case-id case-...]
                                                # chunk 1: warmup ནས་ fork, UDD སློང་བ (200 × 0.05 s)
python flow360/submit_gap110_chunkN_nokick_diag.py <parent_case_id> <N> [--project-id prj-...]
                                                # chunk N: chunk N-1 ནས་ fork
```

phugoid གི་ chunk རེ་རེ་ནི་སྔ་མའི་ chunk ནས་ fork བྱེད་པ་ཡིན། Flow360 ཡིས་
fork སྐབས་ UDD ཡི་ state ཉར་ཚགས་བྱེད་པས་འགུལ་སྐྱོད་རྒྱུན་མཐུད་ཡིན།

---

## Solver གི་པར་གཞི་དང་སྐྱོར་སྒྱུར་ཚིག་གཅིག་གི་ཐད།

- གནད་དོན་ཚང་མ་ Flow360 **release-25.9** (SDK v25.9.x, beta mesher) ཐོག་
  འཁོར་བ་ཡིན།
- སྐྱོར་སྒྱུར་ཁྱམས་བཞི་པོ་ (shoulder, elbow, airframe, htail) ཚང་མའི་
  ཚིག་ (axis) ནི་ `(0, 1, 0)` ཡིན་ཏེ་ **མཉམ་འགྲོ་** (parallel) ཡིན།
  `cfd_setup.py` ནང་གི་ཁྱམས་གཉིས་ཀྱང་དེ་བཞིན།
- **ཚིག་མི་མཉམ་པའི་ནང་ཁྱམས་ཀྱི་སྐྱོན།** (25.12 ལ་བཅོས་ཟིན): 25.12 སྔོན་གྱི་པར་གཞིར། ནང་གི་ཁྱམས་
  ཤིག་ཕ་མའི་ཁྱམས་དང་ཚིག་མི་མཉམ་པར་སྐྱོར་སྒྱུར་བྱེད་ཚེ། གོམ་པ་རེ་རེའི་
  གཟུགས་ཀྱི་གསར་བཅོས་ (ངོས་ཀྱི་ normal, gradient, wall distance) ཀྱིས་
  ནང་གི་ཁྱམས་ཀྱི་ཚིག་རྙིང་པ་སྤྱོད། node ཀྱི་གནས་ས་འགྲིག་ཀྱང་ metric
  དལ་བུར་འཁྱོག ལས་གཞི་འདི་ཚིག་མཉམ་འགྲོ་ཡིན་པས་སྐྱོན་དེ་ལ་མ་འཕྲད།
- **roll ཡང་ན་ yaw ཞིག་ pitch ནང་དུ་འཛུད་ན་ (6-DOF ཆ་ཚང་) 25.12 ཡན་
  ངེས་པར་སྤྱོད་དགོས།**

---

## ལས་གཞིའི་ད་ལྟའི་གནས་སྟངས། (2026-10-30)

| རིམ་པ | འབྲས་བུ | གནས་སྟངས |
|------|---------|----------|
| v2 cruise / takeoff / landing སའོ་མེའོ (རྒྱུན་མཐུད་ + བར་སྟོང་, H-མཇུག་དམའ་ + མཐོ) | `post/v2_gapped/TRIM_RESULTS.md`, `paper/figures/` | ལེགས་གྲུབ |
| v3 boom ཐུང་བའི་ trim | `flow360/submit_v3.py`, `paper/figures/v3_combined_sens.png` | ལེགས་གྲུབ |
| actuator-disk ཤེད་ཀྱི་མི་མཐུན་པ | `paper/figures/FLOW360_AD_DELIVERY_RESOLVED.md` | ཐག་བཅད (ཚད་འཇལ་གྱི་ཚད་གཞིའི་ནོར་འཁྲུལ, solver གི་སྐྱོན་མིན) |
| phugoid རང་འཕུར (v3 cruise, 60 s) | `PHUGOID_UDD_POSTMORTEM.md` | ལེགས་གྲུབ, T = 17.4 s, ζ ≈ 0.026 (explicit Euler; 2026-05-28 ནས་ Euler–Cromer) |
| མར་འབབ་ trim གྱི་ open-loop འགུལ་སྐྱོད | `post/v3/phugoid_runs/landing/OPEN_LOOP_LANDING.md` | བརྟག་དཔྱད་ཟིན (2026-05-29): བརྟན་པོ་མིན, 9 s ནང་ θ = +98° |
| θ_eq(θ_ht, V_inf) 1-DOF དྲ་མིག (15 cell) | `flow360/pitch_eq_grid.py`, `.../landing/pitch_eq_grid.csv` | ལེགས་གྲུབ (2026-06-02) |
| UDD starter pack + 3D animation | `share/udd_starter/`, `post/v3/render_phugoid_3d_native.py` | ལེགས་གྲུབ (2026-06-12) |
| མར་འབབ་ཀྱི་ controller | — | མ་འགོ་བཙུགས |

---

## གོམས་སྲོལ།

- ཚད་གཞི། SI: m / kg / N / s / rad. UDD ནང་དུ་ནི་ solver གི་ non-dim
  ཁོ་ན (`params.nd_force_to_si` སོགས་ལ་གཟིགས)།
- ཐིག་ཁུངས་ནི་ **CG** ཡིན (v2 ནས)། +X རྒྱབ། +Y གཡས། +Z སྟེང་། སྐྱོར་སྒྱུར་
  +θ = མགོ་ཡར (nose up)།
- གྲངས་ཐོ་རྣམས་ `params.py` ནང་དུ་ཡོད། གཞན་ས་གང་དུ་ཡང་ཚད་འཇོག་འཁོད་
  ཡོད་ན་སྐྱོན་ཆ་ཡིན། ρ, a ཡང་ `params.isa_atmosphere` ནས་ལེན་དགོས་ལ།
  ནམ་ཡང་མཐོ་ཚད་ངེས་ཅན་ཞིག་གི་གྲངས་ཀ་ཐད་ཀར་མ་འབྲི།
- Flow360 ལ་ `use_beta_mesher=True` ངེས་པར་སྦྱོར། གནད་དོན་རེ་རེར་ y=0
  `SliceOutput` ངེས་པར་འཇོག (`CLAUDE.md`)།

### 3-ཡན་ལག་མཁའ་འདབ་ཀྱི་མཛོད།

| ཡན་ལག        | མཁའ་འདབ                                | སྦྱོར་ལམ                                  |
|--------------|------------------------------------------|---------------------------------------------|
| `main_wing`  | LS(1)-0417 ལ་ ཨའོ་ཚའོ་ (cove) བཀར་པ      | འདབ་མ་ཡོངས་ལ་རྒྱུན་མཐུད, B-spline ཡང་ཐའོ |
| `vane`       | NACA 9621                                | ཕུ་ལེ་ (Fowler) གཞན་སྦྱོར་གྱི་ཡན་ལག །       |
| `aft_flap`   | NACA 6311                                | vane དང་མཉམ་དུ་གཞན་སྦྱོར།                  |
| `htail`      | LS(1)-0417 ལྡོག་པ                        | $\alpha = 0$ ལ་མར་ཤེད་ (downforce) སྟེར།   |

vane དང་ aft_flap གཉིས་ནི་ ཕུ་ལེའི་ཡན་ལག་གཅིག་ལྟར་སྐྱོར་སྒྱུར་དང་
ས་སྤོ་བྱེད་པ་ཡིན། ཕུ་ལེའི་ phase ནི་ `phase` despmtr-ཀྱིས་ངེས་པ་
(0=ཡུ་འདུག, 1=ཡར་འཕུར, 2=མར་འབབ); ངེས་ཚད་རྣམས་
`estol_config.yaml`-ནང་གནས་ཡོད།

### ESP / OpenCSM གི་གྱོང་གུད (ཉམས་མྱོང་ངན་པ་ལས་ཤེས་པ)

འདི་རྣམས་ `geometry/tsangpo*.csm` ལ་ཀློག་པའམ་ཞུ་བཅོས་བྱེད་སྐབས་སུ་
སོ་རྒྱག་པ་ཡིན།

- **`capsGroup` ངེས་པར་སྦྱོར།** ཡན་ལག་རེ་རེའི་ `extrude` རྗེས་སུ་
  `select face / attribute capsGroup $<name>` ངེས་པར་གཏོགས་དགོས།
  body0000N གི་གོ་རིམ་ནི་བརྟན་པོ་མིན། Flow360 ངོས་ནས་
  `geo.group_faces_by_tag("capsGroup")` གིས་ ངོས་གཟུགས་སུ་འགོད།
- **MARK / RULE།** `MARK` གི་སྡེབ་སྒྲོམ་ནི་རྗེས་སུ་འབྱུང་བའི་ `RULE`,
  `BLEND` ཡང་ན་ `LOFT` གིས་མཇུག་སྒྲིལ་བ་ཡིན། དེའི་ནང་དུ་ `END`
  སོ་རྐྱང་གཅིག་ཀྱང་མི་འགོད།
- **`rotatey angDeg zaxis xaxis`** — pivot ཀྱི་ཡིག་སྡེབ་ནི་ (z, x),
  (x, z) མིན་པར་ `OpenCSM.c` ནང་འཁོད་པ་བཞིན་ཡིན།
- **ཕུ་ལེ་སྦྱོར་གྱི་གོ་རིམ།** vane + aft_flap ལ་སྐྱོར་སྒྱུར་
  (`rotatez`) དང་ས་སྤོ་ (`translate`) ནི་ ཤན་ཚད་མཐུན་ (normalized)
  ངོས་སུ་ངེས་པར་སྦྱོར་དགོས་ལ། `scale wing_chord` སྔོན་ལ་སྦྱོར།
- **`set` ནང་དུ་མིང་ཡོད་པའི་འགྱུར་གྲངས་གཉིས་ `+`/`-` གིས་མ་སྦྲེལ།**
  OpenCSM གིས་ཁ་རོག་ངང་གྲངས་ནོར་བ་སྟེར། (`LESSONS.md` ལ་གཟིགས།)
- **beta mesher ངེས་པར་སྤྱོད།** legacy mesher ཡིས་
  `curvature_resolution_angle` ལ་ལན་མི་གཏོང་ལ། ཆན་ཡུན་ (leading edge)
  གི་ངོས་གཟུགས་ངན།
- **ཡིག་ཆ་གཅིག་ཁོ་ནའི་ inlined CSM ཕུལ་དགོས།** STEP མང་པོ་ཕུལ་ཚེ་
  `body00001` ཁོ་ན་ལ་དྲ་མིག་སྦྱོར་གི། `cfd_setup.inline_udcs()` སྤྱོད།

---

## English postscript — for readers adapting this to another problem

Everything above is in Tibetan by the author's choice; this section is the
minimum an outside reader (human or agent) needs.

**What this repo demonstrates.** Closed-loop rigid-body motion inside
Flow360: four nested sliding-mesh rotation zones driven by four lockstep
`UserDefinedDynamic` blocks that integrate longitudinal 3-DOF (x, z, pitch)
from the CFD wall forces. Translation is obtained from rotations via a
two-link "robot arm" (shoulder + elbow cylinders, inverse kinematics in the
UDD). Result: a validated free-flight phugoid, T = 17.4 s, ζ ≈ 0.026.

**Read in this order.** `CLAUDE.md` → `flow360/PHUGOID_UDD_POSTMORTEM.md`
(§4, §5, §8 are the hard-won rules) → `flow360/LESSONS.md` →
`flow360/unsteady_setup.py` (`mode='warmup'`, `mode='phugoid'` and the 1-DOF
`pitch_step_cyl3_integrated`; the other four `pitch_*` diagnostics are
recoverable from commit c263a1d) →
`cfd_setup.py`, `params.py`. If you are here for a two-body separation, read
`flow360/ADAPTING_TO_TWO_BODY_SEPARATION.md` next: it states which topologies
the nested-cylinder constraint allows and the two open questions to settle
with the Flow360 team before authoring.

**Reusable.** The rotation-chain topology and interface-gap sizing rule
(`unsteady_setup.py` §1); the per-zone lockstep-UDD pattern
(`_build_per_zone_udds`); the non-dim conversions; every rule in the
postmortem (gate at `pseudoStep == 0`, sequential `update_law` evaluation,
lag-correct lever arm, publish `omega` with `theta`, world-frame forces at
`moment_center`, one `output_target` per UDD, ≤10 state slots, no `atan2`);
body forces (actuator disks, engines) are NOT in the wall integral and must
be added by hand.

**Tsangpo-specific, replace wholesale.** `params.py`, the ESP/OpenCSM
airfoil pipeline under `geometry/`, the Flow360 folder names in
`cfd_setup.py` (the IDs themselves come from an optional, gitignored
`flow360_folders.json`), every `submit_*.py` except the phugoid and landing
chains and `pitch_eq_grid.py`.

**Solver version.** All runs used release-25.9 with all rotation axes
parallel (`(0, 1, 0)`). A solver bug fixed in Flow360 25.12 corrupts per-step
geometry metrics when a nested zone rotates about a different axis from its
parent. Any 6-DOF extension (roll or yaw nested inside pitch) must run on
25.12 or later.

**What the 2026-10-30 merge brought in, and what it changes.** The 52
commits from the author's workstation (2026-05-28 to 06-12) are now on
GitHub, minus 1.25 GB of regenerable animation frames. Three things an
adapter must know. (1) The UDD integrator is now symplectic Euler–Cromer
with velocity slots first: `[0] Vx, [1] Vz, [2] q, [3] x, [4] z, [5] θ,
[6,7] link angles`. The postmortem's slot numbers (x in slot 0, q in slot 5,
"read dynamics from Δstate5/dt") describe the explicit-Euler layout the
published cruise phugoid ran with; the rules are unchanged, the indices are
not. (2) Two fixes the pre-merge snapshot lacked: the actuator-disk
cylinders are no longer listed in `enclosed_entities` (the mesher rejected
them, so new meshes could not be built), and the phugoid initial state now
includes `gamma_trim`, so takeoff and landing phases no longer snap by 30° on
the first step. One related defect is documented but not fixed: with a
non-zero trim flight-path angle the frozen warmup sees the wrong angle of
attack, so landing chunk 1 starts with a transient (LESSONS.md, "Landing
warmup convention flaw"). (3) The landing trim is open-loop unstable; the 1-DOF mode
`pitch_step_cyl3_integrated` with `quasi_static_q=True` is the production
tool for mapping θ_eq(θ_ht, V_inf). `share/udd_starter/README.md` is the
generalized how-to. Everything the phugoid chain needs is here: the two
`submit_gap110_*` fork scripts default to the IDs in
`post/v3/unsteady_project_ids.json` (the author's cloud projects) and take
`--project-id` / `--parent-case-id` to fork from a warmup you ran yourself.
