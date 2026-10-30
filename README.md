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
   *མ་འགོ་བཙུགས།*

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
   UDD བཞིའི་སྦྱོར་སྒྲིག (`mode='warmup'` དང་ `mode='phugoid'` ཁོ་ན་
   གལ་ཆེ། `pitch_*` modes རྣམས་ནི་སྐྱོན་འཚོལ་གྱི་ཤུལ་ལྷག་ཡིན།)
5. `cfd_setup.py` དང་ `params.py` — བརྟན་པོའི་ SimulationParams དང་
   ཚད་འཇོག་གི་མཛོད་ཁང་།

---

## གནས་མཛོད་ཀྱི་བཀོད་པ།

```
.
├── CLAUDE.md               ལས་གཞིའི་ཚུལ་ལུགས་དང་ས་བཀྲ། (ཐོག་མར་ཀློག)
├── plan.md                 ལག་རྩལ་གྱི་གནད་གལ་གྱི་ཐོ་གཞུང་།
├── params.py               ཚད་འཇོག་གི་མཛོད་ཁང་གཙོ་བོ (SI: m, kg, N, s, rad; ISA; non-dim → SI)
├── cfd_setup.py            བརྟན་པོའི་ SimulationParams སྦྱོར་བྱེད། (actuator disk, སྐྱོར་སྒྱུར་ཁྱམས་ 2)
├── scitech_outline.md      ཡིག་ཚགས་ཀྱི་གཞི་རྩ།
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
│   ├── AGENT_USABILITY_REPORT_v2.md  SDK ལ་ agent གིས་འཕྲད་པའི་དཀའ་ངལ།
│   ├── unsteady_setup.py           སྐྱོར་སྒྱུར་ཁྱམས་ 4 + UDD 4 (phugoid གི་རྨང་གཞི)
│   ├── submit_unsteady_warmup.py   phugoid གི་ warmup (དྲ་མིག་སྦྱོར་བ)
│   ├── submit_gap110_nokick_diag.py       phugoid chunk 1 (warmup ནས་ fork)
│   ├── submit_gap110_chunkN_nokick_diag.py phugoid chunk N (chunk N-1 ནས་ fork)
│   ├── submit_cruise.py / submit_v3.py    བརྟན་པོའི་ parent གནད་དོན།
│   ├── submit_*_sweep*.py / submit_*_campaign.py   བརྟན་པོའི་ སའོ་མེའོ།
│   └── submit_fine_ad_parents.py   actuator-disk དྲ་མིག་ཞིབ་མོའི་ཚོད་ལྟ།
├── post/
│   ├── _phase_plot.py, cg_shift_trim.py
│   ├── v2_gapped/           v2 བར་སྟོང་ཅིན་ཡིའི་ཐིག་གཞི་དང་ TRIM_RESULTS.md
│   ├── v3/                  phugoid གི་ཐིག་གཞི་ (plot_running_case, viz_nokick_lagfix,
│   │                        render_phugoid_slices) + unsteady_project_ids.json + mp4/png
│   └── FLOW360_GAI_BUG_REPORT.md
└── paper/                   SciTech extended abstract + figures (paper/build.sh)
```

**མ་ཚང་བ།** 2026-06 ཀྱི་ལས་ཀ་ (`flow360/UDD_DYNAMICS_GUIDE.md`,
`slidingInterfaceToUDD.py`, animation གི་ chase-camera ཡིག་ཆ་རྣམས) ད་དུང་
014-v100-dev ཁོ་ནར་ཡོད་ལ། GitHub ལ་མ་ཕུལ། `post/v3/` (phugoid གི་ཐིག་གཞི་
དང་ `unsteady_project_ids.json`) ནི་གནས་མཛོད་ནང་ཡོད།

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
- **FXC-11747** (25.12 ལ་བཅོས་ཟིན): 25.12 སྔོན་གྱི་པར་གཞིར། ནང་གི་ཁྱམས་
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
| phugoid རང་འཕུར (v3 cruise, 60 s) | `PHUGOID_UDD_POSTMORTEM.md` | ལེགས་གྲུབ, T = 17.4 s, ζ ≈ 0.026 |
| 2026-06 ཀྱི་ animation + UDD guide | 014-v100-dev:~/himalaya | GitHub ལ་མ་ཕུལ |
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
`flow360/unsteady_setup.py` (`mode='warmup'` and `mode='phugoid'` only; the
`pitch_*` modes are debugging leftovers) → `cfd_setup.py`, `params.py`.

**Reusable.** The rotation-chain topology and interface-gap sizing rule
(`unsteady_setup.py` §1); the per-zone lockstep-UDD pattern
(`_build_per_zone_udds`); the non-dim conversions; every rule in the
postmortem (gate at `pseudoStep == 0`, sequential `update_law` evaluation,
lag-correct lever arm, publish `omega` with `theta`, world-frame forces at
`moment_center`, one `output_target` per UDD, ≤10 state slots, no `atan2`);
body forces (actuator disks, engines) are NOT in the wall integral and must
be added by hand.

**Tsangpo-specific, replace wholesale.** `params.py`, the ESP/OpenCSM
airfoil pipeline under `geometry/`, the Flow360 folder IDs in
`cfd_setup.py`, every `submit_*.py` except the three phugoid ones.

**Solver version.** All runs used release-25.9 with all rotation axes
parallel (`(0, 1, 0)`). FXC-11747 (fixed in 25.12) corrupts per-step
geometry metrics when a nested zone rotates about a different axis from its
parent. Any 6-DOF extension (roll or yaw nested inside pitch) must run on
25.12 or later.

**Known gaps in this GitHub snapshot.** The June 2026 additions
(`flow360/UDD_DYNAMICS_GUIDE.md`, `slidingInterfaceToUDD.py`, the
chase-camera animation scripts) exist only on the author's workstation and
have not been pushed. Everything the phugoid chain needs is here: the two
`submit_gap110_*` fork scripts default to the IDs in
`post/v3/unsteady_project_ids.json` (the author's cloud projects) and take
`--project-id` / `--parent-case-id` to fork from a warmup you ran yourself.
