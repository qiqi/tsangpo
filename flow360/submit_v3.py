"""
Submit the v3 (short-boom) campaign on tsangpo_v3.csm.

The v3 geometry is identical to tsangpo_gapped.csm except the htail LE
is at 1.5 c aft of the wing LE (was 3.5 c) — see params.X_TAIL_LE_V3_M.

Three parent cases (one per flap phase) at the BO trim points:
  * cruise  (phase 0): α=+7°,  θ_ht=+2°, T_mult=2.17,  V=V_CRUISE,  level
  * takeoff (phase 1): α=+15°, θ_ht=-5°, T_mult=15.21, V=V_TAKEOFF, γ=+30°
  * landing (phase 2): α=+25°, θ_ht=+5°, T_mult=8.69,  V=V_LANDING, γ=-30°

T_mult here = T/W / 0.046 (because each prop's BO disk-loading per
T/W=0.046 corresponds to T_mult=1.0; see params.T_CRUISE_PER_PROP_N
calibration).  T/W = 0.1 → 2.17,  0.7 → 15.21,  0.4 → 8.69.

After each parent submits we fork the per-dimension sensitivity sweep
(alpha / htail / thrust), shape-matching post/v2_gapped/.

Project IDs are written to post/v3/project_ids.json so the v3 plotters
pick them up automatically.

Usage:
    python3 flow360/submit_v3.py                # submit all three
    python3 flow360/submit_v3.py cruise         # one phase
"""
from __future__ import annotations

import json
import sys
import tempfile
import traceback
from math import radians
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
import cfd_setup as C
import params as P
import flow360 as fl

CSM      = REPO / "geometry" / "tsangpo_v3.csm"
AIRFOILS = REPO / "geometry" / "airfoils"

# Gapped-flap geometry constants (must match tsangpo_v3.csm despmtr defaults).
GAP_FRACTION       = 0.40
MID_OUTER_SEMISPAN = 0.39
FLAP_PANEL_FRAC    = 0.55
WING_SIDE_FRAC     = 0.61

# Convergence settings (match v2_gapped_sweeps).
N_PARENT_STEPS = 20
N_FORK_NEW     = 6
N_FORK_TOTAL   = N_PARENT_STEPS + N_FORK_NEW
PSEUDO_PARENT  = 1000
PSEUDO_FORK    = 500

# Project-IDs file (consumed by post/v3 plotters).
PROJECT_IDS_PATH = REPO / "post" / "v3" / "project_ids.json"


# Per-phase BO trim points (v3 short-boom).
PHASES = {
    "cruise": dict(
        phase           = 0,
        alpha_deg       = +7.0,
        theta_htail_deg = +2.0,
        thrust_mult     = 2.17,
        velocity_m_s    = P.V_CRUISE_M_S,
        altitude_m      = P.ALT_CRUISE_M,
        project_name    = "tsangpo_v3_cruise_gap40_short_boom",
        case_name       = "v3_cruise_parent",
        tags_phase      = ["v3", "gapped40", "short_boom", "cruise"],
    ),
    "takeoff": dict(
        phase           = 1,
        alpha_deg       = +15.0,
        theta_htail_deg = -5.0,
        thrust_mult     = 15.21,
        velocity_m_s    = P.V_TAKEOFF_M_S,
        altitude_m      = 0.0,
        project_name    = "tsangpo_v3_takeoff_gap40_short_boom",
        case_name       = "v3_takeoff_parent",
        tags_phase      = ["v3", "gapped40", "short_boom", "takeoff", "gamma_p30"],
    ),
    "landing": dict(
        phase           = 2,
        alpha_deg       = +25.0,
        theta_htail_deg = +5.0,
        thrust_mult     = 8.69,
        velocity_m_s    = P.V_LANDING_M_S,
        altitude_m      = 0.0,
        project_name    = "tsangpo_v3_landing_gap40_short_boom",
        case_name       = "v3_landing_parent",
        tags_phase      = ["v3", "gapped40", "short_boom", "landing", "gamma_m30"],
    ),
}


def _save_project_id(phase_key: str, project_id: str) -> None:
    PROJECT_IDS_PATH.parent.mkdir(parents=True, exist_ok=True)
    if PROJECT_IDS_PATH.exists():
        try:
            d = json.loads(PROJECT_IDS_PATH.read_text())
        except Exception:
            d = {}
    else:
        d = {}
    d[phase_key] = project_id
    PROJECT_IDS_PATH.write_text(json.dumps(d, indent=2))
    print(f"  wrote project_id[{phase_key}] = {project_id} → {PROJECT_IDS_PATH}")


def _name_token(value: float) -> str:
    """Encode a signed float as 'p07p00' / 'm04p00' for case names."""
    sign = "p" if value >= 0 else "m"
    return f"{sign}{abs(value):05.2f}".replace(".", "p")


def _sweep_design(alpha_b: float, theta_ht_b: float, T_b: float):
    """The three 1-D sweep arrays around the phase's BO point."""
    alpha_values = [alpha_b - 4, alpha_b - 2, alpha_b + 2,
                    alpha_b + 4, alpha_b + 6, alpha_b + 8]
    htail_values = [theta_ht_b - 15, theta_ht_b - 10, theta_ht_b - 5,
                    theta_ht_b + 5, theta_ht_b + 10, theta_ht_b + 15]
    thrust_values = [T_b / 4, T_b / 2, T_b * 3 / 4,
                     T_b * 1.25, T_b * 1.5, T_b * 2, T_b * 2.5]
    return alpha_values, htail_values, thrust_values


def submit_one(phase_key: str) -> dict:
    spec = PHASES[phase_key]
    theta_ac_b = radians(spec["alpha_deg"])
    theta_ht_b = radians(spec["theta_htail_deg"])
    T_b        = spec["thrust_mult"]
    V          = spec["velocity_m_s"]
    H          = spec["altitude_m"]

    print(f"\n=== {phase_key.upper()} (phase {spec['phase']}) ===")
    print(f"  α       = {spec['alpha_deg']:+.2f}°   "
          f"θ_ht = {spec['theta_htail_deg']:+.2f}°   "
          f"T_mult = {T_b:+.3f}")
    print(f"  V       = {V:.2f} m/s   altitude = {H:.0f} m")

    # --- Inline UDCs and override despmtrs into a temp .csm ---
    inlined = C.inline_udcs(CSM, AIRFOILS)
    inlined = C.set_csm_despmtrs(
        inlined,
        phase              = spec["phase"],
        gap_fraction       = GAP_FRACTION,
        mid_outer_semispan = MID_OUTER_SEMISPAN,
        flap_panel_frac    = FLAP_PANEL_FRAC,
        wing_side_frac     = WING_SIDE_FRAC,
    )
    with tempfile.NamedTemporaryFile("w", suffix=".csm", delete=False) as f:
        f.write(inlined)
        tmp_csm = f.name
    print(f"  Uploading inlined {CSM.name} "
          f"({len(inlined.splitlines())} lines) → {tmp_csm}")

    # --- Create project + parent case ---
    project = fl.Project.from_geometry(
        tmp_csm,
        name=spec["project_name"],
        length_unit="m",
        tags=["tsangpo", *spec["tags_phase"]],
    )
    print(f"  → project {project.id}")
    _save_project_id(phase_key, project.id)

    surfaces = C.get_gapped_geometry_surfaces(project)
    print(f"  Surfaces: main={surfaces.wing_main_surfs[0].name}, "
          f"vane={[s.name for s in surfaces.wing_vane_surfs]}, "
          f"flap={[s.name for s in surfaces.wing_flap_surfs]}, "
          f"htail={surfaces.htail_surf.name}")

    parent_case = project.run_case(
        params=C.build_params(
            surfaces,
            theta_ac_rad     = theta_ac_b,
            theta_ht_rad     = theta_ht_b,
            thrust_mult      = T_b,
            velocity_m_s     = V,
            altitude_m       = H,
            n_steps_total    = N_PARENT_STEPS,
            max_pseudo_steps = PSEUDO_PARENT,
            htail_x_m        = P.X_TAIL_LE_V3_M + 0.25 * P.HTAIL_CHORD_M,
            # v3 htail c/4 at +1.75 c_w; wing TE at +0.5 c_w.  Shrink the
            # ht_cyl radius so the rotation cylinder doesn't cut through
            # the wing TE at y = ±half-height (was 1.5 c_htail in v2;
            # 1.0 c_htail keeps cylinder x_min at +0.75 c_w, clear of the wing).
            ht_radius_m      = 1.0 * P.HTAIL_CHORD_M,
        ),
        name=spec["case_name"],
        run_async=True,
        tags=["SI", "parent", *spec["tags_phase"]],
        use_beta_mesher=True,
    )
    print(f"  Parent case submitted: {parent_case.id}")

    # --- Fork sweep cases ---
    alpha_values, htail_values, thrust_values = _sweep_design(
        spec["alpha_deg"], spec["theta_htail_deg"], T_b)

    forks = {"alpha": [], "htail": [], "thrust": []}

    def _submit_fork(sweep: str, value: float):
        if sweep == "alpha":
            ta, th, tm = radians(value), theta_ht_b, T_b
        elif sweep == "htail":
            ta, th, tm = theta_ac_b, radians(value), T_b
        elif sweep == "thrust":
            ta, th, tm = theta_ac_b, theta_ht_b, max(value, 1e-4)
        else:
            raise ValueError(sweep)
        name = f"v3_{phase_key}_{sweep}_{_name_token(value)}"
        case = project.run_case(
            params=C.build_params(
                surfaces,
                theta_ac_rad     = ta,
                theta_ht_rad     = th,
                thrust_mult      = tm,
                velocity_m_s     = V,
                altitude_m       = H,
                n_steps_total    = N_FORK_TOTAL,
                max_pseudo_steps = PSEUDO_FORK,
                htail_x_m        = P.X_TAIL_LE_V3_M + 0.25 * P.HTAIL_CHORD_M,
            # v3 htail c/4 at +1.75 c_w; wing TE at +0.5 c_w.  Shrink the
            # ht_cyl radius so the rotation cylinder doesn't cut through
            # the wing TE at y = ±half-height (was 1.5 c_htail in v2;
            # 1.0 c_htail keeps cylinder x_min at +0.75 c_w, clear of the wing).
            ht_radius_m      = 1.0 * P.HTAIL_CHORD_M,
            ),
            name=name,
            run_async=True,
            fork_from=parent_case,
            tags=["SI", "fork", "sweep", f"{sweep}_sweep", *spec["tags_phase"]],
            use_beta_mesher=True,
        )
        forks[sweep].append((value, case.id))
        print(f"  fork  {sweep:6s} {value:+7.2f}  →  {case.id}   ({name})")
        return case.id

    for sweep_name, values in (("alpha",  alpha_values),
                               ("htail",  htail_values),
                               ("thrust", thrust_values)):
        print(f"  --- {sweep_name} sweep: {len(values)} cases ---")
        for v in values:
            try:
                _submit_fork(sweep_name, v)
            except Exception as exc:  # noqa: BLE001
                print(f"  fork  {sweep_name:6s} {v:+7.2f}  FAILED: {exc!r}")
                traceback.print_exc()

    return {
        "phase":   phase_key,
        "project": project.id,
        "parent":  parent_case.id,
        "forks":   forks,
    }


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] in PHASES:
        keys = [sys.argv[1]]
    elif len(sys.argv) > 1:
        sys.exit(f"unknown phase {sys.argv[1]!r}; choices: {list(PHASES)}")
    else:
        keys = list(PHASES)

    results = []
    for k in keys:
        try:
            results.append(submit_one(k))
        except Exception as exc:  # noqa: BLE001
            print(f"\n!!! {k} FAILED at parent stage: {exc!r}")
            traceback.print_exc()

    print("\n\n=== v3 submission summary ===")
    for r in results:
        print(f"\n{r['phase']}: project {r['project']}  parent {r['parent']}")
        for sweep in ("alpha", "htail", "thrust"):
            print(f"  {sweep}:")
            for v, cid in r["forks"][sweep]:
                print(f"    {v:+7.2f}  {cid}")
