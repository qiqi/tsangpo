"""θ_eq(θ_ht, V_inf) grid sweep — damped 1-DOF pitch-only stability test.

Methodology:
  • For each (θ_ht, V_inf) cell:
      1) Submit a fresh warmup at that (θ_ht, V_inf) condition (mode='warmup',
         all four cyl frozen at L-pose+htail incidence).
      2) Once warmup completes, fork a damped 1-DOF chunk1 from it:
         mode = 'pitch_step_cyl3_integrated', cyl_1/2/4 frozen via
         link1/link2_ramp=0, htail at θ_ht_rel constant; cyl_3 free
         under M(θ) with artificial damping.
      3) Once chunk1 completes, extract θ_eq from the final state of
         the UDD log (the integrator's state[4]).
  • Output: a CSV with one row per (θ_ht, V_inf, θ_eq) cell.

Damping rationale:
  The natural pitch dynamics have damping ratio ζ << 1 ⇒ many oscillation
  periods to find equilibrium.  Adding artificial damping D = 10000
  N·m·s/rad makes the dynamics overdamped (ζ ~ 3 for our airframe with
  I=3000 kg·m², k~800 N·m/rad).  The body then slides smoothly to
  M(θ_eq)=0 with no overshoot, settling in ~5-10 s sim time.

Usage:
    python3 flow360/pitch_eq_grid.py [theta_ht_list] [V_inf_list]

  Default grid: θ_ht ∈ {-3.67, -2, 0, +1, +3} deg, V_inf ∈ {12.86} m/s.
  Both can be overridden via command-line CSV lists, e.g.:
    python3 flow360/pitch_eq_grid.py "-3.67,0,+3" "10,12.86,15"

  Re-running is idempotent: skips any (θ_ht, V_inf) cell whose result is
  already in the output CSV.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
import sys
import tempfile
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "post" / "v3"))

import cfd_setup as C
import flow360 as fl

_spec = importlib.util.spec_from_file_location(
    "unsteady_setup", REPO / "flow360" / "unsteady_setup.py")
U = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(U)

# ---- Tunables ---------------------------------------------------------------
# Grid defaults: cover the retrim sweep we've explored + a slightly broader set
# of V_inf around the design landing speed (25 kt = 12.86 m/s).
DEFAULT_TH_HT = [-3.67, -2.0, 0.0, +1.0, +3.0]
DEFAULT_V_INF = [12.86]

# Pitch-only damped run parameters.  Picked for ~10 physical steps to settle.
WARMUP_N_STEPS         = 30
WARMUP_DT_S            = 1.0
WARMUP_MAX_PSEUDO      = 1500
CHUNK_N_STEPS          = 15
CHUNK_DT_S             = 0.5         # ~5× warmup density, room for body to rotate
CHUNK_MAX_PSEUDO       = 1500
PITCH_DAMPING_SI       = 10000.0     # ~3× critical for our system
STEP_OFFSET            = WARMUP_N_STEPS  # tells the UDD where chunk1 starts

OUT_CSV   = REPO / "post" / "v3" / "phugoid_runs" / "landing" / "pitch_eq_grid.csv"
IDS_PATH  = REPO / "post" / "v3" / "unsteady_project_ids.json"
TAG_BASE  = "v3_pitch_eq_grid"

# Geometry — landing flap config (phase=2 in tsangpo_v3.csm).
CSM      = REPO / "geometry" / "tsangpo_v3.csm"
AIRFOILS = REPO / "geometry" / "airfoils"
GAP_FRACTION       = 0.40
MID_OUTER_SEMISPAN = 0.39
FLAP_PANEL_FRAC    = 0.55
WING_SIDE_FRAC     = 0.61


# ---- IDs persistence --------------------------------------------------------
def _load_ids() -> dict:
    if IDS_PATH.exists():
        try:
            return json.loads(IDS_PATH.read_text())
        except Exception:
            return {}
    return {}

def _save_ids(d: dict) -> None:
    IDS_PATH.parent.mkdir(parents=True, exist_ok=True)
    IDS_PATH.write_text(json.dumps(d, indent=2))


# ---- CSV results ------------------------------------------------------------
def _load_results() -> dict:
    """Return {(θ_ht_deg, V_inf_m_s): row_dict} from the CSV."""
    if not OUT_CSV.exists():
        return {}
    out = {}
    import csv
    with OUT_CSV.open() as f:
        for row in csv.DictReader(f):
            key = (float(row["theta_ht_deg"]), float(row["V_inf_m_s"]))
            out[key] = row
    return out

def _append_result(row: dict) -> None:
    """Append (or rewrite if key exists) a row in the CSV."""
    OUT_CSV.parent.mkdir(parents=True, exist_ok=True)
    cur = _load_results()
    key = (float(row["theta_ht_deg"]), float(row["V_inf_m_s"]))
    cur[key] = row
    import csv
    cols = ["theta_ht_deg", "V_inf_m_s", "theta_eq_deg",
            "warmup_case_id", "chunk_case_id", "status", "notes"]
    with OUT_CSV.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for k in sorted(cur):
            r = cur[k]
            w.writerow({c: r.get(c, "") for c in cols})


# ---- Case status ------------------------------------------------------------
def _status(cid: str) -> str:
    try:
        s = fl.Case.from_cloud(case_id=cid).status
        return getattr(s, "value", str(s).split(".")[-1].lower())
    except Exception as e:
        return f"error({e})"

def _is_complete(cid: str) -> bool:
    return _status(cid).lower() in ("completed", "completed_with_warnings")

def _is_failed(cid: str) -> bool:
    s = _status(cid).lower()
    return "error" in s or "diverged" in s or "failed" in s


# ---- Cell submission --------------------------------------------------------
def _cell_key(th_ht: float, V_inf: float) -> str:
    """Stable string key for one grid cell."""
    return f"pitch_eq_th{th_ht:+.2f}_V{V_inf:.2f}".replace(".", "p").replace("+", "P").replace("-", "N")

def submit_warmup(th_ht_deg: float, V_inf_m_s: float) -> str:
    """Create a fresh project for this cell, return the case ID."""
    key = _cell_key(th_ht_deg, V_inf_m_s)
    d = _load_ids()
    warmup_key = f"{key}_warmup_case"
    if warmup_key in d:
        return d[warmup_key]
    # Inline UDCs + landing phase
    inlined = C.inline_udcs(CSM, AIRFOILS)
    inlined = C.set_csm_despmtrs(
        inlined, phase=2,
        gap_fraction=GAP_FRACTION,
        mid_outer_semispan=MID_OUTER_SEMISPAN,
        flap_panel_frac=FLAP_PANEL_FRAC,
        wing_side_frac=WING_SIDE_FRAC,
    )
    with tempfile.NamedTemporaryFile("w", suffix=".csm", delete=False) as f:
        f.write(inlined)
        tmp_csm = f.name
    project = fl.Project.from_geometry(
        tmp_csm, name=key, length_unit="m",
        tags=[TAG_BASE, "warmup", key,
              f"th_ht={th_ht_deg:+.2f}", f"V={V_inf_m_s:.2f}"],
    )
    print(f"  [{key}] project {project.id} created")
    d[f"{key}_project"] = project.id
    surfaces = C.get_gapped_geometry_surfaces(project)
    params = U.build_unsteady_params(
        surfaces, phase="landing", mode="warmup",
        n_physical_steps=WARMUP_N_STEPS,
        timestep_size_s=WARMUP_DT_S,
        max_pseudo_steps=WARMUP_MAX_PSEUDO,
        theta_ht_override_deg=th_ht_deg,
        V_inf_override_m_s=V_inf_m_s,
    )
    case = project.run_case(params=params, name=f"{key}_warmup",
                             run_async=True, use_beta_mesher=True,
                             tags=[TAG_BASE, "warmup", key])
    d[warmup_key] = case.id
    _save_ids(d)
    print(f"  [{key}] warmup case {case.id}")
    return case.id

def submit_pitch_only(th_ht_deg: float, V_inf_m_s: float,
                      warmup_case_id: str) -> str:
    """Fork a damped 1-DOF pitch-only chunk from the warmup."""
    key = _cell_key(th_ht_deg, V_inf_m_s)
    d = _load_ids()
    chunk_key = f"{key}_chunk_case"
    if chunk_key in d:
        return d[chunk_key]
    proj = fl.Project.from_cloud(project_id=d[f"{key}_project"])
    parent = fl.Case.from_cloud(case_id=warmup_case_id)
    surfaces = C.get_gapped_geometry_surfaces(proj)
    params = U.build_unsteady_params(
        surfaces, phase="landing", mode="pitch_step_cyl3_integrated",
        n_physical_steps=CHUNK_N_STEPS,
        timestep_size_s=CHUNK_DT_S,
        max_pseudo_steps=CHUNK_MAX_PSEUDO,
        theta_ht_override_deg=th_ht_deg,
        V_inf_override_m_s=V_inf_m_s,
        pitch_step_link1_ramp_deg=0.0,
        pitch_step_link2_ramp_deg=0.0,
        pitch_step_step_offset=STEP_OFFSET,
        pitch_damping_si=PITCH_DAMPING_SI,
    )
    case = proj.run_case(params=params, name=f"{key}_pitchonly",
                         run_async=True, fork_from=parent,
                         use_beta_mesher=True,
                         tags=[TAG_BASE, "pitchonly", key])
    d[chunk_key] = case.id
    _save_ids(d)
    print(f"  [{key}] pitch-only case {case.id}")
    return case.id


def extract_theta_eq(chunk_case_id: str) -> float:
    """Read final state[4] (θ_body) from the UDD CSV."""
    c = fl.Case.from_cloud(case_id=chunk_case_id)
    fn = "results/udd_pitch_cyl3int_airframe_v2.csv"
    target = Path(f"/tmp/{chunk_case_id[:13]}/")
    target.mkdir(parents=True, exist_ok=True)
    c._download_file(fn, to_folder=str(target), overwrite=True)
    import pandas as pd
    df = pd.read_csv(target / fn.split("/")[-1],
                     sep=r"\s*,\s*", engine="python")
    ends = df.groupby("physical_step").tail(1)
    return math.degrees(float(ends["state[4]"].iloc[-1]))


# ---- Orchestrator -----------------------------------------------------------
def run_grid(th_ht_list: list[float], V_inf_list: list[float],
             poll_interval_s: int = 60) -> None:
    cells = [(th, v) for th in th_ht_list for v in V_inf_list]
    print(f"Grid: {len(cells)} cells")
    results = _load_results()

    # Phase 1: submit all warmups in parallel
    warmups: dict[tuple[float, float], str] = {}
    for th, v in cells:
        key = (th, v)
        if key in results and results[key].get("status") == "completed":
            print(f"  ({th:+.2f}, {v:.2f}): cached θ_eq={results[key]['theta_eq_deg']}; skip")
            continue
        warmups[key] = submit_warmup(th, v)

    # Phase 2: poll until each warmup completes, fork chunk
    chunks: dict[tuple[float, float], str] = {}
    done_warm: set = set()
    while len(done_warm) < len(warmups):
        for key, cid in warmups.items():
            if key in done_warm:
                continue
            if _is_complete(cid):
                done_warm.add(key)
                print(f"  ({key[0]:+.2f}, {key[1]:.2f}) warmup COMPLETE → forking pitch-only")
                chunks[key] = submit_pitch_only(key[0], key[1], cid)
            elif _is_failed(cid):
                done_warm.add(key)
                _append_result({"theta_ht_deg": key[0], "V_inf_m_s": key[1],
                                "theta_eq_deg": "", "warmup_case_id": cid,
                                "chunk_case_id": "", "status": "warmup_failed",
                                "notes": _status(cid)})
                print(f"  ({key[0]:+.2f}, {key[1]:.2f}) warmup FAILED — recorded")
        if len(done_warm) < len(warmups):
            time.sleep(poll_interval_s)

    # Phase 3: poll until each chunk completes, extract θ_eq
    done_chunk: set = set()
    while len(done_chunk) < len(chunks):
        for key, cid in chunks.items():
            if key in done_chunk:
                continue
            if _is_complete(cid):
                done_chunk.add(key)
                try:
                    theta_eq = extract_theta_eq(cid)
                    _append_result({"theta_ht_deg": key[0], "V_inf_m_s": key[1],
                                    "theta_eq_deg": f"{theta_eq:.4f}",
                                    "warmup_case_id": warmups[key],
                                    "chunk_case_id": cid,
                                    "status": "completed", "notes": ""})
                    print(f"  ({key[0]:+.2f}, {key[1]:.2f}) θ_eq = {theta_eq:+.3f}°")
                except Exception as e:
                    _append_result({"theta_ht_deg": key[0], "V_inf_m_s": key[1],
                                    "theta_eq_deg": "",
                                    "warmup_case_id": warmups[key],
                                    "chunk_case_id": cid,
                                    "status": "extract_failed",
                                    "notes": str(e)[:200]})
                    print(f"  ({key[0]:+.2f}, {key[1]:.2f}) extract FAILED: {e!r}")
            elif _is_failed(cid):
                done_chunk.add(key)
                _append_result({"theta_ht_deg": key[0], "V_inf_m_s": key[1],
                                "theta_eq_deg": "",
                                "warmup_case_id": warmups[key],
                                "chunk_case_id": cid,
                                "status": "chunk_failed",
                                "notes": _status(cid)})
        if len(done_chunk) < len(chunks):
            time.sleep(poll_interval_s)

    print(f"\nGrid sweep done.  Results: {OUT_CSV}")


# ---- CLI --------------------------------------------------------------------
def _parse_floats(s: str) -> list[float]:
    return [float(x.strip()) for x in s.split(",") if x.strip()]

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("theta_ht_list", nargs="?",
                    default=",".join(f"{x:+.2f}" for x in DEFAULT_TH_HT),
                    help="comma-separated θ_ht values in degrees")
    ap.add_argument("V_inf_list", nargs="?",
                    default=",".join(f"{x:.2f}" for x in DEFAULT_V_INF),
                    help="comma-separated V_inf values in m/s")
    ap.add_argument("--no-submit", action="store_true",
                    help="dry-run: print the grid and exit")
    args = ap.parse_args()
    th_ht_list = _parse_floats(args.theta_ht_list)
    V_inf_list = _parse_floats(args.V_inf_list)
    if args.no_submit:
        for th in th_ht_list:
            for v in V_inf_list:
                print(f"  ({th:+.2f}, {v:.2f})")
        sys.exit(0)
    run_grid(th_ht_list, V_inf_list)
