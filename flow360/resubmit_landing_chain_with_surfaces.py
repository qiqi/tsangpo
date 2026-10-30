"""Re-run the landing phugoid chain (chunks 1-4) with per-step surface
output so the body's pose-per-step is on disk in world coordinates,
allowing a body+slice animation without any manual coordinate
transforms.

Each chunk forks from the previous chunk; chunk1 forks from the
existing landing_warmup_gap110 warmup case.

surface_frequency = 4 — every 4th physical step.  Matches our standard
animation skip=4 (~10 fps at dt=0.025 s), and keeps the per-chunk
surface-data volume around 2-3 GB instead of 10 GB.
"""
from __future__ import annotations

import importlib.util
import json
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

import cfd_setup as C
import flow360 as fl

_spec = importlib.util.spec_from_file_location(
    "unsteady_setup", REPO / "flow360" / "unsteady_setup.py")
U = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(U)

PROJECT_KEY     = "landing_warmup_gap110"
WARMUP_KEY      = "landing_warmup_gap110_case"
N_STEPS         = 200
DT_S            = 0.025
MAX_PSEUDO      = 200
SURF_FREQ       = 4

IDS_PATH = REPO / "post" / "v3" / "unsteady_project_ids.json"


def _ids() -> dict:
    return json.loads(IDS_PATH.read_text())


def _save(d: dict) -> None:
    IDS_PATH.write_text(json.dumps(d, indent=2))


def submit_chunk(chunk_idx: int, parent_case_id: str) -> str:
    d = _ids()
    project = fl.Project.from_cloud(project_id=d[PROJECT_KEY])
    parent  = fl.Case.from_cloud(case_id=parent_case_id)
    surfaces = C.get_gapped_geometry_surfaces(project)
    params = U.build_unsteady_params(
        surfaces, phase="landing", mode="phugoid",
        n_physical_steps=N_STEPS,
        timestep_size_s=DT_S,
        max_pseudo_steps=MAX_PSEUDO,
        gamma_kick_deg=0.0,
        use_analytic_omegaDot=False,
        diagnostics=True,
        slice_frequency=SURF_FREQ,
        surface_frequency=SURF_FREQ,
    )
    case = project.run_case(
        params=params,
        name=f"v3_phugoid_landing_gap110_nokick_chunk{chunk_idx}_surf{SURF_FREQ}_5s",
        run_async=True, fork_from=parent,
        tags=["SI", "fork", "v3", "phugoid", "gap110", f"chunk{chunk_idx}",
              "nokick", "landing", "symplectic", "diag", "5s",
              f"surf_freq_{SURF_FREQ}"],
        use_beta_mesher=True,
    )
    key = f"phugoid_landing_gap110_nokick_surf_chunk{chunk_idx}"
    d[key] = case.id
    _save(d)
    print(f"  chunk{chunk_idx} submitted: {case.id}  ({key})")
    return case.id


def wait_complete(case_id: str, label: str) -> None:
    """Poll case status until terminal.  Resilient to transient 502/504/network
    errors from the Flow360 control plane — those just sleep and retry rather
    than crashing the orchestrator (we saw a 502 mid-poll on chunk2 once)."""
    while True:
        try:
            c = fl.Case.from_cloud(case_id=case_id)
            s = c.status.value
        except Exception as e:
            print(f"  {label}: transient poll error {e!r}; retrying in 60 s")
            time.sleep(60); continue
        if s in ("completed", "completed_with_warnings"):
            e = c.info.elapsedTimeInSeconds
            print(f"  {label}: COMPLETED  elapsed={e}s = {e/60:.1f}min")
            return
        if "error" in s or "failed" in s or "diverged" in s:
            print(f"  {label}: FAILED  status={s}")
            sys.exit(1)
        time.sleep(60)


def main() -> None:
    d = _ids()
    warmup_id = d[WARMUP_KEY]
    print(f"warmup parent: {warmup_id}")

    parent = warmup_id
    for chunk_idx in [1, 2, 3, 4]:
        key = f"phugoid_landing_gap110_nokick_surf_chunk{chunk_idx}"
        existing = d.get(key)
        if existing:
            # Already submitted in a previous orchestrator invocation; check status.
            try:
                c = fl.Case.from_cloud(case_id=existing)
                s = c.status.value
            except Exception:
                s = "unknown"
            if s in ("completed", "completed_with_warnings"):
                print(f"  chunk{chunk_idx}: already completed ({existing})")
                parent = existing
                continue
            elif s in ("pending", "running", "preprocessing", "uploading", "queued"):
                print(f"  chunk{chunk_idx}: already in flight, waiting ({existing})")
                wait_complete(existing, f"chunk{chunk_idx}")
                parent = existing
                continue
            else:
                print(f"  chunk{chunk_idx}: prior submission in state {s!r}; resubmitting")

        chunk_id = submit_chunk(chunk_idx, parent)
        wait_complete(chunk_id, f"chunk{chunk_idx}")
        parent = chunk_id

    print("\nALL CHUNKS DONE.  IDs saved under "
          "'phugoid_landing_gap110_nokick_surf_chunk{1,2,3,4}'.")


if __name__ == "__main__":
    main()
