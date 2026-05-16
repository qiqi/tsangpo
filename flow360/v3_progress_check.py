"""
Hourly progress check on the v3 short-boom campaign and the v2 gap-high
campaign.

Per (family, phase) project:
  - Count COMPLETED vs total cases.
  - When >= 80% COMPLETED, refresh that phase's sensitivity-plot cache
    by invoking the corresponding plotter (post/v3/plot_<phase>_…
    or post/v2_gapped_high/plot_<phase>_…).
  - When ALL of a family's 3 phases are >= 80%, also run the combined-
    sens overlay plotter so the paper figure auto-updates.

Run from cron, idempotent.  Exits 0 with a one-line status per project.
"""
from __future__ import annotations
import importlib.util, json, subprocess, sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "post"))
import flow360 as fl


# ----- v3 project IDs (from post/v3/project_ids.json) -----
def _load_v3_project_ids() -> dict[str, str]:
    p = REPO / "post" / "v3" / "project_ids.json"
    if not p.exists(): return {}
    return json.loads(p.read_text())


# ----- v2_gapped_high project IDs (from PhaseSpec) -----
def _load_phase_project_ids(family: str) -> dict[str, str]:
    out = {}
    for ph in ("cruise", "takeoff", "landing"):
        pp = REPO / "post" / family / f"plot_{ph}_sensitivities.py"
        if not pp.exists(): continue
        s = importlib.util.spec_from_file_location(f"{family}_{ph}", pp)
        m = importlib.util.module_from_spec(s); s.loader.exec_module(m)
        out[ph] = m.SPEC.project_id
    return out


def _phase_status(project_id: str) -> tuple[int, int, int]:
    """Returns (done, in_flight, errored) counts."""
    if not project_id or project_id == "TBD":
        return 0, 0, 0
    p = fl.Project.from_cloud(project_id=project_id)
    done = run = err = 0
    for cid in p.get_case_ids():
        c = fl.Case.from_cloud(case_id=cid)
        s = str(c.status).replace("Flow360Status.", "")
        if any(b in s for b in ("ERROR", "FAILED", "DIVERGED", "CRASHED")):
            err += 1
        elif "COMPLETED" in s or "STOPPED" in s:
            done += 1
        else:
            run += 1
    return done, run, err


def _run_plotter(family: str, phase: str):
    pp = REPO / "post" / family / f"plot_{phase}_sensitivities.py"
    if not pp.exists(): return None
    print(f"  triggering plotter for {family}/{phase} …")
    r = subprocess.run(
        ["python3", str(pp), "--refresh"],
        capture_output=True, text=True, cwd=str(REPO),
    )
    # tail of output for visibility
    out_tail = "\n".join((r.stdout or "").splitlines()[-6:])
    if out_tail.strip():
        for line in out_tail.splitlines():
            print(f"    {line}")
    if r.returncode != 0:
        print(f"  ! plotter exit {r.returncode}; stderr: {r.stderr[-300:]}")
    return r.returncode


def _trigger_combined_sens():
    """Re-build all combined-sens PNGs (cont-low / cont-high / gap-low).
    Now also generates v3 and v2_gapped_high if those families are listed."""
    sc = REPO / "paper" / "figures" / "_build_combined_sens.py"
    print(f"  re-rendering combined-sens plots …")
    r = subprocess.run(["python3", str(sc)], capture_output=True, text=True,
                        cwd=str(REPO))
    if r.returncode != 0:
        print(f"  ! combined-sens exit {r.returncode}; stderr: {r.stderr[-300:]}")
    else:
        for line in (r.stdout or "").splitlines()[-6:]:
            print(f"    {line}")


def main():
    families: dict[str, dict[str, str]] = {
        "v3":              _load_v3_project_ids(),
        "v2_gapped_high":  _load_phase_project_ids("v2_gapped_high"),
    }
    summary = {}
    for fam, projects in families.items():
        if not projects:
            print(f"=== {fam}: no project IDs known, skipping")
            continue
        print(f"=== {fam} ===")
        family_ready = True
        for ph, pid in projects.items():
            done, run, err = _phase_status(pid)
            tot = done + run + err
            pct = 100.0 * done / tot if tot else 0.0
            print(f"  {ph:8s}  done={done:3d}  run={run:3d}  err={err:3d}  "
                  f"({pct:5.1f}% complete)")
            if pct >= 80.0:
                _run_plotter(fam, ph)
            else:
                family_ready = False
        summary[fam] = family_ready
    # If both v3 and gap-high reached 80% across all phases, refresh the
    # combined-sens overlay figures so the paper picks them up.
    if any(summary.values()):
        _trigger_combined_sens()


if __name__ == "__main__":
    main()
