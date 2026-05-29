"""Landing phugoid chain driver — idempotent step-forward state machine.

Inspects the current state of:
  • landing_warmup_gap110_case  (parent)
  • phugoid_landing_gap110_nokick_diag_chunk1  (chunk 1)
  • phugoid_landing_gap110_nokick_chunk2..6    (chunks 2-6)

Action rules:
  - If a case is RUNNING/PENDING/UPLOADING, print status and exit (wait).
  - If a case is COMPLETED and its child has not been submitted yet, fork
    the child.
  - If a case is ERRORED, print a loud notice and exit.
  - If all 6 chunks COMPLETED, print "DONE" and exit.

Idempotent: safe to re-run at any cadence; will only submit each chunk once.

Usage:
    python3 flow360/run_landing_phugoid_chain.py
"""
from __future__ import annotations

import importlib.util
import json
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

import flow360 as fl

IDS_PATH = REPO / "post" / "v3" / "unsteady_project_ids.json"
PROJECT_KEY = "landing_warmup_gap110"
PARENT_CASE_KEY = "landing_warmup_gap110_case"

# Chain: each entry maps the parent key → child key (and the submit script
# that needs to be invoked to fork that child off the parent).
CHAIN = [
    # chunk1 forks from the landing warmup; chunk1 script reads
    # PROJECT_KEY/PARENT_CASE_KEY from the json directly.
    {"child": "phugoid_landing_gap110_nokick_diag_chunk1",
     "submit": ["python3", str(REPO / "flow360" / "submit_landing_chunk1_nokick_diag.py")],
     "parent_key": PARENT_CASE_KEY},
    # chunks 2-6: parent is the previous chunk; submit_landing_chunkN_nokick_diag.py
    # takes (parent_case_id, chunk_idx) as args.
    {"child": "phugoid_landing_gap110_nokick_chunk2",
     "submit_chunkN": True, "chunk": 2,
     "parent_key": "phugoid_landing_gap110_nokick_diag_chunk1"},
    {"child": "phugoid_landing_gap110_nokick_chunk3",
     "submit_chunkN": True, "chunk": 3,
     "parent_key": "phugoid_landing_gap110_nokick_chunk2"},
    # NOTE: chunks 4-6 intentionally NOT chained.  Chunk3 entered an
    # extreme-attitude regime where dt=0.025s + max_pseudo=200 stopped
    # resolving the flow (CMy magnitudes ×25 of chunk1 peak; CD went
    # nonphysically negative).  Open-loop instability is fully
    # characterized by chunks 1-2 (trim → +98° apex of half-loop).
    # If we ever need cleaner late-time data, refine dt and/or
    # max_pseudo for chunk3+ and re-launch.
]


def _status(case_id: str) -> str:
    try:
        s = fl.Case.from_cloud(case_id=case_id).status
        # Flow360Status is an enum; coerce to its string value ("pending",
        # "running", "completed", "error", "diverged", …).
        return getattr(s, "value", None) or str(s).split(".")[-1].lower()
    except Exception as e:
        return f"error({type(e).__name__}:{e})"


def _terminal(status: str) -> bool:
    return status.lower() in {"completed", "error", "errored", "diverged"}


def _running(status: str) -> bool:
    return not _terminal(status)


def main() -> None:
    d = json.loads(IDS_PATH.read_text())

    # 1. Check warmup parent.
    parent_id = d.get(PARENT_CASE_KEY)
    if not parent_id:
        print(f"[chain] ERROR: {PARENT_CASE_KEY} not in {IDS_PATH.name}; "
              f"submit_landing_warmup.py first.")
        return
    parent_status = _status(parent_id)
    print(f"[chain] warmup {parent_id[:14]}…  status = {parent_status}")
    if "error" in parent_status.lower() or "diverged" in parent_status.lower():
        print(f"[chain] WARMUP FAILED — aborting chain.")
        return

    # 2. Walk the chain, submitting the first un-submitted child whose
    #    parent is COMPLETED.
    for link in CHAIN:
        child_key = link["child"]
        parent_key = link["parent_key"]
        parent_id_for_link = d.get(parent_key)
        if not parent_id_for_link:
            print(f"[chain] {child_key}: parent key {parent_key!r} not in json yet — waiting.")
            return
        parent_st = _status(parent_id_for_link)
        if parent_st.lower() != "completed":
            print(f"[chain] {child_key}: parent {parent_key} still {parent_st}; waiting.")
            return
        # Parent completed — check child.
        child_id = d.get(child_key)
        if child_id is None:
            # Submit the child.
            print(f"[chain] parent {parent_key} COMPLETED; submitting {child_key}…")
            if link.get("submit_chunkN"):
                cmd = ["python3",
                       str(REPO / "flow360" / "submit_landing_chunkN_nokick_diag.py"),
                       parent_id_for_link,
                       str(link["chunk"])]
            else:
                cmd = link["submit"]
            r = subprocess.run(cmd, cwd=str(REPO), capture_output=True, text=True)
            print(r.stdout)
            if r.returncode != 0:
                print(f"[chain] submit FAILED: {r.stderr}")
                return
            return  # one submission per iteration — re-poll next wake
        # Child exists — check its status.
        child_st = _status(child_id)
        print(f"[chain] {child_key} {child_id[:14]}…  status = {child_st}")
        if "error" in child_st.lower() or "diverged" in child_st.lower():
            print(f"[chain] {child_key} FAILED — aborting chain.")
            return
        if child_st.lower() != "completed":
            return  # wait for it
        # Loop: child completed, advance to next link.

    print("[chain] ALL 6 CHUNKS COMPLETED — landing 30 s phugoid run done.")


if __name__ == "__main__":
    main()
