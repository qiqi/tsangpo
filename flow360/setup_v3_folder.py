"""
One-shot helper: create a `6_v3_short_boom` subfolder under the Tsangpo
project folder on Flow360 and move all v3 projects into it.

v3 projects (from post/v3/project_ids.json and unsteady_project_ids.json):
  - cruise        static-CFD parent
  - takeoff       static-CFD parent
  - landing       static-CFD parent
  - cruise_warmup unsteady-CFD warmup (just submitted)

This mirrors the existing v2 folder convention (`1_v1_historical`,
`2_v2_continuous_high_htail`, ..., `5_v2_continuous_low_htail`).
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
import cfd_setup as C
import flow360 as fl
from flow360.cloud.http_util import http


TSANGPO_PARENT_FOLDER_ID = C.TSANGPO_FOLDER_IDS["Tsangpo"]
V3_FOLDER_NAME           = "6_v3_short_boom"


def _list_subfolders(parent_folder_id: str) -> list[dict]:
    """List subfolders under the given parent folder via the API."""
    resp = http.get("v2/folders", params={"parentFolderId": parent_folder_id})
    # response shape varies by SDK version; handle both list and dict-with-data
    if isinstance(resp, dict) and "data" in resp:
        return resp["data"]
    return resp


def _create_v3_folder() -> str:
    """Find or create the v3 folder under Tsangpo.  Returns its ID."""
    parent = fl.Folder(id=TSANGPO_PARENT_FOLDER_ID)
    existing = fl.Folder.from_list(parent_folder=parent) if hasattr(fl.Folder, "from_list") else None
    if existing is not None:
        for f in existing:
            if f.name == V3_FOLDER_NAME:
                print(f"  found existing v3 folder: {f.id}")
                return f.id
    print(f"  creating new v3 folder under Tsangpo …")
    draft = fl.Folder.create(name=V3_FOLDER_NAME, parent_folder=parent)
    folder = draft.submit()
    print(f"  created folder {folder.id} ({V3_FOLDER_NAME})")
    return folder.id


def _move_project_to_folder(project_id: str, folder_id: str) -> None:
    """PATCH the project's parentFolderId."""
    http.patch(f"v2/projects/{project_id}",
                json={"parentFolderId": folder_id})


def main() -> None:
    print("=== Setting up v3 folder on Flow360 ===")
    v3_folder_id = _create_v3_folder()

    # Collect all v3 project IDs.
    static_ids = json.loads(
        (REPO / "post" / "v3" / "project_ids.json").read_text())
    unsteady_path = REPO / "post" / "v3" / "unsteady_project_ids.json"
    unsteady_ids = json.loads(unsteady_path.read_text()) if unsteady_path.exists() else {}

    projects_to_move: dict[str, str] = {}
    for phase, pid in static_ids.items():
        projects_to_move[f"static/{phase}"] = pid
    for key, pid in unsteady_ids.items():
        projects_to_move[f"unsteady/{key}"] = pid

    print(f"\n  Moving {len(projects_to_move)} v3 project(s) into folder {v3_folder_id}:")
    for label, pid in projects_to_move.items():
        try:
            _move_project_to_folder(pid, v3_folder_id)
            print(f"    moved  {label:30s}  {pid}")
        except Exception as e:
            print(f"    FAILED {label:30s}  {pid}   ({e!r})")

    print(f"\n  Folder ID for future submissions: {v3_folder_id}")
    print(f"  Add to cfd_setup.TSANGPO_FOLDER_IDS as '{V3_FOLDER_NAME}'.")


if __name__ == "__main__":
    main()
