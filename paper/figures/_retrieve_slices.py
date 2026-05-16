"""
Retrieve slice data for a Flow360 case.

Pulls slices.tar.gz from the case results and extracts it into
paper/figures/slices/<case-id>/.

Usage:
    python3 paper/figures/_retrieve_slices.py <case-id>
"""
from __future__ import annotations
import argparse, sys, tarfile, tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
SLICES_ROOT = HERE / "slices"

sys.path.insert(0, str(REPO))
import flow360 as fl


def retrieve(case_id: str) -> Path:
    SLICES_ROOT.mkdir(parents=True, exist_ok=True)
    dest = SLICES_ROOT / case_id
    dest.mkdir(parents=True, exist_ok=True)

    c = fl.Case.from_cloud(case_id=case_id)
    if "COMPLETED" not in str(c.status):
        raise RuntimeError(f"case {case_id} status is {c.status} (not COMPLETED)")
    print(f"case: {c.name}  ({c.status})")

    with tempfile.TemporaryDirectory() as tmp:
        c.results.slices.download(to_folder=tmp, overwrite=True)
        tar_path = next(Path(tmp).rglob("slices*.tar.gz"))
        print(f"downloaded: {tar_path.name}  "
              f"({tar_path.stat().st_size / 1024:.1f} KB)")
        with tarfile.open(tar_path) as tf:
            tf.extractall(dest)
            names = tf.getnames()

    print(f"extracted {len(names)} file(s) to {dest}")
    for n in sorted(names)[:30]:
        info_path = dest / n
        sz = info_path.stat().st_size if info_path.exists() else 0
        print(f"  {n}  ({sz/1024:.1f} KB)")
    if len(names) > 30:
        print(f"  ... ({len(names) - 30} more)")
    return dest


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("case_id", help="Flow360 case ID, e.g. case-49ce08a1-...")
    retrieve(ap.parse_args().case_id)
