"""
Build one STEP file per Study 1 case by driving ESP's serveCSM in batch
mode with per-case despmtr overrides written to a temp file.

    source $HOME/esp/ESP129/EngSketchPad/ESPenv.sh
    python flow360/build_geometry.py
"""

from __future__ import annotations

import subprocess, sys, tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
import params as P

CSM        = REPO / "geometry" / "tsangpo_stowed.csm"  # cruise reference
OUT_DIR    = REPO / "geometry" / "out"
AIRFOILS   = REPO / "geometry" / "airfoils"       # the 30P30N .dat3 files


def main() -> int:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for case in P.study1_matrix():
        case_dir = OUT_DIR / case.name
        case_dir.mkdir(exist_ok=True)

        # The CSM references $airfoils/30P30N_*.dat3 relative to serveCSM's
        # cwd (= case_dir). Drop a symlink so the fit reads the canonical
        # files instead of duplicating them per case.
        link = case_dir / "airfoils"
        if link.is_symlink() or link.exists():
            link.unlink()
        link.symlink_to(AIRFOILS, target_is_directory=True)

        with tempfile.NamedTemporaryFile("w", suffix=".despmtrs", delete=False) as f:
            f.write(f"gap_fraction  {case.gap_fraction}\n")
            f.write(f"Z_tail_chords {case.z_tail_chords}\n")
            f.write(f"X_tail_mac    {case.x_tail_ft / P.WING_MAC_FT}\n")
            despmtrs = f.name
        print(f"[build] {case.name}  gap={case.gap_fraction}  z={case.z_tail_chords:+.2f}c")
        subprocess.run(
            ["serveCSM", str(CSM), "-batch", "-despmtrs", despmtrs, "-outLevel", "0"],
            cwd=case_dir, check=True,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
