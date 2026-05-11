"""
Expand the Study 1 case matrix into Flow360 case JSONs by filling
flow360/case_template.json with values from params.py. Writes
flow360/cases/<case>.json and flow360/manifest.json.

    python flow360/run_matrix.py            # write JSONs
    python flow360/run_matrix.py --submit   # also submit via the Flow360 SDK
"""

from __future__ import annotations

import argparse, copy, json, sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
import params as P

TEMPLATE = REPO / "flow360" / "case_template.json"
OUT_DIR  = REPO / "flow360" / "cases"
MANIFEST = REPO / "flow360" / "manifest.json"


def _actuator_disks(case: P.Case) -> list[dict]:
    R = P.PROP_RADIUS_FT
    force_per_area = case.tw * P.W_GROSS_LBF / (P.N_PROPS * P.A_DISK_PER_PROP_FT2)
    return [
        {
            "name":         f"prop_{side}{i}",
            "center":       [P.PROP_X_FT, y_sign * y, P.PROP_Z_FT],
            "axisThrust":   [-1.0, 0.0, 0.0],
            "thickness":    0.05 * R,
            "outerRadius":  R,
            "innerRadius":  0.15 * R,
            "forcePerArea": force_per_area,
            "swirl":        {"model": "constantCt", "Ct_q": 0.012},
        }
        for side, y_sign in (("R", +1), ("L", -1))
        for i, y in enumerate(P.PROP_Y_FT, start=1)
    ]


def _build(case: P.Case) -> dict:
    cfg = copy.deepcopy(json.loads(TEMPLATE.read_text()))
    cfg["geometry"]["refArea"]      = P.WING_AREA_FT2
    cfg["geometry"]["momentLength"] = P.WING_MAC_FT
    cfg["freestream"]["alphaAngle"] = case.alpha_deg
    cfg["freestream"]["Mach"]       = P.MACH_INF
    cfg["freestream"]["muRef"]      = P.MU_12K_SLUG_FT_S
    cfg["freestream"]["Reynolds"]   = P.RE_MAC
    cfg["actuatorDisks"]            = _actuator_disks(case)
    cfg["outputs"]["slices"][1]["origin"][1] = P.PROP_Y_FT[0]
    cfg["outputs"]["slices"][2]["origin"][0] = case.x_tail_ft
    cfg["_provenance"] = {
        "case_name":     case.name,
        "gap_fraction":  case.gap_fraction,
        "z_tail_chords": case.z_tail_chords,
        "x_tail_ft":     case.x_tail_ft,
        "alpha_deg":     case.alpha_deg,
        "TW":            case.tw,
    }
    return cfg


def _submit(case: P.Case, cfg: dict) -> dict:
    import flow360 as f3  # crash loud if not installed
    step = REPO / "geometry" / "out" / case.name / "airframe.step"
    project = f3.Project.from_geometry(name=f"himalaya/{case.name}", files=[str(step)])
    mesh    = project.generate_volume_mesh(params=REPO / "flow360" / "mesh_params.json")
    run     = project.run_case(params=cfg, name=f"{case.name}_alpha{int(case.alpha_deg)}")
    return {"mesh_id": mesh.id, "case_id": run.id}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--submit", action="store_true")
    args = ap.parse_args()

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    manifest = []
    for case in P.study1_matrix():
        cfg = _build(case)
        path = OUT_DIR / f"{case.name}.json"
        path.write_text(json.dumps(cfg, indent=2))
        print(f"[write] {path.relative_to(REPO)}")
        entry = {
            "case_name":     case.name,
            "config_path":   str(path.relative_to(REPO)),
            "gap_fraction":  case.gap_fraction,
            "z_tail_chords": case.z_tail_chords,
        }
        if args.submit:
            entry.update(_submit(case, cfg))
            print(f"[submit] {case.name}: case_id={entry['case_id']}")
        manifest.append(entry)

    MANIFEST.write_text(json.dumps(manifest, indent=2))
    print(f"\nWrote {MANIFEST.relative_to(REPO)} ({len(manifest)} cases)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
