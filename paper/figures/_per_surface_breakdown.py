"""
Compare per-surface CL / CD / CMy between cont-low (Step 0) and cont-high
(Step 1) at the BO trim point of each phase, to answer:
'why does raising the htail INCREASE total drag and REDUCE total lift?'

Pulls surface_forces.csv from each phase's parent case and decomposes
the integrated aerodynamic coefficients by surface (main_wing, vane,
aft_flap, htail).  Writes a clean Markdown table to
paper/figures/per_surface_breakdown.md and a side-by-side bar plot to
paper/figures/per_surface_breakdown.png.

Run from repo root.
"""
from __future__ import annotations
import importlib.util, sys, tempfile, re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "post"))
import flow360 as fl


def load_spec(fam: str, phase: str):
    pp = REPO / "post" / fam / f"plot_{phase}_sensitivities.py"
    s = importlib.util.spec_from_file_location("s", pp)
    m = importlib.util.module_from_spec(s); s.loader.exec_module(m)
    return m.SPEC


def find_parent_case(spec) -> "fl.Case":
    proj = fl.Project.from_cloud(project_id=spec.project_id)
    # Pick the case whose name contains 'parent' or 'BO_estimate'; if
    # none, fall back to any case whose alpha/htail/thrust tokens match
    # the BO trim point.
    candidates = []
    for cid in proj.get_case_ids():
        c = fl.Case.from_cloud(case_id=cid)
        if "COMPLETED" not in str(c.status):
            continue
        n = c.name.lower()
        if "parent" in n or "bo_estimate" in n:
            candidates.append(c)
    if not candidates:
        raise RuntimeError(f"no parent case found in {spec.project_id}")
    return candidates[0]


def pull_surface_breakdown(case: "fl.Case") -> dict[str, dict[str, float]]:
    """Returns {surface_name: {CL: ..., CD: ..., CMy: ...}}."""
    with tempfile.TemporaryDirectory() as tmp:
        case.results.surface_forces.download(to_folder=tmp, overwrite=True)
        # download() returns None in this SDK; the file lands at
        # <to_folder>/surface_forces_v2.csv .
        sf_path = next(Path(tmp).rglob("surface_forces*.csv"))
        sf = pd.read_csv(sf_path)
    sf.columns = [c.strip() for c in sf.columns]
    last_row = sf.iloc[-1]
    # column names like "ac_pitch_zone/main_wing_CL"; pick only the
    # plain CL/CD/CMy columns (no _Pressure, _SkinFriction suffixes).
    pat = re.compile(r"^(?:[\w]+/)?(?P<surf>[\w]+)_(?P<comp>CL|CD|CMy)$")
    out: dict[str, dict[str, float]] = {}
    for col in sf.columns:
        m = pat.match(col)
        if not m: continue
        surf, comp = m.group("surf"), m.group("comp")
        # de-duplicate "ac_pitch_zone/main_wing_CL" vs the same column
        # without the zone prefix; either is fine.
        if "Pressure" in col or "Friction" in col: continue
        out.setdefault(surf, {})[comp] = float(last_row[col])
    return out


SURFACE_GROUPS = {
    "main_wing": ["main_wing"],
    "vane":      ["vane_left", "vane_right"],
    "aft_flap":  ["aft_flap_left", "aft_flap_right"],
    "htail":     ["htail"],
}


def merge_surfaces(per_surf: dict[str, dict[str, float]]) -> dict[str, dict[str, float]]:
    """Combine left+right pairs into single rows for readability."""
    merged = {}
    for group, sources in SURFACE_GROUPS.items():
        vals = {"CL": 0.0, "CD": 0.0, "CMy": 0.0}
        n = 0
        for s in sources:
            if s in per_surf:
                for k in vals: vals[k] += per_surf[s].get(k, 0.0)
                n += 1
        if n: merged[group] = vals
    return merged


def main():
    rows = []
    for phase in ("cruise", "takeoff", "landing"):
        for fam in ("v2_continuous", "v2_continuous_high"):
            spec = load_spec(fam, phase)
            case = find_parent_case(spec)
            print(f"  {fam:22s} {phase:8s}  -> {case.name} ({case.id[:18]})")
            per = merge_surfaces(pull_surface_breakdown(case))
            total = {k: sum(per[g][k] for g in per) for k in ("CL", "CD", "CMy")}
            for g, v in per.items():
                rows.append(dict(phase=phase, config=fam, surface=g,
                                 CL=v["CL"], CD=v["CD"], CMy=v["CMy"]))
            rows.append(dict(phase=phase, config=fam, surface="TOTAL",
                             CL=total["CL"], CD=total["CD"], CMy=total["CMy"]))
    df = pd.DataFrame(rows)
    df.to_csv(HERE / "per_surface_breakdown.csv", index=False)

    # Markdown table per phase
    md_lines = ["# Per-surface aerodynamic breakdown at BO trim points",
                "",
                "Comparing cont-low (Step 0; low-htail + continuous flap) vs",
                "cont-high (Step 1; T-tail + continuous flap) at the *same* BO",
                "trim point, surface-by-surface.  CL, CD, CMy are integrated",
                "coefficients on each surface (wind axes, referenced to",
                "$S_w$, $c_w$, CG).",
                ""]
    for phase in ("cruise", "takeoff", "landing"):
        sub = df[df["phase"] == phase]
        md_lines += [f"## {phase} (BO α / θ_ht / T_mult per phase plotter)", ""]
        md_lines += ["| surface | cont-low CL | cont-high CL | ΔCL | cont-low CD | cont-high CD | ΔCD | cont-low CMy | cont-high CMy | ΔCMy |",
                     "|---|---|---|---|---|---|---|---|---|---|"]
        surfaces = sub["surface"].drop_duplicates().tolist()
        for s in surfaces:
            cl_lo = float(sub[(sub["config"] == "v2_continuous")     & (sub["surface"] == s)]["CL"].iloc[0])
            cl_hi = float(sub[(sub["config"] == "v2_continuous_high")& (sub["surface"] == s)]["CL"].iloc[0])
            cd_lo = float(sub[(sub["config"] == "v2_continuous")     & (sub["surface"] == s)]["CD"].iloc[0])
            cd_hi = float(sub[(sub["config"] == "v2_continuous_high")& (sub["surface"] == s)]["CD"].iloc[0])
            cm_lo = float(sub[(sub["config"] == "v2_continuous")     & (sub["surface"] == s)]["CMy"].iloc[0])
            cm_hi = float(sub[(sub["config"] == "v2_continuous_high")& (sub["surface"] == s)]["CMy"].iloc[0])
            md_lines.append(
                f"| {s:9s} | {cl_lo:+.3f} | {cl_hi:+.3f} | {cl_hi - cl_lo:+.3f} | "
                f"{cd_lo:+.3f} | {cd_hi:+.3f} | {cd_hi - cd_lo:+.3f} | "
                f"{cm_lo:+.3f} | {cm_hi:+.3f} | {cm_hi - cm_lo:+.3f} |"
            )
        md_lines.append("")
    (HERE / "per_surface_breakdown.md").write_text("\n".join(md_lines))

    # Side-by-side bar plot
    fig, axes = plt.subplots(3, 3, figsize=(13.5, 9.5))
    plt.subplots_adjust(hspace=0.5, wspace=0.3)
    for r, phase in enumerate(("cruise", "takeoff", "landing")):
        sub = df[df["phase"] == phase]
        surfaces = [s for s in sub["surface"].unique() if s != "TOTAL"]
        for c, metric in enumerate(("CL", "CD", "CMy")):
            ax = axes[r, c]
            x = np.arange(len(surfaces))
            lo = [float(sub[(sub["config"]=="v2_continuous")     & (sub["surface"]==s)][metric].iloc[0]) for s in surfaces]
            hi = [float(sub[(sub["config"]=="v2_continuous_high")& (sub["surface"]==s)][metric].iloc[0]) for s in surfaces]
            ax.bar(x - 0.18, lo, width=0.36, label="cont-low",  color="#5b8def")
            ax.bar(x + 0.18, hi, width=0.36, label="cont-high", color="#c0392b")
            ax.axhline(0, color="black", lw=0.6)
            ax.set_xticks(x); ax.set_xticklabels(surfaces, rotation=20, fontsize=9)
            ax.set_title(f"{phase} — {metric}", fontsize=10)
            ax.grid(True, axis="y", alpha=0.3)
            if r == 0 and c == 0: ax.legend(fontsize=8)
    fig.suptitle("Per-surface aerodynamic breakdown: cont-low vs cont-high at BO",
                  fontsize=12, y=0.995)
    fig.savefig(HERE / "per_surface_breakdown.png", dpi=160, bbox_inches="tight")
    print(f"wrote {HERE / 'per_surface_breakdown.csv'}")
    print(f"wrote {HERE / 'per_surface_breakdown.md'}")
    print(f"wrote {HERE / 'per_surface_breakdown.png'}")


if __name__ == "__main__":
    main()
