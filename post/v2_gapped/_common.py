"""
Shared helpers for the v2_gapped phase plotters.

`discover_sweep_cases(project_id, name_prefix, baseline_val)` walks the
project's cases, parses sweep names of the form
    f"{name_prefix}_{vary}_{tok}"  where tok ∈ {p, m, x}<value-with-p-as-dp>
and returns a dict
    {'alpha': [(value, case_id), ...], 'htail': [...], 'thrust': [...],
     'parent': parent_case_id}
"""
from __future__ import annotations
import re
from typing import Iterable

SWEEP_RE = re.compile(r"_(alpha|htail|thrust)_([pmx][\d.p]+)$")


def parse_tok(tok: str) -> float:
    sign = +1
    if tok.startswith("p") or tok.startswith("x"):
        tok = tok[1:]
    elif tok.startswith("m"):
        sign = -1
        tok = tok[1:]
    return sign * float(tok.replace("p", "."))


def discover_sweep_cases(project_id: str, name_prefix: str,
                         alpha_b: float, theta_ht_b: float, T_b: float,
                         parent_substr: str = "_parent"):
    """Returns dict with parent + 3 sweep lists, each sorted by value.
    `parent_substr` matches the parent case name; baselines fill in the
    sweep entry at the parent point so the plotters see a continuous
    range."""
    from flow360 import Project, Case
    p = Project.from_cloud(project_id=project_id)
    out = {"alpha": [], "htail": [], "thrust": [], "parent": None}
    for cid in p.get_case_ids():
        c = Case.from_cloud(case_id=cid)
        if parent_substr in c.name:
            out["parent"] = cid
            continue
        m = SWEEP_RE.search(c.name)
        if not m:
            continue
        sweep = m.group(1)
        try:
            val = parse_tok(m.group(2))
        except ValueError:
            continue
        out[sweep].append((val, cid))
    if out["parent"]:
        # Add the parent at the baseline point of each sweep so the plotter
        # sees the BO point inside each sweep range (matches the continuous-
        # flap convention).
        out["alpha"].append((alpha_b, out["parent"]))
        out["htail"].append((theta_ht_b, out["parent"]))
        out["thrust"].append((T_b, out["parent"]))
    for k in ("alpha", "htail", "thrust"):
        out[k] = sorted(out[k])
    return out
