"""
Cruise trim-centered campaign — non-GAI replacement for
submit_gai_trim_campaign.py.

The GAI volume mesher has a bug with nested rotation zones (sliding
interface 7221 — see flow360/LESSONS.md), so this re-runs the same
parent + 30 sweep forks at the linear-trim solution (α=+6.78°,
θ_ht=-0.54°, T_mult=2.13) using the legacy beta mesher.

    python3 flow360/submit_cruise_trim_campaign.py
"""
from __future__ import annotations

import os
import sys
from math import radians
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
import cfd_setup as C
import flow360 as fl

ALPHA_TRIM_DEG    = +6.78
THETA_HT_TRIM_DEG = -0.54
T_MULT_TRIM       = +2.13

ALPHA_SWEEP_DEG = (-2.0, +2.0, +5.0, +7.0, +9.0, +11.0, +13.0, +16.0, +19.0, +22.0)
HTAIL_SWEEP_DEG = (-5.0, -4.0, -3.0, -2.0, -1.0, 0.0, +1.0, +2.0, +3.0, +4.0)
THRUST_SWEEP    = (1.0, 1.4, 1.7, 1.9, 2.1, 2.3, 2.5, 2.7, 3.0, 3.5)

N_PARENT_STEPS = 20
N_FORK_TOTAL   = N_PARENT_STEPS + 10

PROJECT_ID = os.environ.get(
    "TSANGPO_PROJECT_ID",
    "prj-3e8b1ed8-0109-4f1f-8738-fef0c55a573b",
)
project = fl.Project.from_cloud(PROJECT_ID)
surfaces = C.get_geometry_surfaces(project)
print(f"Project: {project.id} ({project.metadata.name})")

parent_case = project.run_case(
    params=C.build_params(surfaces,
                          theta_ac_rad=radians(ALPHA_TRIM_DEG),
                          theta_ht_rad=radians(THETA_HT_TRIM_DEG),
                          thrust_mult=T_MULT_TRIM,
                          n_steps_total=N_PARENT_STEPS),
    name="cruise_trim_centered",
    run_async=True,
    tags=["SI", "trim_centered", "parent", "non-GAI"],
    use_beta_mesher=True,
)
print(f"parent: {parent_case.id}\n")


def submit_sweep(label, values, theta_ac_deg, theta_ht_deg, thrust_mult, vary):
    out = []
    for v in values:
        ta = radians(v) if vary == "alpha" else radians(theta_ac_deg)
        th = radians(v) if vary == "htail" else radians(theta_ht_deg)
        tm = v          if vary == "thrust" else thrust_mult
        name = (f"trim_{vary}_{v:+.2f}".replace("+", "p")
                                       .replace("-", "m")
                                       .replace(".", "p"))
        case = project.run_case(
            params=C.build_params(surfaces, ta, th, tm,
                                  n_steps_total=N_FORK_TOTAL),
            name=name, run_async=True, fork_from=parent_case,
            tags=["SI", "trim_centered", f"{vary}_sweep", "non-GAI"],
            use_beta_mesher=True,
        )
        out.append((v, case.id))
        print(f"  {label} {v:+6.2f}  →  {case.id}")
    return out


print("=== α sweep ===")
alpha_cases = submit_sweep("α     =", ALPHA_SWEEP_DEG,
                           ALPHA_TRIM_DEG, THETA_HT_TRIM_DEG, T_MULT_TRIM, "alpha")
print("\n=== θ_htail sweep ===")
htail_cases = submit_sweep("θ_ht  =", HTAIL_SWEEP_DEG,
                           ALPHA_TRIM_DEG, THETA_HT_TRIM_DEG, T_MULT_TRIM, "htail")
print("\n=== T_mult sweep ===")
thrust_cases = submit_sweep("T_mult=", THRUST_SWEEP,
                            ALPHA_TRIM_DEG, THETA_HT_TRIM_DEG, T_MULT_TRIM, "thrust")

print(f"\n=== Summary ===\nparent: {parent_case.id}")
for label, lst in (("α     ", alpha_cases),
                   ("θ_ht  ", htail_cases),
                   ("T_mult", thrust_cases)):
    print(f"{label} ({len(lst)}):")
    for v, cid in lst:
        print(f"  {v:+6.2f}  {cid}")
