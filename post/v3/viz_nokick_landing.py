"""Trajectory + attitude visualization for the LANDING no-kick phugoid chain.

Self-contained: reads each completed landing-chunk case's user log directly
via fl.Case.logs.print(), parses the per-physical-step UDD outputs to
reconstruct (t, x_cg, z_cg, θ_body), and emits a stitched PNG + MP4.

Parameters specific to landing:
  V_INF   = 12.86 m/s   (25 kt approach speed, SL freestream)
  A_INF   = 340.29 m/s  (sea-level speed of sound)
  RHO_INF = 1.225 kg/m³ (sea-level ISA density)
  dt      = 0.025 s     (half of cruise's 0.05 s — landing dynamics ~3× faster)
  warmup_step_offset = 30 (warmup ran 30 physical steps before chunk1)
  UDD name pattern: phugoid_6DOF_landing_<tag>

Outputs:
  post/v3/phugoid_runs/landing/nokick_landing_trajectory.png   (static)
  post/v3/phugoid_runs/landing/nokick_landing_attitude.mp4     (animation)

FRAME: air-relative.  The freestream blows +X at V_inf; we subtract V_inf·t
to show the body's path THROUGH the air mass.  CSM convention is +X = aft,
so the nose unit vector in (x,z) is (−cosθ, +sinθ).
"""
from __future__ import annotations

import json
import math
import re
import sys
from io import StringIO
from pathlib import Path

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
import flow360 as fl

# Landing-specific constants.
V_INF              = 12.86          # m/s
A_INF              = 340.29         # m/s (SL ISA)
RHO_INF            = 1.225          # kg/m³ (SL ISA)
S_REF              = 15.33          # m² (wing area)
W_GROSS_N          = 11565.0
DT_S               = 0.025
WARMUP_STEP_OFFSET = 30             # warmup ran 30 physical steps
L1 = L2            = 1038.215114993035   # arm length (= 3× R_arm, same as cruise geometry)

# UDD output-vars are published per cylinder; we read 'shoulder' for everything
# since all four UDDs publish the same forces/moments in lockstep.
UDD_PHASE = "landing"

OUT_DIR = REPO / "post" / "v3" / "phugoid_runs" / "landing"
OUT_DIR.mkdir(parents=True, exist_ok=True)


# ---------------------------------------------------------------------------
def download_user_log(case_id: str) -> str:
    c = fl.Case.from_cloud(case_id=case_id)
    c.logs.set_remote_log_file_name("logs/flow360_case.user.log")
    buf = StringIO()
    save = sys.stdout
    sys.stdout = buf
    c.logs.print()
    sys.stdout = save
    out = Path(f"/tmp/{case_id}_userlog.txt")
    out.write_text(buf.getvalue())
    return str(out)


def parse_log(log_path: str) -> list[dict]:
    """Return one dict per converged physical step.

    Each row: {step, t_s, x_cg_m, z_cg_m, theta_body_deg, fX_nd, fZ_nd, mY_nd}.
    """
    text = Path(log_path).read_text()
    rows: list[dict] = []
    for part in re.split(r"(?=Physical step = )", text):
        m = re.search(r"Physical step = (\d+)", part)
        if not m:
            continue
        step = int(m.group(1))
        head = "\n".join(part.split("\n")[:200])
        udd: dict = {}
        for tag in ("shoulder", "elbow", "airframe", "htail"):
            blk_pat = (rf"User Defined Dynamics \(phugoid_6DOF_{UDD_PHASE}_{tag}\):"
                       rf"(.*?)(?=User Defined Dynamics|\Z)")
            mblk = re.search(blk_pat, head, re.DOTALL)
            if not mblk:
                continue
            blk = mblk.group(1)
            def grab(name: str) -> float:
                m2 = re.search(rf"\({name} = ([+-]?[\d.eE+-]+|nan)\)", blk)
                if not m2:
                    return float("nan")
                v = m2.group(1)
                return float("nan") if v == "nan" else float(v)
            udd[tag] = {"fX": grab("forceX"), "fZ": grab("forceZ"),
                        "mY": grab("momentY"), "th": grab("theta")}
        if not udd or "shoulder" not in udd:
            continue
        # Reconstruct CG + body pitch from link angles (Option E forward IK).
        t1 = udd["shoulder"]["th"]
        t2 = udd["elbow"]["th"]
        th_afr = udd["airframe"]["th"]
        try:
            cos_t2 = math.cos(t2)
            r2 = 2.0 * L1 * L2 * (1.0 - cos_t2)
            r = math.sqrt(r2) if r2 > 0 else 0.0
            half = math.atan(L2 * math.sin(t2) / max(1e-9, L1 - L2 * cos_t2))
            psi = half - t1
            x_cg = r * math.cos(psi)
            z_cg = r * math.sin(psi)
            th_body = th_afr + t1 + t2
        except Exception:
            x_cg = z_cg = th_body = float("nan")
        rows.append({"step": step,
                     "t_s": (step - WARMUP_STEP_OFFSET) * DT_S,
                     "fX_nd": udd["shoulder"]["fX"],
                     "fZ_nd": udd["shoulder"]["fZ"],
                     "mY_nd": udd["shoulder"]["mY"],
                     "x_cg_m": x_cg, "z_cg_m": z_cg,
                     "theta_body_rad": th_body,
                     "theta_body_deg": (math.degrees(th_body)
                                         if not math.isnan(th_body)
                                         else float("nan"))})
    return rows


# ---------------------------------------------------------------------------
def collect_chunks() -> list[tuple[str, str]]:
    """Return [(label, case_id), …] for COMPLETED chunks in time order."""
    ids = json.loads((REPO / "post/v3/unsteady_project_ids.json").read_text())
    ordered = [("chunk1", ids.get("phugoid_landing_gap110_nokick_diag_chunk1"))]
    for n in range(2, 7):
        k = f"phugoid_landing_gap110_nokick_chunk{n}"
        if ids.get(k):
            ordered.append((f"chunk{n}", ids[k]))
    out = []
    for label, cid in ordered:
        if cid is None:
            print(f"  {label}: not submitted yet — stop")
            break
        st = str(fl.Case.from_cloud(case_id=cid).status).split(".")[-1]
        if st not in ("COMPLETED", "COMPLETED_WITH_WARNINGS"):
            print(f"  {label} ({cid[:13]}): {st} — stop including here")
            break
        out.append((label, cid))
        print(f"  including {label} ({cid[:13]})")
    return out


def main() -> None:
    chunks = collect_chunks()
    if not chunks:
        print("no completed landing chunks yet"); sys.exit(0)

    t, x, z, th = [], [], [], []
    for label, cid in chunks:
        log = download_user_log(cid)
        rows = [r for r in parse_log(log) if not math.isnan(r["x_cg_m"])]
        t  += [r["t_s"]            for r in rows]
        x  += [r["x_cg_m"]         for r in rows]
        z  += [r["z_cg_m"]         for r in rows]
        th += [r["theta_body_deg"] for r in rows]
        print(f"  {label}: {len(rows)} valid steps  "
              f"(t in [{rows[0]['t_s']:.3f}, {rows[-1]['t_s']:.3f}] s)")
    if not t:
        print("no usable rows"); sys.exit(0)

    order = np.argsort(t)
    t  = np.array(t)[order]
    x  = np.array(x)[order]
    z  = np.array(z)[order]
    th = np.array(th)[order]

    xr = (x - x[0]) - V_INF * t   # air-relative path
    zr = (z - z[0])
    a  = np.radians(th)
    nose = np.column_stack([-np.cos(a), np.sin(a)])   # (-cosθ, +sinθ)

    # ---- Static figure ----
    fig, axes = plt.subplots(2, 2, figsize=(14, 9), constrained_layout=True)

    ax = axes[0, 0]
    sc = ax.scatter(xr, zr, c=t, cmap="viridis", s=14, zorder=3)
    Lb = 6.0   # landing fuselage tick length (m)
    for i in range(0, len(t), max(1, len(t)//40)):
        nx, nz = nose[i]
        ax.plot([xr[i] - nx*Lb, xr[i] + nx*Lb],
                [zr[i] - nz*Lb, zr[i] + nz*Lb],
                "-", color="crimson", lw=1.2, alpha=0.7, zorder=2)
        ax.plot([xr[i] + nx*Lb], [zr[i] + nz*Lb], "o",
                color="crimson", ms=2.5, zorder=4)
    # World-frame +X axis to the right (matches the LIC slice viz, which
    # also has world +X right).  Airplane flies in world −X direction
    # (CSM convention: +X = aft), so the body's nose appears to point
    # LEFT in this orientation, and the flight path reads RIGHT → LEFT.
    ax.set_xlabel("x through air [m]  (world +X →; flight is −X)")
    ax.set_ylabel("altitude gain Δz [m]")
    ax.set_title("Flight path through the air (color=time)\n"
                  "red fuselage ticks, dot = nose")
    plt.colorbar(sc, ax=ax, label="t [s]")
    ax.grid(alpha=0.3); ax.axhline(0, color="k", lw=0.4)
    ax.set_aspect("equal", adjustable="box")

    ax = axes[0, 1]
    ax.plot(t, th, "-", lw=2, color="#1f77b4")
    ax.axhline(th[0], color="gray", ls=":", label=f"trim θ={th[0]:.1f}°")
    ax.set_xlabel("t [s]"); ax.set_ylabel("θ_body [deg]  (+ = nose up)")
    ax.set_title("Body pitch — landing phugoid")
    ax.grid(alpha=0.3); ax.legend(fontsize=9)

    ax = axes[1, 0]
    ax.plot(t, zr, "-", lw=2, color="#2ca02c")
    ax.set_xlabel("t [s]"); ax.set_ylabel("altitude gain Δz [m]")
    ax.set_title("Altitude through the phugoid")
    ax.grid(alpha=0.3); ax.axhline(0, color="k", lw=0.4)

    ax = axes[1, 1]
    omega = np.gradient(th, t)
    ax.plot(t, omega, "-", lw=2, color="#d62728")
    ax.set_xlabel("t [s]"); ax.set_ylabel("ω = dθ/dt [deg/s]")
    ax.set_title("Pitch rate")
    ax.grid(alpha=0.3); ax.axhline(0, color="k", lw=0.4)

    fig.suptitle(f"Landing no-kick phugoid — {len(chunks)} chunk(s) stitched, "
                  f"t ∈ [{t[0]:.2f}, {t[-1]:.2f}] s, world frame +X→  "
                  f"(V=12.86 m/s, α_BO=+27.6°, T_mult=14.17, γ_trim=−30°)",
                  fontsize=12, fontweight="bold")
    out_png = OUT_DIR / "nokick_landing_trajectory.png"
    fig.savefig(out_png, dpi=130)
    print(f"wrote {out_png}")

    # ---- Animation ----
    # World-frame +X to the right (matches LIC slice viz).  Airplane
    # flies in world −X direction, so the body moves RIGHT → LEFT here.
    fig2, ax2 = plt.subplots(figsize=(10, 6))
    ax2.set_xlim(xr.min() - 5, xr.max() + 5)
    ax2.set_ylim(zr.min() - 5, zr.max() + 5)
    ax2.set_xlabel("x through air [m]  (world +X →)")
    ax2.set_ylabel("altitude Δz [m]")
    ax2.grid(alpha=0.3); ax2.set_aspect("equal", adjustable="box")
    path_line, = ax2.plot([], [], "-", color="#bbb", lw=1)
    body_line, = ax2.plot([], [], "-", color="crimson", lw=3)
    nose_dot,  = ax2.plot([], [], "o", color="darkred", ms=6)
    title = ax2.set_title("")
    Lb_anim = 8.0

    def step(i: int):
        path_line.set_data(xr[:i+1], zr[:i+1])
        nx, nz = nose[i]
        body_line.set_data([xr[i] - nx*Lb_anim, xr[i] + nx*Lb_anim],
                           [zr[i] - nz*Lb_anim, zr[i] + nz*Lb_anim])
        nose_dot.set_data([xr[i] + nx*Lb_anim], [zr[i] + nz*Lb_anim])
        title.set_text(f"t={t[i]:.2f}s   θ_body={th[i]:+.1f}°   "
                        f"(nose −X, tilted by θ)")
        return path_line, body_line, nose_dot, title

    ani = animation.FuncAnimation(fig2, step, frames=len(t),
                                   interval=50, blit=True)
    out_mp4 = OUT_DIR / "nokick_landing_attitude.mp4"
    try:
        ani.save(str(out_mp4), writer="ffmpeg", dpi=110, fps=20)
        print(f"wrote {out_mp4}")
    except Exception as e:
        print(f"mp4 save failed: {e}")


if __name__ == "__main__":
    main()
