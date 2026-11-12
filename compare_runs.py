"""Compare synchronized, origin-aligned poses; plotting is optional."""
import argparse
from bisect import bisect_left
import csv
import math
from pathlib import Path
from statistics import mean

ROOT = Path(__file__).resolve().parent
POSE = ("x_mm", "y_mm", "theta_rad")
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("simulation", type=Path)
parser.add_argument("physical", type=Path, help="Arduino CSV or external pose CSV")
parser.add_argument("--kind", choices=("odometry", "external"), default="odometry")
parser.add_argument("--out", type=Path, required=True)
parser.add_argument("--threshold-mm", type=float, default=20)
parser.add_argument("--threshold-deg", type=float, default=5)
parser.add_argument("--max-gap-ms", type=float, default=250)
parser.add_argument("--plot", action="store_true")

def read(path, planned=False):
    lines = path.read_text(encoding="utf-8-sig").splitlines()
    marker = [line for line in lines if line.startswith("# RUN_END ")]
    if marker and (len(marker) != 1 or not marker[0].startswith("# RUN_END complete;")):
        raise ValueError(f"{path}: use exactly one completed run")
    # IDE output should contain one CSV block; preserve the original separately.
    lines = [line for line in lines if line.strip() and not line.startswith("#")]
    rows = list(csv.DictReader(lines))
    fields = ("t_ms", *POSE)
    if planned:
        fields += ("segment", "left_pwm", "right_pwm", "left_count", "right_count")
    if len(rows) < 2 or not set(fields).issubset(rows[0]):
        raise ValueError(f"{path}: expected at least two rows and columns {fields}")
    vals = []
    for row in rows:
        t, *pose = (float(row[k]) for k in ("t_ms", *POSE))
        if not all(map(math.isfinite, (t, *pose))):
            raise ValueError(f"{path}: non-finite input")
        vals.append((t, *pose))
    if vals[0][0] != 0 or any(a[0] >= b[0] for a, b in zip(vals, vals[1:])):
        raise ValueError(f"{path}: timestamps must start at 0 and strictly increase")
    if planned and rows[-1]["segment"] != "done":
        raise ValueError(f"{path}: no completed-run row")
    # Express both trajectories in their own initial robot frame. Preserve time.
    _, x0, y0, th0 = vals[0]
    c, s = math.cos(th0), math.sin(th0)
    norm = [(t, c*(x-x0)+s*(y-y0), -s*(x-x0)+c*(y-y0), th-th0) for t,x,y,th in vals]
    return rows, norm

def at(points, times, t, gap):
    i = bisect_left(times, t)
    if i < len(times) and times[i] == t:
        return points[i][1:]
    if not 0 < i < len(times) or times[i] - times[i-1] > gap:
        return None
    a, b = points[i-1], points[i]
    f = (t-a[0])/(b[0]-a[0])
    return tuple(a[k]+f*(b[k]-a[k]) for k in range(1, 4))

def heading_error(a, b):
    return math.degrees(math.remainder(a-b, 2*math.pi))

def relative_pose(x, y, th, x0, y0, th0):
    c, s = math.cos(th0), math.sin(th0)
    return c*(x-x0)+s*(y-y0), -s*(x-x0)+c*(y-y0), th-th0

def write(path, rows):
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

def main(args):
    sr, sim = read(args.simulation, planned=True)
    rr, real = read(args.physical, planned=args.kind == "odometry")
    if args.kind == "odometry":
        def command_sequence(rows):
            keys = [(r["segment"], int(r["left_pwm"]), int(r["right_pwm"])) for r in rows]
            return [k for i, k in enumerate(keys) if i == 0 or k != keys[i-1]]
        if command_sequence(sr) != command_sequence(rr):
            raise ValueError("Motor sequences differ; verify both projects used the same path plan")
    times = [p[0] for p in real]
    starts = {}
    by_stage = {}
    aligned = []
    for label, (t, x, y, th) in zip(sr, sim):
        segment = label["segment"]
        starts.setdefault(segment, t)
        p = at(real, times, t, args.max_gap_ms)
        if p is None:
            continue
        px, py, pth = p
        d = math.hypot(px-x, py-y)
        h = heading_error(pth, th)
        row = dict(t_ms=t, segment=segment, sim_x_mm=x, sim_y_mm=y, sim_theta_rad=th,
                   physical_x_mm=px, physical_y_mm=py, physical_theta_rad=pth,
                   position_error_mm=d, heading_error_deg=h)
        aligned.append(row)
        by_stage.setdefault(segment, []).append(row)
    if len(aligned) < 2:
        raise ValueError("Not enough synchronized samples; check time origin and coverage")
    summary = []
    for stage, rows in by_stage.items():
        pos = [r["position_error_mm"] for r in rows]
        ang = [r["heading_error_deg"] for r in rows]
        start = rows[0]
        # Rebase at each segment's first matched sample to isolate new drift.
        # The cumulative comparison above remains aligned only at time zero.
        local_pos, local_ang = [], []
        for r in rows:
            sp = relative_pose(*(r[k] for k in ("sim_x_mm", "sim_y_mm", "sim_theta_rad")),
                               *(start[k] for k in ("sim_x_mm", "sim_y_mm", "sim_theta_rad")))
            pp = relative_pose(*(r[k] for k in ("physical_x_mm", "physical_y_mm", "physical_theta_rad")),
                               *(start[k] for k in ("physical_x_mm", "physical_y_mm", "physical_theta_rad")))
            local_pos.append(math.hypot(pp[0]-sp[0], pp[1]-sp[1]))
            local_ang.append(abs(heading_error(pp[2], sp[2])))
        summary.append(dict(segment=stage, samples=len(rows),
            first_aligned_ms=start["t_ms"], last_aligned_ms=rows[-1]["t_ms"],
            position_rmse_mm=math.sqrt(mean(e*e for e in pos)),
            max_position_error_mm=max(pos), heading_rmse_deg=math.sqrt(mean(e*e for e in ang)),
            max_abs_heading_error_deg=max(map(abs, ang)),
            cumulative_pos_error_first_mm=pos[0], cumulative_pos_error_last_mm=pos[-1],
            stage_local_max_position_mm=max(local_pos), stage_local_max_heading_deg=max(local_ang)))
    args.out.mkdir(parents=True, exist_ok=True)
    write(args.out / "aligned.csv", aligned)
    write(args.out / "segments.csv", summary)
    diverged = next((r for r in aligned if r["position_error_mm"] >= args.threshold_mm
                     or abs(r["heading_error_deg"]) >= args.threshold_deg), None)
    report = [f"Reference: {args.kind}", f"Aligned samples: {len(aligned)}/{len(sim)}",
              f"Thresholds: {args.threshold_mm} mm or {args.threshold_deg} deg"]
    if diverged:
        report.append(f"First observed crossing: {diverged['t_ms']:g} ms, {diverged['segment']}")
    else:
        report.append("No threshold crossing in the aligned samples.")
    comparable = [r for r in summary if r["segment"] != "done" and r["samples"] > 1]
    for label, key in (("position", "stage_local_max_position_mm"),
                       ("heading", "stage_local_max_heading_deg")):
        if comparable:
            peak = max(comparable, key=lambda r: r[key])
            report.append(f"Largest segment-local {label} deviation: "
                          + (peak["segment"] if peak[key] > 1e-9 else "none"))
    report.append("Cumulative errors can inherit earlier drift; stage-local errors rebase each stage.")
    report.append("Encoder odometry alone cannot establish physical path accuracy.")
    (args.out / "summary.txt").write_text("\n".join(report) + "\n")
    print(*report, sep="\n")
    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, (xy, err, heading) = plt.subplots(1, 3, figsize=(13, 4.2), layout="constrained")
        xy.plot([p[1] for p in sim], [p[2] for p in sim], label="Processing")
        xy.plot([p[1] for p in real], [p[2] for p in real], "--", label=args.kind)
        xy.set(xlabel="x (mm)", ylabel="y (mm)", title="Trajectory", aspect="equal")
        xy.legend()
        t = [r["t_ms"]/1000 for r in aligned]
        err.plot(t, [r["position_error_mm"] for r in aligned])
        heading.plot(t, [r["heading_error_deg"] for r in aligned])
        err.set(xlabel="Time (s)", ylabel="Position error (mm)", title="Position")
        heading.set(xlabel="Time (s)", ylabel="Heading error (deg)", title="Heading")
        for ax in (xy, err, heading):
            ax.grid(alpha=.25)
        for time in list(starts.values())[1:-1]:
            for ax in (err, heading):
                ax.axvline(time/1000, lw=.7, color=".7")
        fig.savefig(args.out / "comparison.png", dpi=160)

if __name__ == "__main__":
    args = parser.parse_args()
    try:
        if min(args.max_gap_ms, args.threshold_mm, args.threshold_deg) <= 0:
            raise ValueError("Thresholds and maximum gap must be positive")
        main(args)
    except (ValueError, OSError, KeyError) as exc:
        parser.exit(2, f"Error: {exc}\n")
