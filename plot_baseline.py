"""Render the measured output of a Processing run (no physical data)."""
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parent
with (root / "PathTwin/sim_run.csv").open() as f:
    rows = list(csv.DictReader(f))
groups = {}
for r in rows:
    groups.setdefault(r["segment"], []).append(r)
fig, (xy, pwm) = plt.subplots(1, 2, figsize=(11, 4.8), layout="constrained")
motion = ("straight_slow", "straight_fast", "left_arc", "reverse")
for name in motion:
    block = groups[name]
    block = block + [rows[rows.index(block[-1]) + 1]]
    xy.plot([float(r["x_mm"]) for r in block], [float(r["y_mm"]) for r in block],
            lw=2.6, label=name.replace("_", " "))
p = groups["rotate_in_place"][0]
xy.scatter(float(p["x_mm"]), float(p["y_mm"]), marker="o", s=50, facecolors="none",
           edgecolors="black", label="in-place rotation")
xy.scatter(float(rows[-1]["x_mm"]), float(rows[-1]["y_mm"]), marker="x", c="black")
xy.set(xlabel="x (mm)", ylabel="y (mm)", title="Predicted path", aspect="equal")
xy.legend(loc="upper center", bbox_to_anchor=(0.5, -0.34), fontsize=8, ncol=2)
for field, name in (("left_pwm", "Left"), ("right_pwm", "Right")):
    pwm.step([int(r["t_ms"])/1000 for r in rows], [int(r[field]) for r in rows],
             where="post", label=name, linestyle="--" if name == "Right" else "-")
pwm.set(xlabel="Time (s)", ylabel="Requested PWM", title="Identical input schedule for both projects")
pwm.legend()
for ax in (xy, pwm):
    ax.grid(alpha=.25)
fig.suptitle("Path Reproduction — Processing baseline only")
fig.savefig(root / "results/simulation_baseline.png", dpi=170)
