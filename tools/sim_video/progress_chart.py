"""One picture of how every driving-model run and the lap-time search are going.

    python tools/sim_video/progress_chart.py --out docs/img/driving-model/progress.png

Reads ``training/perception/runs/*/metrics.jsonl``. The four panels are the ones
that decide whether a model can drive: how far its racing line is from the
expert's near the kart and farther out, how far its speed request is off, and
the best simulated lap time the optimizer has found. Dashed lines mark what a
model that ignores the image would score, and one minute for the lap.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

RUNS = Path(__file__).resolve().parents[2] / "training" / "perception" / "runs"
COLORS = {"drive-v1": "#6ea2ff", "drive-v2": "#ff8a5c", "drive-v3": "#4cd39b", "laptime-v1": "#b98cf0"}


def epochs(run: str) -> list[dict]:
    path = RUNS / run / "metrics.jsonl"
    if not path.exists():
        return []
    rows = []
    for line in path.read_text().splitlines():
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            continue
        if row.get("type") == "epoch":
            rows.append(row)
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    plt.rcParams.update(
        {
            "figure.facecolor": "#141517",
            "axes.facecolor": "#1c1e21",
            "axes.edgecolor": "#2c2f34",
            "text.color": "#ececea",
            "axes.labelcolor": "#9a9ea6",
            "xtick.color": "#9a9ea6",
            "ytick.color": "#9a9ea6",
            "grid.color": "#2c2f34",
            "font.size": 10,
        }
    )
    fig, axes = plt.subplots(2, 2, figsize=(12, 7.5))
    panels = [
        (
            axes[0, 0],
            "line_error_near_m",
            "Racing-line error, 2 to 10 m ahead (m)",
            [(1.34, "blind guess 1.34"), (0.3, "target 0.3")],
        ),
        (axes[0, 1], "line_error_far_m", "Racing-line error, 12 to 24 m ahead (m)", [(6.78, "blind guess 6.78")]),
        (axes[1, 0], "speed_error_mps", "Target-speed error (m/s)", [(2.14, "blind guess 2.14")]),
    ]
    for ax, key, title, refs in panels:
        for run in ("drive-v1", "drive-v2", "drive-v3"):
            rows = [r for r in epochs(run) if key in r]
            if rows:
                ax.plot(
                    [r["epoch"] for r in rows], [r[key] for r in rows], "o-", color=COLORS[run], label=run, lw=2, ms=4
                )
        for y, label in refs:
            ax.axhline(y, color="#9a9ea6", ls="--", lw=1)
            ax.text(
                0.99, y, label, transform=ax.get_yaxis_transform(), ha="right", va="bottom", fontsize=8, color="#9a9ea6"
            )
        ax.set_title(title, fontsize=10, loc="left")
        ax.set_xlabel("epoch")
        ax.grid(True, lw=0.5)
        ax.legend(loc="upper right", fontsize=8, frameon=False)
    ax = axes[1, 1]
    rows = [r for r in epochs("laptime-v1") if r.get("best_lap_s") is not None]
    if rows:
        ax.plot(
            [r["epoch"] for r in rows], [r["best_lap_s"] for r in rows], "o-", color=COLORS["laptime-v1"], lw=2, ms=3
        )
        ax.annotate(
            f"{rows[-1]['best_lap_s']:.1f} s",
            (rows[-1]["epoch"], rows[-1]["best_lap_s"]),
            textcoords="offset points",
            xytext=(-34, 10),
            color=COLORS["laptime-v1"],
        )
    ax.axhline(60, color="#9a9ea6", ls="--", lw=1)
    ax.text(
        0.99, 60, "one minute", transform=ax.get_yaxis_transform(), ha="right", va="bottom", fontsize=8, color="#9a9ea6"
    )
    ax.set_title("Lap-time search: best lap, slower of nominal and sloppy kart (s)", fontsize=10, loc="left")
    ax.set_xlabel("generation")
    ax.grid(True, lw=0.5)
    fig.suptitle("Driving-model progress (simulator)", x=0.01, ha="left", fontsize=13)
    fig.tight_layout()
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=110)
    print(args.out)


if __name__ == "__main__":
    main()
