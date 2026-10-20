#!/usr/bin/env python3
"""Print track geometry: python tools/track_info.py tracks/synthetic_oval."""

import argparse
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src" / "mcq_sim"))

from mcq_sim.track import Track  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path, help="directory containing track.csv and track.yaml")
    args = parser.parse_args()
    track = Track.load(args.directory)
    points = len(track.x)
    curvature = float(np.max(np.abs(track.curvature_at(track.s))))
    radius = f"{1 / curvature:.3f} m" if curvature else "infinite (straight track)"
    print(f"Track: {track.track_id}")
    print(f"Closed loop: {'yes' if track.closed else 'no'}")
    print(f"Points: {points}")
    print(f"Average point spacing: {track.length / (points if track.closed else points - 1):.3f} m")
    print(f"Total length: {track.length:.3f} m")
    for side, widths in zip(("Left", "Right"), track.width_at(track.s), strict=True):
        print(f"{side} half-width: min {widths.min():.3f}, max {widths.max():.3f}, mean {widths.mean():.3f} m")
    print(f"Maximum absolute curvature (at track points): {curvature:.6f} 1/m")
    print(f"Corresponding radius: {radius}")
    print(f"Datum: {track.meta.get('datum') or 'not specified'}")
    print(f"Survey date: {track.meta.get('survey_date') or 'not specified'}")


if __name__ == "__main__":
    main()
