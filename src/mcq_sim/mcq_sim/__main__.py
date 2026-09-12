"""Command line: run the closed loop or write a synthetic track.

python -m mcq_sim run --track tracks/synthetic_oval --laps 3
python -m mcq_sim run --track oval --mode BOUNDARY --set planner.v_cap=4
python -m mcq_sim make-track --out tracks/synthetic_oval
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from mcq_sim.harness import run_closed_loop
from mcq_sim.params import load_params
from mcq_sim.track import Raceline, Track


def _parse_set(items):
    out = {}
    for item in items or []:
        key, value = item.split("=", 1)
        try:
            out[key] = float(value)
        except ValueError:
            out[key] = value
    return out


def cmd_run(args) -> int:
    params = load_params(args.config).override(_parse_set(args.set))
    if args.speed_cap is not None:
        params.planner["v_cap"] = args.speed_cap
        params.planner["boundary_v_cap"] = min(args.speed_cap, params.planner["boundary_v_cap"])
    track = Track.synthetic_oval() if args.track == "oval" else Track.load(args.track)
    raceline = Raceline.from_csv(args.raceline) if args.raceline else None
    believed = track.shifted(args.shift[0], args.shift[1]) if args.shift else None
    result = run_closed_loop(
        track,
        params,
        laps=args.laps,
        mode=args.mode,
        raceline=raceline,
        believed_track=believed,
        seed=args.seed,
    )
    print(result.summary())
    if args.out:
        result.write_csv(args.out)
        print(f"log written to {args.out}")
    return 0 if result.passed else 1


def cmd_make_track(args) -> int:
    track = Track.synthetic_oval(args.straight, args.radius, args.width, args.spacing, track_id=Path(args.out).name)
    track.save(args.out)
    print(f"wrote {args.out}/track.csv ({len(track.x)} points, {track.length:.1f} m) and track.yaml")
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog="mcq_sim", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = parser.add_subparsers(dest="command", required=True)

    run = sub.add_parser("run", help="run the closed loop on a track")
    run.add_argument("--track", default="oval", help="track directory, or 'oval' for the built-in synthetic oval")
    run.add_argument("--laps", type=int, default=3)
    run.add_argument("--mode", choices=["FOLLOW", "BOUNDARY"], default="FOLLOW")
    run.add_argument("--raceline", help="TUM raceline csv to use as the FOLLOW reference speed")
    run.add_argument("--speed-cap", type=float, help="override planner.v_cap (m/s)")
    run.add_argument("--config", help="parameter yaml (default: config/sim_default.yaml)")
    run.add_argument("--set", action="append", metavar="SECTION.KEY=VALUE", help="override a parameter")
    run.add_argument(
        "--shift", type=float, nargs=2, metavar=("DX", "DY"), help="shift the believed map (fault injection 15)"
    )
    run.add_argument("--seed", type=int, default=0)
    run.add_argument("--out", help="write the per-tick log as csv")
    run.set_defaults(func=cmd_run)

    mk = sub.add_parser("make-track", help="write a synthetic oval track directory")
    mk.add_argument("--out", required=True)
    mk.add_argument("--straight", type=float, default=60.0)
    mk.add_argument("--radius", type=float, default=15.0)
    mk.add_argument("--width", type=float, default=5.0)
    mk.add_argument("--spacing", type=float, default=1.0)
    mk.set_defaults(func=cmd_make_track)

    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
