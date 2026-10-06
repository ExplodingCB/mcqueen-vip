#!/usr/bin/env python3
"""Compare full-run checker metrics; never claim exact equality from noisy rates."""

import argparse
import json
from pathlib import Path


def compare(baseline, dashboard, tolerance):
    failures = []
    for name, run in (("baseline", baseline), ("dashboard", dashboard)):
        if run["exit_code"] != 0 or run["urgent_stops"] != 0 or run["progress_m"] < 120:
            failures.append(f"{name}: graph did not complete 120 m without an urgent stop")
    for topic in ("ego_state", "vehicle_command", "trajectory"):
        before, after = baseline["rates"][topic], dashboard["rates"][topic]
        if before <= 0 or after <= 0 or abs(after / before - 1) > tolerance:
            failures.append(f"{topic}: {before:.2f} -> {after:.2f} Hz exceeds {tolerance:.1%} tolerance")
    return failures


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline", type=Path)
    parser.add_argument("dashboard", type=Path)
    parser.add_argument("--tolerance", type=float, default=0.05)
    args = parser.parse_args()
    if not 0 <= args.tolerance < 1:
        parser.error("tolerance must be in [0, 1)")
    baseline, dashboard = (json.loads(p.read_text()) for p in (args.baseline, args.dashboard))
    failures = compare(baseline, dashboard, args.tolerance)
    print(json.dumps({"baseline": baseline, "dashboard": dashboard}, indent=2))
    print("\n".join(failures) if failures else f"PASS: no urgent stops; rates within {args.tolerance:.1%}")
    return bool(failures)


if __name__ == "__main__":
    raise SystemExit(main())
