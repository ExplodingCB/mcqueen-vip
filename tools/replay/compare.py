"""ROS-independent, fail-closed field comparison used by the MCAP driver."""

import math
from collections.abc import Mapping, Sequence
from numbers import Real


class ReplayMismatch(AssertionError):
    pass


def fields(value, prefix=""):
    """Flatten generated ROS messages, dictionaries and sequences without lossy JSON conversion."""
    if hasattr(value, "get_fields_and_field_types"):
        value = {name: getattr(value, name) for name in value.get_fields_and_field_types()}
    if isinstance(value, Mapping):
        for name, child in value.items():
            yield from fields(child, f"{prefix}.{name}" if prefix else name)
    elif isinstance(value, Sequence) and not isinstance(value, str | bytes):
        yield f"{prefix}.length", len(value)
        for index, child in enumerate(value):
            yield from fields(child, f"{prefix}[{index}]")
    elif hasattr(value, "tolist"):
        yield from fields(value.tolist(), prefix)
    else:
        yield prefix, value


def compare(expected, actual, *, topic, timestamp_ns, tolerances):
    """Compare every leaf. Unlisted fields are exact, including flags and heartbeat."""
    wanted, got = dict(fields(expected)), dict(fields(actual))
    for path in sorted(wanted.keys() | got.keys()):
        before, after = wanted.get(path), got.get(path)
        tolerance = tolerances.get(path, {})
        absolute, relative = tolerance.get("abs", 0.0), tolerance.get("rel", 0.0)
        if not all(math.isfinite(v) and v >= 0 for v in (absolute, relative)):
            raise ValueError(f"invalid tolerance for {topic}.{path}: {tolerance}")
        if path not in wanted or path not in got:
            equal = False
        elif isinstance(before, bool) or isinstance(after, bool):
            equal = type(before) is type(after) and before == after
        elif isinstance(before, Real) and isinstance(after, Real):
            equal = (
                math.isfinite(before)
                and math.isfinite(after)
                and abs(before - after) <= absolute + relative * abs(before)
            )
        else:
            equal = before == after
        if not equal:
            seconds, nanoseconds = divmod(timestamp_ns, 1_000_000_000)
            raise ReplayMismatch(
                f"{topic} field={path} timestamp={seconds}.{nanoseconds:09d} "
                f"({timestamp_ns} ns): expected={before!r} actual={after!r} "
                f"tolerance_abs={absolute} tolerance_rel={relative}"
            )
    unused = tolerances.keys() - wanted.keys()
    if unused:
        raise ValueError(f"unknown tolerance fields for {topic}: {sorted(unused)}")
