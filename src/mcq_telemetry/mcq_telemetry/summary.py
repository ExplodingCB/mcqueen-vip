"""Pure summary helpers; safe to exercise on the host without ROS."""

import math


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def age_and_valid(message, now, limit):
    if message is None:
        return math.nan, False
    age = now - stamp_seconds(message.header.stamp)
    return age, 0 <= age <= limit


def target_speed(points, age):
    """Same time interpolation as ControllerNode::target_at (including endpoints)."""
    if not points:
        return math.nan
    if age <= points[0].t:
        return points[0].v
    for previous, point in zip(points, points[1:], strict=False):
        if age <= point.t:
            span = point.t - previous.t
            fraction = (age - previous.t) / span if span > 1e-9 else 0.0
            return previous.v + fraction * (point.v - previous.v)
    return points[-1].v


def fault_names(flags, definitions):
    names = [name.removeprefix("FAULT_") for name, bit in definitions.items() if flags & bit]
    known = 0
    for bit in definitions.values():
        known |= bit
    unknown = flags & ~known
    if unknown:
        names.append(f"UNKNOWN_BITS(0x{unknown:x})")
    return ", ".join(names) or "NONE"
