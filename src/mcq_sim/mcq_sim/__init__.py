"""Kart simulator, planner prototype and closed-loop harness for McQueen VIP.

Nothing in this package runs on the kart. It exists to run the Phase 0 exit
test (full loop on a synthetic oval), to prototype the planner before it is
ported, and to generate rollouts for training. The controllers it drives are
the C functions in src/mcq_control/core, loaded through ctypes.
"""

from mcq_sim.params import Params, load_params
from mcq_sim.track import Track

__all__ = ["Params", "Track", "load_params"]
