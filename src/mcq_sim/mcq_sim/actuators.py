"""A steering servo that does not land where it is told.

The simulator's steering is a smooth first-order lag that always reaches the
commanded angle. A real hobby-class or industrial servo on a kart does not:

  gain      it delivers a fraction of the commanded angle (linkage, calibration)
  bias      its centre is not the kart's centre (a steady offset)
  deadband  it ignores changes smaller than a threshold, and when it does move
            it lags by that threshold on reversal (backlash in the gears and
            the steering linkage)
  jitter    it hunts around its target by a small random amount

``ServoImperfection`` is applied to the steering the stack commands, before the
simulated kart sees it. The stack reads the kart's real steering angle back (the
steering encoder), so the controllers see what the servo actually did and can
correct for it by feedback; the driving model never sees any of this. It is used
to test and to train against a servo worse than the simulator's.

The numbers in ``DEGRADED`` and ``sample`` are placeholders for the unmeasured
real servo. The roadmap's steering step and frequency response (docs/05) gives
the real ones.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

STEER_MAX = 0.45  # rad, the kart's steering lock


@dataclass
class ServoImperfection:
    gain: float = 1.0  # delivered / commanded
    bias: float = 0.0  # rad, added after the gain
    deadband: float = 0.0  # rad, backlash width
    jitter: float = 0.0  # rad, 1 sigma of the random error on each command
    seed: int = 0

    def __post_init__(self):
        self.rng = np.random.default_rng(self.seed)
        self._held = 0.0  # where the gearing last stopped

    def apply(self, command: float) -> float:
        """The angle the servo delivers for a commanded angle."""
        target = command * self.gain
        if self.deadband > 0.0:
            # backlash: the output follows the target only once it is pushed across the dead zone
            gap = target - self._held
            if abs(gap) > self.deadband:
                self._held = target - np.sign(gap) * self.deadband
            target = self._held
        out = target + self.bias + (self.rng.normal(0.0, self.jitter) if self.jitter else 0.0)
        return float(np.clip(out, -STEER_MAX, STEER_MAX))

    def install(self, stack) -> None:
        """Route the stack's steering command through this servo."""
        tick = stack.tick

        def imperfect(state):
            steer, throttle, brake = tick(state)
            return self.apply(steer), throttle, brake

        stack.tick = imperfect


# a clearly worse servo than the simulator's: about 1 degree of backlash, a 10% gain
# error, a degree of offset and a little hunting
DEGRADED = {"gain": 0.9, "bias": 0.017, "deadband": 0.017, "jitter": 0.005}


def sample(rng: np.random.Generator, severity: float = 1.0) -> ServoImperfection:
    """A random servo between perfect and a bit worse than ``DEGRADED``."""
    k = float(severity)
    return ServoImperfection(
        gain=1.0 + float(rng.uniform(-0.14, 0.14)) * k,
        bias=float(rng.normal(0.0, 0.015)) * k,
        deadband=float(rng.uniform(0.0, 0.03)) * k,
        jitter=float(rng.uniform(0.0, 0.008)) * k,
        seed=int(rng.integers(1 << 31)),
    )
