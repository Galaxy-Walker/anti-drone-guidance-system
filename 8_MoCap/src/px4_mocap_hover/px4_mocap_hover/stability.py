"""Position-estimate stability monitoring without ROS dependencies."""

import math
from typing import Optional, Tuple


Position = Tuple[float, float, float]


class PositionStabilityMonitor:
    """Require position to remain near one reference for a fixed duration."""

    def __init__(self, required_duration: float, tolerance: float) -> None:
        if required_duration <= 0.0:
            raise ValueError('required_duration must be greater than zero')
        if tolerance <= 0.0:
            raise ValueError('tolerance must be greater than zero')
        self.required_duration = required_duration
        self.tolerance = tolerance
        self.reference: Optional[Position] = None
        self.stable_since: Optional[float] = None

    def reset(self) -> None:
        """Discard the current stability window."""
        self.reference = None
        self.stable_since = None

    def update(self, now: float, position: Position, valid: bool = True) -> bool:
        """Consume a position sample and report whether it is stable long enough."""
        if not valid or not all(math.isfinite(value) for value in position):
            self.reset()
            return False

        if (
            self.reference is None
            or math.dist(position, self.reference) > self.tolerance
        ):
            self.reference = position
            self.stable_since = now

        return self.is_stable(now)

    def is_stable(self, now: float) -> bool:
        """Report whether the current uninterrupted window is long enough."""
        return (
            self.stable_since is not None
            and now - self.stable_since >= self.required_duration
        )

    def stable_for(self, now: float) -> float:
        """Return the duration of the current stability window."""
        if self.stable_since is None:
            return 0.0
        return max(0.0, now - self.stable_since)
