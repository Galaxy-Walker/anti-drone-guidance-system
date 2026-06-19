"""Consecutive sample qualification with timeout handling."""

from typing import Optional


class ConsecutiveSampleGate:
    """Require a number of uninterrupted samples before reporting ready."""

    def __init__(self, required_samples: int, timeout: float) -> None:
        """Create a sample gate with count and timing requirements."""
        if required_samples < 1:
            raise ValueError('required_samples must be at least one')
        if timeout <= 0.0:
            raise ValueError('timeout must be greater than zero')
        self.required_samples = required_samples
        self.timeout = timeout
        self.samples = 0
        self.last_sample_at: Optional[float] = None

    @property
    def ready(self) -> bool:
        """Report whether enough consecutive samples have arrived."""
        return self.samples >= self.required_samples

    def reset(self) -> None:
        """Discard all accumulated samples."""
        self.samples = 0
        self.last_sample_at = None

    def accept(self, now: float) -> bool:
        """Record one valid sample and return the updated ready state."""
        if (
            self.last_sample_at is not None
            and now - self.last_sample_at > self.timeout
        ):
            self.reset()
        self.last_sample_at = now
        self.samples += 1
        return self.ready

    def fresh(self, now: float) -> bool:
        """Report freshness, resetting the gate after a timeout."""
        if (
            self.last_sample_at is None
            or now - self.last_sample_at > self.timeout
        ):
            self.reset()
            return False
        return True
