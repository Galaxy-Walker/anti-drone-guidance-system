"""Pure state machine for the PX4 arm-only test node."""

from dataclasses import dataclass
from enum import Enum, auto
import math


class ArmTestState(Enum):
    WAITING_FOR_DATA = auto()
    REQUEST_ARM = auto()
    ARMED = auto()
    REQUEST_DISARM = auto()
    COMPLETE = auto()


@dataclass(frozen=True)
class ArmTestConfig:
    armed_duration: float = 3.0
    arm_request_timeout: float = 10.0
    command_retry_interval: float = 1.0


@dataclass(frozen=True)
class ArmTestInput:
    now: float
    data_ready: bool
    prearm_safety_ok: bool
    armed_safety_ok: bool
    armed: bool


@dataclass(frozen=True)
class ArmTestOutput:
    request_arm: bool = False
    request_disarm: bool = False
    shutdown: bool = False


class ArmTestMission:
    """Arm after guarded checks, then automatically disarm."""

    def __init__(self, config: ArmTestConfig) -> None:
        self.config = config
        self.state = ArmTestState.WAITING_FOR_DATA
        self.state_since = 0.0
        self.last_command_at = -math.inf
        self.abort_reason = ''

    def _transition(self, state: ArmTestState, now: float) -> None:
        self.state = state
        self.state_since = now
        self.last_command_at = -math.inf

    def _command_due(self, now: float) -> bool:
        if now - self.last_command_at >= self.config.command_retry_interval:
            self.last_command_at = now
            return True
        return False

    def update(self, data: ArmTestInput) -> ArmTestOutput:
        """Advance the test and return command requests for this tick."""
        if self.state == ArmTestState.WAITING_FOR_DATA:
            if data.data_ready:
                self._transition(ArmTestState.REQUEST_ARM, data.now)
            return ArmTestOutput()

        if self.state == ArmTestState.REQUEST_ARM:
            if data.armed:
                if not data.armed_safety_ok:
                    self.abort_reason = (
                        'safety conditions became invalid while arming')
                    self._transition(ArmTestState.REQUEST_DISARM, data.now)
                    self.last_command_at = data.now
                    return ArmTestOutput(request_disarm=True)
                self._transition(ArmTestState.ARMED, data.now)
                return ArmTestOutput()
            if not data.prearm_safety_ok:
                self.abort_reason = 'pre-arm safety conditions became invalid'
                self._transition(ArmTestState.REQUEST_DISARM, data.now)
                self.last_command_at = data.now
                return ArmTestOutput(request_disarm=True)
            if data.now - self.state_since >= self.config.arm_request_timeout:
                self.abort_reason = 'arming request timed out'
                self._transition(ArmTestState.REQUEST_DISARM, data.now)
                self.last_command_at = data.now
                return ArmTestOutput(request_disarm=True)
            return ArmTestOutput(request_arm=self._command_due(data.now))

        if self.state == ArmTestState.ARMED:
            if not data.armed:
                self._transition(ArmTestState.REQUEST_DISARM, data.now)
                self.last_command_at = data.now
                return ArmTestOutput(request_disarm=True)
            if not data.armed_safety_ok:
                self.abort_reason = 'safety conditions became invalid while armed'
                self._transition(ArmTestState.REQUEST_DISARM, data.now)
                self.last_command_at = data.now
                return ArmTestOutput(request_disarm=True)
            if data.now - self.state_since >= self.config.armed_duration:
                self._transition(ArmTestState.REQUEST_DISARM, data.now)
                self.last_command_at = data.now
                return ArmTestOutput(request_disarm=True)
            return ArmTestOutput()

        if self.state == ArmTestState.REQUEST_DISARM:
            if (
                not data.armed
                and data.now - self.state_since
                >= self.config.command_retry_interval
            ):
                self._transition(ArmTestState.COMPLETE, data.now)
                return ArmTestOutput(shutdown=True)
            return ArmTestOutput(
                request_disarm=self._command_due(data.now))

        return ArmTestOutput()
