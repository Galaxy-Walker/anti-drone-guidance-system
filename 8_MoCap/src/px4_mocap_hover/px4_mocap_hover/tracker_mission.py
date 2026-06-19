"""Pure mission state machine for motion-capture target tracking."""

from dataclasses import dataclass
from enum import Enum, auto
import math
from typing import Optional, Tuple


class TrackerState(Enum):
    """States used by the tracking mission."""

    WAITING_FOR_DATA = auto()
    PRESTREAM = auto()
    REQUEST_OFFBOARD = auto()
    REQUEST_ARM = auto()
    TAKEOFF = auto()
    WAIT_TARGET = auto()
    TRACKING = auto()
    ABORTED = auto()


@dataclass(frozen=True)
class TrackerMissionConfig:
    """Configuration for takeoff and mission transitions."""

    takeoff_height: float = 1.2
    position_tolerance: float = 0.15
    stable_duration: float = 1.0
    prestream_duration: float = 1.0
    command_retry_interval: float = 1.0


@dataclass(frozen=True)
class TrackerMissionInput:
    """Vehicle and sensor state consumed on each control tick."""

    now: float
    self_mocap_fresh: bool
    self_mocap_stable: bool
    position_stable: bool
    local_valid: bool
    start_allowed: bool
    position: Tuple[float, float, float]
    heading: float
    offboard: bool
    armed: bool
    target_fresh: bool
    target_ready: bool


@dataclass(frozen=True)
class TrackerMissionOutput:
    """Actions requested by the tracking state machine."""

    publish_position: bool = False
    publish_velocity: bool = False
    request_offboard: bool = False
    request_arm: bool = False


class TrackerMission:
    """Coordinate takeoff and continuous target tracking."""

    PREARM_STATES = {
        TrackerState.PRESTREAM,
        TrackerState.REQUEST_OFFBOARD,
        TrackerState.REQUEST_ARM,
    }
    FLIGHT_STATES = {
        TrackerState.TAKEOFF,
        TrackerState.WAIT_TARGET,
        TrackerState.TRACKING,
    }

    def __init__(self, config: TrackerMissionConfig) -> None:
        """Create a tracking mission with the supplied configuration."""
        self.config = config
        self.state = TrackerState.WAITING_FOR_DATA
        self.takeoff_target: Optional[
            Tuple[float, float, float, float]
        ] = None
        self.state_since = 0.0
        self.reached_since: Optional[float] = None
        self.last_command_at = -math.inf
        self.abort_reason = ''

    def _transition(self, state: TrackerState, now: float) -> None:
        self.state = state
        self.state_since = now
        self.reached_since = None
        self.last_command_at = -math.inf

    def _command_due(self, now: float) -> bool:
        if now - self.last_command_at >= self.config.command_retry_interval:
            self.last_command_at = now
            return True
        return False

    @staticmethod
    def _prearm_ready(data: TrackerMissionInput) -> bool:
        return (
            data.self_mocap_fresh
            and data.self_mocap_stable
            and data.position_stable
            and data.local_valid
            and data.start_allowed
        )

    def _wait_for_data(self, now: float) -> None:
        self.takeoff_target = None
        self._transition(TrackerState.WAITING_FOR_DATA, now)

    def _abort(self, reason: str, now: float) -> TrackerMissionOutput:
        self.abort_reason = reason
        self._transition(TrackerState.ABORTED, now)
        return TrackerMissionOutput()

    def update(self, data: TrackerMissionInput) -> TrackerMissionOutput:
        """Advance the mission and return actions for this control tick."""
        if self.state in self.FLIGHT_STATES:
            if not data.self_mocap_fresh:
                return self._abort('self motion capture timed out', data.now)
            if not data.local_valid:
                return self._abort(
                    'PX4 local position became invalid', data.now)
            if not data.offboard:
                return self._abort(
                    'PX4 left Offboard during tracking mission', data.now)
            if not data.armed:
                return self._abort(
                    'vehicle disarmed during tracking mission', data.now)

        if (
            self.state in self.PREARM_STATES
            and not data.armed
            and not self._prearm_ready(data)
        ):
            self._wait_for_data(data.now)
            return TrackerMissionOutput()

        if self.state == TrackerState.WAITING_FOR_DATA:
            if self._prearm_ready(data):
                x, y, z = data.position
                self.takeoff_target = (
                    x,
                    y,
                    z - self.config.takeoff_height,
                    data.heading,
                )
                self._transition(TrackerState.PRESTREAM, data.now)
            return TrackerMissionOutput()

        if self.state == TrackerState.PRESTREAM:
            if data.now - self.state_since >= self.config.prestream_duration:
                self._transition(TrackerState.REQUEST_OFFBOARD, data.now)
            return TrackerMissionOutput(publish_position=True)

        if self.state == TrackerState.REQUEST_OFFBOARD:
            if data.offboard:
                self._transition(TrackerState.REQUEST_ARM, data.now)
                return TrackerMissionOutput(publish_position=True)
            return TrackerMissionOutput(
                publish_position=True,
                request_offboard=self._command_due(data.now),
            )

        if self.state == TrackerState.REQUEST_ARM:
            if not data.offboard:
                self._transition(TrackerState.REQUEST_OFFBOARD, data.now)
                return TrackerMissionOutput(publish_position=True)
            if data.armed:
                self._transition(TrackerState.TAKEOFF, data.now)
                return TrackerMissionOutput(publish_position=True)
            return TrackerMissionOutput(
                publish_position=True,
                request_arm=self._command_due(data.now),
            )

        if self.state == TrackerState.TAKEOFF:
            if self._at_takeoff_target(data.position):
                if self.reached_since is None:
                    self.reached_since = data.now
                elif (
                    data.now - self.reached_since
                    >= self.config.stable_duration
                ):
                    next_state = (
                        TrackerState.TRACKING
                        if data.target_fresh and data.target_ready
                        else TrackerState.WAIT_TARGET
                    )
                    self._transition(next_state, data.now)
                    return TrackerMissionOutput(publish_velocity=True)
            else:
                self.reached_since = None
            return TrackerMissionOutput(publish_position=True)

        if self.state == TrackerState.WAIT_TARGET:
            if data.target_fresh and data.target_ready:
                self._transition(TrackerState.TRACKING, data.now)
            return TrackerMissionOutput(publish_velocity=True)

        if self.state == TrackerState.TRACKING:
            if not data.target_fresh or not data.target_ready:
                self._transition(TrackerState.WAIT_TARGET, data.now)
            return TrackerMissionOutput(publish_velocity=True)

        return TrackerMissionOutput()

    def _at_takeoff_target(
            self, position: Tuple[float, float, float]) -> bool:
        if self.takeoff_target is None:
            return False
        return (
            math.dist(position, self.takeoff_target[:3])
            <= self.config.position_tolerance
        )
