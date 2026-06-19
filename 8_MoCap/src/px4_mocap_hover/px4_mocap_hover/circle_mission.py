"""Pure state machine for a PX4 motion-capture circle mission."""

from dataclasses import dataclass
from enum import Enum, auto
import math
from typing import Optional, Tuple

from px4_mocap_hover.mission import MissionInput, MissionOutput


class CircleMissionState(Enum):
    """States used by the takeoff, circle, and landing mission."""

    WAITING_FOR_DATA = auto()
    PRESTREAM = auto()
    REQUEST_OFFBOARD = auto()
    REQUEST_ARM = auto()
    TAKEOFF = auto()
    HOVER = auto()
    CIRCLE = auto()
    REQUEST_LAND = auto()
    WAIT_DISARM = auto()
    ABORTED = auto()
    COMPLETE = auto()


@dataclass(frozen=True)
class CircleMissionConfig:
    """Configuration values for the circle mission."""

    takeoff_height: float = 1.0
    hover_duration: float = 3.0
    circle_radius: float = 0.5
    circle_duration: float = 10.0
    position_tolerance: float = 0.25
    stable_duration: float = 1.0
    prestream_duration: float = 1.0
    command_retry_interval: float = 1.0


class CircleMission:
    """Take off, hover, fly one counterclockwise circle, and land."""

    PREARM_STATES = {
        CircleMissionState.PRESTREAM,
        CircleMissionState.REQUEST_OFFBOARD,
        CircleMissionState.REQUEST_ARM,
    }
    FLIGHT_STATES = {
        CircleMissionState.TAKEOFF,
        CircleMissionState.HOVER,
        CircleMissionState.CIRCLE,
    }

    def __init__(self, config: CircleMissionConfig) -> None:
        self.config = config
        self.state = CircleMissionState.WAITING_FOR_DATA
        self.takeoff_target: Optional[
            Tuple[float, float, float, float]
        ] = None
        self.target: Optional[Tuple[float, float, float, float]] = None
        self.state_since = 0.0
        self.reached_since: Optional[float] = None
        self.last_command_at = -math.inf
        self.abort_reason = ''

    def _transition(self, state: CircleMissionState, now: float) -> None:
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
    def _prearm_ready(data: MissionInput) -> bool:
        return (
            data.mocap_fresh
            and data.mocap_stable
            and data.position_stable
            and data.local_valid
            and data.start_allowed
        )

    def _wait_for_data(self, now: float) -> None:
        self.takeoff_target = None
        self.target = None
        self._transition(CircleMissionState.WAITING_FOR_DATA, now)

    def update(self, data: MissionInput) -> MissionOutput:
        """Advance the mission and return actions for this control tick."""
        if self.state in self.FLIGHT_STATES and not data.mocap_fresh:
            self.abort_reason = 'motion capture timed out'
            self._transition(CircleMissionState.ABORTED, data.now)
            return MissionOutput()

        if (
            self.state in self.PREARM_STATES
            and not data.armed
            and not self._prearm_ready(data)
        ):
            self._wait_for_data(data.now)
            return MissionOutput()

        if self.state == CircleMissionState.WAITING_FOR_DATA:
            if self._prearm_ready(data):
                x, y, z = data.position
                self.takeoff_target = (
                    x,
                    y,
                    z - self.config.takeoff_height,
                    data.heading,
                )
                self.target = self.takeoff_target
                self._transition(CircleMissionState.PRESTREAM, data.now)
            return MissionOutput()

        if self.state == CircleMissionState.PRESTREAM:
            if data.now - self.state_since >= self.config.prestream_duration:
                self._transition(
                    CircleMissionState.REQUEST_OFFBOARD, data.now)
            return MissionOutput(publish_offboard=True)

        if self.state == CircleMissionState.REQUEST_OFFBOARD:
            if data.offboard:
                self._transition(CircleMissionState.REQUEST_ARM, data.now)
                return MissionOutput(publish_offboard=True)
            return MissionOutput(
                publish_offboard=True,
                request_offboard=self._command_due(data.now),
            )

        if self.state == CircleMissionState.REQUEST_ARM:
            if not data.offboard:
                self._transition(
                    CircleMissionState.REQUEST_OFFBOARD, data.now)
                return MissionOutput(publish_offboard=True)
            if data.armed:
                self._transition(CircleMissionState.TAKEOFF, data.now)
                return MissionOutput(publish_offboard=True)
            return MissionOutput(
                publish_offboard=True,
                request_arm=self._command_due(data.now),
            )

        if self.state == CircleMissionState.TAKEOFF:
            if not data.offboard:
                return self._abort(
                    'PX4 left Offboard during takeoff', data.now)
            if self._at_target(data.position):
                if self.reached_since is None:
                    self.reached_since = data.now
                elif data.now - self.reached_since >= (
                    self.config.stable_duration
                ):
                    self._transition(CircleMissionState.HOVER, data.now)
            else:
                self.reached_since = None
            return MissionOutput(publish_offboard=True)

        if self.state == CircleMissionState.HOVER:
            if not data.offboard:
                return self._abort(
                    'PX4 left Offboard during hover', data.now)
            if data.now - self.state_since >= self.config.hover_duration:
                self._transition(CircleMissionState.CIRCLE, data.now)
                self._set_circle_target(0.0)
            return MissionOutput(publish_offboard=True)

        if self.state == CircleMissionState.CIRCLE:
            if not data.offboard:
                return self._abort(
                    'PX4 left Offboard during circle', data.now)
            elapsed = max(0.0, data.now - self.state_since)
            if elapsed >= self.config.circle_duration:
                self.target = self.takeoff_target
                self._transition(CircleMissionState.REQUEST_LAND, data.now)
                return MissionOutput(
                    publish_offboard=True,
                    request_land=self._command_due(data.now),
                )
            self._set_circle_target(elapsed)
            return MissionOutput(publish_offboard=True)

        if self.state == CircleMissionState.REQUEST_LAND:
            if not data.armed:
                self._transition(CircleMissionState.COMPLETE, data.now)
                return MissionOutput(shutdown=True)
            if data.landing:
                self._transition(CircleMissionState.WAIT_DISARM, data.now)
                return MissionOutput()
            return MissionOutput(
                publish_offboard=True,
                request_land=self._command_due(data.now),
            )

        if self.state == CircleMissionState.WAIT_DISARM:
            if not data.armed:
                self._transition(CircleMissionState.COMPLETE, data.now)
                return MissionOutput(shutdown=True)
            if not data.landing:
                self._transition(CircleMissionState.REQUEST_LAND, data.now)
                return MissionOutput(
                    publish_offboard=True,
                    request_land=self._command_due(data.now),
                )
            return MissionOutput()

        return MissionOutput()

    def _abort(self, reason: str, now: float) -> MissionOutput:
        self.abort_reason = reason
        self._transition(CircleMissionState.ABORTED, now)
        return MissionOutput()

    def _set_circle_target(self, elapsed: float) -> None:
        if self.takeoff_target is None:
            return
        start_x, start_y, z, yaw = self.takeoff_target
        angle = math.tau * elapsed / self.config.circle_duration
        self.target = (
            start_x + self.config.circle_radius * math.sin(angle),
            start_y - self.config.circle_radius
            + self.config.circle_radius * math.cos(angle),
            z,
            yaw,
        )

    def _at_target(self, position: Tuple[float, float, float]) -> bool:
        if self.target is None:
            return False
        return (
            math.dist(position, self.target[:3])
            <= self.config.position_tolerance
        )
