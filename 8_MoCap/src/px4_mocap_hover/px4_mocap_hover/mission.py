"""Pure mission state machine for the PX4 motion-capture hover node."""

from dataclasses import dataclass
from enum import Enum, auto
import math
from typing import Optional, Tuple


class MissionState(Enum):
    WAITING_FOR_DATA = auto()
    PRESTREAM = auto()
    REQUEST_OFFBOARD = auto()
    REQUEST_ARM = auto()
    TAKEOFF = auto()
    HOVER = auto()
    REQUEST_LAND = auto()
    WAIT_DISARM = auto()
    ABORTED = auto()
    COMPLETE = auto()


@dataclass(frozen=True)
class MissionConfig:
    takeoff_height: float = 1.0
    hover_duration: float = 10.0
    position_tolerance: float = 0.15
    stable_duration: float = 1.0
    prestream_duration: float = 1.0
    command_retry_interval: float = 1.0


@dataclass(frozen=True)
class MissionInput:
    now: float
    mocap_fresh: bool
    mocap_stable: bool
    position_stable: bool
    local_valid: bool
    start_allowed: bool
    position: Tuple[float, float, float]
    heading: float
    offboard: bool
    armed: bool
    landing: bool


@dataclass(frozen=True)
class MissionOutput:
    publish_offboard: bool = False
    request_offboard: bool = False
    request_arm: bool = False
    request_land: bool = False
    shutdown: bool = False


class HoverMission:
    """Coordinate mission transitions without ROS dependencies."""

    PREARM_STATES = {
        MissionState.PRESTREAM,
        MissionState.REQUEST_OFFBOARD,
        MissionState.REQUEST_ARM,
    }
    FLIGHT_STATES = {
        MissionState.TAKEOFF,
        MissionState.HOVER,
    }

    def __init__(self, config: MissionConfig) -> None:
        self.config = config
        self.state = MissionState.WAITING_FOR_DATA
        self.target: Optional[Tuple[float, float, float, float]] = None
        self.state_since = 0.0
        self.reached_since: Optional[float] = None
        self.last_command_at = -math.inf
        self.abort_reason = ''

    def _transition(self, state: MissionState, now: float) -> None:
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
        self.target = None
        self._transition(MissionState.WAITING_FOR_DATA, now)

    def update(self, data: MissionInput) -> MissionOutput:
        """Advance the mission and return actions for the current control tick."""
        if self.state in self.FLIGHT_STATES and not data.mocap_fresh:
            self.abort_reason = 'motion capture timed out'
            self._transition(MissionState.ABORTED, data.now)
            return MissionOutput()

        if (
            self.state in self.PREARM_STATES
            and not data.armed
            and not self._prearm_ready(data)
        ):
            self._wait_for_data(data.now)
            return MissionOutput()

        if self.state == MissionState.WAITING_FOR_DATA:
            if self._prearm_ready(data):
                x, y, z = data.position
                self.target = (x, y, z - self.config.takeoff_height, data.heading)
                self._transition(MissionState.PRESTREAM, data.now)
            return MissionOutput()

        if self.state == MissionState.PRESTREAM:
            if data.now - self.state_since >= self.config.prestream_duration:
                self._transition(MissionState.REQUEST_OFFBOARD, data.now)
            return MissionOutput(publish_offboard=True)

        if self.state == MissionState.REQUEST_OFFBOARD:
            if data.offboard:
                self._transition(MissionState.REQUEST_ARM, data.now)
                return MissionOutput(publish_offboard=True)
            return MissionOutput(
                publish_offboard=True,
                request_offboard=self._command_due(data.now),
            )

        if self.state == MissionState.REQUEST_ARM:
            if not data.offboard:
                self._transition(MissionState.REQUEST_OFFBOARD, data.now)
                return MissionOutput(publish_offboard=True)
            if data.armed:
                self._transition(MissionState.TAKEOFF, data.now)
                return MissionOutput(publish_offboard=True)
            return MissionOutput(
                publish_offboard=True,
                request_arm=self._command_due(data.now),
            )

        if self.state == MissionState.TAKEOFF:
            if not data.offboard:
                self.abort_reason = 'PX4 left Offboard during takeoff'
                self._transition(MissionState.ABORTED, data.now)
                return MissionOutput()
            if self._at_target(data.position):
                if self.reached_since is None:
                    self.reached_since = data.now
                elif data.now - self.reached_since >= self.config.stable_duration:
                    self._transition(MissionState.HOVER, data.now)
            else:
                self.reached_since = None
            return MissionOutput(publish_offboard=True)

        if self.state == MissionState.HOVER:
            if not data.offboard:
                self.abort_reason = 'PX4 left Offboard during hover'
                self._transition(MissionState.ABORTED, data.now)
                return MissionOutput()
            if data.now - self.state_since >= self.config.hover_duration:
                self._transition(MissionState.REQUEST_LAND, data.now)
                return MissionOutput(
                    publish_offboard=True,
                    request_land=self._command_due(data.now),
                )
            return MissionOutput(publish_offboard=True)

        if self.state == MissionState.REQUEST_LAND:
            if not data.armed:
                self._transition(MissionState.COMPLETE, data.now)
                return MissionOutput(shutdown=True)
            if data.landing:
                self._transition(MissionState.WAIT_DISARM, data.now)
                return MissionOutput()
            return MissionOutput(
                publish_offboard=True,
                request_land=self._command_due(data.now),
            )

        if self.state == MissionState.WAIT_DISARM:
            if not data.armed:
                self._transition(MissionState.COMPLETE, data.now)
                return MissionOutput(shutdown=True)
            if not data.landing:
                self._transition(MissionState.REQUEST_LAND, data.now)
                return MissionOutput(
                    publish_offboard=True,
                    request_land=self._command_due(data.now),
                )
            return MissionOutput()

        return MissionOutput()

    def _at_target(self, position: Tuple[float, float, float]) -> bool:
        if self.target is None:
            return False
        return math.dist(position, self.target[:3]) <= self.config.position_tolerance
