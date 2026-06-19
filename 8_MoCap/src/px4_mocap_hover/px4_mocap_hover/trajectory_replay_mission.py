"""Pure state machine for replaying an absolute-NED CSV trajectory."""

from dataclasses import dataclass
from enum import Enum, auto
import math
from typing import Optional, Sequence, Tuple

from px4_mocap_hover.mission import MissionInput, MissionOutput
from px4_mocap_hover.trajectory_replay import TrajectorySample


class TrajectoryReplayState(Enum):
    """States used by the guarded trajectory replay mission."""

    WAITING_FOR_DATA = auto()
    PRESTREAM = auto()
    REQUEST_OFFBOARD = auto()
    REQUEST_ARM = auto()
    TAKEOFF = auto()
    HOVER = auto()
    MOVE_TO_START = auto()
    PLAYBACK = auto()
    REQUEST_LAND = auto()
    WAIT_DISARM = auto()
    ABORTED = auto()
    COMPLETE = auto()


@dataclass(frozen=True)
class TrajectoryReplayConfig:
    """Configuration values for trajectory replay."""

    takeoff_height: float = 1.2
    hover_duration: float = 3.0
    position_tolerance: float = 0.15
    stable_duration: float = 1.0
    prestream_duration: float = 1.0
    command_retry_interval: float = 1.0


class TrajectoryReplayMission:
    """Take off, move to the first absolute XY point, replay, and land."""

    PREARM_STATES = {
        TrajectoryReplayState.PRESTREAM,
        TrajectoryReplayState.REQUEST_OFFBOARD,
        TrajectoryReplayState.REQUEST_ARM,
    }
    FLIGHT_STATES = {
        TrajectoryReplayState.TAKEOFF,
        TrajectoryReplayState.HOVER,
        TrajectoryReplayState.MOVE_TO_START,
        TrajectoryReplayState.PLAYBACK,
    }

    def __init__(
        self,
        config: TrajectoryReplayConfig,
        trajectory: Sequence[TrajectorySample],
    ) -> None:
        if not trajectory:
            raise ValueError('trajectory must contain at least one sample')
        self.config = config
        self.trajectory = tuple(trajectory)
        self.state = TrajectoryReplayState.WAITING_FOR_DATA
        self.takeoff_target: Optional[
            Tuple[float, float, float, float]
        ] = None
        self.target: Optional[Tuple[float, float, float, float]] = None
        self.playback_index: Optional[int] = None
        self.state_since = 0.0
        self.reached_since: Optional[float] = None
        self.last_command_at = -math.inf
        self.abort_reason = ''

    @property
    def playback_row(self) -> int:
        """Return the one-based current playback row, or zero before replay."""
        if self.playback_index is None:
            return 0
        return self.playback_index + 1

    def _transition(
            self, state: TrajectoryReplayState, now: float) -> None:
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
        self.playback_index = None
        self._transition(TrajectoryReplayState.WAITING_FOR_DATA, now)

    def _abort(self, reason: str, now: float) -> MissionOutput:
        self.abort_reason = reason
        self._transition(TrajectoryReplayState.ABORTED, now)
        return MissionOutput()

    def update(self, data: MissionInput) -> MissionOutput:
        """Advance the mission and return actions for this control tick."""
        if self.state in self.FLIGHT_STATES:
            if not data.mocap_fresh:
                return self._abort('motion capture timed out', data.now)
            if not data.local_valid:
                return self._abort(
                    'PX4 local position became invalid', data.now)
            if not data.offboard:
                return self._abort(
                    'PX4 left Offboard during trajectory replay', data.now)
            if not data.armed:
                return self._abort(
                    'vehicle disarmed during trajectory replay', data.now)

        if (
            self.state in self.PREARM_STATES
            and not data.armed
            and not self._prearm_ready(data)
        ):
            self._wait_for_data(data.now)
            return MissionOutput()

        if self.state == TrajectoryReplayState.WAITING_FOR_DATA:
            if self._prearm_ready(data):
                x, y, z = data.position
                self.takeoff_target = (
                    x,
                    y,
                    z - self.config.takeoff_height,
                    data.heading,
                )
                self.target = self.takeoff_target
                self._transition(
                    TrajectoryReplayState.PRESTREAM, data.now)
            return MissionOutput()

        if self.state == TrajectoryReplayState.PRESTREAM:
            if data.now - self.state_since >= self.config.prestream_duration:
                self._transition(
                    TrajectoryReplayState.REQUEST_OFFBOARD, data.now)
            return MissionOutput(publish_offboard=True)

        if self.state == TrajectoryReplayState.REQUEST_OFFBOARD:
            if data.offboard:
                self._transition(
                    TrajectoryReplayState.REQUEST_ARM, data.now)
                return MissionOutput(publish_offboard=True)
            return MissionOutput(
                publish_offboard=True,
                request_offboard=self._command_due(data.now),
            )

        if self.state == TrajectoryReplayState.REQUEST_ARM:
            if not data.offboard:
                self._transition(
                    TrajectoryReplayState.REQUEST_OFFBOARD, data.now)
                return MissionOutput(publish_offboard=True)
            if data.armed:
                self._transition(
                    TrajectoryReplayState.TAKEOFF, data.now)
                return MissionOutput(publish_offboard=True)
            return MissionOutput(
                publish_offboard=True,
                request_arm=self._command_due(data.now),
            )

        if self.state == TrajectoryReplayState.TAKEOFF:
            if self._at_target(data.position):
                if self.reached_since is None:
                    self.reached_since = data.now
                elif (
                    data.now - self.reached_since
                    >= self.config.stable_duration
                ):
                    self._transition(
                        TrajectoryReplayState.HOVER, data.now)
            else:
                self.reached_since = None
            return MissionOutput(publish_offboard=True)

        if self.state == TrajectoryReplayState.HOVER:
            if data.now - self.state_since >= self.config.hover_duration:
                self._set_start_target()
                self._transition(
                    TrajectoryReplayState.MOVE_TO_START, data.now)
            return MissionOutput(publish_offboard=True)

        if self.state == TrajectoryReplayState.MOVE_TO_START:
            if self._at_target(data.position):
                if self.reached_since is None:
                    self.reached_since = data.now
                elif (
                    data.now - self.reached_since
                    >= self.config.stable_duration
                ):
                    self._set_playback_target(0)
                    self._transition(
                        TrajectoryReplayState.PLAYBACK, data.now)
            else:
                self.reached_since = None
            return MissionOutput(publish_offboard=True)

        if self.state == TrajectoryReplayState.PLAYBACK:
            assert self.playback_index is not None
            next_index = self.playback_index + 1
            if next_index < len(self.trajectory):
                self._set_playback_target(next_index)
                return MissionOutput(publish_offboard=True)

            self._transition(
                TrajectoryReplayState.REQUEST_LAND, data.now)
            return MissionOutput(
                publish_offboard=True,
                request_land=self._command_due(data.now),
            )

        if self.state == TrajectoryReplayState.REQUEST_LAND:
            if not data.armed:
                self._transition(
                    TrajectoryReplayState.COMPLETE, data.now)
                return MissionOutput(shutdown=True)
            if data.landing:
                self._transition(
                    TrajectoryReplayState.WAIT_DISARM, data.now)
                return MissionOutput()
            return MissionOutput(
                publish_offboard=True,
                request_land=self._command_due(data.now),
            )

        if self.state == TrajectoryReplayState.WAIT_DISARM:
            if not data.armed:
                self._transition(
                    TrajectoryReplayState.COMPLETE, data.now)
                return MissionOutput(shutdown=True)
            if not data.landing:
                self._transition(
                    TrajectoryReplayState.REQUEST_LAND, data.now)
                return MissionOutput(
                    publish_offboard=True,
                    request_land=self._command_due(data.now),
                )
            return MissionOutput()

        return MissionOutput()

    def _set_start_target(self) -> None:
        if self.takeoff_target is None:
            return
        first = self.trajectory[0]
        self.target = (
            first.north_m,
            first.east_m,
            self.takeoff_target[2],
            self.takeoff_target[3],
        )

    def _set_playback_target(self, index: int) -> None:
        if self.takeoff_target is None:
            return
        sample = self.trajectory[index]
        self.playback_index = index
        self.target = (
            sample.north_m,
            sample.east_m,
            self.takeoff_target[2],
            self.takeoff_target[3],
        )

    def _at_target(self, position: Tuple[float, float, float]) -> bool:
        if self.target is None:
            return False
        return (
            math.dist(position, self.target[:3])
            <= self.config.position_tolerance
        )
