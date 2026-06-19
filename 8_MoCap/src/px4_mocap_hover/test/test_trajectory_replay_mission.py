import pytest

from px4_mocap_hover.mission import MissionInput
from px4_mocap_hover.trajectory_replay import TrajectorySample
from px4_mocap_hover.trajectory_replay_mission import (
    TrajectoryReplayConfig,
    TrajectoryReplayMission,
    TrajectoryReplayState,
)


def input_at(
        now,
        *,
        mocap_fresh=True,
        mocap_stable=True,
        position_stable=True,
        local_valid=True,
        start_allowed=True,
        position=(5.0, 6.0, 1.0),
        heading=0.4,
        offboard=False,
        armed=False,
        landing=False):
    return MissionInput(
        now=now,
        mocap_fresh=mocap_fresh,
        mocap_stable=mocap_stable,
        position_stable=position_stable,
        local_valid=local_valid,
        start_allowed=start_allowed,
        position=position,
        heading=heading,
        offboard=offboard,
        armed=armed,
        landing=landing,
    )


def sample(index, north, east, down):
    return TrajectorySample(
        elapsed_s=index / 100.0,
        ros_time_ns=1_000_000_000 + index,
        north_m=north,
        east_m=east,
        down_m=down,
    )


def make_mission(trajectory=None):
    if trajectory is None:
        trajectory = (
            sample(0, 1.0, 2.0, -9.0),
            sample(1, 1.1, 2.2, 4.0),
            sample(2, 1.3, 2.5, 8.0),
        )
    return TrajectoryReplayMission(
        TrajectoryReplayConfig(
            takeoff_height=1.2,
            hover_duration=0.3,
            position_tolerance=0.05,
            stable_duration=0.1,
            prestream_duration=0.1,
            command_retry_interval=0.2,
        ),
        trajectory,
    )


def advance_to_takeoff(mission):
    mission.update(input_at(0.0))
    assert mission.target == pytest.approx((5.0, 6.0, -0.2, 0.4))
    mission.update(input_at(0.1))
    mission.update(input_at(0.11, offboard=True))
    mission.update(input_at(0.12, offboard=True, armed=True))
    assert mission.state == TrajectoryReplayState.TAKEOFF


def advance_to_move_to_start(mission):
    advance_to_takeoff(mission)
    target = (5.0, 6.0, -0.2)
    mission.update(input_at(
        0.2, position=target, offboard=True, armed=True))
    mission.update(input_at(
        0.31, position=target, offboard=True, armed=True))
    assert mission.state == TrajectoryReplayState.HOVER
    mission.update(input_at(
        0.62, position=target, offboard=True, armed=True))
    assert mission.state == TrajectoryReplayState.MOVE_TO_START
    assert mission.target == pytest.approx((1.0, 2.0, -0.2, 0.4))


def advance_to_playback(mission):
    advance_to_move_to_start(mission)
    first = (1.0, 2.0, -0.2)
    mission.update(input_at(
        0.7, position=first, offboard=True, armed=True))
    output = mission.update(input_at(
        0.81, position=first, offboard=True, armed=True))
    assert mission.state == TrajectoryReplayState.PLAYBACK
    assert output.publish_offboard


def test_vertical_takeoff_then_moves_to_absolute_first_xy():
    mission = make_mission()

    advance_to_move_to_start(mission)

    assert mission.takeoff_target == pytest.approx(
        (5.0, 6.0, -0.2, 0.4))
    assert mission.target == pytest.approx((1.0, 2.0, -0.2, 0.4))


def test_playback_advances_exactly_one_row_per_update():
    mission = make_mission()
    advance_to_playback(mission)

    published_targets = [mission.target]
    output = mission.update(input_at(
        0.82, position=(1.0, 2.0, -0.2),
        offboard=True, armed=True))
    assert output.publish_offboard
    published_targets.append(mission.target)

    output = mission.update(input_at(
        0.83, position=(1.1, 2.2, -0.2),
        offboard=True, armed=True))
    assert output.publish_offboard
    published_targets.append(mission.target)

    expected_targets = [
        (1.0, 2.0, -0.2, 0.4),
        (1.1, 2.2, -0.2, 0.4),
        (1.3, 2.5, -0.2, 0.4),
    ]
    for actual, expected in zip(published_targets, expected_targets):
        assert actual == pytest.approx(expected)
    assert mission.playback_row == 3

    output = mission.update(input_at(
        0.84, position=(1.3, 2.5, -0.2),
        offboard=True, armed=True))
    assert mission.state == TrajectoryReplayState.REQUEST_LAND
    assert output.publish_offboard
    assert output.request_land


def test_completion_waits_for_landing_and_disarm():
    mission = make_mission((sample(0, 1.0, 2.0, -3.0),))
    advance_to_playback(mission)

    output = mission.update(input_at(
        0.82, position=(1.0, 2.0, -0.2),
        offboard=True, armed=True))
    assert output.request_land
    assert mission.state == TrajectoryReplayState.REQUEST_LAND

    output = mission.update(input_at(
        0.83, armed=True, landing=True))
    assert not output.publish_offboard
    assert mission.state == TrajectoryReplayState.WAIT_DISARM

    output = mission.update(input_at(0.9, armed=False))
    assert output.shutdown
    assert mission.state == TrajectoryReplayState.COMPLETE


@pytest.mark.parametrize(
    'overrides,reason',
    [
        ({'mocap_fresh': False}, 'motion capture timed out'),
        ({'local_valid': False}, 'local position became invalid'),
        ({'offboard': False}, 'left Offboard'),
        ({'armed': False}, 'vehicle disarmed'),
    ],
)
def test_flight_safety_failures_abort_without_publishing(
        overrides, reason):
    mission = make_mission()
    advance_to_playback(mission)
    values = {
        'position': (1.0, 2.0, -0.2),
        'offboard': True,
        'armed': True,
    }
    values.update(overrides)

    output = mission.update(input_at(1.0, **values))

    assert mission.state == TrajectoryReplayState.ABORTED
    assert reason in mission.abort_reason
    assert not output.publish_offboard


def test_prearm_instability_discards_captured_takeoff_target():
    mission = make_mission()
    mission.update(input_at(0.0))

    output = mission.update(input_at(0.01, position_stable=False))

    assert mission.state == TrajectoryReplayState.WAITING_FOR_DATA
    assert mission.takeoff_target is None
    assert mission.target is None
    assert mission.playback_row == 0
    assert not output.publish_offboard
