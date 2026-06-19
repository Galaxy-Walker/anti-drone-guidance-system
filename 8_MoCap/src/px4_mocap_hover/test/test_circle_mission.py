import math

import pytest

from px4_mocap_hover.circle_mission import (
    CircleMission,
    CircleMissionConfig,
    CircleMissionState,
)
from px4_mocap_hover.mission import MissionInput


def input_at(
        now,
        *,
        mocap_fresh=True,
        mocap_stable=True,
        position_stable=True,
        local_valid=True,
        start_allowed=True,
        position=(1.0, 2.0, 0.0),
        heading=0.25,
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


def advance_to_hover(mission):
    mission.update(input_at(0.0))
    mission.update(input_at(1.0))
    mission.update(input_at(1.1, offboard=True))
    mission.update(input_at(1.2, offboard=True, armed=True))
    mission.update(input_at(
        2.0, position=(1.0, 2.0, -1.0), offboard=True, armed=True))
    mission.update(input_at(
        3.0, position=(1.0, 2.0, -1.0), offboard=True, armed=True))
    assert mission.state == CircleMissionState.HOVER


def test_circle_starts_at_takeoff_point_and_moves_counterclockwise():
    mission = CircleMission(CircleMissionConfig(
        hover_duration=3.0,
        circle_radius=0.5,
        circle_duration=10.0,
    ))
    advance_to_hover(mission)

    mission.update(input_at(
        6.0, position=(1.0, 2.0, -1.0), offboard=True, armed=True))
    assert mission.state == CircleMissionState.CIRCLE
    assert mission.target == pytest.approx((1.0, 2.0, -1.0, 0.25))

    mission.update(input_at(8.5, offboard=True, armed=True))
    assert mission.target == pytest.approx((1.5, 1.5, -1.0, 0.25))

    mission.update(input_at(11.0, offboard=True, armed=True))
    assert mission.target == pytest.approx((1.0, 1.0, -1.0, 0.25))

    mission.update(input_at(13.5, offboard=True, armed=True))
    assert mission.target == pytest.approx((0.5, 1.5, -1.0, 0.25))


def test_circle_completion_requests_land_and_waits_for_disarm():
    mission = CircleMission(CircleMissionConfig(
        hover_duration=3.0,
        circle_duration=10.0,
    ))
    advance_to_hover(mission)
    mission.update(input_at(6.0, offboard=True, armed=True))

    output = mission.update(input_at(16.0, offboard=True, armed=True))
    assert mission.state == CircleMissionState.REQUEST_LAND
    assert mission.target == pytest.approx((1.0, 2.0, -1.0, 0.25))
    assert output.publish_offboard
    assert output.request_land

    output = mission.update(input_at(16.1, armed=True, landing=True))
    assert mission.state == CircleMissionState.WAIT_DISARM
    assert not output.publish_offboard

    output = mission.update(input_at(17.0, armed=False))
    assert mission.state == CircleMissionState.COMPLETE
    assert output.shutdown


def test_circle_has_expected_radius_and_fixed_altitude_and_yaw():
    mission = CircleMission(CircleMissionConfig(
        circle_radius=0.5,
        circle_duration=10.0,
    ))
    mission.takeoff_target = (3.0, 4.0, -2.0, -0.4)

    for elapsed in (0.0, 1.0, 3.7, 7.2, 9.9):
        mission._set_circle_target(elapsed)
        north, east, down, yaw = mission.target
        distance = math.hypot(north - 3.0, east - 3.5)
        assert distance == pytest.approx(0.5)
        assert down == -2.0
        assert yaw == -0.4


def test_prearm_instability_resets_captured_target():
    mission = CircleMission(CircleMissionConfig())
    mission.update(input_at(0.0))

    output = mission.update(input_at(0.1, position_stable=False))
    assert mission.state == CircleMissionState.WAITING_FOR_DATA
    assert mission.takeoff_target is None
    assert mission.target is None
    assert not output.publish_offboard


def test_mocap_timeout_and_offboard_loss_abort_circle_mission():
    mission = CircleMission(CircleMissionConfig())
    mission.state = CircleMissionState.CIRCLE
    mission.takeoff_target = (1.0, 2.0, -1.0, 0.0)
    mission.target = mission.takeoff_target

    output = mission.update(input_at(
        1.0, mocap_fresh=False, offboard=True, armed=True))
    assert mission.state == CircleMissionState.ABORTED
    assert not output.publish_offboard

    mission = CircleMission(CircleMissionConfig())
    mission.state = CircleMissionState.CIRCLE
    mission.takeoff_target = (1.0, 2.0, -1.0, 0.0)
    mission.target = mission.takeoff_target

    output = mission.update(input_at(
        1.0, offboard=False, armed=True))
    assert mission.state == CircleMissionState.ABORTED
    assert not output.publish_offboard
