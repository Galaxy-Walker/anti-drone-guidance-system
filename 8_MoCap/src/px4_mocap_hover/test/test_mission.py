from px4_mocap_hover.mission import (
    HoverMission,
    MissionConfig,
    MissionInput,
    MissionState,
)


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


def test_mission_waits_for_valid_stable_data():
    mission = HoverMission(MissionConfig())

    mission.update(input_at(0.0, mocap_fresh=False))
    mission.update(input_at(1.0, mocap_stable=False))
    mission.update(input_at(2.0, position_stable=False))
    mission.update(input_at(3.0, local_valid=False))
    mission.update(input_at(4.0, start_allowed=False))

    assert mission.state == MissionState.WAITING_FOR_DATA
    assert mission.target is None


def test_full_mission_sequence():
    config = MissionConfig(
        takeoff_height=1.0,
        hover_duration=10.0,
        position_tolerance=0.1,
        stable_duration=1.0,
        prestream_duration=1.0,
        command_retry_interval=1.0,
    )
    mission = HoverMission(config)

    mission.update(input_at(0.0))
    assert mission.state == MissionState.PRESTREAM
    assert mission.target == (1.0, 2.0, -1.0, 0.25)

    assert mission.update(input_at(0.5)).publish_offboard
    mission.update(input_at(1.0))
    assert mission.state == MissionState.REQUEST_OFFBOARD
    assert mission.update(input_at(1.1)).request_offboard

    mission.update(input_at(1.2, offboard=True))
    assert mission.state == MissionState.REQUEST_ARM
    assert mission.update(input_at(1.3, offboard=True)).request_arm

    mission.update(input_at(1.4, offboard=True, armed=True))
    assert mission.state == MissionState.TAKEOFF
    mission.update(input_at(
        2.0, position=(1.0, 2.0, -1.0), offboard=True, armed=True))
    mission.update(input_at(
        3.0, position=(1.0, 2.0, -1.0), offboard=True, armed=True))
    assert mission.state == MissionState.HOVER

    output = mission.update(input_at(
        13.0, position=(1.0, 2.0, -1.0), offboard=True, armed=True))
    assert mission.state == MissionState.REQUEST_LAND
    assert output.request_land
    assert output.publish_offboard

    output = mission.update(input_at(13.1, armed=True))
    assert not output.request_land
    assert output.publish_offboard
    assert mission.state == MissionState.REQUEST_LAND

    output = mission.update(input_at(14.1, armed=True))
    assert output.request_land
    assert output.publish_offboard

    output = mission.update(input_at(14.2, armed=True, landing=True))
    assert not output.publish_offboard
    assert mission.state == MissionState.WAIT_DISARM

    output = mission.update(input_at(15.0, armed=False))
    assert output.shutdown
    assert mission.state == MissionState.COMPLETE


def test_prearm_instability_returns_to_waiting_and_restarts_target_capture():
    mission = HoverMission(MissionConfig(prestream_duration=1.0))
    mission.update(input_at(0.0))

    output = mission.update(input_at(0.1, position_stable=False))
    assert mission.state == MissionState.WAITING_FOR_DATA
    assert mission.target is None
    assert not output.publish_offboard

    mission.update(input_at(1.0, position=(3.0, 4.0, 0.0)))
    assert mission.state == MissionState.PRESTREAM
    assert mission.target == (3.0, 4.0, -1.0, 0.25)


def test_position_instability_prevents_arm_request():
    mission = HoverMission(MissionConfig(prestream_duration=0.0))
    mission.update(input_at(0.0))
    mission.update(input_at(0.1))
    mission.update(input_at(0.2, offboard=True))
    assert mission.state == MissionState.REQUEST_ARM

    output = mission.update(input_at(
        0.3, offboard=True, position_stable=False))
    assert mission.state == MissionState.WAITING_FOR_DATA
    assert not output.request_arm


def test_mocap_timeout_during_flight_aborts_and_never_restarts():
    mission = HoverMission(MissionConfig(prestream_duration=0.0))
    mission.update(input_at(0.0))
    mission.update(input_at(0.1))
    mission.update(input_at(0.2, offboard=True))
    mission.update(input_at(0.3, offboard=True, armed=True))

    output = mission.update(input_at(
        0.4, mocap_fresh=False, offboard=True, armed=True))
    assert mission.state == MissionState.ABORTED
    assert not output.publish_offboard

    mission.update(input_at(1.0, mocap_fresh=True))
    assert mission.state == MissionState.ABORTED


def test_leaving_offboard_during_flight_aborts():
    mission = HoverMission(MissionConfig(prestream_duration=0.0))
    mission.update(input_at(0.0))
    mission.update(input_at(0.1))
    mission.update(input_at(0.2, offboard=True))
    mission.update(input_at(0.3, offboard=True, armed=True))

    output = mission.update(input_at(0.4, offboard=False, armed=True))
    assert mission.state == MissionState.ABORTED
    assert not output.publish_offboard


def test_landing_mode_loss_resumes_land_requests():
    mission = HoverMission(MissionConfig(command_retry_interval=1.0))
    mission.state = MissionState.WAIT_DISARM

    output = mission.update(input_at(1.0, armed=True, landing=True))
    assert not output.request_land

    output = mission.update(input_at(1.1, armed=True, landing=False))
    assert mission.state == MissionState.REQUEST_LAND
    assert output.publish_offboard
    assert output.request_land
