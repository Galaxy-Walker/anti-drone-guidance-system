from px4_mocap_hover.tracker_mission import (
    TrackerMission,
    TrackerMissionConfig,
    TrackerMissionInput,
    TrackerState,
)


def input_at(
        now,
        *,
        self_mocap_fresh=True,
        self_mocap_stable=True,
        position_stable=True,
        local_valid=True,
        start_allowed=True,
        position=(1.0, 2.0, 0.0),
        heading=0.25,
        offboard=False,
        armed=False,
        target_fresh=False,
        target_ready=False):
    return TrackerMissionInput(
        now=now,
        self_mocap_fresh=self_mocap_fresh,
        self_mocap_stable=self_mocap_stable,
        position_stable=position_stable,
        local_valid=local_valid,
        start_allowed=start_allowed,
        position=position,
        heading=heading,
        offboard=offboard,
        armed=armed,
        target_fresh=target_fresh,
        target_ready=target_ready,
    )


def airborne_mission():
    mission = TrackerMission(TrackerMissionConfig(
        takeoff_height=1.2,
        position_tolerance=0.1,
        stable_duration=1.0,
        prestream_duration=1.0,
        command_retry_interval=1.0,
    ))
    mission.update(input_at(0.0))
    mission.update(input_at(1.0))
    mission.update(input_at(1.1, offboard=True))
    mission.update(input_at(1.2, offboard=True, armed=True))
    return mission


def test_takeoff_does_not_require_target_data():
    mission = airborne_mission()

    mission.update(input_at(
        2.0,
        position=(1.0, 2.0, -1.2),
        offboard=True,
        armed=True,
    ))
    output = mission.update(input_at(
        3.0,
        position=(1.0, 2.0, -1.2),
        offboard=True,
        armed=True,
    ))

    assert mission.state == TrackerState.WAIT_TARGET
    assert output.publish_velocity


def test_target_ready_enters_tracking_and_loss_returns_to_waiting():
    mission = airborne_mission()
    mission.update(input_at(
        2.0,
        position=(1.0, 2.0, -1.2),
        offboard=True,
        armed=True,
    ))
    mission.update(input_at(
        3.0,
        position=(1.0, 2.0, -1.2),
        offboard=True,
        armed=True,
    ))

    output = mission.update(input_at(
        3.1,
        position=(1.0, 2.0, -1.2),
        offboard=True,
        armed=True,
        target_fresh=True,
        target_ready=True,
    ))
    assert mission.state == TrackerState.TRACKING
    assert output.publish_velocity

    output = mission.update(input_at(
        3.7,
        position=(1.0, 2.0, -1.2),
        offboard=True,
        armed=True,
        target_fresh=False,
        target_ready=False,
    ))
    assert mission.state == TrackerState.WAIT_TARGET
    assert output.publish_velocity


def test_target_not_ready_does_not_start_tracking():
    mission = airborne_mission()
    mission.update(input_at(
        2.0,
        position=(1.0, 2.0, -1.2),
        offboard=True,
        armed=True,
    ))
    mission.update(input_at(
        3.0,
        position=(1.0, 2.0, -1.2),
        offboard=True,
        armed=True,
    ))

    mission.update(input_at(
        3.1,
        position=(1.0, 2.0, -1.2),
        offboard=True,
        armed=True,
        target_fresh=True,
        target_ready=False,
    ))
    assert mission.state == TrackerState.WAIT_TARGET


def test_self_mocap_loss_aborts_and_stops_offboard_output():
    mission = airborne_mission()

    output = mission.update(input_at(
        2.0,
        self_mocap_fresh=False,
        offboard=True,
        armed=True,
    ))

    assert mission.state == TrackerState.ABORTED
    assert not output.publish_position
    assert not output.publish_velocity


def test_local_position_loss_aborts():
    mission = airborne_mission()

    mission.update(input_at(
        2.0,
        local_valid=False,
        offboard=True,
        armed=True,
    ))

    assert mission.state == TrackerState.ABORTED


def test_manual_offboard_exit_aborts_without_reacquiring():
    mission = airborne_mission()

    mission.update(input_at(2.0, offboard=False, armed=True))
    assert mission.state == TrackerState.ABORTED

    output = mission.update(input_at(
        3.0,
        offboard=True,
        armed=True,
        target_fresh=True,
        target_ready=True,
    ))
    assert mission.state == TrackerState.ABORTED
    assert not output.publish_velocity
