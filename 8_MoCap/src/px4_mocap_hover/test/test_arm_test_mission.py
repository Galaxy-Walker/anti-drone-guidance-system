from px4_mocap_hover.arm_test_mission import (
    ArmTestConfig,
    ArmTestInput,
    ArmTestMission,
    ArmTestState,
)


def input_at(
        now, *, data_ready=True, prearm_safety_ok=True,
        armed_safety_ok=True, armed=False):
    return ArmTestInput(
        now=now,
        data_ready=data_ready,
        prearm_safety_ok=prearm_safety_ok,
        armed_safety_ok=armed_safety_ok,
        armed=armed,
    )


def test_arm_then_automatic_disarm_sequence():
    test = ArmTestMission(ArmTestConfig(
        armed_duration=3.0,
        arm_request_timeout=10.0,
        command_retry_interval=1.0,
    ))

    test.update(input_at(0.0))
    assert test.state == ArmTestState.REQUEST_ARM
    assert test.update(input_at(0.1)).request_arm

    test.update(input_at(0.2, armed=True))
    assert test.state == ArmTestState.ARMED
    assert not test.update(input_at(3.1, armed=True)).request_disarm

    output = test.update(input_at(3.2, armed=True))
    assert output.request_disarm
    assert test.state == ArmTestState.REQUEST_DISARM

    assert not test.update(input_at(3.3, armed=False)).shutdown
    output = test.update(input_at(4.3, armed=False))
    assert output.shutdown
    assert test.state == ArmTestState.COMPLETE


def test_invalid_safety_condition_while_armed_requests_disarm():
    test = ArmTestMission(ArmTestConfig())
    test.update(input_at(0.0))
    test.update(input_at(0.1, armed=True))

    output = test.update(input_at(0.2, armed_safety_ok=False, armed=True))

    assert output.request_disarm
    assert test.state == ArmTestState.REQUEST_DISARM


def test_unexpected_disarmed_report_still_requests_disarm_confirmation():
    test = ArmTestMission(ArmTestConfig())
    test.update(input_at(0.0))
    test.update(input_at(0.1, armed=True))

    output = test.update(input_at(0.2, armed=False))

    assert output.request_disarm
    assert test.state == ArmTestState.REQUEST_DISARM


def test_invalid_prearm_condition_requests_disarm_before_exit():
    test = ArmTestMission(ArmTestConfig())
    test.update(input_at(0.0))

    output = test.update(input_at(0.1, prearm_safety_ok=False))

    assert output.request_disarm
    assert not output.request_arm
    assert test.state == ArmTestState.REQUEST_DISARM


def test_arm_request_timeout_requests_disarm_before_exit():
    test = ArmTestMission(ArmTestConfig(arm_request_timeout=2.0))
    test.update(input_at(0.0))

    output = test.update(input_at(2.0))

    assert output.request_disarm
    assert test.state == ArmTestState.REQUEST_DISARM
