import math

import pytest

from px4_mocap_hover.tracker_control import (
    TrackerControlConfig,
    TrackerVelocityController,
)


def controller():
    return TrackerVelocityController(TrackerControlConfig(
        xy_kp=0.8,
        xy_deadband=0.1,
        max_xy_speed=0.5,
        max_xy_acceleration=0.5,
        z_kp=1.0,
        max_z_speed=0.3,
    ))


def test_xy_error_generates_ned_velocity_with_speed_limit():
    control = controller()
    control.reset(0.0)

    velocity = control.update(
        now=2.0,
        self_xy=(1.0, 1.0),
        target_xy=(2.0, 2.0),
        current_z=-1.2,
        target_z=-1.2,
    )

    assert math.hypot(*velocity[:2]) == pytest.approx(0.5)
    assert velocity[0] == pytest.approx(velocity[1])
    assert velocity[2] == pytest.approx(0.0)


def test_xy_deadband_commands_zero_horizontal_velocity():
    control = controller()
    control.reset(0.0)

    velocity = control.update(
        now=1.0,
        self_xy=(1.0, 1.0),
        target_xy=(1.05, 1.05),
        current_z=-1.2,
        target_z=-1.2,
    )

    assert velocity[:2] == pytest.approx((0.0, 0.0))


def test_horizontal_acceleration_is_limited():
    control = controller()
    control.reset(0.0)

    velocity = control.update(
        now=0.2,
        self_xy=(0.0, 0.0),
        target_xy=(10.0, 0.0),
        current_z=-1.2,
        target_z=-1.2,
    )

    assert velocity[:2] == pytest.approx((0.1, 0.0))


def test_target_loss_slew_limits_deceleration_to_zero():
    control = controller()
    control.reset(0.0)
    control.update(
        now=1.0,
        self_xy=(0.0, 0.0),
        target_xy=(10.0, 0.0),
        current_z=-1.2,
        target_z=-1.2,
    )

    velocity = control.update(
        now=1.5,
        self_xy=(0.0, 0.0),
        target_xy=None,
        current_z=-1.2,
        target_z=-1.2,
    )
    assert velocity[:2] == pytest.approx((0.25, 0.0))

    velocity = control.update(
        now=2.0,
        self_xy=(0.0, 0.0),
        target_xy=None,
        current_z=-1.2,
        target_z=-1.2,
    )
    assert velocity[:2] == pytest.approx((0.0, 0.0))


def test_vertical_velocity_holds_altitude_with_limit():
    control = controller()
    control.reset(0.0)

    up = control.update(
        now=1.0,
        self_xy=(0.0, 0.0),
        target_xy=None,
        current_z=-0.5,
        target_z=-1.2,
    )
    down = control.update(
        now=2.0,
        self_xy=(0.0, 0.0),
        target_xy=None,
        current_z=-2.0,
        target_z=-1.2,
    )

    assert up[2] == pytest.approx(-0.3)
    assert down[2] == pytest.approx(0.3)
