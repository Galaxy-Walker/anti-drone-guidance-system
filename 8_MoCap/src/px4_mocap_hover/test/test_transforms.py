import math

import pytest

from px4_mocap_hover.transforms import (
    enu_flu_quaternion_to_ned_frd_euler,
    enu_flu_to_ned_frd_quaternion,
    enu_to_ned_position,
    mocap_to_ned_position,
    mocap_xyzw_to_euler_xyz,
    quaternion_to_euler,
    valid_pose,
)


def test_enu_position_is_converted_to_ned():
    assert enu_to_ned_position((1.0, 2.0, 3.0)) == (2.0, 1.0, -3.0)


def test_custom_mocap_position_is_converted_to_ned():
    assert mocap_to_ned_position((1.0, 2.0, 3.0)) == (1.0, 3.0, -2.0)


def test_aligned_enu_flu_pose_converts_to_expected_ned_frd_quaternion():
    quaternion = enu_flu_to_ned_frd_quaternion((0.0, 0.0, 0.0, 1.0))

    assert quaternion == pytest.approx(
        (-math.sqrt(0.5), 0.0, 0.0, -math.sqrt(0.5)))


@pytest.mark.parametrize(
    ('position', 'quaternion'),
    [
        ((math.nan, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0)),
        ((0.0, 0.0, 0.0), (math.inf, 0.0, 0.0, 1.0)),
    ],
)
def test_invalid_pose_is_rejected(position, quaternion):
    assert not valid_pose(position, quaternion)


def test_non_unit_quaternion_is_normalized():
    quaternion = enu_flu_to_ned_frd_quaternion((0.0, 0.0, 0.0, 2.0))

    assert math.sqrt(sum(value * value for value in quaternion)) == pytest.approx(1.0)


def test_quaternion_is_converted_to_roll_pitch_yaw():
    half_yaw = math.radians(45.0)

    roll, pitch, yaw = quaternion_to_euler(
        (math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw)))

    assert roll == pytest.approx(0.0)
    assert pitch == pytest.approx(0.0)
    assert yaw == pytest.approx(math.radians(90.0))


def test_mocap_xyzw_is_converted_to_expected_roll_yaw_pitch_xyz():
    euler_xyz = mocap_xyzw_to_euler_xyz((0.021, 0.686, -0.017, 0.727))

    assert tuple(math.degrees(angle) for angle in euler_xyz) == pytest.approx(
        (-7.11269, -86.66135, -4.03171),
        abs=1e-5,
    )


def test_aligned_enu_flu_attitude_is_aircraft_facing_east_in_ned():
    roll, pitch, yaw = enu_flu_quaternion_to_ned_frd_euler(
        (0.0, 0.0, 0.0, 1.0))

    assert roll == pytest.approx(0.0)
    assert pitch == pytest.approx(0.0)
    assert yaw == pytest.approx(math.radians(90.0))
