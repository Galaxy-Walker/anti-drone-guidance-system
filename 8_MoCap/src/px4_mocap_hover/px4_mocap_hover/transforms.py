"""Coordinate transforms used by the motion-capture bridge."""

import math
from typing import Sequence, Tuple


Quaternion = Tuple[float, float, float, float]  # w, x, y, z
Vector3 = Tuple[float, float, float]
EulerAngles = Tuple[float, float, float]  # roll, pitch, yaw
MocapEulerXYZ = Tuple[float, float, float]  # x/roll, y/yaw, z/pitch

SQRT_HALF = math.sqrt(0.5)
ENU_TO_NED_Q: Quaternion = (0.0, SQRT_HALF, SQRT_HALF, 0.0)
FLU_TO_FRD_Q: Quaternion = (0.0, 1.0, 0.0, 0.0)


def enu_to_ned_position(position: Sequence[float]) -> Vector3:
    """Convert an ENU position vector to NED."""
    return (float(position[1]), float(position[0]), -float(position[2]))


def mocap_to_ned_position(position: Sequence[float]) -> Vector3:
    """Convert this motion-capture system's position axes to NED."""
    return (float(position[0]), float(position[2]), -float(position[1]))


def quaternion_multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    """Multiply two wxyz quaternions."""
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )


def normalize_quaternion(quaternion: Quaternion) -> Quaternion:
    """Return a normalized quaternion, rejecting invalid input."""
    if not all(math.isfinite(value) for value in quaternion):
        raise ValueError('quaternion contains a non-finite value')
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1e-6:
        raise ValueError('quaternion norm is zero')
    return tuple(value / norm for value in quaternion)  # type: ignore[return-value]


def quaternion_to_euler(quaternion_wxyz: Sequence[float]) -> EulerAngles:
    """Convert a Hamilton wxyz quaternion to roll, pitch, and yaw radians."""
    if len(quaternion_wxyz) != 4:
        raise ValueError('quaternion must have four components')
    w, x, y, z = normalize_quaternion(tuple(
        float(value) for value in quaternion_wxyz
    ))  # type: ignore[arg-type]

    roll = math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return (roll, pitch, yaw)


def mocap_xyzw_to_euler_xyz(
        quaternion_xyzw: Sequence[float]) -> MocapEulerXYZ:
    """Convert raw mocap xyzw to signed x/y/z (roll/yaw/pitch) radians."""
    if len(quaternion_xyzw) != 4:
        raise ValueError('quaternion must have four components')
    euler_x, euler_y, euler_z = quaternion_to_euler((
        float(quaternion_xyzw[3]),
        float(quaternion_xyzw[0]),
        float(quaternion_xyzw[1]),
        float(quaternion_xyzw[2]),
    ))
    return (-euler_x, -euler_y, -euler_z)


def enu_flu_to_ned_frd_quaternion(
        quaternion_xyzw: Sequence[float]) -> Quaternion:
    """Convert a ROS ENU/FLU xyzw orientation to PX4 NED/FRD wxyz."""
    if len(quaternion_xyzw) != 4:
        raise ValueError('quaternion must have four components')
    ros_wxyz = normalize_quaternion((
        float(quaternion_xyzw[3]),
        float(quaternion_xyzw[0]),
        float(quaternion_xyzw[1]),
        float(quaternion_xyzw[2]),
    ))
    converted = quaternion_multiply(
        quaternion_multiply(ENU_TO_NED_Q, ros_wxyz),
        FLU_TO_FRD_Q,
    )
    return normalize_quaternion(converted)


def enu_flu_quaternion_to_ned_frd_euler(
        quaternion_xyzw: Sequence[float]) -> EulerAngles:
    """Convert a ROS ENU/FLU xyzw quaternion to PX4 NED/FRD RPY radians."""
    return quaternion_to_euler(
        enu_flu_to_ned_frd_quaternion(quaternion_xyzw))


def valid_pose(position: Sequence[float], quaternion_xyzw: Sequence[float]) -> bool:
    """Check that pose values are finite and the quaternion can be normalized."""
    if len(position) != 3 or not all(math.isfinite(value) for value in position):
        return False
    try:
        normalize_quaternion((
            float(quaternion_xyzw[3]),
            float(quaternion_xyzw[0]),
            float(quaternion_xyzw[1]),
            float(quaternion_xyzw[2]),
        ))
    except (IndexError, TypeError, ValueError):
        return False
    return True
