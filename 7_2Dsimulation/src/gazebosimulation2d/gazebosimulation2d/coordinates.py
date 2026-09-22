"""ENU 仿真坐标系和 PX4 NED 坐标系之间的转换工具。"""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np


def enu_to_ned_vector(vector: Sequence[float] | np.ndarray) -> np.ndarray:
    value = np.asarray(vector, dtype=float)
    return np.array([value[1], value[0], -value[2]], dtype=float)


def ned_to_enu_vector(vector: Sequence[float] | np.ndarray) -> np.ndarray:
    value = np.asarray(vector, dtype=float)
    return np.array([value[1], value[0], -value[2]], dtype=float)


def enu_to_ned_list(vector: Sequence[float] | np.ndarray) -> list[float]:
    return enu_to_ned_vector(vector).tolist()


def wrap_angle(angle: float) -> float:
    return float((angle + math.pi) % (2.0 * math.pi) - math.pi)


def yaw_enu_to_ned(yaw_enu: float) -> float:
    return wrap_angle(math.pi * 0.5 - yaw_enu)


def yaw_to_target_enu(origin_enu: Sequence[float] | np.ndarray, target_enu: Sequence[float] | np.ndarray) -> float:
    direction = np.asarray(target_enu, dtype=float) - np.asarray(origin_enu, dtype=float)
    return float(math.atan2(direction[1], direction[0]))


def yaw_to_target_ned(origin_enu: Sequence[float] | np.ndarray, target_enu: Sequence[float] | np.ndarray) -> float:
    return yaw_enu_to_ned(yaw_to_target_enu(origin_enu, target_enu))


def yaw_from_quaternion_ned(quaternion_wxyz: Sequence[float] | np.ndarray) -> float:
    """从 PX4 VehicleOdometry 四元数中提取 ENU yaw。"""
    q = np.asarray(quaternion_wxyz, dtype=float)
    if q.shape[0] < 4 or not np.all(np.isfinite(q)):
        return 0.0

    length = float(np.linalg.norm(q[:4]))
    if length < 1e-9:
        return 0.0

    w, x, y, z = q[:4] / length
    forward_ned = np.array([
        1.0 - 2.0 * (y * y + z * z),
        2.0 * (x * y + w * z),
        2.0 * (x * z - w * y),
    ])
    forward_enu = ned_to_enu_vector(forward_ned)
    return float(math.atan2(forward_enu[1], forward_enu[0]))


# ENU 与 NED 只是原点相同、轴排列不同的两个世界系：NED (n, e, d) → ENU (e, n, -d)。
_ENU_FROM_NED = np.array([
    [0.0, 1.0, 0.0],
    [1.0, 0.0, 0.0],
    [0.0, 0.0, -1.0],
])
# 机体 FLU（前左上）与 PX4 FRD（前右下）只差 y、z 两轴符号。
_FRD_FROM_FLU = np.diag(np.array([1.0, -1.0, -1.0]))
# 相机光学系 C（x 右、y 下、z 前）到相机 link L（x 前、y 左、z 上）的固定旋转；
# 与 Gazebo 相机“+x 为光轴、+y 向左、+z 向上”的 link 约定一致。
_LINK_FROM_OPTICAL = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
])


def rotation_from_quaternion_ned(quaternion_wxyz: Sequence[float] | np.ndarray) -> np.ndarray | None:
    """四元数转 $R_{NED\\leftarrow FRD}$，非法输入返回 `None`。

    PX4 四元数表示 FRD 机体系到 NED 世界的旋转。与 `yaw_from_quaternion_ned`
    的宽松行为不同，这里对非有限、零范数和长度不足的输入直接拒绝，避免把
    非法姿态静默当成零姿态。
    """
    q = np.asarray(quaternion_wxyz, dtype=float)
    if q.shape[0] < 4 or not np.all(np.isfinite(q[:4])):
        return None

    length = float(np.linalg.norm(q[:4]))
    if length < 1e-9:
        return None

    w, x, y, z = q[:4] / length
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
        [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
        [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
    ])


def rotation_enu_from_quaternion_ned(quaternion_wxyz: Sequence[float] | np.ndarray) -> np.ndarray | None:
    """四元数转机体 FLU 到公共 ENU 的完整旋转 $R_{W\\leftarrow B}$。

    $$R_{W\\leftarrow B}=R_{ENU\\leftarrow NED}\\,R_{NED\\leftarrow FRD}(q)\\,
    \\operatorname{diag}(1,-1,-1).$$

    只做 NED→ENU 轴交换不足以得到 FLU 姿态，必须先按上式把 FRD 机体轴翻转。
    非法四元数返回 `None`。
    """
    rotation_ned = rotation_from_quaternion_ned(quaternion_wxyz)
    if rotation_ned is None:
        return None
    return _ENU_FROM_NED @ rotation_ned @ _FRD_FROM_FLU


def rotation_body_from_mount_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """安装外参转 $R_{B\\leftarrow L}=R_z(\\psi)R_y(\\theta)R_x(\\phi)$。

    这里的 rpy 只描述相机 link 相对机体 FLU 的固定安装旋转，不包含光学轴转换，
    也不包含机体姿态。
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def rotation_body_from_optical(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """完整安装链旋转 $R_{B\\leftarrow C}=R_{B\\leftarrow L}\\,R_{L\\leftarrow C}$。

    结果已经是 link→光学系的完整合成，不能再叠加一次光学轴转换。
    """
    return rotation_body_from_mount_rpy(roll, pitch, yaw) @ _LINK_FROM_OPTICAL


def camera_pose_from_odometry(
    position_ned: Sequence[float] | np.ndarray,
    quaternion_wxyz: Sequence[float] | np.ndarray,
    mount_xyz_flu: Sequence[float] | np.ndarray,
    mount_rpy_rad: Sequence[float] | np.ndarray,
) -> tuple[np.ndarray, np.ndarray] | None:
    """合成安装平移/旋转和机体姿态，得到公共 ENU 下的相机位姿。

    返回 `(p_cam,W, R_{W\\leftarrow C})`：

    $$p_{cam,W}=p_{body,W}+R_{W\\leftarrow B}\\,t_{B,L},\\qquad
    R_{W\\leftarrow C}=R_{W\\leftarrow B}\\,R_{B\\leftarrow C}.$$

    四元数非法、位置/安装参数非有限或长度错误时返回 `None`。
    """
    body_rotation = rotation_enu_from_quaternion_ned(quaternion_wxyz)
    if body_rotation is None:
        return None

    position = np.asarray(position_ned, dtype=float)
    if position.shape != (3,) or not np.all(np.isfinite(position)):
        return None

    offset = np.asarray(mount_xyz_flu, dtype=float)
    rpy = np.asarray(mount_rpy_rad, dtype=float)
    if offset.shape != (3,) or rpy.shape != (3,):
        return None
    if not np.all(np.isfinite(offset)) or not np.all(np.isfinite(rpy)):
        return None

    body_position = ned_to_enu_vector(position)
    camera_position = body_position + body_rotation @ offset
    camera_rotation = body_rotation @ rotation_body_from_optical(rpy[0], rpy[1], rpy[2])
    return camera_position, camera_rotation
