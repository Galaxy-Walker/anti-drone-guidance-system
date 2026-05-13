"""仿真坐标系和 PX4 坐标系之间的转换工具。

轻量级算法代码使用 ENU 坐标系，便于绘图和常见机器人学计算：x=东，
y=北，z=上。PX4 里程计和轨迹设定点使用 NED 坐标系：x=北，y=东，
z=下。把所有坐标转换集中放在这里，可以避免导引代码混用 ENU/NED 假设。
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np


def enu_to_ned_vector(vector: Sequence[float] | np.ndarray) -> np.ndarray:
    """将 `[东, 北, 上]` 转为 PX4 使用的 `[北, 东, 下]`。"""
    value = np.asarray(vector, dtype=float)
    # 位置、速度和加速度都适用同一个轴交换和 z 轴取反规则。
    return np.array([value[1], value[0], -value[2]], dtype=float)


def ned_to_enu_vector(vector: Sequence[float] | np.ndarray) -> np.ndarray:
    """将 PX4 的 `[北, 东, 下]` 转为 `[东, 北, 上]`。"""
    value = np.asarray(vector, dtype=float)
    # 对三维向量来说，ENU<->NED 的转换形式互为自身逆变换。
    return np.array([value[1], value[0], -value[2]], dtype=float)


def enu_to_ned_list(vector: Sequence[float] | np.ndarray) -> list[float]:
    """把 ENU 向量转换成可直接填入 PX4 消息字段的列表。"""
    return enu_to_ned_vector(vector).tolist()


def ned_to_enu_list(vector: Sequence[float] | np.ndarray) -> list[float]:
    """把 NED 向量转换成仿真端 ENU 顺序的列表。"""
    return ned_to_enu_vector(vector).tolist()


def wrap_angle(angle: float) -> float:
    """把角度归一化到 `[-pi, pi)`，避免 yaw 出现跳变。"""
    return float((angle + math.pi) % (2.0 * math.pi) - math.pi)


def yaw_enu_to_ned(yaw_enu: float) -> float:
    """将 ENU 约定下的 yaw 转为 PX4 NED 约定下的 yaw。"""
    # ENU yaw 从东向北计角，PX4 NED yaw 从北向东计角；90 度偏移用于交换零方向。
    return wrap_angle(math.pi * 0.5 - yaw_enu)


def yaw_ned_to_enu(yaw_ned: float) -> float:
    """将 PX4 NED 约定下的 yaw 转回 ENU 约定。"""
    return wrap_angle(math.pi * 0.5 - yaw_ned)


def yaw_to_target_enu(origin_enu: Sequence[float] | np.ndarray, target_enu: Sequence[float] | np.ndarray) -> float:
    """计算从 `origin_enu` 指向 `target_enu` 的 ENU yaw。"""
    direction = np.asarray(target_enu, dtype=float) - np.asarray(origin_enu, dtype=float)
    # atan2(y, x) 与 ENU yaw 约定一致：+x 方向为 0，朝 +y 方向为正。
    return float(math.atan2(direction[1], direction[0]))


def yaw_to_target_ned(origin_enu: Sequence[float] | np.ndarray, target_enu: Sequence[float] | np.ndarray) -> float:
    """根据 ENU 位置计算 PX4 NED yaw 设定值。"""
    return yaw_enu_to_ned(yaw_to_target_enu(origin_enu, target_enu))


def yaw_pitch_from_quaternion_ned(quaternion_wxyz: Sequence[float] | np.ndarray) -> tuple[float, float]:
    """从 PX4 VehicleOdometry 四元数中提取 ENU yaw/pitch。

    PX4 在 NED 坐标系下以 `[w, x, y, z]` 上报姿态。FOV 算法使用 ENU 下的
    机体前向方向，所以这里先把机体系 +x 轴转换到 ENU，再计算 yaw 和 pitch。
    非法四元数会安全退化为水平朝向 ENU +x。
    """
    q = np.asarray(quaternion_wxyz, dtype=float)
    if q.shape[0] < 4 or not np.all(np.isfinite(q)):
        return 0.0, 0.0

    norm = float(np.linalg.norm(q[:4]))
    if norm < 1e-9:
        return 0.0, 0.0

    w, x, y, z = q[:4] / norm
    # 四元数旋转矩阵的第一列表示机体系 +x 轴在 NED 中的方向；对多旋翼而言，
    # 这里把它作为 FOV 检查使用的前向方向。
    forward_ned = np.array([
        1.0 - 2.0 * (y * y + z * z),
        2.0 * (x * y + w * z),
        2.0 * (x * z - w * y),
    ])
    forward_enu = ned_to_enu_vector(forward_ned)
    horizontal = float(math.hypot(forward_enu[0], forward_enu[1]))
    yaw = float(math.atan2(forward_enu[1], forward_enu[0]))
    # 当前向向量在 ENU 中向上时，pitch 为正。
    pitch = float(math.atan2(forward_enu[2], horizontal))
    return yaw, pitch
