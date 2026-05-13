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


def ned_to_enu_list(vector: Sequence[float] | np.ndarray) -> list[float]:
    return ned_to_enu_vector(vector).tolist()


def wrap_angle(angle: float) -> float:
    return float((angle + math.pi) % (2.0 * math.pi) - math.pi)


def yaw_enu_to_ned(yaw_enu: float) -> float:
    # ENU yaw is measured from East toward North; PX4 NED yaw is measured from North toward East.
    return wrap_angle(math.pi * 0.5 - yaw_enu)


def yaw_ned_to_enu(yaw_ned: float) -> float:
    return wrap_angle(math.pi * 0.5 - yaw_ned)


def yaw_to_target_enu(origin_enu: Sequence[float] | np.ndarray, target_enu: Sequence[float] | np.ndarray) -> float:
    direction = np.asarray(target_enu, dtype=float) - np.asarray(origin_enu, dtype=float)
    return float(math.atan2(direction[1], direction[0]))


def yaw_to_target_ned(origin_enu: Sequence[float] | np.ndarray, target_enu: Sequence[float] | np.ndarray) -> float:
    return yaw_enu_to_ned(yaw_to_target_enu(origin_enu, target_enu))


def yaw_pitch_from_quaternion_ned(quaternion_wxyz: Sequence[float] | np.ndarray) -> tuple[float, float]:
    q = np.asarray(quaternion_wxyz, dtype=float)
    if q.shape[0] < 4 or not np.all(np.isfinite(q)):
        return 0.0, 0.0

    norm = float(np.linalg.norm(q[:4]))
    if norm < 1e-9:
        return 0.0, 0.0

    w, x, y, z = q[:4] / norm
    forward_ned = np.array([
        1.0 - 2.0 * (y * y + z * z),
        2.0 * (x * y + w * z),
        2.0 * (x * z - w * y),
    ])
    forward_enu = ned_to_enu_vector(forward_ned)
    horizontal = float(math.hypot(forward_enu[0], forward_enu[1]))
    yaw = float(math.atan2(forward_enu[1], forward_enu[0]))
    pitch = float(math.atan2(forward_enu[2], horizontal))
    return yaw, pitch
