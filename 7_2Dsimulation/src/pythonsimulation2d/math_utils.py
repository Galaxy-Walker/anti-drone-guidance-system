from __future__ import annotations

import numpy as np

EPS = 1e-9


def norm(vector: np.ndarray) -> float:
    return float(np.linalg.norm(vector))


def norm_xy(vector: np.ndarray) -> float:
    return float(np.linalg.norm(np.asarray(vector, dtype=float)[:2]))


def normalize(vector: np.ndarray) -> np.ndarray:
    length = norm(vector)
    if length < EPS:
        return np.zeros_like(vector, dtype=float)
    return np.asarray(vector, dtype=float) / length


def normalize_xy(vector: np.ndarray) -> np.ndarray:
    result = np.zeros(3, dtype=float)
    xy = np.asarray(vector, dtype=float)[:2]
    length = float(np.linalg.norm(xy))
    if length >= EPS:
        result[:2] = xy / length
    return result


def clamp_norm(vector: np.ndarray, max_norm: float) -> np.ndarray:
    length = norm(vector)
    if length <= max_norm or length < EPS:
        return np.asarray(vector, dtype=float).copy()
    return np.asarray(vector, dtype=float) * (max_norm / length)


def clamp_norm_xy(vector: np.ndarray, max_norm: float) -> np.ndarray:
    result = np.asarray(vector, dtype=float).copy()
    result[2] = 0.0
    length = norm_xy(result)
    if length > max_norm and length >= EPS:
        result[:2] *= max_norm / length
    return result


def angle_between_xy(a: np.ndarray, b: np.ndarray) -> float:
    a_unit = normalize_xy(a)
    b_unit = normalize_xy(b)
    if norm_xy(a_unit) < EPS or norm_xy(b_unit) < EPS:
        return 0.0
    return float(np.arccos(np.clip(np.dot(a_unit[:2], b_unit[:2]), -1.0, 1.0)))


def wrap_angle(angle: float) -> float:
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


def forward_from_yaw(yaw: float) -> np.ndarray:
    # 水平面内机体/相机前向向量；z 固定为 0。
    return np.array([np.cos(yaw), np.sin(yaw), 0.0])


def yaw_to_target(origin: np.ndarray, target: np.ndarray) -> float:
    direction = np.asarray(target, dtype=float)[:2] - np.asarray(origin, dtype=float)[:2]
    return float(np.arctan2(direction[1], direction[0]))


def update_yaw_toward(
    yaw: float,
    origin: np.ndarray,
    target: np.ndarray,
    yaw_rate_max: float,
    dt: float,
) -> float:
    target_yaw = yaw_to_target(origin, target)
    yaw_delta = np.clip(wrap_angle(target_yaw - yaw), -yaw_rate_max * dt, yaw_rate_max * dt)
    return wrap_angle(yaw + float(yaw_delta))


def fov_visibility(
    observer_position: np.ndarray,
    yaw: float,
    target_position: np.ndarray,
    fov_half_angle: float,
) -> tuple[bool, float]:
    target_direction = np.asarray(target_position, dtype=float) - np.asarray(observer_position, dtype=float)
    los_angle = angle_between_xy(forward_from_yaw(yaw), target_direction)
    return los_angle <= fov_half_angle, los_angle


def lock_pursuer_altitude(position: np.ndarray, fixed_altitude: float) -> np.ndarray:
    result = np.asarray(position, dtype=float).copy()
    result[2] = fixed_altitude
    return result


def lock_ground_target(position: np.ndarray) -> np.ndarray:
    result = np.asarray(position, dtype=float).copy()
    result[2] = 0.0
    return result
