from __future__ import annotations

import numpy as np

EPS = 1e-9


def norm(vector: np.ndarray) -> float:
    return float(np.linalg.norm(vector))


def norm_xy(vector: np.ndarray) -> float:
    return float(np.linalg.norm(np.asarray(vector, dtype=float)[:2]))


def normalize_xy(vector: np.ndarray) -> np.ndarray:
    result = np.zeros(3, dtype=float)
    xy = np.asarray(vector, dtype=float)[:2]
    length = float(np.linalg.norm(xy))
    if length >= EPS:
        result[:2] = xy / length
    return result


def clamp_norm_xy(vector: np.ndarray, max_norm: float) -> np.ndarray:
    result = np.asarray(vector, dtype=float).copy()
    result[2] = 0.0
    length = norm_xy(result)
    if length > max_norm and length >= EPS:
        result[:2] *= max_norm / length
    return result


def wrap_angle(angle: float) -> float:
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


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


def lock_pursuer_altitude(position: np.ndarray, fixed_altitude: float) -> np.ndarray:
    result = np.asarray(position, dtype=float).copy()
    result[2] = fixed_altitude
    return result


def lock_target_altitude(position: np.ndarray, fixed_altitude: float) -> np.ndarray:
    result = np.asarray(position, dtype=float).copy()
    result[2] = fixed_altitude
    return result
