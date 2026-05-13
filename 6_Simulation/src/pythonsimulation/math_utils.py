from __future__ import annotations

import numpy as np

EPS = 1e-9


def norm(vector: np.ndarray) -> float:
    return float(np.linalg.norm(vector))


def normalize(vector: np.ndarray) -> np.ndarray:
    length = norm(vector)
    if length < EPS:
        # 零向量没有方向；返回零向量可以避免除以 0，同时让上层逻辑自然退化。
        return np.zeros_like(vector, dtype=float)
    return np.asarray(vector, dtype=float) / length


def clamp(value: float, low: float, high: float) -> float:
    return float(np.clip(value, low, high))


def clamp_norm(vector: np.ndarray, max_norm: float) -> np.ndarray:
    length = norm(vector)
    if length <= max_norm or length < EPS:
        return np.asarray(vector, dtype=float).copy()
    # 保持方向不变，只把向量长度缩放到最大允许值；用于速度/加速度限幅。
    return np.asarray(vector, dtype=float) * (max_norm / length)


def angle_between(a: np.ndarray, b: np.ndarray) -> float:
    a_unit = normalize(a)
    b_unit = normalize(b)
    if norm(a_unit) < EPS or norm(b_unit) < EPS:
        return 0.0
    # 浮点误差可能让点积略超出 [-1, 1]，clip 后 arccos 才稳定。
    return float(np.arccos(np.clip(np.dot(a_unit, b_unit), -1.0, 1.0)))


def wrap_angle(angle: float) -> float:
    # 把任意角度归一化到 [-pi, pi)，便于计算“最短转向角”。
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


def forward_from_yaw_pitch(yaw: float, pitch: float) -> np.ndarray:
    # 机体/相机前向向量：yaw 控制水平朝向，pitch 控制上仰/下俯。
    return np.array([
        np.cos(pitch) * np.cos(yaw),
        np.cos(pitch) * np.sin(yaw),
        np.sin(pitch),
    ])


def yaw_pitch_to_target(origin: np.ndarray, target: np.ndarray) -> tuple[float, float]:
    direction = np.asarray(target, dtype=float) - np.asarray(origin, dtype=float)
    horizontal = float(np.hypot(direction[0], direction[1]))
    yaw = float(np.arctan2(direction[1], direction[0]))
    pitch = float(np.arctan2(direction[2], horizontal))
    return yaw, pitch


def update_angles_toward(
    yaw: float,
    pitch: float,
    origin: np.ndarray,
    target: np.ndarray,
    yaw_rate_max: float,
    pitch_rate_max: float,
    dt: float,
) -> tuple[float, float]:
    target_yaw, target_pitch = yaw_pitch_to_target(origin, target)
    # yaw/pitch 不能瞬间指向目标，只能按最大角速度逐步转过去。
    yaw_delta = clamp(wrap_angle(target_yaw - yaw), -yaw_rate_max * dt, yaw_rate_max * dt)
    pitch_delta = clamp(target_pitch - pitch, -pitch_rate_max * dt, pitch_rate_max * dt)
    next_yaw = wrap_angle(yaw + yaw_delta)
    next_pitch = clamp(pitch + pitch_delta, -np.pi * 0.5, np.pi * 0.5)
    return next_yaw, next_pitch


def fov_visibility(
    observer_position: np.ndarray,
    yaw: float,
    pitch: float,
    target_position: np.ndarray,
    fov_half_angle: float,
) -> tuple[bool, float]:
    target_direction = np.asarray(target_position, dtype=float) - np.asarray(observer_position, dtype=float)
    # LOS 夹角是“相机前向”和“目标方向”的夹角，小于半视场角才算看得见。
    los_angle = angle_between(forward_from_yaw_pitch(yaw, pitch), target_direction)
    return los_angle <= fov_half_angle, los_angle
