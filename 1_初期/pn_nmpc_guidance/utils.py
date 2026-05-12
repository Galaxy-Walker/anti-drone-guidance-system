import numpy as np


EPS = 1e-9


def format_vec(vec):
    """把向量压成短字符串，避免日志里一长串 numpy 默认格式难读。"""
    return np.array2string(np.asarray(vec, dtype=float), precision=3, suppress_small=True)


def safe_norm(vec):
    """返回向量长度；单独封装是为了让后面的零向量保护更清楚。"""
    return float(np.linalg.norm(vec))


def unit_or_default(vec, default):
    """把向量归一化；如果输入太小，就退回到给定默认方向。"""
    norm = safe_norm(vec)
    if norm > EPS:
        return np.asarray(vec, dtype=float) / norm
    default_norm = safe_norm(default)
    if default_norm > EPS:
        return np.asarray(default, dtype=float) / default_norm
    return np.array([1.0, 0.0, 0.0], dtype=float)


def clip_vector_norm(vec, max_norm):
    """限制向量模长，用于模拟无人机最大加速度约束。"""
    vec = np.asarray(vec, dtype=float)
    norm = safe_norm(vec)
    if norm <= max_norm or norm < EPS:
        return vec
    return vec / norm * max_norm


def clip_speed(velocity, speed_min, speed_max):
    """限制速度大小，但保留速度方向。"""
    velocity = np.asarray(velocity, dtype=float)
    speed = safe_norm(velocity)
    if speed < EPS:
        return velocity
    clipped_speed = float(np.clip(speed, speed_min, speed_max))
    return velocity / speed * clipped_speed


def build_body_axes(forward):
    """根据前向向量构造右向和上向，用于把目标方向分解成水平/垂直角误差。"""
    forward = unit_or_default(forward, np.array([1.0, 0.0, 0.0]))
    world_up = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(forward, world_up))) > 0.95:
        world_up = np.array([0.0, 1.0, 0.0])
    right = unit_or_default(np.cross(forward, world_up), np.array([0.0, 1.0, 0.0]))
    up = unit_or_default(np.cross(right, forward), world_up)
    return right, up


def compute_fov_errors(relative_vector, forward_vector, horizontal_fov_deg, vertical_fov_deg):
    """
    计算目标相对当前前向的水平/垂直角误差。

    返回值单位为度；超过半视场角时，violation=True。
    """
    relative_dir = unit_or_default(relative_vector, forward_vector)
    forward = unit_or_default(forward_vector, relative_dir)
    right, up = build_body_axes(forward)

    forward_component = float(np.dot(relative_dir, forward))
    right_component = float(np.dot(relative_dir, right))
    up_component = float(np.dot(relative_dir, up))

    horizontal_error = np.degrees(np.arctan2(right_component, forward_component))
    vertical_error = np.degrees(
        np.arctan2(up_component, max(np.hypot(forward_component, right_component), EPS))
    )
    violation = (
        abs(horizontal_error) > horizontal_fov_deg * 0.5
        or abs(vertical_error) > vertical_fov_deg * 0.5
    )
    return float(horizontal_error), float(vertical_error), bool(violation)
