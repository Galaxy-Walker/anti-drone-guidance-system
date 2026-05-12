from dataclasses import dataclass

import numpy as np

from .nmpc import PNTrend
from .utils import EPS, safe_norm, unit_or_default


@dataclass
class RelativeGeometry:
    """PN/NMPC 共用的相对运动量。

    R_vec 和 V_rel 都按“目标 - 追踪者”定义；Vc 为正表示距离正在缩短；Omega 是视线
    方向的角速度，越大说明目标在视野中移动得越快、越需要转向。
    """

    R_vec: np.ndarray
    V_rel: np.ndarray
    R_mag: float
    R_sq: float
    Omega: np.ndarray
    Omega_mag: float
    Vc: float
    target_speed: float


def compute_relative_geometry(target_pos, tracker_pos, target_vel, tracker_vel):
    """计算 PN/NMPC 共用的相对几何量。"""
    R_vec = target_pos - tracker_pos
    V_rel = target_vel - tracker_vel
    R_mag = safe_norm(R_vec)
    R_sq = float(np.dot(R_vec, R_vec))
    target_speed = safe_norm(target_vel)

    if R_mag <= EPS or R_sq <= EPS:
        Omega = np.zeros(3)
        Vc = 0.0
    else:
        # 视线角速率 Omega = R x V / |R|^2，是 PN 判断“目标方向变化多快”的核心量。
        Omega = np.cross(R_vec, V_rel) / R_sq
        # 接近速度 Vc 取负号，是为了让“正在靠近”得到正值，后续公式和日志更直观。
        Vc = -float(np.dot(R_vec, V_rel)) / R_mag

    return RelativeGeometry(
        R_vec=R_vec,
        V_rel=V_rel,
        R_mag=R_mag,
        R_sq=R_sq,
        Omega=Omega,
        Omega_mag=safe_norm(Omega),
        Vc=float(Vc),
        target_speed=target_speed,
    )


def compute_pn_acceleration(N, Vc, Omega, R_vec, R_mag):
    """计算 3D PN 加速度命令。

    公式 ac = N * Vc * (Omega x R_unit)。N 越大转向越积极；Vc 越大，说明接近越快，
    PN 给出的横向修正也会越强。
    """
    R_unit = R_vec / max(R_mag, EPS)
    return N * Vc * np.cross(Omega, R_unit)


def build_pn_trend(
    R_vec,
    R_mag,
    Vc,
    Omega_mag,
    ac_vec,
    tracker_vel,
    target_pos,
    target_vel,
    tracker_speed,
    speed_min,
    dt,
    nmpc_config,
):
    """
    把 PN 控制量改造成趋势信息，供 NMPC 作为参考方向、候选控制和 fallback。

    PNTrend 的作用是把“经典 PN 的直觉”交给 NMPC：NMPC 不盲目搜索所有方向，而是在
    PN 推荐方向、当前视线方向以及少量偏置方向里选择更满足速度/FOV/加速度约束的一项。
    """
    R_unit = R_vec / max(R_mag, EPS)
    pn_velocity_hint = tracker_vel + ac_vec * dt
    reference_direction = unit_or_default(pn_velocity_hint, R_unit)

    target_speed = safe_norm(target_vel)
    fallback_relative_speed = max(tracker_speed + target_speed, speed_min, 1.0)
    intercept_time = R_mag / max(Vc, fallback_relative_speed)
    horizon_time = nmpc_config.horizon_steps * (nmpc_config.prediction_dt or dt)
    intercept_time = float(np.clip(intercept_time, dt, max(horizon_time, dt)))
    reference_intercept_point = target_pos + target_vel * intercept_time

    return PNTrend(
        reference_direction=reference_direction,
        pn_acceleration=ac_vec,
        reference_speed=float(tracker_speed),
        reference_intercept_point=reference_intercept_point,
        distance=float(R_mag),
        closing_speed=float(Vc),
        los_rate=float(Omega_mag),
    )
