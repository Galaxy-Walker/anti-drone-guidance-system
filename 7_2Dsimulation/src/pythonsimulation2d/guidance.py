from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from pythonsimulation2d.config import SimulationConfig
from pythonsimulation2d.dynamics import step_pursuer
from pythonsimulation2d.math_utils import EPS, clamp_norm_xy, norm_xy, normalize_xy
from pythonsimulation2d.state import PursuerState, TargetState


@dataclass(slots=True)
class GuidanceMemory:
    # NMPC/MPPI 平滑项需要知道上一步实际使用的水平加速度。
    previous_acceleration: np.ndarray = field(default_factory=lambda: np.zeros(3))
    mppi_rng: np.random.Generator | None = None


@dataclass(slots=True)
class GuidanceResult:
    acceleration: np.ndarray
    look_at_position: np.ndarray


def direct_pursuit(pursuer: PursuerState, target: TargetState, config: SimulationConfig) -> np.ndarray:
    guidance = config.guidance
    direction = normalize_xy(target.position - pursuer.position)
    desired_velocity = guidance.direct_v_cruise * direction
    return clamp_norm_xy((desired_velocity - pursuer.velocity) / guidance.direct_tau, config.pursuer.a_max)


def pn_guidance(pursuer: PursuerState, target: TargetState, config: SimulationConfig) -> np.ndarray:
    guidance = config.guidance
    r = target.position - pursuer.position
    r_norm = norm_xy(r)
    if r_norm < EPS:
        return np.zeros(3)

    u_los = normalize_xy(r)
    v_rel = target.velocity - pursuer.velocity
    closing_speed = max(0.0, -float(np.dot(v_rel[:2], u_los[:2])))

    # 2D PN：LOS 角速度是标量 omega_z = (r x v_rel)_z / |r|^2。
    omega_los = float((r[0] * v_rel[1] - r[1] * v_rel[0]) / max(r_norm**2, EPS))
    lateral = np.array([-u_los[1], u_los[0], 0.0])
    a_pn = guidance.pn_navigation_constant * closing_speed * omega_los * lateral

    current_los_speed = float(np.dot(pursuer.velocity[:2], u_los[:2]))
    a_close = guidance.pn_k_close * (guidance.pn_v_des_along_los - current_los_speed) * u_los
    return clamp_norm_xy(a_pn + a_close, config.pursuer.a_max)


def compute_guidance(
    algorithm: str,
    pursuer: PursuerState,
    target: TargetState,
    memory: GuidanceMemory,
    config: SimulationConfig,
    dt: float,
) -> GuidanceResult:
    del dt  # 当前导引律的离散步长由 config.dt/guidance.mpc_dt 统一管理。

    if algorithm == "basic":
        acceleration = direct_pursuit(pursuer, target, config)
    elif algorithm == "pn":
        acceleration = pn_guidance(pursuer, target, config)
    elif algorithm == "pn_nmpc":
        pn_trend = pn_guidance(pursuer, target, config)
        acceleration = nmpc_acceleration(pursuer, target, pn_trend, memory, config)
    elif algorithm == "pn_mppi":
        pn_trend = pn_guidance(pursuer, target, config)
        acceleration = mppi_acceleration(pursuer, target, pn_trend, memory, config)
    else:
        raise ValueError(f"Unknown algorithm: {algorithm!r}")

    return GuidanceResult(clamp_norm_xy(acceleration, config.pursuer.a_max), target.position.copy())


def nmpc_acceleration(
    pursuer: PursuerState,
    target_reference: TargetState,
    pn_trend: np.ndarray,
    memory: GuidanceMemory,
    config: SimulationConfig,
) -> np.ndarray:
    candidates = _candidate_accelerations(pursuer, target_reference, pn_trend, config)
    best_cost = float("inf")
    best_acceleration = pn_trend
    for acceleration in candidates:
        cost = _rollout_cost(pursuer, target_reference, acceleration, pn_trend, memory, config)
        if cost < best_cost:
            best_cost = cost
            best_acceleration = acceleration
    return clamp_norm_xy(best_acceleration, config.pursuer.a_max)


def mppi_acceleration(
    pursuer: PursuerState,
    target_reference: TargetState,
    pn_trend: np.ndarray,
    memory: GuidanceMemory,
    config: SimulationConfig,
) -> np.ndarray:
    guidance = config.guidance
    horizon = guidance.horizon_steps
    sample_count = guidance.mppi_samples
    a_max = config.pursuer.a_max
    if memory.mppi_rng is None:
        memory.mppi_rng = np.random.default_rng(guidance.mppi_seed)

    nominal = np.tile(clamp_norm_xy(pn_trend, a_max), (sample_count, horizon, 1))
    noise = memory.mppi_rng.normal(0.0, guidance.mppi_noise_scale, size=(sample_count, horizon, 3))
    noise[:, :, 2] = 0.0
    noise[0, :, :] = 0.0
    for step in range(1, horizon):
        noise[:, step, :] = 0.65 * noise[:, step - 1, :] + 0.35 * noise[:, step, :]

    sequences = _clamp_acceleration_sequences(nominal + noise, a_max)
    costs = _mppi_sequence_costs(pursuer, target_reference, sequences, pn_trend, memory, config)
    costs -= float(np.min(costs))
    weights = np.exp(-costs / max(guidance.mppi_temperature, EPS))
    weight_sum = float(np.sum(weights))
    if weight_sum < EPS:
        return clamp_norm_xy(pn_trend, a_max)

    first_controls = sequences[:, 0, :]
    weighted_acceleration = np.sum(first_controls * weights[:, None], axis=0) / weight_sum
    return clamp_norm_xy(weighted_acceleration, a_max)


def _clamp_acceleration_sequences(sequences: np.ndarray, a_max: float) -> np.ndarray:
    sequences = sequences.copy()
    sequences[:, :, 2] = 0.0
    norms = np.linalg.norm(sequences[:, :, :2], axis=2, keepdims=True)
    scales = np.minimum(1.0, a_max / np.maximum(norms, EPS))
    sequences[:, :, :2] *= scales
    return sequences


def _mppi_sequence_costs(
    pursuer: PursuerState,
    target: TargetState,
    acceleration_sequences: np.ndarray,
    pn_trend: np.ndarray,
    memory: GuidanceMemory,
    config: SimulationConfig,
) -> np.ndarray:
    guidance = config.guidance
    sample_count = acceleration_sequences.shape[0]
    position = np.repeat(pursuer.position[None, :], sample_count, axis=0)
    velocity = np.repeat(pursuer.velocity[None, :], sample_count, axis=0)
    previous_acceleration = np.repeat(memory.previous_acceleration[None, :], sample_count, axis=0)

    path_cost = np.zeros(sample_count)
    control_cost = np.zeros(sample_count)
    smooth_cost = np.zeros(sample_count)
    pn_cost = np.zeros(sample_count)

    for step in range(acceleration_sequences.shape[1]):
        acceleration = acceleration_sequences[:, step, :]
        t_pred = (step + 1) * guidance.mpc_dt
        predicted_position = target.position + target.velocity * t_pred
        predicted_position[2] = 0.0

        velocity = _clamp_rows_xy(velocity + acceleration * guidance.mpc_dt, config.pursuer.v_max)
        position = position + velocity * guidance.mpc_dt
        position[:, 2] = config.pursuer.fixed_altitude

        distance = np.linalg.norm(predicted_position[:2] - position[:, :2], axis=1)
        path_cost += distance
        control_cost += np.sum(acceleration[:, :2] ** 2, axis=1) * guidance.mpc_dt
        smooth_cost += np.sum((acceleration[:, :2] - previous_acceleration[:, :2]) ** 2, axis=1)
        pn_cost += np.sum((acceleration[:, :2] - pn_trend[:2]) ** 2, axis=1)
        previous_acceleration = acceleration

    final_position = target.position + target.velocity * acceleration_sequences.shape[1] * guidance.mpc_dt
    final_distance = np.linalg.norm(final_position[:2] - position[:, :2], axis=1)
    return (
        guidance.nmpc_w_dist * final_distance
        + guidance.nmpc_w_path * path_cost
        + guidance.nmpc_w_control * control_cost
        + guidance.nmpc_w_smooth * smooth_cost
        + guidance.nmpc_w_pn * pn_cost
    )


def _clamp_rows_xy(vectors: np.ndarray, max_norm: float) -> np.ndarray:
    result = vectors.copy()
    result[:, 2] = 0.0
    row_norms = np.linalg.norm(result[:, :2], axis=1, keepdims=True)
    scales = np.minimum(1.0, max_norm / np.maximum(row_norms, EPS))
    result[:, :2] *= scales
    return result


def _candidate_accelerations(
    pursuer: PursuerState,
    target: TargetState,
    pn_trend: np.ndarray,
    config: SimulationConfig,
) -> list[np.ndarray]:
    guidance = config.guidance
    a_max = config.pursuer.a_max
    position_error = target.position - pursuer.position
    velocity_error = target.velocity - pursuer.velocity
    r_unit = normalize_xy(position_error)
    intercept_velocity = guidance.pn_v_des_along_los * r_unit
    intercept = (intercept_velocity - pursuer.velocity) / max(guidance.direct_tau, EPS)
    same_speed_velocity = norm_xy(target.velocity) * r_unit
    same_speed = (same_speed_velocity - pursuer.velocity) / max(guidance.direct_tau, EPS)
    velocity_match = (target.velocity - pursuer.velocity) / max(guidance.direct_tau, EPS)

    # 末端稳定跟踪候选：不是在接近末端额外加速，而是把 NMPC 的候选控制中
    # 显式加入“位置误差 + 相对速度误差 + 目标当前加速度前馈”的稳态跟踪模式。
    # 对圆周目标，target.acceleration 提供保持同轨迹所需的向心前馈；
    # velocity_error 项抑制掠过目标后的振荡，帮助把长期误差压低。
    tracking_kp = 0.85
    tracking_kd = 1.75
    stable_tracking = target.acceleration + tracking_kp * position_error + tracking_kd * velocity_error
    soft_tracking = target.acceleration + 0.55 * tracking_kp * position_error + 1.15 * tracking_kd * velocity_error
    velocity_tracking = target.acceleration + 1.45 * velocity_error

    lateral = np.array([-r_unit[1], r_unit[0], 0.0])
    if norm_xy(lateral) < EPS:
        lateral = np.array([0.0, 1.0, 0.0])

    raw_candidates = [
        pn_trend,
        0.55 * pn_trend,
        1.25 * pn_trend,
        0.75 * pn_trend + 0.25 * intercept,
        0.5 * pn_trend + 0.5 * intercept,
        same_speed,
        0.5 * pn_trend + 0.5 * same_speed,
        velocity_match,
        0.5 * pn_trend + 0.5 * velocity_match,
        stable_tracking,
        soft_tracking,
        velocity_tracking,
        0.5 * pn_trend + 0.5 * stable_tracking,
        0.35 * pn_trend + 0.65 * soft_tracking,
        pn_trend + 0.35 * a_max * lateral,
        pn_trend - 0.35 * a_max * lateral,
    ]
    return [clamp_norm_xy(candidate, a_max) for candidate in raw_candidates]


def _predict_target_constant_acceleration(target: TargetState, t_pred: float) -> tuple[np.ndarray, np.ndarray]:
    predicted_position = target.position + target.velocity * t_pred + 0.5 * target.acceleration * t_pred**2
    predicted_velocity = target.velocity + target.acceleration * t_pred
    predicted_position = predicted_position.copy()
    predicted_velocity = predicted_velocity.copy()
    predicted_position[2] = 0.0
    predicted_velocity[2] = 0.0
    return predicted_position, predicted_velocity


def _rollout_cost(
    pursuer: PursuerState,
    target: TargetState,
    acceleration: np.ndarray,
    pn_trend: np.ndarray,
    memory: GuidanceMemory,
    config: SimulationConfig,
) -> float:
    guidance = config.guidance
    state = pursuer.copy()
    previous_acceleration = memory.previous_acceleration
    path_cost = 0.0
    control_cost = 0.0
    smooth_cost = 0.0
    pn_cost = 0.0
    velocity_cost = 0.0
    steady_cost = 0.0

    for step in range(1, guidance.horizon_steps + 1):
        t_pred = step * guidance.mpc_dt
        predicted_position, predicted_velocity = _predict_target_constant_acceleration(target, t_pred)
        state = step_pursuer(state, acceleration, predicted_position, config, guidance.mpc_dt)

        distance = norm_xy(predicted_position - state.position)
        relative_velocity = predicted_velocity - state.velocity
        path_cost += distance
        control_cost += norm_xy(acceleration) ** 2 * guidance.mpc_dt
        smooth_cost += norm_xy(acceleration - previous_acceleration) ** 2
        pn_cost += norm_xy(acceleration - pn_trend) ** 2
        velocity_cost += norm_xy(relative_velocity) ** 2 * guidance.mpc_dt
        if step > guidance.horizon_steps // 2:
            steady_cost += distance**2 + 0.35 * norm_xy(relative_velocity) ** 2
        previous_acceleration = acceleration

    final_position, final_velocity = _predict_target_constant_acceleration(
        target,
        guidance.horizon_steps * guidance.mpc_dt,
    )
    final_distance = norm_xy(final_position - state.position)
    final_velocity_error = norm_xy(final_velocity - state.velocity)
    return (
        guidance.nmpc_w_dist * final_distance
        + guidance.nmpc_w_path * path_cost
        + 0.45 * guidance.nmpc_w_dist * final_velocity_error
        + 0.18 * guidance.nmpc_w_dist * steady_cost
        + 0.35 * guidance.nmpc_w_path * velocity_cost
        + guidance.nmpc_w_control * control_cost
        + guidance.nmpc_w_smooth * smooth_cost
        + guidance.nmpc_w_pn * pn_cost
    )
