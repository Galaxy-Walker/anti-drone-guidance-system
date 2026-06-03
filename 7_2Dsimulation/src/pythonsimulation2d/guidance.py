from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from pythonsimulation2d.config import SimulationConfig
from pythonsimulation2d.dynamics import step_pursuer
from pythonsimulation2d.math_utils import (
    EPS,
    clamp_norm_xy,
    forward_from_yaw,
    fov_visibility,
    norm_xy,
    normalize_xy,
    wrap_angle,
)
from pythonsimulation2d.state import PursuerState, TargetState


@dataclass(slots=True)
class GuidanceMemory:
    # NMPC/MPPI 平滑项需要知道上一步实际使用的水平加速度。
    previous_acceleration: np.ndarray = field(default_factory=lambda: np.zeros(3))
    mppi_rng: np.random.Generator | None = None
    # FOV 算法统一使用水平可见量测，再通过 α-β 滤波估计目标 XY 位置和速度。
    sensor_initialized: bool = False
    sensor_position_estimate: np.ndarray = field(default_factory=lambda: np.zeros(3))
    sensor_velocity_estimate: np.ndarray = field(default_factory=lambda: np.zeros(3))
    previous_yaw: float | None = None


@dataclass(slots=True)
class GuidanceResult:
    acceleration: np.ndarray
    visible: bool
    los_angle: float
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


def sensor_target_reference(
    pursuer: PursuerState,
    target: TargetState,
    memory: GuidanceMemory,
    config: SimulationConfig,
    dt: float,
) -> tuple[TargetState | None, bool, float]:
    visible, los_angle = fov_visibility(
        pursuer.position,
        pursuer.yaw,
        target.position,
        config.guidance.fov_half_angle,
    )
    _update_yaw_rate_memory(pursuer, memory, dt)

    if visible:
        measured_position = target.position.copy()
        measured_position[2] = 0.0
        return alpha_beta_sensor_update(memory, measured_position, dt, config), True, los_angle

    return alpha_beta_sensor_predict(memory, dt), False, los_angle


def _update_yaw_rate_memory(pursuer: PursuerState, memory: GuidanceMemory, dt: float) -> float:
    yaw_rate = 0.0
    if memory.previous_yaw is not None:
        yaw_rate = wrap_angle(pursuer.yaw - memory.previous_yaw) / max(dt, EPS)
    memory.previous_yaw = pursuer.yaw
    return yaw_rate


def alpha_beta_sensor_update(
    memory: GuidanceMemory,
    measured_position: np.ndarray,
    dt: float,
    config: SimulationConfig,
) -> TargetState:
    guidance = config.guidance
    measured_position = measured_position.copy()
    measured_position[2] = 0.0
    if not memory.sensor_initialized:
        memory.sensor_position_estimate = measured_position.copy()
        memory.sensor_velocity_estimate = np.zeros(3)
        memory.sensor_initialized = True
        return TargetState(memory.sensor_position_estimate.copy(), memory.sensor_velocity_estimate.copy(), np.zeros(3))

    position_prediction = memory.sensor_position_estimate + memory.sensor_velocity_estimate * dt
    position_prediction[2] = 0.0
    velocity_prediction = memory.sensor_velocity_estimate.copy()
    velocity_prediction[2] = 0.0
    residual = measured_position - position_prediction
    residual[2] = 0.0

    memory.sensor_position_estimate = position_prediction + guidance.sensor_ab_alpha * residual
    memory.sensor_position_estimate[2] = 0.0
    memory.sensor_velocity_estimate = velocity_prediction + (guidance.sensor_ab_beta / max(dt, EPS)) * residual
    memory.sensor_velocity_estimate[2] = 0.0
    return TargetState(memory.sensor_position_estimate.copy(), memory.sensor_velocity_estimate.copy(), np.zeros(3))


def alpha_beta_sensor_predict(memory: GuidanceMemory, dt: float) -> TargetState | None:
    if not memory.sensor_initialized:
        return None
    memory.sensor_position_estimate = memory.sensor_position_estimate + memory.sensor_velocity_estimate * dt
    memory.sensor_position_estimate[2] = 0.0
    memory.sensor_velocity_estimate[2] = 0.0
    return TargetState(memory.sensor_position_estimate.copy(), memory.sensor_velocity_estimate.copy(), np.zeros(3))


def compute_guidance(
    algorithm: str,
    pursuer: PursuerState,
    target: TargetState,
    memory: GuidanceMemory,
    config: SimulationConfig,
    dt: float,
) -> GuidanceResult:
    if algorithm == "basic":
        _, los_angle = fov_visibility(pursuer.position, pursuer.yaw, target.position, config.guidance.fov_half_angle)
        acceleration = direct_pursuit(pursuer, target, config)
        return GuidanceResult(acceleration, True, los_angle, target.position.copy())

    if algorithm not in {"basic_fov", "pn_fov", "pn_fov_cbf", "pn_fov_nmpc", "pn_fov_mppi"}:
        raise ValueError(f"Unknown algorithm: {algorithm!r}")

    reference_target, visible, los_angle = sensor_target_reference(pursuer, target, memory, config, dt)
    if reference_target is None:
        forward = forward_from_yaw(pursuer.yaw)
        return GuidanceResult(np.zeros(3), False, los_angle, pursuer.position + 10.0 * forward)

    if algorithm == "basic_fov":
        acceleration = direct_pursuit(pursuer, reference_target, config)
        if not visible:
            acceleration *= config.guidance.lost_guidance_gain
        return GuidanceResult(clamp_norm_xy(acceleration, config.pursuer.a_max), visible, los_angle, reference_target.position.copy())

    acceleration = pn_guidance(pursuer, reference_target, config)
    if not visible:
        acceleration *= config.guidance.lost_guidance_gain

    if algorithm == "pn_fov_cbf":
        acceleration = fov_cbf_acceleration(pursuer, reference_target, acceleration, config)
    if algorithm == "pn_fov_nmpc":
        acceleration = nmpc_acceleration(pursuer, reference_target, acceleration, memory, config)
    if algorithm == "pn_fov_mppi":
        acceleration = mppi_acceleration(pursuer, reference_target, acceleration, memory, config)

    return GuidanceResult(clamp_norm_xy(acceleration, config.pursuer.a_max), visible, los_angle, reference_target.position.copy())


def fov_cbf_acceleration(
    pursuer: PursuerState,
    target_reference: TargetState,
    nominal_acceleration: np.ndarray,
    config: SimulationConfig,
) -> np.ndarray:
    guidance = config.guidance
    r_unit = normalize_xy(target_reference.position - pursuer.position)
    if norm_xy(r_unit) < EPS:
        return nominal_acceleration

    forward = forward_from_yaw(pursuer.yaw)
    los_angle = float(np.arccos(np.clip(np.dot(forward[:2], r_unit[:2]), -1.0, 1.0)))
    margin = np.deg2rad(guidance.cbf_fov_margin_deg)
    soft_limit = max(0.0, guidance.fov_half_angle - margin)
    if los_angle <= soft_limit:
        return nominal_acceleration

    lateral_error = r_unit - float(np.dot(r_unit[:2], forward[:2])) * forward
    lateral_direction = normalize_xy(lateral_error)
    if norm_xy(lateral_direction) < EPS:
        return nominal_acceleration

    severity = (los_angle - soft_limit) / max(guidance.fov_half_angle - soft_limit, EPS)
    severity = float(np.clip(severity, 0.0, 1.5))
    correction = guidance.cbf_gain * severity * config.pursuer.a_max * lateral_direction
    match_weight = 0.45 * min(severity, 1.0)
    velocity_match = (target_reference.velocity - pursuer.velocity) / max(guidance.direct_tau, EPS)
    filtered = (1.0 - match_weight) * nominal_acceleration + match_weight * velocity_match + correction
    filtered = clamp_norm_xy(filtered, config.pursuer.a_max)
    if _one_step_fov_score(pursuer, target_reference, filtered, config) > _one_step_fov_score(
        pursuer,
        target_reference,
        nominal_acceleration,
        config,
    ):
        return nominal_acceleration
    return filtered


def _one_step_fov_score(
    pursuer: PursuerState,
    target_reference: TargetState,
    acceleration: np.ndarray,
    config: SimulationConfig,
) -> float:
    predicted_target_position = target_reference.position + target_reference.velocity * config.dt
    predicted_target_position[2] = 0.0
    predicted_pursuer = step_pursuer(pursuer, acceleration, predicted_target_position, config, config.dt)
    _, los_angle = fov_visibility(
        predicted_pursuer.position,
        predicted_pursuer.yaw,
        predicted_target_position,
        config.guidance.fov_half_angle,
    )
    distance = norm_xy(predicted_target_position - predicted_pursuer.position)
    return los_angle + 0.01 * distance


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
    yaw = np.full(sample_count, pursuer.yaw)
    previous_acceleration = np.repeat(memory.previous_acceleration[None, :], sample_count, axis=0)

    path_cost = np.zeros(sample_count)
    fov_cost = np.zeros(sample_count)
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

        target_direction_xy = predicted_position[:2] - position[:, :2]
        target_yaw = np.arctan2(target_direction_xy[:, 1], target_direction_xy[:, 0])
        yaw_delta = np.clip(
            _wrap_angles(target_yaw - yaw),
            -config.pursuer.yaw_rate_max * guidance.mpc_dt,
            config.pursuer.yaw_rate_max * guidance.mpc_dt,
        )
        yaw = _wrap_angles(yaw + yaw_delta)

        distance = np.linalg.norm(target_direction_xy, axis=1)
        forward_xy = np.column_stack((np.cos(yaw), np.sin(yaw)))
        target_unit_xy = target_direction_xy / np.maximum(distance[:, None], EPS)
        los_angle = np.arccos(np.clip(np.sum(forward_xy * target_unit_xy, axis=1), -1.0, 1.0))
        fov_violation = np.maximum(0.0, los_angle - guidance.fov_half_angle)

        path_cost += distance
        fov_cost += fov_violation**2
        control_cost += np.sum(acceleration[:, :2] ** 2, axis=1) * guidance.mpc_dt
        smooth_cost += np.sum((acceleration[:, :2] - previous_acceleration[:, :2]) ** 2, axis=1)
        pn_cost += np.sum((acceleration[:, :2] - pn_trend[:2]) ** 2, axis=1)
        previous_acceleration = acceleration

    final_position = target.position + target.velocity * acceleration_sequences.shape[1] * guidance.mpc_dt
    final_distance = np.linalg.norm(final_position[:2] - position[:, :2], axis=1)
    return (
        guidance.nmpc_w_dist * final_distance
        + guidance.nmpc_w_path * path_cost
        + guidance.nmpc_w_fov * fov_cost
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


def _wrap_angles(angles: np.ndarray) -> np.ndarray:
    return (angles + np.pi) % (2.0 * np.pi) - np.pi


def _candidate_accelerations(
    pursuer: PursuerState,
    target: TargetState,
    pn_trend: np.ndarray,
    config: SimulationConfig,
) -> list[np.ndarray]:
    guidance = config.guidance
    a_max = config.pursuer.a_max
    r_unit = normalize_xy(target.position - pursuer.position)
    intercept_velocity = guidance.pn_v_des_along_los * r_unit
    intercept = (intercept_velocity - pursuer.velocity) / max(guidance.direct_tau, EPS)
    same_speed_velocity = norm_xy(target.velocity) * r_unit
    same_speed = (same_speed_velocity - pursuer.velocity) / max(guidance.direct_tau, EPS)
    velocity_match = (target.velocity - pursuer.velocity) / max(guidance.direct_tau, EPS)

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
        pn_trend + 0.35 * a_max * lateral,
        pn_trend - 0.35 * a_max * lateral,
    ]
    return [clamp_norm_xy(candidate, a_max) for candidate in raw_candidates]


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
    fov_cost = 0.0
    control_cost = 0.0
    smooth_cost = 0.0
    pn_cost = 0.0

    for step in range(1, guidance.horizon_steps + 1):
        t_pred = step * guidance.mpc_dt
        predicted_position = target.position + target.velocity * t_pred
        predicted_position[2] = 0.0
        state = step_pursuer(state, acceleration, predicted_position, config, guidance.mpc_dt)

        distance = norm_xy(predicted_position - state.position)
        _, los_angle = fov_visibility(state.position, state.yaw, predicted_position, guidance.fov_half_angle)
        if los_angle > guidance.fov_half_angle:
            fov_cost += (los_angle - guidance.fov_half_angle) ** 2
        path_cost += distance
        control_cost += norm_xy(acceleration) ** 2 * guidance.mpc_dt
        smooth_cost += norm_xy(acceleration - previous_acceleration) ** 2
        pn_cost += norm_xy(acceleration - pn_trend) ** 2
        previous_acceleration = acceleration

    final_position = target.position + target.velocity * guidance.horizon_steps * guidance.mpc_dt
    final_distance = norm_xy(final_position - state.position)
    return (
        guidance.nmpc_w_dist * final_distance
        + guidance.nmpc_w_path * path_cost
        + guidance.nmpc_w_fov * fov_cost
        + guidance.nmpc_w_control * control_cost
        + guidance.nmpc_w_smooth * smooth_cost
        + guidance.nmpc_w_pn * pn_cost
    )
