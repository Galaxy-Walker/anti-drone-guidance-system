from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from pythonsimulation.config import SimulationConfig
from pythonsimulation.dynamics import step_pursuer
from pythonsimulation.math_utils import EPS, clamp_norm, forward_from_yaw_pitch, fov_visibility, norm, normalize
from pythonsimulation.state import PursuerState, TargetState


@dataclass(slots=True)
class GuidanceMemory:
    # FOV 算法看不见目标时，只能使用最后一次观测到的位置和速度做短时预测。
    last_seen_target: TargetState | None = None
    lost_time: float = 0.0
    # NMPC 代价中的平滑项需要知道上一步实际使用的加速度。
    previous_acceleration: np.ndarray = field(default_factory=lambda: np.zeros(3))
    # MPPI 使用随机采样；第一次使用时按 config 中的 seed 初始化，保证仿真可复现。
    mppi_rng: np.random.Generator | None = None


@dataclass(slots=True)
class GuidanceResult:
    # acceleration 是导引律建议的加速度；真正执行前还会在 simulation.py 中统一限幅。
    acceleration: np.ndarray
    # visible 表示当前真实目标是否在视场内，用于 FOV 指标和可见性曲线。
    visible: bool
    # los_angle 是目标方向与机头/相机前向之间的夹角，单位是弧度。
    los_angle: float
    # look_at_position 决定下一步机头转向哪里；目标丢失时它会是预测点而不是真实点。
    look_at_position: np.ndarray


def direct_pursuit(pursuer: PursuerState, target: TargetState, config: SimulationConfig) -> np.ndarray:
    guidance = config.guidance
    # 基础追踪只关心目标当前位置：期望速度始终指向目标。
    direction = normalize(target.position - pursuer.position)
    desired_velocity = guidance.direct_v_cruise * direction
    # 一阶速度跟踪：用 tau 控制从当前速度追到期望速度的快慢。
    return (desired_velocity - pursuer.velocity) / guidance.direct_tau


def pn_guidance(pursuer: PursuerState, target: TargetState, config: SimulationConfig) -> np.ndarray:
    guidance = config.guidance
    # r 是从追踪机指向目标的相对位置，LOS 是视线方向。
    r = target.position - pursuer.position
    r_norm = norm(r)
    if r_norm < EPS:
        return np.zeros(3)

    u_los = r / r_norm
    v_rel = target.velocity - pursuer.velocity
    # closing_speed 表示沿 LOS 方向的接近速度；小于 0 时说明正在远离，PN 项不再反向放大。
    closing_speed = max(0.0, -float(np.dot(v_rel, u_los)))
    # omega_los 是 LOS 方向的旋转角速度，PN 用它来修正横向拦截误差。
    omega_los = np.cross(r, v_rel) / max(r_norm**2, EPS)
    a_pn = guidance.pn_navigation_constant * closing_speed * np.cross(omega_los, u_los)
    current_los_speed = float(np.dot(pursuer.velocity, u_los))
    # 纯 PN 在初始速度很小时可能接近慢，额外径向闭合项负责“主动往目标靠”。
    a_close = guidance.pn_k_close * (guidance.pn_v_des_along_los - current_los_speed) * u_los
    return a_pn + a_close


def compute_guidance(
    algorithm: str,
    pursuer: PursuerState,
    target: TargetState,
    memory: GuidanceMemory,
    config: SimulationConfig,
    dt: float,
) -> GuidanceResult:
    # 先统一计算真实目标是否在当前 FOV 内；无 FOV 的算法只把它作为绘图数据。
    visible, los_angle = fov_visibility(
        pursuer.position,
        pursuer.yaw,
        pursuer.pitch,
        target.position,
        config.guidance.fov_half_angle,
    )

    if algorithm == "basic":
        acceleration = direct_pursuit(pursuer, target, config)
        # basic/pn 是理想传感器假设：目标始终可用于导引，所以 visible 强制记为 True。
        return GuidanceResult(acceleration, True, los_angle, target.position.copy())

    if algorithm == "pn":
        acceleration = pn_guidance(pursuer, target, config)
        return GuidanceResult(acceleration, True, los_angle, target.position.copy())

    if algorithm not in {"pn_fov", "pn_fov_cbf", "pn_fov_nmpc", "pn_fov_mppi"}:
        raise ValueError(f"Unknown algorithm: {algorithm!r}")

    reference_target = _target_reference_for_fov(target, memory, visible, dt)
    acceleration = pn_guidance(pursuer, reference_target, config)
    if not visible:
        # 丢失目标时仍沿预测点搜索，但降低导引强度，避免基于旧信息过度机动。
        acceleration *= config.guidance.lost_guidance_gain

    if algorithm == "pn_fov_cbf":
        # CBF 风格的安全滤波器不替代 PN，而是在接近 FOV 边界时加入侧向修正。
        acceleration = fov_cbf_acceleration(pursuer, reference_target, acceleration, config)

    if algorithm == "pn_fov_nmpc":
        # NMPC 不替代 PN，而是在 PN 给出的趋势附近尝试若干候选，选一个更兼顾 FOV 的控制。
        acceleration = nmpc_acceleration(pursuer, reference_target, acceleration, memory, config)

    if algorithm == "pn_fov_mppi":
        # MPPI 通过随机采样多条控制序列，按预测代价对第一步控制做加权平均。
        acceleration = mppi_acceleration(pursuer, reference_target, acceleration, memory, config)

    return GuidanceResult(acceleration, visible, los_angle, reference_target.position.copy())


def _target_reference_for_fov(
    target: TargetState,
    memory: GuidanceMemory,
    visible: bool,
    dt: float,
) -> TargetState:
    if visible or memory.last_seen_target is None:
        # 看得见目标时刷新最后观测状态；第一次看不见时也用真实目标初始化，避免空引用。
        memory.last_seen_target = target.copy()
        memory.lost_time = 0.0
        return target

    memory.lost_time += dt
    last_seen = memory.last_seen_target
    # 初版不“偷看”真实未来轨迹，只假设目标保持最后观测到的速度匀速运动。
    predicted_position = last_seen.position + last_seen.velocity * memory.lost_time
    return TargetState(predicted_position, last_seen.velocity.copy(), np.zeros(3))


def nmpc_acceleration(
    pursuer: PursuerState,
    target_reference: TargetState,
    pn_trend: np.ndarray,
    memory: GuidanceMemory,
    config: SimulationConfig,
) -> np.ndarray:
    # 轻量 NMPC：不调用优化器，而是枚举一组候选加速度，前向滚动仿真后选最低代价。
    candidates = _candidate_accelerations(pursuer, target_reference, pn_trend, config)
    best_cost = float("inf")
    best_acceleration = pn_trend

    for acceleration in candidates:
        # 每个候选都假设在预测窗口内保持该加速度，比较谁的综合代价更低。
        cost = _rollout_cost(pursuer, target_reference, acceleration, pn_trend, memory, config)
        if cost < best_cost:
            # 这里只保存当前最低代价的第一步控制；滚动时下一帧会重新计算候选和代价。
            best_cost = cost
            best_acceleration = acceleration

    return best_acceleration


def fov_cbf_acceleration(
    pursuer: PursuerState,
    target_reference: TargetState,
    nominal_acceleration: np.ndarray,
    config: SimulationConfig,
) -> np.ndarray:
    guidance = config.guidance
    r = target_reference.position - pursuer.position
    r_unit = normalize(r)
    if norm(r_unit) < EPS:
        return nominal_acceleration

    forward = forward_from_yaw_pitch(pursuer.yaw, pursuer.pitch)
    los_angle = float(np.arccos(np.clip(np.dot(forward, r_unit), -1.0, 1.0)))
    margin = np.deg2rad(guidance.cbf_fov_margin_deg)
    soft_limit = max(0.0, guidance.fov_half_angle - margin)
    if los_angle <= soft_limit:
        return nominal_acceleration

    # 目标偏在视场哪一侧，就让追踪机向同侧机动，使 LOS 往相机中心回收。
    lateral_error = r_unit - float(np.dot(r_unit, forward)) * forward
    lateral_direction = normalize(lateral_error)
    if norm(lateral_direction) < EPS:
        return nominal_acceleration

    severity = (los_angle - soft_limit) / max(guidance.fov_half_angle - soft_limit, EPS)
    severity = float(np.clip(severity, 0.0, 1.5))
    correction = guidance.cbf_gain * severity * config.pursuer.a_max * lateral_direction
    # 靠近视场边界时适当从拦截切换到速度匹配，降低近距离飞越导致的二次丢失。
    match_weight = 0.45 * min(severity, 1.0)
    velocity_match = (target_reference.velocity - pursuer.velocity) / max(guidance.direct_tau, EPS)
    filtered = (1.0 - match_weight) * nominal_acceleration + match_weight * velocity_match + correction
    filtered = clamp_norm(filtered, config.pursuer.a_max)
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
    predicted_pursuer = step_pursuer(pursuer, acceleration, predicted_target_position, config, config.dt)
    _, los_angle = fov_visibility(
        predicted_pursuer.position,
        predicted_pursuer.yaw,
        predicted_pursuer.pitch,
        predicted_target_position,
        config.guidance.fov_half_angle,
    )
    distance = norm(predicted_target_position - predicted_pursuer.position)
    return los_angle + 0.01 * distance


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

    nominal = np.tile(clamp_norm(pn_trend, a_max), (sample_count, horizon, 1))
    noise = memory.mppi_rng.normal(0.0, guidance.mppi_noise_scale, size=(sample_count, horizon, 3))
    noise[0, :, :] = 0.0
    # 给随机扰动加一点时间相关性，避免 MPPI 候选序列在预测窗口内高频抖动。
    for step in range(1, horizon):
        noise[:, step, :] = 0.65 * noise[:, step - 1, :] + 0.35 * noise[:, step, :]

    sequences = nominal + noise
    sequences = _clamp_acceleration_sequences(sequences, a_max)
    costs = _mppi_sequence_costs(pursuer, target_reference, sequences, pn_trend, memory, config)
    costs -= float(np.min(costs))
    weights = np.exp(-costs / max(guidance.mppi_temperature, EPS))
    weight_sum = float(np.sum(weights))
    if weight_sum < EPS:
        return clamp_norm(pn_trend, a_max)

    first_controls = sequences[:, 0, :]
    weighted_acceleration = np.sum(first_controls * weights[:, None], axis=0) / weight_sum
    return clamp_norm(weighted_acceleration, a_max)


def _clamp_acceleration_sequences(sequences: np.ndarray, a_max: float) -> np.ndarray:
    norms = np.linalg.norm(sequences, axis=2, keepdims=True)
    scales = np.minimum(1.0, a_max / np.maximum(norms, EPS))
    return sequences * scales


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
    pitch = np.full(sample_count, pursuer.pitch)
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

        velocity = _clamp_rows(velocity + acceleration * guidance.mpc_dt, config.pursuer.v_max)
        position = position + velocity * guidance.mpc_dt
        position[:, 2] = np.clip(position[:, 2], config.pursuer.z_min, config.pursuer.z_max)
        low_limited = (position[:, 2] <= config.pursuer.z_min + 1e-9) & (velocity[:, 2] < 0.0)
        high_limited = (position[:, 2] >= config.pursuer.z_max - 1e-9) & (velocity[:, 2] > 0.0)
        velocity[low_limited | high_limited, 2] = 0.0

        target_direction = predicted_position - position
        horizontal = np.hypot(target_direction[:, 0], target_direction[:, 1])
        target_yaw = np.arctan2(target_direction[:, 1], target_direction[:, 0])
        target_pitch = np.arctan2(target_direction[:, 2], horizontal)
        yaw_delta = np.clip(
            _wrap_angles(target_yaw - yaw),
            -config.pursuer.yaw_rate_max * guidance.mpc_dt,
            config.pursuer.yaw_rate_max * guidance.mpc_dt,
        )
        pitch_delta = np.clip(
            target_pitch - pitch,
            -config.pursuer.pitch_rate_max * guidance.mpc_dt,
            config.pursuer.pitch_rate_max * guidance.mpc_dt,
        )
        yaw = _wrap_angles(yaw + yaw_delta)
        pitch = np.clip(pitch + pitch_delta, -np.pi * 0.5, np.pi * 0.5)

        distance = np.linalg.norm(target_direction, axis=1)
        forward = np.column_stack((
            np.cos(pitch) * np.cos(yaw),
            np.cos(pitch) * np.sin(yaw),
            np.sin(pitch),
        ))
        target_unit = target_direction / np.maximum(distance[:, None], EPS)
        los_angle = np.arccos(np.clip(np.sum(forward * target_unit, axis=1), -1.0, 1.0))
        fov_violation = np.maximum(0.0, los_angle - guidance.fov_half_angle)

        path_cost += distance
        fov_cost += fov_violation**2
        control_cost += np.sum(acceleration**2, axis=1) * guidance.mpc_dt
        smooth_cost += np.sum((acceleration - previous_acceleration) ** 2, axis=1)
        pn_cost += np.sum((acceleration - pn_trend) ** 2, axis=1)
        previous_acceleration = acceleration

    final_position = target.position + target.velocity * acceleration_sequences.shape[1] * guidance.mpc_dt
    final_distance = np.linalg.norm(final_position - position, axis=1)
    return (
        guidance.nmpc_w_dist * final_distance
        + guidance.nmpc_w_path * path_cost
        + guidance.nmpc_w_fov * fov_cost
        + guidance.nmpc_w_control * control_cost
        + guidance.nmpc_w_smooth * smooth_cost
        + guidance.nmpc_w_pn * pn_cost
    )


def _clamp_rows(vectors: np.ndarray, max_norm: float) -> np.ndarray:
    row_norms = np.linalg.norm(vectors, axis=1, keepdims=True)
    scales = np.minimum(1.0, max_norm / np.maximum(row_norms, EPS))
    return vectors * scales


def _wrap_angles(angles: np.ndarray) -> np.ndarray:
    return (angles + np.pi) % (2.0 * np.pi) - np.pi


def _candidate_accelerations(
    pursuer: PursuerState,
    target: TargetState,
    pn_trend: np.ndarray,
    config: SimulationConfig,
) -> list[np.ndarray]:
    a_max = config.pursuer.a_max
    r_unit = normalize(target.position - pursuer.position)
    # “拦截”候选偏向快速接近目标；“速度匹配”候选偏向捕获后跟随目标运动。
    intercept_velocity = config.guidance.pn_v_des_along_los * r_unit
    intercept = (intercept_velocity - pursuer.velocity) / max(config.guidance.direct_tau, EPS)
    velocity_match = (target.velocity - pursuer.velocity) / max(config.guidance.direct_tau, EPS)

    # 构造两个与 LOS 垂直的方向，用来测试“向左/右/上/下稍微绕一下”是否更保视场。
    lateral = normalize(np.cross(r_unit, np.array([0.0, 0.0, 1.0])))
    if norm(lateral) < EPS:
        lateral = np.array([0.0, 1.0, 0.0])
    vertical_lateral = normalize(np.cross(lateral, r_unit))

    raw_candidates = [
        # 原始 PN 趋势：作为基准候选，NMPC 至少不会比“什么都不改”更少选择。
        pn_trend,
        # 较小/较大增益：测试同一方向上保守一点或激进一点是否更好。
        0.55 * pn_trend,
        1.25 * pn_trend,
        # 混入“拦截”分量：让候选更偏向直接接近目标，帮助减小最终距离。
        0.75 * pn_trend + 0.25 * intercept,
        0.5 * pn_trend + 0.5 * intercept,
        # “速度匹配”：接近目标后尝试匹配目标速度，减少飞过目标导致的视场丢失。
        velocity_match,
        0.5 * pn_trend + 0.5 * velocity_match,
        # 横向/竖向横向扰动：不是随机扰动，而是沿 LOS 垂直方向的固定试探。
        pn_trend + 0.35 * a_max * lateral,
        pn_trend - 0.35 * a_max * lateral,
        pn_trend + 0.25 * a_max * vertical_lateral,
        pn_trend - 0.25 * a_max * vertical_lateral,
    ]
    # 所有候选都必须满足同一加速度上限，保证算法对比公平。
    return [clamp_norm(candidate, a_max) for candidate in raw_candidates]


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
    # 以下各项分别对应计划文档中的距离、FOV、控制量、平滑性和偏离 PN 趋势惩罚。
    path_cost = 0.0
    fov_cost = 0.0
    control_cost = 0.0
    smooth_cost = 0.0
    pn_cost = 0.0

    for step in range(1, guidance.horizon_steps + 1):
        t_pred = step * guidance.mpc_dt
        # 预测目标未来位置时只用最后观测状态的常速度模型，避免使用真实未来轨迹作弊。
        predicted_position = target.position + target.velocity * t_pred
        # 在候选加速度下前向模拟追踪机，得到预测窗口内的状态序列。
        state = step_pursuer(state, acceleration, predicted_position, config, guidance.mpc_dt)

        distance = norm(predicted_position - state.position)
        _, los_angle = fov_visibility(
            state.position,
            state.yaw,
            state.pitch,
            predicted_position,
            guidance.fov_half_angle,
        )
        if los_angle > guidance.fov_half_angle:
            # 超出视场越多，惩罚越大；平方项会更强烈惩罚大角度丢失。
            fov_cost += (los_angle - guidance.fov_half_angle) ** 2
        # 路径代价累计整个预测窗口内的距离；最终距离只看窗口最后一步。
        path_cost += distance
        # 控制代价惩罚“费力”的控制；这里乘 mpc_dt 近似连续时间积分。
        control_cost += norm(acceleration) ** 2 * guidance.mpc_dt
        # 平滑代价惩罚当前候选与上一控制差太多，减少加速度突变。
        smooth_cost += norm(acceleration - previous_acceleration) ** 2
        # PN 代价惩罚候选偏离 PN 趋势太远，使 NMPC 保持“PN 外环增强”的定位。
        pn_cost += norm(acceleration - pn_trend) ** 2
        previous_acceleration = acceleration

    final_distance = norm(target.position + target.velocity * guidance.horizon_steps * guidance.mpc_dt - state.position)
    # 权重决定 NMPC 更偏向“快速接近”还是“保持视场/控制平滑”。
    return (
        guidance.nmpc_w_dist * final_distance
        + guidance.nmpc_w_path * path_cost
        + guidance.nmpc_w_fov * fov_cost
        + guidance.nmpc_w_control * control_cost
        + guidance.nmpc_w_smooth * smooth_cost
        + guidance.nmpc_w_pn * pn_cost
    )
