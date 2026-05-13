from dataclasses import dataclass

import numpy as np

from .config import NMPCConfig
from .utils import (
    EPS,
    build_body_axes,
    clip_speed,
    clip_vector_norm,
    compute_fov_errors,
    safe_norm,
    unit_or_default,
)


@dataclass
class PNTrend:
    """
    PN 不再直接控制追踪者，而是把趋势信息交给 NMPC。

    reference_direction: PN 推出的参考飞行方向
    pn_acceleration: 原始 PN 加速度，用作 NMPC 候选控制和 fallback
    reference_speed: 当前速度策略给出的参考速度
    reference_intercept_point: 一个短时参考拦截点，帮助 NMPC 不只盯着当前目标位置
    distance/closing_speed/los_rate: 记录当前几何状态，便于日志和后续调参理解
    """
    reference_direction: np.ndarray
    pn_acceleration: np.ndarray
    reference_speed: float
    reference_intercept_point: np.ndarray
    distance: float
    closing_speed: float
    los_rate: float

@dataclass
class NMPCResult:
    """NMPC 单次求解结果，只把第一步控制量真正交给仿真执行。"""
    acceleration: np.ndarray
    predicted_tracker_traj: np.ndarray
    predicted_target_traj: np.ndarray
    fov_horizontal_error_deg: float
    fov_vertical_error_deg: float
    fov_violation: bool
    fallback: bool
    total_cost: float


class NMPCController:
    """
    轻量 NMPC 控制器。

    这里没有引入 CasADi/acados，而是枚举一组候选加速度，在预测窗内滚动仿真，
    选择代价最低的那一个。它适合初期验证算法链路：PN 给趋势，NMPC 负责预测和约束。

    这不是工业级优化器，而是“候选加速度枚举式 NMPC”：把几种可能的控制方向都试一遍，
    预测 horizon_steps 步后的距离、视场和控制代价，再取当前时刻的第一步控制量执行。
    """

    def __init__(self, config=None):
        self.config = config or NMPCConfig()

    def solve(self, tracker_pos, tracker_vel, target_pos, target_vel, pn_trend, dt):
        prediction_dt = self.config.prediction_dt or dt
        forward = unit_or_default(tracker_vel, pn_trend.reference_direction)

        # 每个候选加速度都用同一套预测模型向前滚动；代价越低，说明越接近“追上目标、
        # 保持目标在视场内、控制量不过大”的综合目标。
        best_cost = np.inf
        best_candidate = None
        candidates = self._build_candidates(
            tracker_pos, tracker_vel, target_pos, target_vel, pn_trend, prediction_dt
        )

        for acceleration in candidates:
            rollout = self._rollout(
                tracker_pos, tracker_vel, target_pos, target_vel,
                acceleration, pn_trend, prediction_dt
            )
            if np.isfinite(rollout["cost"]) and rollout["cost"] < best_cost:
                best_cost = rollout["cost"]
                best_candidate = (acceleration, rollout)

        if best_candidate is None:
            return self._fallback_result(
                tracker_pos, tracker_vel, target_pos, target_vel,
                pn_trend, prediction_dt, forward
            )

        acceleration, rollout = best_candidate
        horizontal_error, vertical_error, violation = compute_fov_errors(
            target_pos - tracker_pos,
            forward,
            self.config.fov_horizontal_deg,
            self.config.fov_vertical_deg,
        )

        return NMPCResult(
            acceleration=acceleration,
            predicted_tracker_traj=rollout["tracker_traj"],
            predicted_target_traj=rollout["target_traj"],
            fov_horizontal_error_deg=horizontal_error,
            fov_vertical_error_deg=vertical_error,
            fov_violation=violation,
            fallback=False,
            total_cost=float(best_cost),
        )

    def _build_candidates(self, tracker_pos, tracker_vel, target_pos, target_vel, pn_trend, dt):
        """生成有限个候选加速度，代替连续优化器里的控制变量搜索。"""
        los_vec = target_pos - tracker_pos
        los_dir = unit_or_default(los_vec, pn_trend.reference_direction)
        ref_dir = unit_or_default(pn_trend.reference_direction, los_dir)

        forward = unit_or_default(tracker_vel, ref_dir)
        right, up = build_body_axes(forward)

        # 候选方向围绕 PN 参考方向和当前视线方向展开：中间项负责追踪，两侧/上下偏置项
        # 让 NMPC 在目标快要出视场时有机会选择更合适的转向。
        desired_dirs = [
            ref_dir,
            los_dir,
            unit_or_default(ref_dir + 0.35 * right, ref_dir),
            unit_or_default(ref_dir - 0.35 * right, ref_dir),
            unit_or_default(ref_dir + 0.35 * up, ref_dir),
            unit_or_default(ref_dir - 0.35 * up, ref_dir),
            unit_or_default(los_dir + 0.25 * ref_dir, los_dir),
        ]

        candidates = [np.zeros(3), clip_vector_norm(pn_trend.pn_acceleration, self.config.accel_max)]

        for direction in desired_dirs:
            desired_velocity = direction * pn_trend.reference_speed
            accel_to_velocity = (desired_velocity - tracker_vel) / max(dt, EPS)
            candidates.append(clip_vector_norm(accel_to_velocity, self.config.accel_max))

        unique = []
        for candidate in candidates:
            if not any(safe_norm(candidate - existing) < 1e-6 for existing in unique):
                unique.append(candidate)
        return unique

    def _rollout(self, tracker_pos, tracker_vel, target_pos, target_vel, acceleration, pn_trend, dt):
        """用一个候选加速度预测未来轨迹，并计算该候选的综合代价。"""
        tracker_pos = np.asarray(tracker_pos, dtype=float).copy()
        tracker_vel = np.asarray(tracker_vel, dtype=float).copy()
        target_pos = np.asarray(target_pos, dtype=float).copy()
        target_vel = np.asarray(target_vel, dtype=float).copy()
        acceleration = clip_vector_norm(acceleration, self.config.accel_max)

        tracker_traj = []
        target_traj = []
        running_distance_cost = 0.0
        fov_cost = 0.0
        direction_cost = 0.0
        intercept_cost = 0.0

        for _ in range(max(1, self.config.horizon_steps)):
            tracker_vel = clip_speed(
                tracker_vel + acceleration * dt,
                self.config.speed_min,
                self.config.speed_max,
            )
            tracker_pos = tracker_pos + tracker_vel * dt
            target_pos = target_pos + target_vel * dt

            tracker_traj.append(tracker_pos.copy())
            target_traj.append(target_pos.copy())

            rel_vec = target_pos - tracker_pos
            running_distance_cost += safe_norm(rel_vec)

            forward = unit_or_default(tracker_vel, pn_trend.reference_direction)
            h_err, v_err, _ = compute_fov_errors(
                rel_vec,
                forward,
                self.config.fov_horizontal_deg,
                self.config.fov_vertical_deg,
            )
            # FOV 代价只惩罚超过半视场角的部分；目标还在视场内时不额外扣分。
            h_excess = max(abs(h_err) - self.config.fov_horizontal_deg * 0.5, 0.0)
            v_excess = max(abs(v_err) - self.config.fov_vertical_deg * 0.5, 0.0)
            fov_cost += h_excess * h_excess + v_excess * v_excess

            vel_dir = unit_or_default(tracker_vel, pn_trend.reference_direction)
            direction_error = 1.0 - float(np.clip(np.dot(vel_dir, pn_trend.reference_direction), -1.0, 1.0))
            direction_cost += direction_error

            intercept_cost += safe_norm(tracker_pos - pn_trend.reference_intercept_point)

        terminal_distance = safe_norm(target_pos - tracker_pos)
        control_cost = safe_norm(acceleration) ** 2

        total_cost = (
            self.config.terminal_distance_weight * terminal_distance
            + self.config.running_distance_weight * running_distance_cost
            + self.config.pn_direction_weight * direction_cost
            + self.config.intercept_point_weight * intercept_cost
            + self.config.fov_weight * fov_cost
            + self.config.control_weight * control_cost
        )

        return {
            "cost": float(total_cost),
            "tracker_traj": np.asarray(tracker_traj),
            "target_traj": np.asarray(target_traj),
        }

    def _fallback_result(self, tracker_pos, tracker_vel, target_pos, target_vel, pn_trend, dt, forward):
        """极端数值异常时退回 PN 加速度，保证仿真还能继续并在日志中暴露问题。"""
        acceleration = clip_vector_norm(pn_trend.pn_acceleration, self.config.accel_max)
        next_tracker_vel = clip_speed(
            tracker_vel + acceleration * dt,
            self.config.speed_min,
            self.config.speed_max,
        )
        next_tracker_pos = tracker_pos + next_tracker_vel * dt
        next_target_pos = target_pos + target_vel * dt
        horizontal_error, vertical_error, violation = compute_fov_errors(
            target_pos - tracker_pos,
            forward,
            self.config.fov_horizontal_deg,
            self.config.fov_vertical_deg,
        )
        return NMPCResult(
            acceleration=acceleration,
            predicted_tracker_traj=np.asarray([next_tracker_pos]),
            predicted_target_traj=np.asarray([next_target_pos]),
            fov_horizontal_error_deg=horizontal_error,
            fov_vertical_error_deg=vertical_error,
            fov_violation=violation,
            fallback=True,
            total_cost=np.inf,
        )
