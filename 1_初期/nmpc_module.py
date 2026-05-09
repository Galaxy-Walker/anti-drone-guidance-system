from dataclasses import dataclass

import numpy as np


# 这个模块是给“初期仿真脚本”使用的轻量 NMPC 外环。
# 它不依赖 ROS2/PX4，也不调用复杂优化器；核心思路是：
# 1. PN 先给出一个“应该往哪里飞”的趋势；
# 2. NMPC 在有限预测窗里枚举几种可行动作；
# 3. 用距离、FOV、PN方向一致性和控制能量组成代价函数；
# 4. 只执行代价最低候选的第一步加速度。
EPS = 1e-9


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


@dataclass
class PNTrend:
    """
    PN 不再直接控制追踪者，而是把趋势信息交给 NMPC。

    reference_direction: PN 推出的参考飞行方向
    pn_acceleration: 原始 PN 加速度，用作 NMPC 候选控制和 fallback
    reference_speed: 当前速度策略给出的参考速度
    reference_intercept_point: 一个短时参考拦截点，帮助 NMPC 不只盯着当前目标位置
    """
    # PN 给出的参考飞行方向，NMPC 会尽量让速度方向靠近它，但不会盲目照抄。
    reference_direction: np.ndarray
    # 原始 PN 计算出的加速度；既会作为候选控制量，也会作为求解失败时的保底输出。
    pn_acceleration: np.ndarray
    # 当前速度策略给出的期望速率，NMPC 构造候选速度时会使用它。
    reference_speed: float
    # 用当前目标速度短时外推得到的参考拦截点，帮助预测窗关注未来位置。
    reference_intercept_point: np.ndarray
    # 以下三个量主要用于代价函数、调试输出和后续扩展。
    distance: float
    closing_speed: float
    los_rate: float


@dataclass
class NMPCConfig:
    """NMPC 的所有可调参数集中放在这里，方便后续从仿真升级到 ROS2 配置。"""
    # 预测窗长度。数值越大看得越远，但枚举 rollout 的计算量也越大。
    horizon_steps: int = 12
    # 预测步长；None 表示跟随外部仿真 dt。
    prediction_dt: float | None = None
    # 追踪者加速度和速度约束，用来模拟飞行器能力边界。
    accel_max: float = 20.0
    speed_min: float = 20.0
    speed_max: float = 60.0
    # 相机/传感器视场角。这里按“总视场角”配置，判断时使用半角作为边界。
    fov_horizontal_deg: float = 60.0
    fov_vertical_deg: float = 45.0
    # 距离小于该半径时，外部仿真可以认为已经捕获目标。
    capture_radius: float = 1.0
    # 下面是代价函数权重：权重越大，NMPC 越重视对应目标。
    terminal_distance_weight: float = 8.0
    running_distance_weight: float = 0.08
    pn_direction_weight: float = 2.5
    intercept_point_weight: float = 0.02
    fov_weight: float = 80.0
    control_weight: float = 0.02


@dataclass
class NMPCResult:
    """NMPC 单次求解结果，只把第一步控制量真正交给仿真执行。"""
    # 本次真正要执行的加速度命令。
    acceleration: np.ndarray
    # 预测轨迹只用于绘图/诊断，不会整段下发给追踪者。
    predicted_tracker_traj: np.ndarray
    predicted_target_traj: np.ndarray
    # 当前时刻的 FOV 误差与违规状态，用于评价目标是否仍在视场内。
    fov_horizontal_error_deg: float
    fov_vertical_error_deg: float
    fov_violation: bool
    # True 表示没有找到有限代价候选，已退回 PN 加速度。
    fallback: bool
    total_cost: float


class NMPCController:
    """
    轻量 NMPC 控制器。

    这里没有引入 CasADi/acados，而是枚举一组候选加速度，在预测窗内滚动仿真，
    选择代价最低的那一个。它适合初期验证算法链路：PN 给趋势，NMPC 负责预测和约束。
    """

    def __init__(self, config=None):
        self.config = config or NMPCConfig()

    def solve(self, tracker_pos, tracker_vel, target_pos, target_vel, pn_trend, dt):
        # prediction_dt 是 NMPC 自己用于“想象未来”的步长；默认与仿真步长一致。
        prediction_dt = self.config.prediction_dt or dt
        # 初期模型没有机体姿态/相机姿态，因此用速度方向近似传感器前向。
        # 若速度太小，则退回 PN 方向，避免零速度导致角度不可定义。
        forward = unit_or_default(tracker_vel, pn_trend.reference_direction)

        best_cost = np.inf
        best_candidate = None
        # 这里不是连续优化，而是生成有限个有物理含义的候选加速度：
        # 保持、PN原始加速度、朝 LOS/PN 方向以及上下左右偏置的动作。
        candidates = self._build_candidates(
            tracker_pos, tracker_vel, target_pos, target_vel, pn_trend, prediction_dt
        )

        for acceleration in candidates:
            # 对每个候选做一次预测窗 rollout，得到“如果一直使用该加速度会怎样”。
            rollout = self._rollout(
                tracker_pos, tracker_vel, target_pos, target_vel,
                acceleration, pn_trend, prediction_dt
            )
            if np.isfinite(rollout["cost"]) and rollout["cost"] < best_cost:
                best_cost = rollout["cost"]
                best_candidate = (acceleration, rollout)

        if best_candidate is None:
            # 理论上候选集合通常不会为空；fallback 是为了防止异常参数或数值问题
            # 让仿真直接崩溃。保底策略仍然使用 PN 计算出的加速度。
            return self._fallback_result(
                tracker_pos, tracker_vel, target_pos, target_vel,
                pn_trend, prediction_dt, forward
            )

        acceleration, rollout = best_candidate
        # 注意：这里记录的是“当前时刻”的 FOV 误差，而不是预测窗末端误差。
        # 预测窗中的 FOV 表现已经通过代价函数影响了候选选择。
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
        # LOS(Line Of Sight) 是从追踪者指向目标的相对位置方向。
        # ref_dir 是 PN 提供的趋势方向；两者不一定完全相同。
        los_vec = target_pos - tracker_pos
        los_dir = unit_or_default(los_vec, pn_trend.reference_direction)
        ref_dir = unit_or_default(pn_trend.reference_direction, los_dir)

        # 用当前前向构造局部右/上方向，方便生成“稍微向左/右/上/下修正”的候选。
        forward = unit_or_default(tracker_vel, ref_dir)
        right, up = build_body_axes(forward)

        # desired_dirs 是候选“速度方向”，随后会转换成“达到该速度所需的加速度”。
        # 加入偏置方向的目的，是让 NMPC 有机会为了 FOV 或未来拦截点进行小幅绕飞。
        desired_dirs = [
            ref_dir,
            los_dir,
            unit_or_default(ref_dir + 0.35 * right, ref_dir),
            unit_or_default(ref_dir - 0.35 * right, ref_dir),
            unit_or_default(ref_dir + 0.35 * up, ref_dir),
            unit_or_default(ref_dir - 0.35 * up, ref_dir),
            unit_or_default(los_dir + 0.25 * ref_dir, los_dir),
        ]

        # 零加速度表示继续当前运动；PN加速度表示完全相信原始比例导引。
        # 这两个候选是很重要的基准动作。
        candidates = [np.zeros(3), clip_vector_norm(pn_trend.pn_acceleration, self.config.accel_max)]

        for direction in desired_dirs:
            # 把“我希望朝 direction 以 reference_speed 飞”转换为一步内需要的加速度。
            # 之后再按 accel_max 裁剪，保证候选动作不超过机动能力。
            desired_velocity = direction * pn_trend.reference_speed
            accel_to_velocity = (desired_velocity - tracker_vel) / max(dt, EPS)
            candidates.append(clip_vector_norm(accel_to_velocity, self.config.accel_max))

        # 去掉几乎重复的候选，避免浪费预测计算。
        unique = []
        for candidate in candidates:
            if not any(safe_norm(candidate - existing) < 1e-6 for existing in unique):
                unique.append(candidate)
        return unique

    def _rollout(self, tracker_pos, tracker_vel, target_pos, target_vel, acceleration, pn_trend, dt):
        # 复制输入状态，保证预测过程不会提前改动外部真实仿真状态。
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
            # 追踪者采用简化双积分模型：
            #   v_{k+1} = sat(v_k + a * dt)
            #   p_{k+1} = p_k + v_{k+1} * dt
            # 这里的 sat 同时限制最小/最大速度，模拟速度包线。
            tracker_vel = clip_speed(
                tracker_vel + acceleration * dt,
                self.config.speed_min,
                self.config.speed_max,
            )
            tracker_pos = tracker_pos + tracker_vel * dt
            # 目标在预测窗内按当前速度匀速外推；真实仿真下一步仍可能重新生成机动/噪声。
            target_pos = target_pos + target_vel * dt

            tracker_traj.append(tracker_pos.copy())
            target_traj.append(target_pos.copy())

            rel_vec = target_pos - tracker_pos
            distance = safe_norm(rel_vec)
            # running_distance_cost 鼓励整个预测过程中都靠近目标，而不只是末端靠近。
            running_distance_cost += distance

            forward = unit_or_default(tracker_vel, pn_trend.reference_direction)
            h_err, v_err, _ = compute_fov_errors(
                rel_vec,
                forward,
                self.config.fov_horizontal_deg,
                self.config.fov_vertical_deg,
            )
            # FOV 采用软约束：在半视场角以内不惩罚，超出后按超出角度平方惩罚。
            # 这样 NMPC 仍能在必要时短暂违反视场，但会倾向于尽快把目标拉回视场。
            h_excess = max(abs(h_err) - self.config.fov_horizontal_deg * 0.5, 0.0)
            v_excess = max(abs(v_err) - self.config.fov_vertical_deg * 0.5, 0.0)
            fov_cost += h_excess * h_excess + v_excess * v_excess

            vel_dir = unit_or_default(tracker_vel, pn_trend.reference_direction)
            # direction_error = 1 - cos(theta)，方向完全一致时为 0，偏离越大代价越高。
            direction_error = 1.0 - float(np.clip(np.dot(vel_dir, pn_trend.reference_direction), -1.0, 1.0))
            direction_cost += direction_error

            # 参考拦截点代价让追踪者不要只追当前目标点，而是兼顾目标短时未来位置。
            intercept_cost += safe_norm(tracker_pos - pn_trend.reference_intercept_point)

        terminal_distance = safe_norm(target_pos - tracker_pos)
        control_cost = safe_norm(acceleration) ** 2

        # 总代价是多个目标的加权和。调参时应先明确“更重视命中、FOV还是平滑控制”，
        # 再调整对应权重，而不是同时大幅改动所有参数。
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
        # fallback 仍走 PN 加速度，但继续套用加速度/速度限制，保证输出和正常 NMPC 一致。
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


def build_body_axes(forward):
    """根据前向向量构造右向和上向，用于把目标方向分解成水平/垂直角误差。"""
    forward = unit_or_default(forward, np.array([1.0, 0.0, 0.0]))
    world_up = np.array([0.0, 0.0, 1.0])
    # 当前向几乎竖直时，不能再用世界 Z 轴做参考上方向，否则叉乘会接近零。
    # 这时临时改用世界 Y 轴，保证 right/up 坐标轴稳定。
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
    # relative_dir 是目标相对追踪者的方向；forward 是相机/机体前向近似。
    relative_dir = unit_or_default(relative_vector, forward_vector)
    forward = unit_or_default(forward_vector, relative_dir)
    right, up = build_body_axes(forward)

    # 把目标方向投影到 forward/right/up 三个轴上，再用 atan2 得到角误差。
    # horizontal_error > 0 表示目标偏向 right 方向；
    # vertical_error > 0 表示目标偏向 up 方向。
    forward_component = float(np.dot(relative_dir, forward))
    right_component = float(np.dot(relative_dir, right))
    up_component = float(np.dot(relative_dir, up))

    horizontal_error = np.degrees(np.arctan2(right_component, forward_component))
    vertical_error = np.degrees(
        np.arctan2(up_component, max(np.hypot(forward_component, right_component), EPS))
    )
    # 配置值是完整视场角，所以允许范围是 +/- 半视场角。
    violation = (
        abs(horizontal_error) > horizontal_fov_deg * 0.5
        or abs(vertical_error) > vertical_fov_deg * 0.5
    )
    return float(horizontal_error), float(vertical_error), bool(violation)
