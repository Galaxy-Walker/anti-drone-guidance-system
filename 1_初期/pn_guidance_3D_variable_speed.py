import logging
from pathlib import Path
from typing import cast

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.axes import Axes
try:
    from mpl_toolkits.mplot3d.axes3d import Axes3D
except Exception:
    # 有些 Python/Matplotlib 环境缺少 3D projection 依赖。
    # 日志和仿真不应该因此无法运行；plot_results() 里会自动退回 2D 图。
    Axes3D = object

from nmpc_module import (
    NMPCConfig,
    NMPCController,
    PNTrend,
    compute_fov_errors,
    safe_norm,
    unit_or_default,
)


DEFAULT_RANDOM_SEED = 42


def _format_vec(vec):
    """把向量压成短字符串，避免日志里一长串 numpy 默认格式难读。"""
    return np.array2string(np.asarray(vec, dtype=float), precision=3, suppress_small=True)


# 本脚本是一个独立的 3D 拦截仿真：
# - 目标机按 update_target_state() 中的简单机动模型运动；
# - 追踪者先用比例导引 PN 计算趋势；
# - 当 use_nmpc=True 时，NMPC 外环在预测窗内选择更合适的第一步加速度；
# - 最后把轨迹、速度、距离和 FOV 指标画出来，方便与纯 PN 做对照。
class ProportionalNavigation3D_VariableSpeed:
    def __init__(self, t_pos, m_pos, t_vel, m_speed_init, N=3.0, dt=0.01,
                 speed_min=2, speed_max=30, speed_strategy='adaptive',
                 use_nmpc=True, nmpc_config=None, random_seed=DEFAULT_RANDOM_SEED,
                 enable_logging=True, log_level=logging.INFO, log_interval=1.0,
                 log_to_file=None):
        """
        初始化3D仿真参数（支持可变速度）
        :param t_pos: 目标初始位置 [x, y, z]
        :param m_pos: 追踪无人机初始位置 [x, y, z]
        :param t_vel: 目标速度向量 [vx, vy, vz]
        :param m_speed_init: 追踪无人机初始速率 (标量)
        :param N: 比例导引系数 (通常在 3-5 之间)
        :param dt: 仿真时间步长
        :param speed_min: 最小速度限制
        :param speed_max: 最大速度限制
        :param speed_strategy: 速度调节策略
                              'adaptive' - 自适应速度控制
                              'distance' - 基于距离的速度控制
                              'energy' - 能量优化策略
                              'pursuit' - 追击优化策略
        :param use_nmpc: True 时使用 PN趋势 + NMPC；False 时保留原始 PN 控制路径
        :param nmpc_config: 可选 NMPCConfig，用于覆盖默认预测窗/FOV/代价权重
        :param random_seed: 目标噪声随机种子；默认固定为 42，保证每次运行噪声一致
        :param enable_logging: 是否开启诊断日志
        :param log_level: 日志等级；调试追踪策略时建议使用 logging.DEBUG
        :param log_interval: 常规状态日志的输出间隔，单位秒；异常告警不受该间隔限制
        :param log_to_file: 可选日志文件路径；None 表示只输出到控制台
        """
        self.t_pos = np.array(t_pos, dtype=float)
        self.m_pos = np.array(m_pos, dtype=float)
        self.t_vel = np.array(t_vel, dtype=float)
        self.m_speed = m_speed_init
        self.m_speed_init = m_speed_init
        self.N = N
        self.dt = dt

        # 速度限制参数。这里限制的是追踪者“速率标量”，方向由 PN/NMPC 计算得到。
        self.speed_min = speed_min
        self.speed_max = speed_max
        self.speed_strategy = speed_strategy
        self.use_nmpc = use_nmpc
        self.enable_logging = enable_logging
        self.log_interval = max(float(log_interval), self.dt)
        self._next_log_time = 0.0
        self._step_index = 0
        self._last_distance = None
        self._last_speed_debug = {}
        self._last_speed_update_debug = {}
        # 给每个仿真实例单独保存随机数生成器，避免外部代码调用 np.random 后改变本脚本噪声。
        self.rng = np.random.default_rng(random_seed)
        self.logger = self._setup_logger(log_level, log_to_file)

        if nmpc_config is None:
            # 默认 NMPC 参数与本仿真的 dt、速度范围保持一致。
            # horizon_steps * prediction_dt 决定 NMPC 能“看多远”的未来。
            nmpc_config = NMPCConfig(
                horizon_steps=12,
                prediction_dt=dt,
                accel_max=20.0,
                speed_min=speed_min,
                speed_max=speed_max,
                fov_horizontal_deg=60.0,
                fov_vertical_deg=45.0,
                capture_radius=1.0,
            )
        elif nmpc_config.prediction_dt is None:
            # 如果外部只传了部分 NMPC 参数，预测步长默认沿用仿真步长。
            nmpc_config.prediction_dt = dt
        self.nmpc_config = nmpc_config
        self.nmpc_controller = NMPCController(self.nmpc_config)

        # 初始化追踪者的速度向量（假设初始指向目标）
        diff = self.t_pos - self.m_pos
        dist = np.linalg.norm(diff)
        if dist > 0:
            self.m_vel = diff / dist * m_speed_init
        else:
            # 如果目标和追踪者初始重合，方向无法从相对位置得到，临时给一个 X 方向。
            self.m_vel = np.array([m_speed_init, 0, 0], dtype=float)

        # 用于存储轨迹数据以便绘图。每个列表的第 0 个元素对应初始时刻。
        self.t_traj = [self.t_pos.copy()]
        self.m_traj = [self.m_pos.copy()]
        self.speed_history = [m_speed_init]  # 记录速度变化历史
        self.time_history = [0]              # 记录时间历史
        self.distance_history = [dist]       # 记录距离历史
        self._last_distance = float(dist)

        # NMPC/FOV 指标：初期仿真没有姿态状态，所以默认用速度方向代表相机中心线。
        # 当速度为 0 时，临时用“追踪者指向目标”的方向，避免初始时刻无法计算视场角。
        initial_forward = unit_or_default(self.m_vel, diff)
        h_err, v_err, violation = compute_fov_errors(
            diff,
            initial_forward,
            self.nmpc_config.fov_horizontal_deg,
            self.nmpc_config.fov_vertical_deg,
        )
        self.fov_horizontal_history = [h_err]
        self.fov_vertical_history = [v_err]
        self.fov_violation_history = [violation]
        # cost/fallback 只在 NMPC 模式下有明确意义；初始时刻先填充占位值。
        self.nmpc_cost_history = [np.nan]
        self.fallback_history = [False]
        # 保存 NMPC 每一步的预测轨迹，后续若要扩展可视化可以直接使用。
        self.nmpc_predicted_tracker_traj = []
        self.nmpc_predicted_target_traj = []

        self._log_initial_state(random_seed)

    def _setup_logger(self, log_level, log_to_file):
        """给每个仿真实例创建独立 logger，避免 compare_strategies() 多实例时重复打印。"""
        logger = logging.getLogger(f"{__name__}.sim.{id(self)}")
        logger.handlers.clear()
        logger.propagate = False

        if isinstance(log_level, str):
            resolved_level = getattr(logging, log_level.upper(), logging.INFO)
        else:
            resolved_level = int(log_level)
        logger.setLevel(resolved_level)

        if not self.enable_logging:
            logger.addHandler(logging.NullHandler())
            return logger

        formatter = logging.Formatter(
            fmt="%(asctime)s | %(levelname)-7s | %(message)s",
            datefmt="%H:%M:%S",
        )
        stream_handler = logging.StreamHandler()
        stream_handler.setLevel(resolved_level)
        stream_handler.setFormatter(formatter)
        logger.addHandler(stream_handler)

        if log_to_file:
            file_handler = logging.FileHandler(log_to_file, mode="w", encoding="utf-8")
            file_handler.setLevel(resolved_level)
            file_handler.setFormatter(formatter)
            logger.addHandler(file_handler)

        return logger

    def _should_log_step(self, time):
        """常规 DEBUG 日志按时间间隔输出，避免 dt 很小时刷屏到无法阅读。"""
        if not self.enable_logging or not self.logger.isEnabledFor(logging.DEBUG):
            return False
        if time + 1e-12 < self._next_log_time:
            return False
        self._next_log_time = time + self.log_interval
        return True

    def _log_initial_state(self, random_seed):
        """启动时输出一次完整配置，后续排查时可以确认实验条件是否一致。"""
        self.logger.info(
            "初始化仿真: mode=%s, strategy=%s, N=%.3f, dt=%.4f, speed=[%.2f, %.2f], "
            "capture_radius=%.2f, seed=%s",
            "PN+NMPC" if self.use_nmpc else "Pure PN",
            self.speed_strategy,
            self.N,
            self.dt,
            self.speed_min,
            self.speed_max,
            self.nmpc_config.capture_radius,
            random_seed,
        )
        self.logger.info(
            "初始状态: target_pos=%s, tracker_pos=%s, target_vel=%s, tracker_vel=%s, "
            "distance=%.3f, tracker_speed=%.3f",
            _format_vec(self.t_pos),
            _format_vec(self.m_pos),
            _format_vec(self.t_vel),
            _format_vec(self.m_vel),
            self.distance_history[0],
            self.m_speed,
        )

    # TODO(Refactor): 可拆分为“轨迹计算/追踪模式选择模块”的速度策略接口
    def compute_desired_speed(self, R_mag, Vc, Omega_mag, target_speed):
        """
        根据不同策略计算期望速度
        :param R_mag: 当前距离
        :param Vc: 接近速度，正值表示距离正在缩短
        :param Omega_mag: 视线角速率大小，越大说明相对几何变化越剧烈
        :param target_speed: 目标速度大小
        :return: 期望速度
        """
        if self.speed_strategy == 'adaptive':
            # 自适应速度控制策略
            # 基于距离生成基础速率，而不是使用 m_speed_init 作为乘法基准。
            # 这样即使追踪者从静止启动，期望速率也不会被永久算成 0。
            dist_speed = np.interp(
                np.clip(R_mag, 0.0, 250.0),
                [0.0, 250.0],
                [self.speed_min, self.speed_max],
            )

            # 接近速度补偿：远离时强制加速，接近速度偏小时轻度加速。
            if Vc <= 0.0:
                closing_boost = 1.35
            elif Vc < 0.35 * max(dist_speed, self.speed_min):
                closing_boost = 1.15
            else:
                closing_boost = 1.0

            # 角速率惩罚系数（角速率越大，速度适当降低以保留机动余量）。
            omega_penalty = np.clip(1.0 - Omega_mag * 3.0, 0.75, 1.0)

            # 速度下限既要满足系统最小速度，也要略高于目标速度，保证追得上。
            speed_floor = max(self.speed_min, 0.8 * self.m_speed, 1.1 * target_speed)

            desired_speed = max(dist_speed * closing_boost * omega_penalty, speed_floor)
            debug_factors = {
                "dist_speed": float(dist_speed),
                "closing_boost": float(closing_boost),
                "omega_penalty": float(omega_penalty),
                "speed_floor": float(speed_floor),
            }

        elif self.speed_strategy == 'distance':
            # 基于距离的速度控制
            # 远距离高速，近距离降速以提高拦截精度
            if R_mag > 300:
                desired_speed = self.speed_max
            elif R_mag > 100:
                # 线性插值
                ratio = (R_mag - 100) / 200
                desired_speed = self.speed_min + ratio * (self.speed_max - self.speed_min)
            else:
                # 近距离保持较低速度
                desired_speed = self.speed_min + (R_mag / 100) * (self.m_speed_init - self.speed_min)
            debug_factors = {}

        elif self.speed_strategy == 'energy':
            # 能量优化策略
            # 在保证拦截的前提下尽量节省能量
            # 当预计能够拦截时降低速度

            # 估算拦截时间
            if Vc > 0:
                estimated_intercept_time = R_mag / Vc

                # 如果拦截时间充裕，可以降低速度
                if estimated_intercept_time > 5.0:
                    desired_speed = max(self.speed_min, target_speed * 1.2)
                elif estimated_intercept_time > 2.0:
                    desired_speed = self.m_speed_init
                else:
                    # 接近拦截点时加速确保成功
                    desired_speed = self.speed_max
            else:
                # 如果没有接近，全速追击
                desired_speed = self.speed_max
            debug_factors = {}

        elif self.speed_strategy == 'pursuit':
            # 追击优化策略
            # 根据目标速度和相对几何关系调整

            # 始终保持对目标的速度优势
            speed_advantage_ratio = 1.5
            base_speed = target_speed * speed_advantage_ratio

            # 根据视线角速率微调
            if Omega_mag > 0.1:
                # 目标机动剧烈时，适当加速
                desired_speed = base_speed * 1.2
            else:
                desired_speed = base_speed
            debug_factors = {
                "base_speed": float(base_speed),
                "speed_advantage_ratio": speed_advantage_ratio,
            }

        else:
            # 默认使用初始速度
            desired_speed = self.m_speed_init
            debug_factors = {"unknown_strategy": self.speed_strategy}

        # 应用速度限制
        clipped_speed = float(np.clip(desired_speed, self.speed_min, self.speed_max))
        self._last_speed_debug = {
            "strategy": self.speed_strategy,
            "raw_desired_speed": float(desired_speed),
            "clipped_desired_speed": clipped_speed,
            "R_mag": float(R_mag),
            "Vc": float(Vc),
            "Omega_mag": float(Omega_mag),
            "target_speed": float(target_speed),
            **debug_factors,
        }
        return clipped_speed

    def update_speed(self, desired_speed, acceleration_limit=10.0):
        """
        平滑更新速度（避免突变）
        :param desired_speed: 期望速度
        :param acceleration_limit: 加速度限制 (m/s^2)
        :return: 实际更新后的速度
        """
        speed_diff = desired_speed - self.m_speed
        # 单步最大速度变化 = 最大加速度 * dt，防止速度标量突然跳变。
        max_speed_change = acceleration_limit * self.dt

        # 限制速度变化率
        if abs(speed_diff) > max_speed_change:
            speed_diff = np.sign(speed_diff) * max_speed_change

        self.m_speed += speed_diff
        self._last_speed_update_debug = {
            "desired_speed": float(desired_speed),
            "actual_speed": float(self.m_speed),
            "speed_diff_applied": float(speed_diff),
            "max_speed_change": float(max_speed_change),
            "acceleration_limit": float(acceleration_limit),
        }
        return self.m_speed

    # TODO(Refactor): 可拆分为“位置获取模块”（仿真位置源）
    def update_target_state(self, time):
        """
        更新目标状态：3D机动
        """
        # 目标做3D螺旋机动，并加入随机噪声
        noise = self.rng.normal(0, 5, 3)     # 均值0，标准差5；由 random_seed 保证可复现

        # 简单的3D机动策略
        # X轴: 匀速
        # Y轴: 正弦机动
        # Z轴: 余弦机动 (形成螺旋)
        # 注意：这里每一步都会重写 t_vel，因此 t_vel 参数只作为初始值/占位。
        # 如果后续要接入真实目标状态，应优先替换这个函数。
        self.t_vel[0] = -10 + noise[0] * 0.1
        self.t_vel[1] = -10 * abs(np.sin(0.5 * time)) + noise[1]
        self.t_vel[2] = -5 * np.sin(0.5 * time) + noise[2]

        # 用欧拉积分更新目标位置：p_{k+1} = p_k + v_k * dt。
        self.t_pos += self.t_vel * self.dt
        self.t_traj.append(self.t_pos.copy())

    # TODO(Refactor): 可拆分为“轨迹/追踪计算模块”（PNG 核心）
    def _build_pn_trend(self, R_vec, R_mag, Vc, Omega_mag, ac_vec):
        """
        把原来的 PN 控制量改造成“趋势信息”。

        注意：这里仍然计算 PN 加速度，但它不再直接决定最终控制量；
        NMPC 会把它当成参考方向、候选控制和失败时的 fallback。
        """
        # R_unit 是当前视线方向。若 PN 加速度很小，参考方向会退回到视线方向。
        R_unit = R_vec / max(R_mag, 1e-9)
        # PN 加速度作用一个 dt 后得到一个“速度提示”，再归一化成参考方向。
        pn_velocity_hint = self.m_vel + ac_vec * self.dt
        reference_direction = unit_or_default(pn_velocity_hint, R_unit)

        # 参考拦截点只做短时估计，不假设知道目标未来噪声。
        # Vc 太小或为负时，改用当前速度上限估算一个保守时间。
        target_speed = safe_norm(self.t_vel)
        # fallback_relative_speed 防止 Vc <= 0 时出现负时间或除零。
        fallback_relative_speed = max(self.m_speed + target_speed, self.speed_min, 1.0)
        intercept_time = R_mag / max(Vc, fallback_relative_speed)
        horizon_time = self.nmpc_config.horizon_steps * (self.nmpc_config.prediction_dt or self.dt)
        # 拦截点参考只限制在 NMPC 预测窗内，避免很远未来的粗糙预测主导当前控制。
        intercept_time = float(np.clip(intercept_time, self.dt, max(horizon_time, self.dt)))
        reference_intercept_point = self.t_pos + self.t_vel * intercept_time

        return PNTrend(
            reference_direction=reference_direction,
            pn_acceleration=ac_vec,
            reference_speed=float(self.m_speed),
            reference_intercept_point=reference_intercept_point,
            distance=float(R_mag),
            closing_speed=float(Vc),
            los_rate=float(Omega_mag),
        )

    def _record_guidance_metrics(self, time, R_mag, nmpc_result, pn_trend):
        """统一记录 PN-only 和 PN+NMPC 两种模式下的可视化指标。"""
        if nmpc_result is None:
            # 纯 PN 模式没有 NMPC 结果，但仍然计算 FOV 指标，便于和 PN+NMPC 公平对比。
            forward = unit_or_default(self.m_vel, pn_trend.reference_direction)
            h_err, v_err, violation = compute_fov_errors(
                self.t_pos - self.m_pos,
                forward,
                self.nmpc_config.fov_horizontal_deg,
                self.nmpc_config.fov_vertical_deg,
            )
            cost = np.nan
            fallback = False
        else:
            # NMPC 模式直接使用求解器返回的 FOV、代价和 fallback 标记。
            h_err = nmpc_result.fov_horizontal_error_deg
            v_err = nmpc_result.fov_vertical_error_deg
            violation = nmpc_result.fov_violation
            cost = nmpc_result.total_cost
            fallback = nmpc_result.fallback
            self.nmpc_predicted_tracker_traj.append(nmpc_result.predicted_tracker_traj)
            self.nmpc_predicted_target_traj.append(nmpc_result.predicted_target_traj)

        # 所有历史量在同一处追加，避免不同控制模式下列表长度不一致。
        self.speed_history.append(self.m_speed)
        self.time_history.append(time)
        self.distance_history.append(R_mag)
        self.fov_horizontal_history.append(h_err)
        self.fov_vertical_history.append(v_err)
        self.fov_violation_history.append(violation)
        self.nmpc_cost_history.append(cost)
        self.fallback_history.append(fallback)

    def _log_guidance_step(
        self,
        time,
        R_vec,
        V_rel,
        R_mag,
        distance_after,
        Vc,
        Omega,
        ac_vec,
        control_acc,
        current_speed_before_norm,
        nmpc_result,
        pn_trend,
    ):
        """输出单步制导诊断信息：相对几何、速度策略、控制量和约束执行结果。"""
        distance_delta = 0.0 if self._last_distance is None else R_mag - self._last_distance
        los_dir = unit_or_default(R_vec, pn_trend.reference_direction)
        tracker_forward = unit_or_default(self.m_vel, pn_trend.reference_direction)
        alignment = float(np.dot(tracker_forward, los_dir))
        nmpc_cost = np.nan if nmpc_result is None else nmpc_result.total_cost
        nmpc_fallback = False if nmpc_result is None else nmpc_result.fallback
        nmpc_fov = None if nmpc_result is None else (
            nmpc_result.fov_horizontal_error_deg,
            nmpc_result.fov_vertical_error_deg,
            nmpc_result.fov_violation,
        )

        if Vc <= 0.0:
            self.logger.warning(
                "t=%.2fs step=%d 接近速度 Vc=%.3f <= 0，当前几何正在远离或横向绕飞；"
                "R=%.3f, target_vel=%s, tracker_vel=%s",
                time,
                self._step_index,
                Vc,
                R_mag,
                _format_vec(self.t_vel),
                _format_vec(self.m_vel),
            )
        if distance_delta > max(0.1, self.m_speed * self.dt * 0.25):
            self.logger.warning(
                "t=%.2fs step=%d 距离比上一采样增加 %.3f m: prev=%.3f, current=%.3f, after_update=%.3f",
                time,
                self._step_index,
                distance_delta,
                self._last_distance,
                R_mag,
                distance_after,
            )
        if nmpc_fallback:
            self.logger.warning(
                "t=%.2fs step=%d NMPC fallback，已退回 PN 加速度: pn_acc=%s",
                time,
                self._step_index,
                _format_vec(ac_vec),
            )

        self.logger.debug(
            "\n[GUIDANCE] t=%.2fs step=%d mode=%s"
            "\n  relative: R=%.3f -> after=%.3f, dR_from_prev=%+.3f, Vc=%.3f, |Omega|=%.6f, alignment=%.3f"
            "\n  target:   pos=%s, vel=%s, speed=%.3f"
            "\n  tracker:  pos=%s, vel=%s, speed_cmd=%.3f, speed_vec_before_clip=%.3f"
            "\n  speed_strategy: %s"
            "\n  speed_update:   %s"
            "\n  PN:       acc=%s, |acc|=%.3f, ref_dir=%s, intercept_ref=%s"
            "\n  control:  acc=%s, |acc|=%.3f, nmpc_cost=%.3f, nmpc_fallback=%s, nmpc_fov=%s"
            "\n  V_rel=%s, Omega=%s",
            time,
            self._step_index,
            "PN+NMPC" if self.use_nmpc else "Pure PN",
            R_mag,
            distance_after,
            distance_delta,
            Vc,
            safe_norm(Omega),
            alignment,
            _format_vec(self.t_pos),
            _format_vec(self.t_vel),
            safe_norm(self.t_vel),
            _format_vec(self.m_pos),
            _format_vec(self.m_vel),
            self.m_speed,
            current_speed_before_norm,
            self._last_speed_debug,
            self._last_speed_update_debug,
            _format_vec(ac_vec),
            safe_norm(ac_vec),
            _format_vec(pn_trend.reference_direction),
            _format_vec(pn_trend.reference_intercept_point),
            _format_vec(control_acc),
            safe_norm(control_acc),
            nmpc_cost,
            nmpc_fallback,
            nmpc_fov,
            _format_vec(V_rel),
            _format_vec(Omega),
        )

    def update_missile_state(self, time):
        """
        核心：使用 PN 生成趋势，再由 NMPC 选择最终控制量。

        use_nmpc=False 时，会走原来的 PN 直接控制逻辑，方便做对照实验。
        """
        # 1. 计算相对位置和相对速度
        R_vec = self.t_pos - self.m_pos       # 相对位置向量 (Line of Sight Vector)
        V_rel = self.t_vel - self.m_vel       # 相对速度向量

        R_mag = np.linalg.norm(R_vec)         # 距离
        R_sq = np.dot(R_vec, R_vec)           # 距离平方

        # 如果距离非常近，视为命中
        if R_mag < self.nmpc_config.capture_radius:
            self.logger.info(
                "命中判定: t=%.2fs step=%d distance=%.3f < capture_radius=%.3f",
                time,
                self._step_index,
                R_mag,
                self.nmpc_config.capture_radius,
            )
            return True # Hit

        # 2. 计算视线旋转向量 (Rotation Vector of LOS)
        # Omega = (R x V) / R^2
        # R_sq 很小时前面已经命中返回，因此这里不会除以接近 0 的距离平方。
        Omega = np.cross(R_vec, V_rel) / R_sq
        Omega_mag = np.linalg.norm(Omega)

        # 3. 计算接近速度 (Closing Velocity, Vc)
        # Vc = - (R . V) / |R|
        Vc = -np.dot(R_vec, V_rel) / R_mag

        # 4. 计算目标速度大小
        target_speed = np.linalg.norm(self.t_vel)

        if not np.all(np.isfinite([R_mag, R_sq, Vc, target_speed])):
            self.logger.error(
                "数值异常: t=%.2fs step=%d R_mag=%s R_sq=%s Vc=%s target_speed=%s R_vec=%s V_rel=%s",
                time,
                self._step_index,
                R_mag,
                R_sq,
                Vc,
                target_speed,
                _format_vec(R_vec),
                _format_vec(V_rel),
            )

        # 5. 【新增】根据策略计算期望速度并更新
        # 这一步只更新速度标量 self.m_speed；方向仍由 PN/NMPC 的加速度控制。
        desired_speed = self.compute_desired_speed(R_mag, Vc, Omega_mag, target_speed)
        self.update_speed(desired_speed)

        # 6. 计算期望的加速度向量 (Acceleration Command)
        # 3D PNG 公式: ac = N * Vc * (Omega x R_unit)
        # 直观理解：视线旋转越快、接近速度越大，需要的横向修正加速度越大。
        R_unit = R_vec / R_mag
        ac_vec = self.N * Vc * np.cross(Omega, R_unit)

        # 7. PN 只提供趋势，是否交给 NMPC 由 use_nmpc 控制。
        pn_trend = self._build_pn_trend(R_vec, R_mag, Vc, Omega_mag, ac_vec)
        nmpc_result = None
        control_acc = ac_vec
        if self.use_nmpc:
            # NMPC 看到的是当前追踪者/目标状态和 PNTrend。
            # 它会预测未来若干步，但只返回第一步要执行的 acceleration。
            nmpc_result = self.nmpc_controller.solve(
                tracker_pos=self.m_pos,
                tracker_vel=self.m_vel,
                target_pos=self.t_pos,
                target_vel=self.t_vel,
                pn_trend=pn_trend,
                dt=self.dt,
            )
            control_acc = nmpc_result.acceleration

        # 8. 更新追踪者的速度向量。真正执行的只有 NMPC 选择出的第一步控制量。
        # 这里仍是简单欧拉积分：v_{k+1} = v_k + a * dt。
        self.m_vel += control_acc * self.dt

        # 9. 速度约束 (使用当前计算的可变速度)
        current_speed = np.linalg.norm(self.m_vel)
        if current_speed > 1e-9:
            velocity_dir = self.m_vel / current_speed
            alignment_to_los = float(np.dot(velocity_dir, R_unit))
            min_useful_speed = max(0.5, 0.25 * max(self.m_speed, self.speed_min))
            if current_speed < min_useful_speed or alignment_to_los < 0.0:
                # 纯 PN 的加速度主要是横向修正；从静止起步时需要先建立指向目标的前向速度。
                velocity_dir = unit_or_default(0.65 * R_unit + 0.35 * velocity_dir, R_unit)
            self.m_vel = velocity_dir * self.m_speed
        else:
            self.m_vel = R_unit * self.m_speed

        # 10. 更新位置
        # p_{k+1} = p_k + v_{k+1} * dt；本脚本没有模拟姿态/低层 PID。
        self.m_pos += self.m_vel * self.dt
        self.m_traj.append(self.m_pos.copy())
        distance_after_update = safe_norm(self.t_pos - self.m_pos)

        # 11. 记录历史数据，供绘图和控制台汇总使用。
        self._record_guidance_metrics(time, R_mag, nmpc_result, pn_trend)
        if self._should_log_step(time):
            self._log_guidance_step(
                time=time,
                R_vec=R_vec,
                V_rel=V_rel,
                R_mag=R_mag,
                distance_after=distance_after_update,
                Vc=Vc,
                Omega=Omega,
                ac_vec=ac_vec,
                control_acc=control_acc,
                current_speed_before_norm=current_speed,
                nmpc_result=nmpc_result,
                pn_trend=pn_trend,
            )
        self._last_distance = float(R_mag)
        self._step_index += 1

        return False # Not hit yet

    def run_simulation(self, max_time=600):
        """运行仿真循环"""
        time = 0
        hit = False
        steps = int(max_time / self.dt)
        self.logger.info("开始仿真: max_time=%.2fs, steps=%d, log_interval=%.3fs", max_time, steps, self.log_interval)

        for _ in range(steps):
            # 每个仿真步先更新目标，再让追踪者基于最新目标状态做一次制导更新。
            self.update_target_state(time)
            hit = self.update_missile_state(time)
            if hit:
                print(f"目标在 t={time:.2f}s 时被捕获！")
                print(f"最终速度: {self.m_speed:.2f} m/s")
                break
            time += self.dt

        if not hit:
            print(f"仿真结束，未能在 {max_time}s 内捕获目标。")
        self.logger.info(
            "仿真结束: hit=%s, elapsed=%.2fs, final_distance=%.3f, final_speed=%.3f",
            hit,
            time,
            self.distance_history[-1] if self.distance_history else np.nan,
            self.m_speed,
        )

        self.print_simulation_summary()
        return hit

    def get_simulation_metrics(self):
        """汇总关键指标，方便无绘图测试和后续对比 PN/NMPC。"""
        # 转成 ndarray 后可以直接做 min/max/sum，避免手写循环统计。
        distances = np.asarray(self.distance_history, dtype=float)
        h_errors = np.asarray(self.fov_horizontal_history, dtype=float)
        v_errors = np.asarray(self.fov_vertical_history, dtype=float)
        violations = np.asarray(self.fov_violation_history, dtype=bool)
        fallbacks = np.asarray(self.fallback_history, dtype=bool)

        return {
            # min_distance 越小越接近成功拦截。
            'min_distance': float(np.min(distances)) if len(distances) else np.nan,
            # 每个违规采样点近似代表 dt 秒，因此违规时长 = 违规点数 * dt。
            'fov_violation_time': float(np.sum(violations) * self.dt),
            # 最大角误差用于观察目标是否频繁跑到传感器视场边缘之外。
            'max_fov_horizontal_error_deg': float(np.max(np.abs(h_errors))) if len(h_errors) else np.nan,
            'max_fov_vertical_error_deg': float(np.max(np.abs(v_errors))) if len(v_errors) else np.nan,
            # fallback 次数越多，说明 NMPC 参数/候选集合可能存在数值或约束问题。
            'nmpc_fallback_count': int(np.sum(fallbacks)),
            'history_length': len(self.time_history),
        }

    def print_simulation_summary(self):
        """在控制台输出 NMPC/FOV 结果，避免只看图时遗漏关键约束表现。"""
        metrics = self.get_simulation_metrics()
        mode = 'PN + NMPC + FOV' if self.use_nmpc else 'Pure PN'
        print("\n" + "-" * 60)
        print(f"控制模式: {mode}")
        print(f"最小距离: {metrics['min_distance']:.2f} m")
        print(f"FOV违规总时长: {metrics['fov_violation_time']:.2f} s")
        print(f"最大水平视角误差: {metrics['max_fov_horizontal_error_deg']:.2f} deg")
        print(f"最大垂直视角误差: {metrics['max_fov_vertical_error_deg']:.2f} deg")
        print(f"NMPC fallback次数: {metrics['nmpc_fallback_count']}")
        print("-" * 60)

    def plot_results(self):
        """绘制3D轨迹图和速度变化图（分别绘制在独立窗口中）"""
        t_traj = np.array(self.t_traj)
        m_traj = np.array(self.m_traj)

        # 图1: 3D轨迹
        fig1 = plt.figure(figsize=(10, 8))
        try:
            # Pylance 默认会把 add_subplot() 推断成普通 2D Axes。
            # 这里显式告诉类型检查器：projection='3d' 时返回的是 Axes3D，
            # 否则它会把第三个坐标 z 误认为 scatter() 的尺寸参数 s。
            ax1_3d = cast(Axes3D, fig1.add_subplot(111, projection='3d')) # type: ignore
            ax1: Axes = ax1_3d
        except Exception:
            # 某些环境的 Matplotlib 3D 插件不可用；退回 X-Y 平面图，避免整套结果无法显示。
            ax1 = fig1.add_subplot(111)
            ax1_3d = None

        # 绘制轨迹
        if ax1_3d is not None:
            ax1_3d.plot(t_traj[:, 0], t_traj[:, 1], t_traj[:, 2], 'b--', label='Target', linewidth=2)
        else:
            ax1.plot(t_traj[:, 0], t_traj[:, 1], 'b--', label='Target', linewidth=2)
        tracker_label = 'Tracker (PN+NMPC)' if self.use_nmpc else 'Tracker (PN)'
        if ax1_3d is not None:
            ax1_3d.plot(m_traj[:, 0], m_traj[:, 1], m_traj[:, 2], 'r-', label=tracker_label, linewidth=2)
        else:
            ax1.plot(m_traj[:, 0], m_traj[:, 1], 'r-', label=tracker_label, linewidth=2)

        # 绘制起点和终点
        if ax1_3d is not None:
            ax1_3d.scatter(t_traj[0, 0], t_traj[0, 1], t_traj[0, 2], c='blue', marker='o', s=50, label='Target Start')
            ax1_3d.scatter(m_traj[0, 0], m_traj[0, 1], m_traj[0, 2], c='red', marker='o', s=50, label='Tracker Start')
        else:
            ax1.scatter(t_traj[0, 0], t_traj[0, 1], c='blue', marker='o', s=50, label='Target Start')
            ax1.scatter(m_traj[0, 0], m_traj[0, 1], c='red', marker='o', s=50, label='Tracker Start')

        # 终点
        if ax1_3d is not None:
            ax1_3d.scatter(m_traj[-1, 0], m_traj[-1, 1], m_traj[-1, 2], c='black', marker='x', s=100, label='Intercept Point')
        else:
            ax1.scatter(m_traj[-1, 0], m_traj[-1, 1], c='black', marker='x', s=100, label='Intercept Point')

        # 连接一些连线表示视线 (Line of Sight)
        # 不画每一帧连线，避免图像过密；大约采样 15 条视线即可观察追踪关系。
        step = max(1, len(m_traj) // 15)
        for i in range(0, len(m_traj), step):
            if i < len(t_traj):
                if ax1_3d is not None:
                    ax1_3d.plot([m_traj[i, 0], t_traj[i, 0]],
                                [m_traj[i, 1], t_traj[i, 1]],
                                [m_traj[i, 2], t_traj[i, 2]],
                                'g-', alpha=0.2, linewidth=0.5)
                else:
                    ax1.plot([m_traj[i, 0], t_traj[i, 0]],
                            [m_traj[i, 1], t_traj[i, 1]],
                            'g-', alpha=0.2, linewidth=0.5)

        mode_label = 'PN+NMPC+FOV' if self.use_nmpc else 'Pure PN'
        ax1.set_title(f'3D Guidance ({mode_label}, N={self.N}, Strategy={self.speed_strategy})')
        ax1.set_xlabel('X Position (m)')
        ax1.set_ylabel('Y Position (m)')
        if ax1_3d is not None:
            ax1_3d.set_zlabel('Z Position (m)')
        ax1.legend(fontsize=8)

        # 设置坐标轴比例一致
        # 3D 图默认各轴缩放可能不同，会让轨迹形状产生视觉误导，所以手动统一范围。
        max_range = np.array([t_traj[:,0].max()-t_traj[:,0].min(),
                              t_traj[:,1].max()-t_traj[:,1].min(),
                              t_traj[:,2].max()-t_traj[:,2].min(),
                              m_traj[:,0].max()-m_traj[:,0].min(),
                              m_traj[:,1].max()-m_traj[:,1].min(),
                              m_traj[:,2].max()-m_traj[:,2].min()]).max() / 2.0

        mid_x = (t_traj[:,0].max()+t_traj[:,0].min()) * 0.5
        mid_y = (t_traj[:,1].max()+t_traj[:,1].min()) * 0.5
        mid_z = (t_traj[:,2].max()+t_traj[:,2].min()) * 0.5

        ax1.set_xlim(mid_x - max_range, mid_x + max_range)
        ax1.set_ylim(mid_y - max_range, mid_y + max_range)
        if ax1_3d is not None:
            ax1_3d.set_zlim(mid_z - max_range, mid_z + max_range)
        fig1.tight_layout()

        # 图2: 速度变化曲线
        fig2 = plt.figure(figsize=(10, 6))
        ax2 = fig2.add_subplot(111)
        ax2.plot(self.time_history, self.speed_history, 'r-', linewidth=2, label='Tracker Speed')
        ax2.axhline(y=self.speed_min, color='g', linestyle='--', label=f'Min Speed ({self.speed_min} m/s)')
        ax2.axhline(y=self.speed_max, color='b', linestyle='--', label=f'Max Speed ({self.speed_max} m/s)')
        ax2.axhline(y=self.m_speed_init, color='orange', linestyle=':', label=f'Initial Speed ({self.m_speed_init} m/s)')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Speed (m/s)')
        ax2.set_title('Tracker Speed vs Time')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        fig2.tight_layout()

        # 图3: 距离变化曲线
        fig3 = plt.figure(figsize=(10, 6))
        ax3 = fig3.add_subplot(111)
        ax3.plot(self.time_history, self.distance_history, 'b-', linewidth=2)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Distance (m)')
        ax3.set_title('Distance to Target vs Time')
        ax3.grid(True, alpha=0.3)
        fig3.tight_layout()

        # 图4: 速度-距离关系图
        fig4 = plt.figure(figsize=(10, 6))
        ax4 = fig4.add_subplot(111)
        scatter = ax4.scatter(self.distance_history, self.speed_history, c=self.time_history, cmap='viridis', s=5)
        ax4.set_xlabel('Distance to Target (m)')
        ax4.set_ylabel('Speed (m/s)')
        ax4.set_title('Speed vs Distance (color = time)')
        ax4.grid(True, alpha=0.3)

        # 添加颜色条
        plt.colorbar(scatter, ax=ax4, label='Time (s)')
        fig4.tight_layout()

        # 图5: FOV角误差曲线
        fig5 = plt.figure(figsize=(10, 6))
        ax5 = fig5.add_subplot(111)
        # time_history 与 FOV history 理论上等长；这里按 FOV 长度截断，增强容错性。
        fov_time = np.asarray(self.time_history[:len(self.fov_horizontal_history)], dtype=float)
        h_errors = np.asarray(self.fov_horizontal_history, dtype=float)
        v_errors = np.asarray(self.fov_vertical_history, dtype=float)
        violations = np.asarray(self.fov_violation_history, dtype=bool)

        ax5.plot(fov_time, h_errors, 'm-', linewidth=1.5, label='Horizontal FOV Error')
        ax5.plot(fov_time, v_errors, 'c-', linewidth=1.5, label='Vertical FOV Error')
        # 水平/垂直 FOV 边界用不同线型标出，便于判断何时超出半视场角。
        ax5.axhline(self.nmpc_config.fov_horizontal_deg * 0.5, color='m', linestyle='--', alpha=0.4)
        ax5.axhline(-self.nmpc_config.fov_horizontal_deg * 0.5, color='m', linestyle='--', alpha=0.4)
        ax5.axhline(self.nmpc_config.fov_vertical_deg * 0.5, color='c', linestyle=':', alpha=0.5)
        ax5.axhline(-self.nmpc_config.fov_vertical_deg * 0.5, color='c', linestyle=':', alpha=0.5)
        if np.any(violations):
            ax5.scatter(
                fov_time[violations],
                h_errors[violations],
                c='red',
                s=8,
                alpha=0.6,
                label='FOV Violation',
            )
        ax5.set_xlabel('Time (s)')
        ax5.set_ylabel('Angle Error (deg)')
        ax5.set_title('FOV Angle Error vs Time')
        ax5.legend()
        ax5.grid(True, alpha=0.3)
        fig5.tight_layout()

        plt.show()


def compare_strategies():
    """比较不同速度控制策略的效果"""
    strategies = ['adaptive', 'distance', 'energy', 'pursuit']
    results = {}

    fig = plt.figure(figsize=(16, 12))

    for idx, strategy in enumerate(strategies):
        sim = ProportionalNavigation3D_VariableSpeed(
            target_start, tracker_start, target_velocity, tracker_speed,
            N=4.0, speed_min=20, speed_max=60, speed_strategy=strategy,
            # 每种策略使用同一个种子，保证它们遇到完全相同的目标噪声，比较才公平。
            random_seed=DEFAULT_RANDOM_SEED,
        )

        print(f"\n{'='*50}")
        print(f"策略: {strategy}")
        print(f"{'='*50}")

        hit = sim.run_simulation()
        results[strategy] = {
            'hit': hit,
            'time_history': sim.time_history.copy(),
            'speed_history': sim.speed_history.copy(),
            'distance_history': sim.distance_history.copy(),
            'metrics': sim.get_simulation_metrics(),
        }

        # 绘制速度曲线比较
        ax = fig.add_subplot(2, 2, idx + 1)
        ax.plot(sim.time_history, sim.speed_history, 'r-', linewidth=2, label='Speed')
        ax.axhline(y=sim.speed_min, color='g', linestyle='--', alpha=0.5)
        ax.axhline(y=sim.speed_max, color='b', linestyle='--', alpha=0.5)

        ax2 = ax.twinx()
        ax2.plot(sim.time_history, sim.distance_history, 'b-', linewidth=1, alpha=0.5, label='Distance')
        ax2.set_ylabel('Distance (m)', color='b')

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Speed (m/s)', color='r')
        ax.set_title(f'Strategy: {strategy}')
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('strategy_comparison.png', dpi=150)
    plt.show()

    return results


# --- 主程序 ---
if __name__ == "__main__":
    print("=" * 60)
    print("3D比例导引 - 可变速度控制仿真")
    print("=" * 60)

    # 可选择的速度控制策略:
    # 'adaptive' - 自适应速度控制（综合距离、角速率等因素）
    # 'distance' - 基于距离的速度控制（远距离高速，近距离低速）
    # 'energy'   - 能量优化策略（节省能量）
    # 'pursuit'  - 追击优化策略（保持速度优势）

    # 参数设定
    target_start = [500, 500, 500]      # 目标初始位置 [x, y, z]
    tracker_start = [0, 0, 0]            # 追踪者初始位置 [x, y, z]
    target_velocity = [0, 0, 0]          # 目标初始速度向量
    tracker_speed = 0                   # 追踪者初始速率

    # 实例化并运行
    sim = ProportionalNavigation3D_VariableSpeed(
        target_start, tracker_start, target_velocity, tracker_speed,
        N=4.0,
        speed_min=2,      # 最小速度 2 m/s
        speed_max=30,      # 最大速度 30 m/s
        speed_strategy='adaptive',  # 使用自适应速度控制策略
        use_nmpc=True,      # 启用 NMPC，使用 NMPC 和 PN 控制做基础仿真
        random_seed=DEFAULT_RANDOM_SEED,  # 固定噪声种子，保证每次运行目标扰动一致
        # 调试追踪策略时建议打开 DEBUG：控制台和 pn_guidance_debug.log 都会看到关键诊断量。
        log_level=logging.DEBUG,
        log_interval=0.5,
        log_to_file=Path(__file__).with_name("pn_guidance_debug.log"),
    )

    sim.run_simulation()
    sim.plot_results()

    # 如果想比较所有策略，取消下面的注释:
    # print("\n" + "=" * 60)
    # print("比较不同速度控制策略")
    # print("=" * 60)
    # compare_strategies()
