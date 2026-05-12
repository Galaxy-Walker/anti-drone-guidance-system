import logging
from copy import deepcopy
from pathlib import Path

import numpy as np

from .config import (
    DEFAULT_40CM_CONFIG,
    DEFAULT_RANDOM_SEED,
    NMPCConfig,
    SimulationConfig,
    SpeedControlConfig,
    TargetMotionConfig,
    VehicleConfig,
)
from .nmpc import NMPCController
from .pn_guidance import build_pn_trend, compute_pn_acceleration, compute_relative_geometry
from .speed_control import compute_desired_speed, update_speed, validate_speed_strategy
from .target_model import update_target_state as update_target_model_state
from .utils import compute_fov_errors, format_vec, safe_norm, unit_or_default


class ProportionalNavigation3D_VariableSpeed:
    @classmethod
    def from_config(cls, config: SimulationConfig = DEFAULT_40CM_CONFIG):
        """用集中配置创建仿真实例，是 40cm 四旋翼教学仿真的推荐入口。"""
        nmpc_config = cls._build_nmpc_config_from_simulation_config(config)
        log_to_file = cls._resolve_log_to_file(config.logging.log_to_file)
        return cls(
            config.scenario.target_start,
            config.scenario.tracker_start,
            config.scenario.target_velocity,
            config.scenario.tracker_initial_speed,
            N=config.guidance.N,
            dt=config.guidance.dt,
            speed_min=config.vehicle.speed_min,
            speed_max=config.vehicle.speed_max,
            speed_strategy=config.guidance.speed_strategy,
            use_nmpc=config.guidance.use_nmpc,
            nmpc_config=nmpc_config,
            random_seed=config.random_seed,
            enable_logging=config.logging.enable_logging,
            log_level=config.logging.log_level, # type: ignore
            log_interval=config.logging.log_interval,
            log_to_file=log_to_file,
            speed_accel_limit=config.vehicle.speed_accel_limit,
            speed_control_config=config.speed_control,
            target_motion_config=config.target_motion,
            simulation_config=config,
        )

    @staticmethod
    def _resolve_log_to_file(log_to_file):
        """配置里的相对日志路径默认落在模块目录，避免从不同 cwd 运行时写到奇怪位置。"""
        if not log_to_file:
            return None
        path = Path(log_to_file)
        if path.is_absolute():
            return path
        return Path(__file__).resolve().parent / path

    @staticmethod
    def _build_nmpc_config_from_simulation_config(config):
        """把 vehicle/guidance 中的物理约束同步进 NMPC，避免同一参数改两处。"""
        nmpc_config = deepcopy(config.nmpc)
        nmpc_config.horizon_steps = config.guidance.horizon_steps
        nmpc_config.prediction_dt = nmpc_config.prediction_dt or config.guidance.dt
        nmpc_config.accel_max = config.vehicle.accel_max
        nmpc_config.speed_min = config.vehicle.speed_min
        nmpc_config.speed_max = config.vehicle.speed_max
        nmpc_config.fov_horizontal_deg = config.vehicle.fov_horizontal_deg
        nmpc_config.fov_vertical_deg = config.vehicle.fov_vertical_deg
        nmpc_config.capture_radius = config.vehicle.capture_radius
        return nmpc_config

    def __init__(self, t_pos, m_pos, t_vel, m_speed_init, N=3.5, dt=0.02,
                 speed_min=0.5, speed_max=12.0, speed_strategy="adaptive",
                 use_nmpc=True, nmpc_config=None, random_seed=DEFAULT_RANDOM_SEED,
                 enable_logging=True, log_level=logging.INFO, log_interval=1.0,
                 log_to_file=None, speed_accel_limit=None, speed_control_config=None,
                 target_motion_config=None, simulation_config=None):
        """
        初始化3D仿真参数（支持可变速度）。

        推荐新代码使用 from_config(DEFAULT_40CM_CONFIG)。保留这个构造函数，是为了旧脚本
        仍可直接传入位置、速度和 PN/NMPC 参数运行。

        :param speed_strategy: 速度调节策略；当前支持 "adaptive" 和 "pursuit"
        :param use_nmpc: True 时使用 PN趋势 + NMPC；False 时保留原始 PN 控制路径
        """
        self.simulation_config = simulation_config
        self.speed_control_config = speed_control_config or SpeedControlConfig()
        self.target_motion_config = target_motion_config or TargetMotionConfig()
        default_vehicle = simulation_config.vehicle if simulation_config is not None else VehicleConfig()
        self.speed_accel_limit = (
            float(speed_accel_limit)
            if speed_accel_limit is not None
            else float(default_vehicle.speed_accel_limit)
        )
        self.max_time = (
            float(simulation_config.scenario.max_time)
            if simulation_config is not None
            else 600.0
        )

        self.t_pos = np.array(t_pos, dtype=float)
        self.m_pos = np.array(m_pos, dtype=float)
        self.t_vel = np.array(t_vel, dtype=float)
        self.m_speed = m_speed_init
        self.m_speed_init = m_speed_init
        self.N = N
        self.dt = dt

        self.speed_min = speed_min
        self.speed_max = speed_max
        self.speed_strategy = validate_speed_strategy(speed_strategy)
        self.use_nmpc = use_nmpc
        self.enable_logging = enable_logging
        self.log_interval = max(float(log_interval), self.dt)
        self._next_log_time = 0.0
        self._step_index = 0
        self._last_distance = None
        self._last_speed_debug = {}
        self._last_speed_update_debug = {}
        self.rng = np.random.default_rng(random_seed)
        self.logger = self._setup_logger(log_level, log_to_file)

        if nmpc_config is None:
            nmpc_config = NMPCConfig(
                horizon_steps=(
                    simulation_config.guidance.horizon_steps
                    if simulation_config is not None
                    else DEFAULT_40CM_CONFIG.guidance.horizon_steps
                ),
                prediction_dt=dt,
                accel_max=default_vehicle.accel_max,
                speed_min=speed_min,
                speed_max=speed_max,
                fov_horizontal_deg=default_vehicle.fov_horizontal_deg,
                fov_vertical_deg=default_vehicle.fov_vertical_deg,
                capture_radius=default_vehicle.capture_radius,
            )
        elif nmpc_config.prediction_dt is None:
            nmpc_config.prediction_dt = dt
        self.nmpc_config = nmpc_config
        self.nmpc_controller = NMPCController(self.nmpc_config)

        diff = self.t_pos - self.m_pos
        dist = np.linalg.norm(diff)
        if dist > 0:
            self.m_vel = diff / dist * m_speed_init
        else:
            self.m_vel = np.array([m_speed_init, 0, 0], dtype=float)

        self.t_traj = [self.t_pos.copy()]
        self.m_traj = [self.m_pos.copy()]
        self.speed_history = [m_speed_init]
        self.time_history = [0]
        self.distance_history = [dist]
        self._last_distance = float(dist)

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
        self.nmpc_cost_history = [np.nan]
        self.fallback_history = [False]
        self.nmpc_predicted_tracker_traj = []
        self.nmpc_predicted_target_traj = []

        self._log_initial_state(random_seed)

    def _setup_logger(self, log_level, log_to_file):
        """给每个仿真实例创建独立 logger，避免多实例运行时重复打印。"""
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
            "speed_accel_limit=%.2f, accel_max=%.2f, capture_radius=%.2f, seed=%s",
            "PN+NMPC" if self.use_nmpc else "Pure PN",
            self.speed_strategy,
            self.N,
            self.dt,
            self.speed_min,
            self.speed_max,
            self.speed_accel_limit,
            self.nmpc_config.accel_max,
            self.nmpc_config.capture_radius,
            random_seed,
        )
        self.logger.info(
            "初始状态: target_pos=%s, tracker_pos=%s, target_vel=%s, tracker_vel=%s, "
            "distance=%.3f, tracker_speed=%.3f",
            format_vec(self.t_pos),
            format_vec(self.m_pos),
            format_vec(self.t_vel),
            format_vec(self.m_vel),
            self.distance_history[0],
            self.m_speed,
        )

    def update_target_state(self, time):
        """更新目标状态，并记录目标轨迹。"""
        self.t_pos, self.t_vel = update_target_model_state(
            self.t_pos, self.t_vel, time, self.dt, self.rng, self.target_motion_config
        )
        self.t_traj.append(self.t_pos.copy())

    def _record_guidance_metrics(self, time, R_mag, nmpc_result, pn_trend):
        """统一记录 PN-only 和 PN+NMPC 两种模式下的可视化指标。"""
        if nmpc_result is None:
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
            h_err = nmpc_result.fov_horizontal_error_deg
            v_err = nmpc_result.fov_vertical_error_deg
            violation = nmpc_result.fov_violation
            cost = nmpc_result.total_cost
            fallback = nmpc_result.fallback
            self.nmpc_predicted_tracker_traj.append(nmpc_result.predicted_tracker_traj)
            self.nmpc_predicted_target_traj.append(nmpc_result.predicted_target_traj)

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
                format_vec(self.t_vel),
                format_vec(self.m_vel),
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
                format_vec(ac_vec),
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
            format_vec(self.t_pos),
            format_vec(self.t_vel),
            safe_norm(self.t_vel),
            format_vec(self.m_pos),
            format_vec(self.m_vel),
            self.m_speed,
            current_speed_before_norm,
            self._last_speed_debug,
            self._last_speed_update_debug,
            format_vec(ac_vec),
            safe_norm(ac_vec),
            format_vec(pn_trend.reference_direction),
            format_vec(pn_trend.reference_intercept_point),
            format_vec(control_acc),
            safe_norm(control_acc),
            nmpc_cost,
            nmpc_fallback,
            nmpc_fov,
            format_vec(V_rel),
            format_vec(Omega),
        )

    def update_missile_state(self, time):
        """
        核心：使用 PN 生成趋势，再由 NMPC 选择最终控制量。

        use_nmpc=False 时，会走原来的 PN 直接控制逻辑，方便做 PN-only 基线实验。
        """
        # 1) 相对几何：R_vec 是“目标-追踪者”的位置差，V_rel 是相对速度。
        # PN、速度策略和 NMPC 都围绕这些几何量工作。
        geometry = compute_relative_geometry(self.t_pos, self.m_pos, self.t_vel, self.m_vel)

        # 2) 命中判定：教学仿真把进入 capture_radius 看作捕获成功。
        if geometry.R_mag < self.nmpc_config.capture_radius:
            forward = unit_or_default(self.m_vel, geometry.R_vec)
            h_err, v_err, violation = compute_fov_errors(
                self.t_pos - self.m_pos,
                forward,
                self.nmpc_config.fov_horizontal_deg,
                self.nmpc_config.fov_vertical_deg,
            )
            self.speed_history.append(self.m_speed)
            self.time_history.append(time)
            self.distance_history.append(geometry.R_mag) # type: ignore
            self.fov_horizontal_history.append(h_err)
            self.fov_vertical_history.append(v_err)
            self.fov_violation_history.append(violation)
            self.nmpc_cost_history.append(np.nan)
            self.fallback_history.append(False)
            self.logger.info(
                "命中判定: t=%.2fs step=%d distance=%.3f < capture_radius=%.3f",
                time,
                self._step_index,
                geometry.R_mag,
                self.nmpc_config.capture_radius,
            )
            return True

        if not np.all(np.isfinite([geometry.R_mag, geometry.R_sq, geometry.Vc, geometry.target_speed])):
            self.logger.error(
                "数值异常: t=%.2fs step=%d R_mag=%s R_sq=%s Vc=%s target_speed=%s R_vec=%s V_rel=%s",
                time,
                self._step_index,
                geometry.R_mag,
                geometry.R_sq,
                geometry.Vc,
                geometry.target_speed,
                format_vec(geometry.R_vec),
                format_vec(geometry.V_rel),
            )

        # 3) 标量速度策略：先决定“该飞多快”，暂不决定往哪个方向飞。
        desired_speed, self._last_speed_debug = compute_desired_speed(
            self.speed_strategy,
            geometry.R_mag,
            geometry.Vc,
            geometry.Omega_mag,
            geometry.target_speed,
            self.m_speed,
            self.m_speed_init,
            self.speed_min,
            self.speed_max,
            self.speed_control_config,
        )
        self.m_speed, self._last_speed_update_debug = update_speed(
            self.m_speed, desired_speed, self.dt, self.speed_accel_limit
        )

        # 4) PN 趋势：PN 给出参考加速度和参考方向，NMPC 再在这个趋势附近做约束选择。
        ac_vec = compute_pn_acceleration(
            self.N, geometry.Vc, geometry.Omega, geometry.R_vec, geometry.R_mag
        )
        pn_trend = build_pn_trend(
            geometry.R_vec,
            geometry.R_mag,
            geometry.Vc,
            geometry.Omega_mag,
            ac_vec,
            self.m_vel,
            self.t_pos,
            self.t_vel,
            self.m_speed,
            self.speed_min,
            self.dt,
            self.nmpc_config,
        )

        # 5) 控制选择：开启 NMPC 时用预测窗选最优候选；关闭时直接使用 PN 加速度。
        nmpc_result = None
        control_acc = ac_vec
        if self.use_nmpc:
            nmpc_result = self.nmpc_controller.solve(
                tracker_pos=self.m_pos,
                tracker_vel=self.m_vel,
                target_pos=self.t_pos,
                target_vel=self.t_vel,
                pn_trend=pn_trend,
                dt=self.dt,
            )
            control_acc = nmpc_result.acceleration

        # 6) 状态推进：先按加速度更新速度方向，再把速度模长约束到速度策略给出的 m_speed。
        self.m_vel += control_acc * self.dt

        current_speed = np.linalg.norm(self.m_vel)
        R_unit = geometry.R_vec / max(geometry.R_mag, 1e-9)
        if current_speed > 1e-9:
            velocity_dir = self.m_vel / current_speed
            alignment_to_los = float(np.dot(velocity_dir, R_unit))
            min_useful_speed = max(0.5, 0.25 * max(self.m_speed, self.speed_min))
            if current_speed < min_useful_speed or alignment_to_los < 0.0:
                velocity_dir = unit_or_default(0.65 * R_unit + 0.35 * velocity_dir, R_unit)
            self.m_vel = velocity_dir * self.m_speed
        else:
            self.m_vel = R_unit * self.m_speed

        self.m_pos += self.m_vel * self.dt
        self.m_traj.append(self.m_pos.copy())
        distance_after_update = safe_norm(self.t_pos - self.m_pos)

        # 7) 记录指标和日志：轨迹图、速度图、FOV 图都依赖这些 history。
        self._record_guidance_metrics(time, geometry.R_mag, nmpc_result, pn_trend)
        if self._should_log_step(time):
            self._log_guidance_step(
                time=time,
                R_vec=geometry.R_vec,
                V_rel=geometry.V_rel,
                R_mag=geometry.R_mag,
                distance_after=distance_after_update,
                Vc=geometry.Vc,
                Omega=geometry.Omega,
                ac_vec=ac_vec,
                control_acc=control_acc,
                current_speed_before_norm=current_speed,
                nmpc_result=nmpc_result,
                pn_trend=pn_trend,
            )
        self._last_distance = float(geometry.R_mag)
        self._step_index += 1

        return False

    def run_simulation(self, max_time=None):
        """运行仿真循环。"""
        if max_time is None:
            max_time = self.max_time
        time = 0
        hit = False
        steps = int(max_time / self.dt)
        self.logger.info("开始仿真: max_time=%.2fs, steps=%d, log_interval=%.3fs", max_time, steps, self.log_interval)

        for _ in range(steps):
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
        distances = np.asarray(self.distance_history, dtype=float)
        h_errors = np.asarray(self.fov_horizontal_history, dtype=float)
        v_errors = np.asarray(self.fov_vertical_history, dtype=float)
        violations = np.asarray(self.fov_violation_history, dtype=bool)
        fallbacks = np.asarray(self.fallback_history, dtype=bool)

        return {
            "min_distance": float(np.min(distances)) if len(distances) else np.nan,
            "fov_violation_time": float(np.sum(violations) * self.dt),
            "max_fov_horizontal_error_deg": float(np.max(np.abs(h_errors))) if len(h_errors) else np.nan,
            "max_fov_vertical_error_deg": float(np.max(np.abs(v_errors))) if len(v_errors) else np.nan,
            "nmpc_fallback_count": int(np.sum(fallbacks)),
            "history_length": len(self.time_history),
        }

    def print_simulation_summary(self):
        """在控制台输出 NMPC/FOV 结果，避免只看图时遗漏关键约束表现。"""
        metrics = self.get_simulation_metrics()
        mode = "PN + NMPC + FOV" if self.use_nmpc else "Pure PN"
        print("\n" + "-" * 60)
        print(f"控制模式: {mode}")
        print(f"最小距离: {metrics['min_distance']:.2f} m")
        print(f"FOV违规总时长: {metrics['fov_violation_time']:.2f} s")
        print(f"最大水平视角误差: {metrics['max_fov_horizontal_error_deg']:.2f} deg")
        print(f"最大垂直视角误差: {metrics['max_fov_vertical_error_deg']:.2f} deg")
        print(f"NMPC fallback次数: {metrics['nmpc_fallback_count']}")
        print("-" * 60)

    def plot_results(self):
        """绘制仿真结果。"""
        from .plotting import plot_results as plot_simulation_results

        plot_simulation_results(self)
