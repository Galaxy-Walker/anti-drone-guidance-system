"""
比例导引(PN)核心算法模块

本模块实现了纯粹的导引计算逻辑，与 ROS2 和 PX4 完全解耦。
可独立用于离线仿真测试或集成到任何飞控系统中。

坐标系约定：NED（北-东-地），与 PX4 保持一致。
"""

import numpy as np
import logging
from dataclasses import dataclass, field
from typing import Optional, Callable

# 模块级日志器，离线仿真时使用 logging，ROS2 运行时可替换为 rclpy logger
logger = logging.getLogger(__name__)


@dataclass
class State:
    """
    通用状态类，用于在模块间传递位姿数据。

    属性:
        position: [x, y, z] 位置向量（NED 坐标系）
        velocity: [vx, vy, vz] 速度向量
        speed: 标量速度，兼容 numpy 和 python float 类型
    """
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    speed: float | np.floating = 0.0  # 标量速度


@dataclass
class GuidanceResult:
    """
    导引计算结果，包含控制指令和调试中间量。

    属性:
        pos_cmd: 期望位置设定点 [x, y, z]
        vel_cmd: 期望速度向量 [vx, vy, vz]
        distance: 追踪者与目标的距离 (m)
        closing_speed: 接近速度 (m/s)，正值表示在靠近
        los_rate: 视线角速率大小 (rad/s)
        desired_speed: 期望速率 (m/s)
        acc_cmd: 加速度指令向量
    """
    pos_cmd: np.ndarray
    vel_cmd: np.ndarray
    distance: float
    closing_speed: float
    los_rate: float
    desired_speed: float
    acc_cmd: np.ndarray


# ============================================================
# 速度策略：每个策略是一个独立的纯函数，方便单独测试和扩展
# ============================================================

def speed_strategy_adaptive(
    r_mag: float, vc: float, omega_mag: float,
    target_speed: float, current_base_speed: float,
    speed_min: float, speed_max: float
) -> float:
    """
    可变速比例导引策略：根据距离、接近状态和视线角速率动态调速。

    设计目标：在追踪者存在不利初速度时，仍能快速建立有效接近。
    """
    dist_speed = np.interp(np.clip(r_mag, 0.0, 250.0), [0.0, 250.0], [speed_min, speed_max])

    if vc <= 0.0:
        closing_boost = 1.35
    elif vc < 0.35 * max(dist_speed, speed_min):
        closing_boost = 1.15
    else:
        closing_boost = 1.0

    omega_penalty = np.clip(1.0 - omega_mag * 3.0, 0.75, 1.0)

    speed_floor = max(speed_min, 0.8 * current_base_speed, 1.1 * target_speed)
    desired = max(dist_speed * closing_boost * omega_penalty, speed_floor)

    logger.debug(
        f"[可变速PN] dist_speed={dist_speed:.2f}, closing_boost={closing_boost:.2f}, "
        f"omega_penalty={omega_penalty:.2f}, desired={desired:.2f}"
    )
    return float(np.clip(desired, speed_min, speed_max))


def speed_strategy_pursuit(
    r_mag: float, vc: float, omega_mag: float,
    target_speed: float, current_base_speed: float,
    speed_min: float, speed_max: float
) -> float:
    """
    最简单追踪策略：始终保持目标速度的 1.5 倍。
    """
    desired = target_speed * 1.5
    logger.debug(f"[追击策略] 目标速度={target_speed:.2f}, 期望速度={desired:.2f}")
    return float(np.clip(desired, speed_min, speed_max))


# 策略名称 -> 函数的映射表，方便通过字符串选择策略
SPEED_STRATEGIES = {
    'adaptive': speed_strategy_adaptive,
    'pursuit': speed_strategy_pursuit,
}


class PNGuidanceCore:
    """
    比例导引(PN)核心算法模块。

    负责计算期望的飞行矢量（位置设定点和速度设定点），
    不涉及任何底层通信（ROS2、MAVSDK 等），实现完全解耦。

    参数:
        N: 导引比例系数（通常取 3~5）
        speed_min: 最小速度限制 (m/s)
        speed_max: 最大速度限制 (m/s)
        strategy: 速度策略名称，可选 'adaptive', 'pursuit'
        log_func: 可选的外部日志函数（如 rclpy logger），默认使用 logging
    """

    def __init__(
        self,
        N: float = 4.0,
        speed_min: float = 2.0,
        speed_max: float = 30.0,
        strategy: str = 'adaptive',
        log_func: Optional[Callable] = None,
    ):
        self.N = N
        self.speed_min = speed_min
        self.speed_max = speed_max
        self.desired_speed = speed_min
        self.current_acc_cmd = np.zeros(3)

        # 设置速度策略
        if strategy not in SPEED_STRATEGIES:
            raise ValueError(f"未知的速度策略 '{strategy}'，可选: {list(SPEED_STRATEGIES.keys())}")
        self.strategy_name = strategy
        self._speed_strategy_func = SPEED_STRATEGIES[strategy]

        # 日志函数：允许外部注入（如 ROS2 logger），默认使用 logging
        self._log = log_func or logger.info

        self._log(f"导引核心初始化完成: N={N}, 速度范围=[{speed_min}, {speed_max}], 策略={strategy}")

    def set_strategy(self, strategy: str):
        """动态切换速度策略。"""
        if strategy not in SPEED_STRATEGIES:
            raise ValueError(f"未知的速度策略 '{strategy}'，可选: {list(SPEED_STRATEGIES.keys())}")
        self.strategy_name = strategy
        self._speed_strategy_func = SPEED_STRATEGIES[strategy]
        self._log(f"速度策略已切换为: {strategy}")

    def calculate_guidance(
        self, tracker_state: State, target_state: State, dt: float
    ) -> GuidanceResult:
        """
        计算下一时刻的导引控制指令。

        参数:
            tracker_state: 追踪者（己方无人机）当前状态
            target_state: 目标当前状态
            dt: 时间步长 (s)

        返回:
            GuidanceResult: 包含位置/速度设定点及调试中间量的结果对象
        """
        # ---- 第1步：计算相对运动状态 ----
        R_vec = target_state.position - tracker_state.position  # 相对位置矢量（目标-追踪者）
        V_rel = target_state.velocity - tracker_state.velocity  # 相对速度矢量

        R_mag = float(np.linalg.norm(R_vec))  # 距离标量
        R_sq = float(np.dot(R_vec, R_vec))    # 距离平方

        # 容错：防止除零
        if R_mag < 0.1:
            R_mag = 0.1
        if R_sq < 0.01:
            R_sq = 0.01

        # ---- 第2步：计算视线角速率 Omega ----
        Omega = np.cross(R_vec, V_rel) / R_sq     # 视线角速率矢量
        Omega_mag = float(np.linalg.norm(Omega))   # 视线角速率标量

        # ---- 第3步：计算接近速度 Vc ----
        Vc = float(-np.dot(R_vec, V_rel) / R_mag)

        # ---- 第4步：通过速度策略计算期望速率 ----
        self.desired_speed = self._speed_strategy_func(
            R_mag, Vc, Omega_mag,
            float(target_state.speed), float(tracker_state.speed),
            self.speed_min, self.speed_max
        )

        # ---- 第5步：3D 比例导引加速度指令 ----
        # ac = N * Vc * (Omega × R_unit)
        R_unit = R_vec / R_mag

        # 最简单追踪法：直接沿目标方向飞行，不使用 PN 横向加速度
        if self.strategy_name == 'pursuit':
            vel_cmd_final = R_unit * self.desired_speed
            if dt > 1e-6:
                ac_vec = (vel_cmd_final - tracker_state.velocity) / dt
            else:
                ac_vec = np.zeros(3)
            self.current_acc_cmd = ac_vec

            pos_cmd = tracker_state.position + vel_cmd_final * dt
            return GuidanceResult(
                pos_cmd=pos_cmd,
                vel_cmd=vel_cmd_final,
                distance=R_mag,
                closing_speed=Vc,
                los_rate=Omega_mag,
                desired_speed=self.desired_speed,
                acc_cmd=ac_vec,
            )

        vc_eff = max(Vc, 0.6 * self.desired_speed)
        ac_vec = self.N * vc_eff * np.cross(Omega, R_unit)
        self.current_acc_cmd = ac_vec

        # ---- 第6步：计算期望速度矢量 ----
        # 当前速度 + 加速度指令 × 时间步长
        vel_cmd_raw = tracker_state.velocity + ac_vec * dt

        # ---- 第7步：归一化并应用期望速率 ----
        # PNG 决定飞行方向，速度策略决定飞行快慢
        vel_cmd_norm = np.linalg.norm(vel_cmd_raw)
        if vel_cmd_norm > 0:
            pn_dir = vel_cmd_raw / vel_cmd_norm
            base_blend = 0.30
            if Vc < 0:
                extra_blend = np.clip(-Vc / max(self.desired_speed, 1e-3), 0.0, 0.45)
            else:
                extra_blend = 0.0
            blend_to_target = base_blend + extra_blend

            cmd_dir = (1.0 - blend_to_target) * pn_dir + blend_to_target * R_unit
            cmd_dir_norm = np.linalg.norm(cmd_dir)
            if cmd_dir_norm > 1e-6:
                cmd_dir = cmd_dir / cmd_dir_norm
            else:
                cmd_dir = R_unit

            vel_cmd_final = cmd_dir * self.desired_speed
        else:
            # 初始或静止时直接指向目标
            vel_cmd_final = R_unit * self.desired_speed

        # ---- 第8步：计算期望位置设定点 ----
        # 简单预测：下一位置 = 当前位置 + 期望速度 × dt
        pos_cmd = tracker_state.position + vel_cmd_final * dt

        # 构造结果对象（包含调试信息）
        result = GuidanceResult(
            pos_cmd=pos_cmd,
            vel_cmd=vel_cmd_final,
            distance=R_mag,
            closing_speed=Vc,
            los_rate=Omega_mag,
            desired_speed=self.desired_speed,
            acc_cmd=ac_vec,
        )

        return result
