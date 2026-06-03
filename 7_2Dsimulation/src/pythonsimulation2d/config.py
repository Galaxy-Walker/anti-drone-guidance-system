from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


# 这里集中定义合法场景和算法名称，CLI、仿真循环、绘图和指标输出都复用它们。
# 新版本是“定高俯瞰”的 2D 追踪仿真：状态数组仍保存 [x, y, z]，但控制律只使用 XY。
SCENARIOS = ("stationary", "linear", "circle")
ALGORITHMS = ("basic", "basic_fov", "pn_fov", "pn_fov_cbf", "pn_fov_mppi", "pn_fov_nmpc")

ALGORITHM_LABELS = {
    "basic": "2D direct pursuit",
    "basic_fov": "2D direct pursuit + FOV",
    "pn_fov": "2D PN + FOV",
    "pn_fov_cbf": "2D PN + FOV + CBF",
    "pn_fov_mppi": "2D PN + FOV + MPPI",
    "pn_fov_nmpc": "2D PN + FOV + NMPC",
}


@dataclass(slots=True)
class PursuerConfig:
    # 追踪机定高俯瞰地面目标。z 会在每个动力学积分步强制锁定为 fixed_altitude。
    fixed_altitude: float = 8.0
    initial_position: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 8.0]))
    initial_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    # yaw=0 表示水平面内机头/相机初始朝 +x 方向。
    initial_yaw: float = 0.0
    # v_max/a_max 是所有导引算法共享的水平速度/加速度约束，用来保证对比公平。
    v_max: float = 12.0
    a_max: float = 6.0
    # yaw 角速度限制模拟“机头/相机不能瞬间水平转向”的现实约束。
    yaw_rate_max: float = np.deg2rad(90.0)


@dataclass(slots=True)
class TargetConfig:
    # 地面目标固定 z=0；轨迹只在 XY 平面运动。
    stationary_position: np.ndarray = field(default_factory=lambda: np.array([40.0, 20.0, 0.0]))
    linear_position: np.ndarray = field(default_factory=lambda: np.array([25.0, -20.0, 0.0]))
    linear_velocity: np.ndarray = field(default_factory=lambda: np.array([2.0, 1.0, 0.0]))
    circle_center: np.ndarray = field(default_factory=lambda: np.array([35.0, 0.0, 0.0]))
    circle_radius: float = 12.0
    circle_omega: float = 0.25


@dataclass(slots=True)
class GuidanceConfig:
    # direct_* 是基础追踪法的参数：它不是预测拦截，只是把水平速度指向目标当前位置。
    direct_v_cruise: float = 8.0
    direct_tau: float = 0.8
    # 2D PN 的 N 越大，横向修正越激进；太小会拦截慢，太大可能控制抖动。
    pn_navigation_constant: float = 3.5
    # k_close 和 v_des_along_los 给 PN 加一个沿水平视线方向的“主动接近”速度目标。
    pn_k_close: float = 1.0
    pn_v_des_along_los: float = 8.0
    # FOV 限制算法使用水平扇形视场和 α-β 滤波估计目标 XY 位置/速度。
    sensor_ab_alpha: float = 0.85
    sensor_ab_beta: float = 0.25
    # fov_deg 是总水平视场角，真正判断时会用半角：目标方位角偏差 <= fov_deg/2。
    fov_deg: float = 90.0
    # 目标丢失时降低导引增益，表示追踪机对预测目标没有对真实目标那么确信。
    lost_guidance_gain: float = 0.55
    # NMPC/MPPI 预测窗口：20 步 * 0.1s = 向前看 2 秒；窗口越长越慢。
    horizon_steps: int = 20
    mpc_dt: float = 0.1
    # 下面的权重共同决定预测控制的取舍：距离项让它追上目标，FOV 项让它别把目标甩出水平视场。
    nmpc_w_dist: float = 12.0
    nmpc_w_path: float = 0.1
    nmpc_w_fov: float = 10.0
    nmpc_w_control: float = 0.015
    nmpc_w_smooth: float = 0.08
    nmpc_w_pn: float = 0.04
    # CBF filter 在接近水平 FOV 边界前就开始侧向修正，避免等目标出视场后再搜索。
    cbf_fov_margin_deg: float = 8.0
    cbf_gain: float = 0.75
    # MPPI 采样式预测控制参数；采样数越大越稳但越慢，seed 保证对比可复现。
    mppi_samples: int = 48
    mppi_noise_scale: float = 2.5
    mppi_temperature: float = 6.0
    mppi_seed: int = 7

    @property
    def fov_half_angle(self) -> float:
        return np.deg2rad(self.fov_deg * 0.5)


@dataclass(slots=True)
class SimulationConfig:
    dt: float = 0.05
    sim_time: float = 40.0
    # 捕获半径按 XY 水平距离判断，不包含定高导致的垂向差。
    capture_radius: float = 1.5
    pursuer: PursuerConfig = field(default_factory=PursuerConfig)
    target: TargetConfig = field(default_factory=TargetConfig)
    guidance: GuidanceConfig = field(default_factory=GuidanceConfig)
