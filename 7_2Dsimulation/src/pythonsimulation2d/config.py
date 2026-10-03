from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


# 这里集中定义合法场景和算法名称，CLI、仿真循环、绘图和指标输出都复用它们。
# “定高俯瞰”的 2D 追踪仿真：状态数组仍保存 [x, y, z]，但控制律只使用 XY。
SCENARIOS = ("stationary", "linear", "circle")
ALGORITHMS = ("basic", "pn", "pn_mppi", "pn_nmpc")

ALGORITHM_LABELS = {
    "basic": "2D direct pursuit",
    "pn": "2D PN",
    "pn_mppi": "2D PN + MPPI",
    "pn_nmpc": "2D PN + NMPC",
}

# 论文插图的面板标题：比 ALGORITHM_LABELS 更短，和正文表格里的叫法一致。
# 其中 pn_nmpc 在文档中统称 EMPC（Enumerative MPC），代码标识保持不变，见 AGENTS.md。
ALGORITHM_PANEL_LABELS = {
    "basic": "Direct Pursuit",
    "pn": "PN-only",
    "pn_mppi": "PN-guided MPPI",
    "pn_nmpc": "PN-guided E-MPC",
}


@dataclass(slots=True)
class PursuerConfig:
    # 追踪机定高俯瞰目标。z 会在每个动力学积分步强制锁定为 fixed_altitude。
    fixed_altitude: float = 8.0
    initial_position: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 8.0]))
    initial_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    # yaw=0 表示水平面内机头初始朝 +x 方向。
    initial_yaw: float = 0.0
    # v_max/a_max 是所有导引算法共享的水平速度/加速度约束，用来保证对比公平。
    v_max: float = 12.0
    a_max: float = 6.0
    # yaw 角速度限制模拟“机头不能瞬间水平转向”的现实约束。
    yaw_rate_max: float = np.deg2rad(90.0)


@dataclass(slots=True)
class TargetConfig:
    # 目标固定在离地 1m；轨迹只在 XY 平面运动。
    fixed_altitude: float = 1.0
    stationary_position: np.ndarray = field(default_factory=lambda: np.array([40.0, 20.0, 1.0]))
    linear_position: np.ndarray = field(default_factory=lambda: np.array([25.0, -20.0, 1.0]))
    linear_velocity: np.ndarray = field(default_factory=lambda: np.array([2.0, 1.0, 0.0]))
    circle_center: np.ndarray = field(default_factory=lambda: np.array([35.0, 0.0, 1.0]))
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
    # NMPC/MPPI 预测窗口：8 步 * 0.1s = 向前看 0.8 秒。
    # 真实 PX4 闭环对加速度指令有明显滞后，窗口取 2s 时内部模型会高估自身机动能力、
    # 预判"激进候选会飞过目标"而反复选择偏保守的小修正，视觉闭环里容易停在外侧
    # 大半径轨道上（实测水平误差可到 6.8m）；缩短到与执行器动态同量级后，
    # 候选选择更贴近真实响应，配合下面的 PN 权重可以把闭环误差压回捕获半径附近。
    horizon_steps: int = 8
    mpc_dt: float = 0.1
    # 下面的权重共同决定预测控制的取舍：距离项让它追上目标，控制项和平滑项抑制剧烈机动。
    nmpc_w_dist: float = 12.0
    # 路径代价权重：积分整个预测窗口的距离，权重越大越鼓励持续接近而不是只看窗口末端。
    nmpc_w_path: float = 0.5
    nmpc_w_control: float = 0.015
    nmpc_w_smooth: float = 0.08
    # 偏离 PN 趋势的惩罚：真实闭环下内部模型不可靠，权重提高让 NMPC 在拿不准时
    # 跟随已被验证的 2D PN 趋势（候选集合里仍保留 pn_trend），避免自选动作持续失效。
    nmpc_w_pn: float = 1.0
    # MPPI 采样式预测控制参数；采样数越大越稳但越慢，seed 保证对比可复现。
    mppi_samples: int = 48
    mppi_noise_scale: float = 2.5
    mppi_temperature: float = 6.0
    mppi_seed: int = 7


@dataclass(slots=True)
class SimulationConfig:
    dt: float = 0.05
    sim_time: float = 40.0
    # 捕获半径按 XY 水平距离判断，不包含定高导致的垂向差。
    capture_radius: float = 1.5
    pursuer: PursuerConfig = field(default_factory=PursuerConfig)
    target: TargetConfig = field(default_factory=TargetConfig)
    guidance: GuidanceConfig = field(default_factory=GuidanceConfig)
