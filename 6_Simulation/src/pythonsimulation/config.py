from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


# 这里集中定义合法场景和算法名称，CLI、仿真循环、绘图和指标输出都复用它们。
# 这样新增算法/场景时不容易出现“命令行能选，但仿真循环没跑”的不一致。
SCENARIOS = ("stationary", "linear", "circle")
ALGORITHMS = ("basic", "pn", "pn_fov", "pn_fov_cbf", "pn_fov_mppi", "pn_fov_nmpc")

# 内部算法名适合写代码，图表标签适合给人看；分开保存可以避免到处写重复字符串。
ALGORITHM_LABELS = {
    "basic": "Direct pursuit",
    "pn": "3D PN",
    "pn_fov": "PN + FOV",
    "pn_fov_cbf": "PN + FOV + CBF",
    "pn_fov_mppi": "PN + FOV + MPPI",
    "pn_fov_nmpc": "PN + FOV + NMPC",
}


@dataclass(slots=True)
class PursuerConfig:
    # 追踪机初始状态必须对所有算法相同，否则算法差异会被初始条件差异污染。
    initial_position: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 3.0]))
    initial_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    # yaw=0 表示机头初始朝 +x 方向；pitch=0 表示水平朝前。
    initial_yaw: float = 0.0
    initial_pitch: float = 0.0
    # v_max/a_max 是所有导引算法共享的物理约束，用来保证对比公平。
    v_max: float = 12.0
    a_max: float = 6.0
    # yaw/pitch 角速度限制模拟“机头/相机不能瞬间转向”的现实约束。
    yaw_rate_max: float = np.deg2rad(90.0)
    pitch_rate_max: float = np.deg2rad(60.0)
    # 高度上下限不是完整飞控，只是防止质点模型飞到地下或过高。
    z_min: float = 0.5
    z_max: float = 30.0


@dataclass(slots=True)
class TargetConfig:
    # 静止目标用于验证最基本的收敛能力：算法至少应能飞向一个固定点。
    stationary_position: np.ndarray = field(default_factory=lambda: np.array([40.0, 20.0, 8.0]))
    # 直线目标用于验证算法是否能处理目标速度，而不是只追当前位置。
    linear_position: np.ndarray = field(default_factory=lambda: np.array([25.0, -20.0, 8.0]))
    linear_velocity: np.ndarray = field(default_factory=lambda: np.array([2.0, 1.0, 0.2]))
    # 圆周目标会持续改变 LOS 方向，更容易暴露 FOV 丢失和横向拦截能力差异。
    circle_center: np.ndarray = field(default_factory=lambda: np.array([35.0, 0.0, 8.0]))
    circle_radius: float = 12.0
    circle_omega: float = 0.25
    circle_z_amplitude: float = 2.0
    circle_z_omega: float = 0.15


@dataclass(slots=True)
class GuidanceConfig:
    # direct_* 是基础追踪法的参数：它不是预测拦截，只是把速度指向目标当前位置。
    direct_v_cruise: float = 8.0
    direct_tau: float = 0.8
    # PN 的 N 越大，横向修正越激进；太小会拦截慢，太大可能控制抖动。
    pn_navigation_constant: float = 3.5
    # k_close 和 v_des_along_los 给 PN 加一个沿视线方向的“主动接近”速度目标。
    pn_k_close: float = 1.0
    pn_v_des_along_los: float = 8.0
    # fov_deg 是总视场角，真正判断时会用半角：目标方向夹角 <= fov_deg/2。
    fov_deg: float = 90.0
    # 目标丢失时降低导引增益，表示追踪机对预测目标没有对真实目标那么确信。
    lost_guidance_gain: float = 0.55
    # NMPC 预测窗口：20 步 * 0.1s = 向前看 2 秒；窗口越长越慢。
    horizon_steps: int = 20
    mpc_dt: float = 0.1
    # 下面的权重共同决定 NMPC 的取舍：距离项让它追上目标，FOV 项让它别把目标甩出视场。
    nmpc_w_dist: float = 12.0
    nmpc_w_path: float = 0.1
    nmpc_w_fov: float = 10.0
    # 控制和平滑权重越大，轨迹越保守；太大时可能为了省控制而追不上目标。
    nmpc_w_control: float = 0.015
    nmpc_w_smooth: float = 0.08
    # 偏离 PN 趋势的惩罚让 NMPC 不至于选出完全背离 PN 导引方向的奇怪候选。
    nmpc_w_pn: float = 0.04
    # CBF filter 在接近 FOV 边界前就开始侧向修正，避免等目标出视场后再搜索。
    cbf_fov_margin_deg: float = 8.0
    cbf_gain: float = 0.75
    # MPPI 采样式预测控制参数；采样数越大越稳但越慢，seed 保证对比可复现。
    mppi_samples: int = 48
    mppi_noise_scale: float = 2.5
    mppi_temperature: float = 6.0
    mppi_seed: int = 7

    @property
    def fov_half_angle(self) -> float:
        # 例如 90 度视场表示左右/上下各 45 度范围，因此判断时用半角。
        return np.deg2rad(self.fov_deg * 0.5)


@dataclass(slots=True)
class SimulationConfig:
    # dt 是主仿真步长；减小 dt 通常更平滑但更慢，增大 dt 可能带来积分误差。
    dt: float = 0.05
    # sim_time 是每条算法轨迹的总仿真时长，即使提前捕获也继续记录后续行为。
    sim_time: float = 40.0
    # 距离小于 capture_radius 就认为“捕获”，用于 capture_time 指标和图中捕获点标记。
    capture_radius: float = 1.5
    pursuer: PursuerConfig = field(default_factory=PursuerConfig)
    target: TargetConfig = field(default_factory=TargetConfig)
    guidance: GuidanceConfig = field(default_factory=GuidanceConfig)
