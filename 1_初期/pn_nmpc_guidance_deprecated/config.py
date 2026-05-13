"""PN/NMPC 教学仿真的集中参数配置。

本文件把原先散落在 main.py、simulation.py、speed_control.py、target_model.py
和 nmpc.py 里的数字集中起来。新手调参时优先看这里：带单位的是物理/场景量，
带 weight 的是 NMPC 代价函数权重，通常只在算法调试时调整。
"""

from dataclasses import dataclass, field


DEFAULT_RANDOM_SEED = 42


@dataclass
class ScenarioConfig:
    """仿真场景初值，单位统一为 m、m/s、s。

    默认场景把目标放在几十米量级，而不是几百米外。这样更贴近 40cm 级四旋翼
    常见视觉/短距感知验证距离，也能让一轮教学仿真很快跑完。
    """

    target_start: tuple[float, float, float] = (40.0, 30.0, 15.0)
    tracker_start: tuple[float, float, float] = (0.0, 0.0, 1.0)
    target_velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)
    tracker_initial_speed: float = 0.0
    max_time: float = 45.0


@dataclass
class VehicleConfig:
    """40cm 追踪四旋翼的近似能力约束。

    这些参数影响“真实感”：速度/加速度越大越容易追上目标，但越不像小型四旋翼。
    capture_radius 略大于机体直径，是教学仿真里的命中判定半径，并不是碰撞模型。
    """

    diameter: float = 0.40
    speed_min: float = 0.5
    speed_max: float = 12.0
    speed_accel_limit: float = 4.0
    accel_max: float = 6.0
    capture_radius: float = 0.5
    fov_horizontal_deg: float = 70.0
    fov_vertical_deg: float = 55.0


@dataclass
class GuidanceConfig:
    """PN/NMPC 主循环参数。

    N 是比例导引系数，越大越激进；dt 是离散仿真的步长；horizon_steps 是 NMPC
    每次向前看的步数。默认预测窗约 0.3s，适合当前轻量枚举式 NMPC。
    """

    N: float = 3.5
    dt: float = 0.02
    speed_strategy: str = "adaptive"
    use_nmpc: bool = True
    horizon_steps: int = 15


@dataclass
class SpeedControlConfig:
    """速度策略参数，负责把相对几何量变成期望标量速度。

    distance_full_speed 表示超过该距离时倾向使用最高速度；接近目标后速度会自然降低。
    closing_boost 用于在接近速度不足时补偿；omega_penalty 用于视线转动太快时降速防过冲。
    """

    distance_full_speed: float = 60.0
    no_closing_boost: float = 1.35
    weak_closing_boost: float = 1.15
    weak_closing_ratio: float = 0.35
    omega_penalty_gain: float = 3.0
    omega_penalty_min: float = 0.75
    current_speed_floor_ratio: float = 0.8
    target_speed_margin: float = 1.1
    pursuit_speed_advantage_ratio: float = 1.5
    pursuit_high_los_rate: float = 0.1
    pursuit_turn_boost: float = 1.2


@dataclass
class TargetMotionConfig:
    """教学用目标机动模型参数。

    这里的目标不是要模拟大型高速飞行器，而是给 40cm 追踪机提供一个 3-5m/s
    量级、带轻微横向和高度机动的小目标。noise_std 是速度扰动标准差。
    """

    forward_speed: float = -3.5
    lateral_speed_amplitude: float = 2.5
    vertical_speed_amplitude: float = 1.0
    lateral_frequency: float = 0.4
    vertical_frequency: float = 0.35
    noise_std: float = 0.35
    forward_noise_scale: float = 0.3
    lateral_noise_scale: float = 1.0
    vertical_noise_scale: float = 0.5


@dataclass
class NMPCConfig:
    """轻量 NMPC 的约束和代价函数权重。

    约束项 speed/accel/FOV/capture 来自 40cm 追踪机能力；weight 项只影响候选控制
    的排序，不直接代表物理单位。调试时通常先调物理约束，再微调权重。
    """

    horizon_steps: int = 15
    prediction_dt: float | None = None
    accel_max: float = 6.0
    speed_min: float = 0.5
    speed_max: float = 12.0
    fov_horizontal_deg: float = 70.0
    fov_vertical_deg: float = 55.0
    capture_radius: float = 0.5
    terminal_distance_weight: float = 9.0
    running_distance_weight: float = 0.1
    pn_direction_weight: float = 2.0
    intercept_point_weight: float = 0.02
    fov_weight: float = 70.0
    control_weight: float = 0.08


@dataclass
class LoggingConfig:
    """仿真日志参数。

    log_to_file 为 None 时不写文件；设为相对路径时，会写到 pn_nmpc_guidance 模块目录下。
    想减少控制台输出，可把 log_level 改成 "INFO"；想看每步诊断细节，则改成 "DEBUG"。
    """

    enable_logging: bool = True
    log_level: str = "INFO"
    log_interval: float = 0.5
    log_to_file: str | None = None


@dataclass
class SimulationConfig:
    """完整仿真配置入口。

    from_config 会以 vehicle 为物理约束源，并把速度、加速度、FOV、捕获半径同步到
    NMPCConfig 中，避免同一个物理量在多个地方改漏。
    """

    scenario: ScenarioConfig = field(default_factory=ScenarioConfig)
    vehicle: VehicleConfig = field(default_factory=VehicleConfig)
    guidance: GuidanceConfig = field(default_factory=GuidanceConfig)
    speed_control: SpeedControlConfig = field(default_factory=SpeedControlConfig)
    target_motion: TargetMotionConfig = field(default_factory=TargetMotionConfig)
    nmpc: NMPCConfig = field(default_factory=NMPCConfig)
    logging: LoggingConfig = field(default_factory=LoggingConfig)
    random_seed: int = DEFAULT_RANDOM_SEED


DEFAULT_40CM_CONFIG = SimulationConfig()
