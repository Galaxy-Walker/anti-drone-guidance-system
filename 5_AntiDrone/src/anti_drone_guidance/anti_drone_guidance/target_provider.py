"""
目标源模块 - 提供目标状态数据

本模块将"目标数据从哪来"与"导引算法如何用"完全解耦。
支持以下目标源类型：
  - SimulatedTarget: 虚拟仿真目标，内置多种预设轨迹，用于离线调试
  - RosTopicTarget: 从 ROS2 话题订阅真实目标位置（如雷达/视觉检测结果）

预设轨迹类型（motion_type）：
  - 'static'          : 静止目标，悬停在指定位置
  - 'line'            : 匀速直线运动
  - 'circle'          : 等高圆周运动
  - 'circle_altitude' : 高度变化的圆周运动（水平圆周 + 正弦高度振荡）

扩展方式：继承 TargetProviderBase 并实现 update() 和 get_state() 方法。
"""

import math
import numpy as np
import logging
from abc import ABC, abstractmethod
from typing import Optional, Callable, Dict, Any

from .pn_guidance_core import State

logger = logging.getLogger(__name__)


# ============================================================
# 预设轨迹默认参数
# ============================================================

PRESET_DEFAULTS: Dict[str, Dict[str, Any]] = {
    'static': {
        'start_position': [50.0, 50.0, -20.0],
    },
    'line': {
        'start_position': [50.0, 0.0, -20.0],
        'line_velocity': [2.0, 2.0, 0.0],
    },
    'circle': {
        'center': [50.0, 50.0],
        'radius': 30.0,
        'omega': 0.1,
        'altitude': -20.0,
    },
    'circle_altitude': {
        'center': [50.0, 50.0],
        'radius': 30.0,
        'omega': 0.1,
        'altitude': -20.0,
        'altitude_amplitude': 5.0,   # 高度振荡幅度 (m)
        'altitude_omega': 0.2,       # 高度振荡角频率 (rad/s)
    },
}


class TargetProviderBase(ABC):
    """
    目标源基类。

    所有目标源（仿真/真实）都需要继承此类并实现以下方法：
      - update(dt): 更新目标状态（仿真时推进模拟，真实时可为空操作）
      - get_state(): 获取目标当前状态
    """

    @abstractmethod
    def update(self, dt: float):
        """
        更新目标状态。

        参数:
            dt: 时间步长 (s)
        """
        pass

    @abstractmethod
    def get_state(self) -> State:
        """
        获取目标当前状态。

        返回:
            State: 目标的位置、速度、速率
        """
        pass

    @abstractmethod
    def is_valid(self) -> bool:
        """
        目标数据是否有效（是否已收到数据）。

        返回:
            bool: True 表示目标数据有效
        """
        pass


class SimulatedTarget(TargetProviderBase):
    """
    虚拟仿真目标。

    支持多种运动模式，用于离线调试和 SITL 仿真测试。

    运动类型:
        'static'          - 静止目标，悬停在 start_position
        'line'            - 匀速直线运动，从 start_position 沿 line_velocity 方向匀速飞行
        'circle'          - 等高圆周运动，在 (center, altitude) 处做水平圆周
        'circle_altitude' - 高度变化的圆周运动，水平做圆周并叠加正弦高度振荡

    参数:
        motion_type: 运动类型（见上）
        center: 圆周运动中心点 [x, y] (m)
        radius: 圆周运动半径 (m)
        omega: 圆周运动角速度 (rad/s)
        initial_phase: 圆周运动初始相位 (rad)
        altitude: 基准飞行高度 (NED 下为负值, m)
        altitude_amplitude: 高度振荡幅度 (m)，仅 circle_altitude 有效
        altitude_omega: 高度振荡角频率 (rad/s)，仅 circle_altitude 有效
        line_velocity: 直线运动速度向量 [vx, vy, vz] (m/s)
        start_position: 初始位置 [x, y, z] (m)
        log_func: 可选的外部日志函数
    """

    def __init__(
        self,
        motion_type: str = 'circle',
        center: Optional[list] = None,
        radius: float = 30.0,
        omega: float = 0.1,
        initial_phase: float = 0.0,
        altitude: float = -20.0,
        altitude_amplitude: float = 5.0,
        altitude_omega: float = 0.2,
        line_velocity: Optional[list] = None,
        start_position: Optional[list] = None,
        log_func: Optional[Callable] = None,
    ):
        self.motion_type = motion_type
        self.center = center or [50.0, 50.0]
        self.radius = radius
        self.omega = omega
        # initial_phase 只影响圆周类轨迹，用于评估矩阵指定目标初始切向方向。
        self.initial_phase = initial_phase
        self.altitude = altitude
        self.altitude_amplitude = altitude_amplitude
        self.altitude_omega = altitude_omega
        self.line_velocity = np.array(line_velocity or [2.0, 2.0, 0.0])
        self.start_position = np.array(start_position or [50.0, 50.0, altitude])
        self._log = log_func or logger.info

        self.sim_time = 0.0
        self._state = State(
            position=self.start_position.copy(),
            velocity=np.zeros(3),
            speed=0.0,
        )
        # 初始化后立即刷新一次，使 t=0 时的圆周目标位置/速度与相位参数一致。
        self._refresh_state()

        # 打印创建信息
        self._log_creation_info()

    # ----------------------------------------------------------------
    # 工厂方法
    # ----------------------------------------------------------------

    @classmethod
    def from_config(
        cls,
        config: Dict[str, Any],
        log_func: Optional[Callable] = None,
    ) -> 'SimulatedTarget':
        """
        从配置字典创建仿真目标（供 ROS2 节点调用）。

        配置字典支持的键与构造函数参数一致。
        未提供的键将从 PRESET_DEFAULTS 中按 motion_type 自动填充。

        参数:
            config: 配置字典，至少需要 'motion_type' 键
            log_func: 可选的外部日志函数

        返回:
            SimulatedTarget 实例
        """
        motion_type = config.get('motion_type', 'circle')
        # 合并预设默认值（预设值优先级低于用户传入值）
        defaults = PRESET_DEFAULTS.get(motion_type, {})
        merged = {**defaults, **config}

        return cls(
            motion_type=motion_type,
            center=merged.get('center'),
            radius=float(merged.get('radius', 30.0)),
            omega=float(merged.get('omega', 0.1)),
            initial_phase=float(merged.get('initial_phase', 0.0)),
            altitude=float(merged.get('altitude', -20.0)),
            altitude_amplitude=float(merged.get('altitude_amplitude', 5.0)),
            altitude_omega=float(merged.get('altitude_omega', 0.2)),
            line_velocity=merged.get('line_velocity'),
            start_position=merged.get('start_position'),
            log_func=log_func,
        )

    # ----------------------------------------------------------------
    # 日志
    # ----------------------------------------------------------------

    def _log_creation_info(self):
        """打印仿真目标创建信息。"""
        base = f"仿真目标已创建: 运动类型={self.motion_type}"
        if self.motion_type == 'static':
            self._log(f"{base}, 位置={self.start_position.tolist()}")
        elif self.motion_type == 'line':
            self._log(f"{base}, 起始位置={self.start_position.tolist()}, "
                       f"速度={self.line_velocity.tolist()}")
        elif self.motion_type == 'circle':
            self._log(f"{base}, 中心={self.center}, "
                       f"半径={self.radius}m, 角速度={self.omega}rad/s, "
                       f"初始相位={self.initial_phase:.3f}rad, 高度={-self.altitude}m")
        elif self.motion_type == 'circle_altitude':
            self._log(f"{base}, 中心={self.center}, "
                       f"半径={self.radius}m, 角速度={self.omega}rad/s, "
                       f"初始相位={self.initial_phase:.3f}rad, "
                       f"基准高度={-self.altitude}m, "
                       f"高度振幅={self.altitude_amplitude}m, "
                       f"高度角频率={self.altitude_omega}rad/s")
        else:
            self._log(f"{base} (未知类型，将视为静止)")

    # ----------------------------------------------------------------
    # 核心更新
    # ----------------------------------------------------------------

    def update(self, dt: float):
        """按照设定的运动模式推进目标状态。"""
        self.sim_time += dt
        self._refresh_state()

    def _refresh_state(self):
        """根据当前仿真时间刷新目标状态，不推进时间。"""
        if self.motion_type == 'static':
            self._update_static()
        elif self.motion_type == 'line':
            self._update_line()
        elif self.motion_type == 'circle':
            self._update_circle()
        elif self.motion_type == 'circle_altitude':
            self._update_circle_altitude()
        else:
            # 未知类型按静止处理
            self._update_static()

        self._state.speed = float(np.linalg.norm(self._state.velocity))

    # ---- 各运动模式的更新实现 ----

    def _update_static(self):
        """静止目标：保持在 start_position 不动。"""
        self._state.velocity = np.zeros(3)

    def _update_line(self):
        """匀速直线运动：从 start_position 沿 line_velocity 方向匀速飞行。"""
        self._state.position = self.start_position + self.line_velocity * self.sim_time
        self._state.velocity = self.line_velocity.copy()

    def _update_circle(self):
        """等高圆周运动：在恒定高度上做匀速圆周运动。"""
        t = self.sim_time
        # phase 表示当前位置半径相位；速度方向自然是该相位的切向方向。
        phase = self.initial_phase + self.omega * t
        cx, cy = self.center
        self._state.position[0] = cx + self.radius * math.cos(phase)
        self._state.position[1] = cy + self.radius * math.sin(phase)
        self._state.position[2] = self.altitude

        self._state.velocity[0] = -self.radius * self.omega * math.sin(phase)
        self._state.velocity[1] = self.radius * self.omega * math.cos(phase)
        self._state.velocity[2] = 0.0

    def _update_circle_altitude(self):
        """
        高度变化的圆周运动：水平面做圆周运动，同时高度做正弦振荡。

        z(t) = altitude - amplitude * sin(altitude_omega * t)
        注意 NED 坐标系中高度增加对应 z 减小（更负），
        所以减去正弦项表示高度上升。
        """
        t = self.sim_time
        # 高度振荡和水平圆周共享仿真时间，但水平初始相位可独立配置。
        phase = self.initial_phase + self.omega * t
        cx, cy = self.center

        # 水平圆周
        self._state.position[0] = cx + self.radius * math.cos(phase)
        self._state.position[1] = cy + self.radius * math.sin(phase)
        # 高度正弦振荡
        self._state.position[2] = (
            self.altitude - self.altitude_amplitude * math.sin(self.altitude_omega * t)
        )

        self._state.velocity[0] = -self.radius * self.omega * math.sin(phase)
        self._state.velocity[1] = self.radius * self.omega * math.cos(phase)
        self._state.velocity[2] = (
            -self.altitude_amplitude * self.altitude_omega * math.cos(self.altitude_omega * t)
        )

    def get_state(self) -> State:
        """获取当前目标状态。"""
        return self._state

    def is_valid(self) -> bool:
        """仿真目标始终有效。"""
        return True


class RosTopicTarget(TargetProviderBase):
    """
    ROS2 话题目标源。

    通过外部设置目标状态（由 ROS2 节点的回调函数调用 set_state）。
    这样 target_provider 本身不依赖 rclpy，保持解耦。

    用法:
        在 ROS2 节点中创建订阅，在回调中调用 target.set_state(pos, vel)
    """

    def __init__(self, timeout_sec: float = 2.0, log_func: Optional[Callable] = None):
        """
        参数:
            timeout_sec: 数据超时时间 (s)，超过此时间未收到更新则认为目标无效
            log_func: 可选的外部日志函数
        """
        self._state = State()
        self._last_update_time = 0.0
        self._elapsed_since_update = float('inf')
        self._timeout_sec = timeout_sec
        self._has_received_data = False
        self._log = log_func or logger.info

        self._log(f"ROS话题目标源已创建: 超时={timeout_sec}s")

    def set_state(self, position: np.ndarray, velocity: np.ndarray):
        """
        由外部回调调用，更新目标状态。

        参数:
            position: 目标位置 [x, y, z]
            velocity: 目标速度 [vx, vy, vz]
        """
        self._state.position = position.copy()
        self._state.velocity = velocity.copy()
        self._state.speed = float(np.linalg.norm(velocity))
        self._elapsed_since_update = 0.0
        self._has_received_data = True

    def update(self, dt: float):
        """更新内部计时器，判断数据是否超时。"""
        self._elapsed_since_update += dt
        if self._has_received_data and self._elapsed_since_update > self._timeout_sec:
            self._log(f"警告: 目标数据已 {self._elapsed_since_update:.1f}s 未更新，可能已丢失目标")

    def get_state(self) -> State:
        """获取目标当前状态。"""
        return self._state

    def is_valid(self) -> bool:
        """目标数据是否有效（已接收且未超时）。"""
        return self._has_received_data and self._elapsed_since_update < self._timeout_sec
