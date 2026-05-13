from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(slots=True)
class PursuerState:
    # position/velocity/acceleration 都是三维向量，单位分别是 m、m/s、m/s^2。
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    # yaw/pitch 不是完整姿态动力学，只用于表示相机/机头朝向以计算 FOV。
    yaw: float
    pitch: float

    def copy(self) -> "PursuerState":
        # numpy 数组是可变对象，必须 copy，避免预测仿真意外改到真实仿真状态。
        return PursuerState(
            self.position.copy(),
            self.velocity.copy(),
            self.acceleration.copy(),
            self.yaw,
            self.pitch,
        )


@dataclass(slots=True)
class TargetState:
    # 目标不受追踪机影响，它的状态完全由 target.py 中的预设轨迹给出。
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray

    def copy(self) -> "TargetState":
    # FOV 的最后观测记忆会长期保存目标状态，所以也要复制数组，避免引用共享。
        return TargetState(self.position.copy(), self.velocity.copy(), self.acceleration.copy())


@dataclass(slots=True)
class SimulationResult:
    # SimulationResult 是一次“场景 + 算法”的完整时间序列结果，后续指标和绘图都只读它。
    scenario: str
    algorithm: str
    # time 的长度与下面每个数组第一维一致；第 i 行就是 time[i] 时刻的数据。
    time: np.ndarray
    pursuer_position: np.ndarray
    pursuer_velocity: np.ndarray
    target_position: np.ndarray
    target_velocity: np.ndarray
    acceleration: np.ndarray
    yaw: np.ndarray
    pitch: np.ndarray
    distance: np.ndarray
    visible: np.ndarray
    los_angle: np.ndarray
