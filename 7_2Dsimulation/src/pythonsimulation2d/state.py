from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(slots=True)
class PursuerState:
    # position/velocity/acceleration 仍是三维向量，便于保留定高展示；算法只使用 XY。
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    # yaw 表示水平面内机头朝向。
    yaw: float

    def copy(self) -> "PursuerState":
        return PursuerState(
            self.position.copy(),
            self.velocity.copy(),
            self.acceleration.copy(),
            self.yaw,
        )


@dataclass(slots=True)
class TargetState:
    # 目标不受追踪机影响，它的状态完全由 target.py 中的地面轨迹给出。
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray

    def copy(self) -> "TargetState":
        return TargetState(self.position.copy(), self.velocity.copy(), self.acceleration.copy())


@dataclass(slots=True)
class SimulationResult:
    # SimulationResult 是一次“场景 + 算法”的完整时间序列结果，后续指标和绘图都只读它。
    scenario: str
    algorithm: str
    time: np.ndarray
    pursuer_position: np.ndarray
    pursuer_velocity: np.ndarray
    target_position: np.ndarray
    target_velocity: np.ndarray
    acceleration: np.ndarray
    yaw: np.ndarray
    distance: np.ndarray
