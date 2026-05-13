from __future__ import annotations

import numpy as np

from pythonsimulation.config import SCENARIOS, SimulationConfig
from pythonsimulation.state import TargetState


def target_state(scenario: str, t: float, config: SimulationConfig) -> TargetState:
    target = config.target
    if scenario == "stationary":
        # 静止目标：位置固定，速度和加速度都为 0。
        return TargetState(target.stationary_position.copy(), np.zeros(3), np.zeros(3))

    if scenario == "linear":
        # 匀速直线目标：p(t) = p0 + v * t。
        position = target.linear_position + target.linear_velocity * t
        return TargetState(position.copy(), target.linear_velocity.copy(), np.zeros(3))

    if scenario == "circle":
        # 圆周目标：xy 平面做圆周运动，z 方向叠加一个正弦高度变化。
        phase = target.circle_omega * t
        z_phase = target.circle_z_omega * t
        position = np.array([
            target.circle_center[0] + target.circle_radius * np.cos(phase),
            target.circle_center[1] + target.circle_radius * np.sin(phase),
            target.circle_center[2] + target.circle_z_amplitude * np.sin(z_phase),
        ])
        velocity = np.array([
            -target.circle_radius * target.circle_omega * np.sin(phase),
            target.circle_radius * target.circle_omega * np.cos(phase),
            target.circle_z_amplitude * target.circle_z_omega * np.cos(z_phase),
        ])
        acceleration = np.array([
            -target.circle_radius * target.circle_omega**2 * np.cos(phase),
            -target.circle_radius * target.circle_omega**2 * np.sin(phase),
            -target.circle_z_amplitude * target.circle_z_omega**2 * np.sin(z_phase),
        ])
        return TargetState(position, velocity, acceleration)

    raise ValueError(f"Unknown scenario: {scenario!r}. Expected one of {SCENARIOS}.")


def generate_target_trajectory(scenario: str, config: SimulationConfig) -> tuple[np.ndarray, np.ndarray]:
    times = np.arange(0.0, config.sim_time + config.dt * 0.5, config.dt)
    positions = np.array([target_state(scenario, t, config).position for t in times])
    return times, positions
