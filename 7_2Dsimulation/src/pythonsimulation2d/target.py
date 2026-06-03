from __future__ import annotations

import numpy as np

from pythonsimulation2d.config import SCENARIOS, SimulationConfig
from pythonsimulation2d.math_utils import lock_ground_target
from pythonsimulation2d.state import TargetState


def target_state(scenario: str, t: float, config: SimulationConfig) -> TargetState:
    target = config.target
    if scenario == "stationary":
        return TargetState(lock_ground_target(target.stationary_position), np.zeros(3), np.zeros(3))

    if scenario == "linear":
        position = lock_ground_target(target.linear_position + target.linear_velocity * t)
        velocity = target.linear_velocity.copy()
        velocity[2] = 0.0
        return TargetState(position, velocity, np.zeros(3))

    if scenario == "circle":
        phase = target.circle_omega * t
        position = np.array([
            target.circle_center[0] + target.circle_radius * np.cos(phase),
            target.circle_center[1] + target.circle_radius * np.sin(phase),
            0.0,
        ])
        velocity = np.array([
            -target.circle_radius * target.circle_omega * np.sin(phase),
            target.circle_radius * target.circle_omega * np.cos(phase),
            0.0,
        ])
        acceleration = np.array([
            -target.circle_radius * target.circle_omega**2 * np.cos(phase),
            -target.circle_radius * target.circle_omega**2 * np.sin(phase),
            0.0,
        ])
        return TargetState(position, velocity, acceleration)

    raise ValueError(f"Unknown scenario: {scenario!r}. Expected one of {SCENARIOS}.")


def generate_target_trajectory(scenario: str, config: SimulationConfig) -> tuple[np.ndarray, np.ndarray]:
    times = np.arange(0.0, config.sim_time + config.dt * 0.5, config.dt)
    positions = np.array([target_state(scenario, t, config).position for t in times])
    return times, positions
