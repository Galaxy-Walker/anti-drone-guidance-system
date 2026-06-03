from __future__ import annotations

import numpy as np

from pythonsimulation2d.config import ALGORITHMS, SimulationConfig
from pythonsimulation2d.dynamics import initial_pursuer_state, step_pursuer
from pythonsimulation2d.guidance import GuidanceMemory, compute_guidance
from pythonsimulation2d.math_utils import clamp_norm_xy, norm_xy
from pythonsimulation2d.state import SimulationResult
from pythonsimulation2d.target import target_state


def run_scenario(scenario: str, config: SimulationConfig) -> dict[str, SimulationResult]:
    # 同一个目标场景下依次跑所有算法，保证初始条件和约束完全一致。
    return {algorithm: run_algorithm(scenario, algorithm, config) for algorithm in ALGORITHMS}


def run_algorithm(scenario: str, algorithm: str, config: SimulationConfig) -> SimulationResult:
    times = np.arange(0.0, config.sim_time + config.dt * 0.5, config.dt)
    pursuer = initial_pursuer_state(config)
    memory = GuidanceMemory()

    pursuer_positions: list[np.ndarray] = []
    pursuer_velocities: list[np.ndarray] = []
    target_positions: list[np.ndarray] = []
    target_velocities: list[np.ndarray] = []
    accelerations: list[np.ndarray] = []
    yaws: list[float] = []
    distances: list[float] = []

    for t in times:
        target = target_state(scenario, float(t), config)
        guidance = compute_guidance(algorithm, pursuer, target, memory, config, config.dt)
        applied_acceleration = clamp_norm_xy(guidance.acceleration, config.pursuer.a_max)

        pursuer_positions.append(pursuer.position.copy())
        pursuer_velocities.append(pursuer.velocity.copy())
        target_positions.append(target.position.copy())
        target_velocities.append(target.velocity.copy())
        accelerations.append(applied_acceleration.copy())
        yaws.append(pursuer.yaw)
        distances.append(norm_xy(target.position - pursuer.position))

        pursuer = step_pursuer(pursuer, applied_acceleration, guidance.look_at_position, config, config.dt)
        memory.previous_acceleration = pursuer.acceleration.copy()

    return SimulationResult(
        scenario=scenario,
        algorithm=algorithm,
        time=times,
        pursuer_position=np.array(pursuer_positions),
        pursuer_velocity=np.array(pursuer_velocities),
        target_position=np.array(target_positions),
        target_velocity=np.array(target_velocities),
        acceleration=np.array(accelerations),
        yaw=np.array(yaws),
        distance=np.array(distances),
    )
