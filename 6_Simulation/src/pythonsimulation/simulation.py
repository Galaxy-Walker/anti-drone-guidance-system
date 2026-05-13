from __future__ import annotations

import numpy as np

from pythonsimulation.config import ALGORITHMS, SimulationConfig
from pythonsimulation.dynamics import initial_pursuer_state, step_pursuer
from pythonsimulation.guidance import GuidanceMemory, compute_guidance
from pythonsimulation.math_utils import clamp_norm, norm
from pythonsimulation.state import SimulationResult
from pythonsimulation.target import target_state


def run_scenario(scenario: str, config: SimulationConfig) -> dict[str, SimulationResult]:
    # 同一个目标场景下依次跑所有算法，保证初始条件和约束完全一致。
    return {algorithm: run_algorithm(scenario, algorithm, config) for algorithm in ALGORITHMS}


def run_algorithm(scenario: str, algorithm: str, config: SimulationConfig) -> SimulationResult:
    # 加上 0.5*dt 是为了避免浮点误差导致最后一个时刻 sim_time 被漏掉。
    times = np.arange(0.0, config.sim_time + config.dt * 0.5, config.dt)
    pursuer = initial_pursuer_state(config)
    memory = GuidanceMemory()

    pursuer_positions: list[np.ndarray] = []
    pursuer_velocities: list[np.ndarray] = []
    target_positions: list[np.ndarray] = []
    target_velocities: list[np.ndarray] = []
    # 这些列表会在循环结束后统一转成 numpy 数组，便于后续批量计算指标和绘图。
    accelerations: list[np.ndarray] = []
    yaws: list[float] = []
    pitches: list[float] = []
    distances: list[float] = []
    visible_values: list[bool] = []
    los_angles: list[float] = []

    for t in times:
        target = target_state(scenario, float(t), config)
        # compute_guidance 只负责“想怎么飞”，不直接改状态；真正积分统一放在 step_pursuer。
        guidance = compute_guidance(algorithm, pursuer, target, memory, config, config.dt)
        # 记录和动力学都使用限幅后的加速度，这样控制能量指标反映真实执行量。
        applied_acceleration = clamp_norm(guidance.acceleration, config.pursuer.a_max)

        # 先记录当前时刻状态，再积分到下一时刻；这样数组下标和 time 一一对应。
        pursuer_positions.append(pursuer.position.copy())
        pursuer_velocities.append(pursuer.velocity.copy())
        target_positions.append(target.position.copy())
        target_velocities.append(target.velocity.copy())
        accelerations.append(applied_acceleration.copy())
        yaws.append(pursuer.yaw)
        pitches.append(pursuer.pitch)
        distances.append(norm(target.position - pursuer.position))
        visible_values.append(guidance.visible)
        los_angles.append(guidance.los_angle)

        # look_at_position 可能是真实目标，也可能是 FOV 丢失后的预测目标。
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
        pitch=np.array(pitches),
        distance=np.array(distances),
        visible=np.array(visible_values, dtype=bool),
        los_angle=np.array(los_angles),
    )
