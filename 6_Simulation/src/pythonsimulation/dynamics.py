from __future__ import annotations

import numpy as np

from pythonsimulation.config import SimulationConfig
from pythonsimulation.math_utils import clamp, clamp_norm, update_angles_toward
from pythonsimulation.state import PursuerState


def initial_pursuer_state(config: SimulationConfig) -> PursuerState:
    pursuer = config.pursuer
    return PursuerState(
        position=pursuer.initial_position.copy(),
        velocity=pursuer.initial_velocity.copy(),
        acceleration=np.zeros(3),
        yaw=pursuer.initial_yaw,
        pitch=pursuer.initial_pitch,
    )


def step_pursuer(
    state: PursuerState,
    acceleration_cmd: np.ndarray,
    look_at_position: np.ndarray,
    config: SimulationConfig,
    dt: float,
) -> PursuerState:
    pursuer = config.pursuer
    # 先限制控制输入，再用简化质点模型积分：v[k+1] = v[k] + a[k] * dt。
    acceleration = clamp_norm(acceleration_cmd, pursuer.a_max)
    velocity = clamp_norm(state.velocity + acceleration * dt, pursuer.v_max)
    # 使用更新后的速度积分位置，和计划文档中的离散模型保持一致。
    position = state.position + velocity * dt

    # 高度越界后把垂向速度清零，否则下一步还会继续“顶着边界”积分。
    position[2] = clamp(position[2], pursuer.z_min, pursuer.z_max)
    if position[2] <= pursuer.z_min + 1e-9 and velocity[2] < 0.0:
        velocity[2] = 0.0
    if position[2] >= pursuer.z_max - 1e-9 and velocity[2] > 0.0:
        velocity[2] = 0.0

    # 相机/机头朝向由 look_at_position 决定，FOV 算法会传入真实或预测目标点。
    yaw, pitch = update_angles_toward(
        state.yaw,
        state.pitch,
        state.position,
        look_at_position,
        pursuer.yaw_rate_max,
        pursuer.pitch_rate_max,
        dt,
    )
    return PursuerState(position, velocity, acceleration, yaw, pitch)
