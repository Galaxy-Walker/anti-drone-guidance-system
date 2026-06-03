from __future__ import annotations

import numpy as np

from pythonsimulation2d.config import SimulationConfig
from pythonsimulation2d.math_utils import clamp_norm_xy, lock_pursuer_altitude, update_yaw_toward
from pythonsimulation2d.state import PursuerState


def initial_pursuer_state(config: SimulationConfig) -> PursuerState:
    pursuer = config.pursuer
    position = lock_pursuer_altitude(pursuer.initial_position, pursuer.fixed_altitude)
    velocity = clamp_norm_xy(pursuer.initial_velocity, pursuer.v_max)
    return PursuerState(
        position=position,
        velocity=velocity,
        acceleration=np.zeros(3),
        yaw=pursuer.initial_yaw,
    )


def step_pursuer(
    state: PursuerState,
    acceleration_cmd: np.ndarray,
    look_at_position: np.ndarray,
    config: SimulationConfig,
    dt: float,
) -> PursuerState:
    pursuer = config.pursuer
    # 2D 定高模型：只执行水平加速度，z 方向速度/加速度始终为 0。
    acceleration = clamp_norm_xy(acceleration_cmd, pursuer.a_max)
    velocity = clamp_norm_xy(state.velocity + acceleration * dt, pursuer.v_max)
    position = lock_pursuer_altitude(state.position + velocity * dt, pursuer.fixed_altitude)

    # 机头朝向由 look_at_position 的水平投影决定。
    yaw = update_yaw_toward(
        state.yaw,
        state.position,
        look_at_position,
        pursuer.yaw_rate_max,
        dt,
    )
    return PursuerState(position, velocity, acceleration, yaw)
