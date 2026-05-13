import numpy as np

from .config import TargetMotionConfig


def update_target_state(target_pos, target_vel, time, dt, rng, config=None):
    """
    更新目标状态：教学用 3D 小目标机动，并加入可复现随机噪声。

    target_vel 参数只作为当前速度状态输入；每一步都会按模型重写。默认参数把目标速度
    控制在约 3-5 m/s，适合 40cm 追踪四旋翼的短距拦截教学仿真。
    """
    config = config or TargetMotionConfig()
    noise = rng.normal(0, config.noise_std, 3)
    next_vel = np.asarray(target_vel, dtype=float).copy()
    next_pos = np.asarray(target_pos, dtype=float).copy()

    next_vel[0] = config.forward_speed + noise[0] * config.forward_noise_scale
    next_vel[1] = (
        -config.lateral_speed_amplitude * abs(np.sin(config.lateral_frequency * time))
        + noise[1] * config.lateral_noise_scale
    )
    next_vel[2] = (
        -config.vertical_speed_amplitude * np.sin(config.vertical_frequency * time)
        + noise[2] * config.vertical_noise_scale
    )

    next_pos = next_pos + next_vel * dt
    return next_pos, next_vel
