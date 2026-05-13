import numpy as np

from .config import SpeedControlConfig


VALID_SPEED_STRATEGIES = ("adaptive", "pursuit")


def validate_speed_strategy(strategy):
    """确认速度策略仍是当前保留的实现之一。"""
    if strategy in VALID_SPEED_STRATEGIES:
        return strategy
    valid = ", ".join(VALID_SPEED_STRATEGIES)
    raise ValueError(f"不支持的速度策略: {strategy!r}。当前仅支持: {valid}")


def compute_desired_speed(
    strategy,
    R_mag,
    Vc,
    Omega_mag,
    target_speed,
    current_speed,
    initial_speed,
    speed_min,
    speed_max,
    config=None,
):
    """
    根据保留的速度策略计算期望速度。

    这里输出的是“想要的标量速度”，不是三维速度向量。真正的方向由 PN/NMPC 决定。
    返回值为 (clipped_speed, debug_info)。
    """
    validate_speed_strategy(strategy)
    config = config or SpeedControlConfig()

    if strategy == "adaptive":
        dist_speed = np.interp(
            np.clip(R_mag, 0.0, config.distance_full_speed),
            [0.0, config.distance_full_speed],
            [speed_min, speed_max],
        )

        if Vc <= 0.0:
            closing_boost = config.no_closing_boost
        elif Vc < config.weak_closing_ratio * max(dist_speed, speed_min):
            closing_boost = config.weak_closing_boost
        else:
            closing_boost = 1.0

        omega_penalty = np.clip(
            1.0 - Omega_mag * config.omega_penalty_gain,
            config.omega_penalty_min,
            1.0,
        )
        speed_floor = max(
            speed_min,
            config.current_speed_floor_ratio * current_speed,
            config.target_speed_margin * target_speed,
        )
        desired_speed = max(dist_speed * closing_boost * omega_penalty, speed_floor)
        debug_factors = {
            "dist_speed": float(dist_speed),
            "closing_boost": float(closing_boost),
            "omega_penalty": float(omega_penalty),
            "speed_floor": float(speed_floor),
            "distance_full_speed": float(config.distance_full_speed),
        }
    else:
        base_speed = target_speed * config.pursuit_speed_advantage_ratio
        if Omega_mag > config.pursuit_high_los_rate:
            desired_speed = base_speed * config.pursuit_turn_boost
        else:
            desired_speed = base_speed
        debug_factors = {
            "base_speed": float(base_speed),
            "speed_advantage_ratio": config.pursuit_speed_advantage_ratio,
            "pursuit_turn_boost": config.pursuit_turn_boost,
        }

    clipped_speed = float(np.clip(desired_speed, speed_min, speed_max))
    debug_info = {
        "strategy": strategy,
        "raw_desired_speed": float(desired_speed),
        "clipped_desired_speed": clipped_speed,
        "R_mag": float(R_mag),
        "Vc": float(Vc),
        "Omega_mag": float(Omega_mag),
        "target_speed": float(target_speed),
        "initial_speed": float(initial_speed),
        **debug_factors,
    }
    return clipped_speed, debug_info


def update_speed(current_speed, desired_speed, dt, acceleration_limit=4.0):
    """
    平滑更新速度（避免突变）。

    acceleration_limit 是标量速度变化率上限，40cm 四旋翼默认用 4 m/s^2，
    防止速度指令一帧内跳变得过于理想化。
    返回值为 (updated_speed, debug_info)。
    """
    speed_diff = desired_speed - current_speed
    max_speed_change = acceleration_limit * dt

    if abs(speed_diff) > max_speed_change:
        speed_diff = np.sign(speed_diff) * max_speed_change

    updated_speed = current_speed + speed_diff
    debug_info = {
        "desired_speed": float(desired_speed),
        "actual_speed": float(updated_speed),
        "speed_diff_applied": float(speed_diff),
        "max_speed_change": float(max_speed_change),
        "acceleration_limit": float(acceleration_limit),
    }
    return updated_speed, debug_info
