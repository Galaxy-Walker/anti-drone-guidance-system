"""Velocity controller used by the motion-capture tracking node."""

from dataclasses import dataclass
import math
from typing import Optional, Tuple


Vector2 = Tuple[float, float]
Vector3 = Tuple[float, float, float]


@dataclass(frozen=True)
class TrackerControlConfig:
    """Configuration for horizontal tracking and altitude hold."""

    xy_kp: float = 0.8
    xy_deadband: float = 0.10
    max_xy_speed: float = 0.5
    max_xy_acceleration: float = 0.5
    z_kp: float = 1.0
    max_z_speed: float = 0.3


class TrackerVelocityController:
    """Generate bounded NED velocity commands for tracking and height hold."""

    def __init__(self, config: TrackerControlConfig) -> None:
        """Create a controller with bounded speed and acceleration."""
        self.config = config
        self._validate_config()
        self.last_xy_velocity: Vector2 = (0.0, 0.0)
        self.last_update_at: Optional[float] = None

    def _validate_config(self) -> None:
        if self.config.xy_kp <= 0.0:
            raise ValueError('xy_kp must be greater than zero')
        if self.config.xy_deadband < 0.0:
            raise ValueError('xy_deadband must not be negative')
        if self.config.max_xy_speed <= 0.0:
            raise ValueError('max_xy_speed must be greater than zero')
        if self.config.max_xy_acceleration <= 0.0:
            raise ValueError('max_xy_acceleration must be greater than zero')
        if self.config.z_kp <= 0.0:
            raise ValueError('z_kp must be greater than zero')
        if self.config.max_z_speed <= 0.0:
            raise ValueError('max_z_speed must be greater than zero')

    def reset(self, now: Optional[float] = None) -> None:
        """Reset horizontal command history before velocity control begins."""
        self.last_xy_velocity = (0.0, 0.0)
        self.last_update_at = now

    def update(
        self,
        now: float,
        self_xy: Vector2,
        target_xy: Optional[Vector2],
        current_z: float,
        target_z: float,
    ) -> Vector3:
        """Return a velocity setpoint in the NED frame."""
        desired_xy = self._desired_xy(self_xy, target_xy)
        limited_xy = self._slew_limit(now, desired_xy)
        z_velocity = self._clamp(
            self.config.z_kp * (target_z - current_z),
            self.config.max_z_speed,
        )
        return (limited_xy[0], limited_xy[1], z_velocity)

    def _desired_xy(
            self, self_xy: Vector2,
            target_xy: Optional[Vector2]) -> Vector2:
        if target_xy is None:
            return (0.0, 0.0)

        error = (
            target_xy[0] - self_xy[0],
            target_xy[1] - self_xy[1],
        )
        error_norm = math.hypot(*error)
        if error_norm <= self.config.xy_deadband:
            return (0.0, 0.0)

        velocity = (
            self.config.xy_kp * error[0],
            self.config.xy_kp * error[1],
        )
        speed = math.hypot(*velocity)
        if speed <= self.config.max_xy_speed:
            return velocity
        scale = self.config.max_xy_speed / speed
        return (velocity[0] * scale, velocity[1] * scale)

    def _slew_limit(self, now: float, desired: Vector2) -> Vector2:
        if self.last_update_at is None:
            self.last_update_at = now
            self.last_xy_velocity = (0.0, 0.0)
            return self.last_xy_velocity

        dt = max(0.0, now - self.last_update_at)
        delta = (
            desired[0] - self.last_xy_velocity[0],
            desired[1] - self.last_xy_velocity[1],
        )
        delta_norm = math.hypot(*delta)
        max_delta = self.config.max_xy_acceleration * dt
        if delta_norm > max_delta and delta_norm > 0.0:
            scale = max_delta / delta_norm
            delta = (delta[0] * scale, delta[1] * scale)

        self.last_xy_velocity = (
            self.last_xy_velocity[0] + delta[0],
            self.last_xy_velocity[1] + delta[1],
        )
        self.last_update_at = now
        return self.last_xy_velocity

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))
