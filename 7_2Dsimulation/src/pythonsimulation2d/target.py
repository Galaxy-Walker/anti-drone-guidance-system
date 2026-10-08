from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from pythonsimulation2d.config import SCENARIOS, SimulationConfig, TableOcclusionConfig
from pythonsimulation2d.math_utils import lock_target_altitude
from pythonsimulation2d.state import TargetState


def target_state(scenario: str, t: float, config: SimulationConfig) -> TargetState:
    target = config.target
    if scenario == "stationary":
        return TargetState(
            lock_target_altitude(target.stationary_position, target.fixed_altitude),
            np.zeros(3),
            np.zeros(3),
        )

    if scenario == "linear":
        position = lock_target_altitude(target.linear_position + target.linear_velocity * t, target.fixed_altitude)
        velocity = target.linear_velocity.copy()
        velocity[2] = 0.0
        return TargetState(position, velocity, np.zeros(3))

    if scenario == "circle":
        phase = target.circle_omega * t
        position = np.array([
            target.circle_center[0] + target.circle_radius * np.cos(phase),
            target.circle_center[1] + target.circle_radius * np.sin(phase),
            target.fixed_altitude,
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

    if scenario == "table_occlusion":
        table = target.table
        arrival_s = table_segment_duration(table.start_x, table.center_x, table)
        if t < arrival_s:
            return _table_segment_state(table.start_x, table.center_x, t, config)
        return _table_segment_state(
            table.center_x, table.end_x, max(0.0, t - arrival_s - table.hover_s), config
        )

    raise ValueError(f"Unknown scenario: {scenario!r}. Expected one of {SCENARIOS}.")


def table_segment_duration(start_x: float, end_x: float, table: TableOcclusionConfig) -> float:
    distance = abs(end_x - start_x)
    speed = min(table.speed, np.sqrt(distance * table.acceleration))
    return distance / speed + speed / table.acceleration if distance > 0.0 else 0.0


def _table_segment_state(start_x: float, end_x: float, t: float, config: SimulationConfig) -> TargetState:
    """梯形速度参考：在停点前减速，避免 0.5 m/s 前馈把目标推过桌下悬停点。"""
    table = config.target.table
    distance = abs(end_x - start_x)
    direction = np.sign(end_x - start_x)
    speed = min(table.speed, np.sqrt(distance * table.acceleration))
    ramp_s = speed / table.acceleration
    duration_s = table_segment_duration(start_x, end_x, table)
    if t <= 0.0:
        displacement, velocity, acceleration = 0.0, 0.0, 0.0
    elif t < ramp_s:
        displacement = 0.5 * table.acceleration * t**2
        velocity, acceleration = table.acceleration * t, table.acceleration
    elif t < duration_s - ramp_s:
        displacement = speed * (t - 0.5 * ramp_s)
        velocity, acceleration = speed, 0.0
    elif t < duration_s:
        remaining_s = duration_s - t
        displacement = distance - 0.5 * table.acceleration * remaining_s**2
        velocity, acceleration = table.acceleration * remaining_s, -table.acceleration
    else:
        displacement, velocity, acceleration = distance, 0.0, 0.0
    return TargetState(
        np.array([start_x + direction * displacement, table.center_y, config.target.fixed_altitude]),
        np.array([direction * velocity, 0.0, 0.0]),
        np.array([direction * acceleration, 0.0, 0.0]),
    )


def target_under_table(positions: np.ndarray, table: TableOcclusionConfig) -> np.ndarray:
    """目标中心处于桌面下方时排除误差样本；不替代真实图像的遮挡判定。"""
    positions = np.asarray(positions, dtype=float)
    return (
        (np.abs(positions[..., 0] - table.center_x) <= table.length * 0.5)
        & (np.abs(positions[..., 1] - table.center_y) <= table.width * 0.5)
        & (positions[..., 2] < table.underside_height)
    )


@dataclass(slots=True)
class TableOcclusionMission:
    """Gazebo 目标机任务：实际停稳连续满 3 秒后才开始出桌段。"""

    config: SimulationConfig
    hover_start_s: float | None = None
    departure_s: float | None = None

    def reference(self, elapsed_s: float, actual: TargetState) -> TargetState:
        table = self.config.target.table
        arrival_s = table_segment_duration(table.start_x, table.center_x, table)
        if self.departure_s is not None:
            return _table_segment_state(table.center_x, table.end_x, elapsed_s - self.departure_s, self.config)
        if elapsed_s < arrival_s:
            return _table_segment_state(table.start_x, table.center_x, elapsed_s, self.config)

        stop = _table_segment_state(table.start_x, table.center_x, arrival_s, self.config)
        stable = (
            np.linalg.norm(actual.position - stop.position) <= table.position_tolerance
            and np.linalg.norm(actual.velocity) <= table.velocity_tolerance
        )
        if not stable:
            self.hover_start_s = None
        elif self.hover_start_s is None:
            self.hover_start_s = elapsed_s
        elif elapsed_s - self.hover_start_s >= table.hover_s:
            self.departure_s = elapsed_s
        return stop


def generate_target_trajectory(scenario: str, config: SimulationConfig) -> tuple[np.ndarray, np.ndarray]:
    times = np.arange(0.0, config.sim_time + config.dt * 0.5, config.dt)
    positions = np.array([target_state(scenario, t, config).position for t in times])
    return times, positions
