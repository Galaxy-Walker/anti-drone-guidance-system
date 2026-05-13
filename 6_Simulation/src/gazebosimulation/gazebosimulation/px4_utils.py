from __future__ import annotations

import math
from collections.abc import Sequence

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand


def namespaced_topic(namespace: str, suffix: str) -> str:
    return f"/{namespace.strip('/')}/{suffix.lstrip('/')}"


def timestamp_us(node) -> int:
    return int(node.get_clock().now().nanoseconds // 1000)


def offboard_control_mode(
    timestamp: int,
    *,
    position: bool = False,
    velocity: bool = True,
    acceleration: bool = True,
) -> OffboardControlMode:
    message = OffboardControlMode()
    message.timestamp = timestamp
    message.position = position
    message.velocity = velocity
    message.acceleration = acceleration
    message.attitude = False
    message.body_rate = False
    return message


def trajectory_setpoint(
    timestamp: int,
    *,
    position: Sequence[float] | None = None,
    velocity: Sequence[float] | None = None,
    acceleration: Sequence[float] | None = None,
    yaw: float | None = None,
    yawspeed: float | None = None,
) -> TrajectorySetpoint:
    message = TrajectorySetpoint()
    message.timestamp = timestamp
    message.position = _vector_or_nan(position)
    message.velocity = _vector_or_nan(velocity)
    message.acceleration = _vector_or_nan(acceleration)
    message.yaw = float(yaw) if yaw is not None else math.nan
    message.yawspeed = float(yawspeed) if yawspeed is not None else math.nan
    return message


def arm_command(timestamp: int, target_system: int = 1, arm: bool = True) -> VehicleCommand:
    return vehicle_command(
        timestamp,
        VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
        target_system=target_system,
        param1=1.0 if arm else 0.0,
    )


def offboard_mode_command(timestamp: int, target_system: int = 1) -> VehicleCommand:
    return vehicle_command(
        timestamp,
        VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
        target_system=target_system,
        param1=1.0,
        param2=6.0,
    )


def vehicle_command(
    timestamp: int,
    command: int,
    *,
    target_system: int = 1,
    target_component: int = 1,
    source_system: int = 1,
    source_component: int = 1,
    param1: float = 0.0,
    param2: float = 0.0,
    param3: float = 0.0,
    param4: float = 0.0,
    param5: float = 0.0,
    param6: float = 0.0,
    param7: float = 0.0,
) -> VehicleCommand:
    message = VehicleCommand()
    message.timestamp = timestamp
    message.param1 = float(param1)
    message.param2 = float(param2)
    message.param3 = float(param3)
    message.param4 = float(param4)
    message.param5 = float(param5)
    message.param6 = float(param6)
    message.param7 = float(param7)
    message.command = int(command)
    message.target_system = int(target_system)
    message.target_component = int(target_component)
    message.source_system = int(source_system)
    message.source_component = int(source_component)
    message.from_external = True
    return message


def _vector_or_nan(value: Sequence[float] | None) -> list[float]:
    if value is None:
        return [math.nan, math.nan, math.nan]
    return [float(value[0]), float(value[1]), float(value[2])]
