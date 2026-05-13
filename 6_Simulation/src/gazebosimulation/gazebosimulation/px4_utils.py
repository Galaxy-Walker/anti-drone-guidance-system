"""供 `guidance_node` 使用的 PX4 消息构造工具。

把消息构造集中在这里，可以让控制节点更容易阅读，并把 Offboard 自定义模式值
等 PX4 细节隔离在算法桥接层之外。这些工具函数只填充消息，不负责发布。
"""

from __future__ import annotations

import math
from collections.abc import Sequence

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand


def namespaced_topic(namespace: str, suffix: str) -> str:
    """将 PX4 namespace 和话题后缀拼成规范的绝对话题名。"""
    # 用户传入的 namespace 可能带斜杠也可能不带；这里统一规范化，保证
    # `/px4_1` + `fmu/in/...` 和 `px4_1/` + `/fmu/in/...` 结果一致。
    return f"/{namespace.strip('/')}/{suffix.lstrip('/')}"


def timestamp_us(node) -> int:
    """返回微秒级 ROS 时钟时间，用于匹配 PX4 消息时间戳。"""
    return int(node.get_clock().now().nanoseconds // 1000)


def offboard_control_mode(
    timestamp: int,
    *,
    position: bool = False,
    velocity: bool = True,
    acceleration: bool = True,
) -> OffboardControlMode:
    """根据启用的 setpoint 字段构造 OffboardControlMode 心跳消息。"""
    message = OffboardControlMode()
    message.timestamp = timestamp
    # PX4 通过这些布尔量判断 TrajectorySetpoint 中哪些字段由外部控制；
    # 未启用的字段应在 setpoint 中保持 NaN。
    message.position = position
    message.velocity = velocity
    message.acceleration = acceleration
    # 本包不直接控制姿态或机体系角速度。
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
    """构造 PX4 TrajectorySetpoint，并把未使用的向量字段置为 NaN。"""
    message = TrajectorySetpoint()
    message.timestamp = timestamp
    # PX4 会把 NaN 字段理解为“不控制该维度”。这样目标机可以使用位置+速度控制，
    # 追踪机可以使用速度+加速度控制，同时复用同一个构造函数。
    message.position = _vector_or_nan(position)
    message.velocity = _vector_or_nan(velocity)
    message.acceleration = _vector_or_nan(acceleration)
    message.yaw = float(yaw) if yaw is not None else math.nan
    message.yawspeed = float(yawspeed) if yawspeed is not None else math.nan
    return message


def arm_command(timestamp: int, target_system: int = 1, arm: bool = True) -> VehicleCommand:
    """为指定 PX4 实例构造 MAVLink 解锁/上锁 VehicleCommand。"""
    return vehicle_command(
        timestamp,
        VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
        target_system=target_system,
        param1=1.0 if arm else 0.0,
    )


def offboard_mode_command(timestamp: int, target_system: int = 1) -> VehicleCommand:
    """构造让 PX4 切入 Offboard 模式的命令。"""
    return vehicle_command(
        timestamp,
        VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
        target_system=target_system,
        # param1=1 表示启用 MAV_MODE_FLAG_CUSTOM_MODE_ENABLED。
        param1=1.0,
        # PX4 自定义主模式 6 对应 Offboard。
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
    """构造通用 VehicleCommand，并暴露全部 MAVLink 参数。"""
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
    # 告诉 PX4 该命令来自伴随计算机或外部 ROS 节点。
    message.from_external = True
    return message


def _vector_or_nan(value: Sequence[float] | None) -> list[float]:
    """返回三维向量，或返回 PX4 用于未启用 setpoint 向量的 NaN 哨兵值。"""
    if value is None:
        return [math.nan, math.nan, math.nan]
    return [float(value[0]), float(value[1]), float(value[2])]
