"""`use_sim_time=true` 时的启动防呆。

视觉闭环要求节点时钟与图像 stamp 同源（Gazebo `/clock`）。如果 `/clock` 没有桥接，
rclpy 的 ROS 时间会一直停在 0：

- 基于 ROS 时间的定时器永远不会触发（导引节点静默悬停，也不打日志）；
- 检测节点会把所有帧当成过期帧丢弃，适配节点会拒绝所有量测。

因此本模块的检查必须挂在**墙钟定时器**上，而不是节点的普通定时器。`SimClockGuard`
只判断“是否收到过非零时钟”，`create_sim_clock_guard_timer` 负责在宽限期后报错退出，
并在确认时钟存在后自动销毁自己。
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

DEFAULT_GRACE_S = 2.0
DEFAULT_CHECK_PERIOD_S = 0.5


@dataclass(slots=True)
class SimClockGuard:
    """节点时钟推进检查器；`check()` 返回 None 表示正常，否则返回错误说明。"""

    grace_s: float = DEFAULT_GRACE_S
    _start_mono_ns: int = field(default=0, repr=False)
    _confirmed: bool = field(default=False, repr=False)

    def __post_init__(self) -> None:
        self._start_mono_ns = time.monotonic_ns()

    @property
    def confirmed(self) -> bool:
        return self._confirmed

    def check(self, clock_now_ns: int) -> str | None:
        if self._confirmed:
            return None
        if clock_now_ns > 0:
            # 收到过非零 /clock 即可确认时钟源存在，不要求此刻仍在推进（允许暂停）。
            self._confirmed = True
            return None
        if (time.monotonic_ns() - self._start_mono_ns) * 1e-9 >= self.grace_s:
            return (
                f"use_sim_time=true 但 {self.grace_s:.0f}s 内节点时钟仍为 0："
                "没有收到 /clock，请确认 ros_gz_bridge 的 /clock 桥接已启动"
            )
        return None


def create_sim_clock_guard_timer(node, guard: SimClockGuard, period_s: float = DEFAULT_CHECK_PERIOD_S):
    """在节点上挂一个墙钟定时器驱动 guard；确认或报错后自动销毁。

    返回持有的定时器句柄字典，调用方不需要保存，但便于测试断言。
    """
    from rclpy.clock import Clock, ClockType

    holder: dict[str, object] = {"timer": None}

    def tick() -> None:
        reason = guard.check(int(node.get_clock().now().nanoseconds))
        timer = holder.get("timer")
        if reason is not None:
            node.get_logger().fatal(reason)
            if timer is not None:
                node.destroy_timer(timer)
                holder["timer"] = None
            import rclpy

            rclpy.shutdown()
            return
        if guard.confirmed and timer is not None:
            node.destroy_timer(timer)
            holder["timer"] = None

    holder["timer"] = node.create_timer(
        period_s,
        tick,
        clock=Clock(clock_type=ClockType.STEADY_TIME),
    )
    return holder
