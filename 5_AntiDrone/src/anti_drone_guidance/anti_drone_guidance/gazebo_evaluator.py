#!/usr/bin/env python3
"""PX4 Gazebo 闭环评估器。

本工具只负责评估流程编排，不直接启动或控制 Gazebo/PX4 本体：
1. 等待外部已经启动好的 PX4 uXRCE-DDS 话题；
2. 为每个评估 case 生成临时参数文件并启动导引节点；
3. 同时启动 rosbag 记录 PX4 和评估话题；
4. 订阅真实追踪机状态和虚拟目标真值，在线计算控制性能指标；
5. 停止本 case 的子进程，写入 CSV，并提示用户重启外部仿真。
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from typing import Iterable

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from px4_msgs.msg import VehicleLocalPosition
from std_msgs.msg import String


PX4_LOCAL_POSITION_TOPICS = [
    "/fmu/out/vehicle_local_position_v1",
    "/fmu/out/vehicle_local_position",
]
TARGET_STATE_TOPIC = "/anti_drone_guidance/target_state"
EVENT_TOPIC = "/anti_drone_guidance/evaluation_event"

DEFAULT_BAG_TOPICS = [
    *PX4_LOCAL_POSITION_TOPICS,
    "/fmu/out/vehicle_status_v1",
    "/fmu/out/vehicle_status",
    "/fmu/out/vehicle_control_mode",
    "/fmu/in/trajectory_setpoint",
    TARGET_STATE_TOPIC,
    EVENT_TOPIC,
]

CSV_FIELDS = [
    "case_id",
    "target_motion",
    "target_heading_deg",
    "speed_max",
    "success",
    "min_distance",
    "intercept_time",
    "max_actual_acceleration",
    "final_distance",
    "takeoff_time",
    "guidance_time",
    "total_time",
    "bag_path",
]


@dataclass(frozen=True)
class CaseConfig:
    """单个评估 case 的固定配置。

    line_vx/line_vy 用于直线目标；initial_phase 用于圆周目标，使同一个
    heading 参数可以同时表达直线速度方向和圆周初始切向方向。
    """

    case_id: str
    target_motion: str
    target_heading_deg: float
    speed_max: float
    line_vx: float
    line_vy: float
    initial_phase: float


@dataclass
class CaseMetrics:
    """评估器实时累计的指标。

    时间类指标全部使用 time.monotonic()，避免系统时间跳变影响 case 统计；
    PX4 加速度指标单独使用 PX4 消息时间戳差分速度，以贴近飞控真实状态。
    """

    success: bool = False
    min_distance: float = math.inf
    intercept_time: float = math.nan
    max_actual_acceleration: float = 0.0
    final_distance: float = math.nan
    takeoff_time: float = math.nan
    guidance_time: float = math.nan
    total_time: float = math.nan


class EvaluationMonitor(Node):
    """订阅评估所需话题，并在 ROS spin 过程中持续更新指标。"""

    def __init__(self, intercept_radius: float):
        super().__init__("gazebo_evaluator_monitor")
        self.intercept_radius = intercept_radius
        self.current_case_id = ""

        self.local_position: np.ndarray | None = None
        self.local_velocity: np.ndarray | None = None
        self.active_local_position_topic: str | None = None
        self.target_position: np.ndarray | None = None
        self.target_velocity: np.ndarray | None = None

        self.last_px4_message_time: float | None = None
        self.case_started_at: float | None = None
        self.guidance_started_at: float | None = None
        self.finished_at: float | None = None
        self.last_event_by_name: dict[str, float] = {}
        self.last_event_payload: dict[str, object] = {}

        self.metrics = CaseMetrics()
        # 保存上一帧 PX4 速度，用真实状态速度差分计算实际加速度峰值。
        self._last_velocity_for_accel: np.ndarray | None = None
        self._last_velocity_time: float | None = None

        # PX4 bridge 的 out 话题通常是 BEST_EFFORT。Durability 选择 VOLATILE
        # 是更宽松的请求，可兼容 volatile 和 transient_local 发布端，避免评估器
        # 因 QoS 严格匹配失败而一直等不到位置消息。
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # 事件话题需要可靠传输，并保留最后若干条，方便评估器晚启动时仍能读到状态。
        event_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        for topic in PX4_LOCAL_POSITION_TOPICS:
            def _callback(msg: VehicleLocalPosition, t: str = topic) -> None:
                self._on_local_position(msg, t)

            self.create_subscription(
                VehicleLocalPosition,
                topic,
                _callback,
                px4_qos,
            )
        self.create_subscription(Odometry, TARGET_STATE_TOPIC, self._on_target_state, 10)
        self.create_subscription(String, EVENT_TOPIC, self._on_event, event_qos)
        # 超时结束由评估器判定，因此评估器也会补发 case_finished 事件写入 rosbag。
        self.event_pub = self.create_publisher(String, EVENT_TOPIC, event_qos)

    def clear_px4_sample(self):
        """清掉上一轮样本，确保 wait_for_px4_ready 等到的是当前外部仿真的新消息。"""

        self.local_position = None
        self.local_velocity = None
        self.active_local_position_topic = None
        self.last_px4_message_time = None

    def start_case(self, case_id: str):
        """进入新 case 前重置所有累计状态。"""

        self.current_case_id = case_id
        self.case_started_at = time.monotonic()
        self.guidance_started_at = None
        self.finished_at = None
        self.last_event_by_name = {}
        self.last_event_payload = {}
        self.target_position = None
        self.target_velocity = None
        self.metrics = CaseMetrics()
        self._last_velocity_for_accel = None
        self._last_velocity_time = None

    def finish_case(self):
        """固化 case 结束时刻，并补齐最终距离和总耗时。"""

        now = time.monotonic()
        self.finished_at = now
        if self.case_started_at is not None:
            self.metrics.total_time = now - self.case_started_at
        if self.guidance_started_at is not None:
            self.metrics.guidance_time = now - self.guidance_started_at
        if self.local_position is not None and self.target_position is not None:
            self.metrics.final_distance = float(
                np.linalg.norm(self.local_position - self.target_position)
            )

    def publish_case_finished(self, success: bool, reason: str):
        """由评估器发布 case_finished，覆盖导引节点不会主动发布的超时路径。"""

        payload = {
            "event": "case_finished",
            "case_id": self.current_case_id,
            "success": success,
            "reason": reason,
            "distance": self.metrics.final_distance,
            "stamp": self.get_clock().now().nanoseconds / 1e9,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, sort_keys=True)
        self.event_pub.publish(msg)

    def has_px4_sample(self) -> bool:
        return self.last_px4_message_time is not None

    def guidance_started(self) -> bool:
        return self.guidance_started_at is not None

    def _on_local_position(self, msg: VehicleLocalPosition, topic: str):
        """处理真实追踪机状态。

        指标从 guidance_started 之后开始累计，起飞阶段不计入拦截性能。
        """

        now = time.monotonic()
        self.last_px4_message_time = now
        self.active_local_position_topic = topic
        self.local_position = np.array([msg.x, msg.y, msg.z], dtype=float)
        self.local_velocity = np.array([msg.vx, msg.vy, msg.vz], dtype=float)

        if self.guidance_started_at is None:
            return

        msg_time = _px4_message_time(msg, fallback=now)
        if self._last_velocity_for_accel is not None and self._last_velocity_time is not None:
            dt = msg_time - self._last_velocity_time
            if dt > 1e-3:
                # 最大实际加速度来自 PX4 状态速度差分，不使用 PN 指令加速度。
                acceleration = float(
                    np.linalg.norm(self.local_velocity - self._last_velocity_for_accel) / dt
                )
                self.metrics.max_actual_acceleration = max(
                    self.metrics.max_actual_acceleration, acceleration
                )
        self._last_velocity_for_accel = self.local_velocity.copy()
        self._last_velocity_time = msg_time
        self._update_distance_metrics()

    def _on_target_state(self, msg: Odometry):
        """处理导引节点发布的虚拟目标真值。"""

        self.target_position = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ],
            dtype=float,
        )
        self.target_velocity = np.array(
            [
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
            ],
            dtype=float,
        )
        if self.guidance_started_at is not None:
            self._update_distance_metrics()

    def _on_event(self, msg: String):
        """处理导引节点/评估器发布的 JSON 事件。

        事件中的 case_id 用于隔离上一轮残留事件，避免 rosbag 或 transient_local
        事件在下一轮 case 中误触发。
        """

        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"ignored malformed evaluation event: {msg.data}")
            return

        case_id = str(payload.get("case_id", ""))
        if self.current_case_id and case_id and case_id != self.current_case_id:
            return

        event = str(payload.get("event", ""))
        now = time.monotonic()
        self.last_event_by_name[event] = now
        self.last_event_payload[event] = payload

        if event == "guidance_started" and self.guidance_started_at is None:
            self.guidance_started_at = now
            if self.case_started_at is not None:
                self.metrics.takeoff_time = now - self.case_started_at
        elif event == "intercept_success":
            self.metrics.success = True
            if self.guidance_started_at is not None and math.isnan(self.metrics.intercept_time):
                self.metrics.intercept_time = now - self.guidance_started_at

    def _update_distance_metrics(self):
        """只在追踪机和目标真值都可用时更新距离类指标。"""

        if self.local_position is None or self.target_position is None:
            return

        distance = float(np.linalg.norm(self.local_position - self.target_position))
        self.metrics.final_distance = distance
        self.metrics.min_distance = min(self.metrics.min_distance, distance)

        if distance < self.intercept_radius and not self.metrics.success:
            self.metrics.success = True
            if self.guidance_started_at is not None:
                self.metrics.intercept_time = time.monotonic() - self.guidance_started_at


def _px4_message_time(msg: VehicleLocalPosition, fallback: float) -> float:
    """将 PX4 microsecond 时间戳转换为秒；无时间戳时退回本机单调时钟。"""

    timestamp = float(getattr(msg, "timestamp", 0))
    if timestamp > 0:
        return timestamp / 1_000_000.0
    return fallback


def build_cases(
    motions: Iterable[str],
    speed_max_values: Iterable[float],
    headings: Iterable[float],
    target_speed: float,
) -> list[CaseConfig]:
    """根据计划中的运动类型、速度档位和航向角生成评估矩阵。"""

    cases: list[CaseConfig] = []
    for motion in motions:
        # 静止目标没有航向意义，只保留一个 heading=0 的 case，避免重复运行。
        motion_headings = [0.0] if motion == "static" else [float(h) for h in headings]
        for speed_max in speed_max_values:
            for heading in motion_headings:
                heading_rad = math.radians(heading)
                line_vx = 0.0 if motion == "static" else target_speed * math.cos(heading_rad)
                line_vy = 0.0 if motion == "static" else target_speed * math.sin(heading_rad)
                # 圆周目标的速度方向比半径相位超前 90 度，因此 heading-90 得到初始相位。
                initial_phase = math.radians(heading - 90.0)
                case_id = f"{motion}_h{int(heading):03d}_v{int(speed_max):02d}"
                cases.append(
                    CaseConfig(
                        case_id=case_id,
                        target_motion=motion,
                        target_heading_deg=heading,
                        speed_max=float(speed_max),
                        line_vx=line_vx,
                        line_vy=line_vy,
                        initial_phase=initial_phase,
                    )
                )
    return cases


def write_params_file(case: CaseConfig, args: argparse.Namespace) -> str:
    """为导引节点生成临时 ROS2 参数文件。

    参数文件把当前 case 的目标轨迹、速度档位和 evaluation 开关注入节点。
    case 结束后调用方会删除该临时文件。
    """

    fd, path = tempfile.mkstemp(prefix=f"{case.case_id}_", suffix=".yaml")
    os.close(fd)
    content = f"""pn_guidance_node:
  ros__parameters:
    guidance:
      N: {args.guidance_n}
      speed_min: {args.speed_min}
      speed_max: {case.speed_max}
      strategy: "{args.strategy}"
    flight:
      takeoff_height: {args.takeoff_height}
      intercept_radius: {args.intercept_radius}
      control_freq: {args.control_freq}
    target:
      source: "simulated"
      motion_type: "{case.target_motion}"
      center_x: {args.target_center_x}
      center_y: {args.target_center_y}
      radius: {args.target_radius}
      omega: {args.target_omega}
      initial_phase: {case.initial_phase}
      altitude: {args.target_altitude}
      altitude_amplitude: {args.target_altitude_amplitude}
      altitude_omega: {args.target_altitude_omega}
      line_vx: {case.line_vx}
      line_vy: {case.line_vy}
      line_vz: 0.0
      start_x: {args.target_start_x}
      start_y: {args.target_start_y}
      timeout: 2.0
    evaluation:
      case_id: "{case.case_id}"
      enable_topics: true
    debug:
      log_interval: {args.log_interval}
      verbose: {str(args.guidance_verbose).lower()}
"""
    Path(path).write_text(content, encoding="utf-8")
    return path


def wait_for_px4_ready(node: EvaluationMonitor, timeout: float) -> bool:
    """等待 PX4 本地位置话题恢复，确认外部 Gazebo/uXRCE-DDS 已经可用。"""

    node.clear_px4_sample()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.has_px4_sample():
            print(f"PX4 local position ready: {node.active_local_position_topic}")
            return True
    return False


def px4_topic_hint(node: EvaluationMonitor) -> str:
    """生成等待 PX4 失败时的排查提示。"""

    topic_names = sorted(name for name, _types in node.get_topic_names_and_types())
    fmu_topics = [name for name in topic_names if name.startswith("/fmu/")]
    if fmu_topics:
        preview = ", ".join(fmu_topics[:20])
        if len(fmu_topics) > 20:
            preview += ", ..."
        return f"当前 ROS graph 中可见的 /fmu 话题: {preview}"
    return "当前 ROS graph 中没有 /fmu 话题，请确认 MicroXRCEAgent 和 PX4 SITL 已启动。"


def wait_for_guidance_start(node: EvaluationMonitor, timeout: float) -> bool:
    """等待导引节点完成起飞并发布 guidance_started 事件。"""

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.guidance_started():
            return True
    return False


def run_until_case_done(node: EvaluationMonitor, max_case_time: float) -> None:
    """从 GUIDANCE 阶段开始计时，直到拦截成功或 case 超时。"""

    assert node.guidance_started_at is not None
    deadline = node.guidance_started_at + max_case_time
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.metrics.success:
            return


def start_process(cmd: list[str], show_output: bool) -> subprocess.Popen:
    """启动子进程，并用独立进程组便于后续整体发送 SIGINT。"""

    output = None if show_output else subprocess.DEVNULL
    return subprocess.Popen(
        cmd,
        stdout=output,
        stderr=subprocess.STDOUT if not show_output else None,
        preexec_fn=os.setsid,
    )


def ensure_process_running(process: subprocess.Popen, name: str):
    """启动后短暂检查子进程是否已经异常退出。"""

    return_code = process.poll()
    if return_code is not None:
        raise RuntimeError(f"{name} exited during startup with code {return_code}")


def stop_process(process: subprocess.Popen | None, timeout: float = 8.0):
    """优先用 SIGINT 让 rosbag/ROS 节点正常收尾，超时后再强制终止。"""

    if process is None or process.poll() is not None:
        return

    try:
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
        process.wait(timeout=timeout)
        return
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass

    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3.0)


def unique_bag_path(bag_dir: str, case_id: str) -> str:
    """避免 ros2 bag record 写入已存在目录导致启动失败。"""

    base = Path(bag_dir) / case_id
    if not base.exists():
        return str(base.resolve())

    suffix = time.strftime("%Y%m%d_%H%M%S")
    candidate = Path(bag_dir) / f"{case_id}_{suffix}"
    counter = 1
    while candidate.exists():
        candidate = Path(bag_dir) / f"{case_id}_{suffix}_{counter}"
        counter += 1
    return str(candidate.resolve())


def run_case(
    node: EvaluationMonitor,
    case: CaseConfig,
    args: argparse.Namespace,
    writer: csv.DictWriter,
    output_file,
) -> dict[str, object]:
    """执行单个 case 的完整生命周期。

    流程顺序刻意固定为：等待 PX4 -> 启动 rosbag -> 启动导引节点 -> 等待
    GUIDANCE -> 统计到成功/超时 -> 停止子进程 -> 写 CSV。
    """

    print(f"\n=== case {case.case_id} ===")
    if not wait_for_px4_ready(node, args.wait_px4_timeout):
        raise RuntimeError(
            f"PX4 local position topic not ready within {args.wait_px4_timeout}s: "
            f"{', '.join(PX4_LOCAL_POSITION_TOPICS)}. {px4_topic_hint(node)}"
        )

    node.start_case(case.case_id)
    param_file = write_params_file(case, args)
    bag_path = unique_bag_path(args.bag_dir, case.case_id)
    bag_process = None
    guidance_process = None

    try:
        # rosbag 先启动，确保能记录导引节点发布的 case_started/guidance_started 事件。
        bag_process = start_process(
            ["ros2", "bag", "record", "-o", bag_path, *args.bag_topics],
            show_output=args.show_child_output,
        )
        time.sleep(args.bag_start_delay)
        ensure_process_running(bag_process, "ros2 bag record")
        # 导引节点由评估器生成的参数文件驱动，每个 case 单独启动一次。
        guidance_process = start_process(
            [
                "ros2",
                "run",
                "anti_drone_guidance",
                "anti_drone_guidance_node",
                "--ros-args",
                "--params-file",
                param_file,
            ],
            show_output=args.show_child_output,
        )
        time.sleep(0.5)
        ensure_process_running(guidance_process, "anti_drone_guidance_node")

        if not wait_for_guidance_start(node, args.guidance_start_timeout):
            raise RuntimeError(
                f"case {case.case_id} did not publish guidance_started within "
                f"{args.guidance_start_timeout}s"
            )

        run_until_case_done(node, args.max_case_time)
        node.finish_case()
        if not node.metrics.success:
            # 导引节点只知道拦截成功；超时是评估器判定，需要在停止 rosbag 前补事件。
            node.publish_case_finished(success=False, reason="timeout")
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        stop_process(guidance_process)
        stop_process(bag_process)
        try:
            os.unlink(param_file)
        except OSError:
            pass

    metrics = node.metrics
    if math.isinf(metrics.min_distance):
        metrics.min_distance = math.nan
    # CSV 字段顺序与计划文件保持一致，方便后续脚本稳定解析。
    row = {
        "case_id": case.case_id,
        "target_motion": case.target_motion,
        "target_heading_deg": case.target_heading_deg,
        "speed_max": case.speed_max,
        "success": metrics.success,
        "min_distance": metrics.min_distance,
        "intercept_time": metrics.intercept_time,
        "max_actual_acceleration": metrics.max_actual_acceleration,
        "final_distance": metrics.final_distance,
        "takeoff_time": metrics.takeoff_time,
        "guidance_time": metrics.guidance_time,
        "total_time": metrics.total_time,
        "bag_path": bag_path,
    }
    writer.writerow(row)
    output_file.flush()

    status = "SUCCESS" if metrics.success else "TIMEOUT"
    print(
        f"{status}: min_distance={metrics.min_distance:.3f}, "
        f"final_distance={metrics.final_distance:.3f}, "
        f"max_accel={metrics.max_actual_acceleration:.3f}"
    )
    return row


def print_summary(rows: list[dict[str, object]]):
    """按计划输出总成功率，以及按轨迹类型和速度档位分组的统计。"""

    if not rows:
        print("No cases completed.")
        return

    print("\n=== summary ===")
    print(f"total success rate: {_success_rate(rows):.1f}% ({_success_count(rows)}/{len(rows)})")

    print("\nby target_motion:")
    for motion in sorted({str(row["target_motion"]) for row in rows}):
        group = [row for row in rows if row["target_motion"] == motion]
        print(
            f"  {motion}: success={_success_rate(group):.1f}%, "
            f"avg_min_distance={_avg(group, 'min_distance'):.3f}, "
            f"avg_intercept_time={_avg_success(group, 'intercept_time'):.3f}"
        )

    print("\nby speed_max:")
    for speed in sorted({_as_float(row["speed_max"]) for row in rows}):
        group = [row for row in rows if _as_float(row["speed_max"]) == speed]
        max_accel = max(_as_float(row["max_actual_acceleration"]) for row in group)
        print(
            f"  {speed:g} m/s: success={_success_rate(group):.1f}%, "
            f"max_actual_acceleration_peak={max_accel:.3f}"
        )


def _success_count(rows: list[dict[str, object]]) -> int:
    """统计成功 case 数。"""

    return sum(1 for row in rows if bool(row["success"]))


def _success_rate(rows: list[dict[str, object]]) -> float:
    """计算百分制成功率。"""

    if not rows:
        return 0.0
    return 100.0 * _success_count(rows) / len(rows)


def _as_float(value: object) -> float:
    """CSV 行里可能有 nan/空值，统一做浮点转换。"""

    try:
        if not isinstance(value, (int, float, str, bytes, bytearray)):
            return math.nan
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def _finite(values: Iterable[float]) -> list[float]:
    """过滤掉 nan/inf，避免无效数据污染平均值。"""

    return [value for value in values if math.isfinite(value)]


def _avg(rows: list[dict[str, object]], field: str) -> float:
    """计算所有 case 的字段平均值。"""

    values = _finite(_as_float(row[field]) for row in rows)
    return sum(values) / len(values) if values else math.nan


def _avg_success(rows: list[dict[str, object]], field: str) -> float:
    """只对成功 case 计算字段平均值，例如平均拦截时间。"""

    values = _finite(_as_float(row[field]) for row in rows if bool(row["success"]))
    return sum(values) / len(values) if values else math.nan


def build_arg_parser() -> argparse.ArgumentParser:
    """定义命令行参数，默认值对应计划中的小型评估矩阵。"""

    parser = argparse.ArgumentParser(
        description="Run PX4 Gazebo closed-loop evaluation cases for anti_drone_guidance."
    )
    parser.add_argument("--output", default="gazebo_evaluation_results.csv")
    parser.add_argument("--bag-dir", default="gazebo_evaluation_bags")
    parser.add_argument("--max-case-time", type=float, default=60.0)
    parser.add_argument("--wait-px4-timeout", type=float, default=60.0)
    parser.add_argument("--guidance-start-timeout", type=float, default=45.0)
    parser.add_argument("--bag-start-delay", type=float, default=1.0)
    parser.add_argument("--intercept-radius", type=float, default=2.0)
    parser.add_argument("--motions", nargs="+", default=["static", "line", "circle", "circle_altitude"])
    parser.add_argument("--speed-max", nargs="+", type=float, default=[10.0, 20.0, 30.0])
    parser.add_argument("--headings", nargs="+", type=float, default=[0.0, 90.0, 180.0, 270.0])
    parser.add_argument("--target-speed", type=float, default=2.0)
    parser.add_argument("--target-center-x", type=float, default=50.0)
    parser.add_argument("--target-center-y", type=float, default=50.0)
    parser.add_argument("--target-start-x", type=float, default=50.0)
    parser.add_argument("--target-start-y", type=float, default=50.0)
    parser.add_argument("--target-radius", type=float, default=30.0)
    parser.add_argument("--target-omega", type=float, default=0.1)
    parser.add_argument("--target-altitude", type=float, default=20.0)
    parser.add_argument("--target-altitude-amplitude", type=float, default=5.0)
    parser.add_argument("--target-altitude-omega", type=float, default=0.2)
    parser.add_argument("--guidance-n", type=float, default=4.0)
    parser.add_argument("--speed-min", type=float, default=2.0)
    parser.add_argument("--strategy", default="adaptive")
    parser.add_argument("--takeoff-height", type=float, default=10.0)
    parser.add_argument("--control-freq", type=float, default=20.0)
    parser.add_argument("--log-interval", type=int, default=30)
    parser.add_argument("--guidance-verbose", action="store_true")
    parser.add_argument("--show-child-output", action="store_true")
    parser.add_argument("--no-restart-prompt", action="store_true")
    parser.add_argument("--bag-topics", nargs="+", default=DEFAULT_BAG_TOPICS)
    return parser


def main(argv: list[str] | None = None) -> int:
    """命令行入口。"""

    args = build_arg_parser().parse_args(argv)
    Path(args.bag_dir).mkdir(parents=True, exist_ok=True)
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    cases = build_cases(args.motions, args.speed_max, args.headings, args.target_speed)
    rows: list[dict[str, object]] = []

    rclpy.init(args=None)
    node = EvaluationMonitor(intercept_radius=args.intercept_radius)
    try:
        with output_path.open("w", newline="", encoding="utf-8") as output_file:
            writer = csv.DictWriter(output_file, fieldnames=CSV_FIELDS)
            writer.writeheader()
            for index, case in enumerate(cases):
                rows.append(run_case(node, case, args, writer, output_file))
                if index < len(cases) - 1 and not args.no_restart_prompt:
                    # 计划要求 PX4/Gazebo 重启由用户或外部脚本完成，评估器只在此等待。
                    if sys.stdin.isatty():
                        input("Restart PX4 Gazebo/uXRCE-DDS for the next case, then press Enter...")
                    else:
                        print("Restart prompt skipped because stdin is not interactive.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print_summary(rows)
    print(f"\nCSV written to: {output_path.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
