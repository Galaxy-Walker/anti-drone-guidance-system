"""相机画面周期截图节点：按图像 stamp 每隔固定时间落盘一张 JPEG。

用途：在真值/odometry 制导下排查 YOLO 检测失败，不依赖 `vision_detector` 与 YOLO worker，
只订阅相机图像（默认 `/camera/image_raw`）。与数据集帧录制保持同一约定：

- 以图像 `header.stamp`（Gazebo 仿真时间）节流和命名，文件名 `<stamp_ns>.jpg`；
- 第一帧立即保存，之后间隔不足 `save_hz` 的帧丢弃；重启后已存在的文件跳过；
- 编码转换复用 `image_utils.image_message_to_bgr`，不另写一套 rgb/rgba 分支。

节点只记录画面，不参与导引，也不发布任何话题。
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from gazebosimulation2d.image_utils import image_message_to_bgr


class CameraRecorder(Node):
    """按固定周期把相机图像存成 JPEG 的调试节点。"""

    def __init__(self, parameter_overrides: list[Parameter] | None = None) -> None:
        super().__init__("camera_recorder", parameter_overrides=parameter_overrides)
        self._declare_parameters()
        self._load_parameters()

        # 截图是本节点的唯一功能：缺少 cv2 时直接报错退出，而不是静默收图不落盘。
        try:
            import cv2
        except ImportError as exc:
            raise ValueError(
                "camera_recorder 需要系统 Python 安装 cv2（例如 python3-opencv / ros-jazzy-cv-bridge）"
            ) from exc
        self._cv2 = cv2

        self._last_saved_stamp_ns: int | None = None
        self._saved = 0
        self._dropped_throttle = 0
        self._dropped_invalid = 0
        self._limit_logged = False

        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image, self._image_topic, self._image_callback, image_qos)

        self.get_logger().info(
            f"camera_recorder ready: topic={self._image_topic}, output_dir={self._output_dir}, "
            f"save_hz={self._save_hz:.2f}, jpeg_quality={self._jpeg_quality}, max_frames={self._max_frames}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("output_dir", "outputs/gazebo2d_vision/camera_frames")
        self.declare_parameter("save_hz", 1.0)
        self.declare_parameter("jpeg_quality", 90)
        self.declare_parameter("max_frames", 0)

    def _load_parameters(self) -> None:
        self._image_topic = str(self.get_parameter("image_topic").value)
        if not self._image_topic.startswith("/"):
            raise ValueError("image_topic 必须是绝对话题名")
        self._output_dir = Path(str(self.get_parameter("output_dir").value)).expanduser()

        self._save_hz = self._positive_float("save_hz")
        self._min_period_ns = int(1e9 / self._save_hz)

        quality = int(self.get_parameter("jpeg_quality").value)
        if not 1 <= quality <= 100:
            raise ValueError("jpeg_quality 必须落在 [1, 100]")
        self._jpeg_quality = quality

        max_frames = int(self.get_parameter("max_frames").value)
        if max_frames < 0:
            raise ValueError("max_frames 必须非负（0 表示不限制）")
        self._max_frames = max_frames

    def _positive_float(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} 必须是正的有限数")
        return value

    # ---------------------------------------------------------------- 图像入口

    def _image_callback(self, message: Image) -> None:
        if 0 < self._max_frames <= self._saved:
            if not self._limit_logged:
                self._limit_logged = True
                self.get_logger().info(f"已达到 max_frames={self._max_frames}，停止保存")
            return

        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
        if stamp_ns <= 0:
            # 桥接缺 stamp 时用节点时钟兜底，保证文件名不重复。
            stamp_ns = int(self.get_clock().now().nanoseconds)

        if self._last_saved_stamp_ns is not None:
            delta_ns = stamp_ns - self._last_saved_stamp_ns
            # 时钟回退（仿真重启）时 delta<0，重新开始节流而不是一直跳过。
            if 0 <= delta_ns < self._min_period_ns:
                self._dropped_throttle += 1
                return

        path = self._output_dir / f"{stamp_ns}.jpg"
        if path.exists():
            self._last_saved_stamp_ns = stamp_ns
            return

        try:
            bgr = image_message_to_bgr(message)
        except ValueError as exc:
            self._dropped_invalid += 1
            self.get_logger().warning(f"丢弃非法图像帧：{exc}", throttle_duration_sec=5.0)
            return

        try:
            self._output_dir.mkdir(parents=True, exist_ok=True)
            self._cv2.imwrite(
                str(path),
                bgr,
                [int(self._cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality],
            )
        except Exception as exc:  # noqa: BLE001 - 单帧落盘失败不能中断记录
            self.get_logger().warning(f"图像保存失败：{exc}", throttle_duration_sec=5.0)
            return

        self._last_saved_stamp_ns = stamp_ns
        self._saved += 1
        self.get_logger().info(f"saved camera frame {path} (total={self._saved})")

    def log_summary(self) -> None:
        self._log_shutdown_safe(
            "info",
            f"camera_recorder saved {self._saved} frames to {self._output_dir} "
            f"(dropped throttle={self._dropped_throttle}, invalid={self._dropped_invalid})",
        )

    def _log_shutdown_safe(self, level: str, message: str) -> None:
        if rclpy.ok():
            getattr(self.get_logger(), level)(message)
            return
        stream = sys.stderr if level == "error" else sys.stdout
        print(f"[{level.upper()}] [camera_recorder]: {message}", file=stream)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    try:
        node = CameraRecorder()
    except ValueError as exc:
        print(f"[ERROR] [camera_recorder]: {exc}", file=sys.stderr)
        if rclpy.ok():
            rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.log_summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
