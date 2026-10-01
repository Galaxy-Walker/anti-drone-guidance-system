"""实时查看下视相机画面与检测框的监控旁路（调试用，不进入导引链路）。

`vision_detector` 只发布检测框数据、不发布标注图，rqt_image_view 只能看到原始画面。
本工具订阅 `/camera/image_raw` + `/camera/detections`，把框、中心点和分数用 OpenCV
画到画面上，再发布 `/camera/image_annotated` 供 rqt_image_view 查看：

    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    python3 tools/vision_live_view.py

    # 另开终端
    ros2 run rqt_image_view rqt_image_view /camera/image_annotated

设计取舍：
- 检测到达时按 `header.stamp` 回查图像缓存，而不是“最新框画到最新帧”：图像约
  13~15 Hz、检测约 10 Hz，用最新框会让快速运动时的框明显滞后；按 stamp 配对
  保证框与画面严格同帧（yolo 检测的 stamp 就是 `vision_detector` 处理的那帧图像 stamp）。
- 检测节点每个处理帧都会发消息（未检出是空数组），所以输出频率约等于
  `yolo_process_hz`，漏检时画面原样透传，不会挡住相机画面。
- `truth` 模式的 `/camera/detections_truth` 按生成时刻打 stamp，与图像 stamp 不严格
  一致，用 `--match-tolerance-s` 退化为最近帧配对即可查看。
- 只发布 `/camera/image_annotated`，不发布导引相关话题，也不要求 `use_sim_time`。
"""

from __future__ import annotations

import argparse
import sys
import time
from collections import OrderedDict
from pathlib import Path

import cv2
import numpy as np


def _ensure_gazebosimulation2d_on_path() -> None:
    """开发阶段未 source 工作空间时，也能复用包内共享的图像编码转换。"""
    for parent in Path(__file__).resolve().parents:
        candidate = parent / "src" / "gazebosimulation2d"
        if (candidate / "gazebosimulation2d" / "image_utils.py").is_file():
            sys.path.insert(0, str(candidate))
            return


_ensure_gazebosimulation2d_on_path()

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

from gazebosimulation2d.image_utils import image_message_to_bgr

NANOSECONDS_PER_SECOND = 1_000_000_000
# 图像缓存不足或 stamp 对不上时，告警最多 5 s 打一次，避免刷屏。
WARN_PERIOD_S = 5.0
WINDOW_NAME = "vision_live_view"

BOX_COLOR = (0, 255, 0)
CENTER_COLOR = (0, 0, 255)


def _header_stamp_ns(header) -> int:
    return int(header.stamp.sec) * NANOSECONDS_PER_SECOND + int(header.stamp.nanosec)


def _image_qos() -> QoSProfile:
    """与相机桥接一致：BEST_EFFORT + KEEP_LAST(1)。"""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def _detection_qos() -> QoSProfile:
    """与 `vision_detector` 的发布端一致：BEST_EFFORT + KEEP_LAST(10)。"""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


def _annotated_qos() -> QoSProfile:
    """输出用 RELIABLE：兼容 rqt_image_view 的默认订阅，也不排斥 best_effort 订阅。"""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def draw_detections(frame: np.ndarray, message: Detection2DArray) -> np.ndarray:
    """把检测框、中心点与分数画到 BGR 帧上（原地修改并返回）。"""
    for detection in message.detections:
        cx = float(detection.bbox.center.position.x)
        cy = float(detection.bbox.center.position.y)
        x1 = int(round(cx - 0.5 * float(detection.bbox.size_x)))
        y1 = int(round(cy - 0.5 * float(detection.bbox.size_y)))
        x2 = int(round(cx + 0.5 * float(detection.bbox.size_x)))
        y2 = int(round(cy + 0.5 * float(detection.bbox.size_y)))
        cv2.rectangle(frame, (x1, y1), (x2, y2), BOX_COLOR, 2)
        cv2.circle(frame, (int(round(cx)), int(round(cy))), 3, CENTER_COLOR, -1)

        if not detection.results:
            continue
        hypothesis = detection.results[0].hypothesis
        label = f"{hypothesis.class_id} {hypothesis.score:.2f}".strip()
        # 框顶贴近画面上沿时把文字放到框内，避免被裁掉。
        text_y = y1 - 6 if y1 >= 20 else min(y2 + 18, frame.shape[0] - 4)
        cv2.putText(
            frame,
            label,
            (max(x1, 0), text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            BOX_COLOR,
            2,
            cv2.LINE_AA,
        )
    return frame


class VisionLiveView(Node):
    """订阅图像与检测、按 stamp 配对后发布标注图。"""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("vision_live_view")
        self._args = args
        self._images: OrderedDict[int, Image] = OrderedDict()
        self._show = bool(args.show)
        self._last_warn_ns: int | None = None

        self._publisher = self.create_publisher(Image, args.output_topic, _annotated_qos())
        self.create_subscription(Image, args.image_topic, self._on_image, _image_qos())
        self.create_subscription(
            Detection2DArray, args.detections_topic, self._on_detections, _detection_qos()
        )
        self.get_logger().info(
            f"vision_live_view: {args.image_topic} + {args.detections_topic} -> {args.output_topic} "
            f"(cache={args.cache_size} 帧, 配对容差={args.match_tolerance_s:.3f}s)"
        )

    # ---------------------------------------------------------------- 回调

    def _on_image(self, message: Image) -> None:
        self._images[_header_stamp_ns(message.header)] = message
        while len(self._images) > self._args.cache_size:
            self._images.popitem(last=False)

    def _on_detections(self, message: Detection2DArray) -> None:
        stamp_ns = _header_stamp_ns(message.header)
        image = self._lookup_image(stamp_ns)
        if image is None:
            self._warn(f"stamp={stamp_ns * 1e-9:.3f}s 的检测在图像缓存中没有可配对帧，已跳过")
            return
        try:
            frame = image_message_to_bgr(image)
        except ValueError as exc:
            self._warn(f"图像转换失败：{exc}")
            return
        draw_detections(frame, message)
        self._publish(frame, image)
        if self._show:
            self._show_frame(frame)

    def _lookup_image(self, stamp_ns: int) -> Image | None:
        image = self._images.get(stamp_ns)
        if image is not None:
            return image
        # truth 伪检测不来自图像 stamp；只在这种情况下按最近帧配对。
        if self._args.match_tolerance_s <= 0.0 or not self._images:
            return None
        nearest_ns = min(self._images, key=lambda candidate: abs(candidate - stamp_ns))
        if abs(nearest_ns - stamp_ns) <= self._args.match_tolerance_s * NANOSECONDS_PER_SECOND:
            return self._images[nearest_ns]
        return None

    # ---------------------------------------------------------------- 输出

    def _publish(self, frame: np.ndarray, source: Image) -> None:
        message = Image()
        # 标注图沿用源图 stamp 与 frame_id，便于和真值/记录对齐。
        message.header = source.header
        message.height = int(frame.shape[0])
        message.width = int(frame.shape[1])
        message.encoding = "bgr8"
        message.is_bigendian = False
        message.step = int(frame.shape[1] * 3)
        message.data = frame.tobytes()
        self._publisher.publish(message)

    def _show_frame(self, frame: np.ndarray) -> None:
        try:
            cv2.imshow(WINDOW_NAME, frame)
            cv2.waitKey(1)
        except cv2.error as exc:  # 无 WSLg/显示服务时降级为只发布
            self._show = False
            self.get_logger().warning(f"OpenCV 窗口不可用，后续只发布标注图：{exc}")

    def _warn(self, text: str) -> None:
        now_ns = time.monotonic_ns()
        if self._last_warn_ns is not None and (now_ns - self._last_warn_ns) < WARN_PERIOD_S * 1e9:
            return
        self._last_warn_ns = now_ns
        self.get_logger().warning(text)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="实时查看相机画面与检测框：发布 /camera/image_annotated 供 rqt_image_view 查看"
    )
    parser.add_argument("--image-topic", default="/camera/image_raw", help="相机图像话题")
    parser.add_argument(
        "--detections-topic",
        default="/camera/detections",
        help="检测话题；truth 模式可传 /camera/detections_truth",
    )
    parser.add_argument("--output-topic", default="/camera/image_annotated", help="标注图输出话题")
    parser.add_argument("--cache-size", type=int, default=60, help="图像缓存帧数（按 stamp 回查）")
    parser.add_argument(
        "--match-tolerance-s",
        type=float,
        default=0.05,
        help="没有同 stamp 帧时允许配对的最近帧时间差；0 表示只接受严格同 stamp",
    )
    parser.add_argument("--show", action="store_true", help="额外打开 OpenCV 窗口（WSL2 需 WSLg）")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    if args.cache_size <= 0:
        raise SystemExit("--cache-size 必须为正整数")

    rclpy.init()
    node = VisionLiveView(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
