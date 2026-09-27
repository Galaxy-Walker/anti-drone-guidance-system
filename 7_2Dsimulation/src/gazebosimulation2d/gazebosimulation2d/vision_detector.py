"""把下视相机图像送到常驻 YOLO worker，并发布 `vision_msgs` 检测的 ROS 节点。

本节点是 YOLO 闭环里唯一新增的 ROS 节点，使用系统 Python（不 import torch）：

- 订阅 `/camera/image_raw`（BEST_EFFORT/SENSOR_DATA），做节流与过期丢帧；
- 通过 stdin/stdout 二进制协议把帧送给 conda 环境里的 `yolo_worker.py`（见 scripts/）；
- 把最高分检测发布到 `/camera/detections`（`vision_msgs/Detection2DArray`，BEST_EFFORT/VOLATILE）；
- 未检出时发布空数组，区分“没有目标”和“检测节点挂了”；
- 每处理帧记录一行 `yolo_detections.csv`（含推理耗时与端到端耗时）。

设计决策与协议细节见 `docs/vision_design.md` 第 2、4 节。
"""

from __future__ import annotations

import csv
import json
import math
import os
import select
import struct
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np


def _ensure_pythonsimulation2d_on_path() -> None:
    """开发阶段未安装包时，让同级 `pythonsimulation2d` 可导入。"""
    for parent in Path(__file__).resolve().parents:
        src_candidate = parent / "src"
        if (src_candidate / "pythonsimulation2d").is_dir():
            sys.path.insert(0, str(src_candidate))
            return
        if (parent / "pythonsimulation2d").is_dir():
            sys.path.insert(0, str(parent))
            return


_ensure_pythonsimulation2d_on_path()


import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from gazebosimulation2d.sim_clock import SimClockGuard, create_sim_clock_guard_timer

HEADER_LENGTH = struct.Struct(">I")

# 与 vision_adapter 一致：相机桥接输出这些编码；bigendian 图像直接拒绝。
SUPPORTED_ENCODINGS = frozenset({"rgb8", "bgr8", "rgba8", "bgra8", "mono8"})
ENCODING_CHANNELS = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}

CSV_FIELDS = (
    "stamp_s",
    "u",
    "v",
    "w",
    "h",
    "score",
    "inference_ms",
    "e2e_ms",
    "detections",
)


@dataclass(slots=True)
class _DetectionRow:
    stamp_s: float
    u: float = math.nan
    v: float = math.nan
    w: float = math.nan
    h: float = math.nan
    score: float = math.nan
    inference_ms: float = math.nan
    e2e_ms: float = math.nan
    detections: int = 0


def _default_worker_script() -> str:
    """优先使用安装到 share 的 worker 脚本；开发期回退到源码树。"""
    try:
        from ament_index_python.packages import get_package_share_directory

        installed = Path(get_package_share_directory("gazebosimulation2d")) / "scripts" / "yolo_worker.py"
        if installed.is_file():
            return str(installed)
    except Exception:  # noqa: BLE001 - 开发期可能没有 ament index
        pass
    return str(Path(__file__).resolve().parents[1] / "scripts" / "yolo_worker.py")


class VisionDetector(Node):
    """同步请求-响应式检测节点：图像回调里完成一次送帧和收框。"""

    def __init__(self, parameter_overrides: list[Parameter] | None = None) -> None:
        super().__init__("vision_detector", parameter_overrides=parameter_overrides)
        self._declare_parameters()
        self._load_parameters()

        # use_sim_time=true 但 /clock 缺失时，帧龄检查会静默丢弃所有帧；这里明确报错退出。
        self._sim_clock_guard = (
            SimClockGuard() if self._as_bool("use_sim_time") else None
        )
        if self._sim_clock_guard is not None:
            create_sim_clock_guard_timer(self, self._sim_clock_guard)
        self._worker: subprocess.Popen | None = None
        self._sequence = 0
        self._consecutive_failures = 0
        self._worker_gave_up = False
        self._last_process_mono_ns: int | None = None
        self._last_saved_stamp_ns: int | None = None
        self._rows: list[_DetectionRow] = []
        self._dropped_throttle = 0
        self._dropped_stale = 0
        self._dropped_invalid = 0
        self._last_debug_log_ns: int | None = None
        self._jpeg_warned = False

        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        detection_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Image, "/camera/image_raw", self._image_callback, image_qos)
        self._detections_pub = self.create_publisher(Detection2DArray, "/camera/detections", detection_qos)

        if not self._yolo_python:
            raise ValueError("yolo_python 不能为空：必须显式提供 conda ultralytics 环境的 python 路径")
        if not Path(self._yolo_python).is_file():
            raise ValueError(f"yolo_python 不存在：{self._yolo_python}")
        if not Path(self._worker_script).is_file():
            raise ValueError(f"yolo_worker_script 不存在：{self._worker_script}")
        if not self._model_path:
            raise ValueError("model_path 不能为空：必须显式提供 .engine 或 .pt 权重路径")

        if not self._as_bool("use_sim_time"):
            self.get_logger().warning(
                "use_sim_time=false：图像 stamp 是仿真时间，帧龄检查会拒绝所有帧；"
                "YOLO 闭环请设置 use_sim_time:=true"
            )

        self.get_logger().info(
            f"vision_detector ready: model={self._model_path}, device={self._device}, "
            f"imgsz={self._imgsz}, conf={self._conf}, process_hz={self._process_hz:.1f}, "
            f"frame_format={self._frame_format}, save_frame_hz={self._save_frame_hz:.1f}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("camera_frame_id", "camera_link_optical")
        self.declare_parameter("yolo_python", "")
        self.declare_parameter("yolo_worker_script", _default_worker_script())
        self.declare_parameter("model_path", "")
        self.declare_parameter("model_fallback_path", "")
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("iou", 0.7)
        self.declare_parameter("device", "0")
        self.declare_parameter("half", True)
        self.declare_parameter("max_det", 5)
        self.declare_parameter("class_name", "uav")
        self.declare_parameter("process_hz", 10.0)
        self.declare_parameter("max_frame_age_s", 0.2)
        self.declare_parameter("frame_format", "raw")
        self.declare_parameter("inference_timeout_s", 1.0)
        self.declare_parameter("worker_startup_timeout_s", 30.0)
        self.declare_parameter("worker_restart_limit", 3)
        self.declare_parameter("save_frame_hz", 0.0)
        self.declare_parameter("dataset_output_dir", "outputs/gazebo2d_vision/dataset")
        self.declare_parameter("stats_csv", "outputs/gazebo2d_vision/yolo_detections.csv")
        self.declare_parameter("debug_log", False)
        self.declare_parameter("debug_log_period_s", 1.0)

    def _load_parameters(self) -> None:
        self._camera_frame_id = str(self.get_parameter("camera_frame_id").value)
        if not self._camera_frame_id:
            raise ValueError("camera_frame_id 不能为空")
        self._yolo_python = str(self.get_parameter("yolo_python").value)
        # 空串表示使用包内 share/scripts/yolo_worker.py，便于 YAML 留空而不覆盖默认值。
        self._worker_script = str(self.get_parameter("yolo_worker_script").value) or _default_worker_script()
        self._model_path = str(self.get_parameter("model_path").value)
        self._model_fallback_path = str(self.get_parameter("model_fallback_path").value)
        self._imgsz = self._positive_int("imgsz")
        self._conf = self._bounded_float("conf", 0.0, 1.0)
        self._iou = self._bounded_float("iou", 0.0, 1.0)
        self._device = str(self.get_parameter("device").value)
        self._half = self._as_bool("half")
        self._max_det = self._positive_int("max_det")
        self._class_name = str(self.get_parameter("class_name").value)
        self._process_hz = self._positive_float("process_hz")
        self._max_frame_age_s = self._non_negative_float("max_frame_age_s")
        self._frame_format = str(self.get_parameter("frame_format").value)
        if self._frame_format not in ("raw", "jpeg"):
            raise ValueError(f"frame_format 必须是 raw 或 jpeg，收到 {self._frame_format!r}")
        self._inference_timeout_s = self._positive_float("inference_timeout_s")
        self._startup_timeout_s = self._positive_float("worker_startup_timeout_s")
        self._worker_restart_limit = self._positive_int("worker_restart_limit")
        self._save_frame_hz = self._non_negative_float("save_frame_hz")
        self._dataset_output_dir = Path(str(self.get_parameter("dataset_output_dir").value)).expanduser()
        self._stats_csv = Path(str(self.get_parameter("stats_csv").value)).expanduser()
        self._debug_log = self._as_bool("debug_log")
        self._debug_log_period_s = self._positive_float("debug_log_period_s")
        self._min_process_period_ns = int(1e9 / self._process_hz)

    def _as_bool(self, name: str) -> bool:
        value = self.get_parameter(name).value
        if isinstance(value, str):
            return value.lower() in {"1", "true", "yes", "on"}
        return bool(value)

    def _positive_float(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} 必须是正的有限数")
        return value

    def _non_negative_float(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} 必须是非负有限数")
        return value

    def _bounded_float(self, name: str, lower: float, upper: float) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or not lower <= value <= upper:
            raise ValueError(f"{name} 必须落在 [{lower}, {upper}]")
        return value

    def _positive_int(self, name: str) -> int:
        value = int(self.get_parameter(name).value)
        if value <= 0:
            raise ValueError(f"{name} 必须是正整数")
        return value

    # ---------------------------------------------------------------- 图像入口

    def _image_callback(self, message: Image) -> None:
        now_mono_ns = time.monotonic_ns()
        if (
            self._last_process_mono_ns is not None
            and now_mono_ns - self._last_process_mono_ns < self._min_process_period_ns
        ):
            self._dropped_throttle += 1
            self._maybe_log_debug()
            return

        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
        if self._max_frame_age_s > 0.0:
            now_sim_ns = int(self.get_clock().now().nanoseconds)
            age_s = (now_sim_ns - stamp_ns) * 1e-9
            if age_s > self._max_frame_age_s:
                self._dropped_stale += 1
                self._maybe_log_debug()
                return

        encoded = self._encode_frame(message)
        if encoded is None:
            self._dropped_invalid += 1
            self._maybe_log_debug()
            return
        frame_bytes, width, height, encoding = encoded

        self._last_process_mono_ns = now_mono_ns
        self._maybe_save_frame(message, stamp_ns)

        if not self._ensure_worker():
            self._maybe_log_debug()
            return

        sequence = self._sequence
        self._sequence += 1
        request = {
            "seq": sequence,
            "stamp_ns": stamp_ns,
            "width": width,
            "height": height,
            "encoding": encoding,
            "format": self._frame_format,
            "bytes": len(frame_bytes),
        }
        if not self._send_request(request, frame_bytes):
            self._maybe_log_debug()
            return

        response = self._read_response(self._inference_timeout_s)
        if response is None:
            self._handle_worker_failure(f"第 {sequence} 帧等待回包超时（{self._inference_timeout_s:.2f}s）")
            self._maybe_log_debug()
            return

        self._consecutive_failures = 0
        e2e_ms = (time.monotonic_ns() - now_mono_ns) * 1e-6
        self._handle_response(response, stamp_ns, e2e_ms)
        self._maybe_log_debug()

    def _encode_frame(self, message: Image) -> tuple[bytes, int, int, str] | None:
        width, height = int(message.width), int(message.height)
        encoding = str(message.encoding)
        if width <= 0 or height <= 0 or encoding not in SUPPORTED_ENCODINGS:
            self._log_invalid_frame(f"尺寸或编码不支持：{width}x{height} {encoding!r}")
            return None
        if message.is_bigendian:
            self._log_invalid_frame("bigendian 图像不支持")
            return None

        raw = bytes(message.data)
        expected = width * height * ENCODING_CHANNELS[encoding]
        if len(raw) < expected:
            self._log_invalid_frame(f"图像数据不足：{len(raw)} < {expected}")
            return None

        if self._frame_format == "raw":
            return raw[:expected], width, height, encoding

        jpeg = self._encode_jpeg(raw[:expected], width, height, encoding)
        if jpeg is None:
            self._log_invalid_frame("JPEG 编码失败（缺少 cv2？）")
            return None
        return jpeg, width, height, encoding

    def _encode_jpeg(self, raw: bytes, width: int, height: int, encoding: str) -> bytes | None:
        try:
            import cv2
        except ImportError:
            if not self._jpeg_warned:
                self._jpeg_warned = True
                self.get_logger().error("frame_format=jpeg 需要系统 Python 安装 cv2；请改用 frame_format:=raw")
            return None
        channels = ENCODING_CHANNELS[encoding]
        array = np.frombuffer(raw, dtype=np.uint8).reshape(height, width, channels)
        if encoding == "bgr8":
            bgr = np.ascontiguousarray(array)
        elif encoding == "rgb8":
            bgr = np.ascontiguousarray(array[:, :, ::-1])
        elif encoding == "rgba8":
            bgr = np.ascontiguousarray(array[:, :, 2::-1])
        elif encoding == "bgra8":
            bgr = np.ascontiguousarray(array[:, :, :3])
        else:
            bgr = np.repeat(array, 3, axis=2)
        ok, buffer = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        if not ok:
            return None
        return buffer.tobytes()

    def _log_invalid_frame(self, reason: str) -> None:
        self.get_logger().warning(f"丢弃非法图像帧：{reason}", throttle_duration_sec=5.0)

    def _maybe_save_frame(self, message: Image, stamp_ns: int) -> None:
        """数据集录制：按 save_frame_hz 落盘帧（jpeg），文件名用图像 stamp。"""
        if self._save_frame_hz <= 0.0:
            return
        if self._last_saved_stamp_ns is not None:
            min_period_ns = int(1e9 / self._save_frame_hz)
            if stamp_ns - self._last_saved_stamp_ns < min_period_ns:
                return
        path = self._dataset_output_dir / "frames" / f"{stamp_ns}.jpg"
        if path.exists():
            self._last_saved_stamp_ns = stamp_ns
            return
        try:
            import cv2

            width, height = int(message.width), int(message.height)
            encoding = str(message.encoding)
            channels = ENCODING_CHANNELS[encoding]
            array = np.frombuffer(bytes(message.data), dtype=np.uint8).reshape(height, width, channels)
            if encoding == "bgr8":
                bgr = array
            elif encoding == "rgb8":
                bgr = array[:, :, ::-1]
            elif encoding == "rgba8":
                bgr = array[:, :, 2::-1]
            elif encoding == "bgra8":
                bgr = array[:, :, :3]
            else:
                bgr = np.repeat(array, 3, axis=2)
            path.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(path), np.ascontiguousarray(bgr))
            self._last_saved_stamp_ns = stamp_ns
        except ImportError:
            self._save_frame_hz = 0.0
            self.get_logger().error("数据集录制需要 cv2，已自动关闭 save_frame_hz")
        except Exception as exc:  # noqa: BLE001 - 录制失败不能影响检测主链路
            self.get_logger().warning(f"数据集帧保存失败：{exc}", throttle_duration_sec=5.0)

    # ------------------------------------------------------------- worker 管理

    def _ensure_worker(self) -> bool:
        if self._worker_gave_up:
            return False
        if self._worker is not None:
            if self._worker.poll() is None:
                return True
            returncode = self._worker.returncode
            self._terminate_worker()
            self._consecutive_failures += 1
            self.get_logger().warning(f"YOLO worker 已退出（returncode={returncode}）")
        if self._consecutive_failures >= self._worker_restart_limit:
            self._worker_gave_up = True
            self.get_logger().error(
                f"YOLO worker 连续失败 {self._consecutive_failures} 次，达到 worker_restart_limit，停止检测"
            )
            return False
        return self._start_worker()

    def _start_worker(self) -> bool:
        command = [
            self._yolo_python,
            self._worker_script,
            "--model",
            self._model_path,
            "--imgsz",
            str(self._imgsz),
            "--conf",
            str(self._conf),
            "--iou",
            str(self._iou),
            "--device",
            self._device,
            "--max-det",
            str(self._max_det),
            "--class-name",
            self._class_name,
        ]
        if self._model_fallback_path:
            command.extend(["--fallback-model", self._model_fallback_path])
        command.append("--half" if self._half else "--no-half")

        try:
            self._worker = subprocess.Popen(
                command,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0,
            )
        except OSError as exc:
            self._worker = None
            self._consecutive_failures += 1
            self.get_logger().error(f"启动 YOLO worker 失败：{exc}")
            return False

        self._sequence = 0
        response = self._read_response(self._startup_timeout_s)
        if response is None or not response.get("ready"):
            self.get_logger().error("YOLO worker 握手失败（模型加载超时或异常），详见 worker stderr")
            self._terminate_worker()
            self._consecutive_failures += 1
            return False

        self.get_logger().info(
            f"YOLO worker 就绪：model={response.get('model')}, device={response.get('device')}, "
            f"imgsz={response.get('imgsz')}, names={response.get('names')}"
        )
        return True

    def _send_request(self, request: dict, frame_bytes: bytes) -> bool:
        if self._worker is None or self._worker.stdin is None:
            return False
        header = json.dumps(request, separators=(",", ":")).encode("utf-8")
        payload = HEADER_LENGTH.pack(len(header)) + header + frame_bytes
        try:
            self._write_all(self._worker.stdin, payload)
        except (BrokenPipeError, OSError) as exc:
            self._handle_worker_failure(f"写入 worker 失败：{exc}")
            return False
        return True

    @staticmethod
    def _write_all(stream, payload: bytes) -> None:
        """管道 write() 可能短写，这里循环写满，避免帧数据被截断。"""
        view = memoryview(payload)
        while view:
            written = stream.write(view)
            if written is None:
                raise BrokenPipeError("worker stdin closed")
            view = view[written:]

    def _read_response(self, timeout_s: float) -> dict | None:
        """带超时读取一条 worker 消息；超时或 EOF 返回 None。"""
        if self._worker is None or self._worker.stdout is None:
            return None
        self._drain_worker_stderr()
        try:
            ready, _, _ = select.select([self._worker.stdout], [], [], timeout_s)
        except (OSError, ValueError):
            return None
        if not ready:
            return None

        header_bytes = self._read_exact(self._worker.stdout, HEADER_LENGTH.size)
        if header_bytes is None:
            return None
        (header_size,) = HEADER_LENGTH.unpack(header_bytes)
        if header_size <= 0 or header_size > 1024 * 1024:
            self.get_logger().error(f"worker 回包头长度非法：{header_size}")
            return None
        payload = self._read_exact(self._worker.stdout, header_size)
        if payload is None:
            return None
        try:
            return json.loads(payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            self.get_logger().error(f"worker 回包 JSON 解析失败：{exc}")
            return None

    @staticmethod
    def _read_exact(stream, count: int) -> bytes | None:
        chunks: list[bytes] = []
        remaining = count
        while remaining > 0:
            chunk = stream.read(remaining)
            if not chunk:
                return None
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)

    def _drain_worker_stderr(self) -> None:
        if self._worker is None or self._worker.stderr is None:
            return
        try:
            os.set_blocking(self._worker.stderr.fileno(), False)
        except (OSError, ValueError):
            return
        while True:
            try:
                line = self._worker.stderr.readline()
            except (OSError, ValueError):
                return
            if not line:
                return
            text = line.decode("utf-8", errors="replace").rstrip()
            if text:
                self.get_logger().info(f"[worker] {text[:500]}")

    def _handle_worker_failure(self, reason: str) -> None:
        self._consecutive_failures += 1
        self.get_logger().warning(f"{reason}；连续失败 {self._consecutive_failures}/{self._worker_restart_limit}")
        self._drain_worker_stderr()
        self._terminate_worker()

    def _terminate_worker(self) -> None:
        worker = self._worker
        self._worker = None
        if worker is None:
            return
        try:
            if worker.stdin is not None:
                worker.stdin.close()
        except OSError:
            pass
        try:
            worker.terminate()
            worker.wait(timeout=2.0)
        except (subprocess.TimeoutExpired, OSError):
            try:
                worker.kill()
                worker.wait(timeout=1.0)
            except (subprocess.TimeoutExpired, OSError):
                pass

    # ---------------------------------------------------------------- 结果处理

    def _handle_response(self, response: dict, stamp_ns: int, e2e_ms: float) -> None:
        if not response.get("ok"):
            self.get_logger().warning(f"worker 报错：{response.get('error')}", throttle_duration_sec=5.0)
            self._publish_detections([], stamp_ns)
            self._rows.append(
                _DetectionRow(stamp_s=stamp_ns * 1e-9, inference_ms=_finite_or_nan(response.get("inference_ms")), e2e_ms=e2e_ms)
            )
            return

        boxes = response.get("boxes") or []
        candidates: list[list[float]] = []
        for box in boxes:
            if not isinstance(box, (list, tuple)) or len(box) != 5:
                continue
            values = [float(item) for item in box]
            if not all(math.isfinite(item) for item in values):
                continue
            if values[2] <= 0.0 or values[3] <= 0.0:
                continue
            if not 0.0 <= values[4] <= 1.0:
                continue
            candidates.append(values)
        candidates.sort(key=lambda item: item[4], reverse=True)

        # 单目标契约：只发布最高分一条，下游不用再做“选谁”的歧义决策。
        best = candidates[0] if candidates else None
        self._publish_detections([best] if best is not None else [], stamp_ns)
        self._rows.append(
            _DetectionRow(
                stamp_s=stamp_ns * 1e-9,
                u=best[0] if best else math.nan,
                v=best[1] if best else math.nan,
                w=best[2] if best else math.nan,
                h=best[3] if best else math.nan,
                score=best[4] if best else math.nan,
                inference_ms=_finite_or_nan(response.get("inference_ms")),
                e2e_ms=e2e_ms,
                detections=1 if best is not None else 0,
            )
        )

    def _publish_detections(self, detections: list[list[float]], stamp_ns: int) -> None:
        message = Detection2DArray()
        message.header.stamp.sec = stamp_ns // 1_000_000_000
        message.header.stamp.nanosec = stamp_ns % 1_000_000_000
        message.header.frame_id = self._camera_frame_id

        for box in detections:
            detection = Detection2D()
            detection.header = message.header
            # worker 返回的就是原图像素坐标（ultralytics 已反 letterbox），这里禁止二次缩放。
            detection.bbox.center.position.x = float(box[0])
            detection.bbox.center.position.y = float(box[1])
            detection.bbox.center.theta = 0.0
            detection.bbox.size_x = float(box[2])
            detection.bbox.size_y = float(box[3])

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = "drone"
            hypothesis.hypothesis.score = float(box[4])
            detection.results.append(hypothesis)
            message.detections.append(detection)

        self._detections_pub.publish(message)

    def _maybe_log_debug(self) -> None:
        if not self._debug_log:
            return
        now_mono_ns = time.monotonic_ns()
        if self._last_debug_log_ns is not None:
            if (now_mono_ns - self._last_debug_log_ns) * 1e-9 < self._debug_log_period_s:
                return
        self._last_debug_log_ns = now_mono_ns
        last = self._rows[-1] if self._rows else None
        if last is None:
            self.get_logger().info("vision_detector 尚未处理任何帧")
            return
        self.get_logger().info(
            "vision_detector "
            f"processed={len(self._rows)} fired={self._sequence} "
            f"dropped(throttle/stale/invalid)={self._dropped_throttle}/{self._dropped_stale}/{self._dropped_invalid} "
            f"last: t={last.stamp_s:.3f} n={last.detections} score={last.score:.3f} "
            f"infer_ms={last.inference_ms:.2f} e2e_ms={last.e2e_ms:.2f}"
        )

    # ---------------------------------------------------------------- 落盘

    def save_recording(self) -> None:
        if not self._rows:
            self._log_shutdown_safe("warn", "没有处理任何图像帧，不写 yolo_detections.csv")
            return
        try:
            path = self._stats_csv
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("w", newline="", encoding="utf-8") as file:
                writer = csv.DictWriter(file, fieldnames=CSV_FIELDS)
                writer.writeheader()
                for row in self._rows:
                    writer.writerow(
                        {
                            "stamp_s": row.stamp_s,
                            "u": row.u,
                            "v": row.v,
                            "w": row.w,
                            "h": row.h,
                            "score": row.score,
                            "inference_ms": row.inference_ms,
                            "e2e_ms": row.e2e_ms,
                            "detections": row.detections,
                        }
                    )
            self._log_shutdown_safe("info", f"saved YOLO detection stats to {path}")
        except Exception as exc:  # noqa: BLE001
            self._log_shutdown_safe("error", f"保存 yolo_detections.csv 失败：{exc}")

    def _log_shutdown_safe(self, level: str, message: str) -> None:
        if rclpy.ok():
            getattr(self.get_logger(), level)(message)
            return
        stream = sys.stderr if level == "error" else sys.stdout
        print(f"[{level.upper()}] [vision_detector]: {message}", file=stream)


def _finite_or_nan(value: object) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return math.nan
    return result if math.isfinite(result) else math.nan


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    try:
        node = VisionDetector()
    except ValueError as exc:
        print(f"[ERROR] [vision_detector]: {exc}", file=sys.stderr)
        if rclpy.ok():
            rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.save_recording()
        node._terminate_worker()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
