"""`vision_detector` 的 ROS 环境单元测试。

用假 worker 脚本（不需要 torch/ultralytics）驱动真实子进程协议，覆盖：
握手与字段、原图坐标不缩放、空检测、header 复制、节流与过期丢帧、
worker 超时/崩溃重启与重启上限、错误回包、jpeg 帧格式、统计 CSV 与数据集录制。

运行：

```bash
cd 7_2Dsimulation
colcon build --packages-select gazebosimulation2d
source install/setup.bash
colcon test --packages-select gazebosimulation2d && colcon test-result --verbose
```
"""

from __future__ import annotations

import csv
import sys

import numpy as np
import pytest
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image

from gazebosimulation2d.vision_detector import CSV_FIELDS, VisionDetector

FAKE_WORKER_SOURCE = r'''
import json, os, struct, sys, time

HEADER = struct.Struct(">I")
OUT = os.fdopen(os.dup(sys.stdout.fileno()), "wb", buffering=0)
os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
sys.stdout = sys.stderr


def send(payload):
    body = json.dumps(payload).encode()
    OUT.write(HEADER.pack(len(body)) + body)


def read_exact(count):
    chunks = []
    remaining = count
    while remaining:
        chunk = sys.stdin.buffer.read(remaining)
        if not chunk:
            return None
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


mode = os.environ.get("FAKE_WORKER_MODE", "fixed")
if mode == "crash_on_start":
    # 模拟模型路径不存在等启动失败：只写 stderr 后退出，不发 ready 握手。
    print("worker 启动失败：模型不存在", file=sys.stderr, flush=True)
    sys.exit(3)
boxes = json.loads(os.environ.get("FAKE_WORKER_BOXES", "[[100.0,200.0,20.0,10.0,0.9]]"))
echo = os.environ.get("FAKE_WORKER_ECHO_HEADER", "")
send({"ready": True, "model": "fake.pt", "device": "0", "imgsz": 640, "names": {"0": "uav"}})
if mode == "exit_immediately":
    sys.exit(0)

count = 0
while True:
    size_bytes = read_exact(4)
    if size_bytes is None:
        break
    (size,) = HEADER.unpack(size_bytes)
    header = json.loads(read_exact(size).decode("utf-8"))
    payload_size = int(header.get("bytes", 0))
    if payload_size:
        read_exact(payload_size)
    if echo:
        with open(echo, "a", encoding="utf-8") as handle:
            handle.write(json.dumps(header) + "\n")
    count += 1
    if mode == "sleep":
        time.sleep(5.0)
        continue
    if mode == "error":
        send({"seq": header.get("seq"), "ok": False, "error": "boom"})
        continue
    send({"seq": header.get("seq"), "stamp_ns": header.get("stamp_ns"), "inference_ms": 1.5,
          "boxes": boxes, "class_name": "uav", "ok": True})
    if mode == "exit_after_one" and count == 1:
        sys.exit(0)
'''


class PublisherCapture:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


class LoggerCapture:
    """捕获节点日志：rclpy 日志不走 Python logging，测试里用替身断言消息。"""

    def __init__(self) -> None:
        self.messages: list[tuple[str, str]] = []

    def info(self, message, **kwargs) -> None:
        self.messages.append(("info", str(message)))

    def warning(self, message, **kwargs) -> None:
        self.messages.append(("warning", str(message)))

    def error(self, message, **kwargs) -> None:
        self.messages.append(("error", str(message)))

    def debug(self, message, **kwargs) -> None:
        self.messages.append(("debug", str(message)))


class FakeTime:
    now_ns = 0

    @classmethod
    def monotonic_ns(cls) -> int:
        return cls.now_ns


@pytest.fixture(scope="module")
def fake_worker_script(tmp_path_factory) -> str:
    path = tmp_path_factory.mktemp("fake_worker") / "yolo_worker.py"
    path.write_text(FAKE_WORKER_SOURCE, encoding="utf-8")
    return str(path)


def make_node(fake_worker_script: str, tmp_path, **overrides) -> VisionDetector:
    parameters = {
        "yolo_python": sys.executable,
        "yolo_worker_script": fake_worker_script,
        "model_path": "/nonexistent/fake.pt",
        "stats_csv": str(tmp_path / "yolo_detections.csv"),
        "dataset_output_dir": str(tmp_path / "dataset"),
        "conf": 0.25,
    }
    parameters.update(overrides)
    node = VisionDetector([Parameter(name, value=value) for name, value in parameters.items()])
    node._detections_pub = PublisherCapture()
    return node


def make_image(
    node: VisionDetector,
    stamp_ns: int | None = None,
    width: int = 64,
    height: int = 48,
    encoding: str = "rgb8",
    frame_id: str = "camera_link_optical",
) -> Image:
    if stamp_ns is None:
        stamp_ns = int(node.get_clock().now().nanoseconds)
    message = Image()
    message.header.stamp.sec = stamp_ns // 1_000_000_000
    message.header.stamp.nanosec = stamp_ns % 1_000_000_000
    message.header.frame_id = frame_id
    message.width = width
    message.height = height
    message.encoding = encoding
    message.is_bigendian = 0
    message.step = width * 3
    message.data = np.zeros((height, width, 3), dtype=np.uint8).tobytes()
    return message


class TestParameters:
    def test_missing_python_is_rejected(self, fake_worker_script, tmp_path) -> None:
        with pytest.raises(ValueError):
            make_node(fake_worker_script, tmp_path, yolo_python="")

    def test_missing_model_is_rejected(self, fake_worker_script, tmp_path) -> None:
        with pytest.raises(ValueError):
            make_node(fake_worker_script, tmp_path, model_path="")

    def test_bad_frame_format_is_rejected(self, fake_worker_script, tmp_path) -> None:
        with pytest.raises(ValueError):
            make_node(fake_worker_script, tmp_path, frame_format="png")

    def test_missing_worker_script_is_rejected(self, tmp_path) -> None:
        with pytest.raises(ValueError):
            make_node(str(tmp_path / "missing.py"), tmp_path)


class TestDetectionPipeline:
    def test_best_detection_keeps_original_pixel_coordinates(self, fake_worker_script, tmp_path) -> None:
        node = make_node(fake_worker_script, tmp_path)
        try:
            node._image_callback(make_image(node))
            assert len(node._detections_pub.messages) == 1
            message = node._detections_pub.messages[0]
            assert len(message.detections) == 1
            detection = message.detections[0]
            # 假 worker 返回两条：[[100,200,...], [300,400,...]]。
            assert detection.bbox.center.position.x == pytest.approx(100.0)
            assert detection.bbox.center.position.y == pytest.approx(200.0)
            assert detection.bbox.size_x == pytest.approx(20.0)
            assert detection.bbox.size_y == pytest.approx(10.0)
            assert detection.results[0].hypothesis.class_id == "drone"
            assert detection.results[0].hypothesis.score == pytest.approx(0.9)

            row = node._rows[-1]
            assert row.detections == 1
            assert row.u == pytest.approx(100.0)
            assert row.inference_ms == pytest.approx(1.5)
            assert row.e2e_ms >= 0.0
        finally:
            node._terminate_worker()
            node.destroy_node()

    def test_single_target_contract_publishes_only_best(self, fake_worker_script, tmp_path, monkeypatch) -> None:
        monkeypatch.setenv(
            "FAKE_WORKER_BOXES",
            "[[10.0,20.0,4.0,4.0,0.4],[30.0,40.0,6.0,6.0,0.95]]",
        )
        node = make_node(fake_worker_script, tmp_path)
        try:
            node._image_callback(make_image(node))
            message = node._detections_pub.messages[-1]
            assert len(message.detections) == 1
            assert message.detections[0].bbox.center.position.x == pytest.approx(30.0)
            assert message.detections[0].results[0].hypothesis.score == pytest.approx(0.95)
        finally:
            node._terminate_worker()
            node.destroy_node()

    def test_header_stamp_and_frame_are_copied(self, fake_worker_script, tmp_path) -> None:
        node = make_node(fake_worker_script, tmp_path)
        try:
            stamp_ns = int(node.get_clock().now().nanoseconds)
            node._image_callback(make_image(node, stamp_ns=stamp_ns))
            message = node._detections_pub.messages[-1]
            assert message.header.stamp.sec == stamp_ns // 1_000_000_000
            assert message.header.stamp.nanosec == stamp_ns % 1_000_000_000
            assert message.header.frame_id == "camera_link_optical"
            assert message.detections[0].header.frame_id == "camera_link_optical"
        finally:
            node._terminate_worker()
            node.destroy_node()

    def test_empty_result_publishes_empty_array(self, fake_worker_script, tmp_path, monkeypatch) -> None:
        monkeypatch.setenv("FAKE_WORKER_BOXES", "[]")
        node = make_node(fake_worker_script, tmp_path)
        try:
            node._image_callback(make_image(node))
            message = node._detections_pub.messages[-1]
            assert message.detections == []
            row = node._rows[-1]
            assert row.detections == 0
            assert np.isnan(row.score)
        finally:
            node._terminate_worker()
            node.destroy_node()

    def test_worker_error_response_is_recorded(self, fake_worker_script, tmp_path, monkeypatch) -> None:
        monkeypatch.setenv("FAKE_WORKER_MODE", "error")
        node = make_node(fake_worker_script, tmp_path)
        try:
            node._image_callback(make_image(node))
            message = node._detections_pub.messages[-1]
            assert message.detections == []
            assert node._rows[-1].detections == 0
            assert node._consecutive_failures == 0
        finally:
            node._terminate_worker()
            node.destroy_node()

    def test_unsupported_encoding_is_dropped(self, fake_worker_script, tmp_path) -> None:
        node = make_node(fake_worker_script, tmp_path)
        try:
            image = make_image(node)
            image.encoding = "32FC1"
            node._image_callback(image)
            assert node._detections_pub.messages == []
            assert node._dropped_invalid == 1
        finally:
            node._terminate_worker()
            node.destroy_node()

    def test_short_image_data_is_dropped(self, fake_worker_script, tmp_path) -> None:
        node = make_node(fake_worker_script, tmp_path)
        try:
            image = make_image(node)
            image.data = b"\x00" * 10
            node._image_callback(image)
            assert node._detections_pub.messages == []
            assert node._dropped_invalid == 1
        finally:
            node._terminate_worker()
            node.destroy_node()


class TestThrottleAndAge:
    def test_throttle_drops_frames_within_period(self, fake_worker_script, tmp_path, monkeypatch) -> None:
        import gazebosimulation2d.vision_detector as detector_module

        monkeypatch.setattr(detector_module, "time", FakeTime)
        node = make_node(fake_worker_script, tmp_path, process_hz=10.0)
        try:
            FakeTime.now_ns = 0
            node._image_callback(make_image(node))
            # 30 ms < 100 ms 周期，第二帧必须被节流丢掉。
            FakeTime.now_ns = 30_000_000
            node._image_callback(make_image(node))
            assert len(node._detections_pub.messages) == 1
            assert node._dropped_throttle == 1
            FakeTime.now_ns = 120_000_000
            node._image_callback(make_image(node))
            assert len(node._detections_pub.messages) == 2
        finally:
            node._terminate_worker()
            node.destroy_node()

    def test_stale_frame_is_dropped(self, fake_worker_script, tmp_path) -> None:
        node = make_node(fake_worker_script, tmp_path, max_frame_age_s=0.2)
        try:
            now_ns = int(node.get_clock().now().nanoseconds)
            node._image_callback(make_image(node, stamp_ns=now_ns - 5_000_000_000))
            assert node._detections_pub.messages == []
            assert node._dropped_stale == 1
            assert node._dropped_invalid == 0
        finally:
            node._terminate_worker()
            node.destroy_node()


class TestWorkerLifecycle:
    def test_timeout_restarts_worker_and_recovers(self, fake_worker_script, tmp_path, monkeypatch) -> None:
        monkeypatch.setenv("FAKE_WORKER_MODE", "sleep")
        node = make_node(fake_worker_script, tmp_path, inference_timeout_s=0.3, worker_restart_limit=3)
        try:
            node._image_callback(make_image(node))
            assert node._detections_pub.messages == []
            assert node._consecutive_failures == 1
            assert node._worker is None

            monkeypatch.delenv("FAKE_WORKER_MODE")
            node._image_callback(make_image(node))
            assert len(node._detections_pub.messages) == 1
            assert node._consecutive_failures == 0
        finally:
            node._terminate_worker()
            node.destroy_node()

    def test_worker_crash_between_frames_is_restarted(self, fake_worker_script, tmp_path, monkeypatch) -> None:
        monkeypatch.setenv("FAKE_WORKER_MODE", "exit_after_one")
        # 测试里两次回调间隔远小于默认节流周期，关掉节流才能观察到“崩溃后重启”。
        node = make_node(fake_worker_script, tmp_path, process_hz=1000.0)
        try:
            node._image_callback(make_image(node))
            assert len(node._detections_pub.messages) == 1
            # 等假 worker 真正退出，再触发“下一帧发现 worker 已死并重启”的路径；
            # 否则第二次回调可能抢在进程退出前把请求写进正在关闭的管道。
            assert node._worker is not None
            node._worker.wait(timeout=2.0)
            node._image_callback(make_image(node))
            assert len(node._detections_pub.messages) == 2
            # 重启后的 worker 正常工作，失败计数被成功回包清零。
            assert node._consecutive_failures == 0
        finally:
            node._terminate_worker()
            node.destroy_node()

    def test_handshake_failure_surfaces_worker_stderr(self, fake_worker_script, tmp_path, monkeypatch) -> None:
        monkeypatch.setenv("FAKE_WORKER_MODE", "crash_on_start")
        node = make_node(fake_worker_script, tmp_path, worker_restart_limit=1)
        logger = LoggerCapture()
        monkeypatch.setattr(node, "get_logger", lambda: logger)
        try:
            node._image_callback(make_image(node))
            assert node._consecutive_failures == 1
            assert node._worker is None
            # 启动失败的 worker stderr 必须转进 ROS 日志，而不是只报“握手失败”。
            assert any(
                "[worker]" in message and "模型不存在" in message
                for _, message in logger.messages
            )
        finally:
            node._terminate_worker()
            node.destroy_node()

    def test_gives_up_after_restart_limit(self, fake_worker_script, tmp_path, monkeypatch) -> None:
        monkeypatch.setenv("FAKE_WORKER_MODE", "exit_immediately")
        node = make_node(fake_worker_script, tmp_path, worker_restart_limit=1, process_hz=1000.0)
        try:
            node._image_callback(make_image(node))
            assert node._consecutive_failures == 1
            node._image_callback(make_image(node))
            assert node._worker_gave_up
            assert node._detections_pub.messages == []
        finally:
            node._terminate_worker()
            node.destroy_node()


class TestRecording:
    def test_stats_csv_is_written(self, fake_worker_script, tmp_path) -> None:
        node = make_node(fake_worker_script, tmp_path)
        stats_path = tmp_path / "yolo_detections.csv"
        try:
            node._image_callback(make_image(node))
            node.save_recording()
        finally:
            node._terminate_worker()
            node.destroy_node()

        with stats_path.open(newline="", encoding="utf-8") as file:
            reader = csv.DictReader(file)
            assert reader.fieldnames == list(CSV_FIELDS)
            rows = list(reader)
        assert len(rows) == 1
        assert float(rows[0]["u"]) == pytest.approx(100.0)
        assert int(rows[0]["detections"]) == 1

    def test_dataset_frames_are_saved(self, fake_worker_script, tmp_path) -> None:
        pytest.importorskip("cv2")
        node = make_node(fake_worker_script, tmp_path, save_frame_hz=10.0)
        try:
            stamp_ns = int(node.get_clock().now().nanoseconds)
            node._image_callback(make_image(node, stamp_ns=stamp_ns))
        finally:
            node._terminate_worker()
            node.destroy_node()
        saved = list((tmp_path / "dataset" / "frames").glob("*.jpg"))
        assert len(saved) == 1
        assert saved[0].name == f"{stamp_ns}.jpg"


class TestFrameFormat:
    def test_jpeg_format_is_sent_with_byte_count(self, fake_worker_script, tmp_path, monkeypatch) -> None:
        pytest.importorskip("cv2")
        header_log = tmp_path / "headers.jsonl"
        monkeypatch.setenv("FAKE_WORKER_ECHO_HEADER", str(header_log))
        node = make_node(fake_worker_script, tmp_path, frame_format="jpeg")
        try:
            node._image_callback(make_image(node))
            assert len(node._detections_pub.messages) == 1
        finally:
            node._terminate_worker()
            node.destroy_node()

        import json

        header = json.loads(header_log.read_text(encoding="utf-8").strip())
        assert header["format"] == "jpeg"
        assert header["encoding"] == "rgb8"
        assert header["bytes"] > 0
