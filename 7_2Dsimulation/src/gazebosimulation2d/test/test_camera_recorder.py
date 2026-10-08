"""`camera_recorder` 与 `image_utils` 的 ROS 环境单元测试。

不启动 Gazebo/PX4：直接构造 `sensor_msgs/Image` 驱动回调，覆盖参数校验、按 stamp 节流、
文件名与编码转换、重启跳过已存在帧、max_frames 上限与非法帧丢弃。

运行：

```bash
cd 7_2Dsimulation
colcon build --packages-select gazebosimulation2d
source install/setup.bash
colcon test --packages-select gazebosimulation2d && colcon test-result --verbose
```
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image

# `colcon test` 下没有额外的 PYTHONPATH：按源码树相对位置把 gazebosimulation2d 加进导入路径。
ROOT = Path(__file__).resolve().parents[3]
for candidate in (ROOT / "src", ROOT / "src" / "gazebosimulation2d"):
    if str(candidate) not in sys.path:
        sys.path.insert(0, str(candidate))

from gazebosimulation2d.camera_recorder import CameraRecorder
from gazebosimulation2d.image_utils import image_message_to_bgr

CHANNELS = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}


def make_node(tmp_path, **overrides) -> CameraRecorder:
    parameters = {"output_dir": str(tmp_path / "frames"), "save_hz": 1.0}
    parameters.update(overrides)
    return CameraRecorder([Parameter(name, value=value) for name, value in parameters.items()])


def make_image(
    stamp_ns: int,
    width: int = 4,
    height: int = 3,
    encoding: str = "rgb8",
    frame_id: str = "camera_link_optical",
) -> Image:
    message = Image()
    message.header.stamp.sec = stamp_ns // 1_000_000_000
    message.header.stamp.nanosec = stamp_ns % 1_000_000_000
    message.header.frame_id = frame_id
    message.width = width
    message.height = height
    message.encoding = encoding
    message.is_bigendian = 0
    channels = CHANNELS[encoding]
    message.step = width * channels
    data = np.zeros((height, width, channels), dtype=np.uint8)
    data[..., 0] = 10
    if channels > 1:
        data[..., 1] = 20
    if channels > 2:
        data[..., 2] = 30
    if channels == 4:
        data[..., 3] = 255
    message.data = data.tobytes()
    return message


class TestParameters:
    def test_save_hz_must_be_positive(self, tmp_path) -> None:
        with pytest.raises(ValueError):
            make_node(tmp_path, save_hz=0.0)

    def test_jpeg_quality_range_is_checked(self, tmp_path) -> None:
        with pytest.raises(ValueError):
            make_node(tmp_path, jpeg_quality=0)
        with pytest.raises(ValueError):
            make_node(tmp_path, jpeg_quality=101)

    def test_max_frames_must_be_non_negative(self, tmp_path) -> None:
        with pytest.raises(ValueError):
            make_node(tmp_path, max_frames=-1)

    def test_image_topic_must_be_absolute(self, tmp_path) -> None:
        with pytest.raises(ValueError):
            make_node(tmp_path, image_topic="camera/image_raw")


class TestRecording:
    def test_saves_at_configured_period(self, tmp_path) -> None:
        node = make_node(tmp_path)
        base = 10_000_000_000
        try:
            node._image_callback(make_image(base))
            node._image_callback(make_image(base + 500_000_000))
            node._image_callback(make_image(base + 1_000_000_000))
            frames = sorted((tmp_path / "frames").glob("*.jpg"))
            assert [path.name for path in frames] == [
                f"{base}.jpg",
                f"{base + 1_000_000_000}.jpg",
            ]
            assert node._saved == 2
            assert node._dropped_throttle == 1
        finally:
            node.destroy_node()

    def test_filename_uses_image_stamp_and_content_is_bgr(self, tmp_path) -> None:
        cv2 = pytest.importorskip("cv2")
        node = make_node(tmp_path)
        stamp = 123_000_000_000
        try:
            node._image_callback(make_image(stamp, encoding="rgb8"))
        finally:
            node.destroy_node()

        path = tmp_path / "frames" / f"{stamp}.jpg"
        assert path.is_file()
        image = cv2.imread(str(path))
        assert image is not None
        # rgb8 输入 (10, 20, 30) -> BGR (30, 20, 10)；jpeg 有损，留 5 级容差。
        assert image[0, 0].tolist() == pytest.approx([30, 20, 10], abs=5)

    def test_existing_frame_is_skipped_on_restart(self, tmp_path) -> None:
        stamp = 5_000_000_000
        frames = tmp_path / "frames"
        frames.mkdir(parents=True)
        existing = frames / f"{stamp}.jpg"
        existing.write_bytes(b"already here")
        node = make_node(tmp_path)
        try:
            node._image_callback(make_image(stamp))
            assert node._saved == 0
            assert existing.read_bytes() == b"already here"
            assert node._last_saved_stamp_ns == stamp
        finally:
            node.destroy_node()

    def test_invalid_encoding_is_dropped(self, tmp_path) -> None:
        node = make_node(tmp_path)
        try:
            message = make_image(1_000_000_000)
            message.encoding = "png"
            node._image_callback(message)
            assert node._dropped_invalid == 1
            assert node._saved == 0
            assert list((tmp_path / "frames").glob("*")) == []
        finally:
            node.destroy_node()

    def test_max_frames_stops_saving(self, tmp_path) -> None:
        node = make_node(tmp_path, max_frames=1)
        try:
            node._image_callback(make_image(1_000_000_000))
            node._image_callback(make_image(2_000_000_000))
            assert node._saved == 1
            assert len(list((tmp_path / "frames").glob("*.jpg"))) == 1
        finally:
            node.destroy_node()

    def test_zero_stamp_falls_back_to_node_clock(self, tmp_path) -> None:
        node = make_node(tmp_path)
        try:
            node._image_callback(make_image(0))
            frames = list((tmp_path / "frames").glob("*.jpg"))
            assert node._saved == 1
            assert len(frames) == 1
            assert int(frames[0].stem) > 0
        finally:
            node.destroy_node()


class TestImageConversion:
    def test_rgb8_is_converted_to_bgr(self) -> None:
        bgr = image_message_to_bgr(make_image(0, encoding="rgb8"))
        assert bgr.shape == (3, 4, 3)
        assert bgr[0, 0].tolist() == [30, 20, 10]

    def test_rgba8_drops_alpha(self) -> None:
        bgr = image_message_to_bgr(make_image(0, encoding="rgba8"))
        assert bgr.shape == (3, 4, 3)
        assert bgr[0, 0].tolist() == [30, 20, 10]

    def test_mono8_is_replicated(self) -> None:
        bgr = image_message_to_bgr(make_image(0, encoding="mono8"))
        assert bgr.shape == (3, 4, 3)
        assert bgr[0, 0].tolist() == [10, 10, 10]

    def test_short_data_raises(self) -> None:
        message = make_image(0, encoding="rgb8")
        message.data = message.data[:-1]
        with pytest.raises(ValueError):
            image_message_to_bgr(message)

    def test_bigendian_raises(self) -> None:
        message = make_image(0, encoding="rgb8")
        message.is_bigendian = 1
        with pytest.raises(ValueError):
            image_message_to_bgr(message)
