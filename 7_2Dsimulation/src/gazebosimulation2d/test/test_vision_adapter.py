"""vision_adapter 的 ROS 环境单元测试。

不启动 Gazebo 或 PX4，只构造 `CameraInfo`/`VehicleOdometry` 消息直接驱动节点，
覆盖参数校验、内参校验、odometry 新鲜度与配对、时间基准、消息字段与协方差、
CSV 记录。

运行：

```bash
cd 7_2Dsimulation
colcon build --packages-up-to gazebosimulation2d
source install/setup.bash
colcon test --packages-select gazebosimulation2d && colcon test-result --verbose
```
"""

from __future__ import annotations

import csv
import time

import numpy as np
import pytest
import rclpy
from px4_msgs.msg import VehicleOdometry
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo

from gazebosimulation2d.vision_adapter import (
    CSV_FIELDS,
    POSE_UNKNOWN_VARIANCE,
    TRUTH_BBOX_SIZE_PX,
    VisionAdapter,
)

IDENTITY_QUATERNION = [1.0, 0.0, 0.0, 0.0]
# ENU (0, 0, 8)：追踪机固定高度 8 m。
PURSUER_POSITION_NED = (0.0, 0.0, -8.0)
# ENU (3, 4, 1)：目标在固定 1 m 平面，位于相机视场内。
TARGET_POSITION_NED = (4.0, 3.0, -1.0)


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    adapter = VisionAdapter([Parameter("vision_source", value="truth")])
    yield adapter
    adapter.destroy_node()


def make_camera_info(
    width: int = 1280,
    height: int = 960,
    frame_id: str = "camera_link_optical",
    k: list[float] | None = None,
    d: list[float] | None = None,
) -> CameraInfo:
    message = CameraInfo()
    message.width = width
    message.height = height
    message.k = k if k is not None else [539.9363, 0.0, 640.0, 0.0, 539.9363, 480.0, 0.0, 0.0, 1.0]
    message.d = d if d is not None else [0.0, 0.0, 0.0, 0.0, 0.0]
    message.header.frame_id = frame_id
    return message


def make_odometry(
    position_ned,
    quaternion: list[float] | None = None,
    pose_frame: int = VehicleOdometry.POSE_FRAME_NED,
    timestamp_sample: int = 123456,
) -> VehicleOdometry:
    message = VehicleOdometry()
    message.pose_frame = pose_frame
    message.position = [float(value) for value in position_ned]
    message.q = list(quaternion if quaternion is not None else IDENTITY_QUATERNION)
    message.timestamp_sample = timestamp_sample
    return message


def set_pursuer(adapter: VisionAdapter, position_ned=PURSUER_POSITION_NED, quaternion=None) -> None:
    adapter._pursuer_odometry_callback(make_odometry(position_ned, quaternion))


def set_target(adapter: VisionAdapter, position_ned=TARGET_POSITION_NED) -> None:
    adapter._target_odometry_callback(make_odometry(position_ned))


def prime_valid_inputs(adapter: VisionAdapter) -> None:
    """让节点处于“内参有效 + 两机数据新鲜配对”的状态。"""
    adapter._camera_info_callback(make_camera_info())
    set_pursuer(adapter)
    set_target(adapter)


class TestParameters:
    def test_defaults_are_loaded(self, node: VisionAdapter) -> None:
        assert node._vision_source == "truth"
        assert node._camera_frame_id == "camera_link_optical"
        assert node._camera_mount_xyz == pytest.approx((0.0, 0.0, 0.10))
        assert node._camera_mount_rpy_rad[1] == pytest.approx(np.pi / 2.0)
        assert node._pose_timeout_s == pytest.approx(0.2)
        assert node._pose_pair_tolerance_s == pytest.approx(0.05)
        assert node._target_base_altitude == pytest.approx(1.0)

    def test_unsupported_source_is_rejected(self) -> None:
        with pytest.raises(ValueError):
            VisionAdapter([Parameter("vision_source", value="yolo")])

    def test_off_source_is_rejected_when_started(self) -> None:
        with pytest.raises(ValueError):
            VisionAdapter([Parameter("vision_source", value="off")])

    def test_nonpositive_rate_is_rejected(self) -> None:
        with pytest.raises(ValueError):
            VisionAdapter([
                Parameter("vision_source", value="truth"),
                Parameter("truth_rate_hz", value=0.0),
            ])

    def test_invalid_mount_array_is_rejected(self) -> None:
        with pytest.raises(ValueError):
            VisionAdapter([
                Parameter("vision_source", value="truth"),
                Parameter("camera_mount_xyz", value=[1.0, 2.0]),
            ])
        with pytest.raises(ValueError):
            VisionAdapter([
                Parameter("vision_source", value="truth"),
                Parameter("camera_mount_rpy_deg", value=[0.0, float("nan"), 0.0]),
            ])

    def test_empty_camera_frame_is_rejected(self) -> None:
        with pytest.raises(ValueError):
            VisionAdapter([
                Parameter("vision_source", value="truth"),
                Parameter("camera_frame_id", value=""),
            ])


class TestCameraInfo:
    def test_valid_camera_info_is_cached(self, node: VisionAdapter) -> None:
        node._camera_info_callback(make_camera_info())
        assert node._camera_intrinsics is not None
        assert node._camera_intrinsics.width == 1280
        assert node._camera_intrinsics.fx == pytest.approx(539.9363)
        assert node._camera_info_reason == ""

    def test_wrong_frame_is_rejected(self, node: VisionAdapter) -> None:
        node._camera_info_callback(make_camera_info(frame_id="x500_mono_cam_down_0::camera_link::imager"))
        assert node._camera_intrinsics is None
        assert node._camera_info_reason == "invalid_frame"

    def test_nonzero_distortion_is_rejected(self, node: VisionAdapter) -> None:
        node._camera_info_callback(make_camera_info(d=[0.1, 0.0, 0.0, 0.0, 0.0]))
        assert node._camera_intrinsics is None
        assert node._camera_info_reason == "unsupported_distortion"

    def test_nonfinite_intrinsics_are_rejected(self, node: VisionAdapter) -> None:
        bad_k = [float("nan"), 0.0, 640.0, 0.0, 539.9363, 480.0, 0.0, 0.0, 1.0]
        node._camera_info_callback(make_camera_info(k=bad_k))
        assert node._camera_intrinsics is None
        assert node._camera_info_reason == "invalid_intrinsics"

    def test_invalid_size_is_rejected(self, node: VisionAdapter) -> None:
        node._camera_info_callback(make_camera_info(width=0))
        assert node._camera_intrinsics is None
        assert node._camera_info_reason == "invalid_camera_info"


class TestMeasurement:
    def test_valid_measurement_roundtrip(self, node: VisionAdapter) -> None:
        prime_valid_inputs(node)
        node._on_timer()

        record = node._records[-1]
        assert record.valid
        assert record.invalid_reason == ""
        # camera 位于 ENU (0, 0, 8.1)，目标平面 1 m，水平高差 7.1 m；光学 x 指东、y 指南。
        assert record.u_ref == pytest.approx(640.0 + 539.9363 * 3.0 / 7.1, abs=1e-3)
        assert record.v_ref == pytest.approx(480.0 - 539.9363 * 4.0 / 7.1, abs=1e-3)
        assert record.target_x_est == pytest.approx(3.0, abs=1e-6)
        assert record.target_y_est == pytest.approx(4.0, abs=1e-6)
        assert record.target_x_ref == pytest.approx(3.0, abs=1e-9)
        assert record.target_y_ref == pytest.approx(4.0, abs=1e-9)
        assert record.target_plane_z == pytest.approx(1.0)
        assert record.target_z_odom == pytest.approx(1.0)
        assert record.position_roundtrip_error_m < 1e-6

    def test_missing_data_reasons(self, node: VisionAdapter) -> None:
        node._camera_info_callback(make_camera_info())
        node._on_timer()
        assert node._records[-1].invalid_reason == "no_pursuer_odometry"

        set_pursuer(node)
        node._on_timer()
        assert node._records[-1].invalid_reason == "no_target_odometry"

    def test_missing_camera_info_reason(self, node: VisionAdapter) -> None:
        set_pursuer(node)
        set_target(node)
        node._on_timer()
        assert node._records[-1].invalid_reason == "no_camera_info"

    def test_stale_pursuer_is_rejected(self, node: VisionAdapter) -> None:
        prime_valid_inputs(node)
        now_ns = time.monotonic_ns()
        node._pursuer_received_ns = now_ns - int(0.5e9)
        node._target_received_ns = now_ns

        node._on_timer()
        record = node._records[-1]
        assert record.invalid_reason == "stale_pursuer_odometry"
        assert record.pursuer_age_ms == pytest.approx(500.0, abs=50.0)

    def test_stale_target_is_rejected(self, node: VisionAdapter) -> None:
        prime_valid_inputs(node)
        now_ns = time.monotonic_ns()
        node._pursuer_received_ns = now_ns
        node._target_received_ns = now_ns - int(0.5e9)

        node._on_timer()
        assert node._records[-1].invalid_reason == "stale_target_odometry"

    def test_pair_delta_is_rejected(self, node: VisionAdapter) -> None:
        prime_valid_inputs(node)
        now_ns = time.monotonic_ns()
        node._pursuer_received_ns = now_ns - int(0.02e9)
        node._target_received_ns = now_ns - int(0.10e9)

        node._on_timer()
        record = node._records[-1]
        assert record.invalid_reason == "pose_pair_delta_too_large"
        assert record.pose_pair_delta_ms == pytest.approx(80.0, abs=10.0)

    def test_unsupported_frame_is_rejected(self, node: VisionAdapter) -> None:
        prime_valid_inputs(node)
        node._pursuer_odometry.pose_frame = VehicleOdometry.POSE_FRAME_FRD
        node._on_timer()
        assert node._records[-1].invalid_reason == "unsupported_pursuer_frame"

    def test_invalid_attitude_is_rejected(self, node: VisionAdapter) -> None:
        prime_valid_inputs(node)
        node._pursuer_odometry.q = [0.0, 0.0, 0.0, 0.0]
        node._on_timer()
        assert node._records[-1].invalid_reason == "invalid_pursuer_odometry"

    def test_nonfinite_target_position_is_rejected(self, node: VisionAdapter) -> None:
        prime_valid_inputs(node)
        set_target(node, position_ned=(float("nan"), 0.0, -1.0))
        node._on_timer()
        assert node._records[-1].invalid_reason == "invalid_target_odometry"

    def test_out_of_view_target_is_rejected(self, node: VisionAdapter) -> None:
        prime_valid_inputs(node)
        set_target(node, position_ned=(100.0, 100.0, -1.0))
        node._on_timer()
        record = node._records[-1]
        assert record.invalid_reason == "projection_failed"
        assert record.target_x_ref == pytest.approx(100.0)

    def test_elapsed_time_and_ages_use_monotonic_clock(self, node: VisionAdapter, monkeypatch) -> None:
        import gazebosimulation2d.vision_adapter as adapter_module

        class FakeTime:
            now_ns = 0

            @classmethod
            def monotonic_ns(cls) -> int:
                return cls.now_ns

        monkeypatch.setattr(adapter_module, "time", FakeTime)

        node._start_mono_ns = 0
        prime_valid_inputs(node)
        # 样本接收时刻是 0.95 s，量测时刻是 1.0 s：年龄 50 ms 保持新鲜；
        # 如果实现改用 ROS 系统时间做差值，年龄会变成天文数字并被拒绝。
        FakeTime.now_ns = 1_000_000_000
        node._pursuer_received_ns = 950_000_000
        node._target_received_ns = 950_000_000
        node._on_timer()

        record = node._records[-1]
        assert record.elapsed_s == pytest.approx(1.0)
        assert record.pursuer_age_ms == pytest.approx(50.0)
        assert record.valid

    def test_repeated_odometry_samples_are_identifiable(self, node: VisionAdapter) -> None:
        prime_valid_inputs(node)
        node._on_timer()
        node._on_timer()
        assert len(node._records) == 2
        # 两次量测复用同一 odometry 样本，timestamp_sample 相同，可在 CSV 中识别。
        assert node._records[0].target_timestamp_sample_us == node._records[1].target_timestamp_sample_us


class TestMessages:
    def test_detection_message_fields(self, node: VisionAdapter) -> None:
        message = node._build_detections(np.array([100.5, 200.25]), 1_500_000_000)

        assert message.header.frame_id == "camera_link_optical"
        assert message.header.stamp.sec == 1
        assert message.header.stamp.nanosec == 500_000_000
        assert len(message.detections) == 1

        detection = message.detections[0]
        assert detection.header.frame_id == message.header.frame_id
        assert detection.bbox.center.position.x == pytest.approx(100.5)
        assert detection.bbox.center.position.y == pytest.approx(200.25)
        assert detection.bbox.size_x == pytest.approx(TRUTH_BBOX_SIZE_PX)
        assert detection.bbox.size_y == pytest.approx(TRUTH_BBOX_SIZE_PX)
        assert detection.bbox.size_x > 0.0
        assert detection.results[0].hypothesis.class_id == "drone"
        assert detection.results[0].hypothesis.score == pytest.approx(1.0)

    def test_position_message_covariance_is_symmetric(self, node: VisionAdapter) -> None:
        node._pixel_noise_px = 3.0
        node._target_plane_sigma_m = 0.1
        jacobian = np.array([[2.0, 0.5], [0.5, 3.0]])
        point = np.array([1.5, -2.5, 1.0])

        message = node._build_position_message(point, jacobian, 2_500_000_000)

        assert message.header.frame_id == "enu"
        assert message.header.stamp.sec == 2
        assert message.header.stamp.nanosec == 500_000_000
        assert message.pose.pose.position.x == pytest.approx(1.5)
        assert message.pose.pose.position.y == pytest.approx(-2.5)
        assert message.pose.pose.position.z == pytest.approx(1.0)
        assert message.pose.pose.orientation.w == pytest.approx(1.0)

        covariance = np.asarray(message.pose.covariance, dtype=float).reshape(6, 6)
        expected_xy = (3.0 ** 2) * (jacobian @ jacobian.T)
        assert covariance[0, 0] == pytest.approx(expected_xy[0, 0])
        assert covariance[0, 1] == pytest.approx(expected_xy[0, 1])
        assert covariance[1, 0] == pytest.approx(expected_xy[1, 0])
        assert covariance[1, 1] == pytest.approx(expected_xy[1, 1])
        assert covariance[2, 2] == pytest.approx(0.1 ** 2)
        assert covariance[3, 3] == pytest.approx(POSE_UNKNOWN_VARIANCE)
        assert covariance[4, 4] == pytest.approx(POSE_UNKNOWN_VARIANCE)
        assert covariance[5, 5] == pytest.approx(POSE_UNKNOWN_VARIANCE)
        np.testing.assert_allclose(covariance, covariance.T)

        # 除上述位置外，其余元素必须为零，不能用全零假装未知姿态“已知”。
        expected_mask = np.zeros((6, 6), dtype=bool)
        expected_mask[0, 0] = expected_mask[0, 1] = True
        expected_mask[1, 0] = expected_mask[1, 1] = True
        expected_mask[2, 2] = True
        expected_mask[3, 3] = expected_mask[4, 4] = expected_mask[5, 5] = True
        assert np.all(covariance[~expected_mask] == 0.0)


class TestRecording:
    def test_csv_records_valid_and_invalid_rows(self, tmp_path) -> None:
        adapter = VisionAdapter([
            Parameter("vision_source", value="truth"),
            Parameter("vision_record_output_dir", value=str(tmp_path)),
        ])
        try:
            # 第一行：没有相机内参，记录拒绝原因。
            adapter._on_timer()
            # 第二行：有效量测。
            prime_valid_inputs(adapter)
            adapter._on_timer()
            adapter.save_recording()
        finally:
            adapter.destroy_node()

        path = tmp_path / "vision_samples.csv"
        assert path.is_file()
        with path.open(newline="", encoding="utf-8") as file:
            reader = csv.DictReader(file)
            assert reader.fieldnames == list(CSV_FIELDS)
            rows = list(reader)

        assert len(rows) == 2
        assert rows[0]["valid"] == "0"
        assert rows[0]["invalid_reason"] == "no_camera_info"
        assert rows[1]["valid"] == "1"
        assert rows[1]["invalid_reason"] == ""
        assert float(rows[1]["position_roundtrip_error_m"]) < 1e-6
        assert float(rows[1]["target_x_est"]) == pytest.approx(3.0, abs=1e-6)
        assert float(rows[1]["target_plane_z"]) == pytest.approx(1.0)

    def test_recording_disabled_writes_nothing(self, tmp_path) -> None:
        adapter = VisionAdapter([
            Parameter("vision_source", value="truth"),
            Parameter("vision_record_data", value=False),
            Parameter("vision_record_output_dir", value=str(tmp_path)),
        ])
        try:
            prime_valid_inputs(adapter)
            adapter._on_timer()
            adapter.save_recording()
        finally:
            adapter.destroy_node()

        assert not (tmp_path / "vision_samples.csv").exists()
