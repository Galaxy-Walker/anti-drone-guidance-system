"""`guidance_node_2d` 视觉闭环接线的 ROS 环境单元测试。

不启动 Gazebo/PX4：直接构造 odometry/视觉量测消息驱动节点内部方法，覆盖
参数校验、α-β 接线与 hold、`target_source=odometry` 回归、记录列与 PX4 时间戳。

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
import math
import sys
import time
from pathlib import Path

import numpy as np
import pytest
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.parameter import Parameter

# `colcon test` 下没有额外的 PYTHONPATH：和 tests/test_camera_geometry.py 一样，
# 直接按源码树相对位置把 pythonsimulation2d 加进导入路径。
ROOT = Path(__file__).resolve().parents[3]
for candidate in (ROOT / "src", ROOT / "src" / "gazebosimulation2d"):
    if str(candidate) not in sys.path:
        sys.path.insert(0, str(candidate))

from pythonsimulation2d.config import GuidanceConfig
from pythonsimulation2d.state import PursuerState, TargetState
from pythonsimulation2d.target_filter import STATE_COAST, STATE_LOST, STATE_TRACKING

from gazebosimulation2d.guidance_node import GuidanceNode, _VisionSnapshot
from gazebosimulation2d.px4_utils import timestamp_us


class PublisherCapture:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


def make_node(**overrides) -> GuidanceNode:
    parameters = {"record_data": True, "sim_time": 40.0, "algorithm": "pn"}
    parameters.update(overrides)
    node = GuidanceNode([Parameter(name, value=value) for name, value in parameters.items()])
    # 测试直接调用内部方法，不经过定时器；真实发布器替换成可断言的消息收集器。
    node._sim_clock_guard = None
    node._pursuer_setpoint_pub = PublisherCapture()
    node._target_setpoint_pub = PublisherCapture()
    node._pursuer_offboard_pub = PublisherCapture()
    node._target_offboard_pub = PublisherCapture()
    return node


def set_vehicle_states(
    node: GuidanceNode,
    pursuer_position=(0.0, 0.0, 8.0),
    target_position=(10.0, 0.0, 1.0),
) -> None:
    node._pursuer = PursuerState(
        position=np.array(pursuer_position, dtype=float),
        velocity=np.zeros(3),
        acceleration=np.zeros(3),
        yaw=0.0,
    )
    node._target = TargetState(
        position=np.array(target_position, dtype=float),
        velocity=np.zeros(3),
        acceleration=np.zeros(3),
    )


def make_vision_message(stamp_s: float, x: float, y: float, covariance_xy=None) -> PoseWithCovarianceStamped:
    message = PoseWithCovarianceStamped()
    message.header.stamp.sec = int(stamp_s)
    message.header.stamp.nanosec = int(round((stamp_s - int(stamp_s)) * 1e9))
    message.header.frame_id = "enu"
    message.pose.pose.position.x = float(x)
    message.pose.pose.position.y = float(y)
    message.pose.pose.position.z = 1.0
    message.pose.pose.orientation.w = 1.0
    if covariance_xy is not None:
        covariance = np.zeros((6, 6), dtype=float)
        covariance[0:2, 0:2] = covariance_xy
        message.pose.covariance = covariance.reshape(-1).tolist()
    return message


class TestParameters:
    def test_odometry_source_is_default(self) -> None:
        node = make_node()
        try:
            assert node._target_source == "odometry"
            assert node._vision_tracker is None
            assert node._vision_fallback == "none"
        finally:
            node.destroy_node()

    def test_invalid_target_source_is_rejected(self) -> None:
        with pytest.raises(ValueError):
            make_node(target_source="radar")

    def test_odometry_fallback_is_not_implemented(self) -> None:
        with pytest.raises(ValueError):
            make_node(target_source="vision", vision_fallback="odometry")

    def test_unstable_beta_is_rejected(self) -> None:
        with pytest.raises(ValueError):
            make_node(target_source="vision", vision_alpha=0.9, vision_beta=3.0)

    def test_loss_must_exceed_coast(self) -> None:
        with pytest.raises(ValueError):
            make_node(target_source="vision", vision_coast_s=1.0, vision_loss_s=0.5)

    def test_dt_range_is_checked(self) -> None:
        with pytest.raises(ValueError):
            make_node(target_source="vision", min_dt_s=0.5, max_dt_s=0.1)

    def test_target_speed_scale_scales_circle_omega(self) -> None:
        node = make_node(scenario="circle", target_speed_scale=0.5)
        try:
            # 半径与起点不变，只把角速度（即线速度）减半。
            assert node._config.target.circle_omega == pytest.approx(0.125)
            assert node._config.target.circle_radius == pytest.approx(12.0)
        finally:
            node.destroy_node()

    def test_target_speed_scale_scales_linear_velocity(self) -> None:
        node = make_node(scenario="linear", target_speed_scale=2.0)
        try:
            assert node._config.target.linear_velocity[0] == pytest.approx(4.0)
            assert node._config.target.linear_velocity[1] == pytest.approx(2.0)
        finally:
            node.destroy_node()

    def test_non_positive_target_speed_scale_is_rejected(self) -> None:
        with pytest.raises(ValueError):
            make_node(target_speed_scale=0.0)

    def test_pid_parameters_default_to_offline_config(self) -> None:
        node = make_node()
        try:
            defaults = GuidanceConfig()
            assert node._config.guidance.pid_kp == pytest.approx(defaults.pid_kp)
            assert node._config.guidance.pid_ki == pytest.approx(defaults.pid_ki)
            assert node._config.guidance.pid_kd == pytest.approx(defaults.pid_kd)
            assert node._config.guidance.pid_integral_limit == pytest.approx(defaults.pid_integral_limit)
        finally:
            node.destroy_node()

    def test_pid_parameters_override_guidance_defaults(self) -> None:
        node = make_node(algorithm="pid", pid_kp=3.0, pid_ki=0.2, pid_kd=4.0, pid_integral_limit=1.5)
        try:
            assert node._config.guidance.pid_kp == pytest.approx(3.0)
            assert node._config.guidance.pid_ki == pytest.approx(0.2)
            assert node._config.guidance.pid_kd == pytest.approx(4.0)
            assert node._config.guidance.pid_integral_limit == pytest.approx(1.5)
        finally:
            node.destroy_node()

    @pytest.mark.parametrize("name", ["pid_kp", "pid_ki", "pid_kd", "pid_integral_limit"])
    @pytest.mark.parametrize("value", [-0.5, math.nan, math.inf])
    def test_invalid_pid_parameter_is_rejected(self, name, value) -> None:
        with pytest.raises(ValueError):
            make_node(**{name: value})


class TestVisionTargetState:
    def make_vision_node(self, monkeypatch, **overrides):
        node = make_node(target_source="vision", **overrides)
        clock = {"s": 0.0}
        monkeypatch.setattr(node, "_clock_now_s", lambda: clock["s"])
        return node, clock

    def test_no_measurement_returns_none_and_lost(self, monkeypatch) -> None:
        node, clock = self.make_vision_node(monkeypatch)
        try:
            clock["s"] = 5.0
            target_state, snapshot = node._vision_target_state()
            assert target_state is None
            assert snapshot.state == STATE_LOST
            assert not snapshot.valid
            assert snapshot.measurements == 0
        finally:
            node.destroy_node()

    def test_measurements_are_filtered_and_used(self, monkeypatch) -> None:
        node, clock = self.make_vision_node(monkeypatch)
        try:
            for step in range(10):
                clock["s"] = 1.0 + 0.1 * step
                node._vision_pose_callback(make_vision_message(clock["s"], 3.0 + 0.2 * step, 4.0))
                target_state, snapshot = node._vision_target_state()
            assert target_state is not None
            assert snapshot.valid
            assert snapshot.state == STATE_TRACKING
            assert snapshot.measurements == 10
            assert target_state.position[0] == pytest.approx(3.0 + 0.2 * 9, abs=0.05)
            assert target_state.position[1] == pytest.approx(4.0, abs=0.05)
            assert target_state.position[2] == pytest.approx(1.0)
            # 速度估计应接近 2 m/s 的东向速度（ENU x）。
            assert target_state.velocity[0] == pytest.approx(2.0, abs=0.3)
        finally:
            node.destroy_node()

    def test_duplicate_stamp_is_not_counted_twice(self, monkeypatch) -> None:
        node, clock = self.make_vision_node(monkeypatch)
        try:
            clock["s"] = 1.0
            node._vision_pose_callback(make_vision_message(1.0, 3.0, 4.0))
            node._vision_target_state()
            node._vision_pose_callback(make_vision_message(1.0, 3.0, 4.0))
            node._vision_target_state()
            assert node._vision_measurements == 1
            assert node._vision_received == 2
        finally:
            node.destroy_node()

    def test_stale_measurement_is_dropped(self, monkeypatch) -> None:
        node, clock = self.make_vision_node(monkeypatch, vision_max_age_s=0.2)
        try:
            clock["s"] = 1.0
            node._vision_pose_callback(make_vision_message(1.0, 3.0, 4.0))
            node._vision_target_state()
            # 量测延迟 0.5 s 超过 vision_max_age_s：不得进入滤波器。
            clock["s"] = 2.0
            node._vision_pose_callback(make_vision_message(1.5, 30.0, 40.0))
            target_state, snapshot = node._vision_target_state()
            assert node._vision_stale_drops == 1
            assert node._vision_measurements == 1
            assert snapshot.latency_s == pytest.approx(0.5)
            assert target_state.position[0] == pytest.approx(3.0, abs=0.1)
        finally:
            node.destroy_node()

    def test_lost_triggers_hold_by_default(self, monkeypatch) -> None:
        node, clock = self.make_vision_node(monkeypatch, vision_coast_s=0.3, vision_loss_s=1.0)
        try:
            clock["s"] = 1.0
            node._vision_pose_callback(make_vision_message(1.0, 3.0, 4.0))
            node._vision_target_state()
            clock["s"] = 1.2
            _, snapshot = node._vision_target_state()
            assert snapshot.state != STATE_LOST
            clock["s"] = 2.5
            target_state, snapshot = node._vision_target_state()
            assert snapshot.state == STATE_LOST
            assert target_state is None
        finally:
            node.destroy_node()

    def test_hold_can_be_disabled(self, monkeypatch) -> None:
        node, clock = self.make_vision_node(
            monkeypatch, vision_hold_on_loss=False, vision_coast_s=0.3, vision_loss_s=1.0
        )
        try:
            clock["s"] = 1.0
            node._vision_pose_callback(make_vision_message(1.0, 3.0, 4.0))
            node._vision_target_state()
            clock["s"] = 2.5
            target_state, snapshot = node._vision_target_state()
            assert snapshot.state == STATE_LOST
            assert target_state is not None
        finally:
            node.destroy_node()

    def test_covariance_block_is_passed_to_gate(self, monkeypatch) -> None:
        node, clock = self.make_vision_node(monkeypatch, vision_gate_sigma=3.0)
        try:
            for step in range(20):
                clock["s"] = 1.0 + 0.1 * step
                node._vision_pose_callback(
                    make_vision_message(clock["s"], 3.0, 4.0, covariance_xy=np.diag([0.01, 0.01]))
                )
                node._vision_target_state()
            accepted_before = node._vision_measurements
            clock["s"] += 0.1
            node._vision_pose_callback(
                make_vision_message(clock["s"], 50.0, 4.0, covariance_xy=np.diag([0.01, 0.01]))
            )
            _, snapshot = node._vision_target_state()
            assert node._vision_measurements == accepted_before
            assert node._vision_filter_rejects == 1
            assert snapshot.valid
        finally:
            node.destroy_node()


class TestInitialAcquisition:
    """视觉模式准备阶段把追踪机送到场景起点上方，保证第一帧量测可见。"""

    @pytest.mark.parametrize(
        ("scenario", "start_xy"),
        [
            ("circle", (47.0, 0.0)),
            ("stationary", (40.0, 20.0)),
            ("linear", (25.0, -20.0)),
        ],
    )
    def test_vision_mode_targets_scenario_start(self, scenario, start_xy) -> None:
        node = make_node(target_source="vision", scenario=scenario)
        try:
            # 追踪机 spawn 在原点、目标机停在场景起点：起飞点应改为起点上方。
            set_vehicle_states(
                node,
                pursuer_position=(0.0, 0.0, 8.0),
                target_position=(start_xy[0], start_xy[1], 1.0),
            )
            node._ensure_pursuer_takeoff_position()
            assert node._pursuer_takeoff_position[0] == pytest.approx(start_xy[0])
            assert node._pursuer_takeoff_position[1] == pytest.approx(start_xy[1])
            assert node._pursuer_takeoff_position[2] == pytest.approx(node._pursuer_fixed_altitude)
        finally:
            node.destroy_node()

    def test_vision_takeoff_setpoint_targets_scenario_start(self) -> None:
        node = make_node(target_source="vision", scenario="circle")
        try:
            set_vehicle_states(node)
            node._prepare_vehicles_for_tracking(
                timestamp=0,
                target_start=node._target_start_reference(),
            )
            setpoint = node._pursuer_setpoint_pub.messages[0]
            # ENU (47, 0, 8) -> NED (0, 47, -8)。
            assert setpoint.position[0] == pytest.approx(0.0)
            assert setpoint.position[1] == pytest.approx(47.0)
            assert setpoint.position[2] == pytest.approx(-node._pursuer_fixed_altitude)
        finally:
            node.destroy_node()

    def test_odometry_mode_keeps_spawn_position(self) -> None:
        node = make_node()
        try:
            set_vehicle_states(node, pursuer_position=(3.0, -4.0, 8.2))
            node._ensure_pursuer_takeoff_position()
            # odometry 模式保持原行为：起飞点 = 当前 XY + 固定高度。
            assert node._pursuer_takeoff_position[0] == pytest.approx(3.0)
            assert node._pursuer_takeoff_position[1] == pytest.approx(-4.0)
            assert node._pursuer_takeoff_position[2] == pytest.approx(node._pursuer_fixed_altitude)
        finally:
            node.destroy_node()

    def test_takeoff_position_is_locked_once(self) -> None:
        node = make_node(target_source="vision")
        try:
            set_vehicle_states(node, pursuer_position=(0.0, 0.0, 8.0))
            node._ensure_pursuer_takeoff_position()
            locked = node._pursuer_takeoff_position.copy()
            node._pursuer.position = np.array([1.0, 2.0, 8.0])
            node._ensure_pursuer_takeoff_position()
            assert np.allclose(node._pursuer_takeoff_position, locked)
        finally:
            node.destroy_node()


class TestGuidanceWiring:
    @pytest.mark.parametrize("algorithm", ["pn", "pid"])
    def test_vision_source_drives_setpoint_from_estimate(self, monkeypatch, algorithm) -> None:
        node = make_node(target_source="vision", algorithm=algorithm)
        clock = {"s": 0.0}
        monkeypatch.setattr(node, "_clock_now_s", lambda: clock["s"])
        try:
            set_vehicle_states(node)
            for step in range(10):
                clock["s"] = 1.0 + 0.1 * step
                node._vision_pose_callback(make_vision_message(clock["s"], 3.0 + 0.2 * step, 4.0))
            node._run_tracking_cycle(timestamp=0)

            assert len(node._pursuer_setpoint_pub.messages) == 1
            setpoint = node._pursuer_setpoint_pub.messages[0]
            # 估计目标在东北方向；odometry 真值只在 x 方向。NED 速度两个分量都应显著为正。
            assert setpoint.velocity[0] > 0.1
            assert setpoint.velocity[1] > 0.1
            sample = node._record_samples[-1]
            assert sample.target_source == "vision"
            assert sample.vision_valid == 1.0
            assert sample.target_est_x == pytest.approx(4.8, abs=0.2)
            assert sample.target_est_y == pytest.approx(4.0, abs=0.2)
            # target_x 仍是 odometry 真值，估计值与真值分离记录。
            assert sample.target_position[0] == pytest.approx(10.0)
            assert sample.vision_error_xy > 1.0
        finally:
            node.destroy_node()

    @pytest.mark.parametrize("algorithm", ["pn", "pid"])
    def test_odometry_regression_keeps_old_behavior(self, algorithm) -> None:
        node = make_node(algorithm=algorithm)
        try:
            set_vehicle_states(node)
            node._run_tracking_cycle(timestamp=0)
            assert len(node._pursuer_setpoint_pub.messages) == 1
            setpoint = node._pursuer_setpoint_pub.messages[0]
            # 目标真值在 +x（东）：ENU 速度应为 +x，对应 NED (n, e) = (0, +v)。
            assert setpoint.velocity[1] > 0.1
            sample = node._record_samples[-1]
            assert sample.target_source == "odometry"
            assert math.isnan(sample.vision_valid)
            assert math.isnan(sample.target_est_x)
            assert math.isnan(sample.vision_error_xy)
        finally:
            node.destroy_node()

    def test_hold_publishes_zero_velocity_and_keeps_yaw(self) -> None:
        node = make_node(target_source="vision")
        try:
            set_vehicle_states(node)
            node._pursuer.yaw = 0.7
            snapshot = _VisionSnapshot()
            node._run_hold_cycle(timestamp=123, elapsed=2.0, snapshot=snapshot)

            assert len(node._pursuer_setpoint_pub.messages) == 1
            setpoint = node._pursuer_setpoint_pub.messages[0]
            assert np.allclose(setpoint.velocity, [0.0, 0.0, 0.0])
            assert np.allclose(setpoint.acceleration, [0.0, 0.0, 0.0])
            # 保持 yaw：ENU 0.7 rad -> NED yaw = pi/2 - 0.7。
            assert setpoint.yaw == pytest.approx(math.pi / 2.0 - 0.7, abs=1e-9)
            assert node._record_samples[-1].vision_valid == 0.0
        finally:
            node.destroy_node()


class TestPidLifecycle:
    def test_coast_integrates_hold_clears_and_detection_resumes(self, monkeypatch) -> None:
        node = make_node(algorithm="pid", target_source="vision")
        clock = {"s": 1.0}
        monkeypatch.setattr(node, "_clock_now_s", lambda: clock["s"])
        try:
            # 真值与视觉方向相反，确保漏检时不会切回目标 odometry。
            set_vehicle_states(node, target_position=(-10.0, 0.0, 1.0))
            node._vision_pose_callback(make_vision_message(1.0, 1.0, 0.0))
            node._run_tracking_cycle(timestamp=123)
            np.testing.assert_allclose(node._memory.pid_integral, [0.05, 0.0, 0.0])
            setpoint = node._pursuer_setpoint_pub.messages[-1]
            np.testing.assert_allclose(setpoint.acceleration, [0.0, 2.505, 0.0], atol=1e-6)
            np.testing.assert_allclose(setpoint.velocity, [0.0, 2.505 * 0.05, 0.0], atol=1e-6)
            assert node._pursuer_offboard_pub.messages[-1].velocity
            assert node._pursuer_offboard_pub.messages[-1].acceleration

            clock["s"] = 1.4
            node._run_tracking_cycle(timestamp=124)
            assert node._vision_tracker.predict(clock["s"]).state == STATE_COAST
            np.testing.assert_allclose(node._memory.pid_integral, [0.1, 0.0, 0.0])
            assert node._pursuer_setpoint_pub.messages[-1].acceleration[1] > 0.0
            assert node._vision_measurements == 1

            clock["s"] = 2.2
            node._run_tracking_cycle(timestamp=125)
            np.testing.assert_array_equal(node._memory.pid_integral, np.zeros(3))
            np.testing.assert_array_equal(node._pursuer_setpoint_pub.messages[-1].velocity, np.zeros(3))
            np.testing.assert_array_equal(node._pursuer_setpoint_pub.messages[-1].acceleration, np.zeros(3))
            # 悬停多个周期也不累积积分。
            clock["s"] = 2.25
            node._run_tracking_cycle(timestamp=126)
            np.testing.assert_array_equal(node._memory.pid_integral, np.zeros(3))

            clock["s"] = 2.3
            node._vision_pose_callback(make_vision_message(2.3, -1.0, 0.0))
            node._run_tracking_cycle(timestamp=127)
            sample = node._record_samples[-1]
            assert sample.vision_valid == 1.0
            assert node._vision_measurements == 2
            assert node._memory.pid_integral[0] == pytest.approx(sample.target_est_x * node._config.dt)
            assert node._memory.pid_integral[0] < 0.0
            assert node._pursuer_setpoint_pub.messages[-1].acceleration[1] < 0.0
        finally:
            node.destroy_node()

    def test_derivative_uses_filtered_target_velocity(self, monkeypatch) -> None:
        node = make_node(algorithm="pid", target_source="vision", pid_kp=0.0, pid_ki=0.0)
        clock = {"s": 1.0}
        monkeypatch.setattr(node, "_clock_now_s", lambda: clock["s"])
        try:
            set_vehicle_states(node)
            node._target.velocity[0] = -5.0
            for step in range(10):
                clock["s"] = 1.0 + step * 0.1
                node._vision_pose_callback(make_vision_message(clock["s"], step * 0.2, 0.0))
                node._run_tracking_cycle(timestamp=step)
            sample = node._record_samples[-1]
            assert sample.target_est_vx == pytest.approx(2.0, abs=0.3)
            assert node._pursuer_setpoint_pub.messages[-1].acceleration[1] == pytest.approx(
                2.6 * sample.target_est_vx, abs=1e-6
            )
        finally:
            node.destroy_node()

    def test_start_tracking_clears_integral(self) -> None:
        node = make_node(algorithm="pid")
        try:
            node._memory.pid_integral[:] = [1.0, -2.0, 0.0]
            node._start_tracking()
            np.testing.assert_array_equal(node._memory.pid_integral, np.zeros(3))
        finally:
            node.destroy_node()


class TestRecording:
    @pytest.mark.parametrize("algorithm", ["pn", "pid"])
    def test_csv_contains_vision_columns(self, tmp_path, algorithm) -> None:
        node = make_node(
            algorithm=algorithm, target_source="vision", record_data=True, record_output_dir=str(tmp_path)
        )
        clock = {"s": 0.0}
        node._clock_now_s = lambda: clock["s"]
        try:
            set_vehicle_states(node)
            clock["s"] = 1.0
            node._vision_pose_callback(make_vision_message(1.0, 3.0, 4.0))
            node._run_tracking_cycle(timestamp=0)
            node.save_recording()
        finally:
            node.destroy_node()

        path = tmp_path / "circle" / algorithm / "gazebo_samples.csv"
        with path.open(newline="", encoding="utf-8") as file:
            reader = csv.DictReader(file)
            fieldnames = reader.fieldnames
            row = next(reader)
        for field in (
            "guidance_target_source",
            "vision_valid",
            "vision_age_s",
            "vision_latency_s",
            "vision_measurements",
            "target_est_x",
            "target_est_y",
            "target_est_vx",
            "target_est_vy",
            "target_est_ax",
            "target_est_ay",
            "vision_error_xy",
        ):
            assert field in fieldnames
        assert row["guidance_target_source"] == "vision"
        assert float(row["vision_valid"]) == 1.0
        assert float(row["target_x"]) == pytest.approx(10.0)
        assert float(row["target_est_x"]) == pytest.approx(3.0, abs=0.1)


class TestPx4Timestamp:
    def test_timestamp_uses_host_wall_clock(self) -> None:
        before = time.time_ns() // 1000
        value = timestamp_us()
        after = time.time_ns() // 1000
        assert before <= value <= after
