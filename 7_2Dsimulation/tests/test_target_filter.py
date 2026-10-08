"""`pythonsimulation2d.target_filter` 的离线单元测试。

不依赖 ROS、Gazebo 或 YOLO，只构造量测序列验证 α-β 收敛性、马氏门控、
丢失状态机与时间跳变处理。

运行：

```bash
cd 7_2Dsimulation
uv run python tests/test_target_filter.py
```
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from pythonsimulation2d.target_filter import (  # noqa: E402
    STATE_COAST,
    STATE_LOST,
    STATE_TRACKING,
    TargetFilterConfig,
    VisionTargetTracker,
)


def linear_measurement(time_s: float, start: np.ndarray, velocity: np.ndarray) -> np.ndarray:
    return start + velocity * time_s


def circle_measurement(time_s: float, radius: float, omega: float) -> np.ndarray:
    return np.array([radius * math.cos(omega * time_s), radius * math.sin(omega * time_s)])


class TestInitialization(unittest.TestCase):
    def test_first_measurement_initializes_velocity_at_zero(self) -> None:
        tracker = VisionTargetTracker()
        self.assertTrue(tracker.update(1.0, np.array([3.0, 4.0])))
        estimate = tracker.predict(1.0)
        self.assertEqual(estimate.state, STATE_TRACKING)
        self.assertTrue(estimate.initialized)
        np.testing.assert_allclose(estimate.velocity[:2], [0.0, 0.0])
        self.assertEqual(estimate.measurements, 1)
        self.assertAlmostEqual(estimate.age_s, 0.0)
        self.assertAlmostEqual(float(estimate.position[2]), 1.0)

    def test_predict_without_measurement_is_lost_and_nan(self) -> None:
        tracker = VisionTargetTracker()
        estimate = tracker.predict(5.0)
        self.assertEqual(estimate.state, STATE_LOST)
        self.assertFalse(estimate.initialized)
        self.assertTrue(math.isnan(estimate.position[0]))
        self.assertTrue(math.isnan(estimate.velocity[1]))
        self.assertEqual(estimate.measurements, 0)
        self.assertEqual(estimate.age_s, math.inf)

    def test_invalid_measurement_is_rejected(self) -> None:
        tracker = VisionTargetTracker()
        self.assertFalse(tracker.update(0.0, np.array([math.nan, 0.0])))
        self.assertEqual(tracker.last_rejection, "invalid_measurement")
        self.assertFalse(tracker.update(float("nan"), np.array([0.0, 0.0])))
        self.assertEqual(tracker.last_rejection, "invalid_stamp")
        self.assertFalse(tracker.initialized)


class TestConvergence(unittest.TestCase):
    def setUp(self) -> None:
        self.start = np.array([10.0, -5.0])
        self.velocity = np.array([2.0, 1.0])
        self.dt = 0.1

    def test_constant_velocity_converges(self) -> None:
        tracker = VisionTargetTracker()
        for step in range(80):
            time_s = step * self.dt
            tracker.update(time_s, linear_measurement(time_s, self.start, self.velocity))
        estimate = tracker.predict(79 * self.dt)
        expected = linear_measurement(79 * self.dt, self.start, self.velocity)
        np.testing.assert_allclose(estimate.position[:2], expected, atol=0.02)
        np.testing.assert_allclose(estimate.velocity[:2], self.velocity, atol=0.05)
        self.assertEqual(estimate.state, STATE_TRACKING)
        self.assertEqual(estimate.measurements, 80)
        self.assertTrue(np.all(np.isfinite(estimate.position_covariance_xy)))

    def test_circle_tracking_steady_state_error(self) -> None:
        radius, omega = 12.0, 0.25
        tracker = VisionTargetTracker()
        errors = []
        for step in range(400):
            time_s = step * self.dt
            measurement = circle_measurement(time_s, radius, omega)
            tracker.update(time_s, measurement)
            # predict 只能在当前时刻取估计：回看历史时刻不会倒退内部状态。
            estimate = tracker.predict(time_s)
            if step >= 350:
                errors.append(float(np.linalg.norm(estimate.position[:2] - measurement)))
        # 圆目标机动约 0.75 m/s^2，α-β 的稳态滞后必须明显小于 1.5 m 捕获半径。
        self.assertLess(float(np.mean(errors)), 0.05)

    def test_acceleration_low_pass_tracks_constant_acceleration(self) -> None:
        tracker = VisionTargetTracker(TargetFilterConfig(accel_tau_s=0.4))
        acceleration = np.array([0.5, -0.3])
        for step in range(200):
            time_s = step * self.dt
            position = self.start + self.velocity * time_s + 0.5 * acceleration * time_s**2
            tracker.update(time_s, position)
        estimate = tracker.predict(199 * self.dt)
        np.testing.assert_allclose(estimate.acceleration[:2], acceleration, atol=0.05)

    def test_acceleration_disabled_outputs_zero(self) -> None:
        tracker = VisionTargetTracker(TargetFilterConfig(accel_tau_s=0.0))
        for step in range(20):
            time_s = step * self.dt
            tracker.update(time_s, linear_measurement(time_s, self.start, self.velocity))
        estimate = tracker.predict(19 * self.dt)
        np.testing.assert_allclose(estimate.acceleration, [0.0, 0.0, 0.0])


class TestGating(unittest.TestCase):
    def setUp(self) -> None:
        self.start = np.array([0.0, 0.0])
        self.velocity = np.array([1.0, 0.0])
        self.dt = 0.1
        self.tracker = VisionTargetTracker(TargetFilterConfig(gate_sigma=3.0))

    def _prime(self, steps: int = 40) -> None:
        for step in range(steps):
            time_s = step * self.dt
            self.tracker.update(time_s, linear_measurement(time_s, self.start, self.velocity))

    def test_outlier_is_rejected_and_tracking_continues(self) -> None:
        self._prime()
        outlier_time = 40 * self.dt
        self.assertFalse(self.tracker.update(outlier_time, np.array([50.0, -30.0])))
        self.assertEqual(self.tracker.last_rejection, "gate_rejected")
        estimate = self.tracker.predict(outlier_time)
        self.assertLess(float(np.linalg.norm(estimate.position[:2] - linear_measurement(outlier_time, self.start, self.velocity))), 0.05)

        # 拒绝不能锁死估计器：紧接着的正常量测必须被接受。
        self.assertTrue(self.tracker.update(41 * self.dt, linear_measurement(41 * self.dt, self.start, self.velocity)))
        self.assertEqual(self.tracker.measurements, 41)

    def test_gate_uses_measurement_covariance(self) -> None:
        self._prime()
        outlier_time = 40 * self.dt
        # 量测协方差极大时（sigma≈20 m），同样 55 m 的偏差在统计上不再显著，门控应接受。
        wide_covariance = np.diag([400.0, 400.0])
        self.assertTrue(self.tracker.update(outlier_time, np.array([50.0, -30.0]), wide_covariance))

    def test_gate_disabled_by_default(self) -> None:
        tracker = VisionTargetTracker()
        for step in range(10):
            time_s = step * self.dt
            tracker.update(time_s, linear_measurement(time_s, self.start, self.velocity))
        self.assertTrue(tracker.update(1.0, np.array([100.0, 100.0])))


class TestLossStateMachine(unittest.TestCase):
    def setUp(self) -> None:
        self.tracker = VisionTargetTracker(TargetFilterConfig(coast_s=0.3, loss_s=1.0))
        self.tracker.update(0.0, np.array([1.0, 2.0]))

    def test_thresholds_and_recovery(self) -> None:
        self.assertEqual(self.tracker.predict(0.29).state, STATE_TRACKING)
        self.assertEqual(self.tracker.predict(0.31).state, STATE_COAST)
        self.assertEqual(self.tracker.predict(1.01).state, STATE_LOST)
        self.assertTrue(self.tracker.update(1.02, np.array([1.5, 2.0])))
        self.assertEqual(self.tracker.predict(1.02).state, STATE_TRACKING)
        self.assertEqual(self.tracker.measurements, 2)

    def test_age_is_measured_from_last_accepted_measurement(self) -> None:
        self.tracker.update(0.5, np.array([1.1, 2.0]))
        estimate = self.tracker.predict(1.0)
        self.assertAlmostEqual(estimate.age_s, 0.5)
        self.assertEqual(estimate.state, STATE_COAST)

    def test_predict_before_last_time_does_not_move_state(self) -> None:
        before = self.tracker.predict(0.0)
        after = self.tracker.predict(-5.0)
        np.testing.assert_allclose(before.position, after.position)
        self.assertEqual(after.state, STATE_TRACKING)

    def test_reset_clears_measurements(self) -> None:
        self.tracker.reset()
        self.assertFalse(self.tracker.initialized)
        self.assertEqual(self.tracker.measurements, 0)
        self.assertEqual(self.tracker.predict(10.0).state, STATE_LOST)


class TestTimeHandling(unittest.TestCase):
    def test_duplicate_and_backward_stamps_are_ignored(self) -> None:
        tracker = VisionTargetTracker()
        self.assertTrue(tracker.update(1.0, np.array([0.0, 0.0])))
        self.assertFalse(tracker.update(1.0, np.array([5.0, 5.0])))
        self.assertEqual(tracker.last_rejection, "duplicate_or_backward_stamp")
        self.assertFalse(tracker.update(0.5, np.array([5.0, 5.0])))
        self.assertEqual(tracker.measurements, 1)
        np.testing.assert_allclose(tracker.position_xy, [0.0, 0.0])

    def test_long_gap_propagation_is_clamped_to_max_dt(self) -> None:
        config = TargetFilterConfig(max_dt_s=0.5, min_dt_s=0.01)
        dense = VisionTargetTracker(config)
        sparse = VisionTargetTracker(config)
        for tracker in (dense, sparse):
            tracker.update(0.0, np.array([0.0, 0.0]))
            tracker.update(0.1, np.array([0.4, 0.0]))  # 建立约 3 m/s 的正向速度
        # dense 用 0.5 s 空窗（正好等于 max_dt_s），sparse 用 10 s 空窗；
        # 若 10 s 空窗按真实 dt 外推，sparse 的预测量会多漂移数十米。
        measurement = np.array([1.0, 0.0])
        dense.update(0.6, measurement)
        sparse.update(10.1, measurement)
        np.testing.assert_allclose(sparse.position_xy, dense.position_xy, atol=1e-9)
        np.testing.assert_allclose(sparse.velocity_xy, dense.velocity_xy, atol=1e-9)

    def test_min_dt_prevents_velocity_spike(self) -> None:
        tracker = VisionTargetTracker(TargetFilterConfig(min_dt_s=0.05))
        tracker.update(0.0, np.array([0.0, 0.0]))
        # 两个几乎同刻的量测：若不夹 min_dt_s，beta/dt 会把速度放大到 20 倍。
        tracker.update(0.001, np.array([0.1, 0.0]))
        self.assertLessEqual(abs(float(tracker.velocity_xy[0])), 0.25 / 0.05 * 0.1 + 1e-9)


if __name__ == "__main__":
    unittest.main(verbosity=2)
