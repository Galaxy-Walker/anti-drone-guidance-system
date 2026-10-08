"""EMPC 画面保持（FOV）惩罚的离线单元测试。

验证 `guidance.py` 的 FOV 投影与视觉链路（`coordinates` + `camera_geometry`）
在标称下视安装下完全一致，以及惩罚的软边界/封顶/接线行为。

运行：

```bash
cd 7_2Dsimulation
uv run python tests/test_fov_penalty.py
```
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for candidate in (ROOT / "src", ROOT / "src" / "gazebosimulation2d"):
    if str(candidate) not in sys.path:
        sys.path.insert(0, str(candidate))

from gazebosimulation2d.coordinates import (  # noqa: E402
    camera_pose_from_odometry,
    enu_to_ned_vector,
    yaw_enu_to_ned,
)
from pythonsimulation2d.camera_geometry import CameraIntrinsics, CameraPose, ground_to_pixel  # noqa: E402
from pythonsimulation2d.config import GuidanceConfig, SimulationConfig  # noqa: E402
from pythonsimulation2d.guidance import (  # noqa: E402
    GuidanceMemory,
    _fov_normalized_offset,
    _fov_penalty,
    _rollout_cost,
)
from pythonsimulation2d.state import PursuerState, TargetState  # noqa: E402

NOMINAL_MOUNT_XYZ = (0.0, 0.0, 0.10)
NOMINAL_MOUNT_RPY = (0.0, math.pi / 2.0, 0.0)


def quaternion_from_ned_yaw(yaw_ned: float) -> np.ndarray:
    half = 0.5 * yaw_ned
    return np.array([math.cos(half), 0.0, 0.0, math.sin(half)])


def default_intrinsics() -> CameraIntrinsics:
    return CameraIntrinsics(width=1280, height=960, fx=539.936, fy=539.936, cx=640.0, cy=480.0)


class FovProjectionTest(unittest.TestCase):
    """`_fov_normalized_offset` 与视觉链路正投影的一致性。"""

    def test_matches_vision_chain_projection(self) -> None:
        guidance = GuidanceConfig()
        intrinsics = default_intrinsics()
        position = np.array([10.0, -5.0, 8.0])
        half_width = 0.5 * intrinsics.width
        half_height = 0.5 * intrinsics.height

        for yaw_enu in (0.0, 0.7, math.pi / 2.0, -2.0, 2.9):
            quaternion = quaternion_from_ned_yaw(yaw_enu_to_ned(yaw_enu))
            pose_result = camera_pose_from_odometry(
                enu_to_ned_vector(position),
                quaternion,
                NOMINAL_MOUNT_XYZ,
                NOMINAL_MOUNT_RPY,
            )
            self.assertIsNotNone(pose_result)
            pose = CameraPose(
                position_enu=pose_result[0],
                rotation_world_from_optical=pose_result[1],
            )
            for dx, dy in ((3.0, 0.0), (0.0, -4.0), (-2.0, 2.5), (5.0, -1.0)):
                target = np.array([position[0] + dx, position[1] + dy, guidance.fov_target_plane_z])
                pixel = ground_to_pixel(target, pose, intrinsics)
                self.assertIsNotNone(pixel)
                expected = np.array([
                    (pixel[0] - intrinsics.cx) / half_width,
                    (pixel[1] - intrinsics.cy) / half_height,
                ])
                actual = _fov_normalized_offset(position, yaw_enu, target, guidance)
                self.assertIsNotNone(actual)
                np.testing.assert_allclose(actual, expected, rtol=0.0, atol=1e-9)

    def test_target_below_camera_is_at_center(self) -> None:
        guidance = GuidanceConfig()
        position = np.array([12.0, 34.0, 8.0])
        target = np.array([12.0, 34.0, 1.0])
        for yaw_enu in (0.0, 1.3, -2.4):
            offset = _fov_normalized_offset(position, yaw_enu, target, guidance)
            self.assertIsNotNone(offset)
            np.testing.assert_allclose(offset, np.zeros(2), atol=1e-12)

    def test_rejects_camera_below_target_plane(self) -> None:
        guidance = GuidanceConfig()
        position = np.array([0.0, 0.0, 1.2])
        target = np.array([0.5, 0.0, guidance.fov_target_plane_z])
        self.assertIsNone(_fov_normalized_offset(position, 0.0, target, guidance))

    def test_rejects_nonfinite_inputs(self) -> None:
        guidance = GuidanceConfig()
        target = np.array([1.0, 0.0, 1.0])
        self.assertIsNone(_fov_normalized_offset(np.array([0.0, 0.0, math.nan]), 0.0, target, guidance))
        self.assertIsNone(_fov_normalized_offset(np.array([0.0, 0.0, 8.0]), 0.0,
                                                 np.array([math.inf, 0.0, 1.0]), guidance))
        self.assertIsNone(_fov_normalized_offset(np.array([0.0, 0.0, 8.0]), math.nan, target, guidance))


class FovPenaltyTest(unittest.TestCase):
    """软边界、矩形画幅与权重接线。"""

    def test_zero_inside_soft_margin(self) -> None:
        guidance = GuidanceConfig()
        position = np.array([0.0, 0.0, 8.0])
        target = np.array([2.0, 0.0, guidance.fov_target_plane_z])
        self.assertEqual(_fov_penalty(position, 0.0, target, guidance), 0.0)

    def test_increases_with_offset_until_cap(self) -> None:
        guidance = GuidanceConfig()
        position = np.array([0.0, 0.0, 8.0])
        penalties = [
            _fov_penalty(position, 0.0, np.array([distance, 0.0, guidance.fov_target_plane_z]), guidance)
            for distance in (3.5, 4.0, 4.5, 5.0)
        ]
        self.assertTrue(all(penalties[i] < penalties[i + 1] for i in range(len(penalties) - 1)))

        soft_margin = guidance.fov_soft_margin
        cap_penalty = ((guidance.fov_violation_cap - soft_margin) / (1.0 - soft_margin)) ** 2
        far_penalty = _fov_penalty(position, 0.0, np.array([20.0, 0.0, guidance.fov_target_plane_z]), guidance)
        self.assertAlmostEqual(far_penalty, cap_penalty, places=12)
        self.assertAlmostEqual(
            _fov_penalty(position, 0.0, np.array([30.0, 0.0, guidance.fov_target_plane_z]), guidance),
            cap_penalty,
            places=12,
        )

    def test_short_axis_is_penalized_more_than_wide_axis(self) -> None:
        # 目标机头前方（图像高轴）比侧向（宽轴）更早压边：同样的 5 m 水平距离，
        # 前方偏移应大于侧向偏移。
        guidance = GuidanceConfig()
        position = np.array([0.0, 0.0, 8.0])
        forward = _fov_penalty(position, 0.0, np.array([5.0, 0.0, guidance.fov_target_plane_z]), guidance)
        lateral = _fov_penalty(position, 0.0, np.array([0.0, 5.0, guidance.fov_target_plane_z]), guidance)
        self.assertGreater(forward, lateral)

    def test_rollout_cost_includes_fov_term(self) -> None:
        config = SimulationConfig()
        pursuer = PursuerState(
            position=np.array([0.0, 0.0, 8.0]),
            velocity=np.zeros(3),
            acceleration=np.zeros(3),
            yaw=0.0,
        )
        acceleration = np.array([1.0, 0.0, 0.0])
        pn_trend = np.array([0.5, 0.0, 0.0])

        far_target = TargetState(
            position=np.array([5.0, 0.0, 1.0]),
            velocity=np.zeros(3),
            acceleration=np.zeros(3),
        )
        config.guidance.nmpc_w_fov = 0.0
        cost_without = _rollout_cost(pursuer, far_target, acceleration, pn_trend, GuidanceMemory(), config)
        config.guidance.nmpc_w_fov = 80.0
        cost_with = _rollout_cost(pursuer, far_target, acceleration, pn_trend, GuidanceMemory(), config)
        self.assertGreater(cost_with, cost_without)

        # 目标在软边界内时 FOV 项不改变代价。
        near_target = TargetState(
            position=np.array([1.0, 0.0, 1.0]),
            velocity=np.zeros(3),
            acceleration=np.zeros(3),
        )
        config.guidance.nmpc_w_fov = 0.0
        near_without = _rollout_cost(pursuer, near_target, acceleration, pn_trend, GuidanceMemory(), config)
        config.guidance.nmpc_w_fov = 80.0
        near_with = _rollout_cost(pursuer, near_target, acceleration, pn_trend, GuidanceMemory(), config)
        self.assertAlmostEqual(near_with, near_without, places=12)


if __name__ == "__main__":
    unittest.main()
