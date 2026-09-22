"""相机几何、完整姿态转换与安装外参的离线单元测试。

只依赖标准库 `unittest` 和 numpy，不引入 pytest；通过 `sys.path` 直接引用
仓库内的 `pythonsimulation2d` 与 `gazebosimulation2d`，保持和 `main.py`
一致的开发期导入方式。

运行：

```bash
cd 7_2Dsimulation
uv run python tests/test_camera_geometry.py
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
    ned_to_enu_vector,
    rotation_body_from_mount_rpy,
    rotation_body_from_optical,
    rotation_enu_from_quaternion_ned,
    rotation_from_quaternion_ned,
    yaw_from_quaternion_ned,
)
from pythonsimulation2d.camera_geometry import (  # noqa: E402
    CameraIntrinsics,
    CameraPose,
    ground_to_pixel,
    pixel_in_image,
    pixel_to_ground,
    validate_camera_pose,
    validate_intrinsics,
)


# 光轴竖直向下的相机姿态：光学 x 指向东、y 指向南、z 指向下（公共 ENU）。
LEVEL_DOWN_ROTATION = np.array([
    [1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
    [0.0, 0.0, -1.0],
])

NOMINAL_MOUNT_RPY = (0.0, math.pi / 2.0, 0.0)
NOMINAL_MOUNT_XYZ = (0.0, 0.0, 0.10)


def default_intrinsics(fx: float = 539.94, fy: float = 539.94) -> CameraIntrinsics:
    return CameraIntrinsics(width=1280, height=960, fx=fx, fy=fy, cx=640.0, cy=480.0)


def level_down_pose(x: float = 0.0, y: float = 0.0, z: float = 8.0) -> CameraPose:
    return CameraPose(position_enu=np.array([x, y, z]), rotation_world_from_optical=LEVEL_DOWN_ROTATION.copy())


def quaternion_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton 乘积，输入/输出均为 PX4 的 `[w, x, y, z]`。"""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quaternion_from_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    half = angle / 2.0
    return np.concatenate(([math.cos(half)], axis * math.sin(half)))


class CoordinatesTest(unittest.TestCase):
    """`gazebosimulation2d.coordinates` 的完整姿态与安装链测试。"""

    def test_identity_attitude_gives_expected_flu_basis(self) -> None:
        rotation = rotation_enu_from_quaternion_ned(np.array([1.0, 0.0, 0.0, 0.0]))
        self.assertIsNotNone(rotation)
        # 零姿态下机体前方是 NED 北、左方是西、上方是上；ENU 中分别是 +y、-x、+z。
        expected = np.array([
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ])
        np.testing.assert_allclose(rotation, expected, atol=1e-12)

    def test_yaw_only_attitude_matches_ned_yaw_and_existing_api(self) -> None:
        yaw_ned = math.radians(90.0)
        quaternion = quaternion_from_axis_angle(np.array([0.0, 0.0, 1.0]), yaw_ned)
        rotation = rotation_enu_from_quaternion_ned(quaternion)
        self.assertIsNotNone(rotation)

        # 纯 NED yaw 下，FLU 三个基向量在 ENU 中应分别为
        # 前方 (sinψ, cosψ, 0)、左方 (-cosψ, sinψ, 0)、上方 (0, 0, 1)。
        expected = np.array([
            [math.sin(yaw_ned), -math.cos(yaw_ned), 0.0],
            [math.cos(yaw_ned), math.sin(yaw_ned), 0.0],
            [0.0, 0.0, 1.0],
        ])
        np.testing.assert_allclose(rotation, expected, atol=1e-12)

        # 与现有 yaw API 的 ENU yaw 约定保持一致（ENU yaw = pi/2 - NED yaw）。
        self.assertAlmostEqual(yaw_from_quaternion_ned(quaternion), math.pi / 2.0 - yaw_ned, places=12)

    def test_full_attitude_is_orthonormal_and_matches_rpy_chain(self) -> None:
        roll, pitch, yaw = 0.3, -0.2, 1.1
        quaternion = quaternion_multiply(
            quaternion_multiply(
                quaternion_from_axis_angle(np.array([0.0, 0.0, 1.0]), yaw),
                quaternion_from_axis_angle(np.array([0.0, 1.0, 0.0]), pitch),
            ),
            quaternion_from_axis_angle(np.array([1.0, 0.0, 0.0]), roll),
        )
        rotation = rotation_enu_from_quaternion_ned(quaternion)
        self.assertIsNotNone(rotation)

        np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-12)
        self.assertAlmostEqual(float(np.linalg.det(rotation)), 1.0, places=12)

        # 与“先 NED 轴交换、再 FRD→FLU 翻转、最后应用同一 rpy 链”的结果一致。
        rotation_ned = rotation_body_from_mount_rpy(roll, pitch, yaw)
        enu_from_ned = np.array([
            [0.0, 1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0],
        ])
        expected = enu_from_ned @ rotation_ned @ np.diag([1.0, -1.0, -1.0])
        np.testing.assert_allclose(rotation, expected, atol=1e-12)

    def test_invalid_quaternions_are_rejected(self) -> None:
        self.assertIsNone(rotation_from_quaternion_ned([1.0, 0.0, 0.0]))
        self.assertIsNone(rotation_from_quaternion_ned([math.nan, 0.0, 0.0, 0.0]))
        self.assertIsNone(rotation_from_quaternion_ned([0.0, 0.0, 0.0, 0.0]))
        self.assertIsNone(rotation_enu_from_quaternion_ned([1.0, math.inf, 0.0, 0.0]))

    def test_nominal_mount_matches_plan_matrix(self) -> None:
        rotation = rotation_body_from_optical(*NOMINAL_MOUNT_RPY)
        expected = np.array([
            [0.0, -1.0, 0.0],
            [-1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0],
        ])
        # 安装链结果 R_{B<-C} 是完整合成，不能再叠加一次 link->optical 转换。
        np.testing.assert_allclose(rotation, expected, atol=1e-12)

    def test_mount_rpy_uses_zyx_chain(self) -> None:
        roll, pitch, yaw = 0.25, -0.35, 0.45
        rotation = rotation_body_from_mount_rpy(roll, pitch, yaw)
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
        ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
        rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
        np.testing.assert_allclose(rotation, rz @ ry @ rx, atol=1e-12)

    def test_camera_pose_composition_with_nonzero_mount_translation(self) -> None:
        # NED yaw 90° 对应 R_{W<-B}=I：机体前方指向 ENU 东。
        quaternion = quaternion_from_axis_angle(np.array([0.0, 0.0, 1.0]), math.pi / 2.0)
        position_ned = np.array([10.0, 20.0, -8.0])
        mount_xyz = (0.05, -0.02, 0.10)

        result = camera_pose_from_odometry(position_ned, quaternion, mount_xyz, NOMINAL_MOUNT_RPY)
        self.assertIsNotNone(result)
        camera_position, camera_rotation = result
        np.testing.assert_allclose(camera_position, np.array([20.05, 9.98, 8.10]), atol=1e-12)
        np.testing.assert_allclose(camera_rotation, np.array([
            [0.0, -1.0, 0.0],
            [-1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0],
        ]), atol=1e-12)
        self.assertAlmostEqual(float(np.linalg.det(camera_rotation)), 1.0, places=12)

    def test_camera_pose_rejects_invalid_inputs(self) -> None:
        quaternion = quaternion_from_axis_angle(np.array([0.0, 0.0, 1.0]), 0.5)
        self.assertIsNone(camera_pose_from_odometry([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0],
                                                    NOMINAL_MOUNT_XYZ, NOMINAL_MOUNT_RPY))
        self.assertIsNone(camera_pose_from_odometry([math.nan, 0.0, 0.0], quaternion,
                                                    NOMINAL_MOUNT_XYZ, NOMINAL_MOUNT_RPY))
        self.assertIsNone(camera_pose_from_odometry([0.0, 0.0, 0.0], quaternion,
                                                    (0.0, 0.0), NOMINAL_MOUNT_RPY))
        self.assertIsNone(camera_pose_from_odometry([0.0, 0.0, 0.0], quaternion,
                                                    NOMINAL_MOUNT_XYZ, (0.0, math.nan, 0.0)))
        self.assertIsNone(camera_pose_from_odometry([0.0, 0.0, 0.0], quaternion,
                                                    NOMINAL_MOUNT_XYZ, (0.0, 0.0, 0.0, 0.0)))


class CameraGeometryTest(unittest.TestCase):
    """`pythonsimulation2d.camera_geometry` 的投影/反投影与雅可比测试。"""

    def test_level_down_roundtrip_error_below_tolerance(self) -> None:
        intrinsics = default_intrinsics()
        pose = level_down_pose()
        point = np.array([3.0, 4.0, 1.0])

        pixel = ground_to_pixel(point, pose, intrinsics)
        self.assertIsNotNone(pixel)

        result = pixel_to_ground(pixel, pose, intrinsics, target_plane_z=1.0)
        self.assertIsNotNone(result)
        estimated, jacobian = result
        self.assertLess(float(np.linalg.norm(estimated - point)), 1e-6)
        self.assertEqual(jacobian.shape, (2, 2))

    def test_level_down_roundtrip_with_offset_principal_and_anisotropic_focal(self) -> None:
        intrinsics = CameraIntrinsics(width=1280, height=960, fx=500.0, fy=520.0, cx=650.0, cy=470.0)
        pose = level_down_pose()
        point = np.array([2.0, -3.0, 1.0])

        pixel = ground_to_pixel(point, pose, intrinsics)
        self.assertIsNotNone(pixel)
        result = pixel_to_ground(pixel, pose, intrinsics, target_plane_z=1.0)
        self.assertIsNotNone(result)
        self.assertLess(float(np.linalg.norm(result[0] - point)), 1e-6)

    def test_roundtrip_with_mount_offset_and_tilted_attitude(self) -> None:
        intrinsics = default_intrinsics()
        quaternion = quaternion_multiply(
            quaternion_multiply(
                quaternion_from_axis_angle(np.array([0.0, 0.0, 1.0]), math.radians(30.0)),
                quaternion_from_axis_angle(np.array([0.0, 1.0, 0.0]), math.radians(10.0)),
            ),
            quaternion_from_axis_angle(np.array([1.0, 0.0, 0.0]), math.radians(5.0)),
        )
        result = camera_pose_from_odometry(
            [10.0, 20.0, -8.0],
            quaternion,
            NOMINAL_MOUNT_XYZ,
            NOMINAL_MOUNT_RPY,
        )
        self.assertIsNotNone(result)
        pose = CameraPose(position_enu=result[0], rotation_world_from_optical=result[1])
        point = np.array([21.0, 11.0, 1.0])

        pixel = ground_to_pixel(point, pose, intrinsics)
        self.assertIsNotNone(pixel)
        back = pixel_to_ground(pixel, pose, intrinsics, target_plane_z=1.0)
        self.assertIsNotNone(back)
        self.assertLess(float(np.linalg.norm(back[0] - point)), 1e-6)

    def test_roundtrip_on_nonzero_target_plane(self) -> None:
        intrinsics = default_intrinsics()
        pose = level_down_pose(z=8.0)
        point = np.array([-2.0, 1.5, 2.5])

        pixel = ground_to_pixel(point, pose, intrinsics)
        self.assertIsNotNone(pixel)
        back = pixel_to_ground(pixel, pose, intrinsics, target_plane_z=2.5)
        self.assertIsNotNone(back)
        self.assertLess(float(np.linalg.norm(back[0] - point)), 1e-6)
        self.assertAlmostEqual(float(back[0][2]), 2.5, places=12)

    def test_ground_to_pixel_rejects_behind_camera_and_out_of_view(self) -> None:
        intrinsics = default_intrinsics()
        pose = level_down_pose()

        # 相机上方（光学深度为负）不能成像。
        self.assertIsNone(ground_to_pixel(np.array([0.0, 0.0, 10.0]), pose, intrinsics))
        # 水平距离过远、超出图像边界。
        self.assertIsNone(ground_to_pixel(np.array([200.0, 0.0, 1.0]), pose, intrinsics))
        # 非有限输入。
        self.assertIsNone(ground_to_pixel(np.array([math.nan, 0.0, 1.0]), pose, intrinsics))

    def test_pixel_to_ground_rejects_upward_and_near_parallel_rays(self) -> None:
        intrinsics = default_intrinsics()

        upward = CameraPose(position_enu=np.array([0.0, 0.0, 8.0]),
                            rotation_world_from_optical=np.array([
                                [1.0, 0.0, 0.0],
                                [0.0, -1.0, 0.0],
                                [0.0, 0.0, 1.0],
                            ]))
        self.assertIsNone(pixel_to_ground(np.array([640.0, 480.0]), upward, intrinsics, 1.0))

        # 光轴低于水平面 4°，归一化 z 分量大于 -sin(5°)，视为近平行并拒绝。
        angle = math.radians(94.0)
        near_parallel = CameraPose(
            position_enu=np.array([0.0, 0.0, 8.0]),
            rotation_world_from_optical=np.array([
                [1.0, 0.0, 0.0],
                [0.0, math.cos(angle), -math.sin(angle)],
                [0.0, math.sin(angle), math.cos(angle)],
            ]),
        )
        self.assertIsNone(pixel_to_ground(np.array([640.0, 480.0]), near_parallel, intrinsics, 1.0))

    def test_pixel_to_ground_rejects_camera_below_plane(self) -> None:
        intrinsics = default_intrinsics()
        pose = level_down_pose(z=0.5)
        self.assertIsNone(pixel_to_ground(np.array([640.0, 480.0]), pose, intrinsics, target_plane_z=1.0))

    def test_validation_helpers_report_reasons(self) -> None:
        self.assertIsNone(validate_intrinsics(default_intrinsics()))
        self.assertIsNotNone(validate_intrinsics(CameraIntrinsics(1280, 960, 0.0, 500.0, 640.0, 480.0)))
        self.assertIsNotNone(validate_intrinsics(CameraIntrinsics(0, 960, 500.0, 500.0, 640.0, 480.0)))
        self.assertIsNotNone(validate_intrinsics(CameraIntrinsics(1280, 960, math.nan, 500.0, 640.0, 480.0)))

        self.assertIsNone(validate_camera_pose(level_down_pose()))
        # 非正交矩阵：轴被缩放。
        self.assertIsNotNone(validate_camera_pose(CameraPose(
            position_enu=np.array([0.0, 0.0, 8.0]),
            rotation_world_from_optical=2.0 * LEVEL_DOWN_ROTATION,
        )))
        # 行列式为 -1 的反射矩阵不是合法旋转。
        self.assertIsNotNone(validate_camera_pose(CameraPose(
            position_enu=np.array([0.0, 0.0, 8.0]),
            rotation_world_from_optical=np.diag([1.0, 1.0, -1.0]),
        )))
        self.assertIsNotNone(validate_camera_pose(CameraPose(
            position_enu=np.array([0.0, 0.0, math.inf]),
            rotation_world_from_optical=LEVEL_DOWN_ROTATION.copy(),
        )))

    def test_invalid_intrinsics_and_pose_are_rejected_by_projection(self) -> None:
        pose = level_down_pose()
        bad_intrinsics = CameraIntrinsics(1280, 960, -500.0, 500.0, 640.0, 480.0)
        self.assertIsNone(ground_to_pixel(np.array([0.0, 0.0, 1.0]), pose, bad_intrinsics))
        self.assertIsNone(pixel_to_ground(np.array([640.0, 480.0]), pose, bad_intrinsics, 1.0))

        bad_pose = CameraPose(position_enu=np.array([0.0, 0.0, 8.0]),
                              rotation_world_from_optical=2.0 * LEVEL_DOWN_ROTATION)
        self.assertIsNone(ground_to_pixel(np.array([0.0, 0.0, 1.0]), bad_pose, default_intrinsics()))
        self.assertIsNone(pixel_to_ground(np.array([640.0, 480.0]), bad_pose, default_intrinsics(), 1.0))

    def test_nonfinite_and_out_of_bounds_pixels_are_rejected(self) -> None:
        intrinsics = default_intrinsics()
        pose = level_down_pose()
        self.assertIsNone(pixel_to_ground(np.array([math.nan, 480.0]), pose, intrinsics, 1.0))
        self.assertIsNone(pixel_to_ground(np.array([-1.0, 480.0]), pose, intrinsics, 1.0))
        self.assertIsNone(pixel_to_ground(np.array([1280.0, 480.0]), pose, intrinsics, 1.0))
        self.assertIsNone(pixel_to_ground(np.array([640.0, 960.0]), pose, intrinsics, 1.0))
        self.assertIsNone(pixel_to_ground(np.array([640.0, 480.0]), pose, intrinsics, math.nan))

    def test_pixel_in_image_boundaries_are_inclusive(self) -> None:
        intrinsics = default_intrinsics()
        self.assertTrue(pixel_in_image(np.array([0.0, 0.0]), intrinsics))
        self.assertTrue(pixel_in_image(np.array([1279.0, 959.0]), intrinsics))
        self.assertFalse(pixel_in_image(np.array([1280.0, 959.0]), intrinsics))
        self.assertFalse(pixel_in_image(np.array([math.nan, 0.0]), intrinsics))
        self.assertFalse(pixel_in_image(np.array([0.0]), intrinsics))

    def test_jacobian_matches_central_difference(self) -> None:
        intrinsics = default_intrinsics()
        pose = level_down_pose()
        step = 1e-3

        for pixel in (np.array([740.0, 480.0]), np.array([640.0, 580.0]), np.array([520.0, 380.0])):
            result = pixel_to_ground(pixel, pose, intrinsics, target_plane_z=1.0)
            self.assertIsNotNone(result)
            _, jacobian = result

            numeric = np.zeros((2, 2), dtype=float)
            for axis in range(2):
                delta = np.zeros(2)
                delta[axis] = step
                plus = pixel_to_ground(pixel + delta, pose, intrinsics, 1.0)
                minus = pixel_to_ground(pixel - delta, pose, intrinsics, 1.0)
                self.assertIsNotNone(plus)
                self.assertIsNotNone(minus)
                numeric[:, axis] = (plus[0][:2] - minus[0][:2]) / (2.0 * step)

            np.testing.assert_allclose(jacobian, numeric, rtol=1e-6, atol=1e-9)

    def test_level_camera_jacobian_and_plan_sensitivity_example(self) -> None:
        fx = fy = 539.94
        intrinsics = default_intrinsics(fx=fx, fy=fy)
        pose = level_down_pose(z=8.0)

        result = pixel_to_ground(np.array([640.0, 480.0]), pose, intrinsics, target_plane_z=1.0)
        self.assertIsNotNone(result)
        height_above_plane = 7.0
        expected = np.array([[height_above_plane / fx, 0.0], [0.0, -height_above_plane / fy]])
        np.testing.assert_allclose(result[1], expected, rtol=1e-9)

        # 计划中的数量级核对：相机高于目标平面 7 m、焦距 540 px 时约 12.96 mm/px。
        self.assertAlmostEqual(height_above_plane / fx * 1000.0, 12.96, places=2)
        self.assertAlmostEqual(height_above_plane / fx * 5.0 * 100.0, 6.5, places=1)
        self.assertAlmostEqual(1280 * height_above_plane / fx, 16.6, places=1)
        self.assertAlmostEqual(960 * height_above_plane / fy, 12.4, places=1)
        self.assertAlmostEqual(0.35 / (height_above_plane / fx), 27.0, places=0)


if __name__ == "__main__":
    unittest.main()
