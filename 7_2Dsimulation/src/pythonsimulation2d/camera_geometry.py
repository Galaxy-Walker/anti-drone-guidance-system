"""下视相机的纯几何投影与反投影工具。

本模块只处理“已经构造好的相机位姿”，不持有安装外参、不依赖 ROS，
供离线测试和 `vision_adapter` 共用同一套公式。

坐标系约定：

- W：约定公共原点的 ENU（x 东、y 北、z 上）。
- C：光学系（x 右、y 下、z 前），相机光轴为 +z_C。

`CameraPose.rotation_world_from_optical` 即 $R_{W\\leftarrow C}$，由 ROS 边界
（`gazebosimulation2d.coordinates`）合成机体姿态和安装外参后传入。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# 射线与目标平面近乎平行时，交点位置对像素噪声极度敏感：要求归一化射线方向
# 至少低于水平面 5°。用单位向量的 z 分量判断，避免阈值随射线长度变化。
MIN_DOWNWARD_SIN = float(np.sin(np.deg2rad(5.0)))

# 旋转矩阵必须正交且行列式为 +1，容差用于吸收四元数归一化的浮点误差。
_ROTATION_TOLERANCE = 1e-6


@dataclass(slots=True)
class CameraIntrinsics:
    """针孔相机内参，主点和焦距都使用像素单位。"""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(slots=True)
class CameraPose:
    """相机在公共 ENU 下的位姿；旋转的列向量是光学系的三个基向量。"""

    position_enu: np.ndarray
    rotation_world_from_optical: np.ndarray


def validate_intrinsics(intrinsics: CameraIntrinsics) -> str | None:
    """检查内参是否可用，返回 `None` 表示有效，否则返回中文原因。"""
    values = np.array([intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy], dtype=float)
    if not np.all(np.isfinite(values)):
        return "内参包含非有限值"
    if intrinsics.fx <= 0.0 or intrinsics.fy <= 0.0:
        return "焦距必须为正"
    if intrinsics.width <= 0 or intrinsics.height <= 0:
        return "图像尺寸无效"
    return None


def validate_camera_pose(pose: CameraPose) -> str | None:
    """检查相机位姿是否可用，返回 `None` 表示有效，否则返回中文原因。"""
    position = np.asarray(pose.position_enu, dtype=float)
    if position.shape != (3,) or not np.all(np.isfinite(position)):
        return "相机位置无效"

    rotation = np.asarray(pose.rotation_world_from_optical, dtype=float)
    if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
        return "相机旋转无效"

    orthogonality_error = float(np.max(np.abs(rotation.T @ rotation - np.eye(3))))
    if orthogonality_error > _ROTATION_TOLERANCE:
        return "相机旋转非正交"
    determinant = float(np.linalg.det(rotation))
    if determinant <= 0.0 or abs(determinant - 1.0) > _ROTATION_TOLERANCE:
        return "相机旋转行列式无效"
    return None


def pixel_in_image(pixel: np.ndarray, intrinsics: CameraIntrinsics) -> bool:
    """判断像素是否落在图像范围内，边界像素 `0` 和 `width-1` 视为有效。"""
    value = np.asarray(pixel, dtype=float)
    if value.shape != (2,) or not np.all(np.isfinite(value)):
        return False
    return 0.0 <= value[0] <= intrinsics.width - 1 and 0.0 <= value[1] <= intrinsics.height - 1


def ground_to_pixel(
    point_enu: np.ndarray,
    pose: CameraPose,
    intrinsics: CameraIntrinsics,
) -> np.ndarray | None:
    """把公共 ENU 下的三维点投影到像素。

    需要相机位姿/内参有效、点在光学深度前方且像素落在图像内，否则返回 `None`。
    """
    if validate_intrinsics(intrinsics) is not None:
        return None
    if validate_camera_pose(pose) is not None:
        return None

    point = np.asarray(point_enu, dtype=float)
    if point.shape != (3,) or not np.all(np.isfinite(point)):
        return None

    point_camera = pose.rotation_world_from_optical.T @ (point - pose.position_enu)
    depth = float(point_camera[2])
    # 光学深度非正说明点在相机后方或像平面内，不能成像。
    if depth <= 0.0 or not np.isfinite(depth):
        return None

    pixel = np.array(
        [
            intrinsics.fx * point_camera[0] / depth + intrinsics.cx,
            intrinsics.fy * point_camera[1] / depth + intrinsics.cy,
        ],
        dtype=float,
    )
    if not pixel_in_image(pixel, intrinsics):
        return None
    return pixel


def pixel_to_ground(
    pixel: np.ndarray,
    pose: CameraPose,
    intrinsics: CameraIntrinsics,
    target_plane_z: float,
) -> tuple[np.ndarray, np.ndarray] | None:
    """把像素反投影到高度为 `target_plane_z` 的水平面。

    返回三维交点和对像素的 2×2 XY 雅可比 $J=\\partial(x,y)/\\partial(u,v)$；
    相机低于平面、像素越界、射线朝上/近平行或数值非有限时返回 `None`。
    """
    if validate_intrinsics(intrinsics) is not None:
        return None
    if validate_camera_pose(pose) is not None:
        return None

    value = np.asarray(pixel, dtype=float)
    if value.shape != (2,) or not np.all(np.isfinite(value)):
        return None
    if not np.isfinite(target_plane_z):
        return None
    if not pixel_in_image(value, intrinsics):
        return None
    if pose.position_enu[2] <= target_plane_z:
        return None

    ray_camera = np.array(
        [
            (value[0] - intrinsics.cx) / intrinsics.fx,
            (value[1] - intrinsics.cy) / intrinsics.fy,
            1.0,
        ],
        dtype=float,
    )
    ray_world = pose.rotation_world_from_optical @ ray_camera
    if not np.all(np.isfinite(ray_world)):
        return None

    ray_length = float(np.linalg.norm(ray_world))
    if ray_length <= 0.0:
        return None
    if ray_world[2] / ray_length > -MIN_DOWNWARD_SIN:
        return None

    scale = (target_plane_z - pose.position_enu[2]) / ray_world[2]
    if scale <= 0.0 or not np.isfinite(scale):
        return None

    point = pose.position_enu + scale * ray_world
    if not np.all(np.isfinite(point)):
        return None

    jacobian = _pixel_jacobian(ray_world, scale, pose, intrinsics)
    if not np.all(np.isfinite(jacobian)):
        return None
    return point, jacobian


def _pixel_jacobian(
    ray_world: np.ndarray,
    scale: float,
    pose: CameraPose,
    intrinsics: CameraIntrinsics,
) -> np.ndarray:
    """按 $P=p+s\\,r_W$ 对像素求导，返回 2×2 XY 雅可比。"""
    rotation = pose.rotation_world_from_optical
    derivative_u = rotation[:, 0] / intrinsics.fx
    derivative_v = rotation[:, 1] / intrinsics.fy
    ratio = ray_world[:2] / ray_world[2]
    column_u = scale * (derivative_u[:2] - ratio * derivative_u[2])
    column_v = scale * (derivative_v[:2] - ratio * derivative_v[2])
    return np.column_stack((column_u, column_v))
