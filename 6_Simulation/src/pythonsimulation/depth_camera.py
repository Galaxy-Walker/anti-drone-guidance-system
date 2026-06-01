from __future__ import annotations

import numpy as np

from pythonsimulation.math_utils import EPS, rotation_world_from_yaw_pitch
from pythonsimulation.state import PursuerState, TargetState


def simulate_depth_camera_point(pursuer: PursuerState, target: TargetState) -> np.ndarray | None:
    # 理想深度相机输出目标中心在机体/相机系下的三维点。
    # 第一版假设相机=机头，body x 为光轴前向、body y 为左侧、body z 为上方。
    rotation_world_body = rotation_world_from_yaw_pitch(pursuer.yaw, pursuer.pitch)
    relative_world = target.position - pursuer.position
    point_body = rotation_world_body.T @ relative_world
    if point_body[0] <= EPS:
        return None
    return point_body


def camera_point_to_world(pursuer: PursuerState, point_body: np.ndarray) -> np.ndarray:
    # 用自机地面系位置和 yaw/pitch 姿态，把深度相机点恢复为地面系目标位置量测。
    rotation_world_body = rotation_world_from_yaw_pitch(pursuer.yaw, pursuer.pitch)
    return pursuer.position + rotation_world_body @ point_body
