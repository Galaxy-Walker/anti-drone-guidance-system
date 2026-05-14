from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import foxglove
from foxglove.channels import SceneUpdateChannel
from foxglove.messages import (
    Color,
    LinePrimitive,
    LinePrimitiveLineType,
    Point3,
    Pose,
    Quaternion,
    SceneEntity,
    SceneUpdate,
    SpherePrimitive,
    TextPrimitive,
    Timestamp,
    Vector3,
)

from pythonsimulation.config import ALGORITHM_LABELS, SimulationConfig
from pythonsimulation.math_utils import forward_from_yaw_pitch
from pythonsimulation.metrics import yaw_rate_command
from pythonsimulation.state import SimulationResult


ALGORITHM_COLORS = {
    "basic": (0.45, 0.45, 0.45, 1.0),
    "pn": (0.12, 0.38, 0.95, 1.0),
    "pn_fov": (1.0, 0.55, 0.12, 1.0),
    "pn_fov_cbf": (0.15, 0.65, 0.22, 1.0),
    "pn_fov_mppi": (0.9, 0.18, 0.18, 1.0),
    "pn_fov_nmpc": (0.58, 0.25, 0.85, 1.0),
}
TARGET_COLOR = (0.02, 0.02, 0.02, 1.0)
VISIBLE_LOS_COLOR = (0.05, 0.8, 0.18, 0.9)
LOST_LOS_COLOR = (1.0, 0.05, 0.05, 0.9)


def write_replay_mcap(
    scenario: str,
    results: dict[str, SimulationResult],
    metrics_table: dict[str, dict[str, float]],
    output_path: Path,
    config: SimulationConfig,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    first_result = next(iter(results.values()))
    yaw_rates = {algorithm: yaw_rate_command(result.yaw, config.dt) for algorithm, result in results.items()}

    with foxglove.open_mcap(output_path, allow_overwrite=True):
        target_scene = SceneUpdateChannel("/target/scene")
        algorithm_scenes = {algorithm: SceneUpdateChannel(f"/{algorithm}/scene") for algorithm in results}
        telemetry_schema = _telemetry_schema()
        telemetry_channels = {
            algorithm: foxglove.Channel(
                f"/{algorithm}/telemetry",
                schema=telemetry_schema,
                message_encoding="json",
                metadata={
                    "label": ALGORITHM_LABELS[algorithm],
                    "scenario": scenario,
                    "capture_time": f"{metrics_table[algorithm]['capture_time']:.6g}",
                    "min_distance": f"{metrics_table[algorithm]['min_distance']:.6g}",
                },
            )
            for algorithm in results
        }

        target_scene.log(_target_static_scene(first_result), log_time=0)
        for algorithm, result in results.items():
            algorithm_scenes[algorithm].log(_algorithm_static_scene(algorithm, result), log_time=0)

        for index, time_value in enumerate(first_result.time):
            log_time = _nanoseconds(float(time_value))
            target_scene.log(_target_dynamic_scene(first_result, index), log_time=log_time)
            for algorithm, result in results.items():
                algorithm_scenes[algorithm].log(
                    _algorithm_dynamic_scene(algorithm, result, first_result, index),
                    log_time=log_time,
                )
                telemetry_channels[algorithm].log(
                    _telemetry_payload(result, yaw_rates[algorithm], index),
                    log_time=log_time,
                )


def _target_static_scene(result: SimulationResult) -> SceneUpdate:
    return SceneUpdate(
        entities=[
            SceneEntity(
                timestamp=_timestamp(0.0),
                frame_id="world",
                id="target/path",
                lines=[
                    LinePrimitive(
                        type=LinePrimitiveLineType.LineStrip,
                        points=_points(result.target_position),
                        thickness=0.08,
                        color=_color(TARGET_COLOR),
                    )
                ],
            )
        ]
    )


def _target_dynamic_scene(result: SimulationResult, index: int) -> SceneUpdate:
    time_value = float(result.time[index])
    position = result.target_position[index]
    return SceneUpdate(
        entities=[
            SceneEntity(
                timestamp=_timestamp(time_value),
                frame_id="world",
                id="target/current",
                spheres=[
                    SpherePrimitive(
                        pose=_pose(position),
                        size=Vector3(x=0.9, y=0.9, z=0.9),
                        color=_color(TARGET_COLOR),
                    )
                ],
                texts=[
                    TextPrimitive(
                        pose=_pose(position + np.array([0.0, 0.0, 1.2])),
                        billboard=True,
                        font_size=12.0,
                        scale_invariant=True,
                        color=_color(TARGET_COLOR),
                        text="Target",
                    )
                ],
            )
        ]
    )


def _algorithm_static_scene(algorithm: str, result: SimulationResult) -> SceneUpdate:
    rgba = ALGORITHM_COLORS[algorithm]
    return SceneUpdate(
        entities=[
            SceneEntity(
                timestamp=_timestamp(0.0),
                frame_id="world",
                id=f"{algorithm}/path",
                lines=[
                    LinePrimitive(
                        type=LinePrimitiveLineType.LineStrip,
                        points=_points(result.pursuer_position),
                        thickness=0.08,
                        color=_color(rgba),
                    )
                ],
            ),
            SceneEntity(
                timestamp=_timestamp(0.0),
                frame_id="world",
                id=f"{algorithm}/start",
                spheres=[
                    SpherePrimitive(
                        pose=_pose(result.pursuer_position[0]),
                        size=Vector3(x=0.55, y=0.55, z=0.55),
                        color=_color((rgba[0], rgba[1], rgba[2], 0.55)),
                    )
                ],
            ),
        ]
    )


def _algorithm_dynamic_scene(
    algorithm: str,
    result: SimulationResult,
    target_result: SimulationResult,
    index: int,
) -> SceneUpdate:
    time_value = float(result.time[index])
    position = result.pursuer_position[index]
    target_position = target_result.target_position[index]
    forward = forward_from_yaw_pitch(float(result.yaw[index]), float(result.pitch[index]))
    heading_end = position + forward * 2.2
    rgba = ALGORITHM_COLORS[algorithm]
    los_rgba = VISIBLE_LOS_COLOR if bool(result.visible[index]) else LOST_LOS_COLOR
    label = ALGORITHM_LABELS[algorithm]

    return SceneUpdate(
        entities=[
            SceneEntity(
                timestamp=_timestamp(time_value),
                frame_id="world",
                id=f"{algorithm}/current",
                spheres=[
                    SpherePrimitive(
                        pose=_pose(position, float(result.yaw[index]), float(result.pitch[index])),
                        size=Vector3(x=0.75, y=0.75, z=0.75),
                        color=_color(rgba),
                    )
                ],
                lines=[
                    LinePrimitive(
                        type=LinePrimitiveLineType.LineStrip,
                        points=[_point(position), _point(heading_end)],
                        thickness=0.12,
                        color=_color(rgba),
                    ),
                    LinePrimitive(
                        type=LinePrimitiveLineType.LineStrip,
                        points=[_point(position), _point(target_position)],
                        thickness=0.04,
                        color=_color(los_rgba),
                    ),
                ],
                texts=[
                    TextPrimitive(
                        pose=_pose(position + np.array([0.0, 0.0, 1.0])),
                        billboard=True,
                        font_size=10.0,
                        scale_invariant=True,
                        color=_color(rgba),
                        text=label,
                    )
                ],
            )
        ]
    )


def _telemetry_payload(result: SimulationResult, yaw_rate: np.ndarray, index: int) -> dict[str, float | bool]:
    acceleration_norm = float(np.linalg.norm(result.acceleration[index]))
    yaw_rate_value = float(yaw_rate[index - 1]) if index > 0 and yaw_rate.size >= index else 0.0
    return {
        "time": float(result.time[index]),
        "distance": float(result.distance[index]),
        "visible": bool(result.visible[index]),
        "los_angle_deg": float(np.rad2deg(result.los_angle[index])),
        "acceleration_norm": acceleration_norm,
        "yaw": float(result.yaw[index]),
        "pitch": float(result.pitch[index]),
        "yaw_rate": yaw_rate_value,
    }


def _telemetry_schema() -> foxglove.Schema:
    schema = {
        "type": "object",
        "properties": {
            "time": {"type": "number"},
            "distance": {"type": "number"},
            "visible": {"type": "boolean"},
            "los_angle_deg": {"type": "number"},
            "acceleration_norm": {"type": "number"},
            "yaw": {"type": "number"},
            "pitch": {"type": "number"},
            "yaw_rate": {"type": "number"},
        },
        "required": [
            "time",
            "distance",
            "visible",
            "los_angle_deg",
            "acceleration_norm",
            "yaw",
            "pitch",
            "yaw_rate",
        ],
    }
    return foxglove.Schema(name="GuidanceTelemetry", encoding="jsonschema", data=json.dumps(schema).encode("utf-8"))


def _points(values: np.ndarray) -> list[Point3]:
    return [_point(row) for row in values]


def _point(value: np.ndarray) -> Point3:
    return Point3(x=float(value[0]), y=float(value[1]), z=float(value[2]))


def _pose(position: np.ndarray, yaw: float = 0.0, pitch: float = 0.0) -> Pose:
    return Pose(position=_vector(position), orientation=_quaternion_from_yaw_pitch(yaw, pitch))


def _vector(value: np.ndarray) -> Vector3:
    return Vector3(x=float(value[0]), y=float(value[1]), z=float(value[2]))


def _quaternion_from_yaw_pitch(yaw: float, pitch: float) -> Quaternion:
    half_yaw = yaw * 0.5
    half_pitch = pitch * 0.5
    cy = float(np.cos(half_yaw))
    sy = float(np.sin(half_yaw))
    cp = float(np.cos(half_pitch))
    sp = float(np.sin(half_pitch))
    return Quaternion(x=-sy * sp, y=cy * sp, z=sy * cp, w=cy * cp)


def _color(rgba: tuple[float, float, float, float]) -> Color:
    return Color(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])


def _timestamp(time_value: float) -> Timestamp:
    nanoseconds = _nanoseconds(time_value)
    seconds, nanos = divmod(nanoseconds, 1_000_000_000)
    return Timestamp(sec=seconds, nsec=nanos)


def _nanoseconds(time_value: float) -> int:
    return int(round(time_value * 1_000_000_000))
