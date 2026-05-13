from __future__ import annotations

import csv
from pathlib import Path

import numpy as np

from pythonsimulation.config import ALGORITHM_LABELS, SimulationConfig
from pythonsimulation.state import SimulationResult


METRIC_FIELDS = (
    "capture_time",
    "min_distance",
    "yaw_rate_mean",
    "yaw_rate_variance",
    "mean_distance",
    "path_length",
    "control_energy",
    "lost_duration",
    "visible_ratio",
)


def yaw_rate_command(yaw: np.ndarray, dt: float) -> np.ndarray:
    if yaw.size < 2 or dt <= 0.0:
        return np.array([], dtype=float)
    yaw_delta = (np.diff(yaw) + np.pi) % (2.0 * np.pi) - np.pi
    return yaw_delta / dt


def compute_metrics(result: SimulationResult, config: SimulationConfig) -> dict[str, float]:
    # 捕获时间取第一次进入 capture_radius 的时刻；未捕获则记为 sim_time。
    captured = np.flatnonzero(result.distance <= config.capture_radius)
    capture_time = float(result.time[captured[0]]) if captured.size else float(config.sim_time)
    # 路径长度由相邻位置差累加得到，不依赖速度积分，便于直接从轨迹检查。
    step_distances = np.linalg.norm(np.diff(result.pursuer_position, axis=0), axis=1)
    # 控制能量近似积分 sum(||a||^2 * dt)，用于比较控制强度。
    acceleration_norm_sq = np.sum(result.acceleration**2, axis=1)
    visible_ratio = float(np.mean(result.visible)) if result.visible.size else 0.0
    yaw_rate = yaw_rate_command(result.yaw, config.dt)
    yaw_rate_mean = float(np.mean(np.abs(yaw_rate))) if yaw_rate.size else 0.0
    yaw_rate_variance = float(np.var(yaw_rate)) if yaw_rate.size else 0.0
    return {
        "capture_time": capture_time,
        "min_distance": float(np.min(result.distance)),
        "yaw_rate_mean": yaw_rate_mean,
        "yaw_rate_variance": yaw_rate_variance,
        "mean_distance": float(np.mean(result.distance)),
        "path_length": float(np.sum(step_distances)),
        "control_energy": float(np.sum(acceleration_norm_sq) * config.dt),
        "lost_duration": float(np.sum(~result.visible) * config.dt),
        "visible_ratio": visible_ratio,
    }


def compute_scenario_metrics(
    results: dict[str, SimulationResult],
    config: SimulationConfig,
) -> dict[str, dict[str, float]]:
    return {algorithm: compute_metrics(result, config) for algorithm, result in results.items()}


def write_metrics_csv(metrics_table: dict[str, dict[str, float]], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    with (output_dir / "metrics.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=("algorithm", "label", *METRIC_FIELDS))
        writer.writeheader()
        for algorithm, values in metrics_table.items():
            row = {"algorithm": algorithm, "label": ALGORITHM_LABELS[algorithm]}
            row.update({field: values[field] for field in METRIC_FIELDS})
            writer.writerow(row)
