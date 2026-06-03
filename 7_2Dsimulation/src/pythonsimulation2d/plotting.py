from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from matplotlib import pyplot as plt

from pythonsimulation2d.config import ALGORITHM_LABELS, SimulationConfig
from pythonsimulation2d.metrics import yaw_rate_command
from pythonsimulation2d.state import SimulationResult


CORE_METRICS = (
    ("min_distance", "Minimum miss distance", "m", "{:.2f}"),
    ("capture_time", "Intercept time", "s", "{:.2f}"),
    ("yaw_rate_mean", "Mean |yaw rate|", "rad/s", "{:.3f}"),
    ("yaw_rate_variance", "Yaw rate variance", "(rad/s)^2", "{:.3f}"),
)

ALGORITHM_GRID = (2, 3)


def _algorithm_plot_grid(results: dict[str, SimulationResult]) -> tuple[int, int]:
    if len(results) == 1:
        return (1, 1)
    columns = ALGORITHM_GRID[1]
    rows = max(ALGORITHM_GRID[0], math.ceil(len(results) / columns))
    return rows, columns


def _make_algorithm_axes(
    results: dict[str, SimulationResult],
    figsize: tuple[float, float],
    *,
    sharex: bool = False,
) -> tuple[plt.Figure, np.ndarray]:
    fig, axes = plt.subplots(*_algorithm_plot_grid(results), figsize=figsize, sharex=sharex, squeeze=False)
    return fig, axes


def _algorithm_color(index: int) -> str | None:
    colors = plt.rcParams["axes.prop_cycle"].by_key().get("color", [])
    return colors[index % len(colors)] if colors else None


def _hide_unused_axes(axes: np.ndarray, used_count: int) -> None:
    for ax in axes.ravel()[used_count:]:
        ax.set_visible(False)


def _set_algorithm_grid_labels(axes: np.ndarray, used_count: int, x_label: str, y_label: str) -> None:
    rows, columns = axes.shape
    for index, ax in enumerate(axes.ravel()[:used_count]):
        row, column = divmod(index, columns)
        if row == rows - 1 or index + columns >= used_count:
            ax.set_xlabel(x_label)
        if column == 0:
            ax.set_ylabel(y_label)


def plot_scenario(
    scenario: str,
    results: dict[str, SimulationResult],
    metrics_table: dict[str, dict[str, float]],
    output_dir: Path,
    config: SimulationConfig,
    show: bool = False,
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    _plot_trajectory_xy(scenario, results, output_dir, config)
    _plot_distance_error(scenario, results, output_dir, config)
    _plot_fov_visibility(scenario, results, output_dir, config)
    _plot_acceleration(scenario, results, output_dir)
    _plot_yaw_rate(scenario, results, output_dir, config)
    _plot_metrics(scenario, metrics_table, output_dir)
    if show:
        plt.show()
    else:
        plt.close("all")


def _plot_trajectory_xy(
    scenario: str,
    results: dict[str, SimulationResult],
    output_dir: Path,
    config: SimulationConfig,
) -> None:
    fig, axes = _make_algorithm_axes(results, figsize=(15, 8))
    first = next(iter(results.values()))
    for index, (ax, (algorithm, result)) in enumerate(zip(axes.ravel(), results.items())):
        color = _algorithm_color(index)
        ax.plot(first.target_position[:, 0], first.target_position[:, 1], "k--", label="Ground target")
        ax.plot(result.pursuer_position[:, 0], result.pursuer_position[:, 1], color=color, label="Pursuer @ 8m")
        captured = np.flatnonzero(result.distance <= config.capture_radius)
        if captured.size:
            point = result.pursuer_position[captured[0]]
            ax.scatter(point[0], point[1], s=25, color=color, label="Capture")
        ax.scatter(result.pursuer_position[0, 0], result.pursuer_position[0, 1], c="g", marker="o", s=24, label="Start")
        ax.set_title(ALGORITHM_LABELS[algorithm])
        ax.axis("equal")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7)
    _hide_unused_axes(axes, len(results))
    _set_algorithm_grid_labels(axes, len(results), "x [m]", "y [m]")
    fig.suptitle(f"{scenario}: top-down XY trajectory")
    fig.tight_layout()
    fig.savefig(output_dir / "trajectory_xy.png", dpi=160)


def _plot_distance_error(
    scenario: str,
    results: dict[str, SimulationResult],
    output_dir: Path,
    config: SimulationConfig,
) -> None:
    fig, axes = _make_algorithm_axes(results, figsize=(15, 8), sharex=True)
    for index, (ax, (algorithm, result)) in enumerate(zip(axes.ravel(), results.items())):
        color = _algorithm_color(index)
        (line,) = ax.plot(result.time, result.distance, color=color)
        min_index = int(np.argmin(result.distance))
        min_time = result.time[min_index]
        min_distance = result.distance[min_index]
        color = line.get_color()
        ax.scatter(min_time, min_distance, s=24, color=color, zorder=3)
        ax.annotate(
            f"{min_distance:.2f}m",
            xy=(min_time, min_distance),
            xytext=(4, 5),
            textcoords="offset points",
            fontsize=8,
            color=color,
        )
        ax.axhline(config.capture_radius, color="k", linestyle=":", linewidth=1)
        ax.set_title(ALGORITHM_LABELS[algorithm])
        ax.grid(True, alpha=0.3)
    _hide_unused_axes(axes, len(results))
    _set_algorithm_grid_labels(axes, len(results), "time [s]", "horizontal range error [m]")
    fig.suptitle(f"{scenario}: horizontal distance error")
    fig.tight_layout()
    fig.savefig(output_dir / "distance_error.png", dpi=160)


def _plot_fov_visibility(
    scenario: str,
    results: dict[str, SimulationResult],
    output_dir: Path,
    config: SimulationConfig,
) -> None:
    fig, axes = _make_algorithm_axes(results, figsize=(15, 8), sharex=True)
    fov_deg = np.rad2deg(config.guidance.fov_half_angle)
    for index, (ax, (algorithm, result)) in enumerate(zip(axes.ravel(), results.items())):
        color = _algorithm_color(index)
        visible_ax = ax.twinx()
        ax.plot(result.time, np.rad2deg(result.los_angle), color=color, label="Horizontal LOS angle")
        visible_ax.step(result.time, result.visible.astype(float), where="post", color="0.35", alpha=0.7, label="Visible")
        ax.axhline(fov_deg, color="k", linestyle=":", linewidth=1, label="FOV half angle")
        ax.set_title(ALGORITHM_LABELS[algorithm])
        ax.grid(True, alpha=0.3)
        visible_ax.set_ylim(-0.05, 1.05)
        visible_ax.set_yticks([0.0, 1.0])
        if index % axes.shape[1] != axes.shape[1] - 1:
            visible_ax.set_yticklabels([])
        else:
            visible_ax.set_ylabel("visible")
        lines, labels = ax.get_legend_handles_labels()
        visible_lines, visible_labels = visible_ax.get_legend_handles_labels()
        ax.legend(lines + visible_lines, labels + visible_labels, fontsize=7)
    _hide_unused_axes(axes, len(results))
    _set_algorithm_grid_labels(axes, len(results), "time [s]", "horizontal LOS angle [deg]")
    fig.suptitle(f"{scenario}: horizontal LOS/FOV")
    fig.tight_layout()
    fig.savefig(output_dir / "fov_visibility.png", dpi=160)


def _plot_acceleration(scenario: str, results: dict[str, SimulationResult], output_dir: Path) -> None:
    fig, axes = _make_algorithm_axes(results, figsize=(15, 8), sharex=True)
    for index, (ax, (algorithm, result)) in enumerate(zip(axes.ravel(), results.items())):
        acceleration_norm = np.linalg.norm(result.acceleration[:, :2], axis=1)
        ax.plot(result.time, acceleration_norm, color=_algorithm_color(index))
        ax.set_title(ALGORITHM_LABELS[algorithm])
        ax.grid(True, alpha=0.3)
    _hide_unused_axes(axes, len(results))
    _set_algorithm_grid_labels(axes, len(results), "time [s]", "||a_xy|| [m/s^2]")
    fig.suptitle(f"{scenario}: horizontal acceleration command")
    fig.tight_layout()
    fig.savefig(output_dir / "acceleration.png", dpi=160)


def _plot_yaw_rate(
    scenario: str,
    results: dict[str, SimulationResult],
    output_dir: Path,
    config: SimulationConfig,
) -> None:
    fig, axes = _make_algorithm_axes(results, figsize=(15, 8), sharex=True)
    for index, (ax, (algorithm, result)) in enumerate(zip(axes.ravel(), results.items())):
        yaw_rate = yaw_rate_command(result.yaw, config.dt)
        if not yaw_rate.size:
            continue
        ax.plot(result.time[1:], yaw_rate, color=_algorithm_color(index))
        ax.axhline(0.0, color="k", linestyle=":", linewidth=1)
        ax.set_title(ALGORITHM_LABELS[algorithm])
        ax.grid(True, alpha=0.3)
    _hide_unused_axes(axes, len(results))
    _set_algorithm_grid_labels(axes, len(results), "time [s]", "yaw rate [rad/s]")
    fig.suptitle(f"{scenario}: yaw-rate command")
    fig.tight_layout()
    fig.savefig(output_dir / "yaw_rate.png", dpi=160)


def _plot_metrics(
    scenario: str,
    metrics_table: dict[str, dict[str, float]],
    output_dir: Path,
) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(13, 8))
    algorithms = list(metrics_table)
    labels = [_wrap_algorithm_label(ALGORITHM_LABELS[algorithm]) for algorithm in algorithms]
    for ax, (metric, title, unit, value_format) in zip(axes.ravel(), CORE_METRICS):
        values = [metrics_table[algorithm][metric] for algorithm in algorithms]
        bars = ax.bar(labels, values)
        ax.set_title(f"{title} [{unit}]")
        ax.set_ylabel(unit)
        _annotate_bars(ax, bars, values, value_format)
        ax.tick_params(axis="x", labelsize=8)
        ax.grid(True, axis="y", alpha=0.25)
    fig.suptitle(f"{scenario}: key 2D performance metrics")
    fig.tight_layout()
    fig.savefig(output_dir / "metrics.png", dpi=160)


def _wrap_algorithm_label(label: str) -> str:
    return label.replace(" + ", "\n+ ")


def _annotate_bars(ax: plt.Axes, bars, values: list[float], value_format: str) -> None:
    max_value = max(values) if values else 0.0
    y_limit = max(max_value * 1.18, 1.0)
    ax.set_ylim(0.0, y_limit)
    for bar, value in zip(bars, values):
        ax.annotate(
            value_format.format(value),
            xy=(bar.get_x() + bar.get_width() * 0.5, value),
            xytext=(0, 4),
            textcoords="offset points",
            ha="center",
            va="bottom",
            fontsize=8,
        )
