from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parent
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from pythonsimulation2d.config import ALGORITHMS, SCENARIOS, SimulationConfig  # noqa: E402
from pythonsimulation2d.metrics import compute_scenario_metrics, write_metrics_csv  # noqa: E402
from pythonsimulation2d.state import SimulationResult  # noqa: E402
from pythonsimulation2d.target import target_under_table  # noqa: E402


CSV_FIELDS = (
    "time",
    "pursuer_x",
    "pursuer_y",
    "pursuer_z",
    "pursuer_vx",
    "pursuer_vy",
    "pursuer_vz",
    "target_x",
    "target_y",
    "target_z",
    "target_vx",
    "target_vy",
    "target_vz",
    "acceleration_x",
    "acceleration_y",
    "acceleration_z",
    "yaw",
    "distance_xy",
)

# 视觉闭环记录追加列；旧记录没有这些列时跳过视觉面板。
VISION_FIELDS = (
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
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot Gazebo/PX4 2D guidance data saved by gazebosimulation2d")
    parser.add_argument(
        "input_path",
        type=Path,
        help="Path to gazebo_samples.csv, an algorithm directory, or a scenario directory containing */gazebo_samples.csv",
    )
    parser.add_argument("--scenario", choices=SCENARIOS, help="Scenario name; inferred from directories if omitted")
    parser.add_argument("--algorithm", choices=ALGORITHMS, help="Algorithm name; inferred from CSV parent if omitted")
    parser.add_argument("--output-dir", type=Path, help="Directory for metrics.csv and PNG figures; defaults to input directory")
    parser.add_argument("--dt", type=float, help="Sample interval for yaw-rate and energy metrics; defaults to median CSV dt")
    parser.add_argument("--sim-time", type=float, help="Metric horizon for uncaptured runs; defaults to last CSV time")
    parser.add_argument(
        "--trajectory-window-s",
        type=float,
        help="Only draw the first N seconds of the trajectories (20 keeps the circular target from closing into full loops); defaults to the whole record",
    )
    parser.add_argument("--show", action="store_true", help="Display matplotlib windows after saving figures")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if not args.show:
        import matplotlib

        matplotlib.use("Agg")

    from pythonsimulation2d.plotting import plot_scenario
    from pythonsimulation2d.publication_plots import plot_trajectory_panels

    input_path = args.input_path.expanduser().resolve()
    csv_paths = _resolve_csv_paths(input_path, args.algorithm)
    scenario = args.scenario or _infer_scenario(csv_paths[0])
    output_dir = args.output_dir.expanduser().resolve() if args.output_dir else _default_output_dir(input_path)

    results = {}
    csv_by_algorithm = {}
    for csv_path in csv_paths:
        algorithm = args.algorithm or _infer_algorithm(csv_path)
        if algorithm in results:
            raise ValueError(f"Duplicate Gazebo CSV for algorithm {algorithm!r}")
        results[algorithm] = read_gazebo_csv(csv_path, scenario, algorithm)
        csv_by_algorithm[algorithm] = csv_path

    dt = args.dt if args.dt is not None else _infer_dt(next(iter(results.values())).time)
    sim_time = args.sim_time if args.sim_time is not None else max(float(result.time[-1]) for result in results.values())
    config = SimulationConfig(dt=dt, sim_time=sim_time)

    distance_masks = {
        algorithm: read_table_mask(csv_by_algorithm[algorithm], result, config)
        for algorithm, result in results.items()
    }
    metrics_table = compute_gazebo_metrics(results, config, distance_masks)
    write_metrics_csv(metrics_table, output_dir)
    # 轨迹图换成论文版式的网格图，其余四个面板仍复用离线仿真的绘图口径。
    plot_scenario(
        scenario, results, metrics_table, output_dir, config, show=args.show,
        include_trajectory=False, tracking_metrics=True, distance_masks=distance_masks,
    )
    trajectory_path = plot_trajectory_panels(results, output_dir, window_s=args.trajectory_window_s)
    vision_plots = _plot_vision_panels(scenario, results, csv_by_algorithm, output_dir)
    print(f"Saved Gazebo 2D plots and metrics to {output_dir}")
    print(f"  trajectory: {trajectory_path.name}")
    if vision_plots:
        print(f"Saved {vision_plots} vision estimate figure(s) to {output_dir}")


def read_table_mask(csv_path: Path, result: SimulationResult, config: SimulationConfig) -> np.ndarray:
    """优先使用记录时的桌下标志；旧 CSV 按实际位置与场景桌面几何补算。"""
    if result.scenario != "table_occlusion":
        return np.zeros(result.time.shape, dtype=bool)
    with csv_path.open(newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        if "target_under_table" in (reader.fieldnames or []):
            return np.array([float(row["target_under_table"]) >= 0.5 for row in reader], dtype=bool)
    return target_under_table(result.target_position, config.target.table)


def compute_gazebo_metrics(
    results: dict[str, SimulationResult], config: SimulationConfig,
    distance_masks: dict[str, np.ndarray],
) -> dict[str, dict[str, float]]:
    """误差曲线与距离统计使用相同样本，其他指标仍保留完整控制记录。"""
    metrics_table = compute_scenario_metrics(results, config)
    for algorithm, result in results.items():
        distances = result.distance[~distance_masks[algorithm] & np.isfinite(result.distance)]
        metrics_table[algorithm]["mean_distance"] = float(np.mean(distances)) if distances.size else np.nan
        metrics_table[algorithm]["min_distance"] = float(np.min(distances)) if distances.size else np.nan
    return metrics_table


def _plot_vision_panels(
    scenario: str,
    results: dict[str, SimulationResult],
    csv_by_algorithm: dict[str, Path],
    output_dir: Path,
) -> int:
    """含视觉估计列的记录追加 vision_estimate 图；纯 odometry 记录直接跳过。"""
    count = 0
    for algorithm, csv_path in csv_by_algorithm.items():
        vision = read_vision_csv(csv_path)
        if vision is None:
            continue
        filename = "vision_estimate.png" if len(csv_by_algorithm) == 1 else f"vision_estimate_{algorithm}.png"
        plot_vision_estimate(scenario, algorithm, results[algorithm], vision, output_dir / filename)
        count += 1
    return count


def read_vision_csv(csv_path: Path) -> dict[str, np.ndarray] | None:
    """读取视觉估计列；缺少列或整段为 NaN（纯 odometry 记录）时返回 None。"""
    with csv_path.open("r", newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        fieldnames = reader.fieldnames or []
        if not {"target_est_x", "target_est_y"}.issubset(fieldnames):
            return None
        rows = list(reader)
    if not rows:
        return None

    data = {"time": _column(rows, "time")}
    for field in VISION_FIELDS:
        if field in fieldnames:
            data[field] = _optional_column(rows, field)
    if "target_est_x" not in data or np.all(np.isnan(data["target_est_x"])):
        return None
    return data


def plot_vision_estimate(
    scenario: str,
    algorithm: str,
    result: SimulationResult,
    vision: dict[str, np.ndarray],
    path: Path,
) -> None:
    """真值 vs 视觉估计轨迹、误差、有效标志与年龄/延迟。"""
    from matplotlib import pyplot as plt

    time = vision["time"]
    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    fig.suptitle(f"Vision estimate vs truth: {scenario} / {algorithm}")

    ax = axes[0, 0]
    ax.plot(result.target_position[:, 0], result.target_position[:, 1], label="target truth", color="tab:blue")
    ax.plot(result.pursuer_position[:, 0], result.pursuer_position[:, 1], label="pursuer", color="tab:green")
    valid = vision.get("vision_valid", np.full(time.shape, np.nan))
    estimate_x = vision["target_est_x"]
    estimate_y = vision["target_est_y"]
    ax.plot(estimate_x, estimate_y, label="vision estimate", color="tab:red", linestyle="--")
    invalid = ~np.isfinite(valid) | (valid < 0.5)
    if np.any(invalid):
        ax.scatter(estimate_x[invalid], estimate_y[invalid], s=6, color="tab:orange", label="invalid/hold")
    ax.set_xlabel("ENU x [m]")
    ax.set_ylabel("ENU y [m]")
    ax.set_title("XY trajectory")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8)

    ax = axes[0, 1]
    error = vision.get("vision_error_xy", np.full(time.shape, np.nan))
    ax.plot(time, error, color="tab:red")
    finite = error[np.isfinite(error)]
    if finite.size:
        p50 = float(np.percentile(finite, 50))
        p95 = float(np.percentile(finite, 95))
        ax.axhline(p50, color="tab:blue", linestyle=":", label=f"p50={p50:.3f} m")
        ax.axhline(p95, color="tab:orange", linestyle=":", label=f"p95={p95:.3f} m")
        ax.legend(loc="best", fontsize=8)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("estimate vs odometry XY [m]")
    ax.set_title("Vision estimate error (not an independent calibration)")
    ax.grid(True, alpha=0.3)

    ax = axes[1, 0]
    ax.step(time, valid, where="post", color="tab:blue")
    ax.set_ylim(-0.1, 1.1)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("vision_valid")
    ax.set_title("Estimate valid flag")
    ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    for field, label in (("vision_age_s", "age since accepted"), ("vision_latency_s", "measurement latency")):
        values = vision.get(field)
        if values is not None:
            ax.plot(time, values, label=label)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("seconds")
    ax.set_title("Measurement age / latency")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8)

    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150)
    plt.close(fig)


def _optional_column(rows: list[dict[str, str]], field: str) -> np.ndarray:
    """缺失或空串按 NaN 处理；旧记录列可能不存在。"""
    values = []
    for row in rows:
        raw = row.get(field, "")
        if raw is None or raw == "":
            values.append(np.nan)
            continue
        try:
            values.append(float(raw))
        except ValueError:
            values.append(np.nan)
    return np.array(values, dtype=float)


def read_gazebo_csv(csv_path: Path, scenario: str, algorithm: str) -> SimulationResult:
    if not csv_path.is_file():
        raise FileNotFoundError(f"CSV file not found: {csv_path}")

    with csv_path.open("r", newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        missing = sorted(set(CSV_FIELDS) - set(reader.fieldnames or ()))
        if missing:
            raise ValueError(f"{csv_path} is missing fields: {', '.join(missing)}")
        rows = list(reader)

    if not rows:
        raise ValueError(f"{csv_path} contains no samples")

    return SimulationResult(
        scenario=scenario,
        algorithm=algorithm,
        time=_column(rows, "time"),
        pursuer_position=_vector_columns(rows, "pursuer_x", "pursuer_y", "pursuer_z"),
        pursuer_velocity=_vector_columns(rows, "pursuer_vx", "pursuer_vy", "pursuer_vz"),
        target_position=_vector_columns(rows, "target_x", "target_y", "target_z"),
        target_velocity=_vector_columns(rows, "target_vx", "target_vy", "target_vz"),
        acceleration=_vector_columns(rows, "acceleration_x", "acceleration_y", "acceleration_z"),
        yaw=_column(rows, "yaw"),
        distance=_column(rows, "distance_xy"),
    )


def _resolve_csv_paths(input_path: Path, algorithm: str | None) -> list[Path]:
    if input_path.is_file():
        return [input_path]
    if not input_path.is_dir():
        raise FileNotFoundError(f"Input path not found: {input_path}")

    direct_csv = input_path / "gazebo_samples.csv"
    if direct_csv.is_file():
        return [direct_csv]

    algorithms = (algorithm,) if algorithm else ALGORITHMS
    csv_paths = [input_path / item / "gazebo_samples.csv" for item in algorithms]
    csv_paths = [path for path in csv_paths if path.is_file()]
    if not csv_paths:
        raise FileNotFoundError(f"No gazebo_samples.csv files found under {input_path}")
    return csv_paths


def _column(rows: list[dict[str, str]], field: str) -> np.ndarray:
    return np.array([float(row[field]) for row in rows], dtype=float)


def _vector_columns(rows: list[dict[str, str]], x_field: str, y_field: str, z_field: str) -> np.ndarray:
    return np.array([[float(row[x_field]), float(row[y_field]), float(row[z_field])] for row in rows], dtype=float)


def _infer_algorithm(csv_path: Path) -> str:
    algorithm = csv_path.parent.name
    if algorithm not in ALGORITHMS:
        raise ValueError("Could not infer algorithm from CSV path; pass --algorithm")
    return algorithm


def _infer_scenario(csv_path: Path) -> str:
    scenario = csv_path.parent.parent.name
    if scenario not in SCENARIOS:
        raise ValueError("Could not infer scenario from CSV path; pass --scenario")
    return scenario


def _default_output_dir(input_path: Path) -> Path:
    if input_path.is_file():
        return input_path.parent
    return input_path


def _infer_dt(times: np.ndarray) -> float:
    deltas = np.diff(times)
    positive = deltas[deltas > 0.0]
    if positive.size:
        return float(np.median(positive))
    return 0.05


if __name__ == "__main__":
    main()
