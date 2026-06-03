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
    parser.add_argument("--show", action="store_true", help="Display matplotlib windows after saving figures")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if not args.show:
        import matplotlib

        matplotlib.use("Agg")

    from pythonsimulation2d.plotting import plot_scenario

    input_path = args.input_path.expanduser().resolve()
    csv_paths = _resolve_csv_paths(input_path, args.algorithm)
    scenario = args.scenario or _infer_scenario(csv_paths[0])
    output_dir = args.output_dir.expanduser().resolve() if args.output_dir else _default_output_dir(input_path)

    results = {}
    for csv_path in csv_paths:
        algorithm = args.algorithm or _infer_algorithm(csv_path)
        if algorithm in results:
            raise ValueError(f"Duplicate Gazebo CSV for algorithm {algorithm!r}")
        results[algorithm] = read_gazebo_csv(csv_path, scenario, algorithm)

    dt = args.dt if args.dt is not None else _infer_dt(next(iter(results.values())).time)
    sim_time = args.sim_time if args.sim_time is not None else max(float(result.time[-1]) for result in results.values())
    config = SimulationConfig(dt=dt, sim_time=sim_time)

    metrics_table = compute_scenario_metrics(results, config)
    write_metrics_csv(metrics_table, output_dir)
    plot_scenario(scenario, results, metrics_table, output_dir, config, show=args.show)
    print(f"Saved Gazebo 2D plots and metrics to {output_dir}")


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
