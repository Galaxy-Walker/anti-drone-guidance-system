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

from pythonsimulation.config import ALGORITHMS, SCENARIOS, SimulationConfig  # noqa: E402
from pythonsimulation.metrics import compute_scenario_metrics, write_metrics_csv  # noqa: E402
from pythonsimulation.state import SimulationResult  # noqa: E402


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
    "pitch",
    "distance",
    "visible",
    "los_angle",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot Gazebo/PX4 guidance data saved by gazebosimulation")
    parser.add_argument("csv_path", type=Path, help="Path to gazebo_samples.csv")
    parser.add_argument("--scenario", choices=SCENARIOS, help="Scenario name; inferred from parent directories if omitted")
    parser.add_argument("--algorithm", choices=ALGORITHMS, help="Algorithm name; inferred from parent directories if omitted")
    parser.add_argument("--output-dir", type=Path, help="Directory for metrics.csv and PNG figures; defaults to CSV directory")
    parser.add_argument("--dt", type=float, help="Sample interval for yaw-rate and energy metrics; defaults to median CSV dt")
    parser.add_argument("--sim-time", type=float, help="Metric horizon for uncaptured runs; defaults to last CSV time")
    parser.add_argument("--show", action="store_true", help="Display matplotlib windows after saving figures")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if not args.show:
        import matplotlib

        matplotlib.use("Agg")

    from pythonsimulation.plotting import plot_scenario

    csv_path = args.csv_path.expanduser().resolve()
    scenario = args.scenario or _infer_scenario(csv_path)
    algorithm = args.algorithm or _infer_algorithm(csv_path)
    output_dir = (args.output_dir.expanduser().resolve() if args.output_dir else csv_path.parent)

    result = read_gazebo_csv(csv_path, scenario, algorithm)
    dt = args.dt if args.dt is not None else _infer_dt(result.time)
    sim_time = args.sim_time if args.sim_time is not None else float(result.time[-1])
    config = SimulationConfig(dt=dt, sim_time=sim_time)

    results = {algorithm: result}
    metrics_table = compute_scenario_metrics(results, config)
    write_metrics_csv(metrics_table, output_dir)
    plot_scenario(scenario, results, metrics_table, output_dir, config, show=args.show)
    print(f"Saved Gazebo plots and metrics to {output_dir}")


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
        pitch=_column(rows, "pitch"),
        distance=_column(rows, "distance"),
        visible=np.array([_as_bool(row["visible"]) for row in rows], dtype=bool),
        los_angle=_column(rows, "los_angle"),
    )


def _column(rows: list[dict[str, str]], field: str) -> np.ndarray:
    return np.array([float(row[field]) for row in rows], dtype=float)


def _vector_columns(rows: list[dict[str, str]], x_field: str, y_field: str, z_field: str) -> np.ndarray:
    return np.array([[float(row[x_field]), float(row[y_field]), float(row[z_field])] for row in rows], dtype=float)


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


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


def _infer_dt(times: np.ndarray) -> float:
    deltas = np.diff(times)
    positive = deltas[deltas > 0.0]
    if positive.size:
        return float(np.median(positive))
    return 0.05


if __name__ == "__main__":
    main()
