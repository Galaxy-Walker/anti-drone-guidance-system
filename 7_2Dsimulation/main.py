from __future__ import annotations

import argparse
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    # 项目还没有安装成包时，直接运行 main.py 也能导入 src/pythonsimulation2d。
    sys.path.insert(0, str(SRC))

from pythonsimulation2d.config import ALGORITHMS, SCENARIOS, SimulationConfig  # noqa: E402
from pythonsimulation2d.metrics import compute_scenario_metrics, write_metrics_csv  # noqa: E402
from pythonsimulation2d.simulation import run_scenario  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="2D constant-altitude top-down pursuit guidance simulation")
    # --scenario all 会顺序运行三种地面目标轨迹；单个场景适合调试速度更快。
    parser.add_argument("--scenario", choices=(*SCENARIOS, "all"), default="stationary")
    # --sim-time 和 --dt 会覆盖 SimulationConfig 默认值，便于快速短跑或高精度长跑。
    parser.add_argument("--sim-time", type=float, default=40.0)
    parser.add_argument("--dt", type=float, default=0.05)
    # 默认保存到 outputs/；如果单场景传入自定义目录，就直接保存到该目录。
    parser.add_argument("--save-dir", type=Path, default=Path("outputs"))
    parser.add_argument("--show", action="store_true", help="Display matplotlib windows after saving figures")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    import matplotlib

    if not args.show:
        # 默认只保存图片，不打开窗口；Agg 后端适合命令行和无显示环境。
        matplotlib.use("Agg")

    from pythonsimulation2d.plotting import plot_scenario

    config = SimulationConfig(dt=args.dt, sim_time=args.sim_time)
    scenarios = SCENARIOS if args.scenario == "all" else (args.scenario,)

    for scenario in scenarios:
        # 默认 outputs/ 或 --scenario all 时按场景分目录；单场景自定义目录则直接使用用户路径。
        output_dir = args.save_dir / scenario if args.scenario == "all" or args.save_dir == Path("outputs") else args.save_dir
        results = run_scenario(scenario, config)
        # 指标和绘图都基于同一份 results，避免图表和 CSV 使用不同仿真数据。
        metrics_table = compute_scenario_metrics(results, config)
        write_metrics_csv(metrics_table, output_dir)
        plot_scenario(scenario, results, metrics_table, output_dir, config, show=args.show)
        print_summary(scenario, output_dir, metrics_table)


def print_summary(scenario: str, output_dir: Path, metrics_table: dict[str, dict[str, float]]) -> None:
    print(f"Scenario: {scenario}")
    print(f"Saved: {output_dir}")
    for algorithm in ALGORITHMS:
        values = metrics_table[algorithm]
        print(
            f"  {algorithm}: capture_time={values['capture_time']:.2f}s, "
            f"min_distance={values['min_distance']:.2f}m, "
            f"yaw_rate_mean={values['yaw_rate_mean']:.3f}rad/s, "
            f"yaw_rate_var={values['yaw_rate_variance']:.3f}"
        )


if __name__ == "__main__":
    main()
