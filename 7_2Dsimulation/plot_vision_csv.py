"""绘制 YOLO 视觉闭环记录：检测率、像素残差、延迟与丢失时段。

输入是 `vision_detector` / `vision_adapter` 的输出目录（默认
`outputs/gazebo2d_vision`）：

```bash
cd 7_2Dsimulation
uv run plot_vision_csv.py outputs/gazebo2d_vision --output-dir outputs/vision_report
```

读取两个文件（缺失时跳过对应面板）：

- `yolo_detections.csv`：逐处理帧的检测结果、推理耗时、端到端耗时；
- `vision_samples.csv`：逐量测的有效标志、拒绝原因、像素/位置误差、年龄与延迟。

输出 `detection_rate.png`、`detection_latency.png`、`measurement_error.png`、
`vision_timeline.png`、`rejection_reasons.png`、`vision_metrics.csv`。
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import numpy as np

DETECTION_FIELDS = ("stamp_s", "u", "v", "w", "h", "score", "inference_ms", "e2e_ms", "detections")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot YOLO vision closed-loop CSVs from gazebosimulation2d")
    parser.add_argument(
        "input_path",
        type=Path,
        help="vision output directory, vision_samples.csv, or yolo_detections.csv",
    )
    parser.add_argument("--output-dir", type=Path, help="Figure/metrics output directory; defaults to input directory")
    parser.add_argument("--window-s", type=float, default=1.0, help="Sliding window for detection rate [s]")
    parser.add_argument("--loss-s", type=float, default=1.0, help="Valid-measurement gap counted as a loss event [s]")
    parser.add_argument("--show", action="store_true", help="Display matplotlib windows after saving figures")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not args.show:
        import matplotlib

        matplotlib.use("Agg")

    input_path = args.input_path.expanduser().resolve()
    detections_path, samples_path = _resolve_paths(input_path)
    output_dir = args.output_dir.expanduser().resolve() if args.output_dir else _default_output_dir(input_path)
    output_dir.mkdir(parents=True, exist_ok=True)

    detection_rows = _read_rows(detections_path) if detections_path else []
    sample_rows = _read_rows(samples_path) if samples_path else []
    if not detection_rows and not sample_rows:
        raise FileNotFoundError(f"No vision CSVs found under {input_path}")

    metrics: dict[str, float | str] = {}
    if detection_rows:
        metrics.update(_detection_metrics(detection_rows, args.window_s))
    if sample_rows:
        metrics.update(_sample_metrics(sample_rows, args.loss_s))

    figures = _plot_all(detection_rows, sample_rows, args, output_dir)
    _write_metrics(metrics, output_dir / "vision_metrics.csv")
    print(f"Saved vision figures ({', '.join(figures)}) and metrics to {output_dir}")
    for key, value in metrics.items():
        print(f"  {key}: {value}")


def _resolve_paths(input_path: Path) -> tuple[Path | None, Path | None]:
    if input_path.is_file():
        name = input_path.name
        if name == "yolo_detections.csv":
            return input_path, None
        if name == "vision_samples.csv":
            return None, input_path
        raise ValueError(f"Unrecognized CSV name {name!r}; expected yolo_detections.csv or vision_samples.csv")
    if not input_path.is_dir():
        raise FileNotFoundError(f"Input path not found: {input_path}")
    detections = input_path / "yolo_detections.csv"
    samples = input_path / "vision_samples.csv"
    return (detections if detections.is_file() else None, samples if samples.is_file() else None)


def _default_output_dir(input_path: Path) -> Path:
    return input_path.parent if input_path.is_file() else input_path


def _read_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as file:
        return list(csv.DictReader(file))


def _float(row: dict[str, str], field: str) -> float:
    raw = row.get(field, "")
    if raw is None or raw == "":
        return math.nan
    try:
        return float(raw)
    except ValueError:
        return math.nan


def _detection_metrics(rows: list[dict[str, str]], window_s: float) -> dict[str, float | str]:
    stamps = np.array([_float(row, "stamp_s") for row in rows], dtype=float)
    detections = np.array([_float(row, "detections") for row in rows], dtype=float)
    inference = np.array([_float(row, "inference_ms") for row in rows], dtype=float)
    e2e = np.array([_float(row, "e2e_ms") for row in rows], dtype=float)
    duration = float(np.nanmax(stamps) - np.nanmin(stamps)) if stamps.size > 1 else 0.0

    metrics: dict[str, float | str] = {
        "processed_frames": float(len(rows)),
        "detection_rate": float(np.nanmean(detections)) if detections.size else math.nan,
        "process_hz_observed": float((len(rows) - 1) / duration) if duration > 0.0 else math.nan,
        "inference_ms_p50": _percentile(inference, 50),
        "e2e_ms_p50": _percentile(e2e, 50),
        "e2e_ms_p95": _percentile(e2e, 95),
    }
    return metrics


def _sample_metrics(rows: list[dict[str, str]], loss_s: float) -> dict[str, float | str]:
    yolo_rows = [row for row in rows if row.get("source", "yolo") == "yolo"] or rows
    valid = np.array([_float(row, "valid") for row in yolo_rows], dtype=float)
    stamps = np.array([_float(row, "image_stamp_s") for row in yolo_rows], dtype=float)
    pixel_error = np.array([_float(row, "pixel_error_vs_truth_px") for row in yolo_rows], dtype=float)
    position_error = np.array([_float(row, "position_error_vs_odom_m") for row in yolo_rows], dtype=float)

    valid_stamps = stamps[valid > 0.5]
    loss_gaps = np.diff(valid_stamps) if valid_stamps.size > 1 else np.array([])
    loss_events = loss_gaps[loss_gaps > loss_s] if loss_gaps.size else np.array([])

    reasons: dict[str, int] = {}
    for row in yolo_rows:
        if _float(row, "valid") > 0.5:
            continue
        reason = row.get("invalid_reason") or "unknown"
        reasons[reason] = reasons.get(reason, 0) + 1

    metrics: dict[str, float | str] = {
        "measurement_rows": float(len(yolo_rows)),
        "measurement_valid_rate": float(np.nanmean(valid)) if valid.size else math.nan,
        "pixel_error_px_p50": _percentile(pixel_error, 50),
        "pixel_error_px_p95": _percentile(pixel_error, 95),
        "position_error_m_p50": _percentile(position_error, 50),
        "position_error_m_p95": _percentile(position_error, 95),
        "loss_events": float(loss_events.size),
        "longest_loss_s": float(np.max(loss_events)) if loss_events.size else 0.0,
        "rejection_reasons": ";".join(f"{key}={value}" for key, value in sorted(reasons.items())) or "-",
    }
    return metrics


def _percentile(values: np.ndarray, percentile: float) -> float:
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return math.nan
    return float(np.percentile(finite, percentile))


def _plot_all(
    detection_rows: list[dict[str, str]],
    sample_rows: list[dict[str, str]],
    args: argparse.Namespace,
    output_dir: Path,
) -> list[str]:
    from matplotlib import pyplot as plt

    figures: list[str] = []
    if detection_rows:
        _plot_detection_rate(detection_rows, args.window_s, output_dir, plt)
        figures.append("detection_rate.png")
        _plot_detection_latency(detection_rows, output_dir, plt)
        figures.append("detection_latency.png")
    if sample_rows:
        yolo_rows = [row for row in sample_rows if row.get("source", "yolo") == "yolo"] or sample_rows
        _plot_measurement_error(yolo_rows, output_dir, plt)
        figures.append("measurement_error.png")
        _plot_timeline(yolo_rows, args.loss_s, output_dir, plt)
        figures.append("vision_timeline.png")
        _plot_rejection_reasons(yolo_rows, output_dir, plt)
        figures.append("rejection_reasons.png")
    return figures


def _plot_detection_rate(rows: list[dict[str, str]], window_s: float, output_dir: Path, plt) -> None:
    stamps = np.array([_float(row, "stamp_s") for row in rows], dtype=float)
    detections = np.array([_float(row, "detections") for row in rows], dtype=float)
    rate = float(np.nanmean(detections))

    fig, ax = plt.subplots(figsize=(11, 4.5))
    ax.step(stamps, detections, where="post", color="tab:blue", label="detected (1/0)")
    window_rate, window_time = _sliding_rate(stamps, detections, window_s)
    if window_time.size:
        ax.plot(window_time, window_rate, color="tab:red", label=f"rate in {window_s:g}s window")
    ax.axhline(rate, color="tab:green", linestyle=":", label=f"overall rate={rate:.3f}")
    ax.set_ylim(-0.1, 1.1)
    ax.set_xlabel("image stamp [s]")
    ax.set_ylabel("detection")
    ax.set_title("YOLO detection per processed frame")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(output_dir / "detection_rate.png", dpi=150)
    plt.close(fig)


def _sliding_rate(stamps: np.ndarray, values: np.ndarray, window_s: float) -> tuple[np.ndarray, np.ndarray]:
    finite = np.isfinite(stamps) & np.isfinite(values)
    stamps, values = stamps[finite], values[finite]
    if stamps.size == 0 or window_s <= 0.0:
        return np.array([]), np.array([])
    rates, times = [], []
    for index in range(stamps.size):
        left = np.searchsorted(stamps, stamps[index] - window_s, side="left")
        rates.append(float(np.mean(values[left : index + 1])))
        times.append(float(stamps[index]))
    return np.array(rates), np.array(times)


def _plot_detection_latency(rows: list[dict[str, str]], output_dir: Path, plt) -> None:
    stamps = np.array([_float(row, "stamp_s") for row in rows], dtype=float)
    inference = np.array([_float(row, "inference_ms") for row in rows], dtype=float)
    e2e = np.array([_float(row, "e2e_ms") for row in rows], dtype=float)

    fig, ax = plt.subplots(figsize=(11, 4.5))
    ax.plot(stamps, inference, color="tab:blue", label="worker inference")
    ax.plot(stamps, e2e, color="tab:red", alpha=0.7, label="node end-to-end (wall)")
    for values, color in ((inference, "tab:blue"), (e2e, "tab:red")):
        p95 = _percentile(values, 95)
        if math.isfinite(p95):
            ax.axhline(p95, color=color, linestyle=":", alpha=0.6)
    ax.set_xlabel("image stamp [s]")
    ax.set_ylabel("ms")
    ax.set_title("Detection latency (dotted lines: p95)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(output_dir / "detection_latency.png", dpi=150)
    plt.close(fig)


def _plot_measurement_error(rows: list[dict[str, str]], output_dir: Path, plt) -> None:
    stamps = np.array([_float(row, "image_stamp_s") for row in rows], dtype=float)
    pixel_error = np.array([_float(row, "pixel_error_vs_truth_px") for row in rows], dtype=float)
    position_error = np.array([_float(row, "position_error_vs_odom_m") for row in rows], dtype=float)

    fig, axes = plt.subplots(1, 2, figsize=(13, 4.5))
    ax = axes[0]
    ax.plot(stamps, pixel_error, color="tab:red", marker=".", linestyle="none", markersize=3)
    p50, p95 = _percentile(pixel_error, 50), _percentile(pixel_error, 95)
    if math.isfinite(p50):
        ax.axhline(p50, color="tab:blue", linestyle=":", label=f"p50={p50:.2f} px")
    if math.isfinite(p95):
        ax.axhline(p95, color="tab:orange", linestyle=":", label=f"p95={p95:.2f} px")
    ax.set_xlabel("image stamp [s]")
    ax.set_ylabel("bbox center vs truth projection [px]")
    ax.set_title("Pixel residual (same geometric model; not a calibration)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8)

    ax = axes[1]
    ax.plot(stamps, position_error, color="tab:purple", marker=".", linestyle="none", markersize=3)
    p50, p95 = _percentile(position_error, 50), _percentile(position_error, 95)
    if math.isfinite(p50):
        ax.axhline(p50, color="tab:blue", linestyle=":", label=f"p50={p50:.3f} m")
    if math.isfinite(p95):
        ax.axhline(p95, color="tab:orange", linestyle=":", label=f"p95={p95:.3f} m")
    ax.set_xlabel("image stamp [s]")
    ax.set_ylabel("backprojection vs target odometry XY [m]")
    ax.set_title("Position residual (odometry reference)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8)

    fig.tight_layout()
    fig.savefig(output_dir / "measurement_error.png", dpi=150)
    plt.close(fig)


def _plot_timeline(rows: list[dict[str, str]], loss_s: float, output_dir: Path, plt) -> None:
    stamps = np.array([_float(row, "image_stamp_s") for row in rows], dtype=float)
    valid = np.array([_float(row, "valid") for row in rows], dtype=float)
    age = np.array([_float(row, "detection_age_ms") for row in rows], dtype=float) * 1e-3
    match = np.array([_float(row, "pose_match_dt_ms") for row in rows], dtype=float) * 1e-3

    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    axes[0].step(stamps, valid, where="post", color="tab:blue")
    axes[0].set_ylim(-0.1, 1.1)
    axes[0].set_ylabel("valid")
    axes[0].set_title(f"Vision measurement timeline (dotted: {loss_s:g}s loss threshold)")
    axes[0].grid(True, alpha=0.3)
    for left, right in _loss_intervals(stamps, valid, loss_s):
        axes[0].axvspan(left, right, color="tab:red", alpha=0.15)
    axes[1].plot(stamps, age, color="tab:red", label="image age at adapter")
    axes[1].plot(stamps, match, color="tab:orange", label="pose match dt")
    axes[1].set_xlabel("image stamp [s]")
    axes[1].set_ylabel("seconds")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(output_dir / "vision_timeline.png", dpi=150)
    plt.close(fig)


def _loss_intervals(stamps: np.ndarray, valid: np.ndarray, loss_s: float) -> list[tuple[float, float]]:
    intervals: list[tuple[float, float]] = []
    last_valid: float | None = None
    for stamp, flag in zip(stamps, valid):
        if not math.isfinite(stamp):
            continue
        if flag > 0.5:
            if last_valid is not None and stamp - last_valid > loss_s:
                intervals.append((last_valid, stamp))
            last_valid = stamp
    return intervals


def _plot_rejection_reasons(rows: list[dict[str, str]], output_dir: Path, plt) -> None:
    counts: dict[str, int] = {}
    for row in rows:
        if _float(row, "valid") > 0.5:
            continue
        reason = row.get("invalid_reason") or "unknown"
        counts[reason] = counts.get(reason, 0) + 1
    if not counts:
        counts = {"none": 0}

    labels = sorted(counts, key=counts.get, reverse=True)
    values = [counts[label] for label in labels]
    fig, ax = plt.subplots(figsize=(10, 4.5))
    ax.bar(labels, values, color="tab:orange")
    ax.set_ylabel("frames")
    ax.set_title("Rejected measurement reasons")
    ax.tick_params(axis="x", rotation=30)
    for index, value in enumerate(values):
        ax.text(index, value, str(value), ha="center", va="bottom", fontsize=8)
    fig.tight_layout()
    fig.savefig(output_dir / "rejection_reasons.png", dpi=150)
    plt.close(fig)


def _write_metrics(metrics: dict[str, float | str], path: Path) -> None:
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(("metric", "value"))
        for key, value in metrics.items():
            writer.writerow((key, value))


if __name__ == "__main__":
    main()
