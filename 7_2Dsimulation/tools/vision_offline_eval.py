#!/usr/bin/env python3
"""零样本/微调模型的离线门槛评估（用 conda `ultralytics` python 运行，不属于 uv 项目依赖）。

读取 `vision_detector` + `vision_adapter` 录制的数据集目录：

```text
dataset/
  frames/<stamp_ns>.jpg    # 相机原图
  labels/<stamp_ns>.txt    # YOLO 归一化真值框；无标签帧按背景负样本处理
```

统计 Recall@IoU0.3/0.5、匹配框中心像素误差 p50/p95，并扫描 conf ∈ [0.1, 0.5]：

```bash
/home/srcbit/miniconda3/envs/ultralytics/bin/python tools/vision_offline_eval.py \
  --dataset outputs/gazebo2d_vision/dataset \
  --model /home/srcbit/anti-drone/ultralytics-main/runs/detect/yolo26_caa_p3_dysample_detfly/weights/best.engine \
  --output outputs/gazebo2d_vision/eval
```

门槛（计划 §P3）：Recall@IoU0.3 ≥ 0.8（conf=0.25）直接进入闭环；0.5～0.8 降 conf + 门控后继续；
< 0.5 进入 P7 域适配微调。本工具只产出报告，不做自动决策。
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

IMAGE_SUFFIXES = (".jpg", ".jpeg", ".png", ".bmp")
DEFAULT_CONF_SWEEP = tuple(round(0.1 + 0.05 * index, 2) for index in range(9))  # 0.10 ~ 0.50


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Offline recall/pixel-error evaluation for the YOLO vision loop")
    parser.add_argument("--dataset", type=Path, required=True, help="Dataset dir with frames/ and labels/")
    parser.add_argument("--model", required=True, help=".engine or .pt weights")
    parser.add_argument("--output", type=Path, help="Report output dir; defaults to <dataset>/eval")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--iou", type=float, default=0.7, help="NMS IoU threshold for prediction")
    parser.add_argument("--device", default="0")
    parser.add_argument("--half", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--max-det", type=int, default=20)
    parser.add_argument("--conf-min", type=float, default=DEFAULT_CONF_SWEEP[0])
    parser.add_argument("--conf-max", type=float, default=DEFAULT_CONF_SWEEP[-1])
    parser.add_argument("--conf-step", type=float, default=0.05)
    parser.add_argument("--gate-conf", type=float, default=0.25, help="conf used for the headline gate row")
    parser.add_argument("--save-annotated", type=Path, help="Optional dir for annotated images at gate conf")
    parser.add_argument("--max-frames", type=int, default=0, help="Limit frames for a quick smoke run")
    return parser.parse_args()


def conf_sweep(args: argparse.Namespace) -> list[float]:
    values = []
    value = args.conf_min
    while value <= args.conf_max + 1e-9:
        values.append(round(value, 4))
        value += args.conf_step
    if args.gate_conf not in values:
        values.append(round(args.gate_conf, 4))
    return sorted(set(values))


def image_size(path: Path) -> tuple[int, int]:
    """只读文件头取图像宽高，避免为标注换算解码整张图。"""
    from PIL import Image

    with Image.open(path) as image:
        return int(image.width), int(image.height)


def read_label(path: Path, width: int, height: int) -> np.ndarray | None:
    """YOLO 归一化标注 -> 像素 xywh；空/缺失返回 None（背景帧）。"""
    if not path.is_file():
        return None
    lines = [line.strip() for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    if not lines:
        return None
    fields = lines[0].split()
    if len(fields) < 5:
        raise ValueError(f"Bad label line in {path}: {lines[0]!r}")
    _, center_x, center_y, box_width, box_height = (float(item) for item in fields[:5])
    return np.array(
        [center_x * width, center_y * height, box_width * width, box_height * height],
        dtype=float,
    )


def xywh_to_xyxy(box: np.ndarray) -> np.ndarray:
    cx, cy, width, height = box
    return np.array([cx - width / 2.0, cy - height / 2.0, cx + width / 2.0, cy + height / 2.0], dtype=float)


def iou_xywh(a: np.ndarray, b: np.ndarray) -> float:
    box_a = xywh_to_xyxy(a)
    box_b = xywh_to_xyxy(b)
    left = max(box_a[0], box_b[0])
    top = max(box_a[1], box_b[1])
    right = min(box_a[2], box_b[2])
    bottom = min(box_a[3], box_b[3])
    intersection = max(0.0, right - left) * max(0.0, bottom - top)
    area_a = max(0.0, box_a[2] - box_a[0]) * max(0.0, box_a[3] - box_a[1])
    area_b = max(0.0, box_b[2] - box_b[0]) * max(0.0, box_b[3] - box_b[1])
    union = area_a + area_b - intersection
    return float(intersection / union) if union > 0.0 else 0.0


def collect_frames(dataset: Path, max_frames: int) -> list[Path]:
    frames_dir = dataset / "frames"
    if not frames_dir.is_dir():
        raise FileNotFoundError(f"frames/ not found under {dataset}")
    frames = sorted(
        path for path in frames_dir.iterdir() if path.suffix.lower() in IMAGE_SUFFIXES
    )
    if max_frames > 0:
        frames = frames[:max_frames]
    if not frames:
        raise FileNotFoundError(f"No images found under {frames_dir}")
    return frames


def _quantize_supported() -> bool:
    """ultralytics 8.4 起用 `quantize` 取代 `half`；旧版本没有该参数。"""
    try:
        from ultralytics.cfg import DEFAULT_CFG

        return hasattr(DEFAULT_CFG, "quantize")
    except Exception:  # noqa: BLE001
        return False


def precision_kwargs(half: bool, quantize_supported: bool) -> dict:
    """与 `scripts/yolo_worker.py` 相同：不传 `half=False`，FP16 优先用 `quantize=16`。"""
    if not half:
        return {}
    if quantize_supported:
        return {"quantize": 16}
    return {"half": True}


def load_model(args: argparse.Namespace):
    from ultralytics import YOLO

    half = args.half
    if str(args.device).lower() == "cpu" and half:
        half = False
        print("[eval] device=cpu，已关闭 half")
    if args.model.lower().endswith(".engine") and half:
        half = False
        print("[eval] .engine 已内置 FP16，忽略 --half")
    model = YOLO(args.model)
    return model, half


def predict_boxes(model, image_path: Path, args: argparse.Namespace, half: bool, conf_floor: float) -> tuple[list[np.ndarray], np.ndarray]:
    """一次推理返回所有候选框（conf_floor 以上）与分数，供 conf 扫描复用。"""
    results = model.predict(
        source=str(image_path),
        imgsz=args.imgsz,
        conf=conf_floor,
        iou=args.iou,
        device=args.device,
        max_det=args.max_det,
        verbose=False,
        **precision_kwargs(half, _quantize_supported()),
    )
    boxes: list[np.ndarray] = []
    scores: list[float] = []
    if results and results[0].boxes is not None and len(results[0].boxes) > 0:
        xywh = results[0].boxes.xywh.detach().cpu().numpy()
        conf = results[0].boxes.conf.detach().cpu().numpy()
        for box, score in zip(xywh, conf):
            boxes.append(np.asarray(box, dtype=float))
            scores.append(float(score))
    return boxes, np.asarray(scores, dtype=float)


def evaluate(args: argparse.Namespace) -> dict:
    frames = collect_frames(args.dataset, args.max_frames)
    labels_dir = args.dataset / "labels"
    sweep = conf_sweep(args)
    conf_floor = min(sweep)
    model, half = load_model(args)

    annotations = []
    missing_labels = 0
    for frame in frames:
        width, height = image_size(frame)
        label = read_label(labels_dir / f"{frame.stem}.txt", width, height)
        if label is None:
            missing_labels += 1
        annotations.append(label)
    label_files = {path.stem for path in labels_dir.glob("*.txt")} if labels_dir.is_dir() else set()
    frame_stems = {frame.stem for frame in frames}
    orphan_labels = len(label_files - frame_stems)

    print(
        f"[eval] frames={len(frames)} (background={missing_labels}), "
        f"orphan_labels={orphan_labels}, model={args.model}, device={args.device}"
    )

    per_conf: dict[float, dict[str, list[float]]] = {
        conf: {"iou03": [], "iou05": [], "center_px": [], "fp": [], "scores": []} for conf in sweep
    }
    for index, frame in enumerate(frames):
        boxes, scores = predict_boxes(model, frame, args, half, conf_floor)
        truth = annotations[index]
        for conf in sweep:
            keep = scores >= conf
            selected = [box for box, flag in zip(boxes, keep) if flag]
            selected_scores = [score for score, flag in zip(scores, keep) if flag]
            if truth is None:
                # 背景帧：任何框都是误检。
                per_conf[conf]["fp"].append(float(len(selected)))
                continue
            if not selected:
                per_conf[conf]["iou03"].append(0.0)
                per_conf[conf]["iou05"].append(0.0)
                continue
            best_index = int(np.argmax(selected_scores))
            best = selected[best_index]
            overlap = iou_xywh(best, truth)
            per_conf[conf]["iou03"].append(1.0 if overlap >= 0.3 else 0.0)
            per_conf[conf]["iou05"].append(1.0 if overlap >= 0.5 else 0.0)
            per_conf[conf]["scores"].append(float(selected_scores[best_index]))
            if overlap >= 0.5:
                center_error = float(
                    np.hypot(best[0] - truth[0], best[1] - truth[1])
                )
                per_conf[conf]["center_px"].append(center_error)
        if args.save_annotated is not None and index % 10 == 0:
            _save_annotated(model, frame, args, half, conf_floor, args.save_annotated)
        if (index + 1) % 50 == 0 or index + 1 == len(frames):
            print(f"[eval] processed {index + 1}/{len(frames)}")

    rows = []
    for conf in sweep:
        data = per_conf[conf]
        positive = len(data["iou03"])
        rows.append(
            {
                "conf": conf,
                "positives": positive,
                "background": len(data["fp"]),
                "recall_iou03": float(np.mean(data["iou03"])) if positive else math.nan,
                "recall_iou05": float(np.mean(data["iou05"])) if positive else math.nan,
                "center_px_p50": _percentile(data["center_px"], 50),
                "center_px_p95": _percentile(data["center_px"], 95),
                "false_positives_per_bg_frame": float(np.mean(data["fp"])) if data["fp"] else math.nan,
                "mean_score": float(np.mean(data["scores"])) if data["scores"] else math.nan,
            }
        )
    return {
        "frames": len(frames),
        "background_frames": missing_labels,
        "orphan_labels": orphan_labels,
        "model": str(args.model),
        "imgsz": args.imgsz,
        "device": str(args.device),
        "rows": rows,
    }


def _save_annotated(model, frame: Path, args, half: bool, conf_floor: float, output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    model.predict(
        source=str(frame),
        imgsz=args.imgsz,
        conf=conf_floor,
        iou=args.iou,
        device=args.device,
        max_det=args.max_det,
        save=True,
        project=str(output_dir),
        name="annotated",
        exist_ok=True,
        verbose=False,
        **precision_kwargs(half, _quantize_supported()),
    )


def _percentile(values: list[float], percentile: float) -> float:
    if not values:
        return math.nan
    return float(np.percentile(np.asarray(values, dtype=float), percentile))


def write_report(result: dict, output_dir: Path, gate_conf: float) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    fieldnames = (
        "conf",
        "positives",
        "background",
        "recall_iou03",
        "recall_iou05",
        "center_px_p50",
        "center_px_p95",
        "false_positives_per_bg_frame",
        "mean_score",
    )
    with (output_dir / "eval_report.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(result["rows"])
    (output_dir / "eval_report.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    gate_row = min(result["rows"], key=lambda row: abs(row["conf"] - gate_conf))
    if math.isnan(gate_row["recall_iou03"]):
        conclusion = "数据集没有正样本，无法给出门槛结论"
    elif gate_row["recall_iou03"] >= 0.8:
        conclusion = f"Recall@IoU0.3={gate_row['recall_iou03']:.3f} ≥ 0.8：可直接进入闭环（P4/P5）"
    elif gate_row["recall_iou03"] >= 0.5:
        conclusion = (
            f"Recall@IoU0.3={gate_row['recall_iou03']:.3f} 落在 [0.5, 0.8)："
            "降低 conf + 马氏门控后继续，并记录风险"
        )
    else:
        conclusion = f"Recall@IoU0.3={gate_row['recall_iou03']:.3f} < 0.5：应进入 P7 域适配微调"

    lines = [
        "# YOLO 视觉零样本门槛评估",
        "",
        f"- 模型：`{result['model']}`（imgsz={result['imgsz']}, device={result['device']}）",
        f"- 帧数：{result['frames']}（背景 {result['background_frames']}），孤立标签 {result['orphan_labels']}",
        f"- 门槛行（conf={gate_row['conf']:.2f}）：Recall@IoU0.3={gate_row['recall_iou03']:.3f}、"
        f"Recall@IoU0.5={gate_row['recall_iou05']:.3f}、"
        f"中心像素误差 p50={gate_row['center_px_p50']:.2f} / p95={gate_row['center_px_p95']:.2f}",
        f"- 结论：{conclusion}",
        "",
        "| conf | Recall@0.3 | Recall@0.5 | center px p50 | center px p95 | FP/背景帧 | 平均分 |",
        "| --- | --- | --- | --- | --- | --- | --- |",
    ]
    for row in result["rows"]:
        lines.append(
            f"| {row['conf']:.2f} | {row['recall_iou03']:.3f} | {row['recall_iou05']:.3f} | "
            f"{row['center_px_p50']:.2f} | {row['center_px_p95']:.2f} | "
            f"{row['false_positives_per_bg_frame']:.3f} | {row['mean_score']:.3f} |"
        )
    lines.append("")
    lines.append(
        "> 说明：中心像素误差只在 IoU≥0.5 的匹配框上统计；真值框由目标 8 角点投影 AABB 加 8% margin 得到，"
        "与 bbox 中心误差不是独立标定，不能当作绝对精度验收。"
    )
    (output_dir / "eval_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    output_dir = args.output if args.output is not None else args.dataset / "eval"
    result = evaluate(args)
    write_report(result, output_dir, args.gate_conf)
    print(f"[eval] report saved to {output_dir}")
    for row in result["rows"]:
        print(
            f"[eval] conf={row['conf']:.2f} recall@0.3={row['recall_iou03']:.3f} "
            f"recall@0.5={row['recall_iou05']:.3f} "
            f"center_px p50/p95={row['center_px_p50']:.2f}/{row['center_px_p95']:.2f}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
