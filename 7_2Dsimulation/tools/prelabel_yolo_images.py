#!/usr/bin/env python3
"""合并相机截图并生成 YOLO 预标注，使用现有 conda ultralytics 环境运行。"""

from __future__ import annotations

import argparse
import csv
import json
import shutil
import sys
from pathlib import Path

IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff", ".webp"}
MODULE_DIR = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="合并 run1/run2 图片并生成供人工修正的 YOLO 标签")
    parser.add_argument("--source", type=Path, default=MODULE_DIR / "outputs/table_occlusion_frames")
    parser.add_argument("--runs", nargs="+", default=["run1", "run2"], help="源目录中的子目录")
    parser.add_argument(
        "--weights", type=Path,
        default=Path.home() / "ultralytics-main/runs/detect/yolo26_baseline_detfly/weights/best.pt",
    )
    parser.add_argument("--output", type=Path, default=Path.home() / "table_occlusion_prelabel")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.10, help="预标注采用较低阈值，误检由人工删除")
    parser.add_argument("--device", default="0", help="GPU 编号或 cpu")
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--limit", type=int, default=0, help="仅处理前 N 张；0 表示全部")
    parser.add_argument("--no-previews", action="store_true", help="不保存检测框预览图")
    args = parser.parse_args()
    if not 0 <= args.conf <= 1:
        parser.error("--conf 必须在 0 到 1 之间")
    if args.imgsz <= 0 or args.batch <= 0 or args.limit < 0:
        parser.error("--imgsz 和 --batch 必须大于 0，--limit 不能为负数")
    return args


def collect_images(source: Path, runs: list[str]) -> list[tuple[Path, str]]:
    images: list[tuple[Path, str]] = []
    image_names: set[str] = set()
    label_names: set[str] = set()
    for run in runs:
        run_dir = source / run
        if not run_dir.is_dir():
            raise FileNotFoundError(f"图片目录不存在：{run_dir}")
        for path in sorted(run_dir.rglob("*")):
            if not path.is_file() or path.suffix.lower() not in IMAGE_SUFFIXES:
                continue
            # 保留来源前缀，避免两个录制批次的时间戳文件互相覆盖。
            name = "_".join(path.relative_to(source).parts)
            label_name = Path(name).with_suffix(".txt").name
            if name in image_names or label_name in label_names:
                raise ValueError(f"合并后图片或标签重名：{name}，请先调整源文件名")
            image_names.add(name)
            label_names.add(label_name)
            images.append((path, name))
    if not images:
        raise ValueError(f"没有找到图片：{source}")
    return images


def main() -> None:
    args = parse_args()
    source = args.source.expanduser().resolve()
    weights = args.weights.expanduser().resolve()
    output = args.output.expanduser().resolve()
    images = collect_images(source, args.runs)
    if args.limit:
        images = images[:args.limit]
    if not weights.is_file():
        raise FileNotFoundError(f"权重不存在：{weights}")
    # 人工修订过的标签不可被再次推理覆盖；重跑必须指定新的输出目录。
    if output.exists():
        raise FileExistsError(f"输出目录已存在：{output}；请通过 --output 指定新目录")
    if output == source or source in output.parents:
        raise ValueError("输出目录不能位于源图片目录内")

    import ultralytics
    from ultralytics import YOLO

    print(f"Python：{sys.executable}", flush=True)
    print(f"Ultralytics：{ultralytics.__file__}", flush=True)
    model = YOLO(str(weights))
    if model.task != "detect":
        raise ValueError(f"需要目标检测权重，当前任务为：{model.task}")
    names = {int(index): str(name) for index, name in model.names.items()}
    if sorted(names) != list(range(len(names))):
        raise ValueError("模型类别编号必须从 0 开始且连续")

    output.mkdir(parents=True, exist_ok=False)
    image_dir = output / "images"
    label_dir = output / "labels"
    preview_dir = output / "previews"
    image_dir.mkdir()
    label_dir.mkdir()
    if not args.no_previews:
        preview_dir.mkdir()
    (output / "classes.txt").write_text(
        "".join(f"{names[index]}\n" for index in range(len(names))), encoding="utf-8",
    )

    print(f"共 {len(images)} 张，conf={args.conf}，输出：{output}", flush=True)
    positive_count = 0
    box_count = 0
    with (output / "manifest.csv").open("w", encoding="utf-8", newline="") as manifest:
        writer = csv.writer(manifest)
        writer.writerow(["source", "image", "label", "width", "height", "detections", "max_confidence"])
        for start in range(0, len(images), args.batch):
            batch = images[start:start + args.batch]
            results = model.predict(
                source=[str(path) for path, _ in batch], imgsz=args.imgsz,
                conf=args.conf, device=args.device, batch=args.batch,
                save=False, save_txt=False, verbose=False,
            )
            if len(results) != len(batch):
                raise RuntimeError("推理结果数量与图片数量不一致")
            # 列表输入的结果按输入顺序返回；部分版本把 result.path 改成 imageN.jpg。
            for (path, name), result in zip(batch, results, strict=True):
                if result.boxes is None:
                    raise RuntimeError(f"未返回检测框结果：{path}")
                shutil.copy2(path, image_dir / name)
                label_path = label_dir / Path(name).with_suffix(".txt")
                coordinates = result.boxes.xywhn.cpu().tolist()
                classes = result.boxes.cls.cpu().tolist()
                confidences = result.boxes.conf.cpu().tolist()
                # 训练标签严格使用五列，置信度只保留在预览和清单中。
                lines = [
                    f"{int(class_id)} " + " ".join(f"{value:.8f}" for value in box) + "\n"
                    for class_id, box in zip(classes, coordinates, strict=True)
                ]
                # 即使没有检测也生成空标签；人工复核前不能认定为空背景。
                label_path.write_text("".join(lines), encoding="utf-8")
                if not args.no_previews:
                    result.save(filename=str(preview_dir / name))
                height, width = result.orig_shape
                count = len(lines)
                writer.writerow([
                    str(path), f"images/{name}", f"labels/{label_path.name}",
                    width, height, count, f"{max(confidences):.6f}" if confidences else "",
                ])
                positive_count += int(count > 0)
                box_count += count
            manifest.flush()
            print(f"已完成 {min(start + args.batch, len(images))}/{len(images)}", flush=True)

    summary = {
        "source": str(source), "runs": args.runs, "weights": str(weights),
        "python": sys.executable, "ultralytics": ultralytics.__file__,
        "ultralytics_version": ultralytics.__version__, "classes": names,
        "imgsz": args.imgsz, "conf": args.conf, "device": args.device,
        "images": len(images), "images_with_detections": positive_count,
        "images_without_detections": len(images) - positive_count, "boxes": box_count,
        "annotation_status": "预标注，全部图片均需人工复核后用于微调",
    }
    (output / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8",
    )
    (output / "标注说明.txt").write_text(
        "images/ 为原图副本，labels/ 为同名 YOLO 标签，previews/ 为检测框预览（如启用）。\n"
        "标签格式：class_id x_center y_center width height；坐标归一化到 0～1，不含置信度。\n"
        "classes.txt 的行号从 0 开始，对应模型类别编号。\n"
        "manifest.csv 记录原始路径、图片/标签对应关系及检测数。\n"
        "请在 images/ 原图上复核 labels/ 标签：删除误检、补充漏检、调整检测框。\n"
        "空标签仅代表模型未检测到目标，需要人工确认是否为背景。\n"
        "预览图已画框，仅供查看；微调请使用原图，不要使用预览图。\n"
        "全部复核后再划分训练集/验证集，避免相邻视频帧随机分到两边。\n",
        encoding="utf-8",
    )
    print(
        f"完成：{len(images)} 张图片，{positive_count} 张有框，"
        f"{len(images) - positive_count} 张空标签，共 {box_count} 个框。\n{output}",
        flush=True,
    )


if __name__ == "__main__":
    main()
