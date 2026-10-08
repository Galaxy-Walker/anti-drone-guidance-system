#!/usr/bin/env python3
"""conda `ultralytics` 环境常驻推理 worker。

由 `vision_detector` 节点以子进程方式启动，用 stdin/stdout 上的二进制协议通信；
本脚本**不导入 ROS**，也不做任何 ROS 参数解析，保证模型环境与 ROS 包解耦。

协议（大端 u32 长度前缀）：

```text
节点 → worker : [u32 头长度][JSON 头][帧字节]
  JSON: {"seq":12,"stamp_ns":123456789,"width":1280,"height":960,
         "encoding":"rgb8","format":"raw","bytes":3686400}
worker → 节点 : [u32 头长度][JSON]
  JSON: {"seq":12,"stamp_ns":123456789,"inference_ms":2.4,
         "boxes":[[u,v,w,h,score], ...],"class_name":"uav","ok":true}
```

- 启动后 worker 先发一条 `{"ready":true,...}` 握手消息，节点校验后才开始送帧；
- `boxes` 中心坐标与宽高都在**原图**坐标系（ultralytics 已完成 letterbox 反变换）；
- `format="jpeg"` 时帧字节用 `bytes` 字段给出长度，`raw` 时由 width/height/encoding 推算；
- 任何逐帧异常都回 `{"ok":false,"error":...}`，进程保持存活；
- 模型加载失败且提供了 `--fallback-model` 时自动回落到 `.pt`；
- ultralytics/第三方库写到 stdout 的日志会被重定向到 stderr，stdout 只承载协议。

`--self-test` 用法（离线核对坐标是否与 `predict_one_image.py` 一致）：

```bash
/home/srcbit/miniconda3/envs/ultralytics/bin/python scripts/yolo_worker.py \
  --model runs/.../best.engine --self-test /home/srcbit/Det-Fly-YOLO-1third/images/val/0207134.jpg
```
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import struct
import sys
import time
from pathlib import Path

# 必须先接管 stdout：ultralytics 导入与推理会往 stdout 打日志，会破坏二进制协议。
_PROTOCOL_OUT = os.fdopen(os.dup(sys.stdout.fileno()), "wb", buffering=0)
os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
sys.stdout = sys.stderr

import numpy as np  # noqa: E402

HEADER_LENGTH = struct.Struct(">I")
MAX_HEADER_BYTES = 1024 * 1024

ENCODING_CHANNELS = {
    "rgb8": 3,
    "bgr8": 3,
    "rgba8": 4,
    "bgra8": 4,
    "mono8": 1,
}


def log(message: str) -> None:
    print(message, file=sys.stderr, flush=True)


def _quantize_supported() -> bool:
    """ultralytics 8.4 起用 `quantize` 取代 `half`；旧版本没有该参数。"""
    try:
        from ultralytics.cfg import DEFAULT_CFG

        return hasattr(DEFAULT_CFG, "quantize")
    except Exception:  # noqa: BLE001 - 探测失败时按旧版本处理
        return False


def precision_kwargs(half: bool, quantize_supported: bool) -> dict:
    """构造推理精度参数。

    不传 `half=False`：ultralytics 8.4 会对任何显式 `half=` 参数发弃用警告，并把
    `half=False` 映射成“清空精度到 FP32”；不传参时引擎保持自身精度、`.pt` 保持 FP32。
    需要 FP16 时优先使用新参数 `quantize=16`，旧版本回退到 `half=True`。
    """
    if not half:
        return {}
    if quantize_supported:
        return {"quantize": 16}
    return {"half": True}


def read_exact(stream, count: int) -> bytes | None:
    """读满 count 字节；EOF 返回 None。"""
    chunks: list[bytes] = []
    remaining = count
    while remaining > 0:
        chunk = stream.read(remaining)
        if not chunk:
            return None
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def send_message(payload: dict, frame_bytes: bytes = b"") -> None:
    header = json.dumps(payload, separators=(",", ":")).encode("utf-8")
    _PROTOCOL_OUT.write(HEADER_LENGTH.pack(len(header)) + header + frame_bytes)
    _PROTOCOL_OUT.flush()


def decode_frame(frame_bytes: bytes, width: int, height: int, encoding: str, frame_format: str) -> np.ndarray:
    """把协议帧解码成 ultralytics 需要的 BGR uint8 图像。"""
    if frame_format == "jpeg":
        import cv2

        image = cv2.imdecode(np.frombuffer(frame_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("jpeg 解码失败")
        return image

    if encoding not in ENCODING_CHANNELS:
        raise ValueError(f"不支持的 encoding: {encoding!r}")
    if width <= 0 or height <= 0:
        raise ValueError(f"非法图像尺寸 {width}x{height}")
    channels = ENCODING_CHANNELS[encoding]
    expected = width * height * channels
    if len(frame_bytes) != expected:
        raise ValueError(f"帧字节数 {len(frame_bytes)} 与 {width}x{height}x{channels} 不符")

    array = np.frombuffer(frame_bytes, dtype=np.uint8).reshape(height, width, channels)
    if encoding == "bgr8":
        return np.ascontiguousarray(array)
    if encoding == "rgb8":
        return np.ascontiguousarray(array[:, :, ::-1])
    if encoding == "rgba8":
        return np.ascontiguousarray(array[:, :, 2::-1])
    if encoding == "bgra8":
        return np.ascontiguousarray(array[:, :, :3])
    # mono8：复制成三通道，满足 BGR 输入约定。
    return np.repeat(array, 3, axis=2)


class Detector:
    """持有 ultralytics 模型，负责单帧推理与类别过滤。"""

    def __init__(self, args: argparse.Namespace) -> None:
        self._args = args
        self._effective_half = bool(args.half)
        self._quantize_supported = _quantize_supported()
        self.model, self.model_path = self._load_model()
        self.names = self._resolve_names()
        if args.class_name and args.class_name.lower() not in {name.lower() for name in self.names.values()}:
            log(f"[worker] 警告：模型类别 {list(self.names.values())} 中没有 {args.class_name!r}，将不会过滤类别外的框")
        precision = "fp16" if self._effective_half else ("engine" if self.model_path.lower().endswith(".engine") else "fp32")
        log(
            f"[worker] loaded {self.model_path} device={args.device} imgsz={args.imgsz} "
            f"precision={precision} conf={args.conf} iou={args.iou}"
        )

    def _load_model(self):
        from ultralytics import YOLO

        candidates = [self._args.model]
        if self._args.fallback_model:
            candidates.append(self._args.fallback_model)
        last_error: Exception | None = None
        for candidate in candidates:
            if not candidate:
                continue
            if not Path(candidate).is_file():
                last_error = FileNotFoundError(candidate)
                log(f"[worker] 模型不存在：{candidate}")
                continue
            try:
                model = YOLO(candidate)
                # 在握手前跑一次空图：TensorRT/ONNX 引擎的实际加载错误会在这里暴露。
                self._warmup(model)
                return model, candidate
            except Exception as exc:  # noqa: BLE001 - 加载失败要回退而不是崩溃
                last_error = exc
                log(f"[worker] 模型加载失败 {candidate}：{exc}")
        raise RuntimeError(f"没有可用模型：{last_error}")

    def _warmup(self, model) -> None:
        dummy = np.zeros((self._args.imgsz, self._args.imgsz, 3), dtype=np.uint8)
        model.predict(
            source=dummy,
            imgsz=self._args.imgsz,
            conf=self._args.conf,
            iou=self._args.iou,
            device=self._args.device,
            max_det=self._args.max_det,
            verbose=False,
            **precision_kwargs(self._effective_half, self._quantize_supported),
        )

    def _resolve_names(self) -> dict[int, str]:
        names = getattr(self.model, "names", None) or {}
        if isinstance(names, (list, tuple)):
            return {index: str(name) for index, name in enumerate(names)}
        return {int(key): str(value) for key, value in dict(names).items()}

    @property
    def args(self) -> argparse.Namespace:
        return self._args

    def infer(self, image_bgr: np.ndarray) -> tuple[list[list[float]], float]:
        start = time.perf_counter()
        results = self.model.predict(
            source=image_bgr,
            imgsz=self._args.imgsz,
            conf=self._args.conf,
            iou=self._args.iou,
            device=self._args.device,
            max_det=self._args.max_det,
            verbose=False,
            **precision_kwargs(self._effective_half, self._quantize_supported),
        )
        inference_ms = (time.perf_counter() - start) * 1e3

        boxes: list[list[float]] = []
        if results:
            result = results[0]
            raw_boxes = getattr(result, "boxes", None)
            if raw_boxes is not None and len(raw_boxes) > 0:
                centers = raw_boxes.xywh.detach().cpu().numpy()
                scores = raw_boxes.conf.detach().cpu().numpy()
                classes = raw_boxes.cls.detach().cpu().numpy().astype(int)
                for (cx, cy, width, height), score, class_id in zip(centers, scores, classes):
                    name = self.names.get(int(class_id), str(class_id))
                    if self._args.class_name and name.lower() != self._args.class_name.lower():
                        continue
                    boxes.append([float(cx), float(cy), float(width), float(height), float(score)])
        boxes.sort(key=lambda box: box[4], reverse=True)
        return boxes[: self._args.max_det], inference_ms


def _prepare_device(args: argparse.Namespace) -> None:
    """CPU 或 TensorRT 引擎不支持/不需要 FP16 开关，显式关掉并说明。"""
    if str(args.device).lower() == "cpu" and args.half:
        args.half = False
        log("[worker] device=cpu，已关闭 half")
    if args.model.lower().endswith(".engine") and args.half:
        args.half = False
        log("[worker] .engine 已内置 FP16，忽略 --half")


def run_protocol(detector: Detector) -> int:
    send_message(
        {
            "ready": True,
            "model": detector.model_path,
            "device": str(detector.args.device),
            "imgsz": detector.args.imgsz,
            "names": detector.names,
        }
    )
    stdin = sys.stdin.buffer
    while True:
        header_size_bytes = read_exact(stdin, HEADER_LENGTH.size)
        if header_size_bytes is None:
            log("[worker] stdin EOF，退出")
            return 0
        (header_size,) = HEADER_LENGTH.unpack(header_size_bytes)
        if header_size <= 0 or header_size > MAX_HEADER_BYTES:
            log(f"[worker] 非法头长度 {header_size}，退出")
            return 2

        header_bytes = read_exact(stdin, header_size)
        if header_bytes is None:
            log("[worker] 头读取中断，退出")
            return 0
        try:
            header = json.loads(header_bytes.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            log(f"[worker] 头 JSON 解析失败：{exc}")
            continue

        seq = int(header.get("seq", -1))
        stamp_ns = int(header.get("stamp_ns", 0))
        frame_size = int(header.get("bytes", 0))
        frame_bytes = b""
        if frame_size > 0:
            frame_bytes = read_exact(stdin, frame_size) or b""
            if len(frame_bytes) != frame_size:
                log(f"[worker] 帧数据不完整（{len(frame_bytes)}/{frame_size}），退出")
                return 0

        try:
            image = decode_frame(
                frame_bytes,
                int(header.get("width", 0)),
                int(header.get("height", 0)),
                str(header.get("encoding", "")),
                str(header.get("format", "raw")),
            )
            boxes, inference_ms = detector.infer(image)
            send_message(
                {
                    "seq": seq,
                    "stamp_ns": stamp_ns,
                    "inference_ms": inference_ms,
                    "boxes": boxes,
                    "class_name": detector._args.class_name,
                    "ok": True,
                }
            )
        except Exception as exc:  # noqa: BLE001 - 单帧失败不能杀掉常驻进程
            log(f"[worker] 第 {seq} 帧推理失败：{exc}")
            send_message({"seq": seq, "stamp_ns": stamp_ns, "ok": False, "error": str(exc)})


def run_self_test(detector: Detector, pattern: str) -> int:
    import cv2

    paths: list[str] = []
    for item in sorted(pattern.split(",")):
        item = item.strip()
        if not item:
            continue
        if Path(item).is_dir():
            paths.extend(sorted(glob.glob(str(Path(item) / "*.jpg")) + glob.glob(str(Path(item) / "*.png"))))
        elif any(char in item for char in "*?["):
            paths.extend(sorted(glob.glob(item)))
        else:
            paths.append(item)
    if not paths:
        log(f"[worker] --self-test 没有匹配到图像：{pattern}")
        return 2

    failures = 0
    for path in paths:
        image = cv2.imread(path, cv2.IMREAD_COLOR)
        if image is None:
            log(f"[worker] 图像读取失败：{path}")
            failures += 1
            continue
        try:
            boxes, inference_ms = detector.infer(image)
        except Exception as exc:  # noqa: BLE001
            log(f"[worker] {path} 推理失败：{exc}")
            failures += 1
            continue
        line = json.dumps(
            {"image": str(path), "boxes": boxes, "inference_ms": inference_ms},
            ensure_ascii=False,
        )
        # --self-test 是离线工具模式，结果写到真实 stdout 便于重定向；日志仍在 stderr。
        _PROTOCOL_OUT.write((line + "\n").encode("utf-8"))
    return 1 if failures else 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="常驻 YOLO 推理 worker（stdin/stdout 二进制协议）")
    parser.add_argument("--model", default="", help="模型权重路径（.engine 优先，.pt 亦可）")
    parser.add_argument("--fallback-model", default="", help="主模型加载失败时回落的 .pt")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.7)
    parser.add_argument("--device", default="0")
    parser.add_argument("--half", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--max-det", type=int, default=5)
    parser.add_argument("--class-name", default="uav", help="只保留该类别（大小写不敏感），空串表示不过滤")
    parser.add_argument("--self-test", default="", help="离线自检：图片路径、目录、glob 或逗号分隔列表")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    _prepare_device(args)
    if not args.model:
        log("[worker] 必须提供 --model")
        return 2

    try:
        detector = Detector(args)
    except Exception as exc:  # noqa: BLE001
        log(f"[worker] 初始化失败：{exc}")
        return 1

    if args.self_test:
        return run_self_test(detector, args.self_test)
    return run_protocol(detector)


if __name__ == "__main__":
    sys.exit(main())
