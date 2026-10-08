"""`sensor_msgs/Image` 到 BGR ndarray 的共享转换。

`vision_detector`（jpeg 编码与数据集落盘）和 `camera_recorder`（周期截图）必须走同一份
编码分支，避免每个节点各写一套 rgb/rgba/mono 转换。转换失败抛 `ValueError`，由调用方
决定记日志还是丢帧。
"""

from __future__ import annotations

import numpy as np
from sensor_msgs.msg import Image

# 相机桥接输出这些编码；bigendian 图像直接拒绝。
SUPPORTED_ENCODINGS = frozenset({"rgb8", "bgr8", "rgba8", "bgra8", "mono8"})
ENCODING_CHANNELS = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}


def image_message_to_bgr(message: Image) -> np.ndarray:
    """把 Image 转成连续 BGR uint8 数组；尺寸、编码或字节数不支持时抛 `ValueError`。

    多出的字节按原逻辑截断，避免桥接携带 padding 时误判为非法帧。
    """
    width, height = int(message.width), int(message.height)
    encoding = str(message.encoding)
    if width <= 0 or height <= 0 or encoding not in SUPPORTED_ENCODINGS:
        raise ValueError(f"尺寸或编码不支持：{width}x{height} {encoding!r}")
    if message.is_bigendian:
        raise ValueError("bigendian 图像不支持")

    channels = ENCODING_CHANNELS[encoding]
    raw = bytes(message.data)
    expected = width * height * channels
    if len(raw) < expected:
        raise ValueError(f"图像数据不足：{len(raw)} < {expected}")

    array = np.frombuffer(raw[:expected], dtype=np.uint8).reshape(height, width, channels)
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
