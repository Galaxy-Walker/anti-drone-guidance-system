"""CSV 输出路径统一以二维仿真工作空间为基准。"""

from __future__ import annotations

from pathlib import Path


def resolve_recording_path(value: str) -> Path:
    """保留显式绝对路径；相对路径不随启动终端的工作目录改变。"""
    path = Path(value).expanduser()
    if path.is_absolute():
        return path

    # 同时兼容源码、普通 colcon 安装与符号链接安装，不硬编码机器路径。
    for parent in Path(__file__).resolve().parents:
        if (parent / "src" / "pythonsimulation2d").is_dir():
            return parent / path
    raise RuntimeError("找不到二维仿真工作空间，请为 CSV 输出参数指定绝对路径")
