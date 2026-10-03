"""论文版式的 XY 轨迹网格图（Gazebo 记录后处理专用）。

版式来自一次性出图脚本：面板尺寸先按英寸算好，figure 尺寸再围绕面板反推，因此
``set_aspect("equal")`` 不会被 ``tight_layout`` 拉伸变形，四宫格能贴紧排布、不留空白。
坐标轴范围由所有算法的数据统一推导，四个面板共用同一组范围，便于直接横向对比。

离线仿真（``main.py`` → ``plotting.plot_scenario``）仍用默认 matplotlib 样式，本模块只服务
``plot_gazebo_csv.py``：两套样式分开，改论文插图不会影响离线出图，反之亦然。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from matplotlib import pyplot as plt

from pythonsimulation2d.config import ALGORITHM_LABELS, ALGORITHM_PANEL_LABELS
from pythonsimulation2d.state import SimulationResult


# 衬线字体（Times 系）+ STIX 数学字体，贴近论文排版；系统缺 Times 时按列表回退。
PUBLICATION_RCPARAMS = {
    "font.family": "serif",
    "font.serif": ["Times New Roman", "Nimbus Roman", "DejaVu Serif"],
    "mathtext.fontset": "stix",
    "axes.unicode_minus": False,
    "figure.facecolor": "white",
}

# (线条颜色, 透明度)。颜色沿用默认 tab10 顺序；MPPI/EMPC 的半透明是参考图的画法，
# 用来在一张图里弱化这两条“预测控制”轨迹。
PANEL_STYLES = {
    "basic": ("#1f77b4", 1.00),
    "pn": ("#ff7f0e", 1.00),
    "pn_mppi": ("#2ca02c", 0.55),
    "pn_nmpc": ("#d62728", 0.55),
}
TARGET_COLOR = "#a6a6a6"
TARGET_DASHES = (0.0, (1.0, 1.6))
TARGET_START_COLOR = "#4d4d4d"
GRID_COLOR = "#dcdcdc"

FS_TITLE = 15
FS_LABEL = 14
FS_TICK = 12
FS_LEGEND = 12
FS_SUPTITLE = 18

DPI = 200
DEFAULT_FILENAME = "trajectories_2x2.png"
DEFAULT_TITLE = "2D Trajectories"

# 版面尺寸（英寸）：面板宽度固定，间距用它换算，保证等比例坐标轴填满面板框。
BOX_W = 4.60
GAP_X = 0.80
LEFT_IN = 0.85
RIGHT_IN = 0.12
TITLE_H = 0.36
XLABEL_H = 0.52
ROW_GAP = 0.08
SUPTITLE_H = 0.42
MARGIN_TOP = 0.06
MARGIN_BOT = 0.06
MAX_COLUMNS = 2

# 坐标轴刻度取整用的步长序列；范围按“约 7 个刻度”挑最接近的档。
NICE_STEPS = (0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0)
TARGET_TICKS = 7.0
PAD_FRACTION = 0.04


@dataclass(slots=True)
class _Panel:
    label: str
    color: str
    alpha: float
    pursuer_xy: np.ndarray
    target_xy: np.ndarray


def plot_trajectory_panels(
    results: dict[str, SimulationResult],
    output_dir: Path,
    *,
    window_s: float | None = None,
    filename: str = DEFAULT_FILENAME,
    title: str = DEFAULT_TITLE,
    dpi: int = DPI,
) -> Path:
    """把所有算法的 XY 轨迹画成论文版式网格图，返回写出的 PNG 路径。

    ``window_s`` 只取前 N 秒的样本：圆周场景整段画完会闭合成整圆，截断后轨迹缺口更容易
    看清跟踪偏差；为 None 时画完整记录。
    """
    if not results:
        raise ValueError("results is empty; nothing to plot")

    panels = [_panel(algorithm, result, window_s) for algorithm, result in results.items()]
    x_limits, y_limits, x_step, y_step = _shared_limits(panels)

    columns = 1 if len(panels) == 1 else MAX_COLUMNS
    rows = math.ceil(len(panels) / columns)
    box_h = BOX_W / ((x_limits[1] - x_limits[0]) / (y_limits[1] - y_limits[0]))
    fig_w = columns * BOX_W + (columns - 1) * GAP_X + LEFT_IN + RIGHT_IN
    fig_h = (
        MARGIN_TOP
        + SUPTITLE_H
        + rows * (TITLE_H + box_h + XLABEL_H)
        + (rows - 1) * ROW_GAP
        + MARGIN_BOT
    )

    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / filename
    # 只在画这张图时切换字体等全局参数，离线和其它面板的默认样式不受影响。
    with plt.rc_context(PUBLICATION_RCPARAMS):
        fig = plt.figure(figsize=(fig_w, fig_h))
        _figure_text_in(
            fig,
            fig_w / 2,
            fig_h - MARGIN_TOP - SUPTITLE_H / 2,
            title,
            ha="center",
            va="center",
            fontsize=FS_SUPTITLE,
            fontweight="bold",
        )
        _place_grid(
            fig,
            panels,
            columns,
            box_w=BOX_W,
            box_h=box_h,
            top_in=fig_h - MARGIN_TOP - SUPTITLE_H - TITLE_H,
            x_limits=x_limits,
            y_limits=y_limits,
            x_step=x_step,
            y_step=y_step,
        )
        fig.savefig(path, dpi=dpi)
        plt.close(fig)
    return path


def _panel(algorithm: str, result: SimulationResult, window_s: float | None) -> _Panel:
    mask = np.ones(result.time.shape, dtype=bool)
    if window_s is not None:
        mask = result.time <= float(window_s)
    if not np.any(mask):
        raise ValueError(f"no {algorithm!r} samples within window_s={window_s}")

    color, alpha = _panel_style(algorithm)
    return _Panel(
        label=ALGORITHM_PANEL_LABELS.get(algorithm, ALGORITHM_LABELS.get(algorithm, algorithm)),
        color=color,
        alpha=alpha,
        pursuer_xy=result.pursuer_position[mask][:, :2],
        target_xy=result.target_position[mask][:, :2],
    )


def _panel_style(algorithm: str) -> tuple[str, float]:
    if algorithm in PANEL_STYLES:
        return PANEL_STYLES[algorithm]
    # 未登记的算法退化为默认配色，保证新增算法时不会画不出来。
    colors = plt.rcParams["axes.prop_cycle"].by_key().get("color", [])
    return (colors[0] if colors else "#1f77b4"), 1.0


def _shared_limits(panels: list[_Panel]) -> tuple[tuple[float, float], tuple[float, float], float, float]:
    """所有面板共用的坐标范围与刻度步长；等比例坐标轴要求两轴范围一起定。"""
    x = np.concatenate([panel.pursuer_xy[:, 0] for panel in panels] + [panel.target_xy[:, 0] for panel in panels])
    y = np.concatenate([panel.pursuer_xy[:, 1] for panel in panels] + [panel.target_xy[:, 1] for panel in panels])
    x_limits, x_step = _rounded_limits(x)
    y_limits, y_step = _rounded_limits(y)
    return x_limits, y_limits, x_step, y_step


def _rounded_limits(values: np.ndarray) -> tuple[tuple[float, float], float]:
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return (-1.0, 1.0), 1.0

    low, high = float(np.min(finite)), float(np.max(finite))
    span = high - low
    # 退化情形（所有点重合）给一个固定视野，避免零尺寸面板。
    pad = PAD_FRACTION * span if span > 0.0 else 1.0
    step = _nice_step(span if span > 0.0 else 2.0)
    # 范围向外取到半个步长的整数倍：既保持“整数感”，又不像取整步长那样留出大片空白。
    unit = step / 2.0
    return (math.floor((low - pad) / unit) * unit, math.ceil((high + pad) / unit) * unit), step


def _nice_step(span: float) -> float:
    raw = span / TARGET_TICKS
    for step in NICE_STEPS:
        if step >= raw:
            return step
    return NICE_STEPS[-1] * math.ceil(raw / NICE_STEPS[-1])


def _ticks(limits: tuple[float, float], step: float) -> np.ndarray:
    # 范围不一定落在步长整数倍上，刻度从范围内第一个整数倍起步（与参考图一致）。
    first = math.ceil(limits[0] / step - 1e-9) * step
    return np.arange(first, limits[1] + step * 0.5, step)


def _place_grid(
    fig: plt.Figure,
    panels: list[_Panel],
    columns: int,
    *,
    box_w: float,
    box_h: float,
    top_in: float,
    x_limits: tuple[float, float],
    y_limits: tuple[float, float],
    x_step: float,
    y_step: float,
) -> None:
    """按英寸排版：面板等尺寸、行列间距固定，figure 尺寸由调用方围绕它反推。"""
    fig_w, fig_h = fig.get_size_inches()
    width, height = box_w / fig_w, box_h / fig_h
    x0 = LEFT_IN / fig_w
    dx = (box_w + GAP_X) / fig_w
    pitch = (box_h + TITLE_H + XLABEL_H + ROW_GAP) / fig_h
    top = top_in / fig_h

    for index, panel in enumerate(panels):
        row, column = divmod(index, columns)
        ax = fig.add_axes([x0 + column * dx, top - row * pitch - height, width, height])
        _draw_panel(ax, panel, x_limits, y_limits, x_step, y_step)


def _draw_panel(
    ax: plt.Axes,
    panel: _Panel,
    x_limits: tuple[float, float],
    y_limits: tuple[float, float],
    x_step: float,
    y_step: float,
) -> None:
    target, = ax.plot(
        panel.target_xy[:, 0],
        panel.target_xy[:, 1],
        ls=TARGET_DASHES,
        lw=1.3,
        color=TARGET_COLOR,
        label="Target",
        zorder=2,
    )
    pursuer, = ax.plot(
        panel.pursuer_xy[:, 0],
        panel.pursuer_xy[:, 1],
        lw=1.8,
        color=panel.color,
        alpha=panel.alpha,
        label="Pursuer",
        zorder=3,
    )

    # 起点标记不进图例：参考图里只用形状区分“从哪出发”。
    ax.plot(panel.target_xy[0, 0], panel.target_xy[0, 1], marker="*", ms=12, color=TARGET_START_COLOR, ls="none", zorder=5)
    ax.plot(
        panel.pursuer_xy[0, 0],
        panel.pursuer_xy[0, 1],
        marker="s",
        ms=6.5,
        color=panel.color,
        alpha=panel.alpha,
        markeredgecolor="white",
        markeredgewidth=0.8,
        ls="none",
        zorder=6,
    )

    ax.set_title(panel.label, fontsize=FS_TITLE, fontweight="bold", pad=9)
    ax.set_xlabel("X (m)", fontsize=FS_LABEL, fontweight="bold")
    ax.set_ylabel("Y (m)", fontsize=FS_LABEL, fontweight="bold")
    ax.set_xlim(*x_limits)
    ax.set_ylim(*y_limits)
    ax.set_xticks(_ticks(x_limits, x_step))
    ax.set_yticks(_ticks(y_limits, y_step))
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, color=GRID_COLOR, lw=0.8)
    ax.set_axisbelow(True)
    ax.tick_params(labelsize=FS_TICK)

    legend = ax.legend(
        handles=[pursuer, target],
        loc="upper left",
        fontsize=FS_LEGEND,
        frameon=True,
        borderpad=0.5,
        labelspacing=0.45,
        handlelength=2.2,
        handletextpad=0.6,
    )
    legend.get_frame().set_edgecolor("#c8c8c8")
    legend.get_frame().set_linewidth(0.9)


def _figure_text_in(fig: plt.Figure, x_in: float, y_in: float, text: str, **kwargs) -> None:
    """用“距左下角的英寸数”定位文字，和面板排版同一套坐标。"""
    fig_w, fig_h = fig.get_size_inches()
    fig.text(x_in / fig_w, y_in / fig_h, text, **kwargs)
