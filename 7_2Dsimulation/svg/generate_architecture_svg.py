"""
Generate a detailed SVG architecture diagram for the PN + NMPC guidance algorithm.
Pure Python — no external dependencies.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable

# ── colour palette ──────────────────────────────────────────────
BG = "#0f172a"
BOX_BG = "#1e293b"
BOX_STROKE = "#334155"
TITLE_BAR_BG = "#111827"
TITLE_TEXT = "#f1f5f9"
BODY_TEXT = "#cbd5e1"
DIM_TEXT = "#94a3b8"
HL_TEXT = "#fbbf24"
ACCENT = "#3b82f6"

PN_BG = "#14532d"
PN_STROKE = "#22c55e"
PN_TITLE_BAR = "#052e16"
PN_TEXT = "#bbf7d0"
PN_FORMULA_BG = "#0a2815"
PN_FORMULA_STROKE = "#4ade80"

NMPC_BG = "#0f1d3d"
NMPC_STROKE = "#3b82f6"
NMPC_TITLE_BAR = "#0b1630"
NMPC_TEXT = "#bfdbfe"

CANDIDATE_BG = "#1e293b"
CANDIDATE_STROKE = "#475569"
CANDIDATE_HL_BG = "#1e3a5f"
CANDIDATE_HL_STROKE = "#60a5fa"

COST_RED = "#ef4444"
COST_ORANGE = "#f97316"
COST_YELLOW = "#eab308"
COST_GRAY = "#a3a3a3"

ARROW_BLUE = "#3b82f6"
ARROW_GRAY = "#64748b"
LOOP_ARROW = "#475569"


@dataclass
class Rect:
    x: float
    y: float
    w: float
    h: float
    rx: float = 6
    fill: str = BOX_BG
    stroke: str = BOX_STROKE
    stroke_width: float = 1.5


@dataclass
class Text:
    x: float
    y: float
    text: str
    fill: str = BODY_TEXT
    size: int = 12
    anchor: str = "start"
    weight: str = "normal"
    family: str = "Consolas, Monaco, 'Courier New', monospace"


# ── SVG generation helpers ──────────────────────────────────────

def _esc(s: str) -> str:
    return str(s).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


class SVGBuilder:
    def __init__(self, w: int, h: int) -> None:
        self.w = w
        self.h = h
        self.elements: list[str] = []
        self._defs: list[str] = []
        self._id_counter = 0

    def _next_id(self, prefix: str = "e") -> str:
        self._id_counter += 1
        return f"{prefix}{self._id_counter}"

    def defs(self) -> None:
        self._defs.append(
            """<marker id="arrow-blue" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto">
    <path d="M0,0 L10,5 L0,10 Z" fill="#3b82f6"/>
  </marker>
  <marker id="arrow-gray" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto">
    <path d="M0,0 L10,5 L0,10 Z" fill="#64748b"/>
  </marker>
  <filter id="glow" x="-30%" y="-30%" width="160%" height="160%">
    <feGaussianBlur stdDeviation="2.5" result="blur"/>
    <feMerge><feMergeNode in="blur"/><feMergeNode in="SourceGraphic"/></feMerge>
  </filter>"""
        )

    def rect(self, r: Rect) -> str:
        self.elements.append(
            f'<rect x="{r.x}" y="{r.y}" width="{r.w}" height="{r.h}" '
            f'rx="{r.rx}" fill="{r.fill}" stroke="{r.stroke}" '
            f'stroke-width="{r.stroke_width}"/>'
        )
        return ""

    def text(self, t: Text) -> str:
        self.elements.append(
            f'<text x="{t.x}" y="{t.y}" fill="{_esc(t.fill)}" font-size="{t.size}" '
            f'font-family="{t.family}" text-anchor="{t.anchor}" '
            f'font-weight="{t.weight}">{_esc(t.text)}</text>'
        )
        return ""

    def line(self, x1: float, y1: float, x2: float, y2: float, color: str = ARROW_GRAY, width: float = 1.5, dash: str = "none", marker: str = "") -> str:
        dash_str = f' stroke-dasharray="{dash}"' if dash != "none" else ""
        marker_str = f' marker-end="url(#{marker})"' if marker else ""
        self.elements.append(
            f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" '
            f'stroke="{color}" stroke-width="{width}"{dash_str}{marker_str}/>'
        )
        return ""

    def path(self, d: str, color: str = ARROW_GRAY, width: float = 1.5, fill: str = "none", dash: str = "none", marker: str = "") -> str:
        dash_str = f' stroke-dasharray="{dash}"' if dash != "none" else ""
        marker_str = f' marker-end="url(#{marker})"' if marker else ""
        self.elements.append(
            f'<path d="{d}" fill="{fill}" stroke="{color}" stroke-width="{width}"{dash_str}{marker_str}/>'
        )
        return ""

    def group_open(self, transform: str = "") -> str:
        t = f' transform="{transform}"' if transform else ""
        self.elements.append(f"<g{t}>")
        return ""

    def group_close(self) -> str:
        self.elements.append("</g>")
        return ""

    def render(self) -> str:
        defs_block = "\n    ".join(self._defs)
        body = "\n    ".join(self.elements)
        return (
            f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {self.w} {self.h}" '
            f'width="{self.w}" height="{self.h}">\n'
            f"  <defs>\n    {defs_block}\n  </defs>\n"
            f'  <rect width="{self.w}" height="{self.h}" fill="{BG}"/>\n'
            f"  {body}\n"
            f"</svg>"
        )

    def box_with_title(
        self,
        x: float,
        y: float,
        w: float,
        h: float,
        title: str,
        fill: str = BOX_BG,
        stroke: str = BOX_STROKE,
        title_bar_fill: str = TITLE_BAR_BG,
        title_text: str = TITLE_TEXT,
        title_size: int = 13,
        stroke_w: float = 1.5,
        top_label: str = "",
    ) -> None:
        self.rect(Rect(x, y, w, h, fill=fill, stroke=stroke, stroke_width=stroke_w))
        # title bar
        bar_h = 26
        self.rect(Rect(x, y, w, bar_h, rx=6, fill=title_bar_fill, stroke=stroke, stroke_width=0))
        # cover bottom corners of title bar so they don't stick out of the rounded rect
        self.rect(Rect(x + 2, y + bar_h - 6, w - 4, 6, rx=0, fill=title_bar_fill, stroke=stroke, stroke_width=0))
        if top_label:
            self.text(Text(x + 8, y + 18, top_label, fill=DIM_TEXT, size=10))
        self.text(Text(x + (10 if top_label else 8), y + 18, title, fill=title_text, size=title_size, weight="bold"))

    def section_label(self, x: float, y: float, text: str, color: str = DIM_TEXT, size: int = 11) -> None:
        self.text(Text(x, y, text, fill=color, size=size, weight="bold", family="Arial, sans-serif"))


# ── build the diagram ───────────────────────────────────────────

CANDIDATES = [
    ("pn_trend", "PN趋势基线 ①"),
    ("0.55 × pn_trend", "缩小的PN ②"),
    ("1.25 × pn_trend", "放大的PN ③"),
    ("0.75×PN + 0.25×intercept", "拦截混合A ④"),
    ("0.50×PN + 0.50×intercept", "拦截混合B ⑤"),
    ("same_speed", "等速靠近 ⑥"),
    ("0.50×PN + 0.50×same_speed", "PN+等速 ⑦"),
    ("velocity_match", "速度匹配 ⑧"),
    ("0.50×PN + 0.50×vel_match", "PN+速度匹配 ⑨"),
    ("stable_tracking ★", "前馈+Kp+Kd ⑩"),
    ("soft_tracking", "软跟踪 ⑪"),
    ("velocity_tracking", "速度跟踪 ⑫"),
    ("0.50×PN + 0.50×stable", "PN+稳定 ⑬"),
    ("0.35×PN + 0.65×soft", "PN+软跟踪 ⑭"),
    ("PN + 0.35·a_max·lateral", "右横向探测 ⑮"),
    ("PN − 0.35·a_max·lateral", "左横向探测 ⑯"),
]

COST_ITEMS = [
    ("final_distance", 12.00, COST_RED),
    ("final_vel_error", 5.40, COST_ORANGE),
    ("steady_cost", 2.16, COST_YELLOW),
    ("path_cost", 0.10, COST_GRAY),
    ("smooth_cost", 0.08, COST_GRAY),
    ("pn_cost (偏离PN趋势)", 0.04, "#a0a0a0"),
    ("velocity_cost", 0.035, "#909090"),
    ("control_cost |‖acc‖²", 0.015, "#808080"),
]


def build_svg() -> str:
    W, H = 900, 1080
    s = SVGBuilder(W, H)
    s.defs()

    # ── Title ───────────────────────────────────────────────────
    s.text(Text(W / 2, 30, "PN + NMPC Guidance Architecture", fill=TITLE_TEXT, size=20, weight="bold", anchor="middle", family="Arial, sans-serif"))
    s.text(Text(W / 2, 50, "2D Anti-Drone Pursuit  —  定高俯瞰 比例导引 + 非线性模型预测控制", fill=DIM_TEXT, size=12, anchor="middle", family="Arial, sans-serif"))

    # ── Simulation Loop bar (y=65) ──────────────────────────────
    bar_x, bar_w = 40, W - 80
    s.rect(Rect(bar_x, 70, bar_w, 36, fill="#1e293b", stroke="#334155"))
    s.text(Text(bar_x + 14, 92, "simulation.py: run_algorithm(scenario, \"pn_nmpc\", config)    dt=0.05s    sim_time=40s", fill=DIM_TEXT, size=11))

    # ── Three input boxes (y=125) ───────────────────────────────
    in_y = 125
    in_h = 92
    box_gap = 12
    pw, tw, mw = 285, 265, 210
    px = bar_x
    tx = px + pw + box_gap
    mx = tx + tw + box_gap

    # PursuerState
    s.box_with_title(px, in_y, pw, in_h, "PursuerState", fill=BOX_BG, stroke=BOX_STROKE)
    s.text(Text(px + 10, in_y + 44, "position:  np.ndarray(3)  [x, y, z=8]", fill=BODY_TEXT, size=11))
    s.text(Text(px + 10, in_y + 62, "velocity:  np.ndarray(3)  [vx, vy, 0]", fill=BODY_TEXT, size=11))
    s.text(Text(px + 10, in_y + 80, "yaw:       float", fill=BODY_TEXT, size=11))
    s.text(Text(px + 10, in_y + 98, "accel:     np.ndarray(3)", fill=BODY_TEXT, size=11))

    # TargetState
    s.box_with_title(tx, in_y, tw, in_h, "TargetState", fill=BOX_BG, stroke=BOX_STROKE)
    s.text(Text(tx + 10, in_y + 44, "position:     np.ndarray(3)  [x, y, z=1]", fill=BODY_TEXT, size=11))
    s.text(Text(tx + 10, in_y + 62, "velocity:     np.ndarray(3)", fill=BODY_TEXT, size=11))
    s.text(Text(tx + 10, in_y + 80, "acceleration: np.ndarray(3)", fill=BODY_TEXT, size=11))

    # GuidanceMemory
    s.box_with_title(mx, in_y, mw, in_h, "GuidanceMemory", fill=BOX_BG, stroke="#475569")
    s.text(Text(mx + 10, in_y + 44, "previous_accel: ndarray(3)", fill=BODY_TEXT, size=11))
    s.text(Text(mx + 10, in_y + 62, "  ↑ 上一步实际加速度", fill=DIM_TEXT, size=10))
    s.text(Text(mx + 10, in_y + 85, "用途: 平滑代价项 ‖a−a_prev‖²", fill="#64748b", size=10))

    # ── compute_guidance dispatch (y=240) ───────────────────────
    disp_y = 240
    disp_h = 48
    s.box_with_title(bar_x, disp_y, bar_w, disp_h, "compute_guidance(algorithm=\"pn_nmpc\", pursuer, target, memory, config, dt)", fill=BOX_BG, stroke=BOX_STROKE, title_size=11)

    # Lines from input boxes to dispatch
    centers = [
        (px + pw / 2, in_y + in_h),
        (tx + tw / 2, in_y + in_h),
        (mx + mw / 2, in_y + in_h),
    ]
    disp_ctr = (bar_x + bar_w / 2, disp_y)
    for cx, cy in centers:
        s.line(cx, cy, disp_ctr[0], disp_ctr[1], color=ARROW_GRAY, width=1.2, dash="5,4", marker="arrow-gray")

    # ── Two arrows from dispatch to PN block ────────────────────
    pn_block_y = 315
    pn_block_h = 200
    pn_right_x = bar_x + bar_w
    pn_center_x = bar_x + bar_w / 2

    # Arrow: dispatch → PN
    s.line(pn_center_x, disp_y + disp_h, pn_center_x, pn_block_y, color=ACCENT, width=2.5, marker="arrow-blue")
    s.text(Text(pn_center_x + 10, disp_y + disp_h + 40, "pn_trend = pn_guidance()", fill=ACCENT, size=10))

    # ── Step 1: PN Guidance (y=315, h=200) ──────────────────────
    pn_y = pn_block_y
    s.box_with_title(bar_x - 5, pn_y, bar_w + 10, pn_block_h, "", fill=PN_BG, stroke=PN_STROKE, stroke_w=2, title_bar_fill=PN_TITLE_BAR, title_text=PN_TEXT, top_label="Step 1")
    s.text(Text(bar_x + 8, pn_y + 20, "pn_guidance(pursuer, target, config)", fill=PN_TEXT, size=13, weight="bold"))

    # Formulas inside PN
    fy = pn_y + 42
    s.text(Text(bar_x + 14, fy, "r = target.position − pursuer.position     ← 视线向量", fill=PN_TEXT, size=11))
    s.text(Text(bar_x + 14, fy + 20, "u_LOS = normalize(r)                      ← 单位视线方向", fill=PN_TEXT, size=11))
    s.text(Text(bar_x + 14, fy + 40, "v_rel = target.velocity − pursuer.velocity                          closing_speed = max(0, −v_rel·u_LOS)", fill=PN_TEXT, size=11))

    # Highlighted formula box: omega
    fo_y = fy + 58
    s.rect(Rect(bar_x + 10, fo_y, 380, 52, rx=4, fill=PN_FORMULA_BG, stroke=PN_FORMULA_STROKE, stroke_width=1.5))
    s.text(Text(bar_x + 20, fo_y + 20, "ω_LOS = (r × v_rel)_z  /  |r|²", fill="#fbbf24", size=14, weight="bold"))
    s.text(Text(bar_x + 20, fo_y + 38, "2D 视线角速率 (标量):  (r_x · v_y − r_y · v_x) / r²", fill="#86efac", size=10))

    # a_pn formula
    apn_y = fo_y + 66
    s.rect(Rect(bar_x + 10, apn_y, 380, 32, rx=4, fill=PN_FORMULA_BG, stroke="#fbbf24", stroke_width=1.5))
    s.text(Text(bar_x + 20, apn_y + 21, "a_pn = N · closing_speed · ω_LOS · lateral    N=3.5    lateral=[−u_y, u_x, 0]", fill="#fbbf24", size=11, weight="bold"))

    # a_close
    s.text(Text(bar_x + 14, apn_y + 48, "a_close = k_close · (v_des − v_along_los) · u_LOS    k_close=1.0, v_des=8.0", fill=PN_TEXT, size=11))

    # pn_trend output box
    out_y = apn_y + 62
    s.rect(Rect(bar_x + 10, out_y, 380, 28, rx=4, fill=PN_FORMULA_BG, stroke="#fbbf24", stroke_width=1.5))
    s.text(Text(bar_x + 20, out_y + 19, "pn_trend = clamp(a_pn + a_close, a_max=6.0)   ← PN基线趋势", fill="#fbbf24", size=11, weight="bold"))

    # Role annotation right side of PN block
    s.text(Text(bar_x + 410, pn_y + 60, "► PN 提供拦截方向\"锚点\"", fill=PN_TEXT, size=11, family="Arial, sans-serif"))
    s.text(Text(bar_x + 410, pn_y + 82, "   NMPC 将在该锚点附近", fill=PN_TEXT, size=11, family="Arial, sans-serif"))
    s.text(Text(bar_x + 410, pn_y + 104, "   搜索最优加速度指令", fill=PN_TEXT, size=11, family="Arial, sans-serif"))
    s.text(Text(bar_x + 410, pn_y + 132, "► 避免纯优化发散或", fill=PN_TEXT, size=11, family="Arial, sans-serif"))
    s.text(Text(bar_x + 410, pn_y + 154, "   产生不合理机动", fill=PN_TEXT, size=11, family="Arial, sans-serif"))

    # Arrow: PN → NMPC
    nmpc_y = 540
    s.line(pn_center_x, pn_y + pn_block_h, pn_center_x, nmpc_y, color=ACCENT, width=2.5, marker="arrow-blue")
    s.text(Text(pn_center_x + 10, pn_y + pn_block_h + 18, "pn_trend + memory", fill=ACCENT, size=10))

    # ── Step 2: NMPC Acceleration (y=540, h=520) ────────────────
    nmpc_h = 520
    s.box_with_title(bar_x - 5, nmpc_y, bar_w + 10, nmpc_h, "", fill=NMPC_BG, stroke=NMPC_STROKE, stroke_w=2, title_bar_fill=NMPC_TITLE_BAR, title_text=NMPC_TEXT, top_label="Step 2")
    s.text(Text(bar_x + 8, nmpc_y + 20, "nmpc_acceleration(pursuer, target, pn_trend, memory, config)", fill=NMPC_TEXT, size=13, weight="bold"))

    # ── 2a: Candidate grid ──────────────────────────────────────
    cand_title_y = nmpc_y + 44
    s.text(Text(bar_x + 14, cand_title_y, "2a. _candidate_accelerations()    →    16 个候选加速度  (全部 clamp 到 a_max=6.0)", fill=NMPC_TEXT, size=12, weight="bold"))

    grid_x = bar_x + 10
    grid_y = cand_title_y + 12
    cell_w = 200
    cell_h = 25
    cell_gap_x = 8
    cell_gap_y = 4
    cols = 4

    for i, (formula, label) in enumerate(CANDIDATES):
        col = i % cols
        row = i // cols
        cx = grid_x + col * (cell_w + cell_gap_x)
        cy = grid_y + row * (cell_h + cell_gap_y)

        is_star = i == 9  # stable_tracking highlight
        cell_fill = CANDIDATE_HL_BG if is_star else CANDIDATE_BG
        cell_stroke = CANDIDATE_HL_STROKE if is_star else CANDIDATE_STROKE
        cell_sw = 2 if is_star else 1

        s.rect(Rect(cx, cy, cell_w, cell_h, rx=3, fill=cell_fill, stroke=cell_stroke, stroke_width=cell_sw))
        if is_star:
            s.rect(Rect(cx, cy, cell_w, cell_h, rx=3, fill="none", stroke=CANDIDATE_HL_STROKE, stroke_width=2))
            s.elements.append(f'<rect x="{cx}" y="{cy}" width="{cell_w}" height="{cell_h}" rx="3" fill="none" stroke="#fbbf24" stroke-width="1.5" filter="url(#glow)"/>')

        text_color = "#e0f2fe" if is_star else BODY_TEXT
        s.text(Text(cx + 6, cy + 16, formula, fill=text_color, size=9, weight="bold" if is_star else "normal"))

    # Category labels on the right
    cat_labels_x = grid_x + cols * (cell_w + cell_gap_x) + 14
    cat_labels_y = grid_y + cell_h
    categories = [
        ("PN 缩放类", "#22c55e"),
        ("拦截混合类", "#a78bfa"),
        ("速度匹配类", "#38bdf8"),
        ("★ 末端跟踪类", "#fbbf24"),
        ("横向探测类", "#fb923c"),
    ]
    for i, (cat_label, cat_color) in enumerate(categories):
        ly = cat_labels_y + i * (4 * cell_h + 4 * cell_gap_y) // 2
        s.text(Text(cat_labels_x, ly, cat_label, fill=cat_color, size=10, weight="bold", family="Arial, sans-serif"))

    # ── 2b: Rollout cost ────────────────────────────────────────
    roll_y = grid_y + 4 * (cell_h + cell_gap_y) + 16
    s.text(Text(bar_x + 14, roll_y, "2b. _rollout_cost()    H = 20 steps × mpc_dt=0.1s = 2s 前向预测", fill=NMPC_TEXT, size=12, weight="bold"))
    s.text(Text(bar_x + 14, roll_y + 18, "目标: 恒加速度外推  pos_pred = pos + vel·t + 0.5·acc·t²    pursuer: step_pursuer() 动力学积分", fill=DIM_TEXT, size=10))

    # Cost bars
    bar_start_y = roll_y + 36
    bar_h = 16
    bar_gap = 6
    bar_max_w = 340
    bar_x_start = bar_x + 14
    label_x_offset = bar_max_w + 18

    max_weight = max(w for _, w, _ in COST_ITEMS)

    for i, (name, weight, color) in enumerate(COST_ITEMS):
        by = bar_start_y + i * (bar_h + bar_gap)
        bar_w = (weight / max_weight) * bar_max_w

        s.rect(Rect(bar_x_start, by, bar_w, bar_h, rx=3, fill=color, stroke=color, stroke_width=0))

        # label
        s.text(Text(bar_x_start + bar_w + 6, by + 12, name, fill=BODY_TEXT, size=10))

        # weight value
        s.text(Text(bar_x_start + bar_w + label_x_offset + 40, by + 12, f"×{weight:.3f}", fill=color, size=10, weight="bold"))

    # 8 items explanation
    expl_y = bar_start_y + len(COST_ITEMS) * (bar_h + bar_gap) + 14
    s.text(Text(bar_x_start, expl_y, "Cost = Σ 8项加权和 → 选 min(cost) 对应的候选加速度", fill=NMPC_TEXT, size=11))

    # Decompose explanation for steady_cost, etc
    detail_y = expl_y + 22
    details = [
        "steady_cost: 后半段(distance² + 0.35·|v_rel|²) → 防止末段超调振荡",
        "smooth_cost: ‖acc − previous_acc‖²             → 抑制加速度突变",
        "pn_cost:     ‖acc − pn_trend‖²                  → 不宜过多偏离PN锚点",
    ]
    for i, d in enumerate(details):
        s.text(Text(bar_x_start + 10, detail_y + i * 16, d, fill=DIM_TEXT, size=10))

    # ── 2c: Best selection ──────────────────────────────────────
    best_y = detail_y + 3 * 16 + 12
    s.rect(Rect(bar_x + 10, best_y, bar_w - 20, 38, rx=4, fill="#1e3a5f", stroke="#60a5fa", stroke_width=1.5))
    s.text(Text(bar_x + 20, best_y + 24, "2c. best = argmin(cost)  among 16    →    clamp_norm_xy(best, a_max=6.0)    →    返回 GuidanceResult", fill="#e0f2fe", size=12, weight="bold"))

    # ── Arrow from NMPC output to Dynamics ──────────────────────
    dyn_y = nmpc_y + nmpc_h + 4
    s.line(pn_center_x, best_y + 38, pn_center_x, nmpc_y + nmpc_h, color=ACCENT, width=2, marker="arrow-blue")
    s.line(pn_center_x, nmpc_y + nmpc_h, pn_center_x, dyn_y, color=ACCENT, width=2.5, marker="arrow-blue")
    s.text(Text(pn_center_x + 10, nmpc_y + nmpc_h - 6, "acceleration", fill=ACCENT, size=10))

    # ── Dynamics / Execution bar ─────────────────────────────────
    dyn_h = 50
    s.rect(Rect(bar_x, dyn_y, bar_w, dyn_h, rx=6, fill="#1e293b", stroke="#334155"))
    s.text(Text(bar_x + 14, dyn_y + 18, "step_pursuer(state, acc, target_pos, config, dt=0.05)", fill=TITLE_TEXT, size=12, weight="bold"))
    s.text(Text(bar_x + 14, dyn_y + 38, "vel += acc·dt  (clamp v_max=12)    pos += vel·dt  (z locked=8)    yaw → look_at target    memory.prev_acc = new_acc", fill=DIM_TEXT, size=10))

    # ── Loop back arrow ─────────────────────────────────────────
    loop_mid_y = dyn_y + dyn_h + 20
    s.path(
        f"M {pn_center_x + 160} {dyn_y + dyn_h} "
        f"L {pn_center_x + 160} {loop_mid_y} "
        f"L {bar_x + bar_w + 10} {loop_mid_y} "
        f"L {bar_x + bar_w + 10} {70 + 18}",
        color=LOOP_ARROW, width=1.5, dash="6,4", marker="arrow-gray",
    )
    s.text(Text(pn_center_x + 170, loop_mid_y - 4, "t += dt  ↻ 下一帧", fill=DIM_TEXT, size=10))

    return s.render()


if __name__ == "__main__":
    svg_content = build_svg()
    output_path = "architecture_pn_nmpc.svg"
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(svg_content)
    print(f"SVG written to {output_path}  ({len(svg_content)} bytes)")
