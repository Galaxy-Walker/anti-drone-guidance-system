"""视觉目标 α-β 估计器与丢失状态机（纯算法，不依赖 ROS）。

输入是下视相机反投影得到的 XY 位置量测（见 `vision_adapter` 的 `/vision/target_pose`），
输出是供导引算法使用的目标状态估计：

$$\\hat p_k^-=\\hat p_{k-1}+\\hat v_{k-1}\\Delta t,\\qquad
\\hat p_k=\\hat p_k^-+\\alpha\\,(z_k-\\hat p_k^-),\\qquad
\\hat v_k=\\hat v_{k-1}+\\frac{\\beta}{\\Delta t}(z_k-\\hat p_k^-)$$

- 位置和速度只估计 XY；z 固定为目标平面高度，不参与滤波。
- $\\Delta t$ 用量测 stamp 差（仿真时间）并夹在 `[min_dt_s, max_dt_s]`，避免丢帧后增益爆炸。
- 加速度由速度一阶低通差分给出，供 `pn_mppi`/`pn_nmpc` 的目标预测使用，不参与位置外推。
- 偶发离群量测用马氏门控拒绝（`gate_sigma`，0 关闭），拒绝时保持预测状态。
- 丢失状态机：`tracking`（有新量测）→ `coast`（超过 `coast_s`）→ `lost`（超过 `loss_s`），
  重新收到量测后立即回到 `tracking`。

模块只依赖 numpy 和标准库，可直接用 `uv run python tests/test_target_filter.py` 离线测试。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

# 丢失状态机的三个状态名；节点据此决定是否进入悬停（hold）。
STATE_TRACKING = "tracking"
STATE_COAST = "coast"
STATE_LOST = "lost"

# 门控协方差用的过程噪声强度（m/s^2）。它不是控制算法参数，只影响马氏距离的尺度：
# 假设目标机动不超过该量级，避免把正常机动误判成离群。
PROCESS_ACCEL_STD = 3.0

# 没有任何量测时输出的位置/速度用 NaN 表示“未知”，而不是零。
_UNKNOWN_XY = np.full(2, np.nan, dtype=float)


@dataclass(slots=True)
class TargetFilterConfig:
    alpha: float = 0.85
    beta: float = 0.25
    # 加速度低通时间常数；<=0 时关闭前馈，加速度恒为 0。
    accel_tau_s: float = 0.5
    # 马氏门控阈值（sigma）；<=0 关闭门控。
    gate_sigma: float = 0.0
    coast_s: float = 0.3
    loss_s: float = 1.0
    min_dt_s: float = 0.01
    max_dt_s: float = 0.5
    # 目标平面高度，仅用于补全估计状态的 z 分量。
    plane_z: float = 1.0
    # 量测协方差缺省值（m^2），仅在调用方没有给出协方差、且需要门控时使用。
    default_position_sigma_m: float = 0.2


@dataclass(slots=True)
class TargetEstimate:
    """一次 `predict()` 给出的估计快照；未初始化时 XY 为 NaN。"""

    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    state: str
    age_s: float
    measurements: int
    position_covariance_xy: np.ndarray
    initialized: bool


class VisionTargetTracker:
    """α-β 目标估计器；`update()` 吸收量测，`predict()` 外推到当前时刻。"""

    def __init__(self, config: TargetFilterConfig | None = None) -> None:
        self._config = config if config is not None else TargetFilterConfig()
        self.reset()

    @property
    def config(self) -> TargetFilterConfig:
        return self._config

    def reset(self) -> None:
        """清空状态；下一次 `update()` 会重新初始化，丢失计时重新开始。"""
        self._initialized = False
        self._position_xy = np.zeros(2, dtype=float)
        self._velocity_xy = np.zeros(2, dtype=float)
        self._acceleration_xy = np.zeros(2, dtype=float)
        # 状态向量顺序 [x, y, vx, vy]，只用于门控协方差递推。
        self._covariance = np.zeros((4, 4), dtype=float)
        self._last_time_s: float | None = None
        self._last_measurement_s: float | None = None
        self._measurements = 0
        self._last_rejection = ""

    @property
    def initialized(self) -> bool:
        return self._initialized

    @property
    def measurements(self) -> int:
        return self._measurements

    @property
    def last_measurement_s(self) -> float | None:
        return self._last_measurement_s

    @property
    def position_xy(self) -> np.ndarray:
        return self._position_xy.copy()

    @property
    def velocity_xy(self) -> np.ndarray:
        return self._velocity_xy.copy()

    @property
    def last_rejection(self) -> str:
        return self._last_rejection

    def update(
        self,
        stamp_s: float,
        position_xy: np.ndarray,
        covariance_xy: np.ndarray | None = None,
    ) -> bool:
        """吸收一次位置量测；返回是否接受。

        重复 stamp、时间回跳、非有限输入和马氏门控拒绝都会返回 `False` 并保持状态
        （只在门控拒绝时保留“已经外推到该 stamp”的预测）。拒绝原因见 `last_rejection`。
        """
        if not math.isfinite(stamp_s):
            self._last_rejection = "invalid_stamp"
            return False

        measurement = np.asarray(position_xy, dtype=float)
        if measurement.shape != (2,) or not np.all(np.isfinite(measurement)):
            self._last_rejection = "invalid_measurement"
            return False

        if not self._initialized:
            return self._initialize(stamp_s, measurement, covariance_xy)

        assert self._last_measurement_s is not None
        if stamp_s <= self._last_measurement_s:
            # 同一帧重复投递或乱序旧帧：不能当成新量测，也不能更新丢失计时。
            self._last_rejection = "duplicate_or_backward_stamp"
            return False

        dt_true = stamp_s - self._last_measurement_s
        # 先外推到量测时刻；丢帧间隔很大时只推进 max_dt_s，剩余空窗交给状态机。
        self._propagate(min(dt_true, self._config.max_dt_s))

        residual = measurement - self._position_xy
        if not self._passes_gate(residual, covariance_xy):
            self._last_rejection = "gate_rejected"
            return False

        dt_gain = min(max(dt_true, self._config.min_dt_s), self._config.max_dt_s)
        previous_velocity = self._velocity_xy.copy()
        self._position_xy = self._position_xy + self._config.alpha * residual
        self._velocity_xy = self._velocity_xy + (self._config.beta / dt_gain) * residual
        self._update_acceleration(previous_velocity, dt_true)
        self._apply_covariance_update(dt_gain, covariance_xy)
        self._last_measurement_s = stamp_s
        self._last_rejection = ""
        self._measurements += 1
        return True

    def predict(self, now_s: float) -> TargetEstimate:
        """把估计外推到 `now_s` 并更新丢失状态。"""
        if not self._initialized:
            return TargetEstimate(
                position=np.array([math.nan, math.nan, self._config.plane_z]),
                velocity=np.array([math.nan, math.nan, 0.0]),
                acceleration=np.array([math.nan, math.nan, 0.0]),
                state=STATE_LOST,
                age_s=math.inf,
                measurements=0,
                position_covariance_xy=np.full((2, 2), math.nan),
                initialized=False,
            )

        assert self._last_time_s is not None and self._last_measurement_s is not None
        if math.isfinite(now_s) and now_s > self._last_time_s:
            self._propagate(now_s - self._last_time_s)

        age_s = max(0.0, float(now_s) - self._last_measurement_s) if math.isfinite(now_s) else math.inf
        return TargetEstimate(
            position=np.array([self._position_xy[0], self._position_xy[1], self._config.plane_z]),
            velocity=np.array([self._velocity_xy[0], self._velocity_xy[1], 0.0]),
            acceleration=np.array([self._acceleration_xy[0], self._acceleration_xy[1], 0.0]),
            state=self._state_for_age(age_s),
            age_s=age_s,
            measurements=self._measurements,
            position_covariance_xy=self._position_covariance(),
            initialized=True,
        )

    def _initialize(
        self,
        stamp_s: float,
        measurement: np.ndarray,
        covariance_xy: np.ndarray | None,
    ) -> bool:
        covariance = self._measurement_covariance(covariance_xy)
        self._position_xy = measurement.copy()
        self._velocity_xy = np.zeros(2, dtype=float)
        self._acceleration_xy = np.zeros(2, dtype=float)
        self._covariance = np.zeros((4, 4), dtype=float)
        self._covariance[0, 0] = covariance[0, 0]
        self._covariance[0, 1] = covariance[0, 1]
        self._covariance[1, 0] = covariance[1, 0]
        self._covariance[1, 1] = covariance[1, 1]
        self._covariance[2, 2] = (PROCESS_ACCEL_STD * self._config.min_dt_s) ** 2
        self._covariance[3, 3] = self._covariance[2, 2]
        self._last_time_s = stamp_s
        self._last_measurement_s = stamp_s
        self._measurements = 1
        self._initialized = True
        self._last_rejection = ""
        return True

    def _state_for_age(self, age_s: float) -> str:
        if age_s <= self._config.coast_s:
            return STATE_TRACKING
        if age_s <= self._config.loss_s:
            return STATE_COAST
        return STATE_LOST

    def _propagate(self, dt_s: float) -> None:
        if not math.isfinite(dt_s) or dt_s <= 0.0:
            return
        self._position_xy = self._position_xy + self._velocity_xy * dt_s
        transition = np.eye(4)
        transition[0, 2] = dt_s
        transition[1, 3] = dt_s
        process_std = PROCESS_ACCEL_STD * dt_s * dt_s
        process = np.diag([process_std ** 2] * 2 + [(PROCESS_ACCEL_STD * dt_s) ** 2] * 2)
        self._covariance = transition @ self._covariance @ transition.T + process
        assert self._last_time_s is not None
        self._last_time_s += dt_s

    def _passes_gate(self, residual: np.ndarray, covariance_xy: np.ndarray | None) -> bool:
        if self._config.gate_sigma <= 0.0:
            return True

        measurement_covariance = self._measurement_covariance(covariance_xy)
        innovation_covariance = self._covariance[np.ix_([0, 1], [0, 1])] + measurement_covariance
        try:
            mahalanobis_squared = float(residual @ np.linalg.inv(innovation_covariance) @ residual)
        except np.linalg.LinAlgError:
            return True
        if not math.isfinite(mahalanobis_squared):
            return True
        return mahalanobis_squared <= self._config.gate_sigma ** 2

    def _measurement_covariance(self, covariance_xy: np.ndarray | None) -> np.ndarray:
        if covariance_xy is None:
            variance = self._config.default_position_sigma_m ** 2
            return np.diag([variance, variance])
        value = np.asarray(covariance_xy, dtype=float)
        if value.shape != (2, 2) or not np.all(np.isfinite(value)):
            variance = self._config.default_position_sigma_m ** 2
            return np.diag([variance, variance])
        # 协方差必须正定；数值噪声导致的轻微不对称在这里对称化。
        return 0.5 * (value + value.T)

    def _update_acceleration(self, previous_velocity: np.ndarray, dt_s: float) -> None:
        tau = self._config.accel_tau_s
        if tau <= 0.0 or not math.isfinite(dt_s) or dt_s <= 0.0:
            self._acceleration_xy = np.zeros(2, dtype=float)
            return
        # 一阶低通：dt 越大越信任新的差分速度；tau 是时间常数。
        blend = 1.0 - math.exp(-min(dt_s, self._config.max_dt_s) / tau)
        acceleration = (self._velocity_xy - previous_velocity) / dt_s
        self._acceleration_xy = self._acceleration_xy + blend * (acceleration - self._acceleration_xy)

    def _apply_covariance_update(self, dt_gain_s: float, covariance_xy: np.ndarray | None) -> None:
        """用 α-β 增益的 Joseph 形式递推协方差，供下一次门控使用。"""
        gain = np.zeros((4, 2), dtype=float)
        gain[0, 0] = self._config.alpha
        gain[1, 1] = self._config.alpha
        gain[2, 0] = self._config.beta / dt_gain_s
        gain[3, 1] = self._config.beta / dt_gain_s
        observation = np.zeros((2, 4), dtype=float)
        observation[0, 0] = 1.0
        observation[1, 1] = 1.0
        measurement_covariance = self._measurement_covariance(covariance_xy)

        identity = np.eye(4)
        residual_map = identity - gain @ observation
        self._covariance = (
            residual_map @ self._covariance @ residual_map.T
            + gain @ measurement_covariance @ gain.T
        )
        self._covariance = 0.5 * (self._covariance + self._covariance.T)

    def _position_covariance(self) -> np.ndarray:
        return self._covariance[np.ix_([0, 1], [0, 1])].copy()
