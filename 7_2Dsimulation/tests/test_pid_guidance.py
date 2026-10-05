"""PID 追踪导引的离线单元测试。

覆盖 `guidance.py` 中单环位置 PID 的 P/I/D 三项、积分抗饱和和 `compute_guidance`
的调度与限幅接线，并在静止场景做一次端到端捕获冒烟测试。

运行：

```bash
cd 7_2Dsimulation
uv run python tests/test_pid_guidance.py
```
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from pythonsimulation2d.config import SimulationConfig  # noqa: E402
from pythonsimulation2d.guidance import GuidanceMemory, compute_guidance, pid_guidance  # noqa: E402
from pythonsimulation2d.math_utils import norm_xy  # noqa: E402
from pythonsimulation2d.simulation import run_algorithm  # noqa: E402
from pythonsimulation2d.state import PursuerState, TargetState  # noqa: E402


def make_pursuer(
    position: tuple[float, float] = (0.0, 0.0),
    velocity: tuple[float, float] = (0.0, 0.0),
) -> PursuerState:
    return PursuerState(
        position=np.array([position[0], position[1], 8.0]),
        velocity=np.array([velocity[0], velocity[1], 0.0]),
        acceleration=np.zeros(3),
        yaw=0.0,
    )


def make_target(
    position: tuple[float, float] = (0.0, 0.0),
    velocity: tuple[float, float] = (0.0, 0.0),
) -> TargetState:
    return TargetState(
        position=np.array([position[0], position[1], 1.0]),
        velocity=np.array([velocity[0], velocity[1], 0.0]),
        acceleration=np.zeros(3),
    )


class PidTermsTest(unittest.TestCase):
    """P/I/D 三项的符号、数值与积分抗饱和。"""

    def test_zero_error_returns_zero(self) -> None:
        config = SimulationConfig()
        pursuer = make_pursuer(position=(3.0, 4.0), velocity=(1.0, 0.0))
        target = make_target(position=(3.0, 4.0), velocity=(1.0, 0.0))
        acceleration = pid_guidance(pursuer, target, GuidanceMemory(), config, config.dt)
        np.testing.assert_allclose(acceleration, np.zeros(3), atol=1e-12)

    def test_proportional_term_drives_toward_target(self) -> None:
        config = SimulationConfig()
        config.guidance.pid_kp = 1.0
        config.guidance.pid_ki = 0.0
        config.guidance.pid_kd = 0.0
        acceleration = pid_guidance(make_pursuer(), make_target(position=(3.0, -2.0)), GuidanceMemory(), config, config.dt)
        np.testing.assert_allclose(acceleration, np.array([3.0, -2.0, 0.0]), atol=1e-12)

    def test_derivative_term_damps_closing_motion(self) -> None:
        # 位置误差为零、追踪机正朝目标飞行：D 项应产生反向制动脉冲。
        config = SimulationConfig()
        config.guidance.pid_kp = 0.0
        config.guidance.pid_ki = 0.0
        config.guidance.pid_kd = 2.0
        pursuer = make_pursuer(velocity=(1.5, 0.0))
        target = make_target(position=(0.0, 0.0), velocity=(0.0, 0.0))
        acceleration = pid_guidance(pursuer, target, GuidanceMemory(), config, config.dt)
        np.testing.assert_allclose(acceleration, np.array([-3.0, 0.0, 0.0]), atol=1e-12)

    def test_integral_accumulates_with_dt(self) -> None:
        config = SimulationConfig()
        config.guidance.pid_kp = 0.0
        config.guidance.pid_ki = 1.0
        config.guidance.pid_kd = 0.0
        memory = GuidanceMemory()
        acceleration = pid_guidance(make_pursuer(), make_target(position=(10.0, 0.0)), memory, config, dt=0.05)
        np.testing.assert_allclose(acceleration, np.array([0.5, 0.0, 0.0]), atol=1e-12)
        np.testing.assert_allclose(memory.pid_integral, np.array([0.5, 0.0, 0.0]), atol=1e-12)

    def test_integral_is_clamped_to_limit(self) -> None:
        config = SimulationConfig()
        config.guidance.pid_integral_limit = 2.0
        memory = GuidanceMemory()
        target = make_target(position=(100.0, 0.0))
        pursuer = make_pursuer()
        for _ in range(500):
            pid_guidance(pursuer, target, memory, config, config.dt)
        self.assertLessEqual(norm_xy(memory.pid_integral), 2.0 + 1e-9)
        self.assertGreater(norm_xy(memory.pid_integral), 1.9)

    def test_dispatch_clamps_to_acceleration_limit_and_looks_at_target(self) -> None:
        config = SimulationConfig()
        pursuer = make_pursuer()
        target = make_target(position=(500.0, 0.0))
        guidance = compute_guidance("pid", pursuer, target, GuidanceMemory(), config, config.dt)
        self.assertAlmostEqual(norm_xy(guidance.acceleration), config.pursuer.a_max, places=9)
        np.testing.assert_allclose(guidance.look_at_position, target.position, atol=0.0)


class PidSimulationSmokeTest(unittest.TestCase):
    """端到端：默认参数下 PID 能在静止场景完成捕获。"""

    def test_captures_stationary_target(self) -> None:
        config = SimulationConfig(sim_time=15.0)
        result = run_algorithm("stationary", "pid", config)
        self.assertLess(float(np.min(result.distance)), config.capture_radius)


if __name__ == "__main__":
    unittest.main()
