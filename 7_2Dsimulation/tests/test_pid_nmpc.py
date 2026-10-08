"""PID + EMPC 混合导引的离线单元测试。

覆盖 `guidance.py` 中 `pid_nmpc` 的接线：PID 每个控制周期只累积一次积分、
`nmpc_w_pn` 作为“偏离 PID 参考”权重（权重极大时退化为纯 PID）、
调度限幅以及静止场景的端到端捕获冒烟测试。

运行：

```bash
cd 7_2Dsimulation
uv run python tests/test_pid_nmpc.py
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


class PidNmpcWiringTest(unittest.TestCase):
    """PID 参考与 EMPC 候选选择的接线。"""

    def test_dispatch_clamps_to_acceleration_limit_and_looks_at_target(self) -> None:
        config = SimulationConfig()
        pursuer = make_pursuer()
        target = make_target(position=(500.0, 0.0))
        guidance = compute_guidance("pid_nmpc", pursuer, target, GuidanceMemory(), config, config.dt)
        self.assertAlmostEqual(norm_xy(guidance.acceleration), config.pursuer.a_max, places=9)
        np.testing.assert_allclose(guidance.look_at_position, target.position, atol=0.0)

    def test_integral_updates_once_per_control_cycle(self) -> None:
        # pid_nmpc 内部只调用一次 pid_guidance；若重复调用，积分会翻倍。
        config = SimulationConfig()
        config.guidance.pid_kp = 0.0
        config.guidance.pid_ki = 1.0
        config.guidance.pid_kd = 0.0
        memory = GuidanceMemory()
        compute_guidance("pid_nmpc", make_pursuer(), make_target(position=(10.0, 0.0)), memory, config, dt=0.05)
        np.testing.assert_allclose(memory.pid_integral, np.array([0.5, 0.0, 0.0]), atol=1e-12)

    def test_large_reference_weight_degenerates_to_pid(self) -> None:
        # nmpc_w_pn 极大时，任何偏离 PID 参考的候选代价都被放大，最优候选就是参考本身。
        config = SimulationConfig()
        config.guidance.nmpc_w_pn = 1e9
        config.guidance.nmpc_w_fov = 0.0
        pursuer = make_pursuer(velocity=(0.5, 0.0))
        target = make_target(position=(8.0, -3.0), velocity=(1.0, 0.0))

        pid_memory = GuidanceMemory()
        pid_reference = pid_guidance(pursuer, target, pid_memory, config, config.dt)
        hybrid = compute_guidance("pid_nmpc", pursuer, target, GuidanceMemory(), config, config.dt)
        np.testing.assert_allclose(hybrid.acceleration, pid_reference, atol=1e-9)

    def test_small_reference_weight_allows_empc_correction(self) -> None:
        # 参考权重为 0 时 EMPC 可以选择与 PID 参考不同的候选（横向快速穿越目标）。
        config = SimulationConfig()
        config.guidance.nmpc_w_pn = 0.0
        config.guidance.nmpc_w_fov = 0.0
        pursuer = make_pursuer()
        target = make_target(position=(3.0, 0.0), velocity=(0.0, 5.0))
        pid_reference = pid_guidance(pursuer, target, GuidanceMemory(), config, config.dt)
        hybrid = compute_guidance("pid_nmpc", pursuer, target, GuidanceMemory(), config, config.dt)
        self.assertGreater(norm_xy(hybrid.acceleration - pid_reference), 1e-6)


class PidNmpcSimulationSmokeTest(unittest.TestCase):
    """端到端：默认参数下混合算法能在静止场景完成捕获。"""

    def test_captures_stationary_target(self) -> None:
        config = SimulationConfig(sim_time=20.0)
        result = run_algorithm("stationary", "pid_nmpc", config)
        self.assertLess(float(np.min(result.distance)), config.capture_radius)


if __name__ == "__main__":
    unittest.main()
