# 1_初期 — 算法原型阶段

纯 Python 的比例导引（PN）算法验证阶段。不涉及飞控、不涉及 ROS 2，仅用 NumPy + Matplotlib 做数学模型验证。

## 内容

- `pn_guidance_2D.py` — 二维 PN 仿真
- `pn_guidance_3D.py` — 三维 PN 仿真
- `pn_guidance_low_frequency.py` — 低更新频率下的 PN 验证
- `pn_nmpc_guidance_deprecated/` — 曾尝试的 NMPC 集成版本（已废弃，被更简洁的 PN 方案取代）

## 演进路径

本阶段的 PN 核心算法思想被后续的 `5_AntiDrone` 和 `6_Simulation` 继承并工程化实现。
