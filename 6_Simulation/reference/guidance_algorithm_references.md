# guidance.py 导引算法参考资料汇总

生成时间：2026-05-13

源文件：`6_Simulation/src/pythonsimulation/guidance.py`

## 代码算法对应关系

| 代码入口 | 算法主题 | 参考资料侧重点 |
| --- | --- | --- |
| `direct_pursuit()` / `basic` | 直接追踪 / Pure Pursuit 风格速度指向目标 | 作为基线算法，通常用于和 PN、MPC 类方法对比 |
| `pn_guidance()` / `pn` | Proportional Navigation, PN | 经典拦截导引律、LOS 角速度、closing speed |
| `pn_fov` | PN + 视场约束 + last-seen 常速度预测 | FOV-constrained guidance、视觉伺服、目标丢失后的短期预测 |
| `fov_cbf_acceleration()` / `pn_fov_cbf` | PN 名义控制 + CBF 风格安全滤波 | 控制屏障函数、有限视场安全约束、视觉目标保持在 FOV 内 |
| `nmpc_acceleration()` / `pn_fov_nmpc` | PN 趋势附近滚动预测 / 轻量 NMPC | UAV target tracking、visibility constraint、obstacle / control constraints |
| `mppi_acceleration()` / `pn_fov_mppi` | PN 名义序列 + MPPI 随机采样加权 | sampling-based MPC、Model Predictive Path Integral、并行 rollout |

## 优先阅读资料

| 优先级 | 资料 | 类型 | 开源代码 | 关联算法 | 备注 |
| --- | --- | --- | --- | --- | --- |
| A | Prakrit Tyagi, Yogesh Kumar, P. B. Sujit, “NMPC-based UAV 3D Target Tracking In The Presence Of Obstacles and Visibility Constraints,” ICUAS 2021. DOI: `10.1109/ICUAS51884.2021.9476710` | 论文 | https://github.com/PrakritTyagi/MPC_Target-Tracking_UAV | `pn_fov_nmpc`、FOV/visibility | 论文题目直接覆盖 UAV 3D 目标跟踪、障碍和可见性约束；仓库 README 明确说明是该 ICUAS 2021 论文项目代码。 |
| A | devsonni/MPC-Implementation | 开源项目 | https://github.com/devsonni/MPC-Implementation | `pn_fov_nmpc` | README 标注为 “NMPC-based UAV 3D Target Tracking In The Presence Of Obstacles and Visibility Constraints” 的源码，含 Python/MATLAB/C++，可作为实现参考。 |
| A | Grady Williams, Andrew Aldrich, Evangelos A. Theodorou, “Model Predictive Path Integral Control: From Theory to Parallel Computation,” JGCD 2017. DOI: `10.2514/1.G001921` | 论文 | 相关实现：https://github.com/UM-ARM-Lab/pytorch_mppi | `pn_fov_mppi` | MPPI 理论和并行实现的核心论文；代码里的随机控制序列、代价加权、temperature 对应 MPPI 思路。 |
| A | UM-ARM-Lab/pytorch_mppi | 开源项目 | https://github.com/UM-ARM-Lab/pytorch_mppi | `pn_fov_mppi` | PyTorch MPPI 实现，MIT license，README 明确支持 approximate dynamics、control bounds、batch rollout。 |
| A | Biagio Trimarchi, Fabrizio Schiano, Roberto Tron, “A Control Barrier Function Candidate for Quadrotors with Limited Field of View,” arXiv:2410.01277, 2024/2025. DOI: `10.48550/arXiv.2410.01277` | 论文 | 未发现官方代码 | `pn_fov_cbf`、FOV | 直接讨论 quadrotor 有限视场的 CBF 构造，和代码中 FOV 边界软约束/安全修正最接近。 |
| A | Paul Zarchan, “Tactical and Strategic Missile Guidance,” AIAA, 6th/7th edition. DOI: `10.2514/4.868948` / `10.2514/4.105845` | 专著 | 无 | `pn` | PN/APN/导引闭环的经典工程参考。适合写算法背景和公式来源。 |

## PN / 直接追踪相关

| 资料 | 类型 | 链接 | 代码相关性 |
| --- | --- | --- | --- |
| ajfrewin/pn-guidance | Python 项目 | https://github.com/ajfrewin/pn-guidance | 简洁 PN missile-target 仿真，适合快速对照 LOS rate 和 PN command。 |
| iwishiwasaneagle/proportional_navigation | Python 包 | https://github.com/iwishiwasaneagle/proportional_navigation | Python >=3.10 的 PN 工具包，README 解释 heading/velocity 模型；GPL-3.0。 |
| alti3/missile-proportional-navigation-python | Python 项目 | https://github.com/alti3/missile-proportional-navigation-python | 含 2D/3D Pure PN、True PN、Augmented PN、Generalized PN 示例；对照 `pn_guidance()` 很方便。 |
| Shukla and Mahapatra, “A Generalized Linear Solution of Proportional Navigation,” IEEE TAES 1988. DOI: `10.1109/7.192091` | 论文 | https://doi.org/10.1109/7.192091 | 经典 PN 解析形式参考。 |
| Yang, Yeh, and Chen, “The Closed Form Solution of Generalized Proportional Navigation,” JGCD 1987. DOI: `10.2514/3.20205` | 论文 | https://doi.org/10.2514/3.20205 | Generalized PN 背景。 |
| Guelman, “Proportional Navigation with a Maneuvering Target,” IEEE TAES 1972. DOI: `10.1109/TAES.1972.309520` | 论文 | https://doi.org/10.1109/TAES.1972.309520 | 机动目标 PN 参考。 |

代码备注：`guidance.py` 的 PN 使用三维向量形式 `omega_los = cross(r, v_rel) / |r|^2` 和 `a_pn = N * closing_speed * cross(omega_los, u_los)`，更接近 True PN / 3D PN 的形式；同时加了 `pn_k_close` 径向闭合项，属于工程增强，不是标准 PN 的唯一形式。

## FOV / 目标丢失预测相关

| 资料 | 类型 | 链接 | 代码相关性 |
| --- | --- | --- | --- |
| Zhang, Wang, Wu, “Impact Time Control Guidance with Field-of-View Constraint Accounting for Uncertain System Lag,” Proc. IMechE Part G, 2016 | 论文 | 可从 AIAA/Google Scholar 以题名检索 | FOV-constrained guidance 代表性关键词，适合作为“视场约束导引”背景。 |
| “Time-Constrained Interception with Bounded Field of View and Input ...,” JGCD. DOI: `10.2514/1.G007770` | 论文 | https://doi.org/10.2514/1.G007770 | FOV 受限拦截问题，和 `pn_fov` 的设计目标相关。 |
| Trimarchi, Schiano, Tron, “A Control Barrier Function Candidate for Quadrotors with Limited Field of View,” arXiv:2410.01277 | 论文 | https://arxiv.org/abs/2410.01277 | 将 FOV 约束显式写成 CBF 候选函数，适合作为 `pn_fov_cbf` 的主要参考。 |
| Vision-based finite-time prescribed performance control for uncooperative aerial target tracking subject to field-of-view constraints, ISA Transactions 2024 | 论文 | ScienceDirect 可按题名检索 | 视觉跟踪 + FOV 约束，适合作为相关工作补充。 |

代码备注：`_target_reference_for_fov()` 使用 last-seen position/velocity 的常速度预测。这不是完整滤波器，而是一个轻量级 target motion prediction；如果后续写论文，可以把它描述为 “constant-velocity prediction after target loss”。

## CBF 相关

| 资料 | 类型 | 链接 | 代码相关性 |
| --- | --- | --- | --- |
| Ames, Xu, Grizzle, Tabuada, “Control Barrier Function Based Quadratic Programs for Safety Critical Systems,” IEEE TAC 2017. DOI: `10.1109/TAC.2016.2638961` | 论文 | https://doi.org/10.1109/TAC.2016.2638961 | CBF-QP 基础文献。 |
| Trimarchi, Schiano, Tron, “A Control Barrier Function Candidate for Quadrotors with Limited Field of View,” arXiv:2410.01277 | 论文 | https://arxiv.org/abs/2410.01277 | 与有限视场 CBF 最直接相关。 |
| CBF Based Geometric Visual Servoing Control for Quadrotors ... | 会议章节 | Springer，可按题名检索 | 视觉伺服 + FOV + CBF，可作为相关工作。 |

代码备注：当前 `fov_cbf_acceleration()` 更像 “CBF-inspired safety filter”，没有解 QP，也没有严格构造 `h(x)` 与 `dot h + alpha(h) >= 0` 约束。写文档时建议称为“CBF 风格的启发式安全滤波器”，除非后续改成真正 CBF-QP。

## NMPC 相关

| 资料 | 类型 | 链接 | 开源代码 | 代码相关性 |
| --- | --- | --- | --- | --- |
| Tyagi, Kumar, Sujit, “NMPC-based UAV 3D Target Tracking In The Presence Of Obstacles and Visibility Constraints,” ICUAS 2021. DOI: `10.1109/ICUAS51884.2021.9476710` | 论文 | https://doi.org/10.1109/ICUAS51884.2021.9476710 | https://github.com/PrakritTyagi/MPC_Target-Tracking_UAV | 最相关。 |
| devsonni/MPC-Implementation | 项目 | https://github.com/devsonni/MPC-Implementation | 同链接 | 另一个包含 Python/MATLAB/C++ 的 NMPC target tracking 实现。 |
| sotomotocross/vsc_nmpc_uav_target_tracking | ROS C++ 项目 | https://github.com/sotomotocross/vsc_nmpc_uav_target_tracking | 同链接 | 视觉 NMPC UAV target tracking，适合作 ROS/Gazebo 思路参考。 |
| Chunhui Zhao, Dong Wang, Jinwen Hu, Quan Pan, “Nonlinear model predictive control-based guidance algorithm for quadrotor trajectory tracking with obstacle avoidance,” Journal of Systems Science and Complexity 2021. DOI: `10.1007/s11424-021-0316-9` | 论文 | https://doi.org/10.1007/s11424-021-0316-9 | 未发现官方代码 | NMPC + quadrotor + obstacle avoidance 的背景参考。 |
| Wang et al., “Efficient nonlinear model predictive control for quadrotor trajectory tracking: algorithms and experiment,” IEEE TCYB 2021. DOI: `10.1109/TCYB.2020.3043361` | 论文 | https://doi.org/10.1109/TCYB.2020.3043361 | 未在本次检索中核验官方代码 | 实验型 quadrotor NMPC 参考。 |

代码备注：`nmpc_acceleration()` 没有调用 CasADi/Ipopt 等优化器，而是枚举候选加速度并 rollout 后选最低代价。论文写法建议称为 “sampling/enumeration-based receding-horizon controller” 或 “lightweight NMPC-inspired rollout selection”，避免误称为求解完整非线性规划的 NMPC。

## MPPI 相关

| 资料 | 类型 | 链接 | 开源代码 | 代码相关性 |
| --- | --- | --- | --- | --- |
| Williams, Aldrich, Theodorou, “Model Predictive Path Integral Control: From Theory to Parallel Computation,” JGCD 2017. DOI: `10.2514/1.G001921` | 论文 | https://doi.org/10.2514/1.G001921 | ACDS 相关资料页：https://sites.gatech.edu/acds/mppi/ | MPPI 核心理论。 |
| Williams et al., “Information Theoretic MPC for Model-Based Reinforcement Learning,” ICRA 2017. DOI: `10.1109/ICRA.2017.7989202` | 论文 | https://doi.org/10.1109/ICRA.2017.7989202 | https://github.com/UM-ARM-Lab/pytorch_mppi | approximate dynamics / learned dynamics 的 MPPI 参考。 |
| Williams et al., “Aggressive Driving with Model Predictive Path Integral Control,” ICRA 2016. DOI: `10.1109/ICRA.2016.7487277` | 论文 | https://doi.org/10.1109/ICRA.2016.7487277 | ACDS AutoRally 相关资料 | 早期 MPPI 实验论文。 |
| UM-ARM-Lab/pytorch_mppi | Python 项目 | https://github.com/UM-ARM-Lab/pytorch_mppi | 同链接 | 批量 rollout、采样、代价加权实现参考。 |
| GuanyaShi/Quad_MPPI | Python/PyTorch 项目 | https://github.com/GuanyaShi/Quad_MPPI | 同链接 | Quadrotor simulation with MPPI controller，和无人机平台更接近。 |
| takepiyo/drone-and-robotic-control-algorithms | Python 项目 | https://github.com/takepiyo/drone-and-robotic-control-algorithms | 同链接 | 含 MPPI 和 quadrotor/toy problem 实现，可作辅助参考。 |

代码备注：`mppi_acceleration()` 采用 PN 作为 nominal control sequence，再给整段控制序列加 Gaussian noise，用 `exp(-cost / temperature)` 做权重融合第一步控制。这个结构和 MPPI 的采样加权形式一致，但当前动力学和代价是为本仿真定制的轻量实现。

## 推荐引用组合

如果只需要给当前 `guidance.py` 写一段“参考依据”，建议优先引用以下组合：

1. PN：Zarchan, “Tactical and Strategic Missile Guidance,” AIAA.
2. NMPC + visibility：Tyagi, Kumar, Sujit, ICUAS 2021, DOI `10.1109/ICUAS51884.2021.9476710`，并附 GitHub `PrakritTyagi/MPC_Target-Tracking_UAV`。
3. CBF + limited FOV：Trimarchi, Schiano, Tron, arXiv:2410.01277。
4. MPPI：Williams, Aldrich, Theodorou, JGCD 2017, DOI `10.2514/1.G001921`，并附 GitHub `UM-ARM-Lab/pytorch_mppi`。

## 检索与核验说明

本次优先查找“论文明确关联开源项目”的资料。已核验的强关联开源项目包括 `PrakritTyagi/MPC_Target-Tracking_UAV`、`devsonni/MPC-Implementation`、`UM-ARM-Lab/pytorch_mppi`、`GuanyaShi/Quad_MPPI`、`iwishiwasaneagle/proportional_navigation`、`ajfrewin/pn-guidance` 和 `alti3/missile-proportional-navigation-python`。

DuckDuckGo 在部分查询中触发了反爬验证，Crossref 在部分查询中触发了 429 限流；因此个别 FOV/CBF 扩展论文只保留题名、平台和 DOI/检索入口。核心论文和开源仓库已经通过 IEEE/Crossref/arXiv/GitHub 页面核验。
