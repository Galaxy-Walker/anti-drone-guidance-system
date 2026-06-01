# 仿真中 PN + FOV + NMPC 算法框架原理分析

## 1. 本章目的

本章针对有限视场条件下的无人机目标追踪仿真问题，说明本项目采用 `PN + FOV + NMPC` 导引框架的原因及其实现依据。该框架的基本功能分工为：PN（Proportional Navigation，比例导引）提供基于相对运动几何的三维拦截指令；FOV（Field of View，视场）约束将视觉传感器可见性纳入导引过程；NMPC（Nonlinear Model Predictive Control，非线性模型预测控制）在有限预测时域内综合考虑目标接近、视场保持和控制平滑性。通过上述组合，仿真系统能够在速度、加速度、高度以及机头转向速率受限的条件下，使追踪机稳定接近运动目标，并降低目标离开视场的概率。

## 2. 有限视场追踪问题的建模基础

在离线 Python 仿真中，追踪机被建模为带有速度、加速度、高度和 yaw/pitch 角速度约束的三维质点。追踪机状态表示为

```text
x_p = [p_p, v_p, psi, theta]
```

其中，`p_p` 为三维位置，`v_p` 为三维速度，`psi` 为 yaw 角，`theta` 为 pitch 角。目标状态表示为

```text
x_t = [p_t, v_t]
```

其中，`p_t` 和 `v_t` 分别为目标位置与目标速度。源码中对应的数据结构为 `PursuerState` 和 `TargetState`，定义在 `6_Simulation/src/pythonsimulation/state.py` 中。

追踪机的离散运动更新由 `dynamics.py` 中的 `step_pursuer()` 实现。给定导引律输出的加速度指令 `a_cmd` 后，程序先对加速度和速度进行范数限幅，再进行积分，得到下一时刻状态：

```text
a_k = sat(a_cmd, a_max)
v_{k+1} = sat(v_k + a_k dt, v_max)
p_{k+1} = p_k + v_{k+1} dt
```

其中，`sat()` 表示保持方向不变的范数限幅，对应 `math_utils.py` 中的 `clamp_norm()`。此外，高度被限制在 `[z_min, z_max]` 区间内，以保证简化质点模型的状态满足仿真边界条件。

与完整飞行动力学模型不同，本项目中的 yaw/pitch 主要用于描述机头或相机朝向，而非完整姿态控制。`step_pursuer()` 根据 `look_at_position` 计算目标 yaw/pitch，并在最大角速度约束下逐步调整朝向：

```text
psi_{k+1} = wrap(psi_k + sat(psi_des - psi_k, +/- dot_psi_max dt))
theta_{k+1} = sat(theta_k + sat(theta_des - theta_k, +/- dot_theta_max dt))
```

因此，导引算法不仅需要确定追踪机的加速度方向和大小，还需要给出用于更新机头或相机朝向的参考位置。FOV 约束和 NMPC 预测代价均建立在这一建模基础之上。

## 3. PN 制导：三维拦截趋势的生成

PN 的基本思想是通过调节追踪机加速度，减小目标视线方向的旋转趋势，使追踪机相对于目标的运动逐渐满足拦截条件。有限视场追踪虽然引入了传感器可见性约束，但其基础仍是相对运动下的目标接近问题。因此，PN 适合作为本项目的名义导引律。

在 `guidance.py` 的 `pn_guidance()` 中，追踪机到目标的相对位置定义为

```text
r = p_t - p_p
R = ||r||
u_LOS = r / R
```

其中，`u_LOS` 为由追踪机指向目标的单位视线方向。相对速度定义为

```text
v_rel = v_t - v_p
```

源码中闭合速度采用

```text
V_c = max(0, -v_rel . u_LOS)
```

当 `V_c > 0` 时，追踪机与目标沿 LOS 方向接近；当该量小于零时，二者沿 LOS 方向远离。实现中将负值截断为 0，以避免在远离状态下由 PN 横向项产生不合理的反向放大。

三维 LOS 角速度采用向量形式计算：

```text
omega_LOS = (r x v_rel) / R^2
```

PN 横向加速度为

```text
a_PN = N V_c (omega_LOS x u_LOS)
```

其中，`N` 为比例导引常数，对应 `GuidanceConfig.pn_navigation_constant`，默认值为 `3.5`。该加速度方向垂直于 LOS，用于补偿目标横向运动引起的视线旋转。当目标存在侧向速度时，LOS 会发生变化，PN 通过横向加速度使追踪机向预期交会方向调整。

项目实现还加入了径向闭合项：

```text
a_close = k_close (v_des - v_p . u_LOS) u_LOS
```

其中，`k_close` 对应 `GuidanceConfig.pn_k_close`，`v_des` 对应 `GuidanceConfig.pn_v_des_along_los`。该项用于在初始速度较低或闭合速度不足时增强沿 LOS 方向的接近能力。最终 PN 输出为

```text
a_nom = a_PN + a_close
```

该输出作为后续 FOV 与 NMPC 层使用的名义导引指令。当算法类型为 `pn_fov_nmpc` 时，`compute_guidance()` 先调用 `pn_guidance()` 得到名义加速度，再由 `nmpc_acceleration()` 进行预测优化。因此，NMPC 的搜索并非在整个控制空间内无约束展开，而是在 PN 给出的三维拦截趋势附近选择更符合有限视场追踪目标的控制量。

## 4. FOV 约束：可见性条件的闭环引入

仅使用 PN 时，算法隐含假设目标状态始终可用。然而，在视觉追踪任务中，目标可能因相机视场有限、机头转向速率受限或近距离交会等因素离开画面。目标不可见时，真实系统无法继续直接利用当前真实目标状态，只能基于最后一次观测状态及短时预测进行导引。因此，本项目将 FOV 可见性作为导引计算的一部分。

FOV 判断由 `math_utils.py` 中的 `fov_visibility()` 实现。给定追踪机 yaw/pitch 后，机头或相机前向向量为

```text
f(psi, theta) = [
    cos(theta) cos(psi),
    cos(theta) sin(psi),
    sin(theta)
]
```

目标方向为 LOS 方向：

```text
u_LOS = (p_t - p_p) / ||p_t - p_p||
```

目标方向与前向向量的夹角为

```text
alpha = arccos(clip(f . u_LOS, -1, 1))
```

若相机总视场角为 `fov_deg`，则半视场角为

```text
phi = fov_deg / 2
```

源码中 `GuidanceConfig.fov_deg` 默认值为 `90.0`，因此半视场角 `fov_half_angle` 为 45 度。可见性判据为

```text
visible = (alpha <= phi)
```

该判据在 `compute_guidance()` 开始阶段统一计算。对于 FOV 类算法，若目标可见，则使用真实目标状态，并在 `GuidanceMemory.last_seen_target` 中更新最后观测状态；若目标不可见，则调用 `_target_reference_for_fov()`，根据最后一次观测状态进行常速度外推：

```text
p_hat_t = p_last + v_last t_lost
v_hat_t = v_last
```

其中，`t_lost` 对应 `GuidanceMemory.lost_time`。这意味着目标丢失后，导引律使用预测目标 `target_reference`，而不是继续访问真实目标当前位置。与此同时，源码会对 PN 加速度乘以 `GuidanceConfig.lost_guidance_gain`，其默认值为 `0.55`：

```text
a = lost_guidance_gain * a_PN(p_hat_t, v_hat_t)
```

该处理反映了目标不可见时观测信息不确定性增加的情况，使控制指令相对于可见状态更为保守。

FOV 层还影响机头或相机朝向。`GuidanceResult.look_at_position` 在目标可见时通常为真实目标位置，在目标不可见时为预测目标位置。随后 `step_pursuer()` 根据该位置更新 yaw/pitch。因此，FOV 在本项目中不仅是性能评价指标，也直接影响导引输入和朝向参考。

## 5. 在 PN 和 FOV 基础上引入 NMPC 的必要性

PN 提供拦截趋势，FOV 描述视觉可见性以及目标丢失后的信息退化。然而，仅将二者串联仍难以处理一个关键问题：当前控制指令可能在未来若干步导致目标出视场、控制过大或近距离飞越。有限视场追踪需要在多个优化目标之间进行权衡，主要包括：

1. 追踪机与目标之间的距离应逐步减小；
2. LOS 夹角应尽量不超过 FOV 半角；
3. 加速度幅值应满足约束，并避免不必要的控制能量消耗；
4. 相邻控制指令应保持连续，以减少轨迹和朝向抖动；
5. 控制方向不应显著偏离 PN 给出的拦截趋势，否则可能削弱长期接近能力。

NMPC 适用于处理上述多目标折中问题。其基本流程是在当前时刻基于系统模型向前预测有限时间，对候选控制序列计算综合代价，并仅执行当前一步控制；下一控制周期重新利用最新状态进行预测和优化。该滚动优化机制能够将未来短时范围内的 FOV 风险和控制品质纳入当前控制选择。

本项目实现的是轻量级 NMPC。该实现没有调用 CasADi、acados 或 Ipopt 等通用非线性优化器，而是在 `guidance.py` 中通过 `nmpc_acceleration()` 枚举有限数量的候选加速度，并使用 `_rollout_cost()` 对候选进行滚动仿真评分。该设计具有公式直观、依赖较少、便于复现和便于结合当前 Python 质点模型的特点，适合本项目的教学与仿真分析目标。

## 6. NMPC 的候选控制构造

`nmpc_acceleration()` 首先调用 `_candidate_accelerations()` 构造候选加速度集合。候选控制并非随机生成，而是围绕 PN 名义趋势和有限视场追踪需求构造。设 PN 名义加速度为 `a_PN`，源码中主要包含以下几类候选。

第一类为 PN 缩放候选：

```text
a = a_PN
a = 0.55 a_PN
a = 1.25 a_PN
```

该类候选用于比较不同 PN 强度在预测窗口内对应的综合代价。

第二类为直接接近目标的候选。源码先构造期望拦截速度

```text
v_intercept = v_des u_LOS
```

再采用一阶速度跟踪形式得到加速度：

```text
a_intercept = (v_intercept - v_p) / tau
```

然后将其与 PN 趋势混合：

```text
a = 0.75 a_PN + 0.25 a_intercept
a = 0.5 a_PN + 0.5 a_intercept
```

该类候选用于在距离代价较高时提高沿目标方向的接近能力。

第三类为同速接近和速度匹配候选。源码中的同速接近候选为

```text
v_same = ||v_t|| u_LOS
a_same = (v_same - v_p) / tau
```

速度匹配候选为

```text
a_match = (v_t - v_p) / tau
```

同速接近候选仍以 LOS 方向为主要参考，但速度大小接近目标速度；速度匹配候选则更关注降低相对速度。上述候选有助于减小近距离快速飞越时的 LOS 角速度变化。

第四类为 LOS 垂直方向扰动。源码构造与 `u_LOS` 垂直的横向方向 `lateral` 和竖向横向方向 `vertical_lateral`，并生成

```text
a = a_PN +/- 0.35 a_max lateral
a = a_PN +/- 0.25 a_max vertical_lateral
```

该类候选具有明确的几何含义：当直接接近目标可能导致 LOS 夹角快速增大时，横向或竖向调整可能降低目标离开视场的风险。

所有候选最终均经过 `clamp_norm(candidate, a_max)` 限幅，以保证 NMPC 输出满足追踪机最大加速度约束。

## 7. NMPC 的预测模型与代价函数

每个候选加速度都会在 `_rollout_cost()` 中进行短时预测。预测窗口长度由 `GuidanceConfig.horizon_steps` 和 `GuidanceConfig.mpc_dt` 决定，默认设置为

```text
20 steps * 0.1 s = 2.0 s
```

在预测窗口内，候选加速度保持不变。对第 `i` 个预测步，目标位置采用常速度模型：

```text
p_t(i) = p_t(0) + v_t i dt_mpc
```

其中，`p_t(0), v_t` 为当前可用的目标参考状态。若目标可见，该参考状态来自真实目标；若目标不可见，该参考状态来自 FOV 层基于 last-seen 状态得到的外推结果。因此，NMPC 的预测过程同样遵守有限视场下的信息可用性约束。

追踪机预测状态使用与主仿真一致的 `step_pursuer()` 更新：

```text
x_p(i+1) = F(x_p(i), a, p_t(i))
```

其中，`F` 包含加速度限幅、速度限幅、位置积分、高度约束以及 yaw/pitch 角速度约束。该设计保证 NMPC 评估候选控制时使用的预测模型与主仿真执行模型一致。

预测过程中计算以下代价项。

终端距离代价：

```text
J_dist = ||p_t(H) - p_p(H)||
```

该项用于评价预测窗口末端追踪机与目标之间的距离。

路径距离代价：

```text
J_path = sum_i ||p_t(i) - p_p(i)||
```

该项用于约束预测窗口内整体距离水平，避免控制器只优化终端时刻的距离。

FOV 违反代价：

```text
alpha_i = angle(f(psi_i, theta_i), p_t(i) - p_p(i))
J_fov = sum_i max(0, alpha_i - phi)^2
```

当 LOS 夹角不超过半视场角时，该项为 0；当 LOS 夹角超过 FOV 边界时，惩罚随超出量增大而增大。平方形式使较大的出视场角度受到更高惩罚。

控制能量代价：

```text
J_control = sum_i ||a||^2 dt_mpc
```

该项用于抑制过大的加速度指令，从而提高轨迹平滑性并降低控制能量消耗。

控制平滑代价：

```text
J_smooth = sum_i ||a_i - a_{i-1}||^2
```

在当前实现中，单个候选在预测窗口内保持固定加速度，因此该项主要用于惩罚当前候选与上一控制周期实际加速度之间的突变。上一控制周期的加速度保存在 `GuidanceMemory.previous_acceleration` 中。

PN 趋势偏离代价：

```text
J_PN = sum_i ||a_i - a_PN||^2
```

该项用于约束 NMPC 输出相对于 PN 名义导引趋势的偏离程度，使预测优化仍保持与拦截几何的一致性。

最终总代价为

```text
J =
    w_dist J_dist
  + w_path J_path
  + w_fov J_fov
  + w_control J_control
  + w_smooth J_smooth
  + w_PN J_PN
```

对应源码中的配置项为：

```text
GuidanceConfig.nmpc_w_dist
GuidanceConfig.nmpc_w_path
GuidanceConfig.nmpc_w_fov
GuidanceConfig.nmpc_w_control
GuidanceConfig.nmpc_w_smooth
GuidanceConfig.nmpc_w_pn
```

当前默认权重中，距离项和 FOV 项权重较高，表明该控制器同时强调目标接近和视场保持；控制能量、控制平滑性和 PN 偏离项则用于抑制过大控制、降低控制突变并排除与拦截趋势明显不一致的候选。

## 8. 滚动执行机制

NMPC 不会一次性执行整个预测窗口内的控制计划。`nmpc_acceleration()` 仅选择当前代价最低候选对应的当前步加速度：

```text
a_k = argmin_a J(a)
```

随后，主仿真只执行当前仿真步。下一时刻，系统重新获取追踪机状态、目标状态或 last-seen 预测状态，并重新计算 PN 趋势、生成候选控制、执行滚动预测和代价评估。该 receding horizon 机制使控制器能够根据最新的相对位置、速度、可见性和机头朝向持续修正控制指令。

从 `compute_guidance()` 的调用链可得完整流程：

```text
1. fov_visibility() 计算真实目标当前是否可见。
2. _target_reference_for_fov() 生成当前可用目标参考。
3. pn_guidance() 根据目标参考计算 PN 名义加速度。
4. nmpc_acceleration() 围绕 PN 趋势生成候选并滚动评分。
5. GuidanceResult 返回加速度、可见性、LOS 角和 look_at_position。
6. step_pursuer() 执行加速度并使 yaw/pitch 转向 look_at_position。
```

由此可见，PN、FOV 与 NMPC 在控制链路中分别承担不同功能：PN 生成拦截方向，FOV 约束目标状态的可用性和朝向参考，NMPC 在有限预测时域内选择兼顾距离、视场和控制品质的当前控制量。

## 9. 该组合适合本项目的原因

`PN + FOV + NMPC` 适用于本项目的仿真任务，首先是因为该组合与有限视场目标追踪的物理约束一致。目标追踪不仅取决于空间距离，还受传感器朝向和可见性约束影响。PN 基于相对运动几何生成三维拦截指令；FOV 在目标不可见时将导引参考退化为 last-seen 预测结果，避免继续使用理想目标状态；NMPC 则将未来短时间内的视场违反风险纳入当前控制选择。

其次，该组合具有较好的可解释性。PN 公式由相对位置、相对速度和 LOS 角速度确定；FOV 判据由相机前向向量与 LOS 夹角确定；NMPC 代价函数中的各项分别对应距离、视场、控制能量、控制平滑性和 PN 趋势一致性。源码中的 `pn_guidance()`、`fov_visibility()`、`_target_reference_for_fov()`、`_candidate_accelerations()` 和 `_rollout_cost()` 分别对应上述计算过程，便于对算法行为进行复现和分析。

再次，当前 NMPC 实现与项目的简化动力学模型保持一致。预测阶段使用的 `step_pursuer()` 与主仿真执行阶段使用的是同一函数，因此候选控制的代价评估与实际执行模型具有一致的状态更新逻辑。速度、加速度、高度以及 yaw/pitch 角速度约束均在预测过程中体现，使候选控制评分更接近其实际执行效果。

最后，轻量候选式 NMPC 降低了对通用非线性规划求解器的依赖。对于本项目的仿真目标而言，重点在于分析 PN 与 FOV 约束下引入短时预测后的导引效果，而非展示通用优化器的求解性能。有限候选集合和结构化代价函数有利于复现实验、调整权重和解释结果。

## 10. 小结

本章采用 `PN + FOV + NMPC` 框架的原因可概括为以下三点。

第一，PN 提供基础拦截能力。该方法基于相对位置、相对速度和 LOS 角速度计算三维横向加速度，并通过径向闭合项增强目标接近能力。

第二，FOV 提供视觉追踪约束。该模块由 yaw/pitch 生成机头或相机前向向量，并通过 LOS 夹角判断目标可见性；当目标丢失时，仅使用最后一次观测状态进行常速度外推，并降低导引增益，以描述有限视场传感器下的信息退化。

第三，NMPC 提供短时预测优化能力。该模块围绕 PN 趋势构造候选加速度，在 2 秒预测窗口内评估终端距离、路径距离、FOV 违反、控制能量、控制平滑性和 PN 偏离代价，并采用滚动时域方式执行当前最优控制。

综上，`PN + FOV + NMPC` 构成了由名义拦截、可见性约束和短时预测优化组成的导引框架。该框架在保持实现可解释性的同时，能够在有限视场追踪任务中兼顾目标接近、目标可见性和控制平滑性。
