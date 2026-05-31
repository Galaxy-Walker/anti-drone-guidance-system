# 6_Simulation 中 PN + FOV + NMPC 算法框架原理分析

## 1. 本文目的

`6_Simulation` 的核心任务是在有限视场条件下，使追踪无人机稳定、平滑地接近运动目标，并尽量减少目标离开相机或机头视场的时间。本文重点说明本项目为什么将 `PN + FOV + NMPC` 作为主要导引框架：PN 提供可解释的三维拦截趋势，FOV 将视觉传感器的可见性约束纳入导引闭环，NMPC 在短时预测窗口内对接近目标、保持视场和控制平滑性进行统一优化。

本文主要分析算法原理及其源码实现关系，不展开与其他算法的对比，也不讨论仿真框架选型理由。

## 2. 有限视场追踪问题的建模基础

在离线 Python 仿真中，追踪机被建模为带速度、加速度、高度和 yaw/pitch 角速度约束的三维质点。追踪机状态可写为

```text
x_p = [p_p, v_p, psi, theta]
```

其中 `p_p` 为三维位置，`v_p` 为三维速度，`psi` 为 yaw，`theta` 为 pitch。目标状态可写为

```text
x_t = [p_t, v_t]
```

其中 `p_t` 和 `v_t` 分别为目标位置与速度。源码中的对应数据结构为 `PursuerState` 和 `TargetState`，定义在 `6_Simulation/src/pythonsimulation/state.py`。

追踪机的离散运动更新由 `dynamics.py` 中的 `step_pursuer()` 实现。给定导引律输出的加速度指令 `a_cmd` 后，程序先对加速度和速度进行幅值限制，再积分得到下一时刻状态：

```text
a_k = sat(a_cmd, a_max)
v_{k+1} = sat(v_k + a_k dt, v_max)
p_{k+1} = p_k + v_{k+1} dt
```

这里的 `sat()` 表示保持方向不变的范数限幅，对应 `math_utils.py` 中的 `clamp_norm()`。高度还会被限制在 `[z_min, z_max]` 内，避免简化质点模型飞到地下或过高。

与完整飞行动力学不同，本项目中的 yaw/pitch 主要用于表示机头或相机朝向，而不是完整姿态控制。`step_pursuer()` 会根据 `look_at_position` 计算目标 yaw/pitch，并通过最大角速度限制逐步转向：

```text
psi_{k+1} = wrap(psi_k + sat(psi_des - psi_k, +/- dot_psi_max dt))
theta_{k+1} = sat(theta_k + sat(theta_des - theta_k, +/- dot_theta_max dt))
```

因此，算法不仅要决定追踪机“往哪里加速”，还要决定机头或相机“看向哪里”。这正是后续引入 FOV 约束和 NMPC 预测代价的原因。

## 3. PN 制导：提供三维拦截趋势

PN（Proportional Navigation，比例导引）的基本思想是：如果追踪机能够使目标视线方向的旋转角速度逐渐减小，那么追踪机就不是单纯追向目标当前位置，而是在向目标未来交会方向收敛。它适合作为本项目的基础导引律，因为有限视场追踪首先仍然是一个相对运动拦截问题。

在 `guidance.py` 的 `pn_guidance()` 中，追踪机到目标的相对位置定义为

```text
r = p_t - p_p
R = ||r||
u_LOS = r / R
```

其中 `u_LOS` 是从追踪机指向目标的单位视线方向。相对速度为

```text
v_rel = v_t - v_p
```

源码中闭合速度使用

```text
V_c = max(0, -v_rel . u_LOS)
```

当 `V_c > 0` 时，说明追踪机与目标沿 LOS 方向正在接近；当其小于零时，说明双方正在远离，此时源码将其截断为 0，避免 PN 横向项在远离状态下反向放大。

三维 LOS 角速度采用向量形式计算：

```text
omega_LOS = (r x v_rel) / R^2
```

PN 横向加速度为

```text
a_PN = N V_c (omega_LOS x u_LOS)
```

其中 `N` 是比例导引常数，对应 `GuidanceConfig.pn_navigation_constant`，默认值为 `3.5`。这个公式的方向垂直于 LOS，用于修正由于目标横向运动导致的视线旋转。若目标存在侧向速度，LOS 会持续转动，PN 就会产生横向加速度，使追踪机提前切入拦截方向。

项目实现还额外加入了一个径向闭合项：

```text
a_close = k_close (v_des - v_p . u_LOS) u_LOS
```

其中 `k_close` 对应 `GuidanceConfig.pn_k_close`，`v_des` 对应 `GuidanceConfig.pn_v_des_along_los`。该项的作用是让追踪机在初始速度较小或闭合速度不足时主动沿 LOS 方向接近目标。最终 PN 输出为

```text
a_nom = a_PN + a_close
```

这也是后续 FOV 与 NMPC 层所使用的名义导引趋势。源码中的 `compute_guidance()` 在算法为 `pn_fov_nmpc` 时，先调用 `pn_guidance()` 得到该趋势，再交给 `nmpc_acceleration()` 做预测优化。因此，NMPC 并不是从零搜索控制量，而是在 PN 给出的三维拦截趋势附近寻找更适合有限视场追踪的加速度。

## 4. FOV 约束：把“看得见目标”纳入闭环

如果只使用 PN，算法隐含假设目标状态始终可用。但视觉追踪任务中，目标可能因为相机视场有限、机头转向不及时或近距离交会而离开画面。一旦目标不可见，真实系统无法继续使用当前真实目标状态，只能使用最后一次观测及其短时预测。因此，本项目把 FOV 可见性作为导引过程的一部分。

FOV 判断由 `math_utils.py` 中的 `fov_visibility()` 实现。给定追踪机 yaw/pitch，机头或相机前向向量为

```text
f(psi, theta) = [
    cos(theta) cos(psi),
    cos(theta) sin(psi),
    sin(theta)
]
```

目标方向仍为 LOS 方向：

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

这个判据在 `compute_guidance()` 开始处统一计算。对于 FOV 类算法，若目标可见，则使用真实目标状态，并在 `GuidanceMemory.last_seen_target` 中刷新最后观测状态；若目标不可见，则调用 `_target_reference_for_fov()`，只根据最后一次观测状态做常速度外推：

```text
p_hat_t = p_last + v_last t_lost
v_hat_t = v_last
```

其中 `t_lost` 对应 `GuidanceMemory.lost_time`。这意味着目标一旦丢失，导引律使用的是预测目标 `target_reference`，而不是直接“偷看”真实目标当前位置。与此同时，源码还会对 PN 加速度乘以 `GuidanceConfig.lost_guidance_gain`，默认值为 `0.55`：

```text
a = lost_guidance_gain * a_PN(p_hat_t, v_hat_t)
```

这样做的含义是：目标丢失后，算法仍然沿预测点搜索，但因为预测信息的不确定性更高，控制指令不应像看到真实目标时那样激进。

FOV 层还会影响机头或相机朝向。`GuidanceResult.look_at_position` 在目标可见时通常是真实目标位置，在目标丢失时则是预测目标位置。随后 `step_pursuer()` 根据该位置更新 yaw/pitch。因此，FOV 不只是一个用于画图的评价指标，而是同时改变了导引输入和朝向更新目标。

## 5. 为什么在 PN 和 FOV 基础上加入 NMPC

PN 负责产生拦截趋势，FOV 负责描述视觉可见性和目标丢失后的信息退化，但二者组合后仍然缺少一个关键能力：提前评估当前控制会不会在未来几步造成目标出视场、控制过猛或近距离飞越。

有限视场追踪不是单目标问题。算法至少需要同时考虑：

1. 距离应逐步减小，使追踪机接近目标。
2. LOS 夹角应尽量小于 FOV 半角，使目标保持可见。
3. 加速度不能过大，否则轨迹和能耗都不理想。
4. 相邻控制不应突变，否则 yaw/pitch 和轨迹容易抖动。
5. 控制方向不能完全背离 PN 的拦截趋势，否则可能短期看似保视场，长期却无法接近目标。

NMPC（Nonlinear Model Predictive Control，非线性模型预测控制）的核心思想正适合处理这种多目标折中：在当前时刻基于系统模型向前预测一小段时间，对候选控制序列计算综合代价，然后只执行当前一步控制；下一控制周期重新观测、重新预测、重新选择控制。这种“滚动优化”机制可以把未来的 FOV 风险提前反映到当前控制选择中。

本项目实现的是轻量级 NMPC。它不调用 CasADi、acados 或 Ipopt 等通用非线性优化器，而是在 `guidance.py` 中通过 `nmpc_acceleration()` 枚举有限个候选加速度，再用 `_rollout_cost()` 逐个滚动仿真评分。这样的实现更贴合本项目的教学和仿真目标：公式清晰、依赖少、便于解释，也能直接复用 Python 质点模型。

## 6. NMPC 的候选控制构造

`nmpc_acceleration()` 的第一步是调用 `_candidate_accelerations()` 构造候选加速度集合。候选不是任意生成的，而是围绕 PN 趋势和有限视场追踪需求构造。设 PN 名义加速度为 `a_PN`，源码中主要包含以下几类候选。

第一类是 PN 缩放候选：

```text
a = a_PN
a = 0.55 a_PN
a = 1.25 a_PN
```

这类候选用于测试“保持原趋势、保守一点、激进一点”哪种在预测窗口内代价更低。

第二类是直接接近目标的候选。源码先构造期望拦截速度

```text
v_intercept = v_des u_LOS
```

再用一阶速度跟踪形式得到加速度：

```text
a_intercept = (v_intercept - v_p) / tau
```

然后将其与 PN 趋势混合：

```text
a = 0.75 a_PN + 0.25 a_intercept
a = 0.5 a_PN + 0.5 a_intercept
```

这类候选用于在距离代价较高时增强接近目标的趋势。

第三类是同速接近和速度匹配候选。源码中同速接近候选为

```text
v_same = ||v_t|| u_LOS
a_same = (v_same - v_p) / tau
```

速度匹配候选为

```text
a_match = (v_t - v_p) / tau
```

同速接近倾向于仍朝向目标，但速度大小接近目标速度；速度匹配则更关注接近后的相对速度减小。这两类候选有助于降低近距离飞越导致的 LOS 快速变化。

第四类是 LOS 垂直方向扰动。源码构造了与 `u_LOS` 垂直的横向方向 `lateral` 和竖向横向方向 `vertical_lateral`，并生成

```text
a = a_PN +/- 0.35 a_max lateral
a = a_PN +/- 0.25 a_max vertical_lateral
```

这类候选不是随机扰动，而是有几何意义的局部试探：当直接朝目标加速会使 LOS 角过快增大时，适当绕行可能让目标继续留在视场内。

所有候选最后都会经过 `clamp_norm(candidate, a_max)` 限幅，保证 NMPC 输出仍满足追踪机最大加速度约束。

## 7. NMPC 的预测模型与代价函数

每个候选加速度都会在 `_rollout_cost()` 中进行短时预测。预测窗口长度由 `GuidanceConfig.horizon_steps` 和 `GuidanceConfig.mpc_dt` 决定，默认是

```text
20 steps * 0.1 s = 2.0 s
```

在预测窗口内，候选加速度保持不变。对第 `i` 个预测步，目标位置采用常速度模型：

```text
p_t(i) = p_t(0) + v_t i dt_mpc
```

这里的 `p_t(0), v_t` 是当前可用的目标参考状态。如果目标可见，它来自真实目标；如果目标不可见，它来自 FOV 层的 last-seen 外推结果。因此，NMPC 的预测也遵守有限视场信息约束。

追踪机预测状态使用与主仿真一致的 `step_pursuer()` 更新：

```text
x_p(i+1) = F(x_p(i), a, p_t(i))
```

其中 `F` 包含加速度限幅、速度限幅、位置积分、高度限制以及 yaw/pitch 角速度限制。这保证 NMPC 评估候选控制时使用的模型和实际仿真执行模型一致。

预测过程中计算以下代价项。

终端距离代价：

```text
J_dist = ||p_t(H) - p_p(H)||
```

该项驱动追踪机在预测窗口末端接近目标。

路径距离代价：

```text
J_path = sum_i ||p_t(i) - p_p(i)||
```

该项鼓励整个预测窗口内都保持较小距离，而不是只在最后一步靠近。

FOV 违反代价：

```text
alpha_i = angle(f(psi_i, theta_i), p_t(i) - p_p(i))
J_fov = sum_i max(0, alpha_i - phi)^2
```

当 LOS 夹角没有超过半视场角时，该项为 0；一旦超过 FOV 边界，超出越多惩罚越大。平方形式会更强烈地惩罚大角度失锁。

控制能量代价：

```text
J_control = sum_i ||a||^2 dt_mpc
```

该项抑制过大的加速度指令，使轨迹更平滑、控制更节制。

控制平滑代价：

```text
J_smooth = sum_i ||a_i - a_{i-1}||^2
```

在当前实现中，一个候选在预测窗口内保持固定加速度，因此该项主要惩罚当前候选与上一控制周期实际加速度之间的突变。上一控制保存在 `GuidanceMemory.previous_acceleration` 中。

PN 趋势偏离代价：

```text
J_PN = sum_i ||a_i - a_PN||^2
```

该项保证 NMPC 仍然是 PN 的预测增强层，而不是完全脱离拦截几何的控制器。

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

当前默认权重中，距离项和 FOV 项权重较高，说明算法既要接近目标，也要主动避免目标离开视场；控制能量、平滑项和 PN 偏离项则用于抑制过激控制和异常候选。

## 8. 滚动执行机制

NMPC 并不会一次性执行整个预测窗口内的计划。`nmpc_acceleration()` 只选择当前代价最低候选的第一步加速度：

```text
a_k = argmin_a J(a)
```

随后主仿真只执行当前仿真步。下一时刻，系统会重新获取追踪机状态、目标状态或 last-seen 预测状态，再重新计算 PN 趋势、重新生成候选、重新滚动预测。这种 receding horizon 机制使算法能够持续根据最新相对位置、速度、可见性和机头朝向修正控制。

从 `compute_guidance()` 的调用链可以看出完整流程：

```text
1. fov_visibility() 计算真实目标当前是否可见。
2. _target_reference_for_fov() 生成当前可用目标参考。
3. pn_guidance() 根据目标参考计算 PN 名义加速度。
4. nmpc_acceleration() 围绕 PN 趋势生成候选并滚动评分。
5. GuidanceResult 返回加速度、可见性、LOS 角和 look_at_position。
6. step_pursuer() 执行加速度并让 yaw/pitch 转向 look_at_position。
```

这条链路体现了 PN、FOV、NMPC 三者的分工：PN 解决“如何拦截”，FOV 解决“目标是否仍可观测以及不可见时使用什么参考”，NMPC 解决“当前这一帧选哪个控制更兼顾未来 2 秒的距离、视场和控制品质”。

## 9. 该组合适合本项目的原因

`PN + FOV + NMPC` 适合 `6_Simulation`，首先是因为它符合有限视场目标追踪的物理逻辑。目标追踪不能只考虑空间距离，还必须考虑传感器朝向和可见性。PN 以相对运动几何为基础，能给出稳定的三维拦截方向；FOV 让算法在目标不可见时退化为 last-seen 预测，而不是继续使用理想目标状态；NMPC 则把未来短时间内的 FOV 违反风险加入当前控制选择。

其次，这一组合保持了较强的可解释性。PN 公式直接来自相对位置、相对速度和 LOS 角速度；FOV 判据直接来自相机前向向量和 LOS 夹角；NMPC 代价函数中的每一项都对应明确的任务目标。阅读源码时，可以从 `pn_guidance()`、`fov_visibility()`、`_target_reference_for_fov()`、`_candidate_accelerations()` 和 `_rollout_cost()` 直接追踪每一项公式的实现。

再次，当前 NMPC 实现与项目的简化动力学模型一致。预测时使用的 `step_pursuer()` 与主仿真执行时使用的是同一个函数，因此代价评估不会建立在另一套模型假设上。速度、加速度、高度和 yaw/pitch 角速度限制也都在预测中体现，这使得候选控制的评分更接近实际执行效果。

最后，轻量候选式 NMPC 避免了重型非线性规划求解器依赖。对于本项目的仿真目标而言，重点不是展示通用优化器的求解能力，而是说明在 PN 和 FOV 基础上加入短时预测后，控制器可以主动考虑“看住目标”和“平滑接近目标”之间的折中。候选集合数量有限，代价函数结构清楚，便于复现实验、调整权重和解释结果。

## 10. 小结

本文采用 `PN + FOV + NMPC` 的核心原因可以概括为三点。

PN 提供基础拦截能力。它通过相对位置、相对速度和 LOS 角速度计算三维横向加速度，并加入径向闭合项，使追踪机能够稳定地向目标接近。

FOV 提供视觉追踪约束。它用 yaw/pitch 生成机头或相机前向向量，通过 LOS 夹角判断目标是否可见；目标丢失时，只使用最后一次观测状态进行常速度外推，并降低导引增益，从而模拟有限视场传感器的信息退化。

NMPC 提供短时预测优化。它围绕 PN 趋势构造候选加速度，在 2 秒预测窗口内评估终端距离、路径距离、FOV 违反、控制能量、控制平滑性和 PN 偏离代价，并滚动执行当前最优控制。

因此，`PN + FOV + NMPC` 不是三个模块的简单叠加，而是一条逐层增强的导引链路：PN 负责追，FOV 负责看，NMPC 负责提前权衡“追得上、看得住、控得稳”。
