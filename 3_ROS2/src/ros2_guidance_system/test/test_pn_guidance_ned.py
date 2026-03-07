"""
比例导引 NED 坐标系验证测试

场景：
  - 目标静止在 [50, 50, -30]（NED: 北50m, 东50m, 上方30m）
  - 追踪者从 [, , ] 起飞，初始速度 [, , ]
  - 仿真循环运行导引算法，验证追踪者能否收敛到目标位置

NED 坐标系说明：
  x → 北, y → 东, z → 地（向下为正）
  因此 z = -30 表示目标在地面以上 30m 处

使用方式：
1. 进入功能包目录
cd ~/Project/Anti-Drone/code/3_ROS2/src/ros2_guidance_system
2. 在这里运行测试文件
python3 test/test_pn_guidance_ned.py
"""

import sys
import numpy as np

# 确保能直接运行此脚本，必须在包的根目录下执行
sys.path.insert(0, ".")

from ros2_guidance_system.pn_guidance_core import PNGuidanceCore, State, SPEED_STRATEGIES


def run_simulation(
        strategy: str = "adaptive",
        max_steps: int = 2000,
        dt: float = 0.05,
        target_pos=np.array([0.0, 0.0, -30.0]),
        target_vel=np.array([0.0, 0.0, 0.0]),
        tracker_pos=np.array([0.0, 0.0, 0.0]),
        tracker_vel=np.array([0.0, 0.0, 0.0])):
    """
    运行单次仿真，返回轨迹和最终距离。
    """
    guidance = PNGuidanceCore(
        N=4.0,
        speed_min=2.0,
        speed_max=30.0,
        strategy=strategy,
        log_func=lambda msg: None,  # 静默日志
    )

    target_state = State(position=target_pos, velocity=target_vel, speed=float(np.linalg.norm(target_vel)))

    trajectory = [tracker_pos.copy()]
    distances = []

    for step in range(max_steps):
        tracker_state = State(
            position=tracker_pos.copy(),
            velocity=tracker_vel.copy(),
            speed=float(np.linalg.norm(tracker_vel)),
        )

        result = guidance.calculate_guidance(tracker_state, target_state, dt)

        if step % 50 == 0:
            print(f"位置设定点: [{result.pos_cmd[0]:.1f}, {result.pos_cmd[1]:.1f}, {result.pos_cmd[2]:.1f}]")
            print(f"速度设定点: [{result.vel_cmd[0]:.1f}, {result.vel_cmd[1]:.1f}, {result.vel_cmd[2]:.1f}]")

        # 用导引结果更新追踪者状态（简化动力学：直接采用速度指令）
        tracker_vel = result.vel_cmd.copy()
        tracker_pos = tracker_pos + tracker_vel * dt

        distances.append(result.distance)
        trajectory.append(tracker_pos.copy())

        # 接近到 1m 以内视为拦截成功
        if result.distance < 1.0:
            return True, step, distances, np.array(trajectory), strategy

    return False, max_steps, distances, np.array(trajectory), strategy


def main():
    # 定义测试参数
    # NED 坐标系下的目标和追踪者初始状态
    target_pos = np.array([50.0, 50.0, -30.0])
    target_vel = np.array([0.0, 0.0, 0.0])       # 静止目标
    tracker_pos = np.array([0.0, 0.0, -40.0])   # 追踪者起点
    tracker_vel = np.array([0.0, 0.0, 1.0])     # 追踪者初始速度

    print("=" * 70)
    print("比例导引 NED 坐标系验证测试")
    print(f"目标位置: {target_pos} , 目标速度: {target_vel}")
    print(f"追踪者起点: {tracker_pos}, 初始速度: {tracker_vel}")
    print("=" * 70)

    all_pass = True

    for strategy_name in SPEED_STRATEGIES:
        print(f"\n--- 策略: {strategy_name} ---")

        success, steps, distances, trajectory, _ = run_simulation(strategy=strategy_name, target_pos=target_pos, target_vel=target_vel, tracker_pos=tracker_pos, tracker_vel=tracker_vel)

        final_dist = distances[-1] if distances else float("inf")
        final_pos = trajectory[-1]

        print(f"  拦截成功: {'是' if success else '否'}")
        print(f"  仿真步数: {steps}")
        print(f"  最终距离: {final_dist:.3f} m")
        print(f"  最终位置: [{final_pos[0]:.2f}, {final_pos[1]:.2f}, {final_pos[2]:.2f}]")
        print(f"  最小距离: {min(distances):.3f} m")

        if not success:
            print(f"  ⚠ 未能在限定步数内拦截目标")
            all_pass = False

    # ---- 额外测试：策略动态切换 ----
    print(f"\n--- 额外测试: 动态切换策略 ---")
    guidance = PNGuidanceCore(N=4.0, strategy="adaptive", log_func=lambda m: None)
    guidance.set_strategy("pursuit")
    assert guidance.strategy_name == "pursuit", "策略切换失败"
    print(f"  ✓ 策略动态切换正常")

    # ---- 额外测试：无效策略应报错 ----
    print(f"\n--- 额外测试: 无效策略检测 ---")
    try:
        PNGuidanceCore(strategy="invalid_strategy")
        print(f"  ✗ 未抛出异常")
        all_pass = False
    except ValueError as e:
        print(f"  ✓ 正确抛出异常: {e}")

    # ---- 汇总 ----
    print("\n" + "=" * 70)
    if all_pass:
        print("所有测试通过 ✓")
    else:
        print("存在失败的测试项 ✗")
    print("=" * 70)

    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
