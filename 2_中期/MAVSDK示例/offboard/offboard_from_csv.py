"""
脚本：offboard_from_csv.py
作者：Alireza Ghaderi
联系方式：p30planets@gmail.com
GitHub：github.com/alireza787b
最后更新时间：2023年5月31日

该脚本通过CSV文件中定义的轨迹，在离机模式（offboard mode）下控制无人机。
它建立与无人机的连接，从CSV文件读取轨迹数据，并命令无人机沿轨迹飞行。
在轨迹结束后，无人机将返回其起始位置并着陆。

在运行此脚本之前，请确保无人机已正确配置为支持离机控制。
根据您的应用需求，可按需调整脚本中的时间分辨率（timeStep = 0.1秒）。

先决条件：
- MAVSDK 库（请参阅 MAVSDK 文档以获取安装说明）
- 无人机已设置为支持离机控制
- 无人机具备有效的全局位置估计

使用方法：
python offboard_from_csv.py

输入：
- 与本脚本同目录下的 CSV 文件 “active.csv”。该文件包含轨迹数据，其结构如下：
    - t：时间（单位：秒）
    - px, py, pz：NED 坐标系（北-东-下）中的位置
    - vx, vy, vz：NED 坐标系中的速度
    - ax, ay, az：NED 坐标系中的加速度
    - mode：表示无人机运动不同阶段的模式代码（详见下方“模式代码”说明）
- 位于名为 “trajectory_plot” 的目录中的图像 “trajectory_plot.png”，
  用于提供轨迹的可视化预览。

输出：
- 无人机按照CSV文件中定义的轨迹进行飞行，完成后返回起始点并着陆。

示例用法：
1. 确保无人机已连接至系统并具备有效的全局位置估计。
2. 将轨迹数据放入 “active.csv” 文件中。
3. 运行脚本。

模式代码：
0: "在地面"
10: "初始爬升阶段"
20: "爬升后悬停"
30: "飞往起点"
40: "在起点悬停"
50: "飞往机动起始点"
60: "在机动起始点悬停"
70: "执行机动动作（轨迹飞行）"
80: "在轨迹终点悬停"
90: "返回起始坐标"
100: "着陆"

附加信息：
- 如需有关使用 MAVSDK 库控制无人机的分步教程，请参考 GitHub 仓库（alireza787b）中提供的视频教程。
- 更复杂的无人机表演项目以及多机协同模拟示例可在 mavsdk_drone_show 仓库中找到：
  https://github.com/alireza787b/mavsdk_drone_show

注意：
- 截至2023年5月，set_position_velocity_acceleration_ned 功能尚未包含在通过 pip3 安装的 mavsdk-Python 默认构建版本中。
  如果您需要输入加速度，请自行编译 MAVSDK
"""

import asyncio
import csv
import io
import anyio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, VelocityNedYaw
from mavsdk.offboard import OffboardError
from mavsdk.telemetry import LandedState

# print("=====检查脚本路径 path =======")
# import sys
# print(sys.path)
# print("================================")

# 根据时间查找当前航点
def get_current_waypoint(waypoints, time):
    return next((wp for wp in waypoints if time <= wp[0]), None)


async def run():
    # 定义一个字典，将模式代码映射到其描述
    mode_descriptions = {
        0: "On the ground",        # 0: "在地面"
        10: "Initial climbing state",              # 10: "初始爬升阶段"
        20: "Initial holding after climb",         # 20: "爬升后悬停"
        30: "Moving to start point",               # 30: "飞往起点"
        40: "Holding at start point",              # 40: "在起点悬停"
        50: "Moving to maneuvering start point",   # 50: "飞往机动起始点"
        60: "Holding at maneuver start point",     # 60: "在机动起始点悬停"
        70: "Maneuvering (trajectory)",            # 70: "执行机动动作（轨迹飞行）"
        80: "Holding at the end of the trajectory coordinate",  # 80: "在轨迹终点悬停"
        90: "Returning to home coordinate",        # 90: "返回起始坐标"
        100: "Landing",     # 100: "着陆"
    }

    # Connect to the drone
    drone = System()
    print("等待无人机连接...")
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    # Wait for the drone to connect
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- 无人机已连接！")
            break

    # ==================== 新增/修改的代码区域 ====================
    print("-- 为SITL（仿真）配置参数")
    try:
        # 1. 允许无遥控器模式
        await drone.param.set_param_int("COM_RC_IN_MODE", 1)
        print("-- 参数设置成功")

    except Exception as e:
        print(f"警告: 无法设置参数: {e}")

    # Wait for the drone to have a global position estimate
    print("等待无人机具备全局位置估计...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- 全局位置估计正常！")
            break

    # Arm the drone
    print("-- 解锁")
    await drone.action.arm()

    # Set the```python
    # Set the initial setpoint
    print("-- 设置初始设定点")
    startSetpoint = PositionNedYaw(0.0, 0.0, 0.0, 0.0)
    await drone.offboard.set_position_ned(startSetpoint)

    # Start offboard mode
    print("-- 启动外部（offboard）模式")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"启动外部（offboard）模式失败，错误代码: {error._result.result}")
        print("-- 解锁")
        await drone.action.disarm()
        return

    # Read data from the CSV file
    async with await anyio.open_file("2_中期/MAVSDK示例/offboard/active.csv", "r", newline="") as csvfile:
        content = await csvfile.read()
    waypoints = [
        (
            float(row["t"]),
            float(row["px"]),
            float(row["py"]),
            float(row["pz"]),
            float(row["vx"]),
            float(row["vy"]),
            float(row["vz"]),
            float(row["ax"]),
            float(row["ay"]),
            float(row["az"]),
            int(row["mode"]),
        )
        for row in csv.DictReader(io.StringIO(content))
    ]

    print("-- Performing trajectory")
    total_duration = waypoints[-1][0]
    # 总时长是最后一个航点的时间
    t = 0  # 初始化时间变量
    last_mode = 0
    while t <= total_duration:
        current_waypoint = get_current_waypoint(waypoints, t)
        if current_waypoint is None:
            # 到达轨迹终点
            break

        position = current_waypoint[1:4]
        velocity = current_waypoint[4:7]
        _acceleration = current_waypoint[7:10]  # unused
        mode_code = current_waypoint[-1]
        if last_mode != mode_code:
            # 打印模式编号及其描述
            print(" Mode number: " + f"{mode_code}, Description: {mode_descriptions[mode_code]}")
            last_mode = mode_code
        # set_position_velocity_acceleration_ned 目前在使用 pip3 安装 MAVSDK-Python 的默认版本中尚不可用。
        # 如果你需要输入加速度参数，
        # 你应该自行编译构建 MAVSDK。
        await drone.offboard.set_position_velocity_ned(
            PositionNedYaw(*position, current_waypoint[10]),
            VelocityNedYaw(*velocity, current_waypoint[10]),
        )
        # await drone.offboard.set_position_velocity_acceleration_ned(
        #     PositionNedYaw(*position, current_waypoint[10]),
        #     VelocityNedYaw(*velocity, current_waypoint[10]),
        #     AccelerationNed(*acceleration, current_waypoint[10])
        # )

        timeStep = 0.1
        await asyncio.sleep(timeStep)  # 时间分辨率为0.1秒
        t += timeStep

    print("-- 心型形状已完成")

    print("-- 降落")
    await drone.action.land()

    async for state in drone.telemetry.landed_state():
        if state == LandedState.ON_GROUND:
            break

    print("-- 停止外部（offboard）操作")
    try:
        await drone.offboard.stop()
    except Exception as error:
        print(f"停止外部（offboard）操作失败，错误信息: {error}")

    print("-- 解锁")
    await drone.action.disarm()


if __name__ == "__main__":
    # 运行 asyncio 循环
    asyncio.run(run())
