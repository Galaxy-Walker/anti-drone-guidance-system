#!/usr/bin/env python3

"""
本示例展示了如何使用手动控制插件。

注意：为了降低复杂度，本示例中的手动输入取自一个测试集。
手动输入可以通过第三方 Python 扩展从操纵杆等设备接收。

注意：启用手动输入前并非必须起飞无人机。
可以发送正油门输入使其离地。
本示例中使用起飞是为了降低复杂度。
"""

import asyncio
import random
from mavsdk import System

# Test set of manual inputs. Format: [roll, pitch, throttle, yaw]
# 手动输入的测试集。格式：[横滚，俯仰，油门，偏航]
manual_inputs = [
    [0, 0, 0.5, 0],  # no movement
    [-1, 0, 0.5, 0],  # minimum roll
    [1, 0, 0.5, 0],  # maximum roll
    [0, -1, 0.5, 0],  # minimum pitch
    [0, 1, 0.5, 0],  # maximum pitch
    [0, 0, 0.5, -1],  # minimum yaw
    [0, 0, 0.5, 1],  # maximum yaw
    [0, 0, 1, 0],  # max throttle
    [0, 0, 0, 0],  # minimum throttle
]


async def manual_controls():
    """连接无人机并输入手动控制的主函数"""
    # 连接到模拟器
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    # 这是一个异步循环，只要检测到基于mavlink的无人机连接，
    # 就跳出循环，继续执行。
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!")
            break

    # Checking if Global Position Estimate is ok
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position state is good enough for flying.")
            break

    # 解锁后设置手动控制输入
    await drone.manual_control.set_manual_control_input(float(0), float(0), float(0.5), float(0))

    # Arming the drone
    print("-- Arming")
    await drone.action.arm()

    # Takeoff the vehicle
    print("-- Taking off")
    await drone.action.takeoff()
    await asyncio.sleep(5)

    # set the manual control input after arming
    await drone.manual_control.set_manual_control_input(float(0), float(0), float(0.5), float(0))

    # start manual control
    print("-- Starting manual control")
    await drone.manual_control.start_position_control()

    while True:
        # 从测试列表中通过random获取一个输入
        # 警告 —— 如果你的机器运气不够好，可能会坠毁
        input_index = random.randint(0, len(manual_inputs) - 1)
        input_list = manual_inputs[input_index]

        # 获取横滚轴的当前状态（介于 -1 和 1 之间）
        roll = float(input_list[0])
        # 获取俯仰轴的当前状态（介于 -1 和 1 之间）
        pitch = float(input_list[1])
        # 获取油门的当前状态
        # （介于 -1 和 1 之间，但期望介于 0 和 1 之间）
        throttle = float(input_list[2])
        # 获取偏航轴的当前状态（介于 -1 和 1 之间）
        yaw = float(input_list[3])

        await drone.manual_control.set_manual_control_input(pitch, roll, throttle, yaw)

        await asyncio.sleep(0.1)


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(manual_controls())
