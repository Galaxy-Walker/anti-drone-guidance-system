#!/usr/bin/env python3

# This example shows how to use the follow me plugin

import asyncio
from mavsdk import System
from mavsdk.follow_me import Config, TargetLocation


follow_height = 8.0  # in meters
# distance between drone and target
follow_distance = 2.0  # in meters
responsiveness = 0.02
altitude_mode = Config.FollowAltitudeMode.TARGET_GPS
max_follow_vel = 10
# direction relative to the target
follow_angle_deg = 0

# This list contains fake location coordinates
# (These coordinates are obtained from mission.py example)
fake_location = [
    [47.398039859999997, 8.5455725400000002],
    [47.398036222362471, 8.5450146439425509],
    [47.397825620791885, 8.5450092830163271],
]


async def run():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    # 等待无人机连接
    # 这是一个异步循环，只要检测到 state.is_connected 变为
    # True，就跳出循环，继续执行。
    print("等待无人机连接...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- 已连接到无人机！")
            break

    # 检查 GPS 健康状态
    # 由于跟随模式依赖 GPS 定位，代码必须等待直到
    # 无人机的 GPS 信号良好并且确定了“Home”点（返航点）
    print("等待无人机获得全球定位估计...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("“-- 全球位置估计正常” =>")
            break

    # “为无人机启动”
    print("-- 启动无人机")
    await drone.action.arm()

    # 跟随模式在启动前需要进行一些配置
    conf = Config(
        follow_height,
        follow_distance,
        responsiveness,
        altitude_mode,
        max_follow_vel,
        follow_angle_deg,
    )
    await drone.follow_me.set_config(conf)

    print("-- 起飞")
    await drone.action.takeoff()
    await asyncio.sleep(8)
    print("-- 启动跟随模式")
    await drone.follow_me.start()
    await asyncio.sleep(8)

    # 这个 for 循环从 fake_location 列表中提供
    # 虚假的坐标，以使"跟随模式"能够运行
    # 不过，在模拟器中这样做意义不大
    target = TargetLocation(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    for latitude, longitude in fake_location:
        target.latitude_deg = latitude
        target.longitude_deg = longitude
        target.absolute_altitude_m = 480.0
        target.velocity_x_m_s = 0.0
        target.velocity_y_m_s = 0.0
        target.velocity_z_m_s = 0.0
        print("-- 跟随目标")
        await drone.follow_me.set_target_location(target)
        await asyncio.sleep(2)

    # 停止跟随模式
    print("-- 停止跟随模式")
    await drone.follow_me.stop()
    await asyncio.sleep(5)

    print("-- 降落")
    await drone.action.land()


if __name__ == "__main__":
    # 运行异步循环
    asyncio.run(run())
