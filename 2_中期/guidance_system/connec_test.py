import asyncio
from mavsdk import System

async def test_sim():
    drone = System()

    # 【修正1】使用 udpin:// 消除警告
    # # localhost:14540 表示监听本地的 14540 端口等待飞控数据
    # print("正在初始化 MAVSDK...")
    # await drone.connect(system_address="udpin://localhost:14540")

    # 0.0.0.0 表示监听所有网络接口，解决 inet_pton 报错，同时兼容 WSL 环境
    await drone.connect(system_address="udpin://0.0.0.0:14540")
    print("正在等待仿真飞控连接...")

    # 【修正2】不再尝试从 state 中获取 uuid，仅判断连接状态
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("连接成功！(Drone Discovered)")
            break

    # 如果你确实想要获取 UUID，需要使用 info 模块 (这是正确的方法)
    try:
        uuid = await drone.info.get_identification()
        print(f"获取到硬件 ID: {uuid.hardware_uid}")
    except:
        print("跳过获取 UUID")

    print("等待获取全球定位 (GPS)...")
    # 这一步在仿真中可能需要几秒钟，因为模拟的 GPS 需要时间搜星
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("GPS 定位锁定！")
            break

    print("解锁 (Arming)...")
    try:
        await drone.action.arm()
    except Exception as e:
        print(f"解锁失败: {e}")
        return

    print("起飞 (Takeoff)...")
    await drone.action.takeoff()

    # 悬停 10 秒
    for i in range(10):
        print(f"悬停中... {10-i}")
        await asyncio.sleep(1)

    print("降落 (Landing)...")
    await drone.action.land()

    # 等待直到不再在空中 (可选)
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("已着陆。")
            break

if __name__ == "__main__":
    try:
        asyncio.run(test_sim())
    except KeyboardInterrupt:
        # 允许通过 Ctrl+C 安全退出
        pass
