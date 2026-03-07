import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import Any

# ==========================================
# 0. 数据结构定义 (通用标准)
# ==========================================

@dataclass
class State:
    """通用状态类，用于在模块间传递位姿数据"""
    position: np.ndarray  # [x, y, z] (NED frame for PX4)
    velocity: np.ndarray  # [vx, vy, vz]
    speed: float | np.floating[Any]  # 标量速度，兼容numpy和python float类型

# ==========================================
# 1. 位置获取模块 (Position Acquisition Module)
# ==========================================

class TargetProvider:
    """
    目标状态提供者
    仿真模式：生成虚拟螺旋轨迹
    实机模式：应替换为读取雷达、视觉或 MAVLink 目标数据的接口
    """
    def __init__(self, start_pos, start_vel):
        self.state = State(
            position=np.array(start_pos, dtype=float),
            velocity=np.array(start_vel, dtype=float),
            speed=np.linalg.norm(start_vel)
        )
        self.traj_history = [self.state.position.copy()]

    def update(self, time, dt) -> State:
        """更新目标状态 (仿真逻辑)"""
        # 添加随机噪声
        # noise = np.random.normal(0, 5, 3) * 0.1
        noise = [0, 0, 0]   # 调试用

        # 3D 螺旋机动逻辑 (NED坐标系下，Z轴向下为正，负值为高度上升)
        self.state.velocity[0] = -20 + noise[0] * 0.1
        self.state.velocity[1] = -20 * abs(np.sin(0.5 * time)) + noise[1]
        self.state.velocity[2] = -10 * np.sin(0.5 * time) + noise[2]

        # 运动学积分
        self.state.position += self.state.velocity * dt
        self.state.speed = np.linalg.norm(self.state.velocity)

        # 记录历史
        self.traj_history.append(self.state.position.copy())
        return self.state

# ==========================================
# 2. 轨迹计算与模式选择模块 (Guidance & Logic Module)
# ==========================================

class PNGuidanceCore:
    """
    核心导引算法模块
    负责计算期望的飞行矢量，不涉及底层通信
    """
    def __init__(self, N=3.0, speed_min=1, speed_max=50, strategy='adaptive'):
        self.N = N
        self.speed_min = speed_min
        self.speed_max = speed_max
        self.strategy = strategy
        self.desired_speed = speed_min
        self.current_acc_cmd = np.zeros(3)

    def _compute_variable_speed(self, r_mag, vc, omega_mag, target_speed, current_base_speed) -> float:
        """
        内部函数，计算速度改变策略
        r_mag: 目标与追踪者之间的距离 (m)
        vc: 接近速度 (m/s)
        omega_mag: 视线角速率大小 (rad/s)
        target_speed: 目标当前速度 (m/s)
        current_base_speed: 当前追踪者基准速度 (m/s)
        :return: 期望速度 (m/s)
        """
        desired = current_base_speed

        # 1. 自适应速度策略，根据距离和机动性调整速度
        if self.strategy == 'adaptive':
            # 距离因子：距离越远越快
            dist_factor = np.clip(r_mag / 200.0, 0.5, 1.5)
            # 精度因子：视线角速率越大（机动剧烈），速度越慢以保证转弯半径
            omega_penalty = np.clip(1.0 - omega_mag * 10.0, 0.5, 1.0)

            desired = current_base_speed * dist_factor * omega_penalty
            # 接近速度补偿
            if vc < target_speed * 0.8:
                desired *= 1.2 # 如果追不上，加速

        # 2. 速度分段策略，根据距离分段调整速度        可能速度策略欠佳，极端情况下会出现过慢或过快！
        elif self.strategy == 'distance':
            # 远距离高速
            if r_mag > 300:
                desired = self.speed_max
            # 近距离降速以提高拦截精度
            elif r_mag < 100:
                desired = target_speed * 0.5
            # 中距离线性插值
            else:
                desired = np.interp(r_mag, [100, 300], [target_speed * 0.5, self.speed_max])

        # 3. 节能策略，在保证拦截的前提下尽量节省能量
        elif self.strategy == 'energy':
            # 估算拦截时间
            if vc > 0:
                estimated_intercept_time = r_mag / vc
                # 如果拦截时间充裕，可以降低速度
                if estimated_intercept_time > 5.0:
                    desired = target_speed * 1.2
                elif estimated_intercept_time > 2.0:
                    desired = target_speed * 1.5
                else:
                    # 接近拦截点时加速确保成功
                    desired = self.speed_max
            else:
                # 如果没有接近，全速追击
                desired = self.speed_max

        # 4. 追击策略，始终保持比目标快一定比例
        elif self.strategy == 'pursuit':
            # 始终保持比目标快一定比例
            desired = target_speed * 1.5

        return np.clip(desired, self.speed_min, self.speed_max)

    def calculate_guidance(self, tracker_state: State, target_state: State, dt: float) -> tuple:
        """
        计算下一时刻的控制指令
        返回: (position_setpoint, velocity_setpoint_vector)
        """
        # 1. 相对运动状态
        R_vec = target_state.position - tracker_state.position  # Range Vector 相对位置矢量
        V_rel = target_state.velocity - tracker_state.velocity  # Relative Velocity	相对速度矢量

        R_mag = np.linalg.norm(R_vec)   # Range Magnitude 距离 (标量)
        R_sq = np.dot(R_vec, R_vec)     # Range Squared 距离平方 (标量)

        # 容错：防止除零
        if R_mag < 0.1: R_mag = 0.1
        if R_sq < 0.01: R_sq = 0.01

        # 2. 视线角速率 (Omega)
        Omega = np.cross(R_vec, V_rel) / R_sq   # 视线角速率矢量
        Omega_mag = np.linalg.norm(Omega)       # 视线角速率标量

        # 3. 接近速度 (Vc)
        Vc = -np.dot(R_vec, V_rel) / R_mag

        # 4. 速度策略计算
        self.desired_speed = self._compute_variable_speed(
            R_mag, Vc, Omega_mag, target_state.speed, tracker_state.speed
        )

        # 5. 3D 比例导引加速度指令 (Acceleration Command)
        # ac = N * Vc * (Omega x R_unit)
        R_unit = R_vec / R_mag      # 单位化的相对位置矢量
        ac_vec = self.N * Vc * np.cross(Omega, R_unit)
        self.current_acc_cmd = ac_vec # 保存用于调试

        # 6. 计算期望速度矢量 (Velocity Setpoint)
        # 这里的逻辑是：当前速度矢量 + 期望加速度 * dt
        vel_cmd_raw = tracker_state.velocity + ac_vec * dt

        # 7. 归一化并应用期望速率
        # 提取 vel_cmd_raw 的方向，然后强制将其长度（速率）设置为 desired_speed
        # 这意味着：PNG 决定飞行的方向，速度策略决定飞行的快慢。
        vel_cmd_norm = np.linalg.norm(vel_cmd_raw)
        if vel_cmd_norm > 0:
            vel_cmd_final = (vel_cmd_raw / vel_cmd_norm) * self.desired_speed
        else:
            vel_cmd_final = R_unit * self.desired_speed # 初始或静止时直接指向目标

        # 8. 计算期望位置 (Position Setpoint)
        # 简单预测：下一位置 = 当前位置 + 期望速度 * dt
        pos_cmd = tracker_state.position + vel_cmd_final * dt

        # 注意：不再计算 yaw，让飞控自动处理偏航
        # 如需手动控制 yaw，可取消以下注释：
        # yaw_setpoint = math.degrees(math.atan2(vel_cmd_final[1], vel_cmd_final[0]))

        return pos_cmd, vel_cmd_final

# ==========================================
# 3. 导航指令输出模块 (Navigation Output Module - MAVSDK Standard)
# ==========================================

class MavsdkCommandGenerator:
    """
    负责将计算出的向量转换为 MAVSDK (PX4) 可接受的格式
    参考 MAVSDK-Python Offboard 插件
    """
    @staticmethod
    def format_offboard_position_velocity_ned(position_vec, velocity_vec):
        """
        生成符合 mavsdk.offboard.PositionNed 和 VelocityNed 的数据结构
        注意：PX4 使用 NED (北东地) 坐标系。
        """
        pos_cmd = {
            "north_m": position_vec[0],
            "east_m":  position_vec[1],
            "down_m":  position_vec[2],
        }
        vel_cmd = {
            "north_m_s": velocity_vec[0],
            "east_m_s":  velocity_vec[1],
            "down_m_s":  velocity_vec[2],
        }
        return pos_cmd, vel_cmd

    def send_command(self, position_vec, velocity_vec, simulated=True):
        """
        发送指令接口
        :param position_vec: NED 位置向量 [north, east, down]
        :param velocity_vec: NED 速度向量 [vn, ve, vd]
        :param simulated: True=仅打印/返回, False=调用真实MAVSDK接口(伪代码)
        """
        pos_cmd, vel_cmd = self.format_offboard_position_velocity_ned(position_vec, velocity_vec)

        if simulated:
            # 在仿真中我们只返回数据，不进行网络IO
            return pos_cmd, vel_cmd
        else:
            # TODO: 实机部署时取消注释
            # 仿照 offboard_from_csv.py 的方式，使用 VelocityNed 代替 VelocityNedYaw
            # await drone.offboard.set_velocity_ned(
            #     VelocityNed(vel_cmd["north_m_s"], vel_cmd["east_m_s"], vel_cmd["down_m_s"])
            # )
            pass

# ==========================================
# 4. 仿真环境集成 (Integration)
# ==========================================

class SimulationEnvironment:
    def __init__(self):
        # 初始化模块
        self.target = TargetProvider([500, 500, 500], [0, 0, 0])

        # 追踪者初始状态
        start_vel = np.array([1.0, 0, 0]) # 初始给予微小速度避免奇异值
        self.tracker_state = State(
            position=np.array([0.0, 0.0, 0.0]),
            velocity=start_vel,
            speed=np.linalg.norm(start_vel))

        # 算法模块
        self.guidance = PNGuidanceCore(N=4.0, strategy='adaptive')

        # 输出模块
        self.interface = MavsdkCommandGenerator()

        # 记录数据
        self.tracker_traj = []
        self.speed_log = []
        self.time_log = []

    def run(self, max_time=120.0, dt=0.05):
        time = 0.0
        steps = int(max_time / dt)
        hit = False

        print(f"开始仿真 (dt={dt}s, Strategy={self.guidance.strategy})...")

        for _ in range(steps):
            # 1. 更新目标 (Sensor Step)
            target_state = self.target.update(time, dt)

            # 2. 判断拦截 (Hit Check)
            dist = np.linalg.norm(target_state.position - self.tracker_state.position)
            if dist < 1.0:
                print(f"目标已拦截于 t={time:.2f}s! 最终距离 d={dist:.2f}m")
                hit = True
                break

            # 3. 计算导引指令 (Guidance Step)
            # 输出为 NED 位置设定点和速度设定点（不含 yaw）
            pos_cmd, vel_cmd = self.guidance.calculate_guidance(
                self.tracker_state, target_state, dt
            )

            # 4. 生成 MAVSDK 指令 (Command Output Step)
            mav_cmd = self.interface.send_command(pos_cmd, vel_cmd, simulated=True)

            # 5. 更新追踪者动力学 (Simulation Physics Step)
            # 在实机中，这一步由 PX4 飞控完成，我们只需要发送 mav_cmd
            # 模拟：假设完美响应，或者加入惯性平滑
            self._apply_physics(vel_cmd, dt)

            # 6. 数据记录
            self.tracker_traj.append(self.tracker_state.position.copy())
            self.speed_log.append(self.tracker_state.speed)
            self.time_log.append(time)

            time += dt

        if not hit:
            print("仿真结束：未拦截目标")

        self._plot()

    def _apply_physics(self, vel_setpoint, dt):
        """简单的动力学模拟，用于更新仿真中的自身位置"""
        # 这里模拟一个简单的惯性响应
        k = 0.8 # 响应系数 (1.0 = 完美响应)
        actual_vel = self.tracker_state.velocity * (1-k) + vel_setpoint * k

        self.tracker_state.velocity = actual_vel
        self.tracker_state.position += actual_vel * dt
        self.tracker_state.speed = np.linalg.norm(actual_vel)

    def _plot(self):
        t_traj = np.array(self.target.traj_history)
        m_traj = np.array(self.tracker_traj)

        fig = plt.figure(figsize=(12, 5))

        # 3D 轨迹
        ax1 = fig.add_subplot(1, 2, 1, projection='3d')
        ax1.plot(t_traj[:,0], t_traj[:,1], t_traj[:,2], 'b--', label='目标')
        ax1.plot(m_traj[:,0], m_traj[:,1], m_traj[:,2], 'r-', label='无人机 (PN)')
        ax1.set_title("拦截轨迹 (NED/XYZ)")
        ax1.legend()

        # 速度曲线
        ax2 = fig.add_subplot(1, 2, 2)
        ax2.plot(self.time_log, self.speed_log, 'g-')
        ax2.set_title("无人机速度曲线 (可变策略)")
        ax2.set_ylabel("速度 (m/s)")
        ax2.set_xlabel("时间 (秒)")
        ax2.grid(True)

        plt.tight_layout()
        plt.show()

# ==========================================
if __name__ == "__main__":
    sim = SimulationEnvironment()
    sim.run()
