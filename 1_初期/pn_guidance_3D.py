import numpy as np
import matplotlib.pyplot as plt

class ProportionalNavigation3D:
    def __init__(self, t_pos, m_pos, t_vel, m_speed, N=3.0, dt=0.01):
        """
        初始化3D仿真参数
        :param t_pos: 目标初始位置 [x, y, z]
        :param m_pos: 追踪无人机初始位置 [x, y, z]
        :param t_vel: 目标速度向量 [vx, vy, vz]
        :param m_speed: 追踪无人机速率 (标量)
        :param N: 比例导引系数 (通常在 3-5 之间)
        :param dt: 仿真时间步长
        """
        self.t_pos = np.array(t_pos, dtype=float)
        self.m_pos = np.array(m_pos, dtype=float)
        self.t_vel = np.array(t_vel, dtype=float)
        self.m_speed = m_speed
        self.N = N
        self.dt = dt

        # 初始化追踪者的速度向量（假设初始指向目标）
        diff = self.t_pos - self.m_pos
        dist = np.linalg.norm(diff)
        if dist > 0:
            self.m_vel = diff / dist * m_speed
        else:
            self.m_vel = np.array([m_speed, 0, 0], dtype=float)

        # 用于存储轨迹数据以便绘图
        self.t_traj = [self.t_pos.copy()]
        self.m_traj = [self.m_pos.copy()]

    def update_target_state(self, time):
        """
        更新目标状态：3D机动
        """
        # 目标做3D螺旋机动，并加入随机噪声
        noise = np.random.normal(0, 5, 3)     # 均值0，标准差5

        # 简单的3D机动策略
        # X轴: 匀速
        # Y轴: 正弦机动
        # Z轴: 余弦机动 (形成螺旋)
        self.t_vel[0] = -20 + noise[0] * 0.1
        self.t_vel[1] = -30 * np.sin(0.5 * time) + noise[1]
        self.t_vel[2] = 10 * np.cos(0.5 * time) + noise[2]

        self.t_pos += self.t_vel * self.dt
        self.t_traj.append(self.t_pos.copy())

    def update_missile_state(self):
        """
        核心：使用3D比例导引法更新追踪无人机状态
        """
        # 1. 计算相对位置和相对速度
        R_vec = self.t_pos - self.m_pos       # 相对位置向量 (Line of Sight Vector)
        V_rel = self.t_vel - self.m_vel       # 相对速度向量

        R_mag = np.linalg.norm(R_vec)         # 距离
        R_sq = np.dot(R_vec, R_vec)           # 距离平方

        # 如果距离非常近，视为命中
        if R_mag < 1.0:
            return True # Hit

        # 2. 计算视线旋转向量 (Rotation Vector of LOS)
        # Omega = (R x V) / R^2
        Omega = np.cross(R_vec, V_rel) / R_sq

        # 3. 计算接近速度 (Closing Velocity, Vc)
        # Vc = - (R . V) / |R|
        Vc = -np.dot(R_vec, V_rel) / R_mag

        # 4. 计算期望的加速度向量 (Acceleration Command)
        # 3D PNG 公式: ac = N * Vc * (Omega x R_unit)
        # 这里的加速度方向垂直于视线 (True Proportional Navigation)
        R_unit = R_vec / R_mag
        ac_vec = self.N * Vc * np.cross(Omega, R_unit)

        # 5. 更新追踪者的速度向量
        # 先更新速度向量 V_new = V_old + a * dt
        self.m_vel += ac_vec * self.dt

        # 6. 速度约束 (保持速率恒定)
        # 实际导弹会有阻力和推力，这里简化为恒定速率模型，只改变方向
        current_speed = np.linalg.norm(self.m_vel)
        if current_speed > 0:
            self.m_vel = self.m_vel / current_speed * self.m_speed

        # 7. 更新位置
        self.m_pos += self.m_vel * self.dt
        self.m_traj.append(self.m_pos.copy())

        return False # Not hit yet

    def run_simulation(self, max_time=1000):
        """运行仿真循环"""
        time = 0
        hit = False
        steps = int(max_time / self.dt)

        for _ in range(steps):
            self.update_target_state(time)
            hit = self.update_missile_state()
            if hit:
                print(f"目标在 t={time:.2f}s 时被捕获！")
                break
            time += self.dt

        if not hit:
            print(f"仿真结束，未能在 {max_time}s 内捕获目标。")

        return hit

    def plot_results(self):
        """绘制3D轨迹图"""
        t_traj = np.array(self.t_traj)
        m_traj = np.array(self.m_traj)

        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')

        # 绘制轨迹
        ax.plot(t_traj[:, 0], t_traj[:, 1], t_traj[:, 2], 'b--', label='Target', linewidth=2)
        ax.plot(m_traj[:, 0], m_traj[:, 1], m_traj[:, 2], 'r-', label='Tracker (3D PNG)', linewidth=2)

        # 绘制起点和终点
        ax.scatter(t_traj[0, 0], t_traj[0, 1], t_traj[0, 2], c='blue', marker='o', s=50, label='Start')
        ax.scatter(m_traj[0, 0], m_traj[0, 1], m_traj[0, 2], c='red', marker='o', s=50)

        # 终点
        ax.scatter(m_traj[-1, 0], m_traj[-1, 1], m_traj[-1, 2], c='black', marker='x', s=100, label='Intercept Point')

        # 连接一些连线表示视线 (Line of Sight)
        step = max(1, len(m_traj) // 15)
        for i in range(0, len(m_traj), step):
            if i < len(t_traj):
                ax.plot([m_traj[i, 0], t_traj[i, 0]],
                        [m_traj[i, 1], t_traj[i, 1]],
                        [m_traj[i, 2], t_traj[i, 2]],
                        'g-', alpha=0.2, linewidth=0.5)

        ax.set_title(f'3D Proportional Navigation Guidance (N={self.N})')
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.set_zlabel('Z Position (m)')
        ax.legend()

        # 设置坐标轴比例一致 (Matplotlib 3D 默认不一致，需要手动调整)
        # 获取各轴范围
        max_range = np.array([t_traj[:,0].max()-t_traj[:,0].min(),
                              t_traj[:,1].max()-t_traj[:,1].min(),
                              t_traj[:,2].max()-t_traj[:,2].min(),
                              m_traj[:,0].max()-m_traj[:,0].min(),
                              m_traj[:,1].max()-m_traj[:,1].min(),
                              m_traj[:,2].max()-m_traj[:,2].min()]).max() / 2.0

        mid_x = (t_traj[:,0].max()+t_traj[:,0].min()) * 0.5
        mid_y = (t_traj[:,1].max()+t_traj[:,1].min()) * 0.5
        mid_z = (t_traj[:,2].max()+t_traj[:,2].min()) * 0.5

        # 这种方式在某些matplotlib版本可能不完全生效，但尽量尝试
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)

        plt.show()

# --- 主程序 ---
if __name__ == "__main__":
    # 参数设定
    # 目标初始位置 [x, y, z]
    target_start = [500, 500, 500]
    # 追踪者初始位置 [x, y, z]
    tracker_start = [0, 0, 0]

    # 目标初始速度向量 (会被update覆盖，这里只是占位)
    target_velocity = [0, 0, 0]

    # 追踪者速率
    tracker_speed = 40

    # 实例化并运行
    sim = ProportionalNavigation3D(target_start, tracker_start, target_velocity, tracker_speed, N=4.0)
    sim.run_simulation()
    sim.plot_results()
