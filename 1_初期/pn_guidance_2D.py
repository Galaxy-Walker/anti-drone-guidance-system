import numpy as np
import matplotlib.pyplot as plt

class ProportionalNavigation:
    def __init__(self, t_pos, m_pos, t_vel, m_speed, N=3.0, dt=0.01):
        """
        初始化仿真参数
        :param t_pos: 目标初始位置 [x, y]
        :param m_pos: 追踪无人机初始位置 [x, y]
        :param t_vel: 目标速度向量 [vx, vy]
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
        angle = np.arctan2(diff[1], diff[0])
        self.m_vel = np.array([m_speed * np.cos(angle), m_speed * np.sin(angle)])

        # 用于存储轨迹数据以便绘图
        self.t_traj = [self.t_pos.copy()]
        self.m_traj = [self.m_pos.copy()]

    def update_target_state(self, time):
        """
        更新目标状态：这里让目标做一个机动，增加追踪难度
        """
        # 目标做机动，并加入随机噪声
        noise = np.random.normal(0, 10, 2)     # 均值0，标准差10
        self.t_vel[0] = -20 * np.cos(0.5 * time) + noise[0] # 变化的x方向速度
        self.t_vel[1] = -30 * np.sin(0.5 * time) + noise[1] # 变化的y方向速度

        # 简单的正弦机动
        # self.t_vel[0] = -30                       # x方向做常速前进
        # self.t_vel[1] = -50 * np.sin(0.2 * time)  # y方向做正弦机动

        self.t_pos += self.t_vel * self.dt
        self.t_traj.append(self.t_pos.copy())

    def update_missile_state(self):
        """
        核心：使用比例导引法更新追踪无人机状态
        """
        # 1. 计算相对位置和相对速度
        R_vec = self.t_pos - self.m_pos       # 相对位置向量 (Line of Sight Vector)
        V_rel = self.t_vel - self.m_vel       # 相对速度向量

        R_mag = np.linalg.norm(R_vec)         # 距离

        # 如果距离非常近，视为命中
        if R_mag < 1.0:
            return True # Hit

        # 2. 计算视线角速率 (LOS Rate, lambda_dot)
        # 公式: (Rx * Vry - Ry * Vrx) / R^2
        # 这是二维向量叉乘除以距离平方
        cross_prod = R_vec[0] * V_rel[1] - R_vec[1] * V_rel[0]
        lambda_dot = cross_prod / (R_mag ** 2)

        # 3. 计算接近速度 (Closing Velocity, Vc)
        # 它是相对速度在视线方向上的投影
        Vc = -np.dot(R_vec, V_rel) / R_mag

        # 4. 计算期望的法向加速度 (Lateral Acceleration Command)
        # 核心公式: ac = N * Vc * lambda_dot
        ac = self.N * Vc * lambda_dot

        # 5. 更新追踪者的速度方向
        # 获取当前航向角
        psi = np.arctan2(self.m_vel[1], self.m_vel[0])

        # 航向角变化率 = 法向加速度 / 速度
        psi_dot = ac / self.m_speed

        # 更新航向
        psi_new = psi + psi_dot * self.dt

        # 更新速度向量 (保持速率不变，只改变方向)
        self.m_vel = np.array([self.m_speed * np.cos(psi_new), self.m_speed * np.sin(psi_new)])

        # 更新位置
        self.m_pos += self.m_vel * self.dt
        self.m_traj.append(self.m_pos.copy())

        return False # Not hit yet

    def run_simulation(self, max_time=20):
        """运行仿真循环"""
        time = 0
        hit = False
        max_time = 10000    # 最大迭代次数防止死循环
        while time < max_time:
            self.update_target_state(time)
            hit = self.update_missile_state()
            if hit:
                print(f"目标在 t={time:.2f}s 时被捕获！")
                break
            time += self.dt
        return hit

    def plot_results(self):
        """绘制轨迹图"""
        t_traj = np.array(self.t_traj)
        m_traj = np.array(self.m_traj)

        plt.figure(figsize=(10, 6))

        # 绘制轨迹
        plt.plot(t_traj[:, 0], t_traj[:, 1], 'b--', label='Target', linewidth=2)
        plt.plot(m_traj[:, 0], m_traj[:, 1], 'r-', label='Tracker (PNG)', linewidth=2)

        # 绘制起点和终点
        plt.scatter(t_traj[0, 0], t_traj[0, 1], c='blue', marker='o', s=100, label='Start')
        plt.scatter(m_traj[0, 0], m_traj[0, 1], c='red', marker='o', s=100)
        plt.scatter(m_traj[-1, 0], m_traj[-1, 1], c='black', marker='x', s=200, label='Intercept Point', zorder=10)

        # 连接一些连线表示视线 (Line of Sight)
        step = len(m_traj) // 10
        for i in range(0, len(m_traj), step):
            if i < len(t_traj):
                plt.plot([m_traj[i, 0], t_traj[i, 0]], [m_traj[i, 1], t_traj[i, 1]], 'g-', alpha=0.3, linewidth=0.5)

        plt.title(f'Proportional Navigation Guidance (N={self.N})')
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.legend()
        plt.grid(True)
        plt.axis('equal') # 保证比例一致，看起来不失真
        plt.show()

# --- 主程序 ---
if __name__ == "__main__":
    # 参数设定
    target_start = [0, 500]       # 目标起点
    tracker_start = [0, 0]        # 追踪者起点
    target_velocity = [0, 0]     # 目标初始速度向量 (会被update覆盖)
    tracker_speed = 50            # 追踪者速率 (通常要比目标快，例如 1.5倍)

    # 实例化并运行
    sim = ProportionalNavigation(target_start, tracker_start, target_velocity, tracker_speed, N=4.0)
    sim.run_simulation()
    sim.plot_results()
