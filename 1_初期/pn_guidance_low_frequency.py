import numpy as np
import matplotlib.pyplot as plt

class ProportionalNavigation3D_VariableSpeed:
    def __init__(self, t_pos, m_pos, t_vel, m_speed_init, N=3.0, dt=0.01,
                 dt_control=0.1,speed_min=20, speed_max=60, speed_strategy='adaptive'):
        """
        初始化3D仿真参数（支持可变速度）
        :param t_pos: 目标初始位置 [x, y, z]
        :param m_pos: 追踪无人机初始位置 [x, y, z]
        :param t_vel: 目标速度向量 [vx, vy, vz]
        :param m_speed_init: 追踪无人机初始速率 (标量)
        :param N: 比例导引系数 (通常在 3-5 之间)
        :param dt: 仿真时间步长 (物理更新步长，如 0.01s)
        :param dt_control: 控制更新步长 (制导指令更新步长，如 0.1s)
        :param speed_min: 最小速度限制
        :param speed_max: 最大速度限制
        :param speed_strategy: 速度调节策略
        """
        self.t_pos = np.array(t_pos, dtype=float)
        self.m_pos = np.array(m_pos, dtype=float)
        self.t_vel = np.array(t_vel, dtype=float)
        self.m_speed = m_speed_init
        self.m_speed_init = m_speed_init
        self.N = N
        self.dt = dt
        self.dt_control = dt_control
        self.control_timer = dt_control  # 计时器，强制初始时刻更新控制指令

        # 速度限制参数
        self.speed_min = speed_min
        self.speed_max = speed_max
        self.speed_strategy = speed_strategy

        # 初始化追踪者的速度向量（假设初始指向目标）
        diff = self.t_pos - self.m_pos
        dist = np.linalg.norm(diff)
        if dist > 0:
            self.m_vel = diff / dist * m_speed_init
        else:
            self.m_vel = np.array([m_speed_init, 0, 0], dtype=float)

        # 初始化控制指令存储变量
        self.cmd_acc = np.zeros(3)          # 当前时刻的加速度指令
        self.cmd_desired_speed = m_speed_init # 当前时刻的期望速度指令

        # 用于存储轨迹数据以便绘图
        self.t_traj = [self.t_pos.copy()]
        self.m_traj = [self.m_pos.copy()]
        self.speed_history = [m_speed_init]  # 记录速度变化历史
        self.time_history = [0]              # 记录时间历史
        self.distance_history = [dist]       # 记录距离历史

    def compute_desired_speed(self, R_mag, Vc, Omega_mag, target_speed):
        """
        计算期望速度
        :param R_mag: 当前距离
        :param Vc: 接近速度
        :param Omega_mag: 视线角速率大小
        :param target_speed: 目标速度大小
        :return: 期望速度
        """
        self.speed_strategy = 'adaptive'
        # 自适应速度控制策略
        # 基于距离和视线角速率动态调整
        # 距离远时加速，角速率大时适当减速以保持机动性

        # 基础速度系数（基于距离）
        distance_factor = np.clip(R_mag / 200, 0.5, 1.5)
        # 角速率惩罚系数（角速率越大，速度应该越低以保持精度）
        omega_penalty = np.clip(1.0 - Omega_mag * 0.5, 0.6, 1.0)
        # 接近速度补偿（如果接近速度太低，需要加速）
        vc_factor = 1.0
        if Vc < target_speed * 0.5:
            vc_factor = 1.3  # 加速追击

        desired_speed = self.m_speed_init * distance_factor * omega_penalty * vc_factor
        # 应用速度限制
        return np.clip(desired_speed, self.speed_min, self.speed_max)

    def update_speed(self, desired_speed, acceleration_limit=10.0):
        """
        平滑更新速度（避免突变）
        :param desired_speed: 期望速度
        :param acceleration_limit: 加速度限制 (m/s^2)
        :return: 实际更新后的速度
        """
        speed_diff = desired_speed - self.m_speed
        max_speed_change = acceleration_limit * self.dt

        # 限制速度变化率
        if abs(speed_diff) > max_speed_change:
            speed_diff = np.sign(speed_diff) * max_speed_change

        self.m_speed += speed_diff
        return self.m_speed

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
        self.t_vel[0] = -20 + noise[0]
        self.t_vel[1] = -20 * abs(np.sin(0.5 * time)) + noise[1]
        self.t_vel[2] = -10 * np.sin(0.5 * time) + noise[2]

        self.t_pos += self.t_vel * self.dt
        self.t_traj.append(self.t_pos.copy())

    def update_missile_state(self, time):
        """
        核心：使用3D比例导引法更新追踪无人机状态（支持可变速度）
        实现了双速率更新：物理更新频率(dt) vs 控制更新频率(dt_control)
        """
        # 1. 计算相对位置和相对速度 (用于物理判定和制导计算)
        R_vec = self.t_pos - self.m_pos       # 相对位置向量 (Line of Sight Vector)
        V_rel = self.t_vel - self.m_vel       # 相对速度向量
        R_mag = np.linalg.norm(R_vec)         # 距离

        # 如果距离非常近，视为命中
        if R_mag < 1.0:
            return True # Hit

        # --- [低频] 控制指令(制导)更新模块 ---
        if self.control_timer >= self.dt_control - 1e-9:
            self.control_timer = 0  # 重置计时器

            # 2. 计算制导所需参数
            R_sq = np.dot(R_vec, R_vec)
            # Omega = (R x V) / R^2
            Omega = np.cross(R_vec, V_rel) / R_sq
            Omega_mag = np.linalg.norm(Omega)
            # Vc = - (R . V) / |R|
            Vc = -np.dot(R_vec, V_rel) / R_mag
            target_speed = np.linalg.norm(self.t_vel)

            # 3. 计算期望速度指令
            self.cmd_desired_speed = self.compute_desired_speed(R_mag, Vc, Omega_mag, target_speed)

            # 4. 计算加速度向量指令 (3D PNG 公式)
            # ac = N * Vc * (Omega x R_unit)
            R_unit = R_vec / R_mag
            self.cmd_acc = self.N * Vc * np.cross(Omega, R_unit)

        self.control_timer += self.dt

        # --- [高频] 物理状态积分模块 ---

        # 5. 更新速度标量 (模拟一阶滞后或平滑过渡)
        self.update_speed(self.cmd_desired_speed)

        # 6. 更新速度向量 (应用最近一次的各个方向加速度指令)
        self.m_vel += self.cmd_acc * self.dt

        # 7. 施加速度约束 (方向由m_vel决定，大小由m_speed决定)
        current_speed = np.linalg.norm(self.m_vel)
        if current_speed > 0:
            self.m_vel = self.m_vel / current_speed * self.m_speed

        # 8. 更新位置
        self.m_pos += self.m_vel * self.dt
        self.m_traj.append(self.m_pos.copy())

        # 9. 记录历史数据
        self.speed_history.append(self.m_speed)
        self.time_history.append(time)
        self.distance_history.append(R_mag)

        return False # Not hit yet

    def run_simulation(self, max_time=600):
        """运行仿真循环"""
        time = 0
        hit = False
        steps = int(max_time / self.dt)

        for _ in range(steps):
            self.update_target_state(time)
            hit = self.update_missile_state(time)
            if hit:
                print(f"目标在 t={time:.2f}s 时被捕获！")
                print(f"最终速度: {self.m_speed:.2f} m/s")
                break
            time += self.dt

        if not hit:
            print(f"仿真结束，未能在 {max_time}s 内捕获目标。")

        return hit

    def plot_results(self):
        """绘制3D轨迹图和速度变化图（分别绘制在独立窗口中）"""
        t_traj = np.array(self.t_traj)
        m_traj = np.array(self.m_traj)

        # 图1: 3D轨迹
        fig1 = plt.figure(figsize=(10, 8))
        ax1 = fig1.add_subplot(111, projection='3d')

        # 绘制轨迹
        ax1.plot(t_traj[:, 0], t_traj[:, 1], t_traj[:, 2], 'b--', label='Target', linewidth=2)
        ax1.plot(m_traj[:, 0], m_traj[:, 1], m_traj[:, 2], 'r-', label='Tracker (Variable Speed)', linewidth=2)

        # 绘制起点和终点
        ax1.scatter(t_traj[0, 0], t_traj[0, 1], t_traj[0, 2], c='blue', marker='o', s=50, label='Target Start')
        ax1.scatter(m_traj[0, 0], m_traj[0, 1], m_traj[0, 2], c='red', marker='o', s=50, label='Tracker Start')

        # 终点
        ax1.scatter(m_traj[-1, 0], m_traj[-1, 1], m_traj[-1, 2], c='black', marker='x', s=100, label='Intercept Point')

        # 连接一些连线表示视线 (Line of Sight)
        step = max(1, len(m_traj) // 15)
        for i in range(0, len(m_traj), step):
            if i < len(t_traj):
                ax1.plot([m_traj[i, 0], t_traj[i, 0]],
                        [m_traj[i, 1], t_traj[i, 1]],
                        [m_traj[i, 2], t_traj[i, 2]],
                        'g-', alpha=0.2, linewidth=0.5)

        ax1.set_title(f'3D PNG with Variable Speed (N={self.N}, Strategy={self.speed_strategy})')
        ax1.set_xlabel('X Position (m)')
        ax1.set_ylabel('Y Position (m)')
        ax1.set_zlabel('Z Position (m)')
        ax1.legend(fontsize=8)

        # 设置坐标轴比例一致
        max_range = np.array([t_traj[:,0].max()-t_traj[:,0].min(),
                              t_traj[:,1].max()-t_traj[:,1].min(),
                              t_traj[:,2].max()-t_traj[:,2].min(),
                              m_traj[:,0].max()-m_traj[:,0].min(),
                              m_traj[:,1].max()-m_traj[:,1].min(),
                              m_traj[:,2].max()-m_traj[:,2].min()]).max() / 2.0

        mid_x = (t_traj[:,0].max()+t_traj[:,0].min()) * 0.5
        mid_y = (t_traj[:,1].max()+t_traj[:,1].min()) * 0.5
        mid_z = (t_traj[:,2].max()+t_traj[:,2].min()) * 0.5

        ax1.set_xlim(mid_x - max_range, mid_x + max_range)
        ax1.set_ylim(mid_y - max_range, mid_y + max_range)
        ax1.set_zlim(mid_z - max_range, mid_z + max_range)
        fig1.tight_layout()

        # 图2: 速度变化曲线
        fig2 = plt.figure(figsize=(10, 6))
        ax2 = fig2.add_subplot(111)
        ax2.plot(self.time_history, self.speed_history, 'r-', linewidth=2, label='Tracker Speed')
        ax2.axhline(y=self.speed_min, color='g', linestyle='--', label=f'Min Speed ({self.speed_min} m/s)')
        ax2.axhline(y=self.speed_max, color='b', linestyle='--', label=f'Max Speed ({self.speed_max} m/s)')
        ax2.axhline(y=self.m_speed_init, color='orange', linestyle=':', label=f'Initial Speed ({self.m_speed_init} m/s)')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Speed (m/s)')
        ax2.set_title('Tracker Speed vs Time')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        fig2.tight_layout()

        # 图3: 距离变化曲线
        fig3 = plt.figure(figsize=(10, 6))
        ax3 = fig3.add_subplot(111)
        ax3.plot(self.time_history, self.distance_history, 'b-', linewidth=2)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Distance (m)')
        ax3.set_title('Distance to Target vs Time')
        ax3.grid(True, alpha=0.3)
        fig3.tight_layout()

        # 图4: 速度-距离关系图
        fig4 = plt.figure(figsize=(10, 6))
        ax4 = fig4.add_subplot(111)
        scatter = ax4.scatter(self.distance_history, self.speed_history, c=self.time_history, cmap='viridis', s=5)
        ax4.set_xlabel('Distance to Target (m)')
        ax4.set_ylabel('Speed (m/s)')
        ax4.set_title('Speed vs Distance (color = time)')
        ax4.grid(True, alpha=0.3)

        # 添加颜色条
        plt.colorbar(scatter, ax=ax4, label='Time (s)')
        fig4.tight_layout()

        plt.show()

# --- 主程序 ---
if __name__ == "__main__":
    print("=" * 60)
    print("3D比例导引 - 可变速度控制仿真")
    print("=" * 60)

    # 参数设定
    target_start = [500, 500, 500]      # 目标初始位置 [x, y, z]
    tracker_start = [0, 0, 0]            # 追踪者初始位置 [x, y, z]
    target_velocity = [0, 0, 0]          # 目标初始速度向量
    tracker_speed = 40                   # 追踪者初始速率

    # 实例化并运行
    sim = ProportionalNavigation3D_VariableSpeed(
        target_start, tracker_start, target_velocity, tracker_speed,
        N=4,
        dt=0.01,           # 仿真及目标状态更新步长 (高频)
        dt_control=0.1,    # PN制导指令更新步长 (低频)
        speed_min=20,      # 最小速度 20 m/s
        speed_max=60,      # 最大速度 60 m/s
        speed_strategy='adaptive'  # 使用自适应速度控制策略
    )

    sim.run_simulation()
    sim.plot_results()
