from typing import cast

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.axes import Axes

try:
    from mpl_toolkits.mplot3d.axes3d import Axes3D
except Exception:
    Axes3D = object


def plot_results(sim):
    """绘制3D轨迹图和速度变化图（分别绘制在独立窗口中）。"""
    t_traj = np.array(sim.t_traj)
    m_traj = np.array(sim.m_traj)

    # 图1：空间轨迹。绿色细线是同一时刻追踪者到目标的视线，便于观察是否绕飞。
    fig1 = plt.figure(figsize=(10, 8))
    try:
        ax1_3d = cast(Axes3D, fig1.add_subplot(111, projection="3d"))  # type: ignore
        ax1: Axes = ax1_3d
    except Exception:
        ax1 = fig1.add_subplot(111)
        ax1_3d = None

    if ax1_3d is not None:
        ax1_3d.plot(t_traj[:, 0], t_traj[:, 1], t_traj[:, 2], "b--", label="Target", linewidth=2)
    else:
        ax1.plot(t_traj[:, 0], t_traj[:, 1], "b--", label="Target", linewidth=2)

    tracker_label = "Tracker (PN+NMPC)" if sim.use_nmpc else "Tracker (PN)"
    if ax1_3d is not None:
        ax1_3d.plot(m_traj[:, 0], m_traj[:, 1], m_traj[:, 2], "r-", label=tracker_label, linewidth=2)
    else:
        ax1.plot(m_traj[:, 0], m_traj[:, 1], "r-", label=tracker_label, linewidth=2)

    if ax1_3d is not None:
        ax1_3d.scatter(t_traj[0, 0], t_traj[0, 1], t_traj[0, 2], c="blue", marker="o", s=50, label="Target Start")
        ax1_3d.scatter(m_traj[0, 0], m_traj[0, 1], m_traj[0, 2], c="red", marker="o", s=50, label="Tracker Start")
        ax1_3d.scatter(m_traj[-1, 0], m_traj[-1, 1], m_traj[-1, 2], c="black", marker="x", s=100, label="Intercept Point")
    else:
        ax1.scatter(t_traj[0, 0], t_traj[0, 1], c="blue", marker="o", s=50, label="Target Start")
        ax1.scatter(m_traj[0, 0], m_traj[0, 1], c="red", marker="o", s=50, label="Tracker Start")
        ax1.scatter(m_traj[-1, 0], m_traj[-1, 1], c="black", marker="x", s=100, label="Intercept Point")

    step = max(1, len(m_traj) // 15)
    for i in range(0, len(m_traj), step):
        if i < len(t_traj):
            if ax1_3d is not None:
                ax1_3d.plot(
                    [m_traj[i, 0], t_traj[i, 0]],
                    [m_traj[i, 1], t_traj[i, 1]],
                    [m_traj[i, 2], t_traj[i, 2]],
                    "g-",
                    alpha=0.2,
                    linewidth=0.5,
                )
            else:
                ax1.plot(
                    [m_traj[i, 0], t_traj[i, 0]],
                    [m_traj[i, 1], t_traj[i, 1]],
                    "g-",
                    alpha=0.2,
                    linewidth=0.5,
                )

    mode_label = "PN+NMPC+FOV" if sim.use_nmpc else "Pure PN"
    ax1.set_title(f"3D Guidance ({mode_label}, N={sim.N}, Speed={sim.speed_strategy})")
    ax1.set_xlabel("X Position (m)")
    ax1.set_ylabel("Y Position (m)")
    if ax1_3d is not None:
        ax1_3d.set_zlabel("Z Position (m)")
    ax1.legend(fontsize=8)

    max_range = np.array([
        t_traj[:, 0].max() - t_traj[:, 0].min(),
        t_traj[:, 1].max() - t_traj[:, 1].min(),
        t_traj[:, 2].max() - t_traj[:, 2].min(),
        m_traj[:, 0].max() - m_traj[:, 0].min(),
        m_traj[:, 1].max() - m_traj[:, 1].min(),
        m_traj[:, 2].max() - m_traj[:, 2].min(),
    ]).max() / 2.0

    mid_x = (t_traj[:, 0].max() + t_traj[:, 0].min()) * 0.5
    mid_y = (t_traj[:, 1].max() + t_traj[:, 1].min()) * 0.5
    mid_z = (t_traj[:, 2].max() + t_traj[:, 2].min()) * 0.5

    ax1.set_xlim(mid_x - max_range, mid_x + max_range)
    ax1.set_ylim(mid_y - max_range, mid_y + max_range)
    if ax1_3d is not None:
        ax1_3d.set_zlim(mid_z - max_range, mid_z + max_range)
    fig1.tight_layout()

    # 图2：速度曲线。用它检查速度策略是否符合 40cm 四旋翼的上下限和加速能力。
    fig2 = plt.figure(figsize=(10, 6))
    ax2 = fig2.add_subplot(111)
    ax2.plot(sim.time_history, sim.speed_history, "r-", linewidth=2, label="Tracker Speed")
    ax2.axhline(y=sim.speed_min, color="g", linestyle="--", label=f"Min Speed ({sim.speed_min} m/s)")
    ax2.axhline(y=sim.speed_max, color="b", linestyle="--", label=f"Max Speed ({sim.speed_max} m/s)")
    ax2.axhline(y=sim.m_speed_init, color="orange", linestyle=":", label=f"Initial Speed ({sim.m_speed_init} m/s)")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Speed (m/s)")
    ax2.set_title("Tracker Speed vs Time")
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    fig2.tight_layout()

    # 图3：距离曲线。最小距离和 capture_radius 一起决定是否捕获成功。
    fig3 = plt.figure(figsize=(10, 6))
    ax3 = fig3.add_subplot(111)
    ax3.plot(sim.time_history, sim.distance_history, "b-", linewidth=2)
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Distance (m)")
    ax3.set_title("Distance to Target vs Time")
    ax3.grid(True, alpha=0.3)
    fig3.tight_layout()

    # 图4：距离-速度关系。颜色越亮时间越靠后，可看出接近目标时是否主动降速。
    fig4 = plt.figure(figsize=(10, 6))
    ax4 = fig4.add_subplot(111)
    scatter = ax4.scatter(sim.distance_history, sim.speed_history, c=sim.time_history, cmap="viridis", s=5)
    ax4.set_xlabel("Distance to Target (m)")
    ax4.set_ylabel("Speed (m/s)")
    ax4.set_title("Speed vs Distance (color = time)")
    ax4.grid(True, alpha=0.3)
    plt.colorbar(scatter, ax=ax4, label="Time (s)")
    fig4.tight_layout()

    # 图5：FOV 误差。超过水平/垂直半视场角时会标红，说明目标短暂离开传感器视野。
    fig5 = plt.figure(figsize=(10, 6))
    ax5 = fig5.add_subplot(111)
    fov_time = np.asarray(sim.time_history[:len(sim.fov_horizontal_history)], dtype=float)
    h_errors = np.asarray(sim.fov_horizontal_history, dtype=float)
    v_errors = np.asarray(sim.fov_vertical_history, dtype=float)
    violations = np.asarray(sim.fov_violation_history, dtype=bool)

    ax5.plot(fov_time, h_errors, "m-", linewidth=1.5, label="Horizontal FOV Error")
    ax5.plot(fov_time, v_errors, "c-", linewidth=1.5, label="Vertical FOV Error")
    ax5.axhline(sim.nmpc_config.fov_horizontal_deg * 0.5, color="m", linestyle="--", alpha=0.4)
    ax5.axhline(-sim.nmpc_config.fov_horizontal_deg * 0.5, color="m", linestyle="--", alpha=0.4)
    ax5.axhline(sim.nmpc_config.fov_vertical_deg * 0.5, color="c", linestyle=":", alpha=0.5)
    ax5.axhline(-sim.nmpc_config.fov_vertical_deg * 0.5, color="c", linestyle=":", alpha=0.5)
    if np.any(violations):
        ax5.scatter(
            fov_time[violations],
            h_errors[violations],
            c="red",
            s=8,
            alpha=0.6,
            label="FOV Violation",
        )
    ax5.set_xlabel("Time (s)")
    ax5.set_ylabel("Angle Error (deg)")
    ax5.set_title("FOV Angle Error vs Time")
    ax5.legend()
    ax5.grid(True, alpha=0.3)
    fig5.tight_layout()

    plt.show()
