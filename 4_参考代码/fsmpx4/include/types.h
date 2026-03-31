#ifndef __FSMPX4_TYPES_H
#define __FSMPX4_TYPES_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace fsmpx4 {
namespace types {

// Eigen类型别名
using Vector3 = Eigen::Vector3d;
using Matrix3 = Eigen::Matrix3d;
using Quaternion = Eigen::Quaterniond;

//=============================================================================
// 核心数据结构定义
//=============================================================================

/**
 * UAV状态结构体
 * 包含无人机的完整状态信息
 */
struct UAVState {
    Vector3 position;           // 位置 [x, y, z]
    Vector3 velocity;           // 速度 [vx, vy, vz]
    Matrix3 rotation;           // 旋转矩阵 R
    Vector3 angular_velocity;   // 角速度 [wx, wy, wz]
    double hover_thrust;        // 悬停推力值
    double timestamp;           // 时间戳

    UAVState() :
        position(Vector3::Zero()),
        velocity(Vector3::Zero()),
        rotation(Matrix3::Identity()),
        angular_velocity(Vector3::Zero()),
        hover_thrust(-0.5),
        timestamp(0.0) {}
};

/**
 * 无人机指令结构体
 * 包含期望的位置、速度、姿态等完整指令信息
 */
struct UAVCommand {
    Vector3 position;           // 期望位置
    Vector3 velocity;           // 期望速度
    Vector3 acceleration;       // 期望加速度
    Vector3 b1d;                // 期望方向向量
    double yaw_desired;         // 期望偏航角
    Matrix3 Rd;                 // 期望旋转矩阵
    Matrix3 Rd_dot;             // 期望旋转矩阵一阶导数
    Matrix3 Rd_ddot;            // 期望旋转矩阵二阶导数
    Vector3 Wd;                 // 期望角速度
    Vector3 Wd_dot;             // 期望角加速度
    double timestamp;           // 时间戳

    UAVCommand() :
        position(Vector3::Zero()),
        velocity(Vector3::Zero()),
        acceleration(Vector3::Zero()),
        b1d(Vector3::UnitX()),
        yaw_desired(0.0),
        Rd(Matrix3::Identity()),
        Rd_dot(Matrix3::Zero()),
        Rd_ddot(Matrix3::Zero()),
        Wd(Vector3::Zero()),
        Wd_dot(Vector3::Zero()),
        timestamp(0.0) {}
};

/**
 * 控制器增益结构体
 * 包含各种控制增益参数
 */
struct ControllerGains {
    Vector3 position_gains;            // 位置控制增益 [Kp_x, Kp_y, Kp_z]
    Vector3 velocity_gains;            // 速度控制增益 [Kv_x, Kv_y, Kv_z]
    Vector3 attitude_gains;            // 姿态控制增益 [Kr_x, Kr_y, Kr_z]
    Vector3 angular_velocity_gains;    // 角速度控制增益 [Kw_x, Kw_y, Kw_z]
    Vector3 integral_position_gains;   // 位置积分增益 [Kvi_x, Kvi_y, Kvi_z]
    Vector3 integral_attitude_gains;   // 姿态积分增益 [Kri_x, Kri_y, Kri_z]
    bool use_integral;                 // 是否使用积分控制

    ControllerGains() :
        position_gains(Vector3::Zero()),
        velocity_gains(Vector3::Zero()),
        attitude_gains(Vector3::Zero()),
        angular_velocity_gains(Vector3::Zero()),
        integral_position_gains(Vector3::Zero()),
        integral_attitude_gains(Vector3::Zero()),
        use_integral(false) {}
};

/**
 * UAV物理参数结构体
 * 包含无人机的物理特性参数
 */
struct UAVParameters {
    double mass;                // 质量 [kg]
    double gravity;             // 重力加速度 [m/s^2]
    Matrix3 inertia;            // 惯性矩阵 [kg*m^2]
    double max_thrust;          // 最大推力
    double min_thrust;          // 最小推力
    double max_torque;          // 最大力矩

    UAVParameters() :
        mass(1.0),
        gravity(9.81),
        inertia(Matrix3::Identity()),
        max_thrust(1.0),
        min_thrust(0.0),
        max_torque(1.0) {}
};

/**
 * 控制器配置结构体
 * 包含控制器所需的所有配置参数
 */
struct ControllerConfig {
    ControllerGains gains;
    UAVParameters uav_params;
    double dt;                  // 时间步长

    ControllerConfig() : dt(0.01) {}
};
/**
 * 控制输出结构体
 * 控制器计算得到的控制输出
 */
struct ControlOutput {
    double thrust;              // 归一化推力指令（机体系 z 轴，负值表示向上）
    Vector3 thrust_vector;      // 完整推力向量 [Fx, Fy, Fz] (牛顿)
    Vector3 moment;             // 力矩 [Mx, My, Mz]
    Vector3 A;                  // 期望加速度向量
    Vector3 eR;                 // 旋转误差
    Vector3 eW;                 // 角速度误差
    Matrix3 Rd;                 // 期望旋转矩阵
    Quaternion qd;              // 期望姿态四元数
    Vector3 wd;                 // 期望角速度 [wx, wy, wz]
    bool valid;                 // 输出有效性
    double timestamp;           // 时间戳

    ControlOutput() :
        thrust(0.0),
        thrust_vector(Vector3::Zero()),
        moment(Vector3::Zero()),
        A(Vector3::Zero()),
        eR(Vector3::Zero()),
        eW(Vector3::Zero()),
        Rd(Matrix3::Identity()),
        qd(Quaternion::Identity()),
        wd(Vector3::Zero()),
        valid(false),
        timestamp(0.0) {}
};



} // namespace types
} // namespace fsmpx4

#endif // __FSMPX4_TYPES_H
