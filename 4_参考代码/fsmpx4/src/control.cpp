#include "../include/control.h"
#include "../include/param_loader.h"

#include <algorithm>
#include <cmath>

namespace fsmpx4 {
namespace control {

using namespace fsmpx4::types;

void PositionAttitudeController::Config::load_from_params(const fsmpx4::param_loader::FSMParams& params)
{
    kp.x() = params.gains.Kp_x;
    kp.y() = params.gains.Kp_y;
    kp.z() = params.gains.Kp_z;
    kv.x() = params.gains.Kv_x;
    kv.y() = params.gains.Kv_y;
    kv.z() = params.gains.Kv_z;
    kvi.x() = params.gains.Kvi_x;
    kvi.y() = params.gains.Kvi_y;
    kvi.z() = params.gains.Kvi_z;
    mass = params.physical.mass;
    gravity = params.physical.gravity;
    use_integral = params.basic.use_integral;
    hover_thrust_default = params.thr_map.hover_percentage;
}

ControlOutput createDefaultControlOutput(double timestamp)
{
    ControlOutput output;
    output.thrust = 0.0;
    output.thrust_vector = Vector3::Zero();
    output.moment = Vector3::Zero();
    output.qd = Quaternion::Identity();
    output.Rd = Matrix3::Identity();
    output.valid = false;
    output.timestamp = timestamp;
    return output;
}

bool PositionAttitudeController::initialize(const Config& cfg)
{
    // 基本参数校验
    if (cfg.mass <= 0.0 || cfg.gravity <= 0.0) return false;
    if ((cfg.kp.array() < 0.0).any()) return false;
    if ((cfg.kv.array() < 0.0).any()) return false;
    if ((cfg.kvi.array() < 0.0).any()) return false;
    if (cfg.hover_thrust_default <= 0.0) return false;

    cfg_ = cfg;
    initialized_ = true;
    resetIntegrator();
    return true;
}


ControlOutput PositionAttitudeController::computeControl(const UAVState& state,
                                                         const UAVCommand& cmd) const
{
    ControlOutput output;
    output.timestamp = state.timestamp;

    if (!initialized_) {
        output.Rd = state.rotation;
        output.qd = Quaternion(state.rotation);
        output.qd.normalize();
        return output;
    }

    // eX = x - xd, eV = v - vd
    const Vector3 eX = state.position - cmd.position;
    const Vector3 eV = state.velocity - cmd.velocity;

    // 计算时间间隔
    double dt = computeDeltaTime(state.timestamp);
    
    // 更新时间戳
    last_timestamp_ = std::isfinite(state.timestamp) ? state.timestamp : 0.0;

    accumulateIntegral(eX, dt);

    // A = -Kx eX - Kv eV - m g e3 + m a_d
    const Vector3 A = -cfg_.kp.cwiseProduct(eX)
                      - cfg_.kv.cwiseProduct(eV)
                      - cfg_.kvi.cwiseProduct(position_integral_)
                      - cfg_.mass * cfg_.gravity * Vector3::UnitZ()
                      + cfg_.mass * cmd.acceleration;

    output.A = A;
    output.thrust_vector = -A;

    const double a_norm = A.norm();
    if (a_norm < 1e-6) {
        output.Rd = state.rotation;
        output.qd = Quaternion(state.rotation);
        output.qd.normalize();
        output.thrust = 0.0;
        output.thrust_vector = Vector3::Zero();
        output.valid = false;
        return output;
    }
    // 机体系 z 轴与合力方向相反
    const Vector3 b3c = (-A / a_norm);

    Vector3 b1ref;
    if (cmd.b1d.norm() > 1e-6) {
        b1ref = cmd.b1d;
    } else {
        b1ref = Vector3(std::cos(cmd.yaw_desired), std::sin(cmd.yaw_desired), 0.0);
    }

    // 将 b1ref 投影到与 b3c 正交的平面，得到机体系 x 轴 b1c
    Vector3 b1_proj = b1ref - b3c * (b3c.dot(b1ref));
    double proj_norm = b1_proj.norm();
    if (proj_norm < 1e-6) {
        // 与 b3c 近平行时挑选任意正交向量
        const Vector3 arbitrary = std::abs(b3c.dot(Vector3::UnitX())) < 0.9 ? Vector3::UnitX() : Vector3::UnitY();
        b1_proj = arbitrary - b3c * (b3c.dot(arbitrary));
        proj_norm = std::max(b1_proj.norm(), 1e-6);
    }
    const Vector3 b1c = b1_proj / proj_norm;
    const Vector3 b2c = b3c.cross(b1c);

    Matrix3 Rd;
    Rd.col(0) = b1c;
    Rd.col(1) = b2c;
    Rd.col(2) = b3c;

    Quaternion qd(Rd);
    qd.normalize();

    output.Rd = Rd;
    output.qd = qd;
    const double hover_thrust = (std::isfinite(state.hover_thrust) && state.hover_thrust > 0.0)
                                ? state.hover_thrust
                                : cfg_.hover_thrust_default;
    const double weight = cfg_.mass * cfg_.gravity;
    double normalized_thrust = 0.0;
    if (weight > 0.0)
    {
        const double thrust_ratio = a_norm / weight;
        normalized_thrust = thrust_ratio * hover_thrust;
    }
    // PX4 约定：机体系 Z 轴正向朝下，期望向上的推力为负值
    output.thrust = -normalized_thrust;
    output.valid = true;

    return output;
}


Quaternion PositionAttitudeController::computeDesiredRotation(const UAVState& state,
                                                              const UAVCommand& cmd) const
{
    return computeControl(state, cmd).qd;
}


Matrix3 PositionAttitudeController::projectToSO3(const Matrix3& R)
{
    Eigen::JacobiSVD<Matrix3> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Matrix3 U = svd.matrixU();
    Matrix3 V = svd.matrixV();
    Matrix3 P = U * V.transpose();
    if (P.determinant() < 0.0) {
        U.col(2) *= -1.0;
        P = U * V.transpose();
    }
    return P;
}


void PositionAttitudeController::resetIntegrator()
{
    position_integral_.setZero();
    last_timestamp_ = 0.0;
}

void PositionAttitudeController::accumulateIntegral(const Vector3& eX, double dt) const
{
    if (!cfg_.use_integral)
    {
        position_integral_.setZero();
        return;
    }

    if (dt <= 0.0)
    {
        return;
    }

    position_integral_ += eX * dt;

    const double limit = std::abs(cfg_.integral_limit);
    if (limit > 0.0)
    {
        for (int i = 0; i < 3; ++i)
        {
            position_integral_(i) = std::clamp(position_integral_(i), -limit, limit);
        }
    }
}

double PositionAttitudeController::computeDeltaTime(double current_timestamp) const
{
    double dt = 0.0;
    const double prev_timestamp = last_timestamp_;
    
    if (std::isfinite(current_timestamp) && prev_timestamp > 0.0)
    {
        dt = current_timestamp - prev_timestamp;
        // 检查时间间隔的合理性：必须是正数且不能太大（防止异常值）
        if (!std::isfinite(dt) || dt <= 0.0 || dt > 1.0)
        {
            dt = 0.0;
        }
    }
    
    return dt;
}

} // namespace control
} // namespace fsmpx4
