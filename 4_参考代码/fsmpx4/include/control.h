#ifndef FSMPX4_CONTROL_H
#define FSMPX4_CONTROL_H

#include "types.h"
#include <optional>
#include <memory>

namespace fsmpx4 {
namespace param_loader {
struct FSMParams;
}
}

namespace fsmpx4 {
namespace control {

using namespace fsmpx4::types;


class PositionAttitudeController {
public:
    struct Config {
        Vector3 kp{Vector3::Constant(3.0)};  // 位置比例增益
        Vector3 kv{Vector3::Constant(2.0)};  // 速度比例增益
        Vector3 kvi{Vector3::Zero()};        // 位置积分增益
        double mass{1.0};                    // 质量 kg
        double gravity{9.81};                // 重力 m/s^2
        Vector3 b1d{Vector3::UnitX()};       // 航向参考向量（与 b3 正交），由指令提供
        bool use_integral{false};            // 是否启用位置积分
        double integral_limit{1.5};          // 积分限幅
        double hover_thrust_default{0.6};    // 默认悬停油门（归一化）

        void load_from_params(const fsmpx4::param_loader::FSMParams& params);
    };

    PositionAttitudeController() = default;

    bool initialize(const Config& cfg);
   
    template<typename NodeType>
    bool initialize(const std::shared_ptr<NodeType>& node)
    {
        Config cfg = cfg_;

        // 从参数服务器读取（参数由 --params-file YAML 注入）
        const auto getd = [&](const std::string& key, double defval) {
            return node->template declare_parameter<double>(key, defval);
        };

        // Gains
        cfg.kp.x() = getd("control.Kp_x", cfg.kp.x());
        cfg.kp.y() = getd("control.Kp_y", cfg.kp.y());
        cfg.kp.z() = getd("control.Kp_z", cfg.kp.z());

        cfg.kv.x() = getd("control.Kv_x", cfg.kv.x());
        cfg.kv.y() = getd("control.Kv_y", cfg.kv.y());
        cfg.kv.z() = getd("control.Kv_z", cfg.kv.z());

        cfg.kvi.x() = getd("control.Kvi_x", cfg.kvi.x());
        cfg.kvi.y() = getd("control.Kvi_y", cfg.kvi.y());
        cfg.kvi.z() = getd("control.Kvi_z", cfg.kvi.z());

        cfg.mass = getd("control.mass", cfg.mass);
        cfg.gravity = getd("control.gravity", cfg.gravity);

        cfg.use_integral = node->template declare_parameter<bool>("control.use_integral", cfg.use_integral);
        cfg.integral_limit = getd("control.integral_limit", cfg.integral_limit);

        return initialize(cfg);
    }

    // 输入完整指令cmd（位置/速度/加速度/航向），输出期望四元数
    ControlOutput computeControl(const UAVState& state,
                                 const UAVCommand& cmd) const;

    Quaternion computeDesiredRotation(const UAVState& state,
                                      const UAVCommand& cmd) const;

    const Config& config() const { return cfg_; }
    bool initialized() const { return initialized_; }
    void resetIntegrator();

private:
    static Matrix3 projectToSO3(const Matrix3& R);
    void accumulateIntegral(const Vector3& eX, double dt) const;
    double computeDeltaTime(double current_timestamp) const;

    Config cfg_{};
    bool initialized_{false};
    mutable Vector3 position_integral_{Vector3::Zero()};
    mutable double last_timestamp_{0.0};
};

} // namespace control
} // namespace fsmpx4

#endif // FSMPX4_CONTROL_H
