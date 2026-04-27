#include "dm_control_core/joint_impedance_controller.hpp"

#include <cmath>
#include <stdexcept>

namespace dm_control_core {

// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

/**
 * @brief 配置关节阻抗控制器
 * @param config 控制器配置
 */
void JointImpedanceController::configure(const JointImpedanceControllerConfig& config) {
    config_ = config;
    validate_config();

    const std::size_t n = config_.layout.joint_names.size();
    hold_position_.assign(n, 0.0);
    reference_.position.assign(n, 0.0);
    reference_.velocity.assign(n, 0.0);
    reference_.effort.assign(n, 0.0);
    reference_valid_ = false;
    mode_ = JointImpedanceMode::RIGID_HOLD;
}

/**
 * @brief 重置控制器状态并锁存当前保持位置
 * @param current_state 当前关节状态
 */
void JointImpedanceController::reset(const JointState& current_state) {
    latch_hold_position(current_state);
    reference_valid_ = false;
    mode_ = JointImpedanceMode::RIGID_HOLD;
}

/**
 * @brief 切换控制模式
 * @param mode 目标控制模式
 * @param current_state 当前关节状态
 */
void JointImpedanceController::set_mode(JointImpedanceMode mode, const JointState& current_state) {
    mode_ = mode;

    if(mode == JointImpedanceMode::RIGID_HOLD || mode == JointImpedanceMode::COMPLIANT_HOLD) {
        latch_hold_position(current_state);
    }
}

/**
 * @brief 设置多关节参考输入
 * @param reference 多关节参考输入
 */
void JointImpedanceController::set_reference(const JointReference& reference) {
    const std::size_t n = config_.layout.joint_names.size();
    if(reference.position.size() != n || reference.velocity.size() != n || reference.effort.size() != n) {
        reference_valid_ = false;
        return;
    }

    reference_ = reference;
    reference_valid_ = true;
}

/**
 * @brief 执行一次多关节阻抗控制更新
 * @param input 周期输入
 * @return 周期输出命令
 */
JointImpedanceControllerOutput JointImpedanceController::update(const JointImpedanceControllerInput& input) {
    const std::size_t n = config_.layout.joint_names.size();

    JointImpedanceControllerOutput output;
    output.command.position.resize(n);
    output.command.velocity.resize(n);
    output.command.effort.resize(n);
    output.command.kp.resize(n);
    output.command.kd.resize(n);

    if(input.state.position.size() != n || input.state.velocity.size() != n || input.state.effort.size() != n) return output;
    if(input.state.motor_effort.size() != n) return output;

    for(std::size_t i = 0; i < n; ++i) {
        double q_ref = input.state.position[i];
        double dq_ref = 0.0;
        double residual_effort = 0.0;
        double kp = 0.0;
        double kd = 0.0;

        select_gains(i, kp, kd);

        switch(mode_) {
            case JointImpedanceMode::RIGID_HOLD:
                q_ref = hold_position_[i];
                dq_ref = 0.0;
                break;

            case JointImpedanceMode::COMPLIANT_HOLD:
                q_ref = hold_position_[i];
                dq_ref = 0.0;
                break;

            case JointImpedanceMode::TRACKING:
                if(reference_valid_) {
                    q_ref = reference_.position[i];
                    dq_ref = reference_.velocity[i];
                    residual_effort = reference_.effort[i];
                }
                else {
                    q_ref = input.state.position[i];
                    dq_ref = 0.0;
                }
                break;
        }

        double tau_ff = 0.0;

        if(config_.use_gravity_feedforward && i < input.gravity_effort.size()) {
            tau_ff += sanitize_or_default(input.gravity_effort[i], 0.0);
        }

        if(config_.use_reference_effort) {
            tau_ff += sanitize_or_default(residual_effort, 0.0);
        }

        output.command.position[i] = sanitize_or_default(q_ref, input.state.position[i]);
        output.command.velocity[i] = clamp_abs(sanitize_or_default(dq_ref, 0.0), config_.limits.max_velocity[i]);
        output.command.effort[i] = clamp_abs(sanitize_or_default(tau_ff, 0.0), config_.limits.max_effort[i]);
        output.command.kp[i] = clamp_range(sanitize_or_default(kp, 0.0), config_.limits.min_kp[i], config_.limits.max_kp[i]);
        output.command.kd[i] = clamp_range(sanitize_or_default(kd, 0.0), config_.limits.min_kd[i], config_.limits.max_kd[i]);
    }

    return output;
}

// ! ========================= 私 有 类 方 法 实 现 ========================= ! //

/**
 * @brief 检查配置数组长度
 */
void JointImpedanceController::validate_config() const {
    const std::size_t n = config_.layout.joint_names.size();
    if(n == 0) throw std::runtime_error("JointImpedanceController requires at least one joint");

    if(config_.rigid_hold_gains.kp.size() != n) throw std::runtime_error("Invalid rigid_hold_gains.kp size");
    if(config_.rigid_hold_gains.kd.size() != n) throw std::runtime_error("Invalid rigid_hold_gains.kd size");
    if(config_.compliant_hold_gains.kp.size() != n) throw std::runtime_error("Invalid compliant_hold_gains.kp size");
    if(config_.compliant_hold_gains.kd.size() != n) throw std::runtime_error("Invalid compliant_hold_gains.kd size");
    if(config_.tracking_gains.kp.size() != n) throw std::runtime_error("Invalid tracking_gains.kp size");
    if(config_.tracking_gains.kd.size() != n) throw std::runtime_error("Invalid tracking_gains.kd size");

    if(config_.limits.max_velocity.size() != n) throw std::runtime_error("Invalid max_velocity limits size");
    if(config_.limits.max_effort.size() != n) throw std::runtime_error("Invalid max_effort limits size");
    if(config_.limits.min_kp.size() != n) throw std::runtime_error("Invalid min_kp limits size");
    if(config_.limits.max_kp.size() != n) throw std::runtime_error("Invalid max_kp limits size");
    if(config_.limits.min_kd.size() != n) throw std::runtime_error("Invalid min_kd limits size");
    if(config_.limits.max_kd.size() != n) throw std::runtime_error("Invalid max_kd limits size");
}

/**
 * @brief 锁存保持模式目标位置
 * @param current_state 当前关节状态
 */
void JointImpedanceController::latch_hold_position(const JointState& current_state) {
    const std::size_t n = config_.layout.joint_names.size();
    if(current_state.position.size() != n) return;

    for(std::size_t i = 0; i < n; ++i) {
        hold_position_[i] = sanitize_or_default(current_state.position[i], hold_position_[i]);
    }
}

/**
 * @brief 根据当前模式选择对应关节阻抗参数
 * @param index 关节索引
 * @param kp 当前模式下的位置刚度
 * @param kd 当前模式下的速度阻尼
 */
void JointImpedanceController::select_gains(std::size_t index, double& kp, double& kd) const {
    switch(mode_) {
        case JointImpedanceMode::RIGID_HOLD:
            kp = config_.rigid_hold_gains.kp[index];
            kd = config_.rigid_hold_gains.kd[index];
            return;

        case JointImpedanceMode::COMPLIANT_HOLD:
            kp = config_.compliant_hold_gains.kp[index];
            kd = config_.compliant_hold_gains.kd[index];
            return;

        case JointImpedanceMode::TRACKING:
            kp = config_.tracking_gains.kp[index];
            kd = config_.tracking_gains.kd[index];
            return;
    }

    kp = 0.0;
    kd = 0.0;
}

/**
 * @brief 过滤非有限数
 * @param value 输入值
 * @param default_value 默认值
 * @return value 为有限数时返回 value，否则返回 default_value
 */
double JointImpedanceController::sanitize_or_default(double value, double default_value) const {
    if(std::isfinite(value)) return value;
    return default_value;
}

/**
 * @brief 对称限幅
 * @param value 输入值
 * @param limit 绝对值上限
 * @return 限幅后的值
 */
double JointImpedanceController::clamp_abs(double value, double limit) const {
    if(!std::isfinite(limit) || limit <= 0.0) return value;

    if(value > limit) return limit;
    if(value < -limit) return -limit;
    return value;
}

/**
 * @brief 区间限幅
 * @param value 输入值
 * @param lower 下限
 * @param upper 上限
 * @return 限幅后的值
 */
double JointImpedanceController::clamp_range(double value, double lower, double upper) const {
    if(lower > upper) return value;

    if(value < lower) return lower;
    if(value > upper) return upper;
    return value;
}

}
