#include "dm_control_core/joint_impedance_controller.hpp"

#include <cmath>

namespace dm_control_core {

// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

/**
 * @brief 配置受控关节顺序
 * @param layout 关节控制布局
 */
void JointImpedanceController::configure(const JointControlLayout& layout) {
    _layout_ = layout;
}

/**
 * @brief 由关节状态、参考、阻抗参数和前馈项生成 MIT 关节命令
 * @param state 当前关节状态
 * @param reference 上层参考输入
 * @param command_gains 上层或适配层给出的阻抗参数
 * @param fallback_gains command_gains 非有限时使用的默认阻抗参数
 * @param feedforward 动力学/补偿前馈项
 * @return MIT 模式关节侧命令
 */
MitJointCommand JointImpedanceController::compute_command(const JointState& state, const JointReference& reference,
    const JointImpedanceGains& command_gains, const JointImpedanceGains& fallback_gains,
    const JointFeedforward& feedforward) const {

    MitJointCommand command;
    command.position = sanitize_or_default(reference.position, state.position);
    command.velocity = sanitize_or_default(reference.velocity, 0.0);
    command.effort = sanitize_or_default(reference.effort, 0.0) + sanitize_or_default(feedforward.effort, 0.0);
    command.kp = sanitize_or_default(command_gains.kp, fallback_gains.kp);
    command.kd = sanitize_or_default(command_gains.kd, fallback_gains.kd);

    return command;
}

// ! ========================= 私 有 类 方 法 实 现 ========================= ! //

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

}
