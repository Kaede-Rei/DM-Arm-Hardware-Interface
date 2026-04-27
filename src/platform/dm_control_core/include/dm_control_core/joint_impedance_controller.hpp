#pragma once

#include "dm_control_core/joint_control_types.hpp"

namespace dm_control_core {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief 纯 C++ 关节空间阻抗控制核心
 * @note 该类不包含 ROS 头文件。当前实现对应达妙 MIT 模式语义：
 *       电机侧执行 Kp(q_ref-q)+Kd(dq_ref-dq)+tau_ff
 */
class JointImpedanceController {
public:
    /**
     * @brief 配置受控关节顺序
     * @param layout 关节控制布局
     */
    void configure(const JointControlLayout& layout);

    /**
     * @brief 获取当前受控关节顺序
     * @return 关节控制布局
     */
    const JointControlLayout& layout() const { return _layout_; }

    /**
     * @brief 由关节状态、参考、阻抗参数和前馈项生成 MIT 关节命令
     * @param state 当前关节状态
     * @param reference 上层参考输入
     * @param command_gains 上层或适配层给出的阻抗参数
     * @param fallback_gains command_gains 非有限时使用的默认阻抗参数
     * @param feedforward 动力学/补偿前馈项
     * @return MIT 模式关节侧命令
     */
    MitJointCommand compute_command(const JointState& state, const JointReference& reference,
        const JointImpedanceGains& command_gains, const JointImpedanceGains& fallback_gains,
        const JointFeedforward& feedforward) const;

private:
    /**
     * @brief 过滤非有限数，避免 NaN/Inf 进入控制命令
     * @param value 输入值
     * @param default_value fallback 值
     * @return 有限输入值或 fallback 值
     */
    double sanitize_or_default(double value, double default_value) const;

private:
    JointControlLayout _layout_;
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //



}
