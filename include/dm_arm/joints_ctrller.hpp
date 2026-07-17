#pragma once

#include <tl/expected.hpp>

#include "dm_arm/types.hpp"

namespace dm_arm {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief 关节控制器配置
 */
struct JointCtrllerCfg {
    std::size_t joints_count{ 0 };              ///< 关节数量

    JointImpedanceGains rigid_hold_gains;       ///< 刚性保持模式增益
    JointImpedanceGains compliant_hold_gains;   ///< 柔性保持模式增益
    JointImpedanceGains tracking_gains;         ///< 跟踪模式增益
};

/**
 * @brief 关节控制器输入
 */
struct JointCtrllerInput {
    JointState state;               ///< 关节状态
    JointVector model_feedforward;  ///< 模型前馈力矩
    double dt{ 0.0 };               ///< 时间步长
};

/**
 * @brief 关节控制器输出
 */
struct JointCtrllerOutput {
    JointActuatorCmd cmd;        ///< 执行器命令
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief 关节控制器
 */
class JointCtrller {
public:
    /**
     * @brief 配置关节控制器
     * @param cfg 关节控制器配置
     * @return tl::expected<void, JointCtrllerErr>
     */
    tl::expected<void, JointCtrllerErr> configure(const JointCtrllerCfg& cfg);
    /**
     * @brief 重置关节控制器
     * @param state 关节状态
     * @return tl::expected<void, JointCtrllerErr>
     */
    tl::expected<void, JointCtrllerErr> reset(const JointState& state);

    /**
     * @brief 设置关节阻抗模式
     * @param mode 关节阻抗模式
     * @param state 关节状态
     * @return tl::expected<void, JointCtrllerErr>
     */
    tl::expected<void, JointCtrllerErr> set_impedance_mode(JointImpedanceMode mode, const JointState& state);
    /**
     * @brief 设置关节命令
     * @param cmd 关节命令
     * @return tl::expected<void, JointCtrllerErr>
     */
    tl::expected<void, JointCtrllerErr> set_cmd(const JointCmd& cmd);
    /**
     * @brief 更新关节控制器
     * @param input 关节控制器输入
     * @return tl::expected<JointCtrllerOutput, JointCtrllerErr> 关节控制器输出
     */
    tl::expected<JointCtrllerOutput, JointCtrllerErr> update(const JointCtrllerInput& input);

private:
    JointCtrllerErr validate_cfg(const JointCtrllerCfg& cfg) const;
    JointCtrllerErr validate_state(const JointState& state) const;
    JointCtrllerErr validate_cmd(const JointCmd& cmd) const;
    bool is_finite_vector(const JointVector& vector) const;

private:
    JointCtrllerCfg cfg_;                                                   ///< 关节控制器配置
    JointImpedanceMode impedance_mode_{ JointImpedanceMode::RIGID_HOLD };   ///< 当前关节阻抗模式

    JointVector hold_pos_;  ///< 关节保持位置
    JointCmd cur_cmd_;      ///< 当前关节命令

    bool is_configured_{ false };   ///< 关节控制器是否已配置
    bool has_valid_cmd_{ false };   ///< 是否有有效的关节命令
};

} // namespace dm_arm