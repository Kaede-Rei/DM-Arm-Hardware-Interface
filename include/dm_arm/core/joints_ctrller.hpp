#pragma once

#include <tl/expected.hpp>

#include "dm_arm/core/types.hpp"

#include <cstddef>

namespace dm_arm {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief 关节控制器配置
 */
struct JointCtrllerCfg {
    std::size_t joints_count{ 0 };                       ///< 关节数量

    JointImpedanceGains rigid_hold_gains;                ///< 刚性保持模式增益
    JointImpedanceGains rigid_tracking_gains;            ///< 刚性跟踪模式增益
    JointImpedanceGains compliant_hold_gains;            ///< 柔性保持模式增益
    JointImpedanceGains compliant_drag_gains;            ///< 柔性拖拽模式增益
    JointImpedanceGains compliant_tracking_gains;        ///< 柔性跟踪模式增益

    bool allow_full_cmd{ false };                         ///< 是否允许直接设置关节完整控制命令
};

/**
 * @brief 关节控制器输入
 */
struct JointCtrllerInput {
    JointState state;               ///< 关节状态
    JointVector model_feedforward;  ///< 关节侧模型前馈力矩
    double dt{ 0.0 };               ///< 时间步长
};

/**
 * @brief 关节控制器输出
 */
struct JointCtrllerOutput {
    JointCtrlCmd cmd;               ///< 关节侧完整控制命令
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
     * @brief 初始化关节控制器
     * @param state 初始关节状态
     * @return tl::expected<void, JointCtrllerErr>
     */
    tl::expected<void, JointCtrllerErr> initialize(const JointState& state);

    /**
     * @brief 设置关节阻抗模式
     * @param mode 关节阻抗模式
     * @param state 当前关节状态
     * @return tl::expected<void, JointCtrllerErr>
     */
    tl::expected<void, JointCtrllerErr> set_impedance_mode(
        JointImpedanceMode mode,
        const JointState& state);

    /**
     * @brief 设置关节参考命令
     * @param cmd 关节参考命令
     * @return tl::expected<void, JointCtrllerErr>
     */
    tl::expected<void, JointCtrllerErr> set_cmd(const JointCmd& cmd);

    /**
     * @brief 直接设置关节完整控制命令
     * @param cmd 关节侧完整控制命令
     * @return tl::expected<void, JointCtrllerErr>
     */
    tl::expected<void, JointCtrllerErr> set_full_cmd(const JointCtrlCmd& cmd);

    /**
     * @brief 更新关节控制器
     * @param input 关节控制器输入
     * @return tl::expected<JointCtrllerOutput, JointCtrllerErr> 关节控制器输出
     */
    tl::expected<JointCtrllerOutput, JointCtrllerErr> update(const JointCtrllerInput& input);

    /**
     * @brief 获取关节控制器生命周期状态
     * @return JointCtrllerState 关节控制器生命周期状态
     */
    JointCtrllerState get_state() const noexcept;

    /**
     * @brief 获取当前关节阻抗模式
     * @return JointImpedanceMode 当前关节阻抗模式
     */
    JointImpedanceMode get_impedance_mode() const noexcept;

private:
    /**
     * @brief 验证关节控制器配置
     * @param cfg 关节控制器配置
     * @return JointCtrllerErr 验证结果
     */
    JointCtrllerErr validate_cfg(const JointCtrllerCfg& cfg) const;

    /**
     * @brief 验证关节状态
     * @param state 关节状态
     * @return JointCtrllerErr 验证结果
     */
    JointCtrllerErr validate_state(const JointState& state) const;

    /**
     * @brief 验证关节参考命令
     * @param cmd 关节参考命令
     * @return JointCtrllerErr 验证结果
     */
    JointCtrllerErr validate_cmd(const JointCmd& cmd) const;

    /**
     * @brief 验证关节完整控制命令
     * @param cmd 关节完整控制命令
     * @return JointCtrllerErr 验证结果
     */
    JointCtrllerErr validate_full_cmd(const JointCtrlCmd& cmd) const;

    /**
     * @brief 验证关节阻抗模式
     * @param mode 关节阻抗模式
     * @return JointCtrllerErr 验证结果
     */
    JointCtrllerErr validate_impedance_mode(JointImpedanceMode mode) const;

    /**
     * @brief 检查当前关节阻抗模式是否为跟踪模式
     * @return true 当前为跟踪模式，否则返回 false
     */
    bool is_tracking_mode() const noexcept;

    /**
     * @brief 检查向量是否为有限值
     * @param vector 待检查的向量
     * @return true 如果向量中的所有元素都是有限值，否则返回 false
     */
    bool is_finite_vector(const JointVector& vector) const;

    /**
     * @brief 生成关节保持控制命令
     * @param hold_pos 关节保持位置
     * @param gains 关节阻抗增益
     * @param model_feedforward 模型前馈力矩
     * @return JointCtrlCmd 关节完整控制命令
     */
    JointCtrlCmd build_hold_cmd(const JointVector& hold_pos, const JointImpedanceGains& gains, const JointVector& model_feedforward) const;

    /**
     * @brief 生成关节拖拽控制命令
     * @param state 当前关节状态
     * @param gains 关节阻抗增益
     * @param model_feedforward 模型前馈力矩
     * @return JointCtrlCmd 关节完整控制命令
     */
    JointCtrlCmd build_drag_cmd(const JointState& state, const JointImpedanceGains& gains, const JointVector& model_feedforward) const;

    /**
     * @brief 生成关节跟踪控制命令
     * @param gains 关节阻抗增益
     * @param model_feedforward 模型前馈力矩
     * @return tl::expected<JointCtrlCmd, JointCtrllerErr> 关节完整控制命令
     */
    tl::expected<JointCtrlCmd, JointCtrllerErr> build_tracking_cmd(const JointImpedanceGains& gains, const JointVector& model_feedforward) const;

private:
    JointCtrllerCfg cfg_;                                                   ///< 关节控制器配置
    JointCtrllerState state_{ JointCtrllerState::UNCONFIGURED };            ///< 关节控制器生命周期状态
    JointImpedanceMode impedance_mode_{ JointImpedanceMode::RIGID_HOLD };   ///< 当前关节阻抗模式

    JointVector hold_pos_;          ///< 关节保持位置
    JointVector fallback_pos_;      ///< 关节跟踪回退位置

    JointCmd cur_cmd_{ JointPosCmd{} };  ///< 当前关节参考命令
    JointCtrlCmd full_cmd_;              ///< 当前关节完整控制命令

    bool has_cmd_{ false };              ///< 是否存在有效的关节参考命令
    bool has_full_cmd_{ false };         ///< 是否存在有效的关节完整控制命令
};

} // namespace dm_arm
