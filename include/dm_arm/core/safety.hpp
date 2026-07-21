#pragma once

#include "dm_arm/core/types.hpp"

#include <tl/expected.hpp>
#include <limits>

namespace dm_arm {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief 安全检查错误类型
 */
enum class SafetyErr {
    NOT_CONFIGURED,              ///< Safety 尚未配置
    INVALID_CFG,                 ///< Safety 配置无效

    INVALID_DT,                  ///< 控制周期无效
    INVALID_STATE_AGE,           ///< 状态年龄无效
    INVALID_CMD_AGE,             ///< 命令年龄无效
    STATE_TIMEOUT,               ///< 状态超时
    CMD_TIMEOUT,                 ///< 上层命令超时

    INVALID_JOINT_STATE_SIZE,    ///< JointState 数组长度错误
    INVALID_ACTUATOR_STATE_SIZE, ///< ActuatorState 数组长度错误
    NON_FINITE_JOINT_STATE,      ///< JointState 包含 NaN/Inf
    NON_FINITE_ACTUATOR_STATE,   ///< ActuatorState 包含 NaN/Inf

    JOINT_POS_LIMIT,             ///< 关节状态位置超过硬限制
    JOINT_VEL_LIMIT,             ///< 关节状态速度超过限制

    ACTUATOR_OFFLINE,            ///< 至少一个执行器离线
    ACTUATOR_NOT_ENABLED,        ///< 至少一个执行器未使能
    ACTUATOR_FAULT,              ///< 至少一个执行器上报故障

    INVALID_CMD_SIZE,            ///< JointCtrlCmd 数组长度错误
    NON_FINITE_CMD,              ///< JointCtrlCmd 包含 NaN/Inf
    CMD_POS_LIMIT,               ///< 目标位置超过软限制
    CMD_VEL_LIMIT,               ///< 目标速度超过限制
    CMD_EFFORT_LIMIT,            ///< 前馈力矩超过限制
    CMD_KP_LIMIT,                ///< kp 超过限制
    CMD_KD_LIMIT,                ///< kd 超过限制
    CMD_POS_STEP_LIMIT,          ///< 相邻合法命令位置跳变过大
    CMD_VEL_STEP_LIMIT,          ///< 相邻合法命令速度跳变过大
};

/**
 * @brief Safety 检测出的故障详情
 */
struct SafetyFault {
    SafetyErr code{ SafetyErr::INVALID_CFG };                       ///< Safety 错误码
    std::size_t index{ std::numeric_limits<std::size_t>::max() };   ///< 关节/执行器索引，若不适用则为 std::numeric_limits<std::size_t>::max()
    double value{ 0.0 };                                            ///< 关节/执行器状态或命令值，若不适用则为 0.0
    double limit{ 0.0 };                                            ///< Safety 配置中对应的限制值，若不适用则为 0.0
};

/**
 * @brief Safety 建议 Robot 执行的故障动作
 */
enum class SafetyAction {
    STOP_HOLD,  ///< 状态可信，拒绝错误命令并尝试低风险保持
    DISABLE,    ///< 状态/硬件不可信，直接失能
};

/**
 * @brief Joint 侧软件安全限制
 */
struct JointLimitCfg {
    JointVector min_pos;       ///< 状态位置硬下限，rad
    JointVector max_pos;       ///< 状态位置硬上限，rad
    JointVector max_vel;       ///< 最大速度，rad/s
    JointVector max_acc;       ///< 最大加速度，rad/s^2
    JointVector max_effort;    ///< 最大关节前馈力矩，N·m
    JointVector max_kp;        ///< 最大 Joint 侧 kp
    JointVector max_kd;        ///< 最大 Joint 侧 kd
    JointVector pos_margin;    ///< 命令位置距离硬限位的安全边距，rad
};

/**
 * @brief Safety 配置
 */
struct SafetyCfg {
    std::size_t joints_count{ 0 };              ///< Joint/Actuator 数量
    JointLimitCfg limits;                       ///< Joint 侧安全限制

    double cmd_timeout_s{ 0.10 };               ///< 跟踪命令超时
    double state_timeout_s{ 0.05 };             ///< 状态超时
    double max_dt_s{ 0.02 };                    ///< 允许的最大控制周期
    double numeric_tolerance{ 1.0e-6 };         ///< 浮点误差级越界容差

    bool require_all_actuators_online{ true };  ///< ACTIVE 时要求所有执行器在线
    bool require_all_actuators_enabled{ true }; ///< ACTIVE 时要求所有执行器已使能
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief Joint/Actuator 安全检查器
 */
class Safety {
public:
    /**
     * @brief 配置 Safety
     * @param cfg Safety 配置
     * @return 配置成功返回空，配置失败返回 SafetyFault
     */
    tl::expected<void, SafetyFault> configure(const SafetyCfg& cfg);

    /**
     * @brief 检查 ACTIVE 周期中的 Joint 与 Actuator 状态
     * @param joint_state Joint 侧状态
     * @param actuator_state Actuator 侧状态
     * @param state_age_s 距离上一帧合法状态的时间
     * @return 状态合法返回空，状态不合法返回 SafetyFault
     */
    tl::expected<void, SafetyFault> check_state(const JointState& joint_state, const ActuatorState& actuator_state, double state_age_s) const;
    /**
     * @brief 检查跟踪命令是否超时
     * @param cmd_age_s 距离上一帧合法命令的时间
     * @return 命令合法返回空，命令不合法返回 SafetyFault
     */
    tl::expected<void, SafetyFault> check_cmd_age(double cmd_age_s) const;
    /**
     * @brief 检查并返回可安全下发的 JointCtrlCmd
     * @param state 当前 Joint 侧状态
     * @param cmd 上层下发的 JointCtrlCmd
     * @param dt 当前控制周期
     * @return 命令合法返回安全命令，命令不合法返回 SafetyFault
     */
    tl::expected<JointCtrlCmd, SafetyFault> check_joint_cmd(const JointState& state, const JointCtrlCmd& cmd, double dt);

    /**
     * @brief 使用当前位置和零目标速度初始化/重置命令历史
     * @param state 当前 Joint 侧状态
     * @return 成功返回空，失败返回 SafetyFault
     */
    tl::expected<void, SafetyFault> reset_cmd_history(const JointState& state);
    /**
     * @brief 清空命令历史
     */
    void clear_cmd_history() noexcept;

    /**
     * @brief 将安全错误映射为 Robot 应执行的动作
     * @param err Safety 错误码
     * @return Robot 应执行的动作
     */
    SafetyAction action_for(SafetyErr err) const noexcept;

    /**
     * @brief 检查 Safety 是否已配置
     * @return Safety 是否已配置
     */
    bool is_configured() const noexcept;
    /**
     * @brief 限幅次数
     * @return Safety 限幅次数
     */
    std::uint64_t clamp_count() const noexcept;

private:
    /**
     * @brief 验证 Safety 配置
     * @param cfg Safety 配置
     * @return 配置合法返回空，配置不合法返回 SafetyFault
     */
    tl::expected<void, SafetyFault> validate_cfg(const SafetyCfg& cfg) const;

    /**
     * @brief 检查 Joint/Actuator 状态数组长度
     * @param state Joint 侧状态
     * @return Joint/Actuator 状态数组长度是否正确
     */
    bool has_joint_size(const JointState& state) const noexcept;
    /**
     * @brief 检查 Joint/Actuator 状态数组长度
     * @param state Actuator 侧状态
     * @return Joint/Actuator 状态数组长度是否正确
     */
    bool has_actuator_size(const ActuatorState& state) const noexcept;
    /**
     * @brief 检查 JointCtrlCmd 数组长度
     * @param cmd 上层下发的 JointCtrlCmd
     * @return JointCtrlCmd 数组长度是否正确
     */
    bool has_cmd_size(const JointCtrlCmd& cmd) const noexcept;
    /**
     * @brief 检查 Joint/Actuator 状态是否包含 NaN/Inf
     * @param state Joint 侧状态
     * @return Joint/Actuator 状态是否包含 NaN/Inf
     */
    bool is_finite_vector(const std::vector<double>& values) const noexcept;

    /**
     * @brief 检查并限幅浮点值，若超出 min/max 范围则 clamp 到边界
     * @param value 待检查的浮点值，若超出范围则被修改为边界值
     * @param min_value 最小值
     * @param max_value 最大值
     * @return 若 value 超出范围则返回 true，否则返回 false
     */
    bool clamp_small_violation(double& value, double min_value, double max_value) noexcept;

private:
    SafetyCfg cfg_;                       ///< Safety 配置
    JointCtrlCmd last_accepted_cmd_;      ///< 上一帧通过检查的完整命令

    bool configured_{ false };            ///< 是否已配置
    bool has_last_accepted_cmd_{ false }; ///< 是否存在命令历史
    std::uint64_t clamp_count_{ 0 };      ///< 浮点误差级 clamp 次数
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //

} // namespace dm_arm
