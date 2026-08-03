#pragma once

#include "serial_arm/core/joint_actuator_mapper.hpp"
#include "serial_arm/core/safety.hpp"
#include "serial_arm/hardware/hardware_capability.hpp"
#include "serial_arm/model/model_loader.hpp"

#include <tl/expected.hpp>

#include <string>
#include <vector>

namespace serial_arm {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief Safety 运行策略配置
 */
struct SafetyPolicyCfg {
    double position_margin{ 0.0 };              ///< 命令位置软边距
    double cmd_vel_scale{ 1.0 };                ///< 命令速度比例
    double state_vel_scale{ 1.0 };              ///< 状态速度检查比例
    JointVector max_acc;                        ///< Joint 侧最大加速度
    JointVector max_effort_override;            ///< Joint 侧力矩收窄值
    JointVector max_kp_override;                ///< Joint 侧 kp 收窄值
    JointVector max_kd_override;                ///< Joint 侧 kd 收窄值
    double max_dt_s{ 0.0 };                     ///< 允许的最大控制周期
    double state_timeout_s{ 0.0 };              ///< 状态超时时间
    double cmd_timeout_s{ 0.0 };                ///< 命令超时时间
    bool require_all_actuators_online{ true };  ///< 是否要求所有执行器在线
    bool require_all_actuators_enabled{ true }; ///< 是否要求所有执行器使能
    bool reject_motor_error{ true };            ///< 是否拒绝电机错误码
    bool require_continuous_cmd{ true };         ///< 是否要求相邻命令连续
};

/**
 * @brief 解析后的单 Joint 限制
 */
struct ResolvedJointLimitCfg {
    std::string joint_name;                 ///< Joint 名称
    bool has_position_limit{ false };       ///< 是否存在位置限制
    double hard_min_pos{ 0.0 };             ///< 状态位置硬下限
    double hard_max_pos{ 0.0 };             ///< 状态位置硬上限
    double cmd_min_pos{ 0.0 };              ///< 命令位置软下限
    double cmd_max_pos{ 0.0 };              ///< 命令位置软上限
    double max_cmd_vel{ 0.0 };              ///< 命令速度上限
    double max_state_vel{ 0.0 };            ///< 状态速度检查上限
    double max_acc{ 0.0 };                  ///< 加速度上限
    double max_effort{ 0.0 };               ///< Joint 侧力矩上限
    double max_kp{ 0.0 };                   ///< Joint 侧 kp 上限
    double max_kd{ 0.0 };                   ///< Joint 侧 kd 上限
};

/**
 * @brief 解析后的 Safety 配置
 */
struct ResolvedSafetyCfg {
    std::vector<ResolvedJointLimitCfg> joints;       ///< 按 Joint 顺序排列的限制
    double max_dt_s{ 0.0 };                          ///< 允许的最大控制周期
    double state_timeout_s{ 0.0 };                   ///< 状态超时时间
    double cmd_timeout_s{ 0.0 };                     ///< 命令超时时间
    bool require_all_actuators_online{ true };       ///< 是否要求所有执行器在线
    bool require_all_actuators_enabled{ true };      ///< 是否要求所有执行器使能
    bool reject_motor_error{ true };                 ///< 是否拒绝电机错误码
    bool require_continuous_cmd{ true };             ///< 是否要求相邻命令连续
};

/**
 * @brief LimitResolver 错误类型
 */
enum class LimitResolverErr {
    INVALID_INPUT,      ///< 输入配置无效
    MISSING_ACTUATOR,   ///< 缺少执行器能力
    POLICY_WIDENS_LIMIT ///< Policy 尝试放宽限制
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief 解析 URDF、硬件能力、Calibration 与 Safety Policy 的最终限制
 */
class LimitResolver {
public:
    /**
     * @brief 生成 ResolvedSafetyCfg
     */
    tl::expected<ResolvedSafetyCfg, LimitResolverErr> resolve(const RobotModelInfo& model, const JointActuatorMapCfg& mapper, const HardwareCapabilities& capabilities, const SafetyPolicyCfg& policy) const;
};

/**
 * @brief 将 ResolvedSafetyCfg 转换为 SafetyCfg
 */
SafetyCfg to_safety_cfg(const ResolvedSafetyCfg& resolved);

/**
 * @brief 从 SafetyCfg 生成 ResolvedSafetyCfg
 */
ResolvedSafetyCfg resolve_from_safety_cfg(const std::vector<std::string>& joint_names, const SafetyCfg& cfg);

// ! ========================= 模 版 方 法 实 现 ========================= ! //

} // namespace serial_arm
