#pragma once

#include <tl/expected.hpp>

#include "dm_arm/core/types.hpp"

#include <cstddef>
#include <vector>

namespace dm_arm {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief 关节执行器映射错误类型
 */
enum class JointActuatorMapErr {
    OK,                         ///< 关节执行器映射正常

    NOT_CONFIGURED,             ///< 关节执行器映射未配置
    INVALID_CFG,                ///< 关节执行器映射配置无效
    INVALID_JOINT_STATE,        ///< 关节状态无效
    INVALID_ACTUATOR_STATE,     ///< 执行器状态无效
    INVALID_JOINT_CMD,          ///< 关节控制命令无效
    INVALID_ACTUATOR_CMD,       ///< 执行器 MIT 命令无效
    INVALID_CONVERSION_VALUE,   ///< 映射结果无效
};

/**
 * @brief 关节执行器映射配置
 *
 * pos_ratio 定义为：
 * 执行器位置变化量 / 关节位置变化量
 *
 * tor_ratio 定义为：
 * 关节力矩 / 执行器报告力矩
 *
 * 当执行器 SDK 已经输出减速器输出端位置和力矩时，
 * pos_ratio 与 tor_ratio 通常均配置为 1.0
 */
struct JointActuatorMapCfg {
    std::size_t joints_count{ 0 };            ///< 关节与执行器数量

    ActuatorVector pos_ratio;                 ///< 执行器位置与关节位置比例
    ActuatorVector tor_ratio;                 ///< 关节力矩与执行器力矩比例
    std::vector<int> direction;               ///< 关节执行器方向，取 1 或 -1

    JointVector joint_zero_offset;            ///< 关节侧零位偏置
    ActuatorVector actuator_zero_offset;      ///< 执行器侧零位偏置
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief 关节执行器映射器
 */
class JointActuatorMapper {
public:
    /**
     * @brief 配置关节执行器映射器
     * @param cfg 关节执行器映射配置
     * @return tl::expected<void, JointActuatorMapErr>
     */
    tl::expected<void, JointActuatorMapErr> configure(const JointActuatorMapCfg& cfg);

    /**
     * @brief 将关节控制命令转换为执行器 MIT 命令
     * @param joint_cmd 关节侧完整控制命令
     * @return tl::expected<ActuatorMitCmd, JointActuatorMapErr> 执行器 MIT 命令
     */
    tl::expected<ActuatorMitCmd, JointActuatorMapErr> to_actuator_cmd(const JointCtrlCmd& joint_cmd) const;

    /**
     * @brief 将执行器状态转换为关节状态
     * @param actuator_state 执行器状态
     * @return tl::expected<JointState, JointActuatorMapErr> 关节状态
     */
    tl::expected<JointState, JointActuatorMapErr> to_joint_state(const ActuatorState& actuator_state) const;

    /**
     * @brief 获取映射数量
     * @return std::size_t 映射数量
     */
    std::size_t size() const noexcept;

private:
    /**
     * @brief 验证关节执行器映射配置
     * @param cfg 关节执行器映射配置
     * @return JointActuatorMapErr 验证结果
     */
    JointActuatorMapErr validate_cfg(const JointActuatorMapCfg& cfg) const;

    /**
     * @brief 验证关节控制命令
     * @param cmd 关节控制命令
     * @return JointActuatorMapErr 验证结果
     */
    JointActuatorMapErr validate_joint_cmd(const JointCtrlCmd& cmd) const;

    /**
     * @brief 验证执行器状态
     * @param state 执行器状态
     * @return JointActuatorMapErr 验证结果
     */
    JointActuatorMapErr validate_actuator_state(const ActuatorState& state) const;

    /**
     * @brief 检查向量是否为有限值
     * @param vector 待检查的向量
     * @return true 如果向量中的所有元素都是有限值，否则返回 false
     */
    bool is_finite_vector(const std::vector<double>& vector) const;

private:
    JointActuatorMapCfg cfg_;    ///< 关节执行器映射配置
    bool configured_{ false };   ///< 关节执行器映射器是否已配置
};

} // namespace dm_arm
