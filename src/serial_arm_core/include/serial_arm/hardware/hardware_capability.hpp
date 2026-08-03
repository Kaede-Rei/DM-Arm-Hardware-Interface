#pragma once

#include "serial_arm/config/config.hpp"

#include <tl/expected.hpp>

#include <string>
#include <vector>

namespace serial_arm {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief 单个执行器物理能力
 */
struct ActuatorCapability {
    std::string actuator_name; ///< 执行器名称
    double min_pos{ 0.0 };     ///< 执行器最小位置
    double max_pos{ 0.0 };     ///< 执行器最大位置
    double max_vel{ 0.0 };     ///< 执行器最大速度
    double max_effort{ 0.0 };  ///< 执行器最大力矩
    double max_kp{ 0.0 };      ///< 执行器侧最大 kp
    double max_kd{ 0.0 };      ///< 执行器侧最大 kd
};

using HardwareCapabilities = std::vector<ActuatorCapability>;

/**
 * @brief 硬件能力错误类型
 */
enum class HardwareCapabilityErr {
    INVALID_CFG,         ///< 硬件配置无效
    UNKNOWN_MOTOR_TYPE,  ///< 未知电机型号
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief 从达妙硬件配置读取执行器物理能力
 */
tl::expected<HardwareCapabilities, HardwareCapabilityErr> load_damiao_capabilities(const DamiaoBusCfg& cfg);

// ! ========================= 模 版 方 法 实 现 ========================= ! //

} // namespace serial_arm
