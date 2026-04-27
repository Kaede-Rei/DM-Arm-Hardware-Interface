#pragma once

#include "dm_hw/damiao.hpp"

#include <cstdint>
#include <string>

namespace dm_control_core {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief 控制模式枚举
 */
enum class ControlMode {
    MIT,        ///< MIT 模式
    POS_VEL     ///< 位置+速度模式
};

/**
 * @brief 单个关节对应的达妙电机配置
 */
struct DmMotorConfig {
    std::string joint_name;                ///< ROS 关节名称
    uint32_t motor_id;                     ///< 达妙电机 CAN ID
    damiao::DmMotorType motor_type;        ///< 达妙电机型号
    double joint_to_motor_scale;           ///< 关节侧到电机侧的位置/速度比例
    ControlMode control_mode;              ///< 电机控制模式
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //



}
