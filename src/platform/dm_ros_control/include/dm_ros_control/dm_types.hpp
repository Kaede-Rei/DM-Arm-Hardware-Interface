#pragma once

#include "dm_hw/damiao.hpp"

#include <cstdint>
#include <string>

namespace dm_ros_control {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief 控制模式枚举
 * @param MIT MIT 模式
 * @param POS_VEL 位置+速度模式
 */
enum class ControlMode {
    MIT,
    POS_VEL
};

struct DmMotorConfig {
    std::string joint_name;
    uint32_t motor_id;
    damiao::DmMotorType motor_type;
    double joint_to_motor_scale;
    ControlMode control_mode;
};

struct DmJointState {
    double position{ 0.0 };
    double velocity{ 0.0 };
    double effort{ 0.0 };
    double motor_effort{ 0.0 };
};

struct DmJointCommand {
    double position{ 0.0 };
    double velocity{ 0.0 };
    double effort{ 0.0 };
    double kp{ 0.0 };
    double kd{ 0.0 };
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //



}
