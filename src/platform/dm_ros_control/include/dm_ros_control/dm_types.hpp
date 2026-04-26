#pragma once

#include "dm_hw/damiao.hpp"

namespace dm_ros_control {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief 控制模式枚举
 */
enum class ControlMode {
    MIT,        ///< MIT 模式
    POS_VEL    ///< 位置+速度模式
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

/**
 * @brief 关节侧状态量，由电机反馈换算得到
 */
struct DmJointState {
    double position{ 0.0 };                ///< 关节位置
    double velocity{ 0.0 };                ///< 关节速度
    double effort{ 0.0 };                  ///< 关节侧力矩
    double motor_effort{ 0.0 };            ///< 电机侧原始力矩
};

/**
 * @brief 关节侧控制命令，进入 DmMotorBus 后再换算到电机侧
 */
struct DmJointCommand {
    double position{ 0.0 };                ///< 目标关节位置
    double velocity{ 0.0 };                ///< 目标关节速度
    double effort{ 0.0 };                  ///< 前馈/外部控制关节力矩
    double kp{ 0.0 };                      ///< MIT 模式比例增益
    double kd{ 0.0 };                      ///< MIT 模式微分增益
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //



}
