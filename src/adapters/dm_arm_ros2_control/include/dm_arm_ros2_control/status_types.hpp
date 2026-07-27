#pragma once

#include <vector>
#include <cstdint>

namespace dm_arm_ros2_control {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief 命令帧结构体
 */
struct CommandFrame {
    std::vector<double> pos;        ///< 位置命令
    std::vector<double> vel;        ///< 速度命令
    std::uint64_t sequence{ 0 };    ///< 序列号
};

/**
 * @brief 状态帧结构体
 */
struct StateFrame {
    std::vector<double> pos;                ///< 位置状态
    std::vector<double> vel;                ///< 速度状态
    std::vector<double> effort;             ///< 力矩状态
    std::vector<double> model_feedforward;  ///< 模型前馈
    std::vector<std::uint8_t> online;       ///< 在线状态
    std::vector<std::uint8_t> enabled;      ///< 使能状态
    std::vector<int> motor_error;           ///< 电机错误状态
    double cycle_dt{ 0.0 };                 ///< 控制周期
    bool valid{ false };                    ///< 是否有效
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //



// ! ========================= 模 版 方 法 实 现 ========================= ! //

} // dm_arm_ros2_control
