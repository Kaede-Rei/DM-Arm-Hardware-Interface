#pragma once

#include "tl/optional.hpp"

#include <string>
#include <vector>

namespace dm_control_core {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief 关节阻抗控制模式
 */
enum class JointImpedanceMode {
    RIGID_HOLD,                     ///< 刚性保持模式
    COMPLIANT_HOLD,                 ///< 柔顺保持模式
    TRACKING                        ///< 跟随命令模式
};

/**
 * @brief 上层关节命令语义
 */
enum class JointCommandMode {
    HOLD,                           ///< 保持当前位置
    POSITION,                       ///< 位置命令，速度目标为 0
    POSITION_VELOCITY,              ///< 位置+速度命令
    IMPEDANCE,                      ///< 位置+速度+残差力矩命令
    VELOCITY,                       ///< 速度命令，位置刚度关闭
    TORQUE                          ///< 纯力矩命令，位置/速度刚度关闭
};

/**
 * @brief 关节命令校验错误
 */
enum class JointCommandError {
    MISSING_POSITION,               ///< 当前模式需要位置命令
    MISSING_VELOCITY,               ///< 当前模式需要速度命令
    MISSING_EFFORT,                 ///< 当前模式需要力矩命令
    INVALID_POSITION_SIZE,          ///< 位置命令长度错误
    INVALID_VELOCITY_SIZE,          ///< 速度命令长度错误
    INVALID_EFFORT_SIZE,            ///< 力矩命令长度错误
    INVALID_STATE_SIZE              ///< 状态长度错误
};

/**
 * @brief 关节侧观测状态，不依赖 ROS 消息或 ros2_control 接口
 */
struct JointState {
    std::vector<double> position;       ///< 当前关节位置
    std::vector<double> velocity;       ///< 当前关节速度
    std::vector<double> effort;         ///< 当前关节侧测得力矩
    std::vector<double> motor_effort;   ///< 电机侧原始力矩；无原始电机反馈时保持 0
};

/**
 * @brief 上层关节命令，使用 optional 区分未提供和 0.0 命令
 */
struct JointCommand {
    JointCommandMode mode{ JointCommandMode::HOLD };            ///< 命令语义
    tl::optional<std::vector<double>> position;                 ///< 目标关节位置
    tl::optional<std::vector<double>> velocity;                 ///< 目标关节速度
    tl::optional<std::vector<double>> effort;                   ///< 残差力矩或纯力矩，取决于 mode
};

/**
 * @brief 关节阻抗参数
 */
struct JointImpedanceGains {
    std::vector<double> kp;             ///< 位置刚度
    std::vector<double> kd;             ///< 速度阻尼
};

/**
 * @brief MIT 模式关节侧命令
 */
struct MitJointCommand {
    std::vector<double> position;       ///< 下发到 MIT 模式的位置参考
    std::vector<double> velocity;       ///< 下发到 MIT 模式的速度参考
    std::vector<double> effort;         ///< 下发到 MIT 模式的力矩前馈
    std::vector<double> kp;             ///< 下发到 MIT 模式的位置刚度
    std::vector<double> kd;             ///< 下发到 MIT 模式的速度阻尼
};

/**
 * @brief 多关节配置描述，供非 ROS 适配层复用
 */
struct JointControlLayout {
    std::vector<std::string> joint_names;     ///< 受控关节顺序
};

/**
 * @brief 关节命令限幅配置
 */
struct JointCommandLimits {
    std::vector<double> max_velocity;       ///< 速度绝对值上限，非正数表示不限制
    std::vector<double> max_effort;         ///< 力矩绝对值上限，非正数表示不限制
    std::vector<double> min_kp;             ///< 位置刚度下限
    std::vector<double> max_kp;             ///< 位置刚度上限
    std::vector<double> min_kd;             ///< 速度阻尼下限
    std::vector<double> max_kd;             ///< 速度阻尼上限
};

/**
 * @brief 关节阻抗控制器配置
 */
struct JointImpedanceControllerConfig {
    JointControlLayout layout;                          ///< 多关节布局
    JointImpedanceGains rigid_hold_gains;               ///< 刚性保持模式阻抗参数
    JointImpedanceGains compliant_hold_gains;           ///< 柔顺保持模式阻抗参数
    JointImpedanceGains tracking_gains;                 ///< 跟踪模式阻抗参数
    JointCommandLimits limits;                          ///< 命令限幅配置
    bool use_model_feedforward{ true };                 ///< 是否叠加 input.model_feedforward
    bool use_command_effort{ true };                    ///< 是否叠加 JointCommand::effort
};

/**
 * @brief 关节阻抗控制器周期输入
 */
struct JointImpedanceControllerInput {
    JointState state;                           ///< 当前关节状态
    std::vector<double> model_feedforward;      ///< 与 JointControlLayout::joint_names 对齐的当前模型前馈力矩
    double dt{ 0.0 };                           ///< 控制周期
};

/**
 * @brief 关节阻抗控制器周期输出
 */
struct JointImpedanceControllerOutput {
    bool valid{ false };                        ///< 输出是否有效（输入状态长度不匹配时为 false）
    MitJointCommand command;                    ///< 与 JointControlLayout::joint_names 对齐的 MIT 命令
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //



}
