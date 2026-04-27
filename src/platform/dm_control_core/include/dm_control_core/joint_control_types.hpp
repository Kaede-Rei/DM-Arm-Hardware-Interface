#pragma once

#include <string>
#include <vector>

namespace dm_control_core {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief 关节侧观测状态，不依赖 ROS 消息或 ros2_control 接口
 */
struct JointState {
    double position{ 0.0 };          ///< 当前关节位置
    double velocity{ 0.0 };          ///< 当前关节速度
    double effort{ 0.0 };            ///< 当前关节侧测得力矩
    double motor_effort{ 0.0 };      ///< 电机侧原始力矩；无原始电机反馈时保持 0
};

/**
 * @brief 关节侧参考输入，可由 ROS、LeRobot、Isaac 或其它上层策略生成
 */
struct JointReference {
    double position{ 0.0 };          ///< 目标关节位置
    double velocity{ 0.0 };          ///< 目标关节速度
    double effort{ 0.0 };            ///< 上层外部力矩或 residual effort
};

/**
 * @brief 关节阻抗参数
 */
struct JointImpedanceGains {
    double kp{ 0.0 };                ///< 位置刚度
    double kd{ 0.0 };                ///< 速度阻尼
};

/**
 * @brief 动力学/补偿前馈项
 */
struct JointFeedforward {
    double effort{ 0.0 };            ///< 当前需要叠加的关节侧前馈力矩
};

/**
 * @brief MIT 模式关节侧命令
 */
struct MitJointCommand {
    double position{ 0.0 };          ///< 下发到 MIT 模式的位置参考
    double velocity{ 0.0 };          ///< 下发到 MIT 模式的速度参考
    double effort{ 0.0 };            ///< 下发到 MIT 模式的力矩前馈
    double kp{ 0.0 };                ///< 下发到 MIT 模式的位置刚度
    double kd{ 0.0 };                ///< 下发到 MIT 模式的速度阻尼
};

/**
 * @brief 多关节配置描述，供非 ROS 适配层复用
 */
struct JointControlLayout {
    std::vector<std::string> joint_names;     ///< 受控关节顺序
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //



}
