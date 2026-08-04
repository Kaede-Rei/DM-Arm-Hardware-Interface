#pragma once

#include <cstdint>
#include <variant>
#include <vector>

namespace serial_arm {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

using JointVector = std::vector<double>;
using ActuatorVector = std::vector<double>;

/**
 * @brief 关节阻抗模式
 */
enum class JointImpedanceMode {
    RIGID_HOLD,             ///< 刚性保持模式
    RIGID_TRACKING,         ///< 刚性跟踪模式
    COMPLIANT_HOLD,         ///< 柔性保持模式
    COMPLIANT_DRAG,         ///< 柔性拖拽模式
    COMPLIANT_TRACKING,     ///< 柔性跟踪模式
};

/**
 * @brief 模型前馈策略
 */
enum class ModelFeedforwardMode {
    NONE,                   ///< 不使用模型前馈
    GRAVITY,                ///< 重力补偿 g(q)
    FULL_INVERSE_DYNAMICS,  ///< 完整逆动力学前馈
};

/**
 * @brief 关节状态
 *
 * 所有数据均位于关节侧：
 * - pos: rad
 * - vel: rad/s
 * - tor: N·m
 */
struct JointState {
    JointVector pos;    ///< 关节位置
    JointVector vel;    ///< 关节速度
    JointVector tor;    ///< 关节力矩
};

/**
 * @brief 执行器状态
 *
 * Core 统一定义执行器侧状态单位；Hardware Backend 必须将厂商协议
 * 反馈、厂商单位、电流或其他底层量转换为以下 SerialArm 语义：
 * - pos: rad
 * - vel: rad/s
 * - tor: N·m
 */
struct ActuatorState {
    ActuatorVector pos;             ///< 执行器侧位置，单位 rad
    ActuatorVector vel;             ///< 执行器侧速度，单位 rad/s
    ActuatorVector tor;             ///< 执行器侧力矩，单位 N·m

    std::vector<std::uint8_t> online;   ///< 执行器在线状态
    std::vector<std::uint8_t> enabled;  ///< 执行器使能状态
    std::vector<int> err_code;          ///< 执行器错误码
};

/**
 * @brief 关节阻抗增益
 *
 * 增益均为关节侧增益：
 * - kp: N·m/rad
 * - kd: N·m·s/rad
 */
struct JointImpedanceGains {
    JointVector kp;     ///< 关节比例增益
    JointVector kd;     ///< 关节阻尼增益
};

/**
 * @brief 关节位置命令
 */
struct JointPosCmd {
    JointVector pos;    ///< 关节位置命令
};

/**
 * @brief 关节位置速度命令
 */
struct JointPosVelCmd {
    JointVector pos;    ///< 关节位置命令
    JointVector vel;    ///< 关节速度命令
};

/**
 * @brief 关节位置速度力矩命令
 */
struct JointPosVelTorCmd {
    JointVector pos;    ///< 关节位置命令
    JointVector vel;    ///< 关节速度命令
    JointVector tor;    ///< 关节附加力矩命令
};

/**
 * @brief 关节参考命令
 */
using JointCmd = std::variant<
    JointPosCmd,
    JointPosVelCmd,
    JointPosVelTorCmd
>;

/**
 * @brief 关节完整控制命令
 *
 * 该命令由关节控制器输出，所有数据均位于关节侧
 * 在发送给执行器前，必须经过关节执行器映射
 */
struct JointCtrlCmd {
    JointVector pos;    ///< 关节目标位置
    JointVector vel;    ///< 关节目标速度
    JointVector tor;    ///< 关节前馈力矩
    JointVector kp;     ///< 关节比例增益
    JointVector kd;     ///< 关节阻尼增益
};

/**
 * @brief 执行器控制命令
 *
 * 该命令已经完成关节侧到执行器侧的映射；Hardware Backend 必须按
 * Core 定义的 pos / vel / tor / kp / kd 协议解释，并转换为厂商协议、
 * 厂商单位、电流或其他底层量
 * - pos: rad
 * - vel: rad/s
 * - tor: N·m
 * - kp: N·m/rad
 * - kd: N·m·s/rad
 */
struct ActuatorCtrlCmd {
    ActuatorVector pos;    ///< 执行器侧目标位置，单位 rad
    ActuatorVector vel;    ///< 执行器侧目标速度，单位 rad/s
    ActuatorVector tor;    ///< 执行器侧前馈/目标力矩，单位 N·m
    ActuatorVector kp;     ///< 执行器侧等效位置刚度，单位 N·m/rad
    ActuatorVector kd;     ///< 执行器侧等效速度阻尼，单位 N·m·s/rad
};

} // namespace serial_arm
