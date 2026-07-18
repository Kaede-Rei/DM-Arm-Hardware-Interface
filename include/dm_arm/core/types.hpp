#pragma once

#include <cstdint>
#include <variant>
#include <vector>

namespace dm_arm {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

using JointVector = std::vector<double>;
using ActuatorVector = std::vector<double>;

/**
 * @brief 关节控制器生命周期状态
 */
enum class JointCtrllerState {
    UNCONFIGURED,    ///< 关节控制器未配置
    CONFIGURED,      ///< 关节控制器已配置
    INITIALIZED,     ///< 关节控制器已初始化
};

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
 * @brief 关节控制器错误类型
 */
enum class JointCtrllerErr {
    OK,                         ///< 关节控制器正常

    NOT_CONFIGURED,             ///< 关节控制器未配置
    NOT_INITIALIZED,            ///< 关节控制器未初始化
    ALREADY_INITIALIZED,        ///< 关节控制器已初始化

    INVALID_CFG,                ///< 关节控制器配置无效
    INVALID_STATE,              ///< 关节状态无效
    INVALID_DT,                 ///< 时间步长无效
    INVALID_MODEL_FEEDFORWARD,  ///< 模型前馈力矩无效
    INVALID_IMPEDANCE_MODE,     ///< 关节阻抗模式无效

    INVALID_CMD_SIZE,           ///< 关节参考命令大小无效
    INVALID_CMD_VALUE,          ///< 关节参考命令数值无效
    INVALID_FULL_CMD,           ///< 关节完整控制命令无效

    CMD_NOT_ALLOWED_IN_MODE,    ///< 当前阻抗模式不允许设置关节参考命令
    FULL_CMD_NOT_ALLOWED,       ///< 不允许直接设置关节完整控制命令
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
 * 所有数据均位于执行器侧，其具体含义由执行器驱动定义
 */
struct ActuatorState {
    ActuatorVector pos;             ///< 执行器位置
    ActuatorVector vel;             ///< 执行器速度
    ActuatorVector tor;             ///< 执行器力矩

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
 * @brief 执行器 MIT 命令
 *
 * 该命令已经完成关节侧到执行器侧的映射，
 * 可由达妙驱动进一步编码为 MIT 协议数据
 */
struct ActuatorMitCmd {
    ActuatorVector pos;    ///< 执行器目标位置
    ActuatorVector vel;    ///< 执行器目标速度
    ActuatorVector tor;    ///< 执行器前馈力矩
    ActuatorVector kp;     ///< 执行器 MIT 比例增益
    ActuatorVector kd;     ///< 执行器 MIT 阻尼增益
};

} // namespace dm_arm
