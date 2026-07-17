#pragma once

#include <tl/optional.hpp>

#include <vector>

namespace dm_arm {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

using JointVector = std::vector<double>;

/**
 * @brief 关节命令模式
 */
enum class JointCmdMode {
    HOLD,           ///< 保持当前位置
    POS,            ///< 位置模式
    VEL,            ///< 速度模式
    POS_VEL,        ///< 位置速度模式
    TOR,            ///< 力矩模式
    IMPEDANCE,      ///< 阻抗模式
};

/**
 * @brief 关节阻抗模式
 */
enum class JointImpedanceMode {
    RIGID_HOLD,         ///< 刚性保持模式
    COMPLIANT_HOLD,     ///< 柔性保持模式
    TRACKING,           ///< 跟踪模式
};

/**
 * @brief 关节命令错误类型
 */
enum class JointCtrllerErr {
    OK,                         ///< 关节控制器正常

    NOT_CONFIGURED,             ///< 关节控制器未配置
    NOT_INITIALIZED,             ///< 关节控制器未初始化

    INVALID_CFG,                ///< 关节控制器配置无效
    INVALID_STATE,              ///< 关节状态无效
    INVALID_DT,                 ///< 时间步长无效
    INVALID_MODEL_FEEDFORWARD,  ///< 模型前馈力矩无效

    MISSING_POS,                ///< 缺少位置命令
    MISSING_VEL,                ///< 缺少速度命令
    MISSING_TOR,                ///< 缺少力矩命令
    MISSING_GAINS,              ///< 缺少增益命令

    INVALID_POS,                ///< 无效的位置命令
    INVALID_VEL,                ///< 无效的速度命令
    INVALID_TOR,                ///< 无效的力矩命令
    INVALID_GAINS,              ///< 无效的增益命令
    INVALID_SIZE,               ///< 命令大小无效
    INVALID_MODE,               ///< 命令模式无效
};

/**
 * @brief 执行器状态
 */
struct ActuatorState {
    JointVector pos;            ///< 执行器位置
    JointVector vel;            ///< 执行器速度
    JointVector tor;            ///< 执行器力矩

    std::vector<bool> online;   ///< 执行器在线状态
    std::vector<bool> enabled;  ///< 执行器使能状态
    std::vector<int> err_code;  ///< 执行器错误码
};

/**
 * @brief 关节状态
 */
struct JointState {
    JointVector pos;    ///< 关节位置
    JointVector vel;    ///< 关节速度
    JointVector tor;    ///< 关节力矩
};

/**
 * @brief 关节阻抗增益
 */
struct JointImpedanceGains {
    JointVector kp;     ///< 关节比例增益
    JointVector kd;     ///< 关节阻尼增益
};

/**
 * @brief 关节命令
 */
struct JointCmd {
    JointCmdMode mode{ JointCmdMode::HOLD };            ///< 关节命令模式

    tl::optional<JointVector> pos;                      ///< 关节位置命令
    tl::optional<JointVector> vel;                      ///< 关节速度命令
    tl::optional<JointVector> tor;                      ///< 关节力矩命令

    tl::optional<JointImpedanceGains> gains;            ///< 关节阻抗增益命令
};

/**
 * @brief 关节执行器命令
 */
struct JointActuatorCmd {
    JointVector pos;    ///< 执行器位置命令
    JointVector vel;    ///< 执行器速度命令
    JointVector tor;    ///< 执行器力矩命令
    JointVector kp;     ///< 执行器比例增益命令
    JointVector kd;     ///< 执行器阻尼增益命令
};

} // namespace dm_arm
