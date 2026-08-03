#pragma once

#include <tl/expected.hpp>

#include "serial_arm/core/joint_actuator_mapper.hpp"
#include "serial_arm/core/joints_ctrller.hpp"
#include "serial_arm/core/safety.hpp"
#include "serial_arm/hardware/hardware_capability.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace serial_arm {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief Robot 运行时配置
 */
struct RuntimeCfg {
    double ctrl_frequency_hz{ 200.0 };                 ///< 控制循环频率
    double joint_acc_filter_alpha{ 0.2 };              ///< 关节加速度低通滤波系数
    bool write_enabled{ false };                       ///< 是否允许下发执行器命令
    ModelFeedforwardMode model_feedforward_mode{       ///< 模型前馈策略
        ModelFeedforwardMode::NONE
    };
    JointImpedanceMode tracking_impedance_mode{    ///< 跟踪阻抗模式
        JointImpedanceMode::RIGID_TRACKING
    };
};

/**
 * @brief 正常停机配置
 */
struct ShutdownCfg {
    bool park_before_disable{ true };          ///< 正常停机前是否先回到停放姿态
    JointVector park_pos;                      ///< 停放姿态关节位置
    double speed_scale{ 0.1 };                 ///< 停放轨迹速度比例
    double position_tolerance{ 0.03 };         ///< 停放姿态位置误差阈值
    double velocity_tolerance{ 0.05 };         ///< 停放姿态速度阈值
    double settle_time_s{ 0.25 };              ///< 严格判据下持续稳定时间
    double relaxed_tolerance_ratio{ 2.0 };     ///< 超时前允许使用的宽松判据倍率
    double timeout_s{ 15.0 };                  ///< 停放流程最大允许时间
};

/**
 * @brief 动力学模块配置
 */
struct DynamicsCfg {
    std::string urdf_path;                              ///< URDF 文件路径
    std::vector<std::string> joint_names;               ///< 受控关节名称，顺序与 JointVector 一致
    std::string base_frame{ "base_link" };              ///< 模型底座坐标系名称
    std::string tool_frame{ "tool0" };                  ///< 模型末端工具坐标系名称
    std::array<double, 3> gravity{ 0.0, 0.0, -9.81 };   ///< 重力加速度向量，单位 m/s²
    JointVector gravity_scale;                          ///< 重力补偿缩放系数，顺序与 joint_names 一致
};

/**
 * @brief SerialArm 机器人完整静态配置
 */
struct RobotCfg {
    std::vector<std::string> joint_names;  ///< 固定的 Joint 顺序
    RuntimeCfg runtime;                    ///< Robot 运行参数
    ShutdownCfg shutdown;                  ///< 正常停机参数
    JointCtrllerCfg ctrller;               ///< Joint 控制器参数
    JointActuatorMapCfg mapper;            ///< Joint/Actuator 映射
    SafetyCfg safety;                      ///< Joint/Actuator 安全配置
    DynamicsCfg dynamics;                  ///< 动力学参数
};

/**
 * @brief 配置加载错误码
 */
enum class ConfigErr {
    FILE_OPEN_FAILED,     ///< 配置文件无法打开
    SYNTAX_ERROR,         ///< YAML 语法错误
    MISSING_FIELD,        ///< 缺少必需字段
    INVALID_VALUE,        ///< 字段值或模块配置无效
    INVALID_SIZE,         ///< 数组长度不一致
    DUPLICATE_NAME,       ///< Joint/Actuator 名称重复
};

/**
 * @brief 配置加载错误信息
 */
struct ConfigErrInfo {
    ConfigErr code{ ConfigErr::INVALID_VALUE }; ///< 错误码
    std::string message;                        ///< 消息
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief 使用 yaml-cpp 加载完整机器人配置
 * @param path YAML 文件路径
 */
tl::expected<RobotCfg, ConfigErrInfo> load_robot_cfg(const std::string& path, const HardwareCapabilities& capabilities);

/**
 * @brief 只读比较两个配置解析后的最终配置差异
 */
tl::expected<std::vector<std::string>, ConfigErrInfo> compare_robot_cfg(const std::string& lhs_path, const std::string& rhs_path);
tl::expected<std::vector<std::string>, ConfigErrInfo> compare_robot_cfg(const std::string& lhs_path, const std::string& rhs_path, const HardwareCapabilities& capabilities);

/**
 * @brief 验证 Robot 控制闭环所需的通用配置
 */
tl::expected<void, ConfigErrInfo> validate_robot_core_cfg(const RobotCfg& cfg);

/**
 * @brief 验证完整配置
 */
tl::expected<void, ConfigErrInfo> validate_robot_cfg(const RobotCfg& cfg);

// ! ========================= 模 版 方 法 实 现 ========================= ! //

} // namespace serial_arm
