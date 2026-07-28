#pragma once

#include <tl/expected.hpp>

#include "dm_arm/core/joint_actuator_mapper.hpp"
#include "dm_arm/core/joints_ctrller.hpp"
#include "dm_arm/core/safety.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace dm_arm {

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
    JointImpedanceMode ros2_control_impedance_mode{    ///< ros2_control 跟踪阻抗模式
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
 * @brief 单个达妙执行器的文本配置
 */
struct DamiaoActuatorCfg {
    std::string name;              ///< 执行器名称
    std::string joint_name;        ///< 关联关节名称
    std::uint32_t motor_id{ 0 };   ///< 电机 ID
    std::uint32_t master_id{ 0 };  ///< 主站 ID
    std::string motor_type;        ///< 达妙 SDK 电机型号名称
};

/**
 * @brief 达妙总线配置
 */
struct DamiaoBusCfg {
    std::string serial_port{ "/dev/ttyACM0" };  ///< 串口设备
    int baudrate{ 921600 };                     ///< 波特率
    bool refresh_state_in_read{ false };        ///< read() 是否主动逐轴查询
    double feedback_timeout_s{ 0.05 };          ///< 单个执行器反馈超时时间
    std::size_t activation_retries{ 3 };        ///< 单轴使能与模式切换重试次数
    std::size_t startup_read_cycles{ 5 };       ///< 激活后用于确认状态的读取次数
    double stop_kp{ 3.0 };                      ///< 停止保持的执行器侧 kp
    double stop_kd{ 0.1 };                      ///< 停止保持的执行器侧 kd
    std::size_t stop_cycles{ 5 };               ///< 停止保持命令发送次数
    std::vector<DamiaoActuatorCfg> actuators;   ///< 执行器列表
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
 * @brief 当前 DM-Arm 的完整静态配置
 */
struct RobotCfg {
    std::vector<std::string> joint_names;  ///< 固定的 Joint 顺序
    RuntimeCfg runtime;                    ///< Robot 运行参数
    ShutdownCfg shutdown;                  ///< 正常停机参数
    JointCtrllerCfg ctrller;               ///< Joint 控制器参数
    JointActuatorMapCfg mapper;            ///< Joint/Actuator 映射
    SafetyCfg safety;                      ///< Joint/Actuator 安全配置
    DamiaoBusCfg damiao;                   ///< 达妙后端参数
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
    DUPLICATE_MOTOR_ID,   ///< 电机 ID 重复
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
tl::expected<RobotCfg, ConfigErrInfo> load_robot_cfg(const std::string& path);

/**
 * @brief 只读比较两个配置解析后的最终配置差异
 */
tl::expected<std::vector<std::string>, ConfigErrInfo> compare_robot_cfg(const std::string& lhs_path, const std::string& rhs_path);

/**
 * @brief 验证 Robot 控制闭环所需的通用配置
 */
tl::expected<void, ConfigErrInfo> validate_robot_core_cfg(const RobotCfg& cfg);

/**
 * @brief 验证完整配置，包括当前 YAML 中的 Damiao 后端字段
 */
tl::expected<void, ConfigErrInfo> validate_robot_cfg(const RobotCfg& cfg);

// ! ========================= 模 版 方 法 实 现 ========================= ! //

} // namespace dm_arm
