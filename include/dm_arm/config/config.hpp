#pragma once

#include <tl/expected.hpp>

#include "dm_arm/core/joint_actuator_mapper.hpp"
#include "dm_arm/core/joints_ctrller.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace dm_arm {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

constexpr std::size_t DM_ARM_JOINTS_COUNT = 6;  ///< DM-Arm 主链关节数量

/**
 * @brief Robot 运行时配置
 */
struct RuntimeCfg {
    double ctrl_frequency_hz{ 200.0 };  ///< 控制循环频率
    double cmd_timeout_s{ 0.1 };        ///< 上层命令超时
    double state_timeout_s{ 0.05 };     ///< 硬件状态超时
    bool write_enabled{ false };        ///< 是否允许下发执行器命令
};

/**
 * @brief Joint 侧软件限制
 */
struct JointLimitCfg {
    JointVector min_pos;       ///< 最小位置，rad
    JointVector max_pos;       ///< 最大位置，rad
    JointVector max_vel;       ///< 最大速度，rad/s
    JointVector max_acc;       ///< 最大加速度，rad/s^2
    JointVector max_effort;    ///< 最大关节力矩，N·m
    JointVector max_kp;        ///< 最大 Joint 侧 kp
    JointVector max_kd;        ///< 最大 Joint 侧 kd
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
    std::size_t startup_read_cycles{ 5 };       ///< 激活后用于确认状态的读取次数
    double stop_kp{ 3.0 };                      ///< 停止保持的执行器侧 kp
    double stop_kd{ 0.1 };                      ///< 停止保持的执行器侧 kd
    std::size_t stop_cycles{ 5 };               ///< 停止保持命令发送次数
    std::vector<DamiaoActuatorCfg> actuators;   ///< 执行器列表
};

/**
 * @brief 当前 DM-Arm 的完整静态配置
 */
struct RobotCfg {
    std::vector<std::string> joint_names;  ///< 固定的 Joint 顺序
    RuntimeCfg runtime;                    ///< Robot 运行参数
    JointCtrllerCfg ctrller;               ///< Joint 控制器参数
    JointActuatorMapCfg mapper;             ///< Joint/Actuator 映射
    JointLimitCfg limits;                  ///< Joint 软件限制
    DamiaoBusCfg damiao;                   ///< 达妙后端参数
};

/**
 * @brief 配置加载错误码
 */
enum class ConfigErrc {
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
struct ConfigErr {
    ConfigErrc code{ ConfigErrc::INVALID_VALUE };
    std::string message;
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief 使用 yaml-cpp 加载完整机器人配置
 * @param path YAML 文件路径
 */
tl::expected<RobotCfg, ConfigErr> load_robot_cfg(const std::string& path);

/**
 * @brief 验证与具体 SDK 型号表无关的跨模块配置关系
 */
tl::expected<void, ConfigErr> validate_robot_cfg(const RobotCfg& cfg);

// ! ========================= 模 版 方 法 实 现 ========================= ! //

} // namespace dm_arm
