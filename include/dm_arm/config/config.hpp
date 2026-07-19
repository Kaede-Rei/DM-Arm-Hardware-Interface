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

constexpr std::size_t DM_ARM_JOINTS_COUNT = 6;  ///< 关节数量

/**
 * @brief 运行时配置
 */
struct RuntimeCfg {
    double ctrl_frequency_hz{ 200.0 };      ///< 控制频率
    double cmd_timeout_s{ 0.1 };            ///< 命令超时时间
    double state_timeout_s{ 0.05 };         ///< 状态超时时间
    std::size_t startup_read_cycles{ 5 };   ///< 启动时读取周期数
    bool write_enabled{ false };            ///< 写入是否启用
    bool refresh_state_in_read{ false };    ///< 在读取时刷新状态
};

/**
 * @brief 关节限制配置
 */
struct JointLimitCfg {
    JointVector min_pos;       ///< 关节最小位置限制
    JointVector max_pos;       ///< 关节最大位置限制
    JointVector max_vel;       ///< 关节最大速度限制
    JointVector max_acc;       ///< 关节最大加速度限制
    JointVector max_effort;    ///< 关节最大力矩限制
    JointVector max_kp;        ///< 关节最大位置增益限制
    JointVector max_kd;        ///< 关节最大速度增益限制
};

/**
 * @brief 达妙电机配置
 */
struct DamiaoActuatorCfg {
    std::string name;              ///< 执行器名称
    std::string joint_name;        ///< 关联关节名称
    std::uint32_t motor_id{ 0 };   ///< 电机 ID
    std::uint32_t master_id{ 0 };  ///< 主站 ID
    std::string motor_type;        ///< 电机型号
};

/**
 * @brief 达妙总线配置
 */
struct DamiaoBusCfg {
    std::string serial_port{ "/dev/ttyACM0" };       ///< 串口设备路径
    int baudrate{ 921600 };                          ///< 串口波特率
    bool refresh_state_in_read{ false };             ///< 在读取时刷新状态
    std::size_t startup_read_cycles{ 5 };            ///< 启动时读取周期数
    double stop_kp{ 3.0 };                           ///< 停止命令位置增益
    double stop_kd{ 0.1 };                           ///< 停止命令速度增益
    std::size_t stop_cycles{ 5 };                    ///< 停止命令发送周期数
    std::vector<DamiaoActuatorCfg> actuators;        ///< 达妙执行器列表
};

/**
 * @brief 机器人完整配置
 */
struct RobotCfg {
    std::vector<std::string> joint_names;    ///< 关节名称列表
    RuntimeCfg runtime;                      ///< 运行时配置
    JointCtrllerCfg ctrller;                 ///< 关节控制器配置
    JointActuatorMapCfg mapper;              ///< 关节执行器映射配置
    JointLimitCfg limits;                    ///< 关节限制配置
    DamiaoBusCfg damiao;                     ///< 达妙总线配置
};

/**
 * @brief 配置错误码
 */
enum class ConfigErrc {
    FILE_OPEN_FAILED,           ///< 配置文件打开失败
    SYNTAX_ERROR,               ///< 配置语法错误
    MISSING_FIELD,              ///< 缺少必要字段
    INVALID_VALUE,              ///< 配置值非法
    INVALID_SIZE,               ///< 配置数组长度非法
    DUPLICATE_NAME,             ///< 名称重复
    DUPLICATE_MOTOR_ID,         ///< 电机 ID 重复
    INVALID_MOTOR_TYPE,         ///< 电机型号非法
    ACTUATOR_LIMIT_EXCEEDED,    ///< 映射后的执行器限制超出电机范围
};

/**
 * @brief 配置错误信息
 */
struct ConfigErr {
    ConfigErrc code{ ConfigErrc::INVALID_VALUE };    ///< 配置错误码
    std::string message;                             ///< 配置错误描述
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief 加载机器人配置
 * @param path 配置文件路径
 * @return tl::expected<RobotCfg, ConfigErr> 机器人配置
 */
tl::expected<RobotCfg, ConfigErr> load_robot_cfg(const std::string& path);

/**
 * @brief 验证机器人配置
 * @param cfg 机器人配置
 * @return tl::expected<void, ConfigErr>
 */
tl::expected<void, ConfigErr> validate_robot_cfg(const RobotCfg& cfg);

// ! ========================= 模 版 方 法 实 现 ========================= ! //

} // namespace dm_arm
