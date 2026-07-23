#pragma once

#include "dm_arm/config/config.hpp"
#include "dm_arm/hardware/motor_bus.hpp"
#include "dm_hw/damiao.hpp"

namespace dm_arm {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief 达妙执行器静态信息
 */
struct DamiaoActuatorInfo {
    std::string name;              ///< 执行器名称
    std::string joint_name;        ///< 关联关节名称
    std::uint32_t motor_id{ 0 };   ///< 电机 ID
    std::uint32_t master_id{ 0 };  ///< 主站 ID
    std::string motor_type;        ///< 电机型号名称
    double q_max{ 0.0 };           ///< 执行器最大位置绝对值
    double dq_max{ 0.0 };          ///< 执行器最大速度绝对值
    double tau_max{ 0.0 };         ///< 执行器最大力矩绝对值
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief DamiaoMotorBus 类，继承自 MotorBus，用于与 Damiao 电机总线进行通信
 */
class DamiaoMotorBus final : public MotorBus {
public:
    /**
     * @brief 析构函数
     */
    ~DamiaoMotorBus() override { cleanup(); }

    /**
     * @brief 配置 DamiaoMotorBus
     * @param cfg 配置参数
     * @return 如果配置成功，则返回空的 tl::expected，否则返回错误码
     */
    tl::expected<void, MotorBusErr> configure(const DamiaoBusCfg& cfg);

    /**
     * @brief 连接 DamiaoMotorBus
     * @return 如果连接成功，则返回空的 tl::expected，否则返回错误码
     */
    tl::expected<void, MotorBusErr> connect() override;
    /**
     * @brief 读取 DamiaoMotorBus 的状态
     * @return 如果读取成功，则返回 ActuatorState，否则返回错误码
     */
    tl::expected<ActuatorState, MotorBusErr> read() override;
    /**
     * @brief 激活 DamiaoMotorBus
     * @return 如果激活成功，则返回空的 tl::expected，否则返回错误码
     */
    tl::expected<void, MotorBusErr> activate() override;
    /**
     * @brief 写入 DamiaoMotorBus 的命令
     * @param cmd 待写入的命令
     * @return 如果写入成功，则返回空的 tl::expected，否则返回错误码
     */
    tl::expected<void, MotorBusErr> write(const ActuatorMitCmd& cmd) override;
    /**
     * @brief 停止 DamiaoMotorBus 的运动
     * @return 如果停止成功，则返回空的 tl::expected，否则返回错误码
     */
    tl::expected<void, MotorBusErr> stop() override;
    /**
     * @brief 停用 DamiaoMotorBus
     * @return 如果停用成功，则返回空的 tl::expected，否则返回错误码
     */
    tl::expected<void, MotorBusErr> deactivate() override;
    /**
     * @brief 清理旧串口/协议状态并恢复到已连接、未使能状态
     * @return 如果恢复成功，则返回空的 tl::expected，否则返回错误码
     */
    tl::expected<void, MotorBusErr> recover() override;
    /**
     * @brief 清理 DamiaoMotorBus 的资源
     */
    void cleanup() noexcept override;
    /**
     * @brief 获取 DamiaoMotorBus 的电机数量
     * @return 电机数量
     */
    std::size_t size() const noexcept override;
    /**
     * @brief 获取达妙执行器静态信息
     * @return 达妙执行器静态信息只读引用
     */
    const std::vector<DamiaoActuatorInfo>& get_actuator_info() const noexcept;

private:
    /**
     * @brief 验证 DamiaoMotorBus 的配置参数
     * @param cfg 配置参数
     * @return 如果配置参数有效，则返回空的 tl::expected，否则返回错误码
     */
    tl::expected<void, MotorBusErr> validate_cfg(const DamiaoBusCfg& cfg) const;
    /**
     * @brief 验证 DamiaoMotorBus 的命令参数
     * @param cmd 命令参数
     * @return 如果命令参数有效，则返回空的 tl::expected，否则返回错误码
     */
    tl::expected<void, MotorBusErr> validate_cmd(const ActuatorMitCmd& cmd) const;
    /**
     * @brief 解析 DamiaoMotorBus 的电机类型
     * @param value 电机类型字符串
     * @return 如果解析成功，则返回 damiao::DmMotorType，否则返回错误码
     */
    tl::expected<ActuatorState, MotorBusErr> read_impl(bool refresh);
    /**
     * @brief 解析 DamiaoMotorBus 的电机类型
     * @param value 电机类型字符串
     * @return 如果解析成功，则返回 damiao::DmMotorType，否则返回错误码
     */
    tl::expected<damiao::DmMotorType, MotorBusErr> parse_motor_type(const std::string& value) const;
    /**
     * @brief 失能已经使能的电机，忽略异常
     */
    void disable_enabled_noexcept() noexcept;
    /**
     * @brief 释放串口与电机对象，但可选择保留配置
     * @param keep_config 是否保留配置
     */
    void release_connection_noexcept(bool keep_config) noexcept;

private:
    DamiaoBusCfg cfg_;                      ///< DamiaoMotorBus 的配置参数
    std::shared_ptr<SerialPort> serial_;    ///< 串口对象

    std::shared_ptr<damiao::MotorControl> motor_ctrl_;      ///< Damiao 电机控制对象
    std::vector<std::shared_ptr<damiao::Motor>> motors_;    ///< Damiao 电机对象列表

    std::vector<std::uint8_t> online_;      ///< 电机在线状态列表
    std::vector<std::uint8_t> enabled_;     ///< 电机使能状态列表

    ActuatorState last_state_;              ///< 上一次读取的电机状态
    std::vector<DamiaoActuatorInfo> actuator_info_;  ///< 达妙执行器静态信息

    bool configured_{ false };      ///< 是否已配置 DamiaoMotorBus
    bool connected_{ false };       ///< 是否已连接 DamiaoMotorBus
    bool active_{ false };          ///< 是否已激活 DamiaoMotorBus
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //

} // namespace dm_arm
