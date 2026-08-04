#pragma once

#include <tl/expected.hpp>

#include "serial_arm/core/types.hpp"
#include "serial_arm/hardware/hardware_capability.hpp"

#include <string>

namespace serial_arm {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

enum class MotorBusErr {
    NOT_CONFIGURED,    ///< Backend 尚未完成 configure()
    NOT_CONNECTED,     ///< Backend 尚未连接到底层总线或设备
    NOT_ACTIVE,        ///< Backend 尚未进入可读写激活状态
    INVALID_CFG,       ///< 硬件配置文件无效
    OPEN_FAILED,       ///< 打开底层设备失败
    READ_FAILED,       ///< 读取执行器状态失败
    WRITE_FAILED,      ///< 写入执行器命令失败
    INVALID_STATE,     ///< Backend 当前生命周期状态不允许该操作
    INVALID_CMD,       ///< ActuatorCtrlCmd 内容或维度无效
    ACTUATOR_OFFLINE,  ///< 至少一个执行器离线
    ACTUATOR_FAULT,    ///< 至少一个执行器处于故障状态
    TIMEOUT,           ///< 总线通信超时

    ENABLE_FAILED,       ///< 执行器使能失败
    MODE_SWITCH_FAILED,  ///< 执行器控制模式切换失败
    STOP_FAILED,         ///< 停止或刹停失败
    DISABLE_FAILED,      ///< 执行器失能失败
    RECOVER_FAILED,      ///< 故障恢复失败
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

class MotorBus {
public:
    /**
     * @brief 执行器硬件后端抽象
     *
     * MotorBus 只暴露 SerialArm Hardware Contract；具体厂商协议、单位、
     * 电流到力矩转换和底层控制由 Backend 自行适配
     */
    virtual ~MotorBus() = default;

    /**
     * @brief 读取并校验具体 Hardware Backend 的配置文件
     * @param config_path Backend 专属 YAML 配置路径
     *
     * configure() 不应向真实执行器发送使能或运动命令；需要建立低层资源时，
     * 应保持在 connect()/activate() 前不会驱动硬件运动
     */
    virtual tl::expected<void, MotorBusErr> configure(const std::string& config_path) = 0;

    /**
     * @brief 打开总线或设备连接
     *
     * 成功后 Backend 可以读取设备状态，但不代表执行器已经使能
     */
    virtual tl::expected<void, MotorBusErr> connect() = 0;

    /**
     * @brief 读取执行器侧状态
     *
     * 返回的 pos / vel / tor 必须符合 SerialArm Hardware Contract：
     * rad、rad/s、N·m
     */
    virtual tl::expected<ActuatorState, MotorBusErr> read() = 0;

    /**
     * @brief 使 Backend 进入可写控制状态
     *
     * Backend 可在此阶段使能执行器并切换到 Backend contract 要求的控制模式
     */
    virtual tl::expected<void, MotorBusErr> activate() = 0;

    /**
     * @brief 写入执行器侧控制命令
     * @param cmd Core 输出的完整 pos / vel / tor / kp / kd 执行器侧命令
     */
    virtual tl::expected<void, MotorBusErr> write(const ActuatorCtrlCmd& cmd) = 0;

    /**
     * @brief 请求 Backend 停止当前运动或刷新安全保持命令
     */
    virtual tl::expected<void, MotorBusErr> stop() = 0;

    /**
     * @brief 从激活状态退出并失能执行器
     */
    virtual tl::expected<void, MotorBusErr> deactivate() = 0;

    /**
     * @brief 尝试恢复 Backend 或执行器故障
     */
    virtual tl::expected<void, MotorBusErr> recover() = 0;

    /**
     * @brief 返回 Core Safety 所需的执行器物理范围
     */
    virtual const HardwareCapabilities& capabilities() const noexcept = 0;

    /**
     * @brief 释放 Backend 持有的底层资源
     *
     * cleanup() 必须 noexcept，并允许被重复调用
     */
    virtual void cleanup() noexcept = 0;

    /**
     * @brief 返回 Backend 管理的执行器数量
     */
    virtual std::size_t size() const noexcept = 0;
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //

} // namespace serial_arm
