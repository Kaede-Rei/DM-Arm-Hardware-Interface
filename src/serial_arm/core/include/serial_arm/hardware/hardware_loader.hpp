#pragma once

#include <tl/expected.hpp>

#include "serial_arm/hardware/motor_bus.hpp"

#include <memory>
#include <string>

namespace serial_arm {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

enum class HardwareLoaderErr {
    OPEN_FAILED,       ///< 无法打开 Hardware Backend 共享库
    SYMBOL_FAILED,     ///< 共享库缺少 create_motor_bus 或 destroy_motor_bus
    CREATE_FAILED,     ///< create_motor_bus 未能创建 MotorBus 实例
    CONFIGURE_FAILED,  ///< MotorBus::configure() 失败
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

class HardwareLoader {
public:
    HardwareLoader() = default;
    ~HardwareLoader();

    HardwareLoader(const HardwareLoader&) = delete;
    HardwareLoader& operator=(const HardwareLoader&) = delete;
    HardwareLoader(HardwareLoader&& other) noexcept;
    HardwareLoader& operator=(HardwareLoader&& other) noexcept;

    /**
     * @brief 加载 Hardware Backend 并使用指定配置完成 configure()
     * @param plugin 共享库路径，或不含路径的插件名
     * @param config_path Backend 专属 YAML 配置路径
     * @return 持有 Backend 对象和共享库句柄生命周期的 MotorBus
     *
     * 当 plugin 不含路径分隔符时，Loader 会额外尝试 lib<plugin>.so
     * 返回的 MotorBus 析构时会先调用插件 destroy_motor_bus()，再 dlclose()
     */
    tl::expected<std::unique_ptr<MotorBus>, HardwareLoaderErr> load(const std::string& plugin, const std::string& config_path);

private:
    using CreateFn = MotorBus * (*)();
    using DestroyFn = void (*)(MotorBus*);
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //

} // namespace serial_arm
