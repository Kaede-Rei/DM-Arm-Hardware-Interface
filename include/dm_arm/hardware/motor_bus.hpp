#pragma once

#include <tl/expected.hpp>

#include "dm_arm/core/types.hpp"

namespace dm_arm {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

enum class MotorBusErr {
    NOT_CONFIGURED,
    NOT_CONNECTED,
    NOT_ACTIVE,
    OPEN_FAILED,
    READ_FAILED,
    WRITE_FAILED,
    INVALID_STATE,
    INVALID_CMD,
    ACTUATOR_OFFLINE,
    ACTUATOR_FAULT,
    TIMEOUT,
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

class MotorBus {
public:
    virtual ~MotorBus() = default;

    virtual tl::expected<void, MotorBusErr> connect() = 0;
    virtual tl::expected<ActuatorState, MotorBusErr> read() = 0;
    virtual tl::expected<void, MotorBusErr> activate() = 0;
    virtual tl::expected<void, MotorBusErr> write(const ActuatorMitCmd& cmd) = 0;
    virtual tl::expected<void, MotorBusErr> stop() = 0;
    virtual tl::expected<void, MotorBusErr> deactivate() = 0;

    virtual void cleanup() noexcept = 0;
    virtual std::size_t size() const noexcept = 0;
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //

} // namespace dm_arm
