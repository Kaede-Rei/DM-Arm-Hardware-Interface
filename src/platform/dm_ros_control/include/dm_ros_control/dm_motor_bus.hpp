#pragma once

#include "dm_ros_control/dm_types.hpp"

#include <termios.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

class SerialPort;

namespace damiao {
class Motor;
class MotorControl;
}

namespace dm_ros_control {

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief DmMotorBus 封装达妙电机串口/CAN 通信和关节-电机单位换算
 */
class DmMotorBus {
public:
    void configure(const std::string& serial_port, speed_t baudrate, const std::vector<DmMotorConfig>& configs);
    void activate(int startup_read_cycles, std::vector<DmJointState>& states);
    void deactivate();
    void cleanup();

    bool read(bool refresh_state, std::vector<DmJointState>& states) noexcept;
    bool write(std::size_t index, const DmJointCommand& command) noexcept;

    std::size_t size() const { return _configs_.size(); }

private:
    bool to_dm_control_mode(ControlMode mode, damiao::DmControlMode& dm_mode) const noexcept;
    bool read_one(std::size_t index, bool refresh_state, DmJointState& state) noexcept;

private:
    std::shared_ptr<SerialPort> _serial_;
    std::shared_ptr<damiao::MotorControl> _motor_controller_;
    std::vector<std::shared_ptr<damiao::Motor>> _motors_;
    std::vector<DmMotorConfig> _configs_;
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //



}
