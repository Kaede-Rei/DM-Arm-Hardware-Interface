#ifndef _dm_hardware_interface_hpp_
#define _dm_hardware_interface_hpp_

#include <hardware_interface/system_interface.hpp>

#include "dm_hw/damiao.hpp"

namespace dm_ros_control {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

enum class ControlMode {
    MIT,
    POS_VEL
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

class DmHardwareInterface : public hardware_interface::SystemInterface {
public:
    using CallbackReturn = hardware_interface::CallbackReturn;

    CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    speed_t baudrate_to_speed_t(int baudrate) const;

    std::string _serial_port_{ "/dev/ttyACM0" };
    int _baudrate_{ 921600 };
    double _kp_{ 5.0 };
    double _kd_{ 0.2 };
    double _max_position_change_{ 0.05 };
    double _max_velocity_{ 1.0 };
    bool _enable_write_{ true };
    bool _refresh_state_in_read_{ true };
    int _startup_read_cycles_{ 5 };

    std::shared_ptr<SerialPort> _serial_;
    std::shared_ptr<damiao::MotorControl> _motor_controller_;
    std::vector<std::shared_ptr<damiao::Motor>> _motors_;

    std::vector<std::string> _joint_names_;
    std::vector<uint32_t> _motor_ids_;
    std::vector<damiao::DmMotorType> _motor_types_;
    std::vector<double> _joint_to_motor_scale_;
    std::vector<ControlMode> _control_modes_;

    std::vector<double> _hw_positions_;
    std::vector<double> _hw_velocities_;
    std::vector<double> _hw_commands_;
    std::vector<double> _hw_commands_prev_;
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //



}

#endif
