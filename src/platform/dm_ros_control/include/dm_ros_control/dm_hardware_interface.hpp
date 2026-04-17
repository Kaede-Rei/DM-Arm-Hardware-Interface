#pragma once

#include <hardware_interface/system_interface.hpp>

#include "dm_ros_control/pinocchio_dynamics_model.hpp"
#include "dm_hw/damiao.hpp"

namespace dm_ros_control {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief 控制模式枚举
 * @param MIT MIT 模式
 * @param POS_VEL 位置+速度模式
 */
enum class ControlMode {
    MIT,
    POS_VEL
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief DmHardwareInterface 类，继承自 hardware_interface::SystemInterface，用于实现与达妙电机的通信和控制
 */
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
    std::vector<double> _hw_commands_pos_;
    std::vector<double> _hw_commands_pos_prev_;
    std::vector<double> _hw_commands_vel_;

    bool _enable_dynamics_{ false };
    bool _enable_gravity_feedforward_{ true };
    bool _enable_nonlinear_feedforward_{ true };

    std::string _urdf_path_;
    std::shared_ptr<PinocchioDynamicsModel> _dynamics_model_;
    std::vector<double> _gravity_feedforward_;
    std::vector<double> _nonlinear_feedforward_;
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //



}
