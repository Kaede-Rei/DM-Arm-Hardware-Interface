#pragma once

#include <hardware_interface/system_interface.hpp>

#include "dm_ros_control/dm_motor_bus.hpp"
#include "dm_ros_control/dynamics_observer.hpp"

#include <termios.h>

#include <cstddef>
#include <string>
#include <vector>

namespace dm_ros_control {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

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
    bool load_pd_gains_from_yaml();

    DmJointCommand build_joint_command(std::size_t index) const;
    double select_legacy_feedforward(std::size_t index) const;
    double sanitize_or_default(double value, double default_value) const;

private:
    std::string _serial_port_{ "/dev/ttyACM0" };
    int _baudrate_{ 921600 };
    bool _enable_write_{ true };
    bool _refresh_state_in_read_{ true };
    int _startup_read_cycles_{ 5 };

    // 过渡期使用开关，决定是否继续使用 JTC + hardware_interface 补力矩
    bool _legacy_feedforward_enabled_{ true };
    // 过渡期使用开关，决定是否在 YAML 中加载 PD 增益配置
    bool _legacy_pd_fallback_{ true };

    DmMotorBus _motor_bus_;
    DynamicsObserver _dynamics_observer_;

    std::vector<std::string> _joint_names_;
    std::vector<DmMotorConfig> _motor_configs_;

    // 过渡期使用的 PD 增益，从 YAML 加载，如果加载失败则使用默认值
    std::vector<double> _joint_kp_;
    std::vector<double> _joint_kd_;

    std::vector<double> _hw_positions_;
    std::vector<double> _hw_velocities_;
    std::vector<double> _hw_efforts_;
    std::vector<double> _motor_efforts_;

    std::vector<double> _hw_commands_pos_;
    std::vector<double> _hw_commands_vel_;
    std::vector<double> _hw_commands_effort_;
    std::vector<double> _hw_commands_kp_;
    std::vector<double> _hw_commands_kd_;

    std::vector<DmJointState> _bus_states_;

    bool _enable_dynamics_{ true };
    bool _enable_gravity_feedforward_{ true };
    bool _enable_nonlinear_feedforward_{ false };

    std::string _urdf_path_;

    std::vector<double> _gravity_feedforward_;
    std::vector<double> _nonlinear_feedforward_;
    std::vector<double> _active_feedforward_;
    std::vector<double> _external_efforts_;
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //



}
