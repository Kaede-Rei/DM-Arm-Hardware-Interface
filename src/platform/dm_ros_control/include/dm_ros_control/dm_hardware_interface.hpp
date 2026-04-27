#pragma once

#include <hardware_interface/system_interface.hpp>

#include "dm_control_core/dm_motor_bus.hpp"
#include "dm_control_core/dynamics_observer.hpp"
#include "dm_control_core/joint_impedance_controller.hpp"

namespace dm_ros_control {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief DmHardwareInterface 类，继承自 hardware_interface::SystemInterface，用于实现与达妙电机的通信和控制
 */
class DmHardwareInterface : public hardware_interface::SystemInterface {
public:
    using CallbackReturn = hardware_interface::CallbackReturn;

    /**
     * @brief 初始化硬件接口，解析 ros2_control 参数并分配状态/命令缓冲
     * @param info ros2_control 传入的硬件信息
     * @return 初始化成功返回 SUCCESS，否则返回 ERROR
     */
    CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

    /**
     * @brief 配置硬件资源，建立电机 bus 和可选动力学观测器
     * @param previous_state 上一个生命周期状态
     * @return 配置成功返回 SUCCESS，否则返回 ERROR
     */
    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

    /**
     * @brief 激活硬件，完成电机使能、控制模式切换和初始状态读取
     * @param previous_state 上一个生命周期状态
     * @return 激活成功返回 SUCCESS，否则返回 ERROR
     */
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    /**
     * @brief 停用硬件，执行电机停机和失能流程
     * @param previous_state 上一个生命周期状态
     * @return 停用成功返回 SUCCESS，否则返回 ERROR
     */
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    /**
     * @brief 清理硬件资源，释放 bus 和动力学观测器
     * @param previous_state 上一个生命周期状态
     * @return 清理成功返回 SUCCESS
     */
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

    /**
     * @brief 导出 ros2_control 状态接口
     * @return 状态接口列表
     */
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    /**
     * @brief 导出 ros2_control 命令接口
     * @return 命令接口列表
     */
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    /**
     * @brief 控制周期读取入口，读取电机状态并更新动力学观测
     * @param time 当前 ROS 时间
     * @param period 控制周期
     * @return 读取成功返回 OK，否则返回 ERROR
     */
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    /**
     * @brief 控制周期写入入口，汇总控制输入并发送到电机 bus
     * @param time 当前 ROS 时间
     * @param period 控制周期
     * @return 写入成功返回 OK，否则返回 ERROR
     */
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    /**
     * @brief 将整数波特率转换为 termios speed_t
     * @param baudrate 整数波特率
     * @return termios speed_t
     */
    speed_t baudrate_to_speed_t(int baudrate) const;

    /**
     * @brief 从安装后的 pd_config.yaml 加载过渡期 PD 增益
     * @return 成功返回 true，失败返回 false
     */
    bool load_pd_gains_from_yaml();

    /**
     * @brief 构造纯 C++ 关节阻抗控制器配置
     * @return 关节阻抗控制器配置
     */
    dm_control_core::JointImpedanceControllerConfig build_joint_impedance_config() const;

private:
    std::string serial_port_{ "/dev/ttyACM0" };
    int baudrate_{ 921600 };
    bool enable_write_{ true };
    bool refresh_state_in_read_{ true };
    int startup_read_cycles_{ 5 };

    // 过渡期使用开关，决定是否继续使用 JTC + hardware_interface 补力矩
    bool legacy_feedforward_enabled_{ true };
    // 过渡期使用开关，决定是否在 YAML 中加载 PD 增益配置
    bool legacy_pd_fallback_{ true };

    dm_control_core::DmMotorBus motor_bus_;
    dm_control_core::DynamicsObserver dynamics_observer_;
    dm_control_core::JointImpedanceController joint_impedance_controller_;

    std::vector<std::string> joint_names_;
    std::vector<dm_control_core::DmMotorConfig> motor_configs_;

    // 过渡期使用的 PD 增益，从 YAML 加载，如果加载失败则使用默认值
    std::vector<double> joint_kp_;
    std::vector<double> joint_kd_;

    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
    std::vector<double> hw_efforts_;
    std::vector<double> motor_efforts_;

    std::vector<double> hw_commands_pos_;
    std::vector<double> hw_commands_vel_;
    std::vector<double> hw_commands_effort_;
    std::vector<double> hw_commands_kp_;
    std::vector<double> hw_commands_kd_;

    dm_control_core::JointState bus_state_;

    bool enable_dynamics_{ true };
    bool enable_gravity_feedforward_{ true };
    bool enable_nonlinear_feedforward_{ false };

    std::string urdf_path_;

    std::vector<double> gravity_feedforward_;
    std::vector<double> nonlinear_feedforward_;
    std::vector<double> active_feedforward_;
    std::vector<double> external_efforts_;
    dm_control_core::DynamicsObservation dynamics_observation_;
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //



}
