#pragma once

#include "serial_arm/config/config.hpp"
#include "serial_arm/dynamics/dynamics.hpp"
#include "serial_arm/robot.hpp"
#include "serial_arm_ros2_control/status_types.hpp"

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace serial_arm_ros2_control {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief ros2_control 达妙真机 SystemInterface
 */
class SerialArmSystem final : public hardware_interface::SystemInterface {
public:
    SerialArmSystem() = default;
    /**
     * @brief 停止后台线程并尽力释放真机硬件
     */
    ~SerialArmSystem() override;

    /**
     * @brief 初始化 ros2_control 硬件信息并校验 Core 配置一致性
     * @param info ros2_control 解析得到的硬件信息
     * @return 初始化成功返回 SUCCESS，否则返回 ERROR
     */
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
    /**
     * @brief 配置动力学、达妙总线与 Robot，但不连接或使能真机
     * @param previous_state 生命周期切换前状态
     * @return 配置成功返回 SUCCESS，否则返回 ERROR
     */
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    /**
     * @brief 显式授权后连接真机、初始化命令缓存并启动控制线程
     * @param previous_state 生命周期切换前状态
     * @return 激活成功返回 SUCCESS，否则返回 ERROR
     */
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    /**
     * @brief 停止控制线程并请求刚性保持后失能真机
     * @param previous_state 生命周期切换前状态
     * @return 失能成功返回 SUCCESS，否则返回 ERROR
     */
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    /**
     * @brief 导出每个关节的 position、velocity、effort 状态接口
     * @return ros2_control 状态接口列表
     */
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    /**
     * @brief 导出每个关节的 position、velocity 命令接口
     * @return ros2_control 命令接口列表
     */
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    /**
     * @brief 将后台线程最近合法状态复制到 ros2_control 状态接口
     * @param time controller_manager 当前时间
     * @param period controller_manager 周期
     * @return 读取缓存成功返回 OK
     */
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    /**
     * @brief 将 ros2_control 命令接口复制到后台线程命令缓存
     * @param time controller_manager 当前时间
     * @param period controller_manager 周期
     * @return 写入缓存成功返回 OK，后台错误锁存后返回 ERROR
     */
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    /**
     * @brief 校验 ros2_control joint 名称和接口与 Core YAML 一致
     * @return 校验成功返回 SUCCESS，否则返回 ERROR
     */
    hardware_interface::CallbackReturn validate_hardware_info();
    /**
     * @brief 构造 Dynamics、DamiaoMotorBus 和 Robot 并预分配缓存
     * @return 配置成功返回 SUCCESS，否则返回 ERROR
     */
    hardware_interface::CallbackReturn configure_robot();
    /**
     * @brief 构造 Robot 使用的动力学模型前馈回调
     * @return 模型前馈回调函数
     */
    serial_arm::ModelFeedforwardFn make_model_feedforward();
    /**
     * @brief 固定频率执行 Robot::set_cmd()、Robot::cycle() 和状态缓存更新
     */
    void worker_loop() noexcept;
    /**
     * @brief 请求后台控制线程退出并等待结束
     */
    void stop_worker();
    /**
     * @brief 按 shutdown 配置回到停放姿态
     * @return 停放姿态满足判据返回 true，否则返回 false
     */
    bool park_before_deactivate();
    /**
     * @brief 清除外部命令并切换为最近状态位置保持
     */
    void clear_command();
    /**
     * @brief 锁存后台线程错误并写入 ROS 日志
     * @param message 错误文本
     */
    void set_error(const std::string& message) noexcept;
    /**
     * @brief 将 RobotFault 转换为日志文本
     * @param action 当前失败操作名称
     * @param fault Robot 故障详情
     * @return 包含顶层错误和子错误的文本
     */
    std::string make_robot_error(const char* action, const serial_arm::RobotFault& fault) const;

private:
    serial_arm::RobotCfg cfg_;                         ///< Core 完整配置
    std::unique_ptr<serial_arm::Robot> robot_;         ///< 后台线程独占的 Robot 实例
    std::unique_ptr<serial_arm::Dynamics> dynamics_;   ///< 后台线程独占的动力学实例

    std::string config_file_;              ///< Core YAML 配置文件路径
    bool configured_{ false };             ///< 是否已完成 configure 阶段

    std::vector<double> hw_position_;      ///< ros2_control 位置状态接口缓存
    std::vector<double> hw_velocity_;      ///< ros2_control 速度状态接口缓存
    std::vector<double> hw_effort_;        ///< ros2_control 力矩状态接口缓存
    std::vector<double> cmd_position_;     ///< ros2_control 位置命令接口缓存
    std::vector<double> cmd_velocity_;     ///< ros2_control 速度命令接口缓存

    CommandFrame command_frame_;           ///< 后台线程消费的最新命令帧
    StateFrame state_frame_;               ///< read() 消费的最近合法状态帧

    mutable std::mutex command_mutex_;     ///< 命令帧互斥锁
    mutable std::mutex state_mutex_;       ///< 状态帧互斥锁
    std::thread worker_;                   ///< 真机控制后台线程
    std::atomic_bool worker_running_{ false };  ///< 后台线程运行标志
    std::atomic_bool worker_error_{ false };    ///< 后台线程错误锁存标志
    std::string last_error_;               ///< 最近一次后台线程错误文本
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //

} // namespace serial_arm_ros2_control
