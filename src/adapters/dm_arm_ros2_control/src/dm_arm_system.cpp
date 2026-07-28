#include "dm_arm_ros2_control/dm_arm_system.hpp"

#include "dm_arm/hardware/damiao_motor_bus.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <utility>

namespace dm_arm_ros2_control {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //

namespace {

constexpr const char* kLoggerName = "dm_arm_ros2_control";  ///< ROS 日志器名称

// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //

/**
 * @brief 检查接口列表是否包含指定名称
 * @param interfaces ros2_control 接口描述列表
 * @param name 目标接口名称
 * @return 包含目标接口时返回 true，否则返回 false
 */
bool has_interface(const std::vector<hardware_interface::InterfaceInfo>& interfaces, const std::string& name) {
    return std::any_of(interfaces.begin(), interfaces.end(), [&name](const auto& item) { return item.name == name; });
}

/**
 * @brief 离线模拟 MotorBus，用于 runtime.write_enabled=false 的 ros2_control 后端
 */
class MockMotorBus final : public dm_arm::MotorBus {
public:
    explicit MockMotorBus(std::size_t size) {
        state_.pos.assign(size, 0.0);
        state_.vel.assign(size, 0.0);
        state_.tor.assign(size, 0.0);
        state_.online.assign(size, 1);
        state_.enabled.assign(size, 1);
        state_.err_code.assign(size, 0);
    }

    tl::expected<void, dm_arm::MotorBusErr> connect() override {
        connected_ = true;
        return {};
    }
    tl::expected<dm_arm::ActuatorState, dm_arm::MotorBusErr> read() override {
        if(!connected_) return tl::make_unexpected(dm_arm::MotorBusErr::NOT_CONNECTED);
        return state_;
    }
    tl::expected<void, dm_arm::MotorBusErr> activate() override {
        if(!connected_) return tl::make_unexpected(dm_arm::MotorBusErr::NOT_CONNECTED);
        active_ = true;
        return {};
    }
    tl::expected<void, dm_arm::MotorBusErr> write(const dm_arm::ActuatorMitCmd& cmd) override {
        if(!active_) return tl::make_unexpected(dm_arm::MotorBusErr::NOT_ACTIVE);
        state_.pos = cmd.pos;
        state_.vel = cmd.vel;
        state_.tor = cmd.tor;
        return {};
    }
    tl::expected<void, dm_arm::MotorBusErr> stop() override { return {}; }
    tl::expected<void, dm_arm::MotorBusErr> deactivate() override {
        active_ = false;
        return {};
    }
    tl::expected<void, dm_arm::MotorBusErr> recover() override { return {}; }
    void cleanup() noexcept override {
        active_ = false;
        connected_ = false;
    }
    std::size_t size() const noexcept override { return state_.pos.size(); }

private:
    dm_arm::ActuatorState state_;
    bool connected_{ false };
    bool active_{ false };
};

} // namespace

// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

/**
 * @brief 停止后台线程并尽力释放真机硬件
 */
DmArmSystem::~DmArmSystem() {
    stop_worker();
    if(robot_ && robot_->get_state() == dm_arm::RobotState::ACTIVE) {
        static_cast<void>(robot_->force_deactivate());
    }
}

/**
 * @brief 初始化 ros2_control 硬件信息并校验 Core 配置一致性
 * @param info ros2_control 解析得到的硬件信息
 * @return 初始化成功返回 SUCCESS，否则返回 ERROR
 */
hardware_interface::CallbackReturn DmArmSystem::on_init(const hardware_interface::HardwareInfo& info) {
    if(SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    auto config_iter = info_.hardware_parameters.find("config_file");
    if(config_iter == info_.hardware_parameters.end() || config_iter->second.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "hardware parameter config_file is required");
        return hardware_interface::CallbackReturn::ERROR;
    }
    config_file_ = config_iter->second;

    const auto cfg_result = dm_arm::load_robot_cfg(config_file_);
    if(!cfg_result) {
        RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "load_robot_cfg failed: %s", cfg_result.error().message.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }
    cfg_ = cfg_result.value();

    return validate_hardware_info();
}

/**
 * @brief 配置动力学、达妙总线与 Robot，但不连接或使能真机
 * @param previous_state 生命周期切换前状态
 * @return 配置成功返回 SUCCESS，否则返回 ERROR
 */
hardware_interface::CallbackReturn DmArmSystem::on_configure(const rclcpp_lifecycle::State& previous_state) {
    static_cast<void>(previous_state);
    if(configured_) {
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    return configure_robot();
}

/**
 * @brief 显式授权后连接真机、初始化命令缓存并启动控制线程
 * @param previous_state 生命周期切换前状态
 * @return 激活成功返回 SUCCESS，否则返回 ERROR
 */
hardware_interface::CallbackReturn DmArmSystem::on_activate(const rclcpp_lifecycle::State& previous_state) {
    static_cast<void>(previous_state);
    if(!configured_ || !robot_) {
        RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "DmArmSystem is not configured");
        return hardware_interface::CallbackReturn::ERROR;
    }
    const auto activate_result = robot_->activate();
    if(!activate_result) {
        RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", make_robot_error("Robot activate", activate_result.error()).c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    const auto& joint_state = robot_->get_joint_state();
    {
        std::lock_guard<std::mutex> command_lock(command_mutex_);
        command_frame_.pos = joint_state.pos;
        std::fill(command_frame_.vel.begin(), command_frame_.vel.end(), 0.0);
        ++command_frame_.sequence;
        cmd_position_ = command_frame_.pos;
        cmd_velocity_ = command_frame_.vel;
    }
    {
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        state_frame_.pos = joint_state.pos;
        state_frame_.vel = joint_state.vel;
        state_frame_.effort = joint_state.tor;
        state_frame_.valid = true;
        hw_position_ = state_frame_.pos;
        hw_velocity_ = state_frame_.vel;
        hw_effort_ = state_frame_.effort;
    }

    const auto mode_result = robot_->set_impedance_mode(dm_arm::JointImpedanceMode::RIGID_TRACKING);
    if(!mode_result) {
        RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", make_robot_error("Robot set_impedance_mode", mode_result.error()).c_str());
        static_cast<void>(robot_->force_deactivate());
        return hardware_interface::CallbackReturn::ERROR;
    }

    worker_error_.store(false);
    worker_running_.store(true);
    worker_ = std::thread(&DmArmSystem::worker_loop, this);
    return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 停止控制线程并请求刚性保持后失能真机
 * @param previous_state 生命周期切换前状态
 * @return 失能成功返回 SUCCESS，否则返回 ERROR
 */
hardware_interface::CallbackReturn DmArmSystem::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
    static_cast<void>(previous_state);
    stop_worker();
    if(!robot_) {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    if(robot_->get_state() == dm_arm::RobotState::ACTIVE) {
        const auto hold_result = robot_->set_impedance_mode(dm_arm::JointImpedanceMode::RIGID_HOLD);
        if(!hold_result) {
            RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", make_robot_error("Robot set_impedance_mode", hold_result.error()).c_str());
        }
        const auto result = robot_->deactivate();
        if(!result) {
            RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", make_robot_error("Robot deactivate", result.error()).c_str());
            const auto force_result = robot_->force_deactivate();
            if(!force_result) {
                RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", make_robot_error("Robot force_deactivate", force_result.error()).c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 导出每个关节的 position、velocity、effort 状态接口
 * @return ros2_control 状态接口列表
 */
std::vector<hardware_interface::StateInterface> DmArmSystem::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> interfaces;
    interfaces.reserve(info_.joints.size() * 3);
    for(std::size_t i = 0; i < info_.joints.size(); ++i) {
        interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_[i]);
        interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_[i]);
        interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_effort_[i]);
    }
    return interfaces;
}

/**
 * @brief 导出每个关节的 position、velocity 命令接口
 * @return ros2_control 命令接口列表
 */
std::vector<hardware_interface::CommandInterface> DmArmSystem::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> interfaces;
    interfaces.reserve(info_.joints.size() * 2);
    for(std::size_t i = 0; i < info_.joints.size(); ++i) {
        interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &cmd_position_[i]);
        interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &cmd_velocity_[i]);
    }
    return interfaces;
}

/**
 * @brief 将后台线程最近合法状态复制到 ros2_control 状态接口
 * @param time controller_manager 当前时间
 * @param period controller_manager 周期
 * @return 读取缓存成功返回 OK
 */
hardware_interface::return_type DmArmSystem::read(const rclcpp::Time& time, const rclcpp::Duration& period) {
    static_cast<void>(time);
    static_cast<void>(period);
    std::lock_guard<std::mutex> lock(state_mutex_);
    if(state_frame_.valid) {
        hw_position_ = state_frame_.pos;
        hw_velocity_ = state_frame_.vel;
        hw_effort_ = state_frame_.effort;
    }
    return hardware_interface::return_type::OK;
}

/**
 * @brief 将 ros2_control 命令接口复制到后台线程命令缓存
 * @param time controller_manager 当前时间
 * @param period controller_manager 周期
 * @return 写入缓存成功返回 OK，后台错误锁存后返回 ERROR
 */
hardware_interface::return_type DmArmSystem::write(const rclcpp::Time& time, const rclcpp::Duration& period) {
    static_cast<void>(time);
    static_cast<void>(period);
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_frame_.pos = cmd_position_;
    command_frame_.vel = cmd_velocity_;
    ++command_frame_.sequence;
    return worker_error_.load() ? hardware_interface::return_type::ERROR : hardware_interface::return_type::OK;
}

// ! ========================= 私 有 类 方 法 实 现 ========================= !

/**
 * @brief 校验 ros2_control joint 名称和接口与 Core YAML 一致
 * @return 校验成功返回 SUCCESS，否则返回 ERROR
 */
hardware_interface::CallbackReturn DmArmSystem::validate_hardware_info() {
    if(info_.joints.size() != cfg_.joint_names.size()) {
        RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "joint count mismatch: ros2_control=%zu config=%zu", info_.joints.size(), cfg_.joint_names.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    for(std::size_t i = 0; i < info_.joints.size(); ++i) {
        const auto& joint = info_.joints[i];
        if(joint.name != cfg_.joint_names[i]) {
            RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "joint name mismatch at %zu: ros2_control=%s config=%s", i, joint.name.c_str(), cfg_.joint_names[i].c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if(joint.command_interfaces.size() != 2 || !has_interface(joint.command_interfaces, hardware_interface::HW_IF_POSITION) || !has_interface(joint.command_interfaces, hardware_interface::HW_IF_VELOCITY)) {
            RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "joint %s must have exactly position and velocity command interfaces", joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if(!has_interface(joint.state_interfaces, hardware_interface::HW_IF_POSITION) || !has_interface(joint.state_interfaces, hardware_interface::HW_IF_VELOCITY) || !has_interface(joint.state_interfaces, hardware_interface::HW_IF_EFFORT)) {
            RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "joint %s must have position, velocity and effort state interfaces", joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 构造 Dynamics、DamiaoMotorBus 和 Robot 并预分配缓存
 * @return 配置成功返回 SUCCESS，否则返回 ERROR
 */
hardware_interface::CallbackReturn DmArmSystem::configure_robot() {
    dynamics_ = std::make_unique<dm_arm::Dynamics>();
    const auto dynamics_result = dynamics_->configure(cfg_.dynamics);
    if(!dynamics_result) {
        RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "Dynamics configure failed; DynamicsErr=%d", static_cast<int>(dynamics_result.error()));
        return hardware_interface::CallbackReturn::ERROR;
    }

    dm_arm::RobotCfg robot_cfg = cfg_;
    std::unique_ptr<dm_arm::MotorBus> bus;
    if(cfg_.runtime.write_enabled) {
        auto damiao_bus = std::make_unique<dm_arm::DamiaoMotorBus>();
        const auto bus_result = damiao_bus->configure(cfg_.damiao);
        if(!bus_result) {
            RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "DamiaoMotorBus configure failed; MotorBusErr=%d", static_cast<int>(bus_result.error()));
            dynamics_.reset();
            return hardware_interface::CallbackReturn::ERROR;
        }
        bus = std::move(damiao_bus);
        RCLCPP_WARN(rclcpp::get_logger(kLoggerName), "DM-Arm ros2_control backend: damiao; runtime.write_enabled=true");
    }
    else {
        robot_cfg.runtime.write_enabled = true;
        bus = std::make_unique<MockMotorBus>(cfg_.joint_names.size());
        RCLCPP_INFO(rclcpp::get_logger(kLoggerName), "DM-Arm ros2_control backend: offline; runtime.write_enabled=false");
    }

    robot_ = std::make_unique<dm_arm::Robot>();
    const auto robot_result = robot_->configure(robot_cfg, std::move(bus), make_model_feedforward());
    if(!robot_result) {
        RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", make_robot_error("Robot configure", robot_result.error()).c_str());
        robot_.reset();
        dynamics_.reset();
        return hardware_interface::CallbackReturn::ERROR;
    }

    const std::size_t joint_count = cfg_.joint_names.size();
    hw_position_.assign(joint_count, 0.0);
    hw_velocity_.assign(joint_count, 0.0);
    hw_effort_.assign(joint_count, 0.0);
    cmd_position_.assign(joint_count, 0.0);
    cmd_velocity_.assign(joint_count, 0.0);
    command_frame_.pos.assign(joint_count, 0.0);
    command_frame_.vel.assign(joint_count, 0.0);
    state_frame_.pos.assign(joint_count, 0.0);
    state_frame_.vel.assign(joint_count, 0.0);
    state_frame_.effort.assign(joint_count, 0.0);
    state_frame_.model_feedforward.assign(joint_count, 0.0);
    state_frame_.valid = false;
    configured_ = true;
    return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 构造 Robot 使用的动力学模型前馈回调
 * @return 模型前馈回调函数
 */
dm_arm::ModelFeedforwardFn DmArmSystem::make_model_feedforward() {
    return [this](dm_arm::ModelFeedforwardMode mode, const dm_arm::JointState& state, const dm_arm::JointVector& acc, const dm_arm::JointVector& ref_acc, double) -> tl::expected<dm_arm::JointVector, dm_arm::ModelFeedforwardErr> {
        const auto result = dynamics_->update(state, acc, ref_acc);
        if(!result) {
            return tl::make_unexpected(dm_arm::ModelFeedforwardErr::COMPUTE_FAILED);
        }
        if(mode == dm_arm::ModelFeedforwardMode::GRAVITY) {
            return dynamics_->get_gravity_compensation();
        }
        if(mode == dm_arm::ModelFeedforwardMode::FULL_INVERSE_DYNAMICS) {
            return dynamics_->get_inverse_dynamics();
        }
        return dm_arm::JointVector(cfg_.joint_names.size(), 0.0);
        };
}

/**
 * @brief 固定频率执行 Robot::set_cmd()、Robot::cycle() 和状态缓存更新
 */
void DmArmSystem::worker_loop() noexcept {
    const double target_dt = 1.0 / cfg_.runtime.ctrl_frequency_hz;
    const auto period = std::chrono::duration_cast<dm_arm::Robot::Clock::duration>(std::chrono::duration<double>(target_dt));
    auto next_time = dm_arm::Robot::Clock::now();
    dm_arm::JointPosVelCmd cmd;
    cmd.pos.assign(cfg_.joint_names.size(), 0.0);
    cmd.vel.assign(cfg_.joint_names.size(), 0.0);

    while(worker_running_.load()) {
        const auto now = dm_arm::Robot::Clock::now();
        if(now > next_time) next_time = now;

        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            cmd.pos = command_frame_.pos;
            cmd.vel = command_frame_.vel;
        }

        if(robot_->get_state() == dm_arm::RobotState::FAULT) {
            clear_command();
            if(robot_->is_fault_holding()) {
                const auto hold_result = robot_->maintain_fault_hold();
                if(!hold_result) {
                    set_error(make_robot_error("Robot maintain_fault_hold", hold_result.error()));
                    worker_running_.store(false);
                    break;
                }
            }
        }
        else {
            const auto cmd_result = robot_->set_cmd(cmd, now);
            if(!cmd_result) {
                set_error(make_robot_error("Robot set_cmd", cmd_result.error()));
                worker_running_.store(false);
                break;
            }

            const auto cycle_result = robot_->cycle(now);
            if(!cycle_result) {
                if(robot_->get_state() == dm_arm::RobotState::FAULT && robot_->is_fault_holding()) {
                    clear_command();
                    const auto hold_result = robot_->maintain_fault_hold();
                    if(!hold_result) {
                        set_error(make_robot_error("Robot maintain_fault_hold", hold_result.error()));
                        worker_running_.store(false);
                        break;
                    }
                }
                else {
                    set_error(make_robot_error("Robot cycle", cycle_result.error()));
                    worker_running_.store(false);
                    break;
                }
            }
            else {
                const auto& output = cycle_result.value();
                std::lock_guard<std::mutex> lock(state_mutex_);
                state_frame_.pos = output.joint_state.pos;
                state_frame_.vel = output.joint_state.vel;
                state_frame_.effort = output.joint_state.tor;
                state_frame_.model_feedforward = output.model_feedforward;
                state_frame_.cycle_dt = output.dt;
                state_frame_.valid = true;
            }
        }

        next_time += period;
        std::this_thread::sleep_until(next_time);
    }
}

/**
 * @brief 请求后台控制线程退出并等待结束
 */
void DmArmSystem::stop_worker() {
    worker_running_.store(false);
    if(worker_.joinable()) {
        worker_.join();
    }
}

/**
 * @brief 清除外部命令并切换为最近状态位置保持
 */
void DmArmSystem::clear_command() {
    std::vector<double> hold_pos;
    {
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        hold_pos = state_frame_.pos;
    }

    std::lock_guard<std::mutex> lock(command_mutex_);
    command_frame_.pos = std::move(hold_pos);
    std::fill(command_frame_.vel.begin(), command_frame_.vel.end(), 0.0);
    cmd_position_ = command_frame_.pos;
    cmd_velocity_ = command_frame_.vel;
    ++command_frame_.sequence;
}

/**
 * @brief 锁存后台线程错误并写入 ROS 日志
 * @param message 错误文本
 */
void DmArmSystem::set_error(const std::string& message) noexcept {
    try {
        std::lock_guard<std::mutex> lock(state_mutex_);
        last_error_ = message;
        worker_error_.store(true);
        RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", last_error_.c_str());
    }
    catch(...) {
        worker_error_.store(true);
    }
}

/**
 * @brief 将 RobotFault 转换为日志文本
 * @param action 当前失败操作名称
 * @param fault Robot 故障详情
 * @return 包含顶层错误和子错误的文本
 */
std::string DmArmSystem::make_robot_error(const char* action, const dm_arm::RobotFault& fault) const {
    std::ostringstream stream;
    stream << action << " failed; RobotErr=" << static_cast<int>(fault.code);
    stream << "; MotorBusErr=" << static_cast<int>(fault.motor_bus_err);
    stream << "; JointActuatorMapErr=" << static_cast<int>(fault.mapper_err);
    stream << "; JointCtrllerErr=" << static_cast<int>(fault.ctrller_err);
    stream << "; SafetyErr=" << static_cast<int>(fault.safety_fault.code);
    stream << "; ModelFeedforwardErr=" << static_cast<int>(fault.model_feedforward_err);
    return stream.str();
}

} // namespace dm_arm_ros2_control

PLUGINLIB_EXPORT_CLASS(dm_arm_ros2_control::DmArmSystem, hardware_interface::SystemInterface)
