#include "dm_ros_control/dm_hardware_interface.hpp"
#include "tl/optional.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <filesystem>
#include <limits>
#include <stdexcept>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace dm_ros_control {

// ! ========================= 宏 定 义 ========================= ! //

using CallbackReturn = hardware_interface::CallbackReturn;

// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 工 具 函 数 声 明 ========================= ! //

namespace {

/**
 * @brief 尝试从硬件参数表读取并转换参数
 * @tparam T 目标参数类型
 * @param info ros2_control 硬件信息
 * @param key 参数名
 * @return 转换成功返回参数值，否则返回 nullopt
 */
template<typename T>
tl::optional<T> try_get_param(const hardware_interface::HardwareInfo& info, const std::string& key) {
    const auto it = info.hardware_parameters.find(key);
    if(it == info.hardware_parameters.end()) return tl::nullopt;

    try {
        return YAML::Node(it->second).as<T>();
    }
    catch(...) {
        return tl::nullopt;
    }
}

/**
 * @brief 从硬件参数表读取参数，失败时返回默认值
 * @tparam T 目标参数类型
 * @param info ros2_control 硬件信息
 * @param key 参数名
 * @param default_value 默认值
 * @return 参数值或默认值
 */
template<typename T>
T get_param(const hardware_interface::HardwareInfo& info, const std::string& key, const T& default_value) {
    return try_get_param<T>(info, key).value_or(default_value);
}

}

// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

/**
 * @brief 初始化硬件接口
 * @param info 硬件信息
 * @return CallbackReturn
 */
CallbackReturn DmHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
    if(hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;

    serial_port_ = get_param<std::string>(info, "serial_port", "/dev/ttyACM0");
    baudrate_ = get_param<int>(info, "baudrate", 921600);
    enable_write_ = get_param<bool>(info, "enable_write", true);
    refresh_state_in_read_ = get_param<bool>(info, "refresh_state_in_read", false);
    startup_read_cycles_ = get_param<int>(info, "startup_read_cycles", 5);

    if(startup_read_cycles_ < 1) {
        RCLCPP_WARN(rclcpp::get_logger("DmHardwareInterface"),
            "Invalid startup_read_cycles (%d), must be at least 1. Defaulting to 5.", startup_read_cycles_);
        startup_read_cycles_ = 5;
    }

    legacy_feedforward_enabled_ = get_param<bool>(info, "legacy_feedforward_enabled", true);
    legacy_pd_fallback_ = get_param<bool>(info, "legacy_pd_fallback", true);
    command_mode_ = parse_command_mode(get_param<std::string>(info, "command_mode", "impedance"));

    enable_dynamics_ = get_param<bool>(info, "enable_dynamics", false);
    enable_gravity_feedforward_ = get_param<bool>(info, "enable_gravity_feedforward", false);
    enable_nonlinear_feedforward_ = get_param<bool>(info, "enable_nonlinear_feedforward", false);
    urdf_path_ = get_param<std::string>(info, "urdf_path", "");

    const auto n = info.joints.size();

    joint_names_.resize(n);
    motor_configs_.resize(n);

    joint_kp_.assign(n, 0.0);
    joint_kd_.assign(n, 0.0);

    hw_positions_.assign(n, 0.0);
    hw_velocities_.assign(n, 0.0);
    hw_efforts_.assign(n, 0.0);
    motor_efforts_.assign(n, 0.0);

    hw_commands_pos_.assign(n, 0.0);
    hw_commands_vel_.assign(n, 0.0);
    hw_commands_effort_.assign(n, 0.0);
    hw_commands_kp_.assign(n, 0.0);
    hw_commands_kd_.assign(n, 0.0);
    bus_state_.position.assign(n, 0.0);
    bus_state_.velocity.assign(n, 0.0);
    bus_state_.effort.assign(n, 0.0);
    bus_state_.motor_effort.assign(n, 0.0);

    gravity_feedforward_.assign(n, 0.0);
    nonlinear_feedforward_.assign(n, 0.0);
    active_feedforward_.assign(n, 0.0);
    external_efforts_.assign(n, 0.0);
    dynamics_observation_.gravity.assign(n, 0.0);
    dynamics_observation_.nonlinear.assign(n, 0.0);
    dynamics_observation_.active_feedforward.assign(n, 0.0);
    dynamics_observation_.external_effort.assign(n, 0.0);

    for(size_t i = 0; i < n; ++i) {
        const auto& joint = info.joints[i];
        joint_names_[i] = joint.name;

        motor_configs_[i].joint_name = joint.name;
        motor_configs_[i].motor_id = static_cast<uint32_t>(std::stoul(joint.parameters.at("motor_id")));
        motor_configs_[i].motor_type = static_cast<damiao::DmMotorType>(std::stoi(joint.parameters.at("motor_type")));
        motor_configs_[i].joint_to_motor_scale = std::stod(joint.parameters.at("joint_to_motor_scale"));

        if(motor_configs_[i].joint_to_motor_scale == 0.0) {
            RCLCPP_ERROR(rclcpp::get_logger("DmHardwareInterface"), "joint_to_motor_scale for joint '%s' must not be zero.", joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        const auto mode = joint.parameters.at("control_mode");
        if(mode == "MIT") motor_configs_[i].control_mode = dm_control_core::ControlMode::MIT;
        else if(mode == "POS_VEL")motor_configs_[i].control_mode = dm_control_core::ControlMode::POS_VEL;
        else {
            RCLCPP_ERROR(rclcpp::get_logger("DmHardwareInterface"), "Unknown control_mode '%s' for joint '%s'", mode.c_str(), joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    if(legacy_pd_fallback_) {
        if(!load_pd_gains_from_yaml()) {
            RCLCPP_WARN(rclcpp::get_logger("DmHardwareInterface"),
                "Failed to load pd_config.yaml, fallback to zero gains for all joints.");
        }

        for(std::size_t i = 0; i < n; ++i) {
            hw_commands_kp_[i] = joint_kp_[i];
            hw_commands_kd_[i] = joint_kd_[i];
        }
    }

    try {
        joint_impedance_controller_.configure(build_joint_impedance_config());
    }
    catch(const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("DmHardwareInterface"), "Failed to configure joint impedance controller: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("DmHardwareInterface"),
        "Initialized DmHardwareInterface: joints=%zu, write=%s, dynamics=%s, legacy_feedforward=%s, legacy_pd=%s", n,
        enable_write_ ? "true" : "false",
        enable_dynamics_ ? "true" : "false",
        legacy_feedforward_enabled_ ? "true" : "false",
        legacy_pd_fallback_ ? "true" : "false");

    return CallbackReturn::SUCCESS;
}

/**
 * @brief 配置硬件接口，建立与电机的通信
 * @param previous_state 上一个生命周期状态
 * @return CallbackReturn
 */
CallbackReturn DmHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;

    try {
        motor_bus_.configure(serial_port_, baudrate_to_speed_t(baudrate_), motor_configs_);

        if(enable_dynamics_) {
            if(urdf_path_.empty()) {
                RCLCPP_ERROR(rclcpp::get_logger("DmHardwareInterface"), "URDF path must be specified to enable dynamics model.");
                return CallbackReturn::ERROR;
            }
            dynamics_observer_.configure(urdf_path_, joint_names_);
        }
    }
    catch(const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("DmHardwareInterface"), "Exception during on_configure: %s", e.what());
        return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}

/**
 * @brief 激活硬件接口，使能电机并切换到指定控制模式
 * @param previous_state 上一个生命周期状态
 * @return CallbackReturn
 */
CallbackReturn DmHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;

    dm_control_core::JointState startup_state;
    try {
        motor_bus_.activate(startup_read_cycles_, startup_state);
    }
    catch(const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("DmHardwareInterface"), "Exception during on_activate: %s", e.what());
        return CallbackReturn::ERROR;
    }

    for(size_t i = 0; i < joint_names_.size(); ++i) {
        hw_positions_[i] = startup_state.position[i];
        hw_velocities_[i] = startup_state.velocity[i];
        hw_efforts_[i] = startup_state.effort[i];
        motor_efforts_[i] = startup_state.motor_effort[i];

        hw_commands_pos_[i] = hw_positions_[i];
        hw_commands_vel_[i] = 0.0;
        hw_commands_effort_[i] = 0.0;

        if(legacy_pd_fallback_) {
            hw_commands_kp_[i] = joint_kp_[i];
            hw_commands_kd_[i] = joint_kd_[i];
        }

        active_feedforward_[i] = 0.0;
        external_efforts_[i] = 0.0;
    }

    bus_state_ = startup_state;
    joint_impedance_controller_.reset(startup_state);
    joint_impedance_controller_.set_mode(dm_control_core::JointImpedanceMode::TRACKING, startup_state);

    return CallbackReturn::SUCCESS;
}

/**
 * @brief 停用硬件接口，禁用电机
 * @param previous_state 上一个生命周期状态
 * @return CallbackReturn
 */
CallbackReturn DmHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;

    try {
        motor_bus_.deactivate();
    }
    catch(const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("DmHardwareInterface"), "Exception during on_deactivate: %s", e.what());
        return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}

/**
 * @brief 清理硬件接口，释放资源
 * @param previous_state 上一个生命周期状态
 * @return CallbackReturn
 */
CallbackReturn DmHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;

    motor_bus_.cleanup();
    dynamics_observer_.cleanup();

    return CallbackReturn::SUCCESS;
}

/**
 * @brief 导出状态接口
 * @return 状态接口列表
 */
std::vector<hardware_interface::StateInterface> DmHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for(size_t i = 0; i < joint_names_.size(); ++i) {
        state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
        state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
        state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]);
        state_interfaces.emplace_back(joint_names_[i], "motor_effort", &motor_efforts_[i]);
        state_interfaces.emplace_back(joint_names_[i], "gravity_effort", &gravity_feedforward_[i]);
        state_interfaces.emplace_back(joint_names_[i], "nonlinear_effort", &nonlinear_feedforward_[i]);
        state_interfaces.emplace_back(joint_names_[i], "feedforward_effort", &active_feedforward_[i]);
        state_interfaces.emplace_back(joint_names_[i], "external_effort", &external_efforts_[i]);
    }
    return state_interfaces;
}

/**
 * @brief 导出命令接口
 * @return 命令接口列表
 */
std::vector<hardware_interface::CommandInterface> DmHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for(size_t i = 0; i < joint_names_.size(); ++i) {
        command_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_commands_pos_[i]);
        command_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_commands_vel_[i]);
        command_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_EFFORT, &hw_commands_effort_[i]);
        command_interfaces.emplace_back(joint_names_[i], "kp", &hw_commands_kp_[i]);
        command_interfaces.emplace_back(joint_names_[i], "kd", &hw_commands_kd_[i]);
    }
    return command_interfaces;
}

/**
 * @brief 控制周期读取电机状态和动力学观测
 * @param time 当前时间
 * @param period 周期时间
 * @return return_type
 */
hardware_interface::return_type DmHardwareInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period) {
    (void)time;
    (void)period;

    if(!motor_bus_.read(refresh_state_in_read_, bus_state_)) {
        RCLCPP_ERROR(rclcpp::get_logger("DmHardwareInterface"), "Failed to read motor bus state.");
        return hardware_interface::return_type::ERROR;
    }

    for(size_t i = 0; i < joint_names_.size(); ++i) {
        hw_positions_[i] = bus_state_.position[i];
        hw_velocities_[i] = bus_state_.velocity[i];
        hw_efforts_[i] = bus_state_.effort[i];
        motor_efforts_[i] = bus_state_.motor_effort[i];
    }

    if(enable_dynamics_) {
        const bool ok = dynamics_observer_.observe(hw_positions_, hw_velocities_, hw_efforts_,
            enable_gravity_feedforward_, enable_nonlinear_feedforward_, dynamics_observation_);

        if(ok) {
            gravity_feedforward_ = dynamics_observation_.gravity;
            nonlinear_feedforward_ = dynamics_observation_.nonlinear;
            active_feedforward_ = dynamics_observation_.active_feedforward;
            external_efforts_ = dynamics_observation_.external_effort;
        }
        else {
            RCLCPP_ERROR(rclcpp::get_logger("DmHardwareInterface"), "Failed to update dynamics observer with current joint states.");
            std::fill(gravity_feedforward_.begin(), gravity_feedforward_.end(), 0.0);
            std::fill(nonlinear_feedforward_.begin(), nonlinear_feedforward_.end(), 0.0);
            std::fill(active_feedforward_.begin(), active_feedforward_.end(), 0.0);
            std::fill(external_efforts_.begin(), external_efforts_.end(), 0.0);
        }
    }
    else {
        std::fill(gravity_feedforward_.begin(), gravity_feedforward_.end(), 0.0);
        std::fill(nonlinear_feedforward_.begin(), nonlinear_feedforward_.end(), 0.0);
        std::fill(active_feedforward_.begin(), active_feedforward_.end(), 0.0);
        std::fill(external_efforts_.begin(), external_efforts_.end(), 0.0);
    }

    return hardware_interface::return_type::OK;
}

/**
 * @brief 控制周期写入命令到电机
 * @param time 当前时间
 * @param period 周期时间
 * @return return_type
 */
hardware_interface::return_type DmHardwareInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period) {
    (void)time;

    if(!enable_write_) return hardware_interface::return_type::OK;

    dm_control_core::JointCommand command;
    command.mode = command_mode_;

    switch(command_mode_) {
        case dm_control_core::JointCommandMode::HOLD:
            break;

        case dm_control_core::JointCommandMode::POSITION:
            command.position = hw_commands_pos_;
            break;

        case dm_control_core::JointCommandMode::POSITION_VELOCITY:
            command.position = hw_commands_pos_;
            command.velocity = hw_commands_vel_;
            break;

        case dm_control_core::JointCommandMode::IMPEDANCE:
            command.position = hw_commands_pos_;
            command.velocity = hw_commands_vel_;
            command.effort = hw_commands_effort_;
            break;

        case dm_control_core::JointCommandMode::VELOCITY:
            command.velocity = hw_commands_vel_;
            break;

        case dm_control_core::JointCommandMode::TORQUE:
            command.effort = hw_commands_effort_;
            break;
    }

    const auto command_result = joint_impedance_controller_.set_command(command);
    if(!command_result) {
        RCLCPP_ERROR(rclcpp::get_logger("DmHardwareInterface"),
            "Invalid joint command: %s.", command_error_to_string(command_result.error()));
        return hardware_interface::return_type::ERROR;
    }

    dm_control_core::JointImpedanceControllerInput input;
    input.state = bus_state_;
    input.model_feedforward = active_feedforward_;
    input.dt = period.seconds();

    const auto output = joint_impedance_controller_.update(input);
    if(!output.valid) {
        RCLCPP_ERROR(rclcpp::get_logger("DmHardwareInterface"), "Joint impedance controller returned invalid output.");
        return hardware_interface::return_type::ERROR;
    }

    if(output.command.position.size() != motor_bus_.size()) {
        RCLCPP_ERROR(rclcpp::get_logger("DmHardwareInterface"), "Joint impedance controller returned invalid command size.");
        return hardware_interface::return_type::ERROR;
    }

    for(size_t i = 0; i < motor_bus_.size(); ++i) {
        if(!motor_bus_.write(i, output.command)) {
            RCLCPP_ERROR(rclcpp::get_logger("DmHardwareInterface"), "Failed to write command for joint '%s'.", joint_names_[i].c_str());
            return hardware_interface::return_type::ERROR;
        }
    }

    return hardware_interface::return_type::OK;
}

// ! ========================= 私 有 类 方 法 实 现 ========================= ! //

/**
 * @brief 将波特率转换为 speed_t 类型
 * @param baudrate 波特率
 * @return speed_t 对应的速度值
 */
speed_t DmHardwareInterface::baudrate_to_speed_t(int baudrate) const {
    switch(baudrate) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default:
            throw std::runtime_error("Unsupported baudrate: " + std::to_string(baudrate));
    }
}

/**
 * @brief 解析上层关节命令语义
 * @param mode 命令模式字符串
 * @return 关节命令模式
 */
dm_control_core::JointCommandMode DmHardwareInterface::parse_command_mode(const std::string& mode) const {
    if(mode == "hold") return dm_control_core::JointCommandMode::HOLD;
    if(mode == "position") return dm_control_core::JointCommandMode::POSITION;
    if(mode == "position_velocity") return dm_control_core::JointCommandMode::POSITION_VELOCITY;
    if(mode == "impedance") return dm_control_core::JointCommandMode::IMPEDANCE;
    if(mode == "velocity") return dm_control_core::JointCommandMode::VELOCITY;
    if(mode == "torque") return dm_control_core::JointCommandMode::TORQUE;

    RCLCPP_WARN(rclcpp::get_logger("DmHardwareInterface"),
        "Unknown command_mode '%s', fallback to impedance.", mode.c_str());
    return dm_control_core::JointCommandMode::IMPEDANCE;
}

/**
 * @brief 将命令校验错误转为日志字符串
 * @param error 命令校验错误
 * @return 错误描述
 */
const char* DmHardwareInterface::command_error_to_string(dm_control_core::JointCommandError error) const {
    switch(error) {
        case dm_control_core::JointCommandError::MISSING_POSITION: return "missing position command";
        case dm_control_core::JointCommandError::MISSING_VELOCITY: return "missing velocity command";
        case dm_control_core::JointCommandError::MISSING_EFFORT: return "missing effort command";
        case dm_control_core::JointCommandError::INVALID_POSITION_SIZE: return "invalid position command size";
        case dm_control_core::JointCommandError::INVALID_VELOCITY_SIZE: return "invalid velocity command size";
        case dm_control_core::JointCommandError::INVALID_EFFORT_SIZE: return "invalid effort command size";
        case dm_control_core::JointCommandError::INVALID_STATE_SIZE: return "invalid state size";
    }

    return "unknown command error";
}

/**
 * @brief 从安装目录读取 pd_config.yaml 中的关节 PD 增益
 * @return 读取成功返回 true，文件缺失或格式错误返回 false
 */
bool DmHardwareInterface::load_pd_gains_from_yaml() {
    try {
        const auto package_share = ament_index_cpp::get_package_share_directory("dm_ros_control");
        const auto config_path = std::filesystem::path(package_share) / "config" / "pd_config.yaml";

        if(!std::filesystem::exists(config_path)) {
            RCLCPP_WARN(rclcpp::get_logger("DmHardwareInterface"),
                "PD config file not found: %s", config_path.c_str());
            return false;
        }

        const YAML::Node root = YAML::LoadFile(config_path.string());
        const YAML::Node joint_pd = root["joint_pd"];
        if(!joint_pd || !joint_pd.IsMap()) {
            RCLCPP_WARN(rclcpp::get_logger("DmHardwareInterface"),
                "Invalid pd_config.yaml: 'joint_pd' section is missing or not a map.");
            return false;
        }

        for(size_t i = 0; i < joint_names_.size(); ++i) {
            const auto& joint_name = joint_names_[i];
            const YAML::Node gains = joint_pd[joint_name];
            if(!gains || !gains.IsMap()) {
                RCLCPP_WARN(rclcpp::get_logger("DmHardwareInterface"),
                    "No PD config for joint '%s', fallback to kp=%.3f kd=%.3f.",
                    joint_name.c_str(), joint_kp_[i], joint_kd_[i]);
                continue;
            }

            if(gains["kp"]) joint_kp_[i] = gains["kp"].as<double>();
            if(gains["kd"]) joint_kd_[i] = gains["kd"].as<double>();
        }
    }
    catch(const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("DmHardwareInterface"),
            "Exception while loading pd_config.yaml: %s", e.what());
        return false;
    }

    return true;
}

/**
 * @brief 构造纯 C++ 关节阻抗控制器配置
 * @return 关节阻抗控制器配置
 */
dm_control_core::JointImpedanceControllerConfig DmHardwareInterface::build_joint_impedance_config() const {
    const std::size_t n = joint_names_.size();

    dm_control_core::JointImpedanceControllerConfig config;
    config.layout.joint_names = joint_names_;
    config.rigid_hold_gains.kp = joint_kp_;
    config.rigid_hold_gains.kd = joint_kd_;
    config.compliant_hold_gains.kp = joint_kp_;
    config.compliant_hold_gains.kd = joint_kd_;
    config.tracking_gains.kp = joint_kp_;
    config.tracking_gains.kd = joint_kd_;
    config.limits.max_velocity.assign(n, 0.0);
    config.limits.max_effort.assign(n, 0.0);
    config.limits.min_kp.assign(n, std::numeric_limits<double>::lowest());
    config.limits.max_kp.assign(n, std::numeric_limits<double>::max());
    config.limits.min_kd.assign(n, std::numeric_limits<double>::lowest());
    config.limits.max_kd.assign(n, std::numeric_limits<double>::max());
    config.use_model_feedforward = legacy_feedforward_enabled_;
    config.use_command_effort = true;

    return config;
}

}

PLUGINLIB_EXPORT_CLASS(dm_ros_control::DmHardwareInterface, hardware_interface::SystemInterface)
