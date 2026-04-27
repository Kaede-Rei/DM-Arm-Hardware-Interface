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

    _serial_port_ = get_param<std::string>(info, "serial_port", "/dev/ttyACM0");
    _baudrate_ = get_param<int>(info, "baudrate", 921600);
    _enable_write_ = get_param<bool>(info, "enable_write", true);
    _refresh_state_in_read_ = get_param<bool>(info, "refresh_state_in_read", false);
    _startup_read_cycles_ = get_param<int>(info, "startup_read_cycles", 5);

    if(_startup_read_cycles_ < 1) {
        RCLCPP_WARN(rclcpp::get_logger("DmHardwareInterface"),
            "Invalid startup_read_cycles (%d), must be at least 1. Defaulting to 5.", _startup_read_cycles_);
        _startup_read_cycles_ = 5;
    }

    _legacy_feedforward_enabled_ = get_param<bool>(info, "legacy_feedforward_enabled", true);
    _legacy_pd_fallback_ = get_param<bool>(info, "legacy_pd_fallback", true);

    _enable_dynamics_ = get_param<bool>(info, "enable_dynamics", false);
    _enable_gravity_feedforward_ = get_param<bool>(info, "enable_gravity_feedforward", false);
    _enable_nonlinear_feedforward_ = get_param<bool>(info, "enable_nonlinear_feedforward", false);
    _urdf_path_ = get_param<std::string>(info, "urdf_path", "");

    const auto n = info.joints.size();

    _joint_names_.resize(n);
    _motor_configs_.resize(n);

    _joint_kp_.assign(n, 0.0);
    _joint_kd_.assign(n, 0.0);

    _hw_positions_.assign(n, 0.0);
    _hw_velocities_.assign(n, 0.0);
    _hw_efforts_.assign(n, 0.0);
    _motor_efforts_.assign(n, 0.0);

    _hw_commands_pos_.assign(n, 0.0);
    _hw_commands_vel_.assign(n, 0.0);
    _hw_commands_effort_.assign(n, 0.0);
    _hw_commands_kp_.assign(n, 0.0);
    _hw_commands_kd_.assign(n, 0.0);
    _bus_state_.position.assign(n, 0.0);
    _bus_state_.velocity.assign(n, 0.0);
    _bus_state_.effort.assign(n, 0.0);
    _bus_state_.motor_effort.assign(n, 0.0);

    _gravity_feedforward_.assign(n, 0.0);
    _nonlinear_feedforward_.assign(n, 0.0);
    _active_feedforward_.assign(n, 0.0);
    _external_efforts_.assign(n, 0.0);
    _dynamics_observation_.gravity.assign(n, 0.0);
    _dynamics_observation_.nonlinear.assign(n, 0.0);
    _dynamics_observation_.active_feedforward.assign(n, 0.0);
    _dynamics_observation_.external_effort.assign(n, 0.0);

    for(size_t i = 0; i < n; ++i) {
        const auto& joint = info.joints[i];
        _joint_names_[i] = joint.name;

        _motor_configs_[i].joint_name = joint.name;
        _motor_configs_[i].motor_id = static_cast<uint32_t>(std::stoul(joint.parameters.at("motor_id")));
        _motor_configs_[i].motor_type = static_cast<damiao::DmMotorType>(std::stoi(joint.parameters.at("motor_type")));
        _motor_configs_[i].joint_to_motor_scale = std::stod(joint.parameters.at("joint_to_motor_scale"));

        if(_motor_configs_[i].joint_to_motor_scale == 0.0) {
            RCLCPP_ERROR(rclcpp::get_logger("DmHardwareInterface"), "joint_to_motor_scale for joint '%s' must not be zero.", joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        const auto mode = joint.parameters.at("control_mode");
        if(mode == "MIT") _motor_configs_[i].control_mode = dm_control_core::ControlMode::MIT;
        else if(mode == "POS_VEL")_motor_configs_[i].control_mode = dm_control_core::ControlMode::POS_VEL;
        else {
            RCLCPP_ERROR(rclcpp::get_logger("DmHardwareInterface"), "Unknown control_mode '%s' for joint '%s'", mode.c_str(), joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    if(_legacy_pd_fallback_) {
        if(!load_pd_gains_from_yaml()) {
            RCLCPP_WARN(rclcpp::get_logger("DmHardwareInterface"),
                "Failed to load pd_config.yaml, fallback to zero gains for all joints.");
        }

        for(std::size_t i = 0; i < n; ++i) {
            _hw_commands_kp_[i] = _joint_kp_[i];
            _hw_commands_kd_[i] = _joint_kd_[i];
        }
    }

    try {
        _joint_impedance_controller_.configure(build_joint_impedance_config());
    }
    catch(const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("DmHardwareInterface"), "Failed to configure joint impedance controller: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("DmHardwareInterface"),
        "Initialized DmHardwareInterface: joints=%zu, write=%s, dynamics=%s, legacy_feedforward=%s, legacy_pd=%s", n,
        _enable_write_ ? "true" : "false",
        _enable_dynamics_ ? "true" : "false",
        _legacy_feedforward_enabled_ ? "true" : "false",
        _legacy_pd_fallback_ ? "true" : "false");

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
        _motor_bus_.configure(_serial_port_, baudrate_to_speed_t(_baudrate_), _motor_configs_);

        if(_enable_dynamics_) {
            if(_urdf_path_.empty()) {
                RCLCPP_ERROR(rclcpp::get_logger("DmHardwareInterface"), "URDF path must be specified to enable dynamics model.");
                return CallbackReturn::ERROR;
            }
            _dynamics_observer_.configure(_urdf_path_, _joint_names_);
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
        _motor_bus_.activate(_startup_read_cycles_, startup_state);
    }
    catch(const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("DmHardwareInterface"), "Exception during on_activate: %s", e.what());
        return CallbackReturn::ERROR;
    }

    for(size_t i = 0; i < _joint_names_.size(); ++i) {
        _hw_positions_[i] = startup_state.position[i];
        _hw_velocities_[i] = startup_state.velocity[i];
        _hw_efforts_[i] = startup_state.effort[i];
        _motor_efforts_[i] = startup_state.motor_effort[i];

        _hw_commands_pos_[i] = _hw_positions_[i];
        _hw_commands_vel_[i] = 0.0;
        _hw_commands_effort_[i] = 0.0;

        if(_legacy_pd_fallback_) {
            _hw_commands_kp_[i] = _joint_kp_[i];
            _hw_commands_kd_[i] = _joint_kd_[i];
        }

        _active_feedforward_[i] = 0.0;
        _external_efforts_[i] = 0.0;
    }

    _bus_state_ = startup_state;
    _joint_impedance_controller_.reset(startup_state);
    _joint_impedance_controller_.set_mode(dm_control_core::JointImpedanceMode::TRACKING, startup_state);

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
        _motor_bus_.deactivate();
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

    _motor_bus_.cleanup();
    _dynamics_observer_.cleanup();

    return CallbackReturn::SUCCESS;
}

/**
 * @brief 导出状态接口
 * @return 状态接口列表
 */
std::vector<hardware_interface::StateInterface> DmHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for(size_t i = 0; i < _joint_names_.size(); ++i) {
        state_interfaces.emplace_back(_joint_names_[i], hardware_interface::HW_IF_POSITION, &_hw_positions_[i]);
        state_interfaces.emplace_back(_joint_names_[i], hardware_interface::HW_IF_VELOCITY, &_hw_velocities_[i]);
        state_interfaces.emplace_back(_joint_names_[i], hardware_interface::HW_IF_EFFORT, &_hw_efforts_[i]);
        state_interfaces.emplace_back(_joint_names_[i], "motor_effort", &_motor_efforts_[i]);
        state_interfaces.emplace_back(_joint_names_[i], "gravity_effort", &_gravity_feedforward_[i]);
        state_interfaces.emplace_back(_joint_names_[i], "nonlinear_effort", &_nonlinear_feedforward_[i]);
        state_interfaces.emplace_back(_joint_names_[i], "feedforward_effort", &_active_feedforward_[i]);
        state_interfaces.emplace_back(_joint_names_[i], "external_effort", &_external_efforts_[i]);
    }
    return state_interfaces;
}

/**
 * @brief 导出命令接口
 * @return 命令接口列表
 */
std::vector<hardware_interface::CommandInterface> DmHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for(size_t i = 0; i < _joint_names_.size(); ++i) {
        command_interfaces.emplace_back(_joint_names_[i], hardware_interface::HW_IF_POSITION, &_hw_commands_pos_[i]);
        command_interfaces.emplace_back(_joint_names_[i], hardware_interface::HW_IF_VELOCITY, &_hw_commands_vel_[i]);
        command_interfaces.emplace_back(_joint_names_[i], hardware_interface::HW_IF_EFFORT, &_hw_commands_effort_[i]);
        command_interfaces.emplace_back(_joint_names_[i], "kp", &_hw_commands_kp_[i]);
        command_interfaces.emplace_back(_joint_names_[i], "kd", &_hw_commands_kd_[i]);
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

    if(!_motor_bus_.read(_refresh_state_in_read_, _bus_state_)) {
        RCLCPP_ERROR(rclcpp::get_logger("DmHardwareInterface"), "Failed to read motor bus state.");
        return hardware_interface::return_type::ERROR;
    }

    for(size_t i = 0; i < _joint_names_.size(); ++i) {
        _hw_positions_[i] = _bus_state_.position[i];
        _hw_velocities_[i] = _bus_state_.velocity[i];
        _hw_efforts_[i] = _bus_state_.effort[i];
        _motor_efforts_[i] = _bus_state_.motor_effort[i];
    }

    if(_enable_dynamics_) {
        const bool ok = _dynamics_observer_.observe(_hw_positions_, _hw_velocities_, _hw_efforts_,
            _enable_gravity_feedforward_, _enable_nonlinear_feedforward_, _dynamics_observation_);

        if(ok) {
            _gravity_feedforward_ = _dynamics_observation_.gravity;
            _nonlinear_feedforward_ = _dynamics_observation_.nonlinear;
            _active_feedforward_ = _dynamics_observation_.active_feedforward;
            _external_efforts_ = _dynamics_observation_.external_effort;
        }
        else {
            RCLCPP_ERROR(rclcpp::get_logger("DmHardwareInterface"), "Failed to update dynamics observer with current joint states.");
            std::fill(_gravity_feedforward_.begin(), _gravity_feedforward_.end(), 0.0);
            std::fill(_nonlinear_feedforward_.begin(), _nonlinear_feedforward_.end(), 0.0);
            std::fill(_active_feedforward_.begin(), _active_feedforward_.end(), 0.0);
            std::fill(_external_efforts_.begin(), _external_efforts_.end(), 0.0);
        }
    }
    else {
        std::fill(_gravity_feedforward_.begin(), _gravity_feedforward_.end(), 0.0);
        std::fill(_nonlinear_feedforward_.begin(), _nonlinear_feedforward_.end(), 0.0);
        std::fill(_active_feedforward_.begin(), _active_feedforward_.end(), 0.0);
        std::fill(_external_efforts_.begin(), _external_efforts_.end(), 0.0);
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

    if(!_enable_write_) return hardware_interface::return_type::OK;

    dm_control_core::JointReference reference;
    reference.position = _hw_commands_pos_;
    reference.velocity = _hw_commands_vel_;
    reference.effort = _hw_commands_effort_;

    _joint_impedance_controller_.set_reference(reference);

    dm_control_core::JointImpedanceControllerInput input;
    input.state = _bus_state_;
    input.gravity_effort = _active_feedforward_;
    input.dt = period.seconds();

    const auto output = _joint_impedance_controller_.update(input);
    if(output.command.position.size() != _motor_bus_.size()) {
        RCLCPP_ERROR(rclcpp::get_logger("DmHardwareInterface"), "Joint impedance controller returned invalid command size.");
        return hardware_interface::return_type::ERROR;
    }

    for(size_t i = 0; i < _motor_bus_.size(); ++i) {
        if(!_motor_bus_.write(i, output.command)) {
            RCLCPP_ERROR(rclcpp::get_logger("DmHardwareInterface"), "Failed to write command for joint '%s'.", _joint_names_[i].c_str());
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

        for(size_t i = 0; i < _joint_names_.size(); ++i) {
            const auto& joint_name = _joint_names_[i];
            const YAML::Node gains = joint_pd[joint_name];
            if(!gains || !gains.IsMap()) {
                RCLCPP_WARN(rclcpp::get_logger("DmHardwareInterface"),
                    "No PD config for joint '%s', fallback to kp=%.3f kd=%.3f.",
                    joint_name.c_str(), _joint_kp_[i], _joint_kd_[i]);
                continue;
            }

            if(gains["kp"]) _joint_kp_[i] = gains["kp"].as<double>();
            if(gains["kd"]) _joint_kd_[i] = gains["kd"].as<double>();
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
    const std::size_t n = _joint_names_.size();

    dm_control_core::JointImpedanceControllerConfig config;
    config.layout.joint_names = _joint_names_;
    config.rigid_hold_gains.kp = _joint_kp_;
    config.rigid_hold_gains.kd = _joint_kd_;
    config.compliant_hold_gains.kp = _joint_kp_;
    config.compliant_hold_gains.kd = _joint_kd_;
    config.tracking_gains.kp = _joint_kp_;
    config.tracking_gains.kd = _joint_kd_;
    config.limits.max_velocity.assign(n, 0.0);
    config.limits.max_effort.assign(n, 0.0);
    config.limits.min_kp.assign(n, std::numeric_limits<double>::lowest());
    config.limits.max_kp.assign(n, std::numeric_limits<double>::max());
    config.limits.min_kd.assign(n, std::numeric_limits<double>::lowest());
    config.limits.max_kd.assign(n, std::numeric_limits<double>::max());
    config.use_gravity_feedforward = _legacy_feedforward_enabled_;
    config.use_reference_effort = true;

    return config;
}

}

PLUGINLIB_EXPORT_CLASS(dm_ros_control::DmHardwareInterface, hardware_interface::SystemInterface)
