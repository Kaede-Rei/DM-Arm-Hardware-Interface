#include "dm_ros_control/dm_hardware_interface.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <stdexcept>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace dm_ros_control {

// ! ========================= 宏 定 义 ========================= ! //

using CallbackReturn = hardware_interface::CallbackReturn;

// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 函 数 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

/**
 * @brief 初始化硬件接口
 * @param info 硬件信息
 * @return CallbackReturn
 */
CallbackReturn DmHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
    if(hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;

    _serial_port_ = info.hardware_parameters.at("serial_port");
    _baudrate_ = std::stoi(info.hardware_parameters.at("baudrate"));
    _enable_write_ = info.hardware_parameters.at("enable_write") == "true";
    _refresh_state_in_read_ = info.hardware_parameters.at("refresh_state_in_read") == "true";
    _startup_read_cycles_ = std::stoi(info.hardware_parameters.at("startup_read_cycles"));

    const auto n = info.joints.size();
    _joint_names_.resize(n);
    _motor_ids_.resize(n);
    _motor_types_.resize(n);
    _joint_to_motor_scale_.resize(n);
    _control_modes_.resize(n);
    _joint_kp_.assign(n, 0.0);
    _joint_kd_.assign(n, 0.0);

    _hw_positions_.assign(n, 0.0);
    _hw_velocities_.assign(n, 0.0);
    _hw_commands_pos_.assign(n, 0.0);
    _hw_commands_pos_prev_.assign(n, 0.0);
    _hw_commands_vel_.assign(n, 0.0);

    for(size_t i = 0; i < n; ++i) {
        const auto& joint = info.joints[i];
        _joint_names_[i] = joint.name;

        _motor_ids_[i] = static_cast<uint32_t>(std::stoul(joint.parameters.at("motor_id")));
        _motor_types_[i] = static_cast<damiao::DmMotorType>(std::stoi(joint.parameters.at("motor_type")));
        _joint_to_motor_scale_[i] = std::stod(joint.parameters.at("joint_to_motor_scale"));

        const auto mode = joint.parameters.at("control_mode");
        if(mode == "MIT") _control_modes_[i] = ControlMode::MIT;
        else if(mode == "POS_VEL")_control_modes_[i] = ControlMode::POS_VEL;
        else {
            RCLCPP_ERROR(rclcpp::get_logger("DmHardwareInterface"), "Unknown control_mode '%s' for joint '%s'", mode.c_str(), joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    _enable_dynamics_ = info.hardware_parameters.at("enable_dynamics") == "true";
    _enable_gravity_feedforward_ = info.hardware_parameters.at("enable_gravity_feedforward") == "true";
    _enable_nonlinear_feedforward_ = info.hardware_parameters.at("enable_nonlinear_feedforward") == "true";
    _urdf_path_ = info.hardware_parameters.at("urdf_path");

    _gravity_feedforward_.assign(info.joints.size(), 0.0);
    _nonlinear_feedforward_.assign(info.joints.size(), 0.0);

    if(!load_pd_gains_from_yaml()) {
        RCLCPP_WARN(rclcpp::get_logger("DmHardwareInterface"),
            "Failed to load pd_config.yaml, fallback to zero gains for all joints.");
    }

    return CallbackReturn::SUCCESS;
}

/**
 * @brief 配置硬件接口，建立与电机的通信
 * @param previous_state 上一个生命周期状态
 * @return CallbackReturn
 */
CallbackReturn DmHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;

    _serial_ = std::make_shared<SerialPort>(_serial_port_, baudrate_to_speed_t(_baudrate_));
    _motor_controller_ = std::make_shared<damiao::MotorControl>(_serial_);
    _motors_.clear();
    _motors_.reserve(_joint_names_.size());

    for(size_t i = 0; i < _joint_names_.size(); ++i) {
        auto motor = std::make_shared<damiao::Motor>(_motor_types_[i], _motor_ids_[i], 0x00);
        _motor_controller_->add_motor(motor.get());
        _motors_.push_back(motor);
    }

    if(_enable_dynamics_) _dynamics_model_ = std::make_shared<PinocchioDynamicsModel>(_urdf_path_, _joint_names_);

    return CallbackReturn::SUCCESS;
}

/**
 * @brief 激活硬件接口，使能电机并切换到指定控制模式
 * @param previous_state 上一个生命周期状态
 * @return CallbackReturn
 */
CallbackReturn DmHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;

    for(size_t i = 0; i < _motors_.size(); ++i) {
        _motor_controller_->enable(*_motors_[i]);

        if(_control_modes_[i] == ControlMode::MIT) {
            _motor_controller_->switch_control_mode(*_motors_[i], damiao::DmControlMode::MIT_MODE);
        }
        else if(_control_modes_[i] == ControlMode::POS_VEL) {
            _motor_controller_->switch_control_mode(*_motors_[i], damiao::DmControlMode::POS_VEL_MODE);
        }
        else {
            RCLCPP_ERROR(rclcpp::get_logger("DmHardwareInterface"), "Unsupported control mode for motor ID %u", _motor_ids_[i]);
            return CallbackReturn::ERROR;
        }
    }

    std::vector<double> sum_pos(_joint_names_.size(), 0.0);
    for(uint8_t i = 0; i < _startup_read_cycles_; ++i) {
        for(size_t j = 0; j < _motors_.size(); ++j) {
            _motor_controller_->refresh_motor_status(*_motors_[j]);
            const double scale = _joint_to_motor_scale_[j];
            sum_pos[j] += _motors_[j]->get_position() / scale;
        }
    }

    for(size_t i = 0; i < _joint_names_.size(); ++i) {
        _hw_positions_[i] = sum_pos[i] / static_cast<double>(_startup_read_cycles_);
        _hw_velocities_[i] = 0.0;
        _hw_commands_pos_[i] = _hw_positions_[i];
        _hw_commands_pos_prev_[i] = _hw_positions_[i];
        _hw_commands_vel_[i] = _hw_velocities_[i];
    }

    return CallbackReturn::SUCCESS;
}

/**
 * @brief 停用硬件接口，禁用电机
 * @param previous_state 上一个生命周期状态
 * @return CallbackReturn
 */
CallbackReturn DmHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;

    for(size_t i = 0; i < _motors_.size(); ++i) {
        _motor_controller_->switch_control_mode(*_motors_[i], damiao::DmControlMode::POS_VEL_MODE);
        _motor_controller_->control_pos_vel(*_motors_[i], static_cast<float>(0.0f), static_cast<float>(1.0f));
    }

    rclcpp::sleep_for(std::chrono::seconds(5));

    for(auto& motor : _motors_) {
        _motor_controller_->disable(*motor);
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

    _motors_.clear();
    _motor_controller_.reset();
    _serial_.reset();

    return CallbackReturn::SUCCESS;
}

/**
 * @brief 导出状态接口，提供位置和速度状态
 * @return 状态接口列表
 */
std::vector<hardware_interface::StateInterface> DmHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for(size_t i = 0; i < _joint_names_.size(); ++i) {
        state_interfaces.emplace_back(_joint_names_[i], hardware_interface::HW_IF_POSITION, &_hw_positions_[i]);
        state_interfaces.emplace_back(_joint_names_[i], hardware_interface::HW_IF_VELOCITY, &_hw_velocities_[i]);
    }
    return state_interfaces;
}

/**
 * @brief 导出命令接口，提供位置命令
 * @return 命令接口列表
 */
std::vector<hardware_interface::CommandInterface> DmHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for(size_t i = 0; i < _joint_names_.size(); ++i) {
        command_interfaces.emplace_back(_joint_names_[i], hardware_interface::HW_IF_POSITION, &_hw_commands_pos_[i]);
        command_interfaces.emplace_back(_joint_names_[i], hardware_interface::HW_IF_VELOCITY, &_hw_commands_vel_[i]);
    }
    return command_interfaces;
}

/**
 * @brief 读取电机状态，更新位置和速度
 * @param time 当前时间
 * @param period 周期时间
 * @return return_type
 */
hardware_interface::return_type DmHardwareInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period) {
    (void)time;
    (void)period;

    for(size_t i = 0; i < _motors_.size(); ++i) {
        if(_refresh_state_in_read_) _motor_controller_->refresh_motor_status(*_motors_[i]);
        const double scale = _joint_to_motor_scale_[i];
        _hw_positions_[i] = _motors_[i]->get_position() / scale;
        _hw_velocities_[i] = _motors_[i]->get_velocity() / scale;
    }

    if(_enable_dynamics_ && _dynamics_model_) {
        const bool ok = _dynamics_model_->update(_hw_positions_, _hw_velocities_);
        if(ok) {
            _gravity_feedforward_ = _dynamics_model_->get_gravity_std();
            _nonlinear_feedforward_ = _dynamics_model_->get_nonlinear_effects_std();
        }
        else RCLCPP_ERROR(rclcpp::get_logger("DmHardwareInterface"), "Failed to update dynamics model with current joint states.");
    }

    return hardware_interface::return_type::OK;
}

/**
 * @brief 写入命令到电机，根据控制模式计算目标位置和速度
 * @param time 当前时间
 * @param period 周期时间
 * @return return_type
 */
hardware_interface::return_type DmHardwareInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period) {
    (void)time;
    (void)period;

    if(!_enable_write_) return hardware_interface::return_type::OK;

    for(size_t i = 0; i < _motors_.size(); ++i) {
        const double scale = _joint_to_motor_scale_[i];

        double cmd_joint = _hw_commands_pos_[i];
        double cmd_vel_joint = _hw_commands_vel_[i];

        const double cmd_motor = cmd_joint * scale;
        const double cmd_vel_motor = cmd_vel_joint * scale;

        double tau_feedforward = 0.0;

        if(_enable_dynamics_) {
            if(_enable_nonlinear_feedforward_) tau_feedforward = _nonlinear_feedforward_[i];
            else if(_enable_gravity_feedforward_) tau_feedforward = _gravity_feedforward_[i];
        }

        if(_control_modes_[i] == ControlMode::MIT) {
            _motor_controller_->control_mit(*_motors_[i], static_cast<float>(_joint_kp_[i]), static_cast<float>(_joint_kd_[i]), static_cast<float>(cmd_motor), static_cast<float>(cmd_vel_motor), static_cast<float>(tau_feedforward));
        }
        else if(_control_modes_[i] == ControlMode::POS_VEL) {
            _motor_controller_->control_pos_vel(*_motors_[i], static_cast<float>(cmd_motor), static_cast<float>(cmd_vel_motor));
        }
        else {
            RCLCPP_ERROR(rclcpp::get_logger("DmHardwareInterface"), "Unsupported control mode for motor ID %u", _motor_ids_[i]);
            return hardware_interface::return_type::ERROR;
        }

        _hw_commands_pos_prev_[i] = cmd_joint;
    }

    return hardware_interface::return_type::OK;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

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

}

PLUGINLIB_EXPORT_CLASS(dm_ros_control::DmHardwareInterface, hardware_interface::SystemInterface)
