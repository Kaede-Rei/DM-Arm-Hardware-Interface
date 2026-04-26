#include "dm_ros_control/dm_motor_bus.hpp"
#include "dm_hw/serial_port.hpp"

#include <unistd.h>

#include <stdexcept>

namespace dm_ros_control {

// ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

/**
 * @brief 配置电机 bus，创建串口、达妙控制器和电机对象
 * @param serial_port 串口设备路径
 * @param baudrate termios 波特率
 * @param configs 关节-电机配置表
 */
void DmMotorBus::configure(const std::string& serial_port, speed_t baudrate, const std::vector<DmMotorConfig>& configs) {
    _configs_ = configs;
    _serial_ = std::make_shared<SerialPort>(serial_port, baudrate);
    _motor_controller_ = std::make_shared<damiao::MotorControl>(_serial_);
    _motors_.clear();
    _motors_.reserve(_configs_.size());

    for(const auto& config : _configs_) {
        auto motor = std::make_shared<damiao::Motor>(config.motor_type, config.motor_id, 0x00);
        _motor_controller_->add_motor(motor.get());
        _motors_.push_back(motor);
    }
}

/**
 * @brief 激活 bus 管理的全部电机并读取启动初始状态
 * @param startup_read_cycles 启动阶段状态读取次数
 * @param states 输出启动阶段平均后的关节状态
 */
void DmMotorBus::activate(int startup_read_cycles, std::vector<DmJointState>& states) {
    for(size_t i = 0; i < _motors_.size(); ++i) {
        damiao::DmControlMode dm_mode;
        if(!to_dm_control_mode(_configs_[i].control_mode, dm_mode)) throw std::runtime_error("DmMotorBus unsupported control mode");

        _motor_controller_->enable(*_motors_[i]);
        _motor_controller_->switch_control_mode(*_motors_[i], dm_mode);
    }

    states.assign(_configs_.size(), {});
    for(int i = 0; i < startup_read_cycles; ++i) {
        for(size_t j = 0; j < _motors_.size(); ++j) {
            DmJointState state;
            if(!read_one(j, true, state)) throw std::runtime_error("DmMotorBus failed to read startup state");
            states[j].position += state.position;
            states[j].velocity += state.velocity;
            states[j].effort += state.effort;
            states[j].motor_effort += state.motor_effort;
        }
    }

    for(auto& state : states) {
        state.position /= static_cast<double>(startup_read_cycles);
        state.velocity /= static_cast<double>(startup_read_cycles);
        state.effort /= static_cast<double>(startup_read_cycles);
        state.motor_effort /= static_cast<double>(startup_read_cycles);
    }
}

/**
 * @brief 停用全部电机
 * @note 先切换到 POS_VEL 模式并发送零位置/低速度命令，再等待后失能
 */
void DmMotorBus::deactivate() {
    for(auto& motor : _motors_) {
        _motor_controller_->switch_control_mode(*motor, damiao::DmControlMode::POS_VEL_MODE);
        _motor_controller_->control_pos_vel(*motor, static_cast<float>(0.0f), static_cast<float>(1.0f));
    }

    usleep(5000000);

    for(auto& motor : _motors_) {
        _motor_controller_->disable(*motor);
    }
}

/**
 * @brief 清理 bus 持有的串口、电机控制器和电机对象
 */
void DmMotorBus::cleanup() {
    _motors_.clear();
    _motor_controller_.reset();
    _serial_.reset();
    _configs_.clear();
}

/**
 * @brief 读取所有电机状态并换算到关节侧
 * @param refresh_state 是否向电机主动请求状态刷新
 * @param states 预分配状态缓冲，大小必须等于电机数量
 * @return 成功返回 true，失败返回 false
 */
bool DmMotorBus::read(bool refresh_state, std::vector<DmJointState>& states) noexcept {
    try {
        if(!_motor_controller_ || states.size() != _motors_.size()) return false;

        for(size_t i = 0; i < _motors_.size(); ++i) {
            if(!read_one(i, refresh_state, states[i])) return false;
        }

        return true;
    }
    catch(...) {
        return false;
    }
}

/**
 * @brief 写入单个关节命令
 * @param index 关节/电机索引
 * @param command 关节侧命令
 * @return 成功返回 true，失败返回 false
 */
bool DmMotorBus::write(std::size_t index, const DmJointCommand& command) noexcept {
    try {
        if(!_motor_controller_ || index >= _motors_.size() || index >= _configs_.size()) return false;

        const auto& config = _configs_[index];
        const double scale = config.joint_to_motor_scale;

        const double cmd_motor = command.position * scale;
        const double cmd_vel_motor = command.velocity * scale;

        if(config.control_mode == ControlMode::MIT) {
            const double tau_motor = command.effort / scale;
            _motor_controller_->control_mit(*_motors_[index],
                static_cast<float>(command.kp),
                static_cast<float>(command.kd),
                static_cast<float>(cmd_motor),
                static_cast<float>(cmd_vel_motor),
                static_cast<float>(tau_motor));
        }
        else if(config.control_mode == ControlMode::POS_VEL) {
            _motor_controller_->control_pos_vel(*_motors_[index], static_cast<float>(cmd_motor), static_cast<float>(cmd_vel_motor));
        }
        else return false;

        return true;
    }
    catch(...) {
        return false;
    }
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 将内部控制模式转换为达妙 SDK 控制模式
 * @param mode 内部控制模式
 * @param dm_mode 输出达妙控制模式
 * @return 成功返回 true，未知模式返回 false
 */
bool DmMotorBus::to_dm_control_mode(ControlMode mode, damiao::DmControlMode& dm_mode) const noexcept {
    if(mode == ControlMode::MIT) {
        dm_mode = damiao::DmControlMode::MIT_MODE;
        return true;
    }
    if(mode == ControlMode::POS_VEL) {
        dm_mode = damiao::DmControlMode::POS_VEL_MODE;
        return true;
    }
    return false;
}

/**
 * @brief 读取单个电机并换算为关节侧状态
 * @param index 关节/电机索引
 * @param refresh_state 是否向电机主动请求状态刷新
 * @param state 输出关节侧状态
 * @return 成功返回 true，失败返回 false
 */
bool DmMotorBus::read_one(std::size_t index, bool refresh_state, DmJointState& state) noexcept {
    try {
        if(index >= _motors_.size() || index >= _configs_.size()) return false;
        if(refresh_state) _motor_controller_->refresh_motor_status(*_motors_[index]);

        const auto& config = _configs_[index];
        const double scale = config.joint_to_motor_scale;
        const double motor_effort = _motors_[index]->get_tau();

        state.position = _motors_[index]->get_position() / scale;
        state.velocity = _motors_[index]->get_velocity() / scale;
        state.motor_effort = motor_effort;
        state.effort = motor_effort * scale;
        return true;
    }
    catch(...) {
        return false;
    }
}

}
