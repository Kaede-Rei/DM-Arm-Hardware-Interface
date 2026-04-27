#include "dm_control_core/dm_motor_bus.hpp"
#include "dm_hw/serial_port.hpp"

#include <stdexcept>
#include <unistd.h>

namespace dm_control_core {

// ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

/**
 * @brief 配置电机 bus，创建串口、达妙控制器和电机对象
 * @param serial_port 串口设备路径
 * @param baudrate termios 波特率
 * @param configs 关节-电机配置表
 */
void DmMotorBus::configure(const std::string& serial_port, speed_t baudrate, const std::vector<DmMotorConfig>& configs) {
    configs_ = configs;
    serial_ = std::make_shared<SerialPort>(serial_port, baudrate);
    motor_controller_ = std::make_shared<damiao::MotorControl>(serial_);
    motors_.clear();
    motors_.reserve(configs_.size());

    for(const auto& config : configs_) {
        auto motor = std::make_shared<damiao::Motor>(config.motor_type, config.motor_id, 0x00);
        motor_controller_->add_motor(motor.get());
        motors_.push_back(motor);
    }
}

/**
 * @brief 激活 bus 管理的全部电机并读取启动初始状态
 * @param startup_read_cycles 启动阶段状态读取次数
 * @param state 输出启动阶段平均后的关节状态
 */
void DmMotorBus::activate(int startup_read_cycles, JointState& state) {
    for(size_t i = 0; i < motors_.size(); ++i) {
        damiao::DmControlMode dm_mode;
        if(!to_dm_control_mode(configs_[i].control_mode, dm_mode)) throw std::runtime_error("DmMotorBus unsupported control mode");

        motor_controller_->enable(*motors_[i]);
        motor_controller_->switch_control_mode(*motors_[i], dm_mode);
    }

    const std::size_t n = configs_.size();
    state.position.assign(n, 0.0);
    state.velocity.assign(n, 0.0);
    state.effort.assign(n, 0.0);
    state.motor_effort.assign(n, 0.0);

    JointState sample;
    sample.position.assign(n, 0.0);
    sample.velocity.assign(n, 0.0);
    sample.effort.assign(n, 0.0);
    sample.motor_effort.assign(n, 0.0);

    for(int i = 0; i < startup_read_cycles; ++i) {
        for(size_t j = 0; j < motors_.size(); ++j) {
            if(!read_one(j, true, sample)) throw std::runtime_error("DmMotorBus failed to read startup state");
            state.position[j] += sample.position[j];
            state.velocity[j] += sample.velocity[j];
            state.effort[j] += sample.effort[j];
            state.motor_effort[j] += sample.motor_effort[j];
        }
    }

    for(std::size_t i = 0; i < n; ++i) {
        state.position[i] /= static_cast<double>(startup_read_cycles);
        state.velocity[i] /= static_cast<double>(startup_read_cycles);
        state.effort[i] /= static_cast<double>(startup_read_cycles);
        state.motor_effort[i] /= static_cast<double>(startup_read_cycles);
    }
}

/**
 * @brief 停用全部电机
 * @note 先切换到 POS_VEL 模式并发送零位置/低速度命令，再等待后失能
 */
void DmMotorBus::deactivate() {
    for(auto& motor : motors_) {
        motor_controller_->switch_control_mode(*motor, damiao::DmControlMode::POS_VEL_MODE);
        motor_controller_->control_pos_vel(*motor, static_cast<float>(0.0f), static_cast<float>(1.0f));
    }

    usleep(5000000);

    for(auto& motor : motors_) {
        motor_controller_->disable(*motor);
    }
}

/**
 * @brief 清理 bus 持有的串口、电机控制器和电机对象
 */
void DmMotorBus::cleanup() {
    motors_.clear();
    motor_controller_.reset();
    serial_.reset();
    configs_.clear();
}

/**
 * @brief 读取所有电机状态并换算到关节侧
 * @param refresh_state 是否向电机主动请求状态刷新
 * @param state 预分配状态缓冲，大小必须等于电机数量
 * @return 成功返回 true，失败返回 false
 */
bool DmMotorBus::read(bool refresh_state, JointState& state) noexcept {
    try {
        if(!motor_controller_) return false;
        if(state.position.size() != motors_.size()) return false;
        if(state.velocity.size() != motors_.size()) return false;
        if(state.effort.size() != motors_.size()) return false;
        if(state.motor_effort.size() != motors_.size()) return false;

        for(size_t i = 0; i < motors_.size(); ++i) {
            if(!read_one(i, refresh_state, state)) return false;
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
bool DmMotorBus::write(std::size_t index, const MitJointCommand& command) noexcept {
    try {
        if(!motor_controller_ || index >= motors_.size() || index >= configs_.size()) return false;
        if(index >= command.position.size() || index >= command.velocity.size() || index >= command.effort.size()) return false;
        if(index >= command.kp.size() || index >= command.kd.size()) return false;

        const auto& config = configs_[index];
        const double scale = config.joint_to_motor_scale;

        const double cmd_motor = command.position[index] * scale;
        const double cmd_vel_motor = command.velocity[index] * scale;

        if(config.control_mode == ControlMode::MIT) {
            const double tau_motor = command.effort[index] / scale;
            motor_controller_->control_mit(*motors_[index],
                static_cast<float>(command.kp[index]), static_cast<float>(command.kd[index]),
                static_cast<float>(cmd_motor), static_cast<float>(cmd_vel_motor), static_cast<float>(tau_motor));
        }
        else if(config.control_mode == ControlMode::POS_VEL) {
            motor_controller_->control_pos_vel(*motors_[index], static_cast<float>(cmd_motor), static_cast<float>(cmd_vel_motor));
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
bool DmMotorBus::read_one(std::size_t index, bool refresh_state, JointState& state) noexcept {
    try {
        if(index >= motors_.size() || index >= configs_.size()) return false;
        if(index >= state.position.size() || index >= state.velocity.size() || index >= state.effort.size()) return false;
        if(index >= state.motor_effort.size()) return false;
        if(refresh_state) motor_controller_->refresh_motor_status(*motors_[index]);

        const auto& config = configs_[index];
        const double scale = config.joint_to_motor_scale;
        const double motor_effort = motors_[index]->get_tau();

        state.position[index] = motors_[index]->get_position() / scale;
        state.velocity[index] = motors_[index]->get_velocity() / scale;
        state.motor_effort[index] = motor_effort;
        state.effort[index] = motor_effort * scale;
        return true;
    }
    catch(...) {
        return false;
    }
}

}
