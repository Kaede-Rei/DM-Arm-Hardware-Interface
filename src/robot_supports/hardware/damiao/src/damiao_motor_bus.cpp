#include "serial_arm_hardware_damiao/damiao_motor_bus.hpp"

#include "serial_arm_protocol_damiao_usb2can/bus.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <thread>

namespace serial_arm {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //

namespace {

/**
 * @brief 检查向量中的所有值是否为有限值
 * @param values 待检查的向量
 * @return 如果所有值都是有限值，则返回 true，否则返回 false
 */
bool finite_vector(const std::vector<double>& values) {
    return std::all_of(values.begin(), values.end(), [](double value) {
        return std::isfinite(value);
        });
}

template<typename T>
tl::expected<T, MotorBusErr> require_as(const YAML::Node& parent, const char* key) {
    const YAML::Node node = parent[key];
    if(!node) return tl::make_unexpected(MotorBusErr::INVALID_CFG);
    try {
        return node.as<T>();
    }
    catch(const YAML::BadConversion&) {
        return tl::make_unexpected(MotorBusErr::INVALID_CFG);
    }
}

} // namespace

// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

/**
 * @brief 配置 DamiaoMotorBus
 * @param config_path 配置文件路径
 * @return 如果配置成功，则返回空的 tl::expected，否则返回错误码
 */
tl::expected<void, MotorBusErr> DamiaoMotorBus::configure(const std::string& config_path) {
    try {
        const YAML::Node root = YAML::LoadFile(config_path);
        const YAML::Node damiao = root["damiao"] ? root["damiao"] : root;
        if(!damiao || !damiao.IsMap()) return tl::make_unexpected(MotorBusErr::INVALID_CFG);

        DamiaoBusCfg cfg;
        auto bus = damiao["bus"] ? tl::expected<std::string, MotorBusErr>{ damiao["bus"].as<std::string>() } :
            tl::expected<std::string, MotorBusErr>{ cfg.bus };
        auto serial_port = require_as<std::string>(damiao, "serial_port");
        auto baudrate = require_as<int>(damiao, "baudrate");
        auto refresh_state_in_read = require_as<bool>(damiao, "refresh_state_in_read");
        auto feedback_timeout_s = require_as<double>(damiao, "feedback_timeout_s");
        auto activation_retries = require_as<std::size_t>(damiao, "activation_retries");
        auto startup_read_cycles = require_as<std::size_t>(damiao, "startup_read_cycles");
        auto stop_kp = require_as<double>(damiao, "stop_kp");
        auto stop_kd = require_as<double>(damiao, "stop_kd");
        auto stop_cycles = require_as<std::size_t>(damiao, "stop_cycles");
        if(!bus || !serial_port || !baudrate || !refresh_state_in_read || !feedback_timeout_s ||
            !activation_retries || !startup_read_cycles || !stop_kp || !stop_kd || !stop_cycles) {
            return tl::make_unexpected(MotorBusErr::INVALID_CFG);
        }
        cfg.bus = *bus;
        cfg.serial_port = *serial_port;
        cfg.baudrate = *baudrate;
        cfg.refresh_state_in_read = *refresh_state_in_read;
        cfg.feedback_timeout_s = *feedback_timeout_s;
        cfg.activation_retries = *activation_retries;
        cfg.startup_read_cycles = *startup_read_cycles;
        cfg.stop_kp = *stop_kp;
        cfg.stop_kd = *stop_kd;
        cfg.stop_cycles = *stop_cycles;

        const YAML::Node actuators = damiao["actuators"];
        if(!actuators || !actuators.IsMap()) return tl::make_unexpected(MotorBusErr::INVALID_CFG);
        for(const auto& item : actuators) {
            if(!item.second.IsMap()) return tl::make_unexpected(MotorBusErr::INVALID_CFG);
            DamiaoActuatorCfg actuator;
            actuator.joint_name = item.first.as<std::string>();
            auto name = require_as<std::string>(item.second, "name");
            auto motor_id = require_as<std::uint32_t>(item.second, "motor_id");
            auto master_id = require_as<std::uint32_t>(item.second, "master_id");
            auto motor_type = require_as<std::string>(item.second, "motor_type");
            if(!name || !motor_id || !master_id || !motor_type) return tl::make_unexpected(MotorBusErr::INVALID_CFG);
            actuator.name = *name;
            actuator.motor_id = *motor_id;
            actuator.master_id = *master_id;
            actuator.motor_type = *motor_type;
            cfg.actuators.push_back(std::move(actuator));
        }
        return configure(cfg);
    }
    catch(const YAML::Exception&) {
        return tl::make_unexpected(MotorBusErr::INVALID_CFG);
    }
}

/**
 * @brief 配置 DamiaoMotorBus
 * @param cfg 配置参数
 * @return 如果配置成功，则返回空的 tl::expected，否则返回错误码
 */
tl::expected<void, MotorBusErr> DamiaoMotorBus::configure(const DamiaoBusCfg& cfg) {
    cleanup();
    auto valid = validate_cfg(cfg);
    if(!valid) return tl::make_unexpected(valid.error());
    cfg_ = cfg;
    actuator_info_.clear();
    capabilities_.clear();
    actuator_info_.reserve(cfg_.actuators.size());
    capabilities_.reserve(cfg_.actuators.size());
    for(const auto& actuator : cfg_.actuators) {
        const auto type = parse_motor_type(actuator.motor_type);
        if(!type) return tl::make_unexpected(type.error());

        const damiao::LimitParam limit = damiao::limit_param[*type];
        DamiaoActuatorInfo info;
        info.name = actuator.name;
        info.joint_name = actuator.joint_name;
        info.motor_id = actuator.motor_id;
        info.master_id = actuator.master_id;
        info.motor_type = actuator.motor_type;
        info.q_max = limit.q_max;
        info.dq_max = limit.dq_max;
        info.tau_max = limit.tau_max;
        actuator_info_.push_back(std::move(info));

        ActuatorCapability capability;
        capability.actuator_name = actuator.name;
        capability.min_pos = -static_cast<double>(limit.q_max);
        capability.max_pos = static_cast<double>(limit.q_max);
        capability.max_vel = static_cast<double>(limit.dq_max);
        capability.max_effort = static_cast<double>(limit.tau_max);
        capability.max_kp = 500.0;
        capability.max_kd = 5.0;
        capabilities_.push_back(std::move(capability));
    }
    configured_ = true;
    return {};
}

/**
 * @brief 连接 DamiaoMotorBus
 * @return 如果连接成功，则返回空的 tl::expected，否则返回错误码
 */
tl::expected<void, MotorBusErr> DamiaoMotorBus::connect() {
    if(!configured_) return tl::make_unexpected(MotorBusErr::NOT_CONFIGURED);
    if(connected_) return {};

    try {
        protocol::damiao_usb2can::Config usb2can_cfg;
        usb2can_cfg.serial_port = cfg_.serial_port;
        usb2can_cfg.baudrate = cfg_.baudrate;
        std::vector<transport::CanFilter> filters;
        filters.reserve(cfg_.actuators.size() * 2);
        for(const auto& actuator : cfg_.actuators) {
            filters.push_back(transport::CanFilter{ actuator.motor_id, 0x7FF });
            filters.push_back(transport::CanFilter{ actuator.master_id, 0x7FF });
        }
        auto channel = protocol::damiao_usb2can::acquire_channel(cfg_.bus, usb2can_cfg, std::move(filters));
        if(!channel) {
            if(channel.error() == protocol::damiao_usb2can::Err::CONFIG_CONFLICT ||
                channel.error() == protocol::damiao_usb2can::Err::TYPE_MISMATCH) {
                return tl::make_unexpected(MotorBusErr::INVALID_CFG);
            }
            return tl::make_unexpected(MotorBusErr::OPEN_FAILED);
        }
        can_channel_ = *channel;
        motor_ctrl_ = std::make_shared<damiao::MotorControl>(can_channel_);
        motors_.clear();
        motors_.reserve(cfg_.actuators.size());

        for(const auto& actuator : cfg_.actuators) {
            auto type = parse_motor_type(actuator.motor_type);
            if(!type) {
                motors_.clear();
                motor_ctrl_.reset();
                can_channel_.reset();
                connected_ = false;
                return tl::make_unexpected(type.error());
            }
            auto motor = std::make_shared<damiao::Motor>(*type, actuator.motor_id, actuator.master_id);
            motor_ctrl_->add_motor(motor.get());
            motors_.push_back(std::move(motor));
        }

        const std::size_t n = motors_.size();
        online_.assign(n, 0);
        enabled_.assign(n, 0);
        has_feedback_.assign(n, 0);
        last_feedback_time_.assign(n, TimePoint{});
        last_state_.pos.assign(n, 0.0);
        last_state_.vel.assign(n, 0.0);
        last_state_.tor.assign(n, 0.0);
        last_state_.online.assign(n, 0);
        last_state_.enabled.assign(n, 0);
        last_state_.err_code.assign(n, 0);
        connected_ = true;
        return {};
    }
    catch(...) {
        motors_.clear();
        motor_ctrl_.reset();
        can_channel_.reset();
        online_.clear();
        enabled_.clear();
        has_feedback_.clear();
        last_feedback_time_.clear();
        last_state_ = ActuatorState{};
        connected_ = false;
        active_ = false;
        return tl::make_unexpected(MotorBusErr::OPEN_FAILED);
    }
}

/**
 * @brief 读取 DamiaoMotorBus 的状态
 * @param refresh 是否刷新状态
 * @return 如果读取成功，则返回 ActuatorState，否则返回错误码
 */
tl::expected<ActuatorState, MotorBusErr> DamiaoMotorBus::read_impl(bool refresh) {
    if(!connected_ || !motor_ctrl_) return tl::make_unexpected(MotorBusErr::NOT_CONNECTED);

    try {
        ActuatorState state = last_state_;
        std::vector<std::uint64_t> previous_seq;
        previous_seq.reserve(motors_.size());
        for(const auto& motor : motors_) previous_seq.push_back(motor->get_state_seq());

        if(refresh) {
            for(std::size_t i = 0; i < motors_.size(); ++i) motor_ctrl_->refresh_motor_status(*motors_[i]);
        }
        else {
            for(std::size_t i = 0; i < motors_.size(); ++i) motor_ctrl_->receive();
        }

        const TimePoint now = Clock::now();
        for(std::size_t i = 0; i < motors_.size(); ++i) {
            if(motors_[i]->get_state_seq() != previous_seq[i]) {
                has_feedback_[i] = 1;
                last_feedback_time_[i] = now;
            }

            double feedback_age_s = std::numeric_limits<double>::infinity();
            if(has_feedback_[i]) feedback_age_s = std::chrono::duration<double>(now - last_feedback_time_[i]).count();
            online_[i] = feedback_age_s <= cfg_.feedback_timeout_s ? 1 : 0;
            state.pos[i] = motors_[i]->get_position();
            state.vel[i] = motors_[i]->get_velocity();
            state.tor[i] = motors_[i]->get_tau();
            state.online[i] = online_[i];
            state.enabled[i] = enabled_[i];
            state.err_code[i] = online_[i] ? 0 : -1;
        }

        if(!finite_vector(state.pos) || !finite_vector(state.vel) || !finite_vector(state.tor)) {
            return tl::make_unexpected(MotorBusErr::INVALID_STATE);
        }

        last_state_ = state;
        return state;
    }
    catch(...) {
        return tl::make_unexpected(MotorBusErr::READ_FAILED);
    }
}

/**
 * @brief 读取 DamiaoMotorBus 的状态
 * @return 如果读取成功，则返回 ActuatorState，否则返回错误码
 */
tl::expected<ActuatorState, MotorBusErr> DamiaoMotorBus::read() {
    return read_impl(cfg_.refresh_state_in_read);
}

/**
 * @brief 激活 DamiaoMotorBus
 * @return 如果激活成功，则返回空的 tl::expected，否则返回错误码
 */
tl::expected<void, MotorBusErr> DamiaoMotorBus::activate() {
    if(!connected_ || !motor_ctrl_ || !can_channel_) {
        return tl::make_unexpected(MotorBusErr::NOT_CONNECTED);
    }
    if(active_) return {};

    try {
        if(can_channel_) can_channel_->flush();
        std::fill(enabled_.begin(), enabled_.end(), 0);

        for(std::size_t i = 0; i < motors_.size(); ++i) {
            const auto result = activate_motor(i);
            if(!result) {
                disable_enabled_noexcept();
                return result;
            }
        }

        active_ = true;

        ActuatorState average = last_state_;
        std::fill(average.pos.begin(), average.pos.end(), 0.0);
        std::fill(average.vel.begin(), average.vel.end(), 0.0);
        std::fill(average.tor.begin(), average.tor.end(), 0.0);

        for(std::size_t cycle = 0; cycle < cfg_.startup_read_cycles; ++cycle) {
            auto sample = read_impl(true);
            if(!sample || std::any_of(sample->online.begin(), sample->online.end(),
                [](std::uint8_t value) { return value == 0; })) {
                disable_enabled_noexcept();
                return tl::make_unexpected(MotorBusErr::ACTUATOR_OFFLINE);
            }
            for(std::size_t i = 0; i < motors_.size(); ++i) {
                average.pos[i] += sample->pos[i];
                average.vel[i] += sample->vel[i];
                average.tor[i] += sample->tor[i];
            }
        }

        for(std::size_t i = 0; i < motors_.size(); ++i) {
            average.pos[i] /= static_cast<double>(cfg_.startup_read_cycles);
            average.vel[i] /= static_cast<double>(cfg_.startup_read_cycles);
            average.tor[i] /= static_cast<double>(cfg_.startup_read_cycles);
            average.online[i] = online_[i];
            average.enabled[i] = enabled_[i];
            average.err_code[i] = 0;
        }
        last_state_ = average;

        auto stopped = stop();
        if(!stopped) {
            disable_enabled_noexcept();
            return tl::make_unexpected(MotorBusErr::STOP_FAILED);
        }
        return {};
    }
    catch(...) {
        disable_enabled_noexcept();
        return tl::make_unexpected(MotorBusErr::ENABLE_FAILED);
    }
}

/**
 * @brief 写入 DamiaoMotorBus 的命令
 * @param cmd 待写入的命令
 * @return 如果写入成功，则返回空的 tl::expected，否则返回错误码
 */
tl::expected<void, MotorBusErr> DamiaoMotorBus::write(const ActuatorCtrlCmd& cmd) {
    if(!active_) return tl::make_unexpected(MotorBusErr::NOT_ACTIVE);
    auto valid = validate_cmd(cmd);
    if(!valid) return tl::make_unexpected(valid.error());

    try {
        for(std::size_t i = 0; i < motors_.size(); ++i) {
            if(!motor_ctrl_->control_mit(*motors_[i],
                static_cast<float>(cmd.kp[i]),
                static_cast<float>(cmd.kd[i]),
                static_cast<float>(cmd.pos[i]),
                static_cast<float>(cmd.vel[i]),
                static_cast<float>(cmd.tor[i]),
                false)) {
                return tl::make_unexpected(MotorBusErr::WRITE_FAILED);
            }
        }
        return {};
    }
    catch(...) {
        return tl::make_unexpected(MotorBusErr::WRITE_FAILED);
    }
}

/**
 * @brief 停止 DamiaoMotorBus 的运动
 * @return 如果停止成功，则返回空的 tl::expected，否则返回错误码
 */
tl::expected<void, MotorBusErr> DamiaoMotorBus::stop() {
    if(!active_) return tl::make_unexpected(MotorBusErr::NOT_ACTIVE);
    if(last_state_.pos.size() != motors_.size() || !finite_vector(last_state_.pos)) {
        auto state = read_impl(true);
        if(!state) return tl::make_unexpected(state.error());
    }

    ActuatorCtrlCmd stop_cmd;
    stop_cmd.pos = last_state_.pos;
    stop_cmd.vel.assign(motors_.size(), 0.0);
    stop_cmd.tor.assign(motors_.size(), 0.0);
    stop_cmd.kp.assign(motors_.size(), cfg_.stop_kp);
    stop_cmd.kd.assign(motors_.size(), cfg_.stop_kd);

    for(std::size_t cycle = 0; cycle < cfg_.stop_cycles; ++cycle) {
        auto result = write(stop_cmd);
        if(!result) return result;
    }
    return {};
}

/**
 * @brief 停用 DamiaoMotorBus
 * @return 如果停用成功，则返回空的 tl::expected，否则返回错误码
 */
tl::expected<void, MotorBusErr> DamiaoMotorBus::deactivate() {
    if(!connected_ || !motor_ctrl_) {
        return tl::make_unexpected(MotorBusErr::NOT_CONNECTED);
    }

    bool stop_failed = false;
    if(active_) {
        const auto stopped = stop();
        stop_failed = !stopped;
    }

    bool disable_failed = false;
    try {
        for(std::size_t i = 0; i < motors_.size(); ++i) {
            if(!motor_ctrl_->disable(*motors_[i])) {
                disable_failed = true;
            }
            if(i < enabled_.size()) enabled_[i] = 0;
        }
        if(can_channel_) can_channel_->flush();
    }
    catch(...) {
        disable_failed = true;
    }

    active_ = false;
    last_state_.enabled = enabled_;

    if(disable_failed) {
        return tl::make_unexpected(MotorBusErr::DISABLE_FAILED);
    }
    if(stop_failed) {
        return tl::make_unexpected(MotorBusErr::STOP_FAILED);
    }
    return {};
}

/**
 * @brief 在 FAULT 后重建达妙通信状态
 * @return 如果恢复成功，则返回空的 tl::expected，否则返回错误码
 */
tl::expected<void, MotorBusErr> DamiaoMotorBus::recover() {
    if(!configured_) {
        return tl::make_unexpected(MotorBusErr::NOT_CONFIGURED);
    }

    release_connection_noexcept(true);

    const auto connected = connect();
    if(!connected) {
        return tl::make_unexpected(connected.error());
    }

    try {
        for(std::size_t i = 0; i < motors_.size(); ++i) {
            if(!motor_ctrl_->disable(*motors_[i])) {
                release_connection_noexcept(true);
                return tl::make_unexpected(MotorBusErr::DISABLE_FAILED);
            }
            enabled_[i] = 0;
        }
        if(can_channel_) can_channel_->flush();
        active_ = false;
        last_state_.enabled = enabled_;
        return {};
    }
    catch(...) {
        release_connection_noexcept(true);
        return tl::make_unexpected(MotorBusErr::RECOVER_FAILED);
    }
}

/**
 * @brief 清理 DamiaoMotorBus 的资源
 */
void DamiaoMotorBus::cleanup() noexcept {
    release_connection_noexcept(false);
}

/**
 * @brief 获取 DamiaoMotorBus 的电机数量
 * @return 电机数量
 */
std::size_t DamiaoMotorBus::size() const noexcept {
    return motors_.empty() ? cfg_.actuators.size() : motors_.size();
}

/**
 * @brief 获取达妙执行器静态信息
 * @return 达妙执行器静态信息只读引用
 */
const std::vector<DamiaoActuatorInfo>& DamiaoMotorBus::get_actuator_info() const noexcept {
    return actuator_info_;
}

/**
 * @brief 获取 HardwareCapabilities
 * @return HardwareCapabilities 只读引用
 */
const HardwareCapabilities& DamiaoMotorBus::capabilities() const noexcept {
    return capabilities_;
}

// ! ========================= 私 有 类 方 法 实 现 ========================= ! //

/**
 * @brief 以可重试流程准备并使能单个电机
 * @param index 电机索引
 * @return 如果准备成功，则返回空的 tl::expected，否则返回错误码
 */
tl::expected<void, MotorBusErr> DamiaoMotorBus::activate_motor(std::size_t index) {
    if(index >= motors_.size() || !motor_ctrl_ || !can_channel_) return tl::make_unexpected(MotorBusErr::NOT_CONNECTED);

    bool disable_sent = false;
    bool enable_sent = false;
    for(std::size_t attempt = 0; attempt < cfg_.activation_retries; ++attempt) {
        if(can_channel_) can_channel_->flush();
        disable_sent = motor_ctrl_->disable(*motors_[index]);
        enabled_[index] = 0;
        if(!disable_sent) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        enable_sent = motor_ctrl_->enable(*motors_[index]);
        if(!enable_sent) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        enabled_[index] = 1;

        if(motor_ctrl_->switch_control_mode(*motors_[index], damiao::MIT_MODE)) return {};

        (void)motor_ctrl_->disable(*motors_[index]);
        enabled_[index] = 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if(!disable_sent) return tl::make_unexpected(MotorBusErr::DISABLE_FAILED);
    if(!enable_sent) return tl::make_unexpected(MotorBusErr::ENABLE_FAILED);
    return tl::make_unexpected(MotorBusErr::MODE_SWITCH_FAILED);
}

/**
 * @brief 验证 DamiaoMotorBus 的配置参数
 * @param cfg 配置参数
 * @return 如果配置参数有效，则返回空的 tl::expected，否则返回错误码
 */
tl::expected<void, MotorBusErr> DamiaoMotorBus::validate_cfg(const DamiaoBusCfg& cfg) const {
    if(cfg.bus.empty() || cfg.serial_port.empty() || cfg.baudrate <= 0 || cfg.actuators.empty() ||
        cfg.activation_retries == 0 || cfg.startup_read_cycles == 0 || cfg.stop_cycles == 0 ||
        !std::isfinite(cfg.feedback_timeout_s) || cfg.feedback_timeout_s <= 0.0 ||
        !std::isfinite(cfg.stop_kp) || !std::isfinite(cfg.stop_kd) ||
        cfg.stop_kp < 0.0 || cfg.stop_kp > 500.0 || cfg.stop_kd < 0.0 || cfg.stop_kd > 5.0) {
        return tl::make_unexpected(MotorBusErr::INVALID_CFG);
    }

    std::vector<std::uint32_t> motor_ids;
    motor_ids.reserve(cfg.actuators.size());
    for(const auto& actuator : cfg.actuators) {
        if(actuator.name.empty() || actuator.joint_name.empty() || actuator.motor_id == 0 || !parse_motor_type(actuator.motor_type)) {
            return tl::make_unexpected(MotorBusErr::INVALID_CFG);
        }
        if(std::find(motor_ids.begin(), motor_ids.end(), actuator.motor_id) != motor_ids.end()) {
            return tl::make_unexpected(MotorBusErr::INVALID_CFG);
        }
        motor_ids.push_back(actuator.motor_id);
    }
    return {};
}

/**
 * @brief 验证 DamiaoMotorBus 的命令参数
 * @param cmd 命令参数
 * @return 如果命令参数有效，则返回空的 tl::expected，否则返回错误码
 */
tl::expected<void, MotorBusErr> DamiaoMotorBus::validate_cmd(const ActuatorCtrlCmd& cmd) const {
    const std::size_t n = motors_.size();
    if(cmd.pos.size() != n || cmd.vel.size() != n || cmd.tor.size() != n ||
        cmd.kp.size() != n || cmd.kd.size() != n ||
        !finite_vector(cmd.pos) || !finite_vector(cmd.vel) || !finite_vector(cmd.tor) ||
        !finite_vector(cmd.kp) || !finite_vector(cmd.kd)) {
        return tl::make_unexpected(MotorBusErr::INVALID_CMD);
    }

    constexpr double epsilon = 1e-9;
    for(std::size_t i = 0; i < n; ++i) {
        const auto limit = motors_[i]->get_limit_param();
        if(cmd.kp[i] < 0.0 || cmd.kp[i] > 500.0 + epsilon ||
            cmd.kd[i] < 0.0 || cmd.kd[i] > 5.0 + epsilon ||
            std::abs(cmd.pos[i]) > static_cast<double>(limit.q_max) + epsilon ||
            std::abs(cmd.vel[i]) > static_cast<double>(limit.dq_max) + epsilon ||
            std::abs(cmd.tor[i]) > static_cast<double>(limit.tau_max) + epsilon) {
            return tl::make_unexpected(MotorBusErr::INVALID_CMD);
        }
    }
    return {};
}

/**
 * @brief 解析 DamiaoMotorBus 的电机类型
 * @param value 电机类型字符串
 * @return 如果解析成功，则返回 damiao::DmMotorType，否则返回错误码
 */
tl::expected<damiao::DmMotorType, MotorBusErr> DamiaoMotorBus::parse_motor_type(const std::string& value) const {
    if(value == "DM4310") return damiao::DM4310;
    if(value == "DM4310_48V") return damiao::DM4310_48V;
    if(value == "DM4340") return damiao::DM4340;
    if(value == "DM4340_48V") return damiao::DM4340_48V;
    if(value == "DM6006") return damiao::DM6006;
    if(value == "DM6248P") return damiao::DM6248P;
    if(value == "DM8006") return damiao::DM8006;
    if(value == "DM8009") return damiao::DM8009;
    if(value == "DM10010L") return damiao::DM10010L;
    if(value == "DM10010") return damiao::DM10010;
    if(value == "DMH3510") return damiao::DMH3510;
    if(value == "DMH6215") return damiao::DMH6215;
    if(value == "DMG6220") return damiao::DMG6220;
    if(value == "DMJH11") return damiao::DMJH11;
    return tl::make_unexpected(MotorBusErr::INVALID_CFG);
}

/**
 * @brief 失能已经使能的电机，忽略异常
 */
void DamiaoMotorBus::disable_enabled_noexcept() noexcept {
    if(!motor_ctrl_) {
        active_ = false;
        std::fill(enabled_.begin(), enabled_.end(), 0);
        return;
    }
    try {
        for(std::size_t i = 0; i < motors_.size(); ++i) {
            (void)motor_ctrl_->disable(*motors_[i]);
            if(i < enabled_.size()) enabled_[i] = 0;
        }
        if(can_channel_) can_channel_->flush();
    }
    catch(...) {
    }
    active_ = false;
}

/**
 * @brief 释放连接资源
 * @param keep_config 是否保留配置
 */
void DamiaoMotorBus::release_connection_noexcept(bool keep_config) noexcept {
    disable_enabled_noexcept();
    motors_.clear();
    motor_ctrl_.reset();
    can_channel_.reset();
    online_.clear();
    enabled_.clear();
    has_feedback_.clear();
    last_feedback_time_.clear();
    last_state_ = ActuatorState{};
    connected_ = false;
    active_ = false;
    if(!keep_config) {
        cfg_ = DamiaoBusCfg{};
        actuator_info_.clear();
        configured_ = false;
    }
}

} // namespace serial_arm

extern "C" serial_arm::MotorBus* create_motor_bus() {
    return new serial_arm::DamiaoMotorBus();
}

extern "C" void destroy_motor_bus(serial_arm::MotorBus* bus) {
    delete bus;
}
