#include "dm_arm/hardware/damiao_motor_bus.hpp"

#include "dm_hw/serial_port.hpp"

namespace dm_arm {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //

namespace {

/**
 * @brief 波特率转换为 speed_t 类型
 * @param baudrate 波特率
 * @return speed_t 对应的 speed_t 类型值，如果不支持则返回 0
 */
speed_t to_speed_t(int baudrate) {
    switch(baudrate) {
        case 115200: return B115200;
#ifdef B460800
        case 460800: return B460800;
#endif
#ifdef B921600
        case 921600: return B921600;
#endif
        default: return 0;
    }
}

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

} // namespace

// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

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
        const speed_t speed = to_speed_t(cfg_.baudrate);
        if(speed == 0) return tl::make_unexpected(MotorBusErr::OPEN_FAILED);

        serial_ = std::make_shared<SerialPort>(cfg_.serial_port, speed);
        motor_ctrl_ = std::make_shared<damiao::MotorControl>(serial_);
        motors_.clear();
        motors_.reserve(cfg_.actuators.size());

        for(const auto& actuator : cfg_.actuators) {
            auto type = parse_motor_type(actuator.motor_type);
            if(!type) {
                motors_.clear();
                motor_ctrl_.reset();
                serial_.reset();
                connected_ = false;
                return tl::make_unexpected(type.error());
            }
            auto motor = std::make_shared<damiao::Motor>(
                *type, actuator.motor_id, actuator.master_id);
            motor_ctrl_->add_motor(motor.get());
            motors_.push_back(std::move(motor));
        }

        const std::size_t n = motors_.size();
        online_.assign(n, 0);
        enabled_.assign(n, 0);
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
        serial_.reset();
        online_.clear();
        enabled_.clear();
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
        if(!refresh) {
            std::vector<std::uint64_t> previous_seq;
            previous_seq.reserve(motors_.size());
            for(const auto& motor : motors_) previous_seq.push_back(motor->get_state_seq());

            for(std::size_t i = 0; i < motors_.size(); ++i) motor_ctrl_->receive();

            for(std::size_t i = 0; i < motors_.size(); ++i) {
                online_[i] = motors_[i]->get_state_seq() != previous_seq[i] ? 1 : 0;
            }
        }
        for(std::size_t i = 0; i < motors_.size(); ++i) {
            if(refresh) online_[i] = motor_ctrl_->refresh_motor_status(*motors_[i]) ? 1 : 0;

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
    if(!connected_ || !motor_ctrl_) return tl::make_unexpected(MotorBusErr::NOT_CONNECTED);
    if(active_) return {};

    try {
        for(std::size_t i = 0; i < motors_.size(); ++i) {
            if(!motor_ctrl_->enable(*motors_[i])) {
                disable_enabled_noexcept();
                return tl::make_unexpected(MotorBusErr::WRITE_FAILED);
            }
            enabled_[i] = 1;
            if(!motor_ctrl_->switch_control_mode(*motors_[i], damiao::MIT_MODE)) {
                disable_enabled_noexcept();
                return tl::make_unexpected(MotorBusErr::WRITE_FAILED);
            }
        }

        active_ = true;

        ActuatorState average = last_state_;
        std::fill(average.pos.begin(), average.pos.end(), 0.0);
        std::fill(average.vel.begin(), average.vel.end(), 0.0);
        std::fill(average.tor.begin(), average.tor.end(), 0.0);

        for(std::size_t cycle = 0; cycle < cfg_.startup_read_cycles; ++cycle) {
            auto sample = read_impl(true);
            if(!sample || std::any_of(sample->online.begin(), sample->online.end(), [](std::uint8_t value) { return value == 0; })) {
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
            return tl::make_unexpected(stopped.error());
        }
        return {};
    }
    catch(...) {
        disable_enabled_noexcept();
        return tl::make_unexpected(MotorBusErr::WRITE_FAILED);
    }
}

/**
 * @brief 写入 DamiaoMotorBus 的命令
 * @param cmd 待写入的命令
 * @return 如果写入成功，则返回空的 tl::expected，否则返回错误码
 */
tl::expected<void, MotorBusErr> DamiaoMotorBus::write(const ActuatorMitCmd& cmd) {
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

    ActuatorMitCmd stop_cmd;
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
    if(!connected_) return tl::make_unexpected(MotorBusErr::NOT_CONNECTED);

    MotorBusErr first_error = MotorBusErr::WRITE_FAILED;
    bool failed = false;
    if(active_) {
        auto stopped = stop();
        if(!stopped) {
            first_error = stopped.error();
            failed = true;
        }
    }

    try {
        for(std::size_t i = 0; i < motors_.size(); ++i) {
            if(enabled_[i] && !motor_ctrl_->disable(*motors_[i])) {
                if(!failed) first_error = MotorBusErr::WRITE_FAILED;
                failed = true;
            }
            enabled_[i] = 0;
        }
    }
    catch(...) {
        if(!failed) first_error = MotorBusErr::WRITE_FAILED;
        failed = true;
    }

    active_ = false;
    last_state_.enabled = enabled_;
    if(failed) return tl::make_unexpected(first_error);
    return {};
}

/**
 * @brief 清理 DamiaoMotorBus 的资源
 */
void DamiaoMotorBus::cleanup() noexcept {
    disable_enabled_noexcept();
    motors_.clear();
    motor_ctrl_.reset();
    serial_.reset();
    online_.clear();
    enabled_.clear();
    last_state_ = ActuatorState{};
    connected_ = false;
    active_ = false;
    configured_ = false;
}

/**
 * @brief 获取 DamiaoMotorBus 的电机数量
 * @return 电机数量
 */
std::size_t DamiaoMotorBus::size() const noexcept {
    return motors_.empty() ? cfg_.actuators.size() : motors_.size();
}

// ! ========================= 私 有 类 方 法 实 现 ========================= ! //

/**
 * @brief 验证 DamiaoMotorBus 的配置参数
 * @param cfg 配置参数
 * @return 如果配置参数有效，则返回空的 tl::expected，否则返回错误码
 */
tl::expected<void, MotorBusErr> DamiaoMotorBus::validate_cfg(const DamiaoBusCfg& cfg) const {
    if(cfg.serial_port.empty() || to_speed_t(cfg.baudrate) == 0 || cfg.actuators.empty() ||
        cfg.startup_read_cycles == 0 || cfg.stop_cycles == 0 ||
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
tl::expected<void, MotorBusErr> DamiaoMotorBus::validate_cmd(const ActuatorMitCmd& cmd) const {
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
        return;
    }
    try {
        for(std::size_t i = 0; i < motors_.size() && i < enabled_.size(); ++i) {
            if(enabled_[i]) motor_ctrl_->disable(*motors_[i]);
            enabled_[i] = 0;
        }
    }
    catch(...) {
    }
    active_ = false;
}

} // namespace dm_arm
