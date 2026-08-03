#include "serial_arm/robot.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace serial_arm {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //

namespace {

/**
 * @brief 判断向量中的所有元素是否均为有限值
 * @param values 待检查向量
 * @return 所有元素均有限时返回 true，否则返回 false
 */
bool finite_vector(const JointVector& values) {
    return std::all_of(values.begin(), values.end(), [](double value) {
        return std::isfinite(value);
        });
}

} // namespace

// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

/**
 * @brief 析构时尽力安全停机并释放硬件资源
 */
Robot::~Robot() {
    if(motor_bus_) {
        disable_noexcept();
        motor_bus_->cleanup();
    }
}

/**
 * @brief 配置 Robot 并接管 MotorBus 所有权
 * @param cfg Robot 完整配置
 * @param motor_bus 待接管的 MotorBus 实例
 * @param model_feedforward 可选模型前馈函数
 * @return 配置成功返回空 expected，失败返回 RobotFault
 */
tl::expected<void, RobotFault> Robot::configure(const RobotCfg& cfg, std::unique_ptr<MotorBus> motor_bus, ModelFeedforwardFn model_feedforward) {
    if(state_ != RobotState::UNCONFIGURED) {
        return tl::make_unexpected(make_fault(RobotErr::ALREADY_CONFIGURED));
    }
    if(!motor_bus) {
        return tl::make_unexpected(make_fault(RobotErr::NULL_MOTOR_BUS));
    }

    const auto valid_cfg = validate_robot_core_cfg(cfg);
    if(!valid_cfg) {
        return tl::make_unexpected(make_fault(RobotErr::INVALID_CFG));
    }
    if(motor_bus->size() != cfg.joint_names.size()) {
        return tl::make_unexpected(make_fault(RobotErr::MOTOR_BUS_SIZE_MISMATCH));
    }
    if(cfg.runtime.model_feedforward_mode != ModelFeedforwardMode::NONE && !model_feedforward) {
        RobotFault fault = make_model_fault(ModelFeedforwardErr::NOT_CONFIGURED);
        fault.code = RobotErr::INVALID_CFG;
        return tl::make_unexpected(fault);
    }

    const auto ctrller_result = ctrller_.configure(cfg.ctrller);
    if(!ctrller_result) {
        return tl::make_unexpected(make_ctrller_fault(ctrller_result.error()));
    }
    const auto mapper_result = mapper_.configure(cfg.mapper);
    if(!mapper_result) {
        return tl::make_unexpected(make_mapper_fault(mapper_result.error()));
    }
    const auto safety_result = safety_.configure(cfg.safety);
    if(!safety_result) {
        return tl::make_unexpected(make_safety_fault(safety_result.error()));
    }

    cfg_ = cfg;
    motor_bus_ = std::move(motor_bus);
    model_feedforward_ = std::move(model_feedforward);
    last_fault_.reset();
    clear_runtime_state();
    state_ = RobotState::INACTIVE;
    return {};
}

/**
 * @brief 连接、使能并用真实状态初始化控制器
 * @return 成功返回空 expected，失败返回 RobotFault
 */
tl::expected<void, RobotFault> Robot::activate() {
    if(state_ == RobotState::UNCONFIGURED) {
        return tl::make_unexpected(make_fault(RobotErr::NOT_CONFIGURED));
    }
    if(state_ == RobotState::FAULT) {
        return tl::make_unexpected(make_fault(RobotErr::FAULTED));
    }
    if(state_ == RobotState::ACTIVE) {
        return tl::make_unexpected(make_fault(RobotErr::ALREADY_ACTIVE));
    }
    if(!cfg_.runtime.write_enabled) {
        return tl::make_unexpected(make_fault(RobotErr::WRITE_DISABLED));
    }

    const auto connected = motor_bus_->connect();
    if(!connected) {
        const RobotFault fault = make_bus_fault(RobotErr::MOTOR_BUS_CONNECT_FAILED, connected.error());
        enter_fault(fault, SafetyAction::DISABLE);
        return tl::make_unexpected(fault);
    }

    const auto active = motor_bus_->activate();
    if(!active) {
        const RobotFault fault = make_bus_fault(RobotErr::MOTOR_BUS_ACTIVATE_FAILED, active.error());
        enter_fault(fault, SafetyAction::DISABLE);
        return tl::make_unexpected(fault);
    }

    const auto actuator_state = motor_bus_->read();
    if(!actuator_state) {
        const RobotFault fault = make_bus_fault(RobotErr::MOTOR_BUS_READ_FAILED, actuator_state.error());
        enter_fault(fault, SafetyAction::DISABLE);
        return tl::make_unexpected(fault);
    }

    const auto joint_state = mapper_.to_joint_state(actuator_state.value());
    if(!joint_state) {
        const RobotFault fault = make_mapper_fault(joint_state.error());
        enter_fault(fault, SafetyAction::DISABLE);
        return tl::make_unexpected(fault);
    }

    const auto checked_state = safety_.check_state(joint_state.value(), actuator_state.value(), 0.0);
    if(!checked_state) {
        const RobotFault fault = make_safety_fault(checked_state.error());
        enter_fault(fault, safety_.action_for(checked_state.error().code));
        return tl::make_unexpected(fault);
    }

    ctrller_.reset();
    const auto initialized = ctrller_.initialize(joint_state.value());
    if(!initialized) {
        const RobotFault fault = make_ctrller_fault(initialized.error());
        enter_fault(fault, SafetyAction::DISABLE);
        return tl::make_unexpected(fault);
    }

    const auto history = safety_.reset_cmd_history(joint_state.value());
    if(!history) {
        const RobotFault fault = make_safety_fault(history.error());
        enter_fault(fault, SafetyAction::DISABLE);
        return tl::make_unexpected(fault);
    }

    actuator_state_ = actuator_state.value();
    joint_state_ = joint_state.value();
    joint_acc_.assign(cfg_.joint_names.size(), 0.0);
    joint_ref_acc_.assign(cfg_.joint_names.size(), 0.0);
    model_feedforward_cache_.assign(cfg_.joint_names.size(), 0.0);

    last_joint_cmd_.pos = joint_state_.pos;
    last_joint_cmd_.vel.assign(cfg_.joint_names.size(), 0.0);
    last_joint_cmd_.tor.assign(cfg_.joint_names.size(), 0.0);
    last_joint_cmd_.kp.assign(cfg_.joint_names.size(), 0.0);
    last_joint_cmd_.kd.assign(cfg_.joint_names.size(), 0.0);

    has_state_ = true;
    has_completed_cycle_ = false;
    has_external_cmd_ = false;
    has_last_joint_cmd_ = true;

    const TimePoint activated_at = Clock::now();
    last_cycle_time_ = activated_at;
    last_state_time_ = activated_at;
    last_cmd_time_ = activated_at;

    last_fault_.reset();
    state_ = RobotState::ACTIVE;
    return {};
}

/**
 * @brief 设置跟踪参考命令
 */
tl::expected<void, RobotFault> Robot::set_cmd(const JointCmd& cmd, TimePoint now) {
    if(state_ == RobotState::FAULT) return tl::make_unexpected(make_fault(RobotErr::FAULTED));
    if(state_ != RobotState::ACTIVE) return tl::make_unexpected(make_fault(RobotErr::NOT_ACTIVE));
    if(now < last_cycle_time_ || (has_external_cmd_ && now < last_cmd_time_)) {
        return tl::make_unexpected(make_fault(RobotErr::INVALID_TIME));
    }

    const auto result = ctrller_.set_cmd(cmd);
    if(!result) return tl::make_unexpected(make_ctrller_fault(result.error()));

    last_cmd_time_ = now;
    has_external_cmd_ = true;
    return {};
}

/**
 * @brief 设置完整 Joint 控制命令
 */
tl::expected<void, RobotFault> Robot::set_full_cmd(const JointCtrlCmd& cmd, TimePoint now) {
    if(state_ == RobotState::FAULT) return tl::make_unexpected(make_fault(RobotErr::FAULTED));
    if(state_ != RobotState::ACTIVE) return tl::make_unexpected(make_fault(RobotErr::NOT_ACTIVE));
    if(now < last_cycle_time_ || (has_external_cmd_ && now < last_cmd_time_)) {
        return tl::make_unexpected(make_fault(RobotErr::INVALID_TIME));
    }

    const auto result = ctrller_.set_full_cmd(cmd);
    if(!result) return tl::make_unexpected(make_ctrller_fault(result.error()));

    last_cmd_time_ = now;
    has_external_cmd_ = true;
    return {};
}

/**
 * @brief 切换阻抗模式
 */
tl::expected<void, RobotFault> Robot::set_impedance_mode(JointImpedanceMode mode, TimePoint now) {
    if(state_ == RobotState::FAULT) return tl::make_unexpected(make_fault(RobotErr::FAULTED));
    if(state_ != RobotState::ACTIVE || !has_state_) return tl::make_unexpected(make_fault(RobotErr::NOT_ACTIVE));
    if(now < last_cycle_time_ || (has_external_cmd_ && now < last_cmd_time_)) {
        return tl::make_unexpected(make_fault(RobotErr::INVALID_TIME));
    }

    const auto mode_result = ctrller_.set_impedance_mode(mode, joint_state_);
    if(!mode_result) return tl::make_unexpected(make_ctrller_fault(mode_result.error()));

    const auto history = safety_.reset_cmd_history(joint_state_);
    if(!history) {
        const RobotFault fault = make_safety_fault(history.error());
        enter_fault(fault, SafetyAction::DISABLE);
        return tl::make_unexpected(fault);
    }

    last_joint_cmd_.pos = joint_state_.pos;
    std::fill(last_joint_cmd_.vel.begin(), last_joint_cmd_.vel.end(), 0.0);
    std::fill(last_joint_cmd_.tor.begin(), last_joint_cmd_.tor.end(), 0.0);
    std::fill(last_joint_cmd_.kp.begin(), last_joint_cmd_.kp.end(), 0.0);
    std::fill(last_joint_cmd_.kd.begin(), last_joint_cmd_.kd.end(), 0.0);
    joint_ref_acc_.assign(cfg_.joint_names.size(), 0.0);
    has_last_joint_cmd_ = true;
    has_external_cmd_ = false;
    last_cmd_time_ = now;
    return {};
}

/**
 * @brief 设置模型前馈模式
 */
tl::expected<void, RobotFault> Robot::set_model_feedforward_mode(ModelFeedforwardMode mode) {
    if(state_ == RobotState::UNCONFIGURED) return tl::make_unexpected(make_fault(RobotErr::NOT_CONFIGURED));
    if(state_ == RobotState::FAULT) return tl::make_unexpected(make_fault(RobotErr::FAULTED));
    if(state_ != RobotState::INACTIVE) return tl::make_unexpected(make_fault(RobotErr::NOT_INACTIVE));
    if(mode != ModelFeedforwardMode::NONE && !model_feedforward_) {
        return tl::make_unexpected(make_model_fault(ModelFeedforwardErr::NOT_CONFIGURED));
    }

    cfg_.runtime.model_feedforward_mode = mode;
    return {};
}

/**
 * @brief 执行一次完整控制周期
 */
tl::expected<RobotCycleOutput, RobotFault> Robot::cycle(TimePoint now) {
    if(state_ == RobotState::FAULT) return tl::make_unexpected(make_fault(RobotErr::FAULTED));
    if(state_ != RobotState::ACTIVE) return tl::make_unexpected(make_fault(RobotErr::NOT_ACTIVE));
    if(now < last_cycle_time_ || now < last_state_time_ || (has_external_cmd_ && now < last_cmd_time_)) {
        const RobotFault fault = make_fault(RobotErr::INVALID_TIME);
        enter_fault(fault, SafetyAction::STOP_HOLD);
        return tl::make_unexpected(fault);
    }

    const double nominal_dt = 1.0 / cfg_.runtime.ctrl_frequency_hz;
    const double dt = has_completed_cycle_ ? seconds_between(now, last_cycle_time_) : nominal_dt;

    const auto actuator_state = motor_bus_->read();
    if(!actuator_state) {
        const RobotFault fault = make_bus_fault(RobotErr::MOTOR_BUS_READ_FAILED, actuator_state.error());
        enter_fault(fault, SafetyAction::STOP_HOLD);
        return tl::make_unexpected(fault);
    }

    const TimePoint state_received_at = Clock::now();
    const double state_age_s = has_completed_cycle_ ? seconds_between(state_received_at, last_state_time_) : 0.0;

    const auto joint_state = mapper_.to_joint_state(actuator_state.value());
    if(!joint_state) {
        const RobotFault fault = make_mapper_fault(joint_state.error());
        enter_fault(fault, SafetyAction::DISABLE);
        return tl::make_unexpected(fault);
    }

    const auto checked_state = safety_.check_state(joint_state.value(), actuator_state.value(), state_age_s);
    if(!checked_state) {
        const RobotFault fault = make_safety_fault(checked_state.error());
        enter_fault(fault, safety_.action_for(checked_state.error().code));
        return tl::make_unexpected(fault);
    }

    if(is_tracking_mode() && has_external_cmd_) {
        const double cmd_age_s = seconds_between(now, last_cmd_time_);
        const auto cmd_age = safety_.check_cmd_age(cmd_age_s);
        if(!cmd_age) {
            const RobotFault fault = make_safety_fault(cmd_age.error());
            enter_fault(fault, safety_.action_for(cmd_age.error().code));
            return tl::make_unexpected(fault);
        }
    }

    const JointVector joint_acc = estimate_joint_acc(joint_state.value(), dt);

    JointCtrllerInput input;
    input.state = joint_state.value();
    input.model_feedforward.assign(cfg_.joint_names.size(), 0.0);
    input.dt = dt;

    const auto ctrl_output = ctrller_.update(input);
    if(!ctrl_output) {
        const RobotFault fault = make_ctrller_fault(ctrl_output.error());
        enter_fault(fault, SafetyAction::STOP_HOLD);
        return tl::make_unexpected(fault);
    }

    JointCtrlCmd joint_cmd = ctrl_output->cmd;
    const JointVector joint_ref_acc = estimate_joint_ref_acc(joint_cmd, dt);
    const auto model_feedforward = compute_model_feedforward(joint_state.value(), joint_acc, joint_ref_acc, dt);
    if(!model_feedforward) {
        enter_fault(model_feedforward.error(), SafetyAction::STOP_HOLD);
        return tl::make_unexpected(model_feedforward.error());
    }

    for(std::size_t i = 0; i < joint_cmd.tor.size(); ++i) {
        joint_cmd.tor[i] += model_feedforward.value()[i];
    }

    const auto safe_cmd = safety_.check_joint_cmd(joint_state.value(), joint_cmd, dt);
    if(!safe_cmd) {
        const RobotFault fault = make_safety_fault(safe_cmd.error());
        enter_fault(fault, safety_.action_for(safe_cmd.error().code));
        return tl::make_unexpected(fault);
    }

    const auto actuator_cmd = mapper_.to_actuator_cmd(safe_cmd.value());
    if(!actuator_cmd) {
        const RobotFault fault = make_mapper_fault(actuator_cmd.error());
        enter_fault(fault, SafetyAction::STOP_HOLD);
        return tl::make_unexpected(fault);
    }

    const auto write_result = motor_bus_->write(actuator_cmd.value());
    if(!write_result) {
        const RobotFault fault = make_bus_fault(RobotErr::MOTOR_BUS_WRITE_FAILED, write_result.error());
        enter_fault(fault, SafetyAction::DISABLE);
        return tl::make_unexpected(fault);
    }

    actuator_state_ = actuator_state.value();
    joint_state_ = joint_state.value();
    joint_acc_ = joint_acc;
    joint_ref_acc_ = joint_ref_acc;
    model_feedforward_cache_ = model_feedforward.value();
    last_joint_cmd_ = safe_cmd.value();
    has_state_ = true;
    has_completed_cycle_ = true;
    has_last_joint_cmd_ = true;
    last_cycle_time_ = now;
    last_state_time_ = state_received_at;

    RobotCycleOutput output;
    output.actuator_state = actuator_state_;
    output.joint_state = joint_state_;
    output.joint_acc = joint_acc_;
    output.joint_ref_acc = joint_ref_acc_;
    output.model_feedforward = model_feedforward_cache_;
    output.joint_cmd = safe_cmd.value();
    output.actuator_cmd = actuator_cmd.value();
    output.dt = dt;
    return output;
}

/**
 * @brief 安全停止并失能，回到 INACTIVE
 */
tl::expected<void, RobotFault> Robot::deactivate() {
    if(state_ == RobotState::UNCONFIGURED) return tl::make_unexpected(make_fault(RobotErr::NOT_CONFIGURED));
    if(state_ == RobotState::FAULT) return tl::make_unexpected(make_fault(RobotErr::FAULTED));
    if(state_ == RobotState::INACTIVE) return {};

    const auto result = motor_bus_->deactivate();
    if(!result && result.error() != MotorBusErr::NOT_CONNECTED) {
        const RobotFault fault = make_bus_fault(RobotErr::MOTOR_BUS_DEACTIVATE_FAILED, result.error());
        state_ = RobotState::FAULT;
        last_fault_ = fault;
        return tl::make_unexpected(fault);
    }

    ctrller_.reset();
    safety_.clear_cmd_history();
    clear_runtime_state();
    last_fault_.reset();
    state_ = RobotState::INACTIVE;
    return {};
}

/**
 * @brief 无条件停止并失能，允许从 ACTIVE 或 FAULT 回到 INACTIVE
 */
tl::expected<void, RobotFault> Robot::force_deactivate() {
    if(state_ == RobotState::UNCONFIGURED) return tl::make_unexpected(make_fault(RobotErr::NOT_CONFIGURED));
    if(state_ == RobotState::INACTIVE) return {};

    fault_hold_active_ = false;
    fault_hold_cmd_ = ActuatorMitCmd{};
    const auto result = motor_bus_->deactivate();
    if(!result && result.error() != MotorBusErr::NOT_CONNECTED) {
        const RobotFault fault = make_bus_fault(RobotErr::MOTOR_BUS_DEACTIVATE_FAILED, result.error());
        last_fault_ = fault;
        state_ = RobotState::FAULT;
        return tl::make_unexpected(fault);
    }

    ctrller_.reset();
    safety_.clear_cmd_history();
    clear_runtime_state();
    state_ = RobotState::INACTIVE;
    return {};
}

/**
 * @brief 清除 FAULT 锁存并回到 INACTIVE
 */
tl::expected<void, RobotFault> Robot::reset_fault() {
    return clear_fault();
}

/**
 * @brief 人工请求进入 FAULT 受限柔性恢复
 */
tl::expected<void, RobotFault> Robot::enter_fault_compliant_recovery() {
    if(state_ == RobotState::UNCONFIGURED) return tl::make_unexpected(make_fault(RobotErr::NOT_CONFIGURED));
    if(state_ != RobotState::FAULT) return tl::make_unexpected(make_fault(RobotErr::NOT_FAULTED));
    if(!fault_hold_active_) return tl::make_unexpected(make_fault(RobotErr::FAULTED));

    const auto& recovery = cfg_.safety.fault_recovery;
    // 柔性恢复降低刚度，必须由操作员显式请求；重力模型未验证时低刚度无法作为防坠保障
    if(!recovery.allow_compliant_recovery || recovery.default_mode != FaultHoldMode::RIGID_HOLD ||
        !recovery.require_operator_request || !recovery.gravity_model_validated || !is_compliant_recovery_fault_allowed()) {
        return tl::make_unexpected(make_fault(RobotErr::FAULT_RECOVERY_NOT_ALLOWED));
    }

    const auto actuator_state = motor_bus_->read();
    if(!actuator_state) return tl::make_unexpected(make_bus_fault(RobotErr::MOTOR_BUS_READ_FAILED, actuator_state.error()));

    const auto joint_state = mapper_.to_joint_state(actuator_state.value());
    if(!joint_state) return tl::make_unexpected(make_mapper_fault(joint_state.error()));

    const auto checked_state = safety_.check_state(joint_state.value(), actuator_state.value(), 0.0);
    if(!checked_state) {
        clear_fault_valid_cycles_ = 0;
        return tl::make_unexpected(make_safety_fault(checked_state.error()));
    }

    const auto reference = reset_reference_to_measured(joint_state.value());
    if(!reference) return tl::make_unexpected(reference.error());

    actuator_state_ = actuator_state.value();
    joint_state_ = joint_state.value();
    fault_hold_mode_ = FaultHoldMode::COMPLIANT_RECOVERY;
    fault_recovery_started_at_ = Clock::now();
    return maintain_fault_hold();
}

/**
 * @brief 操作员取消柔性恢复并返回 FAULT 刚性保持
 */
tl::expected<void, RobotFault> Robot::return_to_fault_rigid_hold() {
    if(state_ == RobotState::UNCONFIGURED) return tl::make_unexpected(make_fault(RobotErr::NOT_CONFIGURED));
    if(state_ != RobotState::FAULT) return tl::make_unexpected(make_fault(RobotErr::NOT_FAULTED));
    fault_hold_mode_ = FaultHoldMode::RIGID_HOLD;
    return maintain_fault_hold();
}

/**
 * @brief 清除 FAULT 并以当前实测位置进入 ACTIVE + RIGID_HOLD
 */
tl::expected<void, RobotFault> Robot::clear_fault() {
    if(state_ == RobotState::UNCONFIGURED) return tl::make_unexpected(make_fault(RobotErr::NOT_CONFIGURED));
    if(state_ != RobotState::FAULT) return tl::make_unexpected(make_fault(RobotErr::NOT_FAULTED));
    if(!fault_hold_active_) return tl::make_unexpected(make_fault(RobotErr::FAULTED));

    const auto actuator_state = motor_bus_->read();
    if(!actuator_state) return tl::make_unexpected(make_bus_fault(RobotErr::MOTOR_BUS_READ_FAILED, actuator_state.error()));

    const auto joint_state = mapper_.to_joint_state(actuator_state.value());
    if(!joint_state) return tl::make_unexpected(make_mapper_fault(joint_state.error()));

    const auto checked_state = safety_.check_state(joint_state.value(), actuator_state.value(), 0.0);
    if(!checked_state) return tl::make_unexpected(make_safety_fault(checked_state.error()));

    for(std::size_t i = 0; i < cfg_.joint_names.size(); ++i) {
        if(std::abs(joint_state->vel[i]) > cfg_.shutdown.velocity_tolerance) {
            return tl::make_unexpected(make_fault(RobotErr::FAULT_RECOVERY_NOT_ALLOWED));
        }
    }

    if(clear_fault_valid_cycles_ < 3) {
        return tl::make_unexpected(make_fault(RobotErr::FAULT_RECOVERY_NOT_ALLOWED));
    }

    const auto reference = reset_reference_to_measured(joint_state.value());
    if(!reference) return tl::make_unexpected(reference.error());

    const TimePoint resumed_at = Clock::now();
    actuator_state_ = actuator_state.value();
    joint_state_ = joint_state.value();
    last_cycle_time_ = resumed_at;
    last_state_time_ = resumed_at;
    last_cmd_time_ = resumed_at;
    has_state_ = true;
    has_completed_cycle_ = false;
    has_external_cmd_ = false;
    has_last_joint_cmd_ = true;
    fault_hold_active_ = false;
    fault_hold_cmd_ = ActuatorMitCmd{};
    fault_hold_mode_ = FaultHoldMode::RIGID_HOLD;
    current_fault_.reset();
    state_ = RobotState::ACTIVE;
    return {};
}

/**
 * @brief 在 FAULT 状态持续刷新刚性保持命令
 */
tl::expected<void, RobotFault> Robot::maintain_fault_hold() {
    if(state_ != RobotState::FAULT) return tl::make_unexpected(make_fault(RobotErr::NOT_FAULTED));
    if(!fault_hold_active_) return tl::make_unexpected(make_fault(RobotErr::FAULTED));
    return update_fault_reaction(Clock::now());
}

/**
 * @brief 获取当前 Robot 生命周期状态
 */
RobotState Robot::get_state() const noexcept {
    return state_;
}

/**
 * @brief 获取当前控制器阻抗模式
 */
JointImpedanceMode Robot::get_impedance_mode() const noexcept {
    return ctrller_.get_impedance_mode();
}

/**
 * @brief 获取当前模型前馈模式
 */
ModelFeedforwardMode Robot::get_model_feedforward_mode() const noexcept {
    return cfg_.runtime.model_feedforward_mode;
}

/**
 * @brief 获取最近一次合法的关节状态
 */
const JointState& Robot::get_joint_state() const noexcept {
    return joint_state_;
}

/**
 * @brief 获取最近一次关节加速度估计
 */
const JointVector& Robot::get_joint_acc() const noexcept {
    return joint_acc_;
}

/**
 * @brief 获取最近一次关节参考加速度
 */
const JointVector& Robot::get_joint_ref_acc() const noexcept {
    return joint_ref_acc_;
}

/**
 * @brief 获取最近一次模型前馈力矩
 */
const JointVector& Robot::get_model_feedforward() const noexcept {
    return model_feedforward_cache_;
}

/**
 * @brief 获取最近一次合法的执行器状态
 */
const ActuatorState& Robot::get_actuator_state() const noexcept {
    return actuator_state_;
}

/**
 * @brief 获取最近一次锁存的故障信息
 */
const tl::optional<RobotFault>& Robot::get_last_fault() const noexcept {
    return last_fault_;
}

/**
 * @brief 获取当前是否正在执行故障刚性保持
 */
bool Robot::is_fault_holding() const noexcept {
    return fault_hold_active_;
}

/**
 * @brief 获取 FAULT 内部保持模式
 */
FaultHoldMode Robot::get_fault_hold_mode() const noexcept {
    return fault_hold_mode_;
}

// ! ========================= 私 有 类 方 法 实 现 ========================= ! //

/**
 * @brief 计算当前周期的关节加速度估计
 */
JointVector Robot::estimate_joint_acc(const JointState& state, double dt) const {
    JointVector result(cfg_.joint_names.size(), 0.0);
    if(!has_completed_cycle_ || joint_state_.vel.size() != state.vel.size()) return result;

    const double alpha = cfg_.runtime.joint_acc_filter_alpha;
    for(std::size_t i = 0; i < result.size(); ++i) {
        const double raw_acc = (state.vel[i] - joint_state_.vel[i]) / dt;
        const double previous_acc = i < joint_acc_.size() ? joint_acc_[i] : 0.0;
        result[i] = alpha * raw_acc + (1.0 - alpha) * previous_acc;
    }
    return result;
}

/**
 * @brief 计算当前周期的关节参考加速度
 */
JointVector Robot::estimate_joint_ref_acc(const JointCtrlCmd& cmd, double dt) const {
    JointVector result(cfg_.joint_names.size(), 0.0);
    if(!has_last_joint_cmd_ || last_joint_cmd_.vel.size() != cmd.vel.size()) return result;

    for(std::size_t i = 0; i < result.size(); ++i) {
        result[i] = (cmd.vel[i] - last_joint_cmd_.vel[i]) / dt;
    }
    return result;
}

/**
 * @brief 计算当前周期的模型前馈项
 */
tl::expected<JointVector, RobotFault> Robot::compute_model_feedforward(const JointState& state, const JointVector& joint_acc, const JointVector& joint_ref_acc, double dt) const {
    if(!model_feedforward_) {
        if(cfg_.runtime.model_feedforward_mode == ModelFeedforwardMode::NONE) {
            return JointVector(cfg_.joint_names.size(), 0.0);
        }
        return tl::make_unexpected(make_model_fault(ModelFeedforwardErr::NOT_CONFIGURED));
    }

    const auto result = model_feedforward_(cfg_.runtime.model_feedforward_mode, state, joint_acc, joint_ref_acc, dt);
    if(!result) return tl::make_unexpected(make_model_fault(result.error()));
    if(result->size() != cfg_.joint_names.size() || !finite_vector(result.value())) {
        return tl::make_unexpected(make_fault(RobotErr::INVALID_MODEL_FEEDFORWARD));
    }
    return result.value();
}

/**
 * @brief 构造仅包含通用错误码的故障对象
 */
RobotFault Robot::make_fault(RobotErr code) const noexcept {
    RobotFault fault;
    fault.code = code;
    return fault;
}

/**
 * @brief 构造包含 MotorBus 子错误的故障对象
 */
RobotFault Robot::make_bus_fault(RobotErr code, MotorBusErr err) const noexcept {
    RobotFault fault = make_fault(code);
    fault.motor_bus_err = err;
    return fault;
}

/**
 * @brief 构造包含映射器子错误的故障对象
 */
RobotFault Robot::make_mapper_fault(JointActuatorMapErr err) const noexcept {
    RobotFault fault = make_fault(RobotErr::MAPPER_FAILED);
    fault.mapper_err = err;
    return fault;
}

/**
 * @brief 构造包含控制器子错误的故障对象
 */
RobotFault Robot::make_ctrller_fault(JointCtrllerErr err) const noexcept {
    RobotFault fault = make_fault(RobotErr::CTRLLER_FAILED);
    fault.ctrller_err = err;
    return fault;
}

/**
 * @brief 构造包含安全检查子错误的故障对象
 */
RobotFault Robot::make_safety_fault(const SafetyFault& safety_fault) const noexcept {
    RobotFault fault = make_fault(RobotErr::SAFETY_FAILED);
    fault.safety_fault = safety_fault;
    return fault;
}

/**
 * @brief 构造包含模型前馈子错误的故障对象
 */
RobotFault Robot::make_model_fault(ModelFeedforwardErr err) const noexcept {
    RobotFault fault = make_fault(RobotErr::MODEL_FEEDFORWARD_FAILED);
    fault.model_feedforward_err = err;
    return fault;
}

/**
 * @brief 进入 FAULT 状态并执行对应安全动作
 */
void Robot::enter_fault(const RobotFault& fault, SafetyAction action) noexcept {
    fault_hold_active_ = false;
    fault_hold_cmd_ = ActuatorMitCmd{};
    fault_hold_mode_ = FaultHoldMode::RIGID_HOLD;
    clear_fault_valid_cycles_ = 0;
    if(action == SafetyAction::STOP_HOLD) {
        // FAULT 默认刚性保持：先清外部命令，再以最新合法实测位置建参考，避免旧轨迹在清故障后继续执行
        fault_hold_active_ = start_fault_hold_noexcept();
        if(!fault_hold_active_) stop_or_disable_noexcept();
    }
    else {
        disable_noexcept();
    }

    ctrller_.reset();
    safety_.clear_cmd_history();
    has_external_cmd_ = false;
    state_ = RobotState::FAULT;
    last_fault_ = fault;
    current_fault_ = fault;
}

/**
 * @brief 尝试停机，失败后降级为失能
 */
void Robot::stop_or_disable_noexcept() noexcept {
    if(!motor_bus_) return;
    const auto stopped = motor_bus_->stop();
    if(!stopped) (void)motor_bus_->deactivate();
}

/**
 * @brief 使用最近一次合法状态构造并发送故障刚性保持命令
 */
bool Robot::start_fault_hold_noexcept() noexcept {
    if(!motor_bus_ || !has_state_ || joint_state_.pos.size() != cfg_.joint_names.size()) return false;

    if(!reset_reference_to_measured(joint_state_)) return false;
    const double nominal_dt = 1.0 / cfg_.runtime.ctrl_frequency_hz;
    const auto hold_cmd = build_fault_joint_cmd(joint_state_, FaultHoldMode::RIGID_HOLD, nominal_dt);
    if(!hold_cmd) return false;
    const auto actuator_cmd = mapper_.to_actuator_cmd(hold_cmd.value());
    if(!actuator_cmd) return false;
    const auto written = motor_bus_->write(actuator_cmd.value());
    if(!written) return false;

    fault_hold_cmd_ = actuator_cmd.value();
    return true;
}

/**
 * @brief 根据 FAULT 内部模式刷新保持命令
 */
tl::expected<void, RobotFault> Robot::update_fault_reaction(TimePoint now) {
    const double nominal_dt = 1.0 / cfg_.runtime.ctrl_frequency_hz;
    const double dt = last_cycle_time_ == TimePoint{} ? nominal_dt : std::max(nominal_dt, seconds_between(now, last_cycle_time_));

    if(fault_hold_mode_ == FaultHoldMode::COMPLIANT_RECOVERY &&
        seconds_between(now, fault_recovery_started_at_) > cfg_.safety.fault_recovery.recovery_timeout_s) {
        fault_hold_mode_ = FaultHoldMode::RIGID_HOLD;
    }

    const auto actuator_state = motor_bus_->read();
    if(!actuator_state) {
        const RobotFault fault = make_bus_fault(RobotErr::MOTOR_BUS_READ_FAILED, actuator_state.error());
        current_fault_ = fault;
        last_fault_ = fault;
        return tl::make_unexpected(fault);
    }
    const auto joint_state = mapper_.to_joint_state(actuator_state.value());
    if(!joint_state) return tl::make_unexpected(make_mapper_fault(joint_state.error()));

    const auto checked_state = safety_.check_state(joint_state.value(), actuator_state.value(), 0.0);
    if(!checked_state) {
        clear_fault_valid_cycles_ = 0;
        if(fault_hold_mode_ == FaultHoldMode::COMPLIANT_RECOVERY && checked_state.error().code == SafetyErr::JOINT_POS_LIMIT) {
            // Joint 限位恢复只允许向限位内侧运动；若实测继续深入限位，立即退回刚性保持
            fault_hold_mode_ = FaultHoldMode::RIGID_HOLD;
        }
        if(checked_state.error().code != SafetyErr::JOINT_POS_LIMIT) return tl::make_unexpected(make_safety_fault(checked_state.error()));
    }
    else {
        ++clear_fault_valid_cycles_;
    }

    const auto joint_cmd = build_fault_joint_cmd(joint_state.value(), fault_hold_mode_, dt);
    if(!joint_cmd) return tl::make_unexpected(joint_cmd.error());
    const auto safe_cmd = safety_.check_joint_cmd(joint_state.value(), joint_cmd.value(), std::min(dt, cfg_.safety.max_dt_s));
    if(!safe_cmd) {
        fault_hold_mode_ = FaultHoldMode::RIGID_HOLD;
        const auto rigid_cmd = build_fault_joint_cmd(joint_state.value(), FaultHoldMode::RIGID_HOLD, nominal_dt);
        if(!rigid_cmd) return tl::make_unexpected(rigid_cmd.error());
        const auto rigid_safe_cmd = safety_.check_joint_cmd(joint_state.value(), rigid_cmd.value(), nominal_dt);
        if(!rigid_safe_cmd) return tl::make_unexpected(make_safety_fault(rigid_safe_cmd.error()));
        const auto rigid_actuator_cmd = mapper_.to_actuator_cmd(rigid_safe_cmd.value());
        if(!rigid_actuator_cmd) return tl::make_unexpected(make_mapper_fault(rigid_actuator_cmd.error()));
        fault_hold_cmd_ = rigid_actuator_cmd.value();
    }
    else {
        const auto actuator_cmd = mapper_.to_actuator_cmd(safe_cmd.value());
        if(!actuator_cmd) return tl::make_unexpected(make_mapper_fault(actuator_cmd.error()));
        fault_hold_cmd_ = actuator_cmd.value();
    }

    const auto result = motor_bus_->write(fault_hold_cmd_);
    if(!result) {
        fault_hold_active_ = false;
        return tl::make_unexpected(make_bus_fault(RobotErr::MOTOR_BUS_WRITE_FAILED, result.error()));
    }

    actuator_state_ = actuator_state.value();
    joint_state_ = joint_state.value();
    last_cycle_time_ = now;
    last_state_time_ = now;
    return {};
}

/**
 * @brief 构造故障保持 Joint 命令
 */
tl::expected<JointCtrlCmd, RobotFault> Robot::build_fault_joint_cmd(const JointState& state, FaultHoldMode mode, double dt) {
    JointCtrlCmd cmd;
    cmd.pos = last_joint_cmd_.pos.size() == cfg_.joint_names.size() ? last_joint_cmd_.pos : state.pos;
    cmd.vel.assign(cfg_.joint_names.size(), 0.0);
    cmd.tor.assign(cfg_.joint_names.size(), 0.0);
    cmd.kp = cfg_.ctrller.rigid_hold_gains.kp;
    cmd.kd = cfg_.ctrller.rigid_hold_gains.kd;

    if(mode == FaultHoldMode::COMPLIANT_RECOVERY) {
        cmd.kp = cfg_.safety.fault_recovery.compliant_recovery.kp;
        cmd.kd = cfg_.safety.fault_recovery.compliant_recovery.kd;
        for(std::size_t i = 0; i < cfg_.joint_names.size(); ++i) {
            const double error = last_joint_cmd_.pos[i] - state.pos[i];
            cmd.vel[i] = std::clamp(error / std::max(dt, 1.0e-6), -cfg_.safety.fault_recovery.compliant_recovery.max_vel[i], cfg_.safety.fault_recovery.compliant_recovery.max_vel[i]);
            if(current_fault_ && current_fault_->code == RobotErr::SAFETY_FAILED && current_fault_->safety_fault.code == SafetyErr::JOINT_POS_LIMIT && current_fault_->safety_fault.index == i) {
                if(state.pos[i] > cfg_.safety.limits.max_pos[i] && cmd.vel[i] > 0.0) return tl::make_unexpected(make_fault(RobotErr::FAULT_RECOVERY_NOT_ALLOWED));
                if(state.pos[i] < cfg_.safety.limits.min_pos[i] && cmd.vel[i] < 0.0) return tl::make_unexpected(make_fault(RobotErr::FAULT_RECOVERY_NOT_ALLOWED));
            }
            cmd.pos[i] = state.pos[i] + cmd.vel[i] * dt;
        }
    }

    if(model_feedforward_) {
        const JointVector zero(cfg_.joint_names.size(), 0.0);
        const auto gravity = model_feedforward_(ModelFeedforwardMode::GRAVITY, state, zero, zero, dt);
        if(!gravity) return tl::make_unexpected(make_model_fault(gravity.error()));
        if(gravity->size() != cfg_.joint_names.size() || !finite_vector(gravity.value())) return tl::make_unexpected(make_fault(RobotErr::INVALID_MODEL_FEEDFORWARD));
        cmd.tor = gravity.value();
        if(mode == FaultHoldMode::COMPLIANT_RECOVERY) {
            for(double& value : cmd.tor) value *= cfg_.safety.fault_recovery.compliant_recovery.effort_scale;
        }
    }
    return cmd;
}

/**
 * @brief 判断当前锁存故障是否允许柔性恢复
 */
bool Robot::is_compliant_recovery_fault_allowed() const noexcept {
    if(!current_fault_) return false;
    if(current_fault_->code == RobotErr::MOTOR_BUS_READ_FAILED || current_fault_->code == RobotErr::MOTOR_BUS_WRITE_FAILED) return false;
    if(current_fault_->code != RobotErr::SAFETY_FAILED) return false;
    // 通信、供电、执行器内部错误、非有限状态与持续状态超时禁止柔性恢复；软件只能继续尽力刷新最后合法保持命令
    switch(current_fault_->safety_fault.code) {
        case SafetyErr::CMD_TIMEOUT:
        case SafetyErr::CMD_POS_LIMIT:
        case SafetyErr::CMD_VEL_LIMIT:
        case SafetyErr::CMD_POS_STEP_LIMIT:
        case SafetyErr::CMD_VEL_STEP_LIMIT:
        case SafetyErr::JOINT_POS_LIMIT:
        case SafetyErr::JOINT_VEL_LIMIT:
            return true;
        default:
            return false;
    }
}

/**
 * @brief 清除外部命令并把参考重置为实测位置
 */
tl::expected<void, RobotFault> Robot::reset_reference_to_measured(const JointState& state) {
    ctrller_.reset();
    const auto initialized = ctrller_.initialize(state);
    if(!initialized) return tl::make_unexpected(make_ctrller_fault(initialized.error()));

    const auto mode_result = ctrller_.set_impedance_mode(JointImpedanceMode::RIGID_HOLD, state);
    if(!mode_result) return tl::make_unexpected(make_ctrller_fault(mode_result.error()));

    const auto history = safety_.reset_cmd_history(state);
    if(!history) return tl::make_unexpected(make_safety_fault(history.error()));

    joint_acc_.assign(cfg_.joint_names.size(), 0.0);
    joint_ref_acc_.assign(cfg_.joint_names.size(), 0.0);
    model_feedforward_cache_.assign(cfg_.joint_names.size(), 0.0);
    last_joint_cmd_.pos = state.pos;
    last_joint_cmd_.vel.assign(cfg_.joint_names.size(), 0.0);
    last_joint_cmd_.tor.assign(cfg_.joint_names.size(), 0.0);
    last_joint_cmd_.kp.assign(cfg_.joint_names.size(), 0.0);
    last_joint_cmd_.kd.assign(cfg_.joint_names.size(), 0.0);
    has_external_cmd_ = false;
    has_last_joint_cmd_ = true;
    return {};
}

/**
 * @brief 直接失能硬件，忽略错误
 */
void Robot::disable_noexcept() noexcept {
    if(!motor_bus_) return;
    (void)motor_bus_->deactivate();
}

/**
 * @brief 清空运行时缓存状态
 */
void Robot::clear_runtime_state() noexcept {
    joint_state_ = JointState{};
    joint_acc_.clear();
    joint_ref_acc_.clear();
    model_feedforward_cache_.clear();
    last_joint_cmd_ = JointCtrlCmd{};
    actuator_state_ = ActuatorState{};
    has_state_ = false;
    has_completed_cycle_ = false;
    has_external_cmd_ = false;
    has_last_joint_cmd_ = false;
    fault_hold_active_ = false;
    fault_hold_cmd_ = ActuatorMitCmd{};
    last_cycle_time_ = TimePoint{};
    last_state_time_ = TimePoint{};
    last_cmd_time_ = TimePoint{};
}

/**
 * @brief 判断当前控制模式是否属于跟踪模式
 */
bool Robot::is_tracking_mode() const noexcept {
    const JointImpedanceMode mode = ctrller_.get_impedance_mode();
    return mode == JointImpedanceMode::RIGID_TRACKING || mode == JointImpedanceMode::COMPLIANT_TRACKING;
}

/**
 * @brief 计算两个时间点之间的秒数差
 */
double Robot::seconds_between(TimePoint newer, TimePoint older) const noexcept {
    return std::chrono::duration<double>(newer - older).count();
}

} // namespace serial_arm
