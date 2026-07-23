#include "dm_arm/robot.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace dm_arm {

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
        enter_fault(fault, SafetyAction::DISABLE);
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
 * @brief 清除 FAULT 锁存并回到 INACTIVE
 */
tl::expected<void, RobotFault> Robot::reset_fault() {
    if(state_ == RobotState::UNCONFIGURED) return tl::make_unexpected(make_fault(RobotErr::NOT_CONFIGURED));
    if(state_ != RobotState::FAULT) return tl::make_unexpected(make_fault(RobotErr::NOT_FAULTED));

    const auto recovered = motor_bus_->recover();
    if(!recovered) {
        const RobotFault fault = make_bus_fault(RobotErr::MOTOR_BUS_RECOVER_FAILED, recovered.error());
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
    if(action == SafetyAction::STOP_HOLD) stop_or_disable_noexcept();
    else disable_noexcept();

    ctrller_.reset();
    safety_.clear_cmd_history();
    has_external_cmd_ = false;
    state_ = RobotState::FAULT;
    last_fault_ = fault;
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

} // namespace dm_arm
