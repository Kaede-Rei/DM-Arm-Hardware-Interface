#include "robot_session.hpp"

#ifdef DM_ARM_PYTHON_HAS_DAMIAO
#include "dm_arm/hardware/damiao_motor_bus.hpp"
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <utility>

namespace dm_arm {

namespace {

/**
 * @brief 将 RobotFault 转换为 Python 会话错误文本
 * @param action 当前失败操作名称
 * @param fault Robot 故障详情
 * @return 包含顶层错误和有效子错误的文本
 */
std::string make_robot_error(const char* action, const RobotFault& fault) {
    std::ostringstream stream;
    stream << action << " failed; RobotErr=" << static_cast<int>(fault.code);

    const bool is_start_error = fault.code == RobotErr::MOTOR_BUS_CONNECT_FAILED || fault.code == RobotErr::MOTOR_BUS_ACTIVATE_FAILED;
    const bool is_cycle_error = fault.code == RobotErr::MOTOR_BUS_READ_FAILED || fault.code == RobotErr::MOTOR_BUS_WRITE_FAILED;
    const bool is_end_error = fault.code == RobotErr::MOTOR_BUS_DEACTIVATE_FAILED || fault.code == RobotErr::MOTOR_BUS_RECOVER_FAILED;
    const bool is_motor_bus_error = is_start_error || is_cycle_error || is_end_error;
    if(is_motor_bus_error) {
        stream << "; MotorBusErr=" << static_cast<int>(fault.motor_bus_err);
    }
    if(fault.code == RobotErr::MAPPER_FAILED) {
        stream << "; JointActuatorMapErr=" << static_cast<int>(fault.mapper_err);
    }
    if(fault.code == RobotErr::CTRLLER_FAILED) {
        stream << "; JointCtrllerErr=" << static_cast<int>(fault.ctrller_err);
    }
    if(fault.code == RobotErr::SAFETY_FAILED) {
        stream << "; SafetyErr=" << static_cast<int>(fault.safety_fault.code) << "; index=" << fault.safety_fault.index
            << "; value=" << fault.safety_fault.value << "; limit=" << fault.safety_fault.limit;
    }
    if(fault.code == RobotErr::MODEL_FEEDFORWARD_FAILED || fault.code == RobotErr::INVALID_MODEL_FEEDFORWARD) {
        stream << "; ModelFeedforwardErr=" << static_cast<int>(fault.model_feedforward_err);
    }

    return stream.str();
}

/**
 * @brief 判断阻抗模式是否需要连续外部参考
 * @param mode 阻抗模式
 * @return 需要连续参考时返回 true，否则返回 false
 */
bool is_tracking_mode(JointImpedanceMode mode) {
    return mode == JointImpedanceMode::RIGID_TRACKING || mode == JointImpedanceMode::COMPLIANT_TRACKING;
}

} // namespace

// ! ========================= 构 造 / 析 构 方 法 实 现 ========================= ! //

PyRobotSession::~PyRobotSession() {
    try {
        stop();
    }
    catch(...) {
    }
}

// ! ========================= 配 置 / 生 命 周 期 方 法 实 现 ========================= ! //

void PyRobotSession::configure(const std::string& config_file) {
    if(configured_) {
        throw DmArmPythonError("RobotSession is already configured");
    }

#ifndef DM_ARM_PYTHON_HAS_DAMIAO
    static_cast<void>(config_file);
    throw DmArmPythonError("RobotSession requires DM_ARM_BUILD_DAMIAO=ON");
#else
    const auto cfg_result = load_robot_cfg(config_file);
    if(!cfg_result) {
        throw DmArmPythonError(cfg_result.error().message);
    }

    RobotCfg cfg = cfg_result.value();
    auto dynamics = std::make_unique<Dynamics>();
    const auto dynamics_result = dynamics->configure(cfg.dynamics);
    if(!dynamics_result) {
        throw DmArmPythonError("Dynamics configure failed; DynamicsErr=" + std::to_string(static_cast<int>(dynamics_result.error())));
    }

    auto bus = std::make_unique<DamiaoMotorBus>();
    const auto bus_result = bus->configure(cfg.damiao);
    if(!bus_result) {
        throw DmArmPythonError("DamiaoMotorBus configure failed; MotorBusErr=" + std::to_string(static_cast<int>(bus_result.error())));
    }

    std::vector<RobotSessionActuatorInfo> actuator_info;
    actuator_info.reserve(bus->get_actuator_info().size());
    for(const auto& item : bus->get_actuator_info()) {
        actuator_info.push_back(RobotSessionActuatorInfo{ item.name, item.joint_name, item.motor_id, item.master_id, item.motor_type, item.q_max, item.dq_max, item.tau_max });
    }

    cfg_ = std::move(cfg);
    dynamics_ = std::move(dynamics);
    actuator_info_ = std::move(actuator_info);

    auto robot = std::make_unique<Robot>();
    const auto robot_result = robot->configure(cfg_, std::move(bus), make_model_feedforward());
    if(!robot_result) {
        dynamics_.reset();
        actuator_info_.clear();
        throw DmArmPythonError(make_robot_error("Robot configure", robot_result.error()));
    }

    robot_ = std::move(robot);
    requested_impedance_mode_ = JointImpedanceMode::RIGID_HOLD;
    requested_gravity_scale_ = cfg_.dynamics.gravity_scale;
    goal_pos_.assign(cfg_.joint_names.size(), 0.0);
    ref_pos_.assign(cfg_.joint_names.size(), 0.0);
    ref_vel_.assign(cfg_.joint_names.size(), 0.0);
    snapshot_.robot_state = RobotState::INACTIVE;
    snapshot_.last_error.clear();
    snapshot_.valid = false;
    configured_ = true;
#endif
}

void PyRobotSession::start(bool allow_hardware) {
    if(!configured_ || !robot_) {
        throw DmArmPythonError("RobotSession is not configured");
    }
    if(!allow_hardware) {
        throw DmArmPythonError("start() requires allow_hardware=True");
    }
    if(!cfg_.runtime.write_enabled) {
        throw DmArmPythonError("runtime.write_enabled must be true before hardware start");
    }
    if(running_.load()) {
        throw DmArmPythonError("RobotSession is already running");
    }
    if(worker_.joinable()) {
        worker_.join();
    }
    if(robot_->get_state() == RobotState::FAULT) {
        throw DmArmPythonError("Robot is in FAULT; call reset_fault() before start()");
    }

    const auto activate_result = robot_->activate();
    if(!activate_result) {
        throw DmArmPythonError(make_robot_error("Robot activate", activate_result.error()));
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        ref_pos_ = robot_->get_joint_state().pos;
        ref_vel_.assign(ref_pos_.size(), 0.0);
        goal_pos_ = ref_pos_;
        requested_impedance_mode_ = JointImpedanceMode::RIGID_HOLD;
        requested_gravity_scale_ = dynamics_->get_gravity_scale();
        impedance_sequence_ = 0;
        gravity_sequence_ = 0;
        speed_scale_ = 0.3;
        has_goal_ = false;
        snapshot_.robot_state = RobotState::ACTIVE;
        snapshot_.last_error.clear();
        snapshot_.valid = false;
    }

    running_.store(true);
    try {
        worker_ = std::thread(&PyRobotSession::loop, this);
    }
    catch(...) {
        running_.store(false);
        static_cast<void>(robot_->deactivate());
        throw;
    }
}

void PyRobotSession::stop() {
    running_.store(false);
    if(worker_.joinable()) {
        worker_.join();
    }
    if(!robot_) {
        return;
    }

    if(robot_->get_state() == RobotState::ACTIVE) {
        const auto result = robot_->deactivate();
        if(!result) {
            throw DmArmPythonError(make_robot_error("Robot deactivate", result.error()));
        }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.robot_state = robot_->get_state();
}

void PyRobotSession::reset_fault() {
    clear_fault();
}

void PyRobotSession::clear_fault() {
    running_.store(false);
    if(worker_.joinable()) {
        worker_.join();
    }
    if(!robot_) {
        throw DmArmPythonError("RobotSession is not configured");
    }

    const auto result = robot_->clear_fault();
    if(!result) {
        throw DmArmPythonError(make_robot_error("Robot clear_fault", result.error()));
    }

    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.robot_state = robot_->get_state();
    snapshot_.last_error.clear();
    snapshot_.valid = false;
    has_goal_ = false;
}

void PyRobotSession::enter_fault_compliant_recovery() {
    if(!robot_) {
        throw DmArmPythonError("RobotSession is not configured");
    }
    const auto result = robot_->enter_fault_compliant_recovery();
    if(!result) {
        throw DmArmPythonError(make_robot_error("Robot enter_fault_compliant_recovery", result.error()));
    }
}

void PyRobotSession::return_to_fault_rigid_hold() {
    if(!robot_) {
        throw DmArmPythonError("RobotSession is not configured");
    }
    const auto result = robot_->return_to_fault_rigid_hold();
    if(!result) {
        throw DmArmPythonError(make_robot_error("Robot return_to_fault_rigid_hold", result.error()));
    }
}

// ! ========================= 命 令 / 调 参 方 法 实 现 ========================= ! //

void PyRobotSession::set_impedance_mode(JointImpedanceMode mode) {
    if(!running_.load()) {
        throw DmArmPythonError("RobotSession must be running before set_impedance_mode()");
    }

    std::lock_guard<std::mutex> lock(mutex_);
    requested_impedance_mode_ = mode;
    ++impedance_sequence_;
}

void PyRobotSession::set_model_feedforward_mode(ModelFeedforwardMode mode) {
    if(!configured_ || !robot_) {
        throw DmArmPythonError("RobotSession is not configured");
    }
    if(running_.load() || robot_->get_state() != RobotState::INACTIVE) {
        throw DmArmPythonError("set_model_feedforward_mode() requires an INACTIVE session");
    }
    if(worker_.joinable()) {
        worker_.join();
    }

    const auto result = robot_->set_model_feedforward_mode(mode);
    if(!result) {
        throw DmArmPythonError(make_robot_error("Robot set_model_feedforward_mode", result.error()));
    }

    std::lock_guard<std::mutex> lock(mutex_);
    cfg_.runtime.model_feedforward_mode = mode;
}

void PyRobotSession::set_gravity_scale(const JointVector& gravity_scale) {
    validate_joint_vector(gravity_scale, "gravity_scale");
    for(double value : gravity_scale) {
        if(value < 0.0 || value > 1.0) {
            throw DmArmPythonError("gravity_scale values must be in [0, 1]");
        }
    }

    if(!running_.load()) {
        if(worker_.joinable()) {
            worker_.join();
        }
        const auto result = dynamics_->set_gravity_scale(gravity_scale);
        if(!result) {
            throw DmArmPythonError("Dynamics set_gravity_scale failed; DynamicsErr=" + std::to_string(static_cast<int>(result.error())));
        }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    requested_gravity_scale_ = gravity_scale;
    ++gravity_sequence_;
}

void PyRobotSession::move_to(const JointVector& pos, double speed_scale) {
    validate_joint_vector(pos, "pos");
    if(!running_.load()) {
        throw DmArmPythonError("RobotSession must be running before move_to()");
    }
    if(!std::isfinite(speed_scale) || speed_scale <= 0.0 || speed_scale > 1.0) {
        throw DmArmPythonError("speed_scale must be in (0, 1]");
    }

    for(std::size_t i = 0; i < pos.size(); ++i) {
        const double min_pos = cfg_.safety.limits.min_pos[i] + cfg_.safety.limits.pos_margin[i];
        const double max_pos = cfg_.safety.limits.max_pos[i] - cfg_.safety.limits.pos_margin[i];
        if(pos[i] < min_pos || pos[i] > max_pos) {
            throw DmArmPythonError("position goal exceeds Safety soft limit at joint index " + std::to_string(i));
        }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if(!is_tracking_mode(requested_impedance_mode_)) {
        throw DmArmPythonError("move_to() requires RIGID_TRACKING or COMPLIANT_TRACKING");
    }
    goal_pos_ = pos;
    speed_scale_ = speed_scale;
    has_goal_ = true;
}

void PyRobotSession::hold_current() {
    if(!running_.load()) {
        throw DmArmPythonError("RobotSession must be running before hold_current()");
    }

    std::lock_guard<std::mutex> lock(mutex_);
    requested_impedance_mode_ = JointImpedanceMode::RIGID_HOLD;
    ++impedance_sequence_;
    has_goal_ = false;
}

// ! ========================= 状 态 / 快 照 读 取 方 法 实 现 ========================= ! //

RobotSessionSnapshot PyRobotSession::get_snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_;
}

RobotState PyRobotSession::get_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_.robot_state;
}

FaultHoldMode PyRobotSession::get_fault_hold_mode() const {
    if(!robot_) {
        throw DmArmPythonError("RobotSession is not configured");
    }
    return robot_->get_fault_hold_mode();
}

bool PyRobotSession::is_configured() const noexcept {
    return configured_;
}

bool PyRobotSession::is_running() const noexcept {
    return running_.load();
}

RobotCfg PyRobotSession::get_config() const {
    if(!configured_) {
        throw DmArmPythonError("RobotSession is not configured");
    }
    return cfg_;
}

DynamicsInfo PyRobotSession::get_dynamics_info() const {
    if(!dynamics_) {
        throw DmArmPythonError("Dynamics is not configured");
    }
    return dynamics_->get_info();
}

std::vector<RobotSessionActuatorInfo> PyRobotSession::get_actuator_info() const {
    return actuator_info_;
}

// ! ========================= 周 期 线 程 / 参 考 生 成 方 法 实 现 ========================= ! //

/**
 * @brief 运行固定频率控制周期并应用 Python 侧提交的最新请求
 *
 * 工作线程串行执行模式切换、重力比例更新、参考生成、Robot::set_cmd() 和 Robot::cycle()；退出前在 ACTIVE 状态下执行安全失能
 */
void PyRobotSession::loop() noexcept {
    const double target_dt = 1.0 / cfg_.runtime.ctrl_frequency_hz;
    const auto period = std::chrono::duration_cast<Robot::Clock::duration>(std::chrono::duration<double>(target_dt));
    auto previous_time = Robot::Clock::now();
    auto next_time = previous_time;

    std::uint64_t applied_impedance_sequence = 0;
    std::uint64_t applied_gravity_sequence = 0;
    JointImpedanceMode applied_impedance_mode = JointImpedanceMode::RIGID_HOLD;

    while(running_.load()) {
        const auto now = Robot::Clock::now();
        if(now > next_time) next_time = now;
        double dt = std::chrono::duration<double>(now - previous_time).count();
        previous_time = now;
        if(!std::isfinite(dt) || dt <= 0.0) {
            dt = target_dt;
        }

        JointImpedanceMode impedance_mode;
        JointVector gravity_scale;
        std::uint64_t impedance_sequence;
        std::uint64_t gravity_sequence;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            impedance_mode = requested_impedance_mode_;
            gravity_scale = requested_gravity_scale_;
            impedance_sequence = impedance_sequence_;
            gravity_sequence = gravity_sequence_;
        }

        if(gravity_sequence != applied_gravity_sequence) {
            const auto result = dynamics_->set_gravity_scale(gravity_scale);
            if(!result) {
                set_worker_error("Dynamics set_gravity_scale failed; DynamicsErr=" + std::to_string(static_cast<int>(result.error())));
                running_.store(false);
                break;
            }
            applied_gravity_sequence = gravity_sequence;
        }

        if(impedance_sequence != applied_impedance_sequence) {
            const auto result = robot_->set_impedance_mode(impedance_mode, now);
            if(!result) {
                set_worker_error(make_robot_error("Robot set_impedance_mode", result.error()));
                running_.store(false);
                break;
            }

            applied_impedance_mode = impedance_mode;
            applied_impedance_sequence = impedance_sequence;
            ref_pos_ = robot_->get_joint_state().pos;
            ref_vel_.assign(ref_pos_.size(), 0.0);

            std::lock_guard<std::mutex> lock(mutex_);
            if(!has_goal_) {
                goal_pos_ = ref_pos_;
            }
        }

        if(is_tracking_mode(applied_impedance_mode)) {
            update_reference(dt);
            const auto cmd_result = robot_->set_cmd(JointPosVelCmd{ ref_pos_, ref_vel_ }, now);
            if(!cmd_result) {
                set_worker_error(make_robot_error("Robot set_cmd", cmd_result.error()));
                running_.store(false);
                break;
            }
        }

        const auto cycle_result = robot_->cycle(now);
        if(!cycle_result) {
            set_worker_error(make_robot_error("Robot cycle", cycle_result.error()));
            running_.store(false);
            break;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            snapshot_.cycle = cycle_result.value();
            snapshot_.dynamics = dynamics_->get_state();
            snapshot_.robot_state = robot_->get_state();
            snapshot_.valid = true;
            snapshot_.last_error.clear();
        }

        next_time += period;
        std::this_thread::sleep_until(next_time);
    }

    if(robot_ && robot_->get_state() == RobotState::ACTIVE) {
        const auto result = robot_->deactivate();
        if(!result) {
            set_worker_error(make_robot_error("Robot deactivate", result.error()));
        }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.robot_state = robot_ ? robot_->get_state() : RobotState::UNCONFIGURED;
}

/**
 * @brief 根据当前目标更新梯形位置速度参考
 * @param dt 当前参考生成时间步长
 */
void PyRobotSession::update_reference(double dt) {
    JointVector goal;
    double speed_scale;
    bool has_goal;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        goal = goal_pos_;
        speed_scale = speed_scale_;
        has_goal = has_goal_;
    }

    if(!has_goal) {
        ref_vel_.assign(ref_vel_.size(), 0.0);
        return;
    }

    for(std::size_t i = 0; i < goal.size(); ++i) {
        const double error = goal[i] - ref_pos_[i];
        const double max_vel = cfg_.safety.limits.max_vel[i] * speed_scale;
        const double max_acc = cfg_.safety.limits.max_acc[i];
        const double direction = error >= 0.0 ? 1.0 : -1.0;
        const double braking_vel = std::sqrt(std::max(0.0, 2.0 * max_acc * std::abs(error)));
        const double desired_vel = direction * std::min(max_vel, braking_vel);
        const double max_delta_vel = max_acc * dt;
        const double next_vel = ref_vel_[i] + std::clamp(desired_vel - ref_vel_[i], -max_delta_vel, max_delta_vel);
        const double next_pos = ref_pos_[i] + next_vel * dt;
        const bool crossed_target = (goal[i] - ref_pos_[i]) * (goal[i] - next_pos) <= 0.0;

        if(crossed_target) {
            ref_pos_[i] = goal[i];
            ref_vel_[i] += std::clamp(-ref_vel_[i], -max_delta_vel, max_delta_vel);
        }
        else {
            ref_pos_[i] = next_pos;
            ref_vel_[i] = next_vel;
        }
    }
}

/**
 * @brief 校验 Python 输入关节向量的长度和有限性
 * @param values 待校验关节向量
 * @param name 参数名称
 */
void PyRobotSession::validate_joint_vector(const JointVector& values, const char* name) const {
    if(!configured_) {
        throw DmArmPythonError("RobotSession is not configured");
    }
    if(values.size() != cfg_.joint_names.size()) {
        throw DmArmPythonError(std::string(name) + " size mismatch");
    }
    for(double value : values) {
        if(!std::isfinite(value)) {
            throw DmArmPythonError(std::string(name) + " contains NaN or Inf");
        }
    }
}

/**
 * @brief 将周期线程错误写入会话快照
 * @param message 错误文本
 */
void PyRobotSession::set_worker_error(const std::string& message) noexcept {
    try {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_.last_error = message;
        snapshot_.robot_state = robot_ ? robot_->get_state() : RobotState::UNCONFIGURED;
    }
    catch(...) {
    }
}

/**
 * @brief 创建 Robot 使用的动力学前馈回调
 * @return 根据前馈模式返回零向量、重力补偿或逆动力学结果的回调
 */
ModelFeedforwardFn PyRobotSession::make_model_feedforward() {
    return [this](auto mode, const auto& state, const auto& acc, const auto& ref_acc, double) -> tl::expected<JointVector, ModelFeedforwardErr> {
        const auto result = dynamics_->update(state, acc, ref_acc);
        if(!result) {
            return tl::make_unexpected(ModelFeedforwardErr::COMPUTE_FAILED);
        }
        if(mode == ModelFeedforwardMode::GRAVITY) {
            return dynamics_->get_gravity_compensation();
        }
        if(mode == ModelFeedforwardMode::FULL_INVERSE_DYNAMICS) {
            return dynamics_->get_inverse_dynamics();
        }
        return JointVector(cfg_.joint_names.size(), 0.0);
        };
}

} // namespace dm_arm
