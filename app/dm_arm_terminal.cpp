#include "dm_arm/config/config.hpp"
#include "dm_arm/hardware/motor_bus.hpp"
#include "dm_arm/robot.hpp"

#ifdef DM_ARM_CLI_HAS_DAMIAO
#include "dm_arm/hardware/damiao_motor_bus.hpp"
#endif

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#ifndef DM_ARM_DEFAULT_CONFIG_PATH
#define DM_ARM_DEFAULT_CONFIG_PATH "config/dm_arm.yaml"
#endif

namespace {

using namespace dm_arm;

constexpr std::size_t kInvalidIndex = std::numeric_limits<std::size_t>::max();

struct CliOptions {
    std::string config_path{ DM_ARM_DEFAULT_CONFIG_PATH };
    std::string backend{ "fake" };
    bool allow_hardware{ false };
    bool show_help{ false };
};

std::string to_string(RobotState value) {
    switch(value) {
        case RobotState::UNCONFIGURED: return "UNCONFIGURED";
        case RobotState::INACTIVE: return "INACTIVE";
        case RobotState::ACTIVE: return "ACTIVE";
        case RobotState::FAULT: return "FAULT";
    }
    return "UNKNOWN";
}

std::string to_string(JointImpedanceMode value) {
    switch(value) {
        case JointImpedanceMode::RIGID_HOLD: return "RIGID_HOLD";
        case JointImpedanceMode::RIGID_TRACKING: return "RIGID_TRACKING";
        case JointImpedanceMode::COMPLIANT_HOLD: return "COMPLIANT_HOLD";
        case JointImpedanceMode::COMPLIANT_DRAG: return "COMPLIANT_DRAG";
        case JointImpedanceMode::COMPLIANT_TRACKING: return "COMPLIANT_TRACKING";
    }
    return "UNKNOWN";
}

std::string to_string(ModelFeedforwardMode value) {
    switch(value) {
        case ModelFeedforwardMode::NONE: return "NONE";
        case ModelFeedforwardMode::GRAVITY: return "GRAVITY";
        case ModelFeedforwardMode::FULL_INVERSE_DYNAMICS: return "FULL_INVERSE_DYNAMICS";
    }
    return "UNKNOWN";
}

std::string to_string(RobotErr value) {
    switch(value) {
        case RobotErr::NOT_CONFIGURED: return "NOT_CONFIGURED";
        case RobotErr::ALREADY_CONFIGURED: return "ALREADY_CONFIGURED";
        case RobotErr::INVALID_CFG: return "INVALID_CFG";
        case RobotErr::NULL_MOTOR_BUS: return "NULL_MOTOR_BUS";
        case RobotErr::MOTOR_BUS_SIZE_MISMATCH: return "MOTOR_BUS_SIZE_MISMATCH";
        case RobotErr::WRITE_DISABLED: return "WRITE_DISABLED";
        case RobotErr::NOT_ACTIVE: return "NOT_ACTIVE";
        case RobotErr::ALREADY_ACTIVE: return "ALREADY_ACTIVE";
        case RobotErr::FAULTED: return "FAULTED";
        case RobotErr::NOT_FAULTED: return "NOT_FAULTED";
        case RobotErr::INVALID_TIME: return "INVALID_TIME";
        case RobotErr::MOTOR_BUS_CONNECT_FAILED: return "MOTOR_BUS_CONNECT_FAILED";
        case RobotErr::MOTOR_BUS_ACTIVATE_FAILED: return "MOTOR_BUS_ACTIVATE_FAILED";
        case RobotErr::MOTOR_BUS_READ_FAILED: return "MOTOR_BUS_READ_FAILED";
        case RobotErr::MOTOR_BUS_WRITE_FAILED: return "MOTOR_BUS_WRITE_FAILED";
        case RobotErr::MOTOR_BUS_DEACTIVATE_FAILED: return "MOTOR_BUS_DEACTIVATE_FAILED";
        case RobotErr::MOTOR_BUS_RECOVER_FAILED: return "MOTOR_BUS_RECOVER_FAILED";
        case RobotErr::MAPPER_FAILED: return "MAPPER_FAILED";
        case RobotErr::CTRLLER_FAILED: return "CTRLLER_FAILED";
        case RobotErr::SAFETY_FAILED: return "SAFETY_FAILED";
        case RobotErr::MODEL_FEEDFORWARD_FAILED: return "MODEL_FEEDFORWARD_FAILED";
        case RobotErr::INVALID_MODEL_FEEDFORWARD: return "INVALID_MODEL_FEEDFORWARD";
    }
    return "UNKNOWN";
}

std::string to_string(MotorBusErr value) {
    switch(value) {
        case MotorBusErr::NOT_CONFIGURED: return "NOT_CONFIGURED";
        case MotorBusErr::NOT_CONNECTED: return "NOT_CONNECTED";
        case MotorBusErr::NOT_ACTIVE: return "NOT_ACTIVE";
        case MotorBusErr::INVALID_CFG: return "INVALID_CFG";
        case MotorBusErr::OPEN_FAILED: return "OPEN_FAILED";
        case MotorBusErr::READ_FAILED: return "READ_FAILED";
        case MotorBusErr::WRITE_FAILED: return "WRITE_FAILED";
        case MotorBusErr::INVALID_STATE: return "INVALID_STATE";
        case MotorBusErr::INVALID_CMD: return "INVALID_CMD";
        case MotorBusErr::ACTUATOR_OFFLINE: return "ACTUATOR_OFFLINE";
        case MotorBusErr::ACTUATOR_FAULT: return "ACTUATOR_FAULT";
        case MotorBusErr::TIMEOUT: return "TIMEOUT";
        case MotorBusErr::ENABLE_FAILED: return "ENABLE_FAILED";
        case MotorBusErr::MODE_SWITCH_FAILED: return "MODE_SWITCH_FAILED";
        case MotorBusErr::STOP_FAILED: return "STOP_FAILED";
        case MotorBusErr::DISABLE_FAILED: return "DISABLE_FAILED";
        case MotorBusErr::RECOVER_FAILED: return "RECOVER_FAILED";
    }
    return "UNKNOWN";
}

std::string to_string(JointCtrllerErr value) {
    switch(value) {
        case JointCtrllerErr::OK: return "OK";
        case JointCtrllerErr::NOT_CONFIGURED: return "NOT_CONFIGURED";
        case JointCtrllerErr::NOT_INITIALIZED: return "NOT_INITIALIZED";
        case JointCtrllerErr::ALREADY_INITIALIZED: return "ALREADY_INITIALIZED";
        case JointCtrllerErr::INVALID_CFG: return "INVALID_CFG";
        case JointCtrllerErr::INVALID_STATE: return "INVALID_STATE";
        case JointCtrllerErr::INVALID_DT: return "INVALID_DT";
        case JointCtrllerErr::INVALID_MODEL_FEEDFORWARD: return "INVALID_MODEL_FEEDFORWARD";
        case JointCtrllerErr::INVALID_IMPEDANCE_MODE: return "INVALID_IMPEDANCE_MODE";
        case JointCtrllerErr::INVALID_CMD_SIZE: return "INVALID_CMD_SIZE";
        case JointCtrllerErr::INVALID_CMD_VALUE: return "INVALID_CMD_VALUE";
        case JointCtrllerErr::INVALID_FULL_CMD: return "INVALID_FULL_CMD";
        case JointCtrllerErr::CMD_NOT_ALLOWED_IN_MODE: return "CMD_NOT_ALLOWED_IN_MODE";
        case JointCtrllerErr::FULL_CMD_NOT_ALLOWED: return "FULL_CMD_NOT_ALLOWED";
    }
    return "UNKNOWN";
}

std::string to_string(JointActuatorMapErr value) {
    switch(value) {
        case JointActuatorMapErr::OK: return "OK";
        case JointActuatorMapErr::NOT_CONFIGURED: return "NOT_CONFIGURED";
        case JointActuatorMapErr::INVALID_CFG: return "INVALID_CFG";
        case JointActuatorMapErr::INVALID_JOINT_STATE: return "INVALID_JOINT_STATE";
        case JointActuatorMapErr::INVALID_ACTUATOR_STATE: return "INVALID_ACTUATOR_STATE";
        case JointActuatorMapErr::INVALID_JOINT_CMD: return "INVALID_JOINT_CMD";
        case JointActuatorMapErr::INVALID_ACTUATOR_CMD: return "INVALID_ACTUATOR_CMD";
        case JointActuatorMapErr::INVALID_CONVERSION_VALUE: return "INVALID_CONVERSION_VALUE";
    }
    return "UNKNOWN";
}

std::string to_string(SafetyErr value) {
    switch(value) {
        case SafetyErr::NOT_CONFIGURED: return "NOT_CONFIGURED";
        case SafetyErr::INVALID_CFG: return "INVALID_CFG";
        case SafetyErr::INVALID_DT: return "INVALID_DT";
        case SafetyErr::INVALID_STATE_AGE: return "INVALID_STATE_AGE";
        case SafetyErr::INVALID_CMD_AGE: return "INVALID_CMD_AGE";
        case SafetyErr::STATE_TIMEOUT: return "STATE_TIMEOUT";
        case SafetyErr::CMD_TIMEOUT: return "CMD_TIMEOUT";
        case SafetyErr::INVALID_JOINT_STATE_SIZE: return "INVALID_JOINT_STATE_SIZE";
        case SafetyErr::INVALID_ACTUATOR_STATE_SIZE: return "INVALID_ACTUATOR_STATE_SIZE";
        case SafetyErr::NON_FINITE_JOINT_STATE: return "NON_FINITE_JOINT_STATE";
        case SafetyErr::NON_FINITE_ACTUATOR_STATE: return "NON_FINITE_ACTUATOR_STATE";
        case SafetyErr::JOINT_POS_LIMIT: return "JOINT_POS_LIMIT";
        case SafetyErr::JOINT_VEL_LIMIT: return "JOINT_VEL_LIMIT";
        case SafetyErr::ACTUATOR_OFFLINE: return "ACTUATOR_OFFLINE";
        case SafetyErr::ACTUATOR_NOT_ENABLED: return "ACTUATOR_NOT_ENABLED";
        case SafetyErr::ACTUATOR_FAULT: return "ACTUATOR_FAULT";
        case SafetyErr::INVALID_CMD_SIZE: return "INVALID_CMD_SIZE";
        case SafetyErr::NON_FINITE_CMD: return "NON_FINITE_CMD";
        case SafetyErr::CMD_POS_LIMIT: return "CMD_POS_LIMIT";
        case SafetyErr::CMD_VEL_LIMIT: return "CMD_VEL_LIMIT";
        case SafetyErr::CMD_EFFORT_LIMIT: return "CMD_EFFORT_LIMIT";
        case SafetyErr::CMD_KP_LIMIT: return "CMD_KP_LIMIT";
        case SafetyErr::CMD_KD_LIMIT: return "CMD_KD_LIMIT";
        case SafetyErr::CMD_POS_STEP_LIMIT: return "CMD_POS_STEP_LIMIT";
        case SafetyErr::CMD_VEL_STEP_LIMIT: return "CMD_VEL_STEP_LIMIT";
    }
    return "UNKNOWN";
}

std::string to_string(ModelFeedforwardErr value) {
    switch(value) {
        case ModelFeedforwardErr::NOT_CONFIGURED: return "NOT_CONFIGURED";
        case ModelFeedforwardErr::INVALID_INPUT: return "INVALID_INPUT";
        case ModelFeedforwardErr::COMPUTE_FAILED: return "COMPUTE_FAILED";
    }
    return "UNKNOWN";
}

void print_vector(const std::string& name, const std::vector<double>& values) {
    std::cout << std::left << std::setw(14) << name << " [";
    for(std::size_t i = 0; i < values.size(); ++i) {
        if(i != 0) std::cout << ", ";
        std::cout << std::fixed << std::setprecision(5) << values[i];
    }
    std::cout << "]\n";
}

void print_bytes(const std::string& name, const std::vector<std::uint8_t>& values) {
    std::cout << std::left << std::setw(14) << name << " [";
    for(std::size_t i = 0; i < values.size(); ++i) {
        if(i != 0) std::cout << ", ";
        std::cout << static_cast<int>(values[i]);
    }
    std::cout << "]\n";
}

void print_ints(const std::string& name, const std::vector<int>& values) {
    std::cout << std::left << std::setw(14) << name << " [";
    for(std::size_t i = 0; i < values.size(); ++i) {
        if(i != 0) std::cout << ", ";
        std::cout << values[i];
    }
    std::cout << "]\n";
}

void print_fault(const RobotFault& fault) {
    std::cout << "RobotFault: " << to_string(fault.code) << '\n';
    switch(fault.code) {
        case RobotErr::MOTOR_BUS_CONNECT_FAILED:
        case RobotErr::MOTOR_BUS_ACTIVATE_FAILED:
        case RobotErr::MOTOR_BUS_READ_FAILED:
        case RobotErr::MOTOR_BUS_WRITE_FAILED:
        case RobotErr::MOTOR_BUS_DEACTIVATE_FAILED:
        case RobotErr::MOTOR_BUS_RECOVER_FAILED:
            std::cout << "  MotorBusErr: " << to_string(fault.motor_bus_err) << '\n';
            break;
        case RobotErr::MAPPER_FAILED:
            std::cout << "  MapperErr: " << to_string(fault.mapper_err) << '\n';
            break;
        case RobotErr::CTRLLER_FAILED:
            std::cout << "  CtrllerErr: " << to_string(fault.ctrller_err) << '\n';
            break;
        case RobotErr::SAFETY_FAILED: {
            const SafetyErr code = fault.safety_fault.code;
            const bool has_global_value =
                code == SafetyErr::INVALID_DT ||
                code == SafetyErr::INVALID_STATE_AGE ||
                code == SafetyErr::INVALID_CMD_AGE ||
                code == SafetyErr::STATE_TIMEOUT ||
                code == SafetyErr::CMD_TIMEOUT;

            std::cout << "  SafetyErr: " << to_string(code);
            if(fault.safety_fault.index != kInvalidIndex) {
                std::cout << ", index=" << fault.safety_fault.index;
            }
            if(fault.safety_fault.index != kInvalidIndex || has_global_value) {
                std::cout << ", value=" << fault.safety_fault.value
                    << ", limit=" << fault.safety_fault.limit;
            }
            std::cout << '\n';
            break;
        }
        case RobotErr::MODEL_FEEDFORWARD_FAILED:
        case RobotErr::INVALID_MODEL_FEEDFORWARD:
            std::cout << "  ModelFeedforwardErr: "
                << to_string(fault.model_feedforward_err) << '\n';
            break;
        default:
            break;
    }
}

std::vector<double> joint_to_actuator_pos(
    const RobotCfg& cfg,
    const std::vector<double>& joint_pos) {
    std::vector<double> result(joint_pos.size(), 0.0);
    for(std::size_t i = 0; i < result.size(); ++i) {
        result[i] = cfg.mapper.actuator_zero_offset[i]
            + static_cast<double>(cfg.mapper.direction[i])
            * cfg.mapper.pos_ratio[i]
            * (joint_pos[i] - cfg.mapper.joint_zero_offset[i]);
    }
    return result;
}

class FakeMotorBus final : public MotorBus {
public:
    explicit FakeMotorBus(const RobotCfg& cfg)
        : size_(cfg.joint_names.size()) {
        std::vector<double> initial_joint_pos(size_, 0.0);
        for(std::size_t i = 0; i < size_; ++i) {
            const double low = cfg.safety.limits.min_pos[i]
                + cfg.safety.limits.pos_margin[i];
            const double high = cfg.safety.limits.max_pos[i]
                - cfg.safety.limits.pos_margin[i];
            initial_joint_pos[i] = 0.5 * (low + high);
        }

        state_.pos = joint_to_actuator_pos(cfg, initial_joint_pos);
        state_.vel.assign(size_, 0.0);
        state_.tor.assign(size_, 0.0);
        state_.online.assign(size_, 0);
        state_.enabled.assign(size_, 0);
        state_.err_code.assign(size_, 0);

        last_cmd_.pos = state_.pos;
        last_cmd_.vel.assign(size_, 0.0);
        last_cmd_.tor.assign(size_, 0.0);
        last_cmd_.kp.assign(size_, 0.0);
        last_cmd_.kd.assign(size_, 0.0);
    }

    tl::expected<void, MotorBusErr> connect() override {
        if(fail_connect_) return tl::make_unexpected(MotorBusErr::OPEN_FAILED);
        connected_ = true;
        refresh_status_flags();
        return {};
    }

    tl::expected<ActuatorState, MotorBusErr> read() override {
        if(!connected_) return tl::make_unexpected(MotorBusErr::NOT_CONNECTED);
        if(fail_read_) return tl::make_unexpected(MotorBusErr::READ_FAILED);

        if(active_ && has_written_) {
            state_.pos = last_cmd_.pos;
            state_.vel = last_cmd_.vel;
            state_.tor = last_cmd_.tor;
        }
        refresh_status_flags();
        return state_;
    }

    tl::expected<void, MotorBusErr> activate() override {
        if(!connected_) return tl::make_unexpected(MotorBusErr::NOT_CONNECTED);
        if(fail_activate_) return tl::make_unexpected(MotorBusErr::ACTUATOR_FAULT);
        active_ = true;
        refresh_status_flags();
        return {};
    }

    tl::expected<void, MotorBusErr> write(const ActuatorMitCmd& cmd) override {
        if(!active_) return tl::make_unexpected(MotorBusErr::NOT_ACTIVE);
        if(fail_write_) return tl::make_unexpected(MotorBusErr::WRITE_FAILED);
        if(cmd.pos.size() != size_ || cmd.vel.size() != size_ ||
            cmd.tor.size() != size_ || cmd.kp.size() != size_ ||
            cmd.kd.size() != size_) {
            return tl::make_unexpected(MotorBusErr::INVALID_CMD);
        }
        last_cmd_ = cmd;
        has_written_ = true;
        ++write_count_;
        return {};
    }

    tl::expected<void, MotorBusErr> stop() override {
        if(!connected_) return tl::make_unexpected(MotorBusErr::NOT_CONNECTED);
        ++stop_count_;
        last_cmd_.pos = state_.pos;
        std::fill(last_cmd_.vel.begin(), last_cmd_.vel.end(), 0.0);
        std::fill(last_cmd_.tor.begin(), last_cmd_.tor.end(), 0.0);
        std::fill(last_cmd_.kp.begin(), last_cmd_.kp.end(), 0.0);
        std::fill(last_cmd_.kd.begin(), last_cmd_.kd.end(), 0.0);
        has_written_ = true;
        return {};
    }

    tl::expected<void, MotorBusErr> deactivate() override {
        if(!connected_) return tl::make_unexpected(MotorBusErr::NOT_CONNECTED);
        if(fail_deactivate_) return tl::make_unexpected(MotorBusErr::DISABLE_FAILED);
        active_ = false;
        refresh_status_flags();
        ++deactivate_count_;
        return {};
    }

    tl::expected<void, MotorBusErr> recover() override {
        if(fail_connect_) return tl::make_unexpected(MotorBusErr::RECOVER_FAILED);
        connected_ = true;
        active_ = false;
        has_written_ = false;
        clear_injections();
        refresh_status_flags();
        return {};
    }

    void cleanup() noexcept override {
        active_ = false;
        connected_ = false;
        refresh_status_flags();
    }

    std::size_t size() const noexcept override { return size_; }

    void inject_offline(std::size_t index) {
        offline_index_ = index < size_ ? std::optional<std::size_t>(index)
            : std::nullopt;
        refresh_status_flags();
    }

    void inject_fault(std::size_t index, int code) {
        fault_index_ = index < size_ ? std::optional<std::size_t>(index)
            : std::nullopt;
        fault_code_ = code;
        refresh_status_flags();
    }

    void set_fail_connect(bool value) noexcept { fail_connect_ = value; }
    void set_fail_activate(bool value) noexcept { fail_activate_ = value; }
    void set_fail_read(bool value) noexcept { fail_read_ = value; }
    void set_fail_write(bool value) noexcept { fail_write_ = value; }
    void set_fail_deactivate(bool value) noexcept { fail_deactivate_ = value; }

    void clear_injections() noexcept {
        fail_connect_ = false;
        fail_activate_ = false;
        fail_read_ = false;
        fail_write_ = false;
        fail_deactivate_ = false;
        offline_index_.reset();
        fault_index_.reset();
        fault_code_ = 0;
        refresh_status_flags();
    }

    const ActuatorMitCmd& last_cmd() const noexcept { return last_cmd_; }
    std::uint64_t write_count() const noexcept { return write_count_; }
    std::uint64_t stop_count() const noexcept { return stop_count_; }
    std::uint64_t deactivate_count() const noexcept { return deactivate_count_; }

private:
    void refresh_status_flags() noexcept {
        for(std::size_t i = 0; i < size_; ++i) {
            state_.online[i] = connected_ ? 1 : 0;
            state_.enabled[i] = active_ ? 1 : 0;
            state_.err_code[i] = 0;
        }
        if(offline_index_) state_.online[*offline_index_] = 0;
        if(fault_index_) state_.err_code[*fault_index_] = fault_code_;
    }

private:
    std::size_t size_{ 0 };
    ActuatorState state_;
    ActuatorMitCmd last_cmd_;

    bool connected_{ false };
    bool active_{ false };
    bool has_written_{ false };

    bool fail_connect_{ false };
    bool fail_activate_{ false };
    bool fail_read_{ false };
    bool fail_write_{ false };
    bool fail_deactivate_{ false };

    std::optional<std::size_t> offline_index_;
    std::optional<std::size_t> fault_index_;
    int fault_code_{ 0 };

    std::uint64_t write_count_{ 0 };
    std::uint64_t stop_count_{ 0 };
    std::uint64_t deactivate_count_{ 0 };
};

enum class StreamKind {
    NONE,
    SAFE_POSITION_TARGET,
    JOINT_POS,
    JOINT_POS_VEL,
    JOINT_POS_VEL_TOR,
    FULL_CMD,
};

struct StreamState {
    StreamKind kind{ StreamKind::NONE };
    JointVector target_pos;
    JointVector streamed_pos;
    JointVector streamed_vel;
    double speed_scale{ 0.5 };
    Robot::TimePoint last_update_time{};
    bool has_last_update_time{ false };
    JointCmd joint_cmd{ JointPosCmd{} };
    JointCtrlCmd full_cmd;
};

class TerminalApp {
public:
    TerminalApp(RobotCfg cfg, std::string backend)
        : cfg_(std::move(cfg)), backend_(std::move(backend)) {}

    bool configure_robot(std::unique_ptr<MotorBus> bus, FakeMotorBus* fake_bus) {
        fake_bus_ = fake_bus;

        ModelFeedforwardFn feedforward;
        if(cfg_.runtime.model_feedforward_mode != ModelFeedforwardMode::NONE) {
            if(backend_ != "fake") {
                std::cerr << "当前尚未实现真实 Dynamics，真机后端只允许 NONE 前馈模式\n";
                return false;
            }
            std::cout << "[提示] Fake 后端使用零向量模拟 "
                << to_string(cfg_.runtime.model_feedforward_mode)
                << " 前馈回调，仅测试 API 接线\n";
            feedforward = [](ModelFeedforwardMode,
                const JointState& state,
                double) -> tl::expected<JointVector, ModelFeedforwardErr> {
                    return JointVector(state.pos.size(), 0.0);
                };
        }

        const auto result = robot_.configure(cfg_, std::move(bus), std::move(feedforward));
        if(!result) {
            std::cerr << "Robot configure() 失败：\n";
            print_fault(result.error());
            return false;
        }
        return true;
    }

    int run() {
        print_banner();
        worker_ = std::thread(&TerminalApp::control_loop, this);

        while(!exit_.load()) {
            print_menu();
            const auto choice = read_int("请选择: ");
            if(!choice) {
                if(std::cin.eof()) {
                    exit_.store(true);
                    break;
                }
                std::cout << "输入无效\n";
                continue;
            }
            handle_menu(*choice);
        }

        auto_cycle_.store(false);
        exit_.store(true);
        if(worker_.joinable()) worker_.join();

        std::lock_guard<std::mutex> lock(robot_mutex_);
        if(robot_.get_state() == RobotState::ACTIVE) {
            const auto result = robot_.deactivate();
            if(!result) {
                std::cerr << "退出时 deactivate() 失败：\n";
                print_fault(result.error());
            }
        }
        return 0;
    }

private:
    void print_banner() const {
        std::cout << "\n==============================================\n"
            << " DM-Arm Terminal Main\n"
            << " backend: " << backend_ << '\n'
            << " config : " << config_path_display_ << '\n'
            << "==============================================\n";
        if(backend_ == "damiao") {
            std::cout << "[危险] 当前是真机后端；切换模式和发送命令前必须确认机械臂已支撑、限位和零位正确\n";
        }
    }

    void print_menu() const {
        std::cout << "\n------------ 主菜单 ------------\n"
            << " 1. 查看 Robot 状态与 getter 输出\n"
            << " 2. activate()\n"
            << " 3. deactivate()\n"
            << " 4. reset_fault()\n"
            << " 5. 切换阻抗模式\n"
            << " 6. 安全流式移动到 " << cfg_.joint_names.size() << " 轴目标位置\n"
            << " 7. 安全流式相对移动\n"
            << " 8. 测试 JointPosCmd 输入\n"
            << " 9. 测试 JointPosVelCmd 输入\n"
            << "10. 测试 JointPosVelTorCmd 输入\n"
            << "11. 测试 set_full_cmd()\n"
            << "12. 取消当前输入流并切回当前位置保持\n"
            << "13. 启动/暂停后台 cycle()（Fake；真机 ACTIVE 时禁止暂停）\n"
            << "14. 手动执行单次 cycle()（仅 Fake）\n"
            << "15. 查看最近一次周期完整输入/输出\n"
            << "16. 查看配置摘要\n"
            << "17. FakeBus 故障注入\n"
            << "18. 清除 FakeBus 故障\n"
            << "19. 运行离线 API 全流程演示\n"
            << " 0. 安全退出\n";
    }

    void handle_menu(int choice) {
        switch(choice) {
            case 0: exit_.store(true); break;
            case 1: show_status(); break;
            case 2: activate(); break;
            case 3: deactivate(); break;
            case 4: reset_fault(); break;
            case 5: switch_mode(); break;
            case 6: set_safe_absolute_target(); break;
            case 7: set_safe_relative_target(); break;
            case 8: queue_joint_pos(); break;
            case 9: queue_joint_pos_vel(); break;
            case 10: queue_joint_pos_vel_tor(); break;
            case 11: queue_full_cmd(); break;
            case 12: cancel_stream_and_hold(); break;
            case 13: toggle_auto_cycle(); break;
            case 14: single_cycle(); break;
            case 15: show_last_cycle(); break;
            case 16: show_config(); break;
            case 17: inject_fake_fault(); break;
            case 18: clear_fake_faults(); break;
            case 19: run_api_demo(); break;
            default: std::cout << "未知菜单项\n"; break;
        }
    }

    void activate() {
        std::lock_guard<std::mutex> lock(robot_mutex_);
        const auto result = robot_.activate();
        if(!result) {
            std::cout << "activate() 失败：\n";
            print_fault(result.error());
            return;
        }
        clear_stream_locked();
        last_output_.reset();
        auto_cycle_.store(true);
        std::cout << "activate() 成功，后台 cycle() 已启动\n";
    }

    void deactivate() {
        auto_cycle_.store(false);
        std::lock_guard<std::mutex> lock(robot_mutex_);
        clear_stream_locked();
        last_output_.reset();
        const auto result = robot_.deactivate();
        if(!result) {
            std::cout << "deactivate() 失败：\n";
            print_fault(result.error());
            return;
        }
        std::cout << "deactivate() 成功\n";
    }

    void reset_fault() {
        auto_cycle_.store(false);
        std::lock_guard<std::mutex> lock(robot_mutex_);
        clear_stream_locked();
        last_output_.reset();
        const auto result = robot_.reset_fault();
        if(!result) {
            std::cout << "reset_fault() 失败：\n";
            print_fault(result.error());
            return;
        }
        std::cout << "reset_fault() 成功，Robot 回到 INACTIVE\n";
    }

    void switch_mode() {
        std::cout << "\n1 RIGID_HOLD\n"
            << "2 RIGID_TRACKING\n"
            << "3 COMPLIANT_HOLD\n"
            << "4 COMPLIANT_DRAG\n"
            << "5 COMPLIANT_TRACKING\n";
        const auto value = read_int("模式: ");
        if(!value || *value < 1 || *value > 5) {
            std::cout << "模式输入无效\n";
            return;
        }
        const JointImpedanceMode mode = static_cast<JointImpedanceMode>(*value - 1);

        std::lock_guard<std::mutex> lock(robot_mutex_);
        clear_stream_locked();
        const auto result = robot_.set_impedance_mode(mode);
        if(!result) {
            std::cout << "set_impedance_mode() 失败：\n";
            print_fault(result.error());
            return;
        }
        // Robot::set_impedance_mode() 会把 Safety 命令历史重置到当前实测状态
        // 清除旧周期输出，保证下一段轨迹使用同一参考基准
        last_output_.reset();
        std::cout << "模式已切换为 " << to_string(mode) << "\n";
    }

    void set_safe_absolute_target() {
        const auto target = read_joint_vector(
            "输入 " + std::to_string(joint_count()) +
            " 个目标关节位置（旋转关节 rad，移动关节 m），以空格分隔: ");
        if(!target) return;
        const auto scale = read_double("速度比例 (0, 1]，建议 0.3~1.0: ");
        if(!scale || *scale <= 0.0 || *scale > 1.0) {
            std::cout << "速度比例无效\n";
            return;
        }

        std::lock_guard<std::mutex> lock(robot_mutex_);
        if(robot_.get_state() != RobotState::ACTIVE) {
            std::cout << "Robot 不是 ACTIVE\n";
            return;
        }
        if(!is_tracking_mode(robot_.get_impedance_mode())) {
            std::cout << "请先切换到 RIGID_TRACKING 或 COMPLIANT_TRACKING\n";
            return;
        }
        if(!target_inside_soft_limits(*target)) return;

        start_safe_position_stream_locked(*target, *scale);
    }

    void set_safe_relative_target() {
        const auto delta = read_joint_vector(
            "输入 " + std::to_string(joint_count()) +
            " 个相对关节位移（旋转关节 rad，移动关节 m），以空格分隔: ");
        if(!delta) return;
        const auto scale = read_double("速度比例 (0, 1]，建议 0.3~1.0: ");
        if(!scale || *scale <= 0.0 || *scale > 1.0) {
            std::cout << "速度比例无效\n";
            return;
        }

        std::lock_guard<std::mutex> lock(robot_mutex_);
        if(robot_.get_state() != RobotState::ACTIVE) {
            std::cout << "Robot 不是 ACTIVE\n";
            return;
        }
        if(!is_tracking_mode(robot_.get_impedance_mode())) {
            std::cout << "请先切换到 Tracking 模式\n";
            return;
        }

        JointVector target = robot_.get_joint_state().pos;
        for(std::size_t i = 0; i < target.size(); ++i) target[i] += (*delta)[i];
        if(!target_inside_soft_limits(target)) return;

        start_safe_position_stream_locked(std::move(target), *scale);
    }

    void queue_joint_pos() {
        const auto pos = read_joint_vector(
            "输入 JointPosCmd.pos " + std::to_string(joint_count()) + " 个值: ");
        if(!pos) return;
        JointPosCmd cmd{ *pos };
        queue_joint_command(JointCmd{ cmd }, StreamKind::JOINT_POS);
    }

    void queue_joint_pos_vel() {
        const auto pos = read_joint_vector(
            "输入 pos " + std::to_string(joint_count()) + " 个值: ");
        if(!pos) return;
        const auto vel = read_joint_vector(
            "输入 vel " + std::to_string(joint_count()) + " 个值: ");
        if(!vel) return;
        JointPosVelCmd cmd{ *pos, *vel };
        queue_joint_command(JointCmd{ cmd }, StreamKind::JOINT_POS_VEL);
    }

    void queue_joint_pos_vel_tor() {
        const auto pos = read_joint_vector(
            "输入 pos " + std::to_string(joint_count()) + " 个值: ");
        if(!pos) return;
        const auto vel = read_joint_vector(
            "输入 vel " + std::to_string(joint_count()) + " 个值: ");
        if(!vel) return;
        const auto tor = read_joint_vector(
            "输入 tor " + std::to_string(joint_count()) + " 个值: ");
        if(!tor) return;
        JointPosVelTorCmd cmd{ *pos, *vel, *tor };
        queue_joint_command(JointCmd{ cmd }, StreamKind::JOINT_POS_VEL_TOR);
    }

    void queue_full_cmd() {
        const auto pos = read_joint_vector(
            "输入 full.pos " + std::to_string(joint_count()) + " 个值: ");
        if(!pos) return;
        const auto vel = read_joint_vector(
            "输入 full.vel " + std::to_string(joint_count()) + " 个值: ");
        if(!vel) return;
        const auto tor = read_joint_vector(
            "输入 full.tor " + std::to_string(joint_count()) + " 个值: ");
        if(!tor) return;
        const auto kp = read_joint_vector(
            "输入 full.kp " + std::to_string(joint_count()) + " 个值: ");
        if(!kp) return;
        const auto kd = read_joint_vector(
            "输入 full.kd " + std::to_string(joint_count()) + " 个值: ");
        if(!kd) return;

        std::lock_guard<std::mutex> lock(robot_mutex_);
        if(robot_.get_state() != RobotState::ACTIVE) {
            std::cout << "Robot 不是 ACTIVE\n";
            return;
        }
        stream_.kind = StreamKind::FULL_CMD;
        stream_.full_cmd = JointCtrlCmd{ *pos, *vel, *tor, *kp, *kd };
        std::cout << "已排队 set_full_cmd()；若 controller.allow_full_cmd=false，会返回预期错误\n";
    }

    void queue_joint_command(JointCmd cmd, StreamKind kind) {
        std::lock_guard<std::mutex> lock(robot_mutex_);
        if(robot_.get_state() != RobotState::ACTIVE) {
            std::cout << "Robot 不是 ACTIVE\n";
            return;
        }
        stream_.kind = kind;
        stream_.joint_cmd = std::move(cmd);
        std::cout << "命令已排队并会在每个控制周期重复发送；明显跳变会由 Safety 拒绝\n";
    }

    void cancel_stream_and_hold() {
        std::lock_guard<std::mutex> lock(robot_mutex_);
        clear_stream_locked();
        if(robot_.get_state() != RobotState::ACTIVE) {
            std::cout << "输入流已清除；Robot 当前不是 ACTIVE\n";
            return;
        }
        const auto result = robot_.set_impedance_mode(JointImpedanceMode::RIGID_HOLD);
        if(!result) {
            std::cout << "切换 RIGID_HOLD 失败：\n";
            print_fault(result.error());
            return;
        }
        std::cout << "输入流已清除，并切换到 RIGID_HOLD\n";
    }

    void toggle_auto_cycle() {
        std::lock_guard<std::mutex> lock(robot_mutex_);
        const RobotState state = robot_.get_state();

        if(state != RobotState::ACTIVE) {
            auto_cycle_.store(false);
            std::cout << "Robot 当前不是 ACTIVE；无需预先启动 cycle()；activate() 成功后会自动启动后台周期\n";
            return;
        }

        if(backend_ == "damiao" && auto_cycle_.load()) {
            std::cout << "真机 ACTIVE 状态下禁止暂停后台 cycle()；如需停止，请先执行 deactivate()\n";
            return;
        }

        const bool new_value = !auto_cycle_.load();
        auto_cycle_.store(new_value);
        std::cout << "后台 cycle(): " << (new_value ? "RUNNING" : "PAUSED") << '\n';
    }

    void single_cycle() {
        if(backend_ == "damiao") {
            std::cout << "真机后端不提供手动单周期运行；请保持后台 cycle() 连续运行\n";
            return;
        }
        if(auto_cycle_.load()) {
            std::cout << "请先暂停后台 cycle()，避免重复周期\n";
            return;
        }
        std::lock_guard<std::mutex> lock(robot_mutex_);
        run_one_cycle_locked(Robot::Clock::now(), true);
    }

    void show_status() {
        RobotState state;
        JointImpedanceMode mode;
        ModelFeedforwardMode feedforward_mode;
        JointState joint;
        ActuatorState actuator;
        tl::optional<RobotFault> last_fault;
        StreamKind stream_kind;
        std::uint64_t fake_writes = 0;
        std::uint64_t fake_stops = 0;
        std::uint64_t fake_deactivates = 0;

        {
            std::lock_guard<std::mutex> lock(robot_mutex_);
            state = robot_.get_state();
            mode = robot_.get_impedance_mode();
            feedforward_mode = robot_.get_model_feedforward_mode();
            joint = robot_.get_joint_state();
            actuator = robot_.get_actuator_state();
            last_fault = robot_.get_last_fault();
            stream_kind = stream_.kind;
            if(fake_bus_) {
                fake_writes = fake_bus_->write_count();
                fake_stops = fake_bus_->stop_count();
                fake_deactivates = fake_bus_->deactivate_count();
            }
        }

        std::cout << "\nRobotState          : " << to_string(state) << '\n'
            << "ImpedanceMode       : " << to_string(mode) << '\n'
            << "ModelFeedforward    : " << to_string(feedforward_mode) << '\n'
            << "Auto cycle          : " << (auto_cycle_.load() ? "RUNNING" : "PAUSED") << '\n'
            << "Stream kind         : " << stream_name(stream_kind) << '\n';

        if(!joint.pos.empty()) {
            print_vector("joint.pos", joint.pos);
            print_vector("joint.vel", joint.vel);
            print_vector("joint.tor", joint.tor);
        }
        if(!actuator.pos.empty()) {
            print_vector("actuator.pos", actuator.pos);
            print_vector("actuator.vel", actuator.vel);
            print_vector("actuator.tor", actuator.tor);
            print_bytes("online", actuator.online);
            print_bytes("enabled", actuator.enabled);
            print_ints("err_code", actuator.err_code);
        }
        if(last_fault) print_fault(*last_fault);
        else std::cout << "Last fault          : none\n";

        if(fake_bus_) {
            std::cout << "Fake writes         : " << fake_writes << '\n'
                << "Fake stops          : " << fake_stops << '\n'
                << "Fake deactivates    : " << fake_deactivates << '\n';
        }
    }

    void show_last_cycle() {
        std::optional<RobotCycleOutput> output;
        {
            std::lock_guard<std::mutex> lock(robot_mutex_);
            output = last_output_;
        }
        if(!output) {
            std::cout << "尚无成功控制周期输出\n";
            return;
        }
        std::cout << "\ndt = " << std::fixed << std::setprecision(6)
            << output->dt << " s\n";
        print_vector("joint.pos", output->joint_state.pos);
        print_vector("joint.vel", output->joint_state.vel);
        print_vector("joint.tor", output->joint_state.tor);
        print_vector("cmd.pos", output->joint_cmd.pos);
        print_vector("cmd.vel", output->joint_cmd.vel);
        print_vector("cmd.tor", output->joint_cmd.tor);
        print_vector("cmd.kp", output->joint_cmd.kp);
        print_vector("cmd.kd", output->joint_cmd.kd);
        print_vector("act_cmd.pos", output->actuator_cmd.pos);
        print_vector("act_cmd.vel", output->actuator_cmd.vel);
        print_vector("act_cmd.tor", output->actuator_cmd.tor);
        print_vector("act_cmd.kp", output->actuator_cmd.kp);
        print_vector("act_cmd.kd", output->actuator_cmd.kd);
    }

    void show_config() const {
        std::cout << "\nJoint count         : " << cfg_.joint_names.size() << '\n'
            << "Control frequency   : " << cfg_.runtime.ctrl_frequency_hz << " Hz\n"
            << "Write enabled       : " << (cfg_.runtime.write_enabled ? "true" : "false") << '\n'
            << "Full cmd allowed    : " << (cfg_.ctrller.allow_full_cmd ? "true" : "false") << '\n'
            << "Feedforward mode    : " << to_string(cfg_.runtime.model_feedforward_mode) << '\n'
            << "Command timeout     : " << cfg_.safety.cmd_timeout_s << " s\n"
            << "State timeout       : " << cfg_.safety.state_timeout_s << " s\n"
            << "State vel ratio     : " << cfg_.safety.state_vel_fault_ratio << " x max_vel\n"
            << "Max dt              : " << cfg_.safety.max_dt_s << " s\n";
        print_vector("min_pos", cfg_.safety.limits.min_pos);
        print_vector("max_pos", cfg_.safety.limits.max_pos);
        print_vector("max_vel", cfg_.safety.limits.max_vel);
        print_vector("max_acc", cfg_.safety.limits.max_acc);
        print_vector("max_effort", cfg_.safety.limits.max_effort);
    }

    void inject_fake_fault() {
        if(!fake_bus_) {
            std::cout << "故障注入仅支持 FakeBus\n";
            return;
        }
        std::cout << "\n1 connect 失败\n"
            << "2 activate 失败\n"
            << "3 read 失败\n"
            << "4 write 失败\n"
            << "5 deactivate 失败\n"
            << "6 指定执行器离线\n"
            << "7 指定执行器错误码\n";
        const auto type = read_int("类型: ");
        if(!type) return;

        std::optional<int> index;
        std::optional<int> code;
        if(*type == 6 || *type == 7) {
            const std::size_t actuator_count = fake_bus_->size();
            if(actuator_count == 0) {
                std::cout << "当前没有可注入故障的执行器\n";
                return;
            }

            index = read_int(
                "执行器编号 1~" + std::to_string(actuator_count) + ": ");
            if(!index || *index < 1 ||
                static_cast<std::size_t>(*index) > actuator_count) {
                std::cout << "编号无效，有效范围为 1~"
                    << actuator_count << '\n';
                return;
            }
        }
        if(*type == 7) {
            code = read_int("错误码(非0): ");
            if(!code || *code == 0) {
                std::cout << "错误码无效\n";
                return;
            }
        }

        std::lock_guard<std::mutex> lock(robot_mutex_);
        switch(*type) {
            case 1: fake_bus_->set_fail_connect(true); break;
            case 2: fake_bus_->set_fail_activate(true); break;
            case 3: fake_bus_->set_fail_read(true); break;
            case 4: fake_bus_->set_fail_write(true); break;
            case 5: fake_bus_->set_fail_deactivate(true); break;
            case 6: fake_bus_->inject_offline(static_cast<std::size_t>(*index - 1)); break;
            case 7: fake_bus_->inject_fault(static_cast<std::size_t>(*index - 1), *code); break;
            default:
                std::cout << "类型无效\n";
                return;
        }
        std::cout << "故障已注入，下一相关 API/周期会体现结果\n";
    }

    void clear_fake_faults() {
        if(!fake_bus_) {
            std::cout << "当前不是 FakeBus\n";
            return;
        }
        std::lock_guard<std::mutex> lock(robot_mutex_);
        fake_bus_->clear_injections();
        std::cout << "FakeBus 故障已清除\n";
    }

    void run_api_demo() {
        if(backend_ != "fake") {
            std::cout << "离线 API 全流程演示只允许 Fake 后端\n";
            return;
        }

        std::cout << "\n[API Demo] 在独立 Robot/FakeBus 实例中运行，不影响当前 Robot\n";
        RobotCfg cfg = cfg_;
        cfg.runtime.write_enabled = true;
        cfg.runtime.model_feedforward_mode = ModelFeedforwardMode::NONE;
        cfg.ctrller.allow_full_cmd = true;

        Robot demo;
        auto bus = std::make_unique<FakeMotorBus>(cfg);
        FakeMotorBus* bus_ptr = bus.get();

        auto configured = demo.configure(cfg, std::move(bus));
        report_demo("configure", configured);
        if(!configured) return;

        auto duplicate_configure = demo.configure(cfg, std::make_unique<FakeMotorBus>(cfg));
        report_demo_expected_error("configure again", duplicate_configure, RobotErr::ALREADY_CONFIGURED);

        auto demo_now = Robot::Clock::now();
        const auto demo_period = std::chrono::duration_cast<Robot::Clock::duration>(
            std::chrono::duration<double>(1.0 / cfg.runtime.ctrl_frequency_hz));
        const auto next_cycle_time = [&]() {
            demo_now += demo_period;
            return demo_now;
            };

        auto activated = demo.activate();
        report_demo("activate", activated);
        if(!activated) return;

        // activate() 使用硬件激活完成时刻初始化内部时间基准
        demo_now = Robot::Clock::now();

        auto second_activate = demo.activate();
        report_demo_expected_error("activate again", second_activate, RobotErr::ALREADY_ACTIVE);

        std::cout << "  getter state=" << to_string(demo.get_state())
            << ", mode=" << to_string(demo.get_impedance_mode())
            << ", feedforward=" << to_string(demo.get_model_feedforward_mode()) << '\n';

        JointVector q = demo.get_joint_state().pos;
        const double tiny = 1.0e-4;

        for(const JointImpedanceMode mode : {
            JointImpedanceMode::RIGID_HOLD,
                JointImpedanceMode::RIGID_TRACKING,
                JointImpedanceMode::COMPLIANT_HOLD,
                JointImpedanceMode::COMPLIANT_DRAG,
                JointImpedanceMode::COMPLIANT_TRACKING}) {
            auto mode_result = demo.set_impedance_mode(mode, demo_now);
            report_demo("set mode " + to_string(mode), mode_result);
            if(!mode_result) continue;

            if(mode == JointImpedanceMode::RIGID_TRACKING ||
                mode == JointImpedanceMode::COMPLIANT_TRACKING) {
                q[0] += tiny;
                JointPosCmd pos_cmd{ q };
                auto set_pos = demo.set_cmd(JointCmd{ pos_cmd }, demo_now);
                report_demo("set JointPosCmd", set_pos);
                if(set_pos) report_demo("cycle JointPosCmd", demo.cycle(next_cycle_time()));

                JointPosVelCmd pos_vel_cmd{ q, JointVector(q.size(), 0.0) };
                auto set_pos_vel = demo.set_cmd(JointCmd{ pos_vel_cmd }, demo_now);
                report_demo("set JointPosVelCmd", set_pos_vel);
                if(set_pos_vel) report_demo("cycle JointPosVelCmd", demo.cycle(next_cycle_time()));

                JointPosVelTorCmd pos_vel_tor_cmd{
                    q, JointVector(q.size(), 0.0), JointVector(q.size(), 0.0) };
                auto set_pvt = demo.set_cmd(JointCmd{ pos_vel_tor_cmd }, demo_now);
                report_demo("set JointPosVelTorCmd", set_pvt);
                if(set_pvt) report_demo("cycle JointPosVelTorCmd", demo.cycle(next_cycle_time()));
            }
            else {
                auto forbidden = demo.set_cmd(JointCmd{ JointPosCmd{q} }, demo_now);
                report_demo_expected_error(
                    "set cmd in non-tracking mode",
                    forbidden,
                    RobotErr::CTRLLER_FAILED);
            }
        }

        JointCtrlCmd full;
        full.pos = q;
        full.vel.assign(q.size(), 0.0);
        full.tor.assign(q.size(), 0.0);
        full.kp.assign(q.size(), 0.0);
        full.kd.assign(q.size(), 0.0);
        auto full_result = demo.set_full_cmd(full, demo_now);
        report_demo("set_full_cmd", full_result);
        if(full_result) report_demo("cycle full cmd", demo.cycle(next_cycle_time()));

        bus_ptr->inject_offline(0);
        auto offline_cycle = demo.cycle(next_cycle_time());
        report_demo_expected_error("cycle with actuator offline", offline_cycle, RobotErr::SAFETY_FAILED);
        bus_ptr->clear_injections();

        auto reset = demo.reset_fault();
        report_demo("reset_fault", reset);
        auto reset_again = demo.reset_fault();
        report_demo_expected_error("reset_fault again", reset_again, RobotErr::NOT_FAULTED);

        auto inactive_cycle = demo.cycle(next_cycle_time());
        report_demo_expected_error("cycle while inactive", inactive_cycle, RobotErr::NOT_ACTIVE);

        std::cout << "[API Demo] 完成\n";
    }

    template<typename T>
    static void report_demo(const std::string& name, const tl::expected<T, RobotFault>& result) {
        std::cout << "  " << std::left << std::setw(34) << name
            << (result ? "PASS" : "FAIL") << '\n';
        if(!result) print_fault(result.error());
    }

    template<typename T>
    static void report_demo_expected_error(
        const std::string& name,
        const tl::expected<T, RobotFault>& result,
        RobotErr expected) {
        const bool pass = !result && result.error().code == expected;
        std::cout << "  " << std::left << std::setw(34) << name
            << (pass ? "PASS(expected error)" : "FAIL") << '\n';
        if(!pass && !result) print_fault(result.error());
    }

    void control_loop() {
        using Clock = Robot::Clock;
        const auto period = std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(1.0 / cfg_.runtime.ctrl_frequency_hz));
        auto next = Clock::now();

        while(!exit_.load()) {
            if(auto_cycle_.load()) {
                std::lock_guard<std::mutex> lock(robot_mutex_);
                if(robot_.get_state() == RobotState::ACTIVE) {
                    run_one_cycle_locked(Clock::now(), false);
                }
            }

            next += period;
            const auto now = Clock::now();
            if(next <= now) next = now + period;
            std::this_thread::sleep_until(next);
        }
    }

    void run_one_cycle_locked(Robot::TimePoint now, bool verbose) {
        const auto stream_result = apply_stream_locked(now);
        if(!stream_result) {
            if(verbose) {
                std::cout << "命令 API 调用失败：\n";
                print_fault(stream_result.error());
            }
            else {
                print_async_fault_once("stream", stream_result.error());
            }
            clear_stream_locked();
            return;
        }

        const auto result = robot_.cycle(now);
        if(!result) {
            if(verbose) {
                std::cout << "cycle() 失败：\n";
                print_fault(result.error());
            }
            else {
                print_async_fault_once("cycle", result.error());
            }
            clear_stream_locked();
            return;
        }
        last_output_ = result.value();
        last_async_fault_code_.reset();
        if(verbose) {
            std::cout << "cycle() 成功，dt=" << result->dt << " s\n";
        }
    }

    tl::expected<void, RobotFault> apply_stream_locked(Robot::TimePoint now) {
        switch(stream_.kind) {
            case StreamKind::NONE:
                return {};
            case StreamKind::SAFE_POSITION_TARGET: {
                const double nominal_dt = 1.0 / cfg_.runtime.ctrl_frequency_hz;
                double dt = nominal_dt;
                if(stream_.has_last_update_time && now > stream_.last_update_time) {
                    dt = std::chrono::duration<double>(
                        now - stream_.last_update_time).count();
                }
                stream_.last_update_time = now;
                stream_.has_last_update_time = true;

                // 轨迹生成与 Robot::cycle() 使用同一个 now这里限制最大步长，
                // 避免一次调度抖动把参考速度/位置推进过多；若实际周期超过
                // Safety::max_dt_s，Robot::cycle() 仍会按 Safety 规则报告 INVALID_DT
                dt = std::clamp(dt, 1.0e-6, cfg_.safety.max_dt_s);

                const double pos_tolerance = std::max(
                    1.0e-4,
                    cfg_.safety.numeric_tolerance * 100.0);
                bool done = true;

                for(std::size_t i = 0; i < stream_.streamed_pos.size(); ++i) {
                    const double error =
                        stream_.target_pos[i] - stream_.streamed_pos[i];
                    const double max_vel =
                        cfg_.safety.limits.max_vel[i] * stream_.speed_scale;
                    const double max_acc = cfg_.safety.limits.max_acc[i];
                    const double max_delta_vel = max_acc * dt;
                    const double vel_tolerance = std::max(
                        1.0e-4,
                        cfg_.safety.numeric_tolerance * 100.0);
                    const double current_vel = stream_.streamed_vel[i];

                    if(std::abs(error) <= pos_tolerance &&
                        std::abs(current_vel) <= vel_tolerance) {
                        stream_.streamed_pos[i] = stream_.target_pos[i];
                        stream_.streamed_vel[i] = 0.0;
                        continue;
                    }

                    done = false;

                    // 由剩余距离得到当前允许的制动速度距离越小，目标速度越低，
                    // 从源头避免到达目标时再把速度一帧清零
                    const double direction = error > 0.0 ? 1.0 :
                        (error < 0.0 ? -1.0 : 0.0);
                    const double braking_speed = std::sqrt(
                        std::max(0.0, 2.0 * max_acc * std::abs(error)));
                    const double target_vel = direction *
                        std::min(max_vel, braking_speed);

                    double next_vel = current_vel + std::clamp(
                        target_vel - current_vel,
                        -max_delta_vel,
                        max_delta_vel);
                    double next_pos = stream_.streamed_pos[i] +
                        0.5 * (current_vel + next_vel) * dt;

                    const double next_error = stream_.target_pos[i] - next_pos;
                    const bool crossed_target =
                        (error > 0.0 && next_error < 0.0) ||
                        (error < 0.0 && next_error > 0.0);

                    if(crossed_target || std::abs(error) <= pos_tolerance) {
                        // 位置可以落在目标点，但速度只能按 max_acc 逐步降到 0
                        // 原实现这里直接 streamed_vel=0，正是
                        // CMD_VEL_STEP_LIMIT 的根因
                        next_pos = stream_.target_pos[i];
                        next_vel = current_vel + std::clamp(
                            -current_vel,
                            -max_delta_vel,
                            max_delta_vel);
                    }

                    if(std::abs(next_vel) <= vel_tolerance &&
                        std::abs(stream_.target_pos[i] - next_pos) <= pos_tolerance) {
                        next_pos = stream_.target_pos[i];
                        next_vel = 0.0;
                    }

                    stream_.streamed_pos[i] = next_pos;
                    stream_.streamed_vel[i] = next_vel;
                }

                JointPosVelCmd cmd{
                    stream_.streamed_pos,
                    stream_.streamed_vel
                };
                const auto result = robot_.set_cmd(JointCmd{ cmd }, now);
                if(result && done) {
                    JointVector zero_vel(stream_.target_pos.size(), 0.0);
                    stream_.kind = StreamKind::JOINT_POS_VEL;
                    stream_.joint_cmd = JointCmd{ JointPosVelCmd{
                        stream_.target_pos,
                        std::move(zero_vel)
                    } };
                }
                return result;
            }
            case StreamKind::JOINT_POS:
            case StreamKind::JOINT_POS_VEL:
            case StreamKind::JOINT_POS_VEL_TOR:
                return robot_.set_cmd(stream_.joint_cmd, now);
            case StreamKind::FULL_CMD:
                return robot_.set_full_cmd(stream_.full_cmd, now);
        }
        return {};
    }

    void print_async_fault_once(const std::string& source, const RobotFault& fault) {
        if(last_async_fault_code_ && *last_async_fault_code_ == fault.code) return;
        last_async_fault_code_ = fault.code;
        std::lock_guard<std::mutex> io_lock(io_mutex_);
        std::cout << "\n[后台 " << source << " 失败]\n";
        print_fault(fault);
        std::cout << "请输入菜单编号继续\n";
    }

    void start_safe_position_stream_locked(JointVector target, double speed_scale) {
        JointVector start_pos;
        JointVector start_vel;
        bool continued_from_last_cmd = false;

        // Safety 的单周期连续性基准是“上一帧已接受并实际发送的命令”，
        // 因此中途重定向时必须从同一个命令参考继续规划，不能突然退回
        // 当前实测位置；机械臂存在跟踪滞后时，实测位置与命令位置可能
        // 相差数度，直接用实测位置重建轨迹会制造假的 CMD_POS_STEP_LIMIT
        if(last_output_ &&
            last_output_->joint_cmd.pos.size() == cfg_.joint_names.size() &&
            last_output_->joint_cmd.vel.size() == cfg_.joint_names.size()) {
            start_pos = last_output_->joint_cmd.pos;
            start_vel = last_output_->joint_cmd.vel;
            continued_from_last_cmd = true;
        }
        else {
            start_pos = robot_.get_joint_state().pos;
            start_vel.assign(start_pos.size(), 0.0);
        }

        const JointVector actual_pos = robot_.get_joint_state().pos;
        double max_reference_lag = 0.0;
        std::size_t max_reference_lag_index = kInvalidIndex;
        if(actual_pos.size() == start_pos.size()) {
            for(std::size_t i = 0; i < start_pos.size(); ++i) {
                const double lag = std::abs(start_pos[i] - actual_pos[i]);
                if(lag > max_reference_lag) {
                    max_reference_lag = lag;
                    max_reference_lag_index = i;
                }
            }
        }

        stream_ = StreamState{};
        stream_.kind = StreamKind::SAFE_POSITION_TARGET;
        stream_.target_pos = std::move(target);
        stream_.streamed_pos = std::move(start_pos);
        stream_.streamed_vel = std::move(start_vel);
        stream_.speed_scale = speed_scale;
        stream_.last_update_time = Robot::Clock::now();
        stream_.has_last_update_time = true;

        std::cout << "已开始梯形速度参考，速度上限比例=" << speed_scale;
        if(continued_from_last_cmd) {
            std::cout << "，从上一帧已发送参考连续重规划";
        }
        else {
            std::cout << "，从当前实测状态起步";
        }
        std::cout << "\n";

        if(max_reference_lag > 0.05) {
            std::cout << "[提示] " << joint_name(max_reference_lag_index)
                << " 当前命令-实测位置滞后="
                << std::fixed << std::setprecision(4)
                << max_reference_lag
                << "（关节位置单位）；参考会保持连续，但这说明该关节跟踪不足；"
                "请用菜单 15 对比 joint.pos 与 cmd.pos\n";
        }
    }

    bool target_inside_soft_limits(const JointVector& target) const {
        for(std::size_t i = 0; i < target.size(); ++i) {
            const double low = cfg_.safety.limits.min_pos[i]
                + cfg_.safety.limits.pos_margin[i];
            const double high = cfg_.safety.limits.max_pos[i]
                - cfg_.safety.limits.pos_margin[i];
            if(!std::isfinite(target[i]) || target[i] < low || target[i] > high) {
                std::cout << joint_name(i) << " 目标 " << target[i]
                    << " 超出命令软限位 [" << low << ", " << high << "]\n";
                return false;
            }
        }
        return true;
    }

    static bool is_tracking_mode(JointImpedanceMode mode) {
        return mode == JointImpedanceMode::RIGID_TRACKING ||
            mode == JointImpedanceMode::COMPLIANT_TRACKING;
    }

    static std::string stream_name(StreamKind kind) {
        switch(kind) {
            case StreamKind::NONE: return "NONE";
            case StreamKind::SAFE_POSITION_TARGET: return "SAFE_POSITION_TARGET";
            case StreamKind::JOINT_POS: return "JOINT_POS";
            case StreamKind::JOINT_POS_VEL: return "JOINT_POS_VEL";
            case StreamKind::JOINT_POS_VEL_TOR: return "JOINT_POS_VEL_TOR";
            case StreamKind::FULL_CMD: return "FULL_CMD";
        }
        return "UNKNOWN";
    }

    void clear_stream_locked() {
        stream_ = StreamState{};
    }

    static std::optional<int> read_int(const std::string& prompt) {
        std::cout << prompt;
        std::string line;
        if(!std::getline(std::cin, line)) return std::nullopt;
        std::istringstream input(line);
        int value = 0;
        char extra = '\0';
        if(!(input >> value) || (input >> extra)) return std::nullopt;
        return value;
    }

    static std::optional<double> read_double(const std::string& prompt) {
        std::cout << prompt;
        std::string line;
        if(!std::getline(std::cin, line)) return std::nullopt;
        std::istringstream input(line);
        double value = 0.0;
        char extra = '\0';
        if(!(input >> value) || (input >> extra) || !std::isfinite(value)) return std::nullopt;
        return value;
    }

    std::optional<JointVector> read_joint_vector(
        const std::string& prompt) const {
        std::cout << prompt;
        std::string line;
        if(!std::getline(std::cin, line)) return std::nullopt;

        std::istringstream input(line);
        JointVector values;
        double value = 0.0;
        while(input >> value) values.push_back(value);

        const bool parsed_all = input.eof();
        const bool valid_size = values.size() == joint_count();
        const bool all_finite = std::all_of(
            values.begin(),
            values.end(),
            [](double v) { return std::isfinite(v); });

        if(!parsed_all || !valid_size || !all_finite) {
            std::cout << "必须输入 " << joint_count()
                << " 个有限浮点数\n";
            return std::nullopt;
        }
        return values;
    }

    std::size_t joint_count() const noexcept {
        return cfg_.joint_names.size();
    }

    std::string joint_name(std::size_t index) const {
        if(index < cfg_.joint_names.size()) {
            return cfg_.joint_names[index];
        }
        return "joint[" + std::to_string(index) + "]";
    }

public:
    void set_config_path_display(std::string path) {
        config_path_display_ = std::move(path);
    }

private:
    RobotCfg cfg_;
    std::string backend_;
    std::string config_path_display_;

    Robot robot_;
    FakeMotorBus* fake_bus_{ nullptr };

    mutable std::mutex robot_mutex_;
    mutable std::mutex io_mutex_;
    std::thread worker_;
    std::atomic<bool> exit_{ false };
    std::atomic<bool> auto_cycle_{ false };

    StreamState stream_;
    std::optional<RobotCycleOutput> last_output_;
    std::optional<RobotErr> last_async_fault_code_;
};

void print_usage(const char* program) {
    std::cout << "Usage: " << program << " [options]\n"
        << "  --config <path>          YAML 配置文件\n"
        << "  --backend fake|damiao    后端，默认 fake\n"
        << "  --allow-hardware         允许创建达妙真机后端\n"
        << "  --help                   显示帮助\n\n"
        << "真机启用同时要求：\n"
        << "  1) 编译时 -DDM_ARM_BUILD_DAMIAO=ON\n"
        << "  2) 启动时 --backend damiao --allow-hardware\n"
        << "  3) YAML runtime.write_enabled: true\n";
}

std::optional<CliOptions> parse_options(int argc, char** argv) {
    CliOptions options;
    for(int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if(arg == "--help") {
            options.show_help = true;
            return options;
        }
        if(arg == "--config") {
            if(i + 1 >= argc) {
                std::cerr << "--config 缺少路径\n";
                return std::nullopt;
            }
            options.config_path = argv[++i];
            continue;
        }
        if(arg == "--backend") {
            if(i + 1 >= argc) {
                std::cerr << "--backend 缺少值\n";
                return std::nullopt;
            }
            options.backend = argv[++i];
            if(options.backend != "fake" && options.backend != "damiao") {
                std::cerr << "backend 只能是 fake 或 damiao\n";
                return std::nullopt;
            }
            continue;
        }
        if(arg == "--allow-hardware") {
            options.allow_hardware = true;
            continue;
        }
        std::cerr << "未知参数: " << arg << '\n';
        return std::nullopt;
    }
    return options;
}

} // namespace

int main(int argc, char** argv) {
    const auto options = parse_options(argc, argv);
    if(!options) return EXIT_FAILURE;
    if(options->show_help) {
        print_usage(argv[0]);
        return EXIT_SUCCESS;
    }

    auto loaded = dm_arm::load_robot_cfg(options->config_path);
    if(!loaded) {
        std::cerr << "加载配置失败: " << loaded.error().message << '\n';
        return EXIT_FAILURE;
    }

    dm_arm::RobotCfg cfg = loaded.value();
    std::unique_ptr<dm_arm::MotorBus> bus;
    FakeMotorBus* fake_bus = nullptr;

    if(options->backend == "fake") {
        cfg.runtime.write_enabled = true;
        auto fake = std::make_unique<FakeMotorBus>(cfg);
        fake_bus = fake.get();
        bus = std::move(fake);
    }
    else {
        if(!options->allow_hardware) {
            std::cerr << "拒绝创建真机后端：请显式增加 --allow-hardware\n";
            return EXIT_FAILURE;
        }
#ifdef DM_ARM_CLI_HAS_DAMIAO
        auto damiao = std::make_unique<dm_arm::DamiaoMotorBus>();
        const auto configured = damiao->configure(cfg.damiao);
        if(!configured) {
            std::cerr << "DamiaoMotorBus configure() 失败: "
                << to_string(configured.error()) << '\n';
            return EXIT_FAILURE;
        }
        bus = std::move(damiao);
#else
        std::cerr << "当前构建没有启用达妙后端，请使用 -DDM_ARM_BUILD_DAMIAO=ON 重新构建\n";
        return EXIT_FAILURE;
#endif
    }

    TerminalApp app(std::move(cfg), options->backend);
    app.set_config_path_display(options->config_path);
    if(!app.configure_robot(std::move(bus), fake_bus)) return EXIT_FAILURE;
    return app.run();
}
