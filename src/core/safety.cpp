#include "dm_arm/core/safety.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace dm_arm {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //

namespace {

/**
 * @brief SafetyFault 构造函数
 * @param code Safety 错误码
 * @param index 关节/执行器索引，若不适用则为 std::numeric_limits<std::size_t>::max()
 * @param value 关节/执行器状态或命令值，若不适用则为 0.0
 * @param limit Safety 配置中对应的限制值，若不适用则为 0.0
 * @return SafetyFault 实例
 */
SafetyFault fault(SafetyErr code, std::size_t index = std::numeric_limits<std::size_t>::max(), double value = 0.0, double limit = 0.0) {
    return SafetyFault{ code, index, value, limit };
}

} // namespace

// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

/**
 * @brief 配置 Safety
 * @param cfg Safety 配置
 * @return 配置成功返回空，配置失败返回 SafetyFault
 */
tl::expected<void, SafetyFault> Safety::configure(const SafetyCfg& cfg) {
    const auto valid = validate_cfg(cfg);
    if(!valid) {
        configured_ = false;
        clear_cmd_history();
        return tl::make_unexpected(valid.error());
    }

    cfg_ = cfg;
    configured_ = true;
    clamp_count_ = 0;
    clear_cmd_history();
    return {};
}

/**
 * @brief 检查 Joint/Actuator 状态
 * @param joint_state Joint 侧状态
 * @param actuator_state Actuator 侧状态
 * @param state_age_s 状态数据的时间戳与当前时间的差值，单位秒
 * @return 状态合法返回空，状态不合法返回 SafetyFault
 */
tl::expected<void, SafetyFault> Safety::check_state(
    const JointState& joint_state,
    const ActuatorState& actuator_state,
    double state_age_s) const {
    if(!configured_) {
        return tl::make_unexpected(fault(SafetyErr::NOT_CONFIGURED));
    }

    if(!std::isfinite(state_age_s) || state_age_s < 0.0) {
        return tl::make_unexpected(fault(SafetyErr::INVALID_STATE_AGE, std::numeric_limits<std::size_t>::max(), state_age_s, cfg_.state_timeout_s));
    }
    if(state_age_s > cfg_.state_timeout_s) {
        return tl::make_unexpected(fault(SafetyErr::STATE_TIMEOUT, std::numeric_limits<std::size_t>::max(), state_age_s, cfg_.state_timeout_s));
    }

    if(!has_joint_size(joint_state)) {
        return tl::make_unexpected(fault(SafetyErr::INVALID_JOINT_STATE_SIZE));
    }
    if(!has_actuator_size(actuator_state)) {
        return tl::make_unexpected(fault(SafetyErr::INVALID_ACTUATOR_STATE_SIZE));
    }

    if(!is_finite_vector(joint_state.pos) || !is_finite_vector(joint_state.vel) || !is_finite_vector(joint_state.tor)) {
        return tl::make_unexpected(fault(SafetyErr::NON_FINITE_JOINT_STATE));
    }
    if(!is_finite_vector(actuator_state.pos) || !is_finite_vector(actuator_state.vel) || !is_finite_vector(actuator_state.tor)) {
        return tl::make_unexpected(fault(SafetyErr::NON_FINITE_ACTUATOR_STATE));
    }

    const double tolerance = cfg_.numeric_tolerance;
    for(std::size_t i = 0; i < cfg_.joints_count; ++i) {
        if(cfg_.require_all_actuators_online && actuator_state.online[i] == 0) {
            return tl::make_unexpected(fault(SafetyErr::ACTUATOR_OFFLINE, i, static_cast<double>(actuator_state.online[i]), 1.0));
        }
        if(cfg_.require_all_actuators_enabled && actuator_state.enabled[i] == 0) {
            return tl::make_unexpected(fault(SafetyErr::ACTUATOR_NOT_ENABLED, i, static_cast<double>(actuator_state.enabled[i]), 1.0));
        }
        if(actuator_state.err_code[i] != 0) {
            return tl::make_unexpected(fault(SafetyErr::ACTUATOR_FAULT, i, static_cast<double>(actuator_state.err_code[i]), 0.0));
        }

        if(joint_state.pos[i] < cfg_.limits.min_pos[i] - tolerance) {
            return tl::make_unexpected(fault(SafetyErr::JOINT_POS_LIMIT, i, joint_state.pos[i], cfg_.limits.min_pos[i]));
        }
        if(joint_state.pos[i] > cfg_.limits.max_pos[i] + tolerance) {
            return tl::make_unexpected(fault(SafetyErr::JOINT_POS_LIMIT, i, joint_state.pos[i], cfg_.limits.max_pos[i]));
        }
        const double state_vel_fault_limit =
            cfg_.limits.max_vel[i] * cfg_.state_vel_fault_ratio;
        if(std::abs(joint_state.vel[i]) > state_vel_fault_limit + tolerance) {
            return tl::make_unexpected(fault(SafetyErr::JOINT_VEL_LIMIT, i, joint_state.vel[i], state_vel_fault_limit));
        }
    }

    return {};
}

/**
 * @brief 检查跟踪命令是否超时
 * @param cmd_age_s 距离上一帧合法命令的时间
 * @return 命令合法返回空，命令不合法返回 SafetyFault
 */
tl::expected<void, SafetyFault> Safety::check_cmd_age(double cmd_age_s) const {
    if(!configured_) {
        return tl::make_unexpected(fault(SafetyErr::NOT_CONFIGURED));
    }
    if(!std::isfinite(cmd_age_s) || cmd_age_s < 0.0) {
        return tl::make_unexpected(fault(SafetyErr::INVALID_CMD_AGE, std::numeric_limits<std::size_t>::max(), cmd_age_s, cfg_.cmd_timeout_s));
    }
    if(cmd_age_s > cfg_.cmd_timeout_s) {
        return tl::make_unexpected(fault(SafetyErr::CMD_TIMEOUT, std::numeric_limits<std::size_t>::max(), cmd_age_s, cfg_.cmd_timeout_s));
    }
    return {};
}

/**
 * @brief 检查并返回可安全下发的 JointCtrlCmd
 * @param state 当前 Joint 侧状态
 * @param cmd 上层下发的 JointCtrlCmd
 * @param dt 当前控制周期
 * @return 命令合法返回安全命令，命令不合法返回 SafetyFault
 */
tl::expected<JointCtrlCmd, SafetyFault> Safety::check_joint_cmd(const JointState& state, const JointCtrlCmd& cmd, double dt) {
    if(!configured_) {
        return tl::make_unexpected(fault(SafetyErr::NOT_CONFIGURED));
    }
    if(!std::isfinite(dt) || dt <= 0.0 || dt > cfg_.max_dt_s) {
        return tl::make_unexpected(fault(SafetyErr::INVALID_DT, std::numeric_limits<std::size_t>::max(), dt, cfg_.max_dt_s));
    }
    if(!has_joint_size(state)) {
        return tl::make_unexpected(fault(SafetyErr::INVALID_JOINT_STATE_SIZE));
    }
    if(!is_finite_vector(state.pos) || !is_finite_vector(state.vel)) {
        return tl::make_unexpected(fault(SafetyErr::NON_FINITE_JOINT_STATE));
    }
    if(!has_cmd_size(cmd)) {
        return tl::make_unexpected(fault(SafetyErr::INVALID_CMD_SIZE));
    }
    if(!is_finite_vector(cmd.pos) || !is_finite_vector(cmd.vel) || !is_finite_vector(cmd.tor) ||
        !is_finite_vector(cmd.kp) || !is_finite_vector(cmd.kd)) {
        return tl::make_unexpected(fault(SafetyErr::NON_FINITE_CMD));
    }

    JointCtrlCmd safe_cmd = cmd;

    for(std::size_t i = 0; i < cfg_.joints_count; ++i) {
        const double cmd_min_pos = cfg_.limits.min_pos[i] + cfg_.limits.pos_margin[i];
        const double cmd_max_pos = cfg_.limits.max_pos[i] - cfg_.limits.pos_margin[i];

        if(!clamp_small_violation(safe_cmd.pos[i], cmd_min_pos, cmd_max_pos)) {
            const double limit = safe_cmd.pos[i] < cmd_min_pos ? cmd_min_pos : cmd_max_pos;
            return tl::make_unexpected(fault(SafetyErr::CMD_POS_LIMIT, i, safe_cmd.pos[i], limit));
        }
        if(!clamp_small_violation(safe_cmd.vel[i], -cfg_.limits.max_vel[i], cfg_.limits.max_vel[i])) {
            return tl::make_unexpected(fault(SafetyErr::CMD_VEL_LIMIT, i, safe_cmd.vel[i], cfg_.limits.max_vel[i]));
        }
        if(!clamp_small_violation(safe_cmd.tor[i], -cfg_.limits.max_effort[i], cfg_.limits.max_effort[i])) {
            return tl::make_unexpected(fault(SafetyErr::CMD_EFFORT_LIMIT, i, safe_cmd.tor[i], cfg_.limits.max_effort[i]));
        }
        if(!clamp_small_violation(safe_cmd.kp[i], 0.0, cfg_.limits.max_kp[i])) {
            return tl::make_unexpected(fault(SafetyErr::CMD_KP_LIMIT, i, safe_cmd.kp[i], cfg_.limits.max_kp[i]));
        }
        if(!clamp_small_violation(safe_cmd.kd[i], 0.0, cfg_.limits.max_kd[i])) {
            return tl::make_unexpected(fault(SafetyErr::CMD_KD_LIMIT, i, safe_cmd.kd[i], cfg_.limits.max_kd[i]));
        }
    }

    const JointVector& reference_pos = has_last_accepted_cmd_ ? last_accepted_cmd_.pos : state.pos;
    const JointVector& reference_vel = has_last_accepted_cmd_ ? last_accepted_cmd_.vel : state.vel;

    for(std::size_t i = 0; i < cfg_.joints_count; ++i) {
        if(safe_cmd.kp[i] > cfg_.numeric_tolerance) {
            const double max_pos_delta = cfg_.limits.max_vel[i] * dt + 0.5 * cfg_.limits.max_acc[i] * dt * dt;
            const double pos_delta = safe_cmd.pos[i] - reference_pos[i];
            if(std::abs(pos_delta) > max_pos_delta + cfg_.numeric_tolerance) {
                return tl::make_unexpected(fault(SafetyErr::CMD_POS_STEP_LIMIT, i, pos_delta, max_pos_delta));
            }
        }

        if(safe_cmd.kd[i] > cfg_.numeric_tolerance) {
            const double max_vel_delta = cfg_.limits.max_acc[i] * dt;
            const double vel_delta = safe_cmd.vel[i] - reference_vel[i];
            if(std::abs(vel_delta) > max_vel_delta + cfg_.numeric_tolerance) {
                return tl::make_unexpected(fault(SafetyErr::CMD_VEL_STEP_LIMIT, i, vel_delta, max_vel_delta));
            }
        }
    }

    last_accepted_cmd_ = safe_cmd;
    has_last_accepted_cmd_ = true;
    return safe_cmd;
}

/**
 * @brief 使用当前位置和零目标速度初始化/重置命令历史
 * @param state 当前 Joint 侧状态
 * @return 成功返回空，失败返回 SafetyFault
 */
tl::expected<void, SafetyFault> Safety::reset_cmd_history(const JointState& state) {
    if(!configured_) {
        return tl::make_unexpected(fault(SafetyErr::NOT_CONFIGURED));
    }
    if(!has_joint_size(state)) {
        return tl::make_unexpected(fault(SafetyErr::INVALID_JOINT_STATE_SIZE));
    }
    if(!is_finite_vector(state.pos) || !is_finite_vector(state.vel)) {
        return tl::make_unexpected(fault(SafetyErr::NON_FINITE_JOINT_STATE));
    }

    last_accepted_cmd_.pos = state.pos;

    last_accepted_cmd_.vel.assign(cfg_.joints_count, 0.0);
    last_accepted_cmd_.tor.assign(cfg_.joints_count, 0.0);
    last_accepted_cmd_.kp.assign(cfg_.joints_count, 0.0);
    last_accepted_cmd_.kd.assign(cfg_.joints_count, 0.0);
    has_last_accepted_cmd_ = true;
    return {};
}

/**
 * @brief 清空命令历史
 */
void Safety::clear_cmd_history() noexcept {
    last_accepted_cmd_ = JointCtrlCmd{};
    has_last_accepted_cmd_ = false;
}

/**
 * @brief 将安全错误映射为 Robot 应执行的动作
 * @param err Safety 错误码
 * @return Robot 应执行的动作
 */
SafetyAction Safety::action_for(SafetyErr err) const noexcept {
    switch(err) {
        case SafetyErr::INVALID_DT:
        case SafetyErr::INVALID_CMD_AGE:
        case SafetyErr::CMD_TIMEOUT:
        case SafetyErr::INVALID_CMD_SIZE:
        case SafetyErr::NON_FINITE_CMD:
        case SafetyErr::CMD_POS_LIMIT:
        case SafetyErr::CMD_VEL_LIMIT:
        case SafetyErr::CMD_EFFORT_LIMIT:
        case SafetyErr::CMD_KP_LIMIT:
        case SafetyErr::CMD_KD_LIMIT:
        case SafetyErr::CMD_POS_STEP_LIMIT:
        case SafetyErr::CMD_VEL_STEP_LIMIT:
        case SafetyErr::JOINT_VEL_LIMIT:
        case SafetyErr::STATE_TIMEOUT:
        case SafetyErr::JOINT_POS_LIMIT:
        case SafetyErr::ACTUATOR_OFFLINE:
            return SafetyAction::STOP_HOLD;

        case SafetyErr::NOT_CONFIGURED:
        case SafetyErr::INVALID_CFG:
        case SafetyErr::INVALID_STATE_AGE:
        case SafetyErr::INVALID_JOINT_STATE_SIZE:
        case SafetyErr::INVALID_ACTUATOR_STATE_SIZE:
        case SafetyErr::NON_FINITE_JOINT_STATE:
        case SafetyErr::NON_FINITE_ACTUATOR_STATE:
        case SafetyErr::ACTUATOR_NOT_ENABLED:
        case SafetyErr::ACTUATOR_FAULT:
            return SafetyAction::DISABLE;
    }

    return SafetyAction::DISABLE;
}

/**
 * @brief 检查 Safety 是否已配置
 * @return Safety 是否已配置
 */
bool Safety::is_configured() const noexcept {
    return configured_;
}

/**
 * @brief 限幅次数
 * @return Safety 限幅次数
 */
std::uint64_t Safety::clamp_count() const noexcept {
    return clamp_count_;
}

// ! ========================= 私 有 类 方 法 实 现 ========================= ! //

/**
 * @brief 验证 Safety 配置
 * @param cfg Safety 配置
 * @return 配置合法返回空，配置不合法返回 SafetyFault
 */
tl::expected<void, SafetyFault> Safety::validate_cfg(const SafetyCfg& cfg) const {
    if(cfg.joints_count == 0) {
        return tl::make_unexpected(fault(SafetyErr::INVALID_CFG));
    }

    const auto size_is_n = [&cfg](const JointVector& values) {
        return values.size() == cfg.joints_count;
        };
    if(!size_is_n(cfg.limits.min_pos) || !size_is_n(cfg.limits.max_pos) ||
        !size_is_n(cfg.limits.max_vel) || !size_is_n(cfg.limits.max_acc) || !size_is_n(cfg.limits.max_effort) ||
        !size_is_n(cfg.limits.max_kp) || !size_is_n(cfg.limits.max_kd) || !size_is_n(cfg.limits.pos_margin)) {
        return tl::make_unexpected(fault(SafetyErr::INVALID_CFG));
    }

    const std::array<const JointVector*, 8> vectors = {
        &cfg.limits.min_pos,
        &cfg.limits.max_pos,
        &cfg.limits.max_vel,
        &cfg.limits.max_acc,
        &cfg.limits.max_effort,
        &cfg.limits.max_kp,
        &cfg.limits.max_kd,
        &cfg.limits.pos_margin,
    };
    for(const auto* values : vectors) {
        if(!is_finite_vector(*values)) {
            return tl::make_unexpected(fault(SafetyErr::INVALID_CFG));
        }
    }

    if(!std::isfinite(cfg.cmd_timeout_s) || cfg.cmd_timeout_s <= 0.0 ||
        !std::isfinite(cfg.state_timeout_s) || cfg.state_timeout_s <= 0.0 ||
        !std::isfinite(cfg.max_dt_s) || cfg.max_dt_s <= 0.0 ||
        !std::isfinite(cfg.numeric_tolerance) || cfg.numeric_tolerance < 0.0 ||
        !std::isfinite(cfg.state_vel_fault_ratio) || cfg.state_vel_fault_ratio < 1.0) {
        return tl::make_unexpected(fault(SafetyErr::INVALID_CFG));
    }

    for(std::size_t i = 0; i < cfg.joints_count; ++i) {
        const double range = cfg.limits.max_pos[i] - cfg.limits.min_pos[i];
        if(cfg.limits.min_pos[i] >= cfg.limits.max_pos[i] ||
            cfg.limits.max_vel[i] <= 0.0 || cfg.limits.max_acc[i] <= 0.0 || cfg.limits.max_effort[i] <= 0.0 ||
            cfg.limits.max_kp[i] < 0.0 || cfg.limits.max_kd[i] < 0.0 || cfg.limits.pos_margin[i] < 0.0 ||
            2.0 * cfg.limits.pos_margin[i] >= range) {
            return tl::make_unexpected(fault(SafetyErr::INVALID_CFG, i));
        }
    }

    return {};
}

/**
 * @brief 检查 Joint/Actuator 状态数组长度
 * @param state Joint 侧状态
 * @return Joint/Actuator 状态数组长度是否正确
 */
bool Safety::has_joint_size(const JointState& state) const noexcept {
    return state.pos.size() == cfg_.joints_count && state.vel.size() == cfg_.joints_count && state.tor.size() == cfg_.joints_count;
}

/**
 * @brief 检查 Joint/Actuator 状态数组长度
 * @param state Actuator 侧状态
 * @return Joint/Actuator 状态数组长度是否正确
 */
bool Safety::has_actuator_size(const ActuatorState& state) const noexcept {
    return state.pos.size() == cfg_.joints_count && state.vel.size() == cfg_.joints_count && state.tor.size() == cfg_.joints_count &&
        state.online.size() == cfg_.joints_count && state.enabled.size() == cfg_.joints_count && state.err_code.size() == cfg_.joints_count;
}

/**
 * @brief 检查 JointCtrlCmd 数组长度
 * @param cmd 上层下发的 JointCtrlCmd
 * @return JointCtrlCmd 数组长度是否正确
 */
bool Safety::has_cmd_size(const JointCtrlCmd& cmd) const noexcept {
    return cmd.pos.size() == cfg_.joints_count && cmd.vel.size() == cfg_.joints_count && cmd.tor.size() == cfg_.joints_count &&
        cmd.kp.size() == cfg_.joints_count && cmd.kd.size() == cfg_.joints_count;
}

/**
 * @brief 检查 Joint/Actuator 状态是否包含 NaN/Inf
 * @param state Joint 侧状态
 * @return Joint/Actuator 状态是否包含 NaN/Inf
 */
bool Safety::is_finite_vector(const std::vector<double>& values) const noexcept {
    return std::all_of(values.begin(), values.end(), [](double value) {
        return std::isfinite(value);
        });
}

/**
 * @brief 检查并限幅浮点值，若超出 min/max 范围则 clamp 到边界
 * @param value 待检查的浮点值，若超出范围则被修改为边界值
 * @param min_value 最小值
 * @param max_value 最大值
 * @return 若 value 超出范围则返回 true，否则返回 false
 */
bool Safety::clamp_small_violation(double& value, double min_value, double max_value) noexcept {
    const double scale = std::max({ 1.0, std::abs(min_value), std::abs(max_value) });
    const double tolerance = cfg_.numeric_tolerance * scale;

    if(value < min_value) {
        if(value >= min_value - tolerance) {
            value = min_value;
            ++clamp_count_;
            return true;
        }
        return false;
    }
    if(value > max_value) {
        if(value <= max_value + tolerance) {
            value = max_value;
            ++clamp_count_;
            return true;
        }
        return false;
    }
    return true;
}

} // namespace dm_arm
