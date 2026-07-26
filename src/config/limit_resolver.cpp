#include "dm_arm/config/limit_resolver.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace dm_arm {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //

namespace {

/**
 * @brief 检查可选收窄值
 */
tl::expected<double, LimitResolverErr> narrowed(double base, const JointVector& overrides, std::size_t index) {
    if(overrides.empty()) return base;
    if(index >= overrides.size() || !std::isfinite(overrides[index]) || overrides[index] <= 0.0) return tl::make_unexpected(LimitResolverErr::INVALID_INPUT);
    if(overrides[index] > base) return tl::make_unexpected(LimitResolverErr::POLICY_WIDENS_LIMIT);
    return overrides[index];
}

} // namespace

// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

/**
 * @brief 生成 ResolvedSafetyCfg
 */
tl::expected<ResolvedSafetyCfg, LimitResolverErr> LimitResolver::resolve(const RobotModelInfo& model, const DamiaoBusCfg& hardware, const JointActuatorMapCfg& mapper, const HardwareCapabilities& capabilities, const SafetyPolicyCfg& policy) const {
    const std::size_t n = model.joint_names.size();
    if(n == 0 || model.joint_limits.size() != n || hardware.actuators.size() != n ||
        mapper.pos_ratio.size() != n || mapper.tor_ratio.size() != n || policy.max_acc.size() != n ||
        policy.position_margin < 0.0 || policy.cmd_vel_scale <= 0.0 || policy.cmd_vel_scale > 1.0 ||
        policy.state_vel_scale <= 0.0 || policy.state_vel_scale < policy.cmd_vel_scale ||
        policy.max_dt_s <= 0.0 || policy.state_timeout_s <= 0.0 || policy.cmd_timeout_s <= 0.0) {
        return tl::make_unexpected(LimitResolverErr::INVALID_INPUT);
    }

    std::unordered_map<std::string, ActuatorCapability> capability_by_name;
    for(const auto& capability : capabilities) capability_by_name.emplace(capability.actuator_name, capability);

    ResolvedSafetyCfg output;
    output.max_dt_s = policy.max_dt_s;
    output.state_timeout_s = policy.state_timeout_s;
    output.cmd_timeout_s = policy.cmd_timeout_s;
    output.require_all_actuators_online = policy.require_all_actuators_online;
    output.require_all_actuators_enabled = policy.require_all_actuators_enabled;
    output.reject_motor_error = policy.reject_motor_error;
    output.require_continuous_cmd = policy.require_continuous_cmd;
    output.joints.reserve(n);

    for(std::size_t i = 0; i < n; ++i) {
        const auto& joint = model.joint_limits[i];
        const auto& actuator = hardware.actuators[i];
        const auto capability_iter = capability_by_name.find(actuator.name);
        if(capability_iter == capability_by_name.end()) return tl::make_unexpected(LimitResolverErr::MISSING_ACTUATOR);
        const auto& capability = capability_iter->second;
        const double gain_ratio = mapper.pos_ratio[i] * mapper.tor_ratio[i];
        const double hardware_effort = capability.max_effort * mapper.tor_ratio[i];
        const double hardware_kp = capability.max_kp * gain_ratio;
        const double hardware_kd = capability.max_kd * gain_ratio;

        ResolvedJointLimitCfg resolved;
        resolved.joint_name = model.joint_names[i];
        resolved.has_position_limit = joint.has_position_limit;
        resolved.hard_min_pos = joint.min_pos;
        resolved.hard_max_pos = joint.max_pos;
        resolved.cmd_min_pos = joint.min_pos + policy.position_margin;
        resolved.cmd_max_pos = joint.max_pos - policy.position_margin;
        resolved.max_cmd_vel = joint.max_vel * policy.cmd_vel_scale;
        resolved.max_state_vel = joint.max_vel * policy.state_vel_scale;
        resolved.max_acc = policy.max_acc[i];
        resolved.max_effort = std::min(joint.max_effort, hardware_effort);
        resolved.max_kp = hardware_kp;
        resolved.max_kd = hardware_kd;

        const auto effort = narrowed(resolved.max_effort, policy.max_effort_override, i);
        const auto kp = narrowed(resolved.max_kp, policy.max_kp_override, i);
        const auto kd = narrowed(resolved.max_kd, policy.max_kd_override, i);
        if(!effort) return tl::make_unexpected(effort.error());
        if(!kp) return tl::make_unexpected(kp.error());
        if(!kd) return tl::make_unexpected(kd.error());
        resolved.max_effort = *effort;
        resolved.max_kp = *kp;
        resolved.max_kd = *kd;

        if(!resolved.has_position_limit || resolved.cmd_min_pos >= resolved.cmd_max_pos ||
            resolved.max_cmd_vel <= 0.0 || resolved.max_state_vel <= 0.0 || resolved.max_acc <= 0.0 ||
            resolved.max_effort <= 0.0 || resolved.max_kp < 0.0 || resolved.max_kd < 0.0) {
            return tl::make_unexpected(LimitResolverErr::INVALID_INPUT);
        }
        output.joints.push_back(std::move(resolved));
    }

    return output;
}

/**
 * @brief 将 ResolvedSafetyCfg 转换为 SafetyCfg
 */
SafetyCfg to_safety_cfg(const ResolvedSafetyCfg& resolved) {
    SafetyCfg cfg;
    cfg.joints_count = resolved.joints.size();
    cfg.cmd_timeout_s = resolved.cmd_timeout_s;
    cfg.state_timeout_s = resolved.state_timeout_s;
    cfg.max_dt_s = resolved.max_dt_s;
    cfg.require_all_actuators_online = resolved.require_all_actuators_online;
    cfg.require_all_actuators_enabled = resolved.require_all_actuators_enabled;
    cfg.reject_motor_error = resolved.reject_motor_error;
    cfg.require_continuous_cmd = resolved.require_continuous_cmd;

    cfg.limits.min_pos.reserve(resolved.joints.size());
    cfg.limits.max_pos.reserve(resolved.joints.size());
    cfg.limits.max_vel.reserve(resolved.joints.size());
    cfg.limits.max_acc.reserve(resolved.joints.size());
    cfg.limits.max_effort.reserve(resolved.joints.size());
    cfg.limits.max_kp.reserve(resolved.joints.size());
    cfg.limits.max_kd.reserve(resolved.joints.size());
    cfg.limits.pos_margin.reserve(resolved.joints.size());
    for(const auto& joint : resolved.joints) {
        cfg.limits.min_pos.push_back(joint.hard_min_pos);
        cfg.limits.max_pos.push_back(joint.hard_max_pos);
        cfg.limits.max_vel.push_back(joint.max_cmd_vel);
        cfg.limits.max_acc.push_back(joint.max_acc);
        cfg.limits.max_effort.push_back(joint.max_effort);
        cfg.limits.max_kp.push_back(joint.max_kp);
        cfg.limits.max_kd.push_back(joint.max_kd);
        cfg.limits.pos_margin.push_back(joint.cmd_min_pos - joint.hard_min_pos);
    }
    return cfg;
}

/**
 * @brief 从 SafetyCfg 生成 ResolvedSafetyCfg
 */
ResolvedSafetyCfg resolve_from_safety_cfg(const std::vector<std::string>& joint_names, const SafetyCfg& cfg) {
    ResolvedSafetyCfg resolved;
    resolved.max_dt_s = cfg.max_dt_s;
    resolved.state_timeout_s = cfg.state_timeout_s;
    resolved.cmd_timeout_s = cfg.cmd_timeout_s;
    resolved.require_all_actuators_online = cfg.require_all_actuators_online;
    resolved.require_all_actuators_enabled = cfg.require_all_actuators_enabled;
    resolved.reject_motor_error = cfg.reject_motor_error;
    resolved.require_continuous_cmd = cfg.require_continuous_cmd;
    resolved.joints.reserve(joint_names.size());
    for(std::size_t i = 0; i < joint_names.size(); ++i) {
        ResolvedJointLimitCfg joint;
        joint.joint_name = joint_names[i];
        joint.has_position_limit = true;
        joint.hard_min_pos = cfg.limits.min_pos[i];
        joint.hard_max_pos = cfg.limits.max_pos[i];
        joint.cmd_min_pos = cfg.limits.min_pos[i] + cfg.limits.pos_margin[i];
        joint.cmd_max_pos = cfg.limits.max_pos[i] - cfg.limits.pos_margin[i];
        joint.max_cmd_vel = cfg.limits.max_vel[i];
        joint.max_state_vel = cfg.limits.max_vel[i] * cfg.state_vel_fault_ratio;
        joint.max_acc = cfg.limits.max_acc[i];
        joint.max_effort = cfg.limits.max_effort[i];
        joint.max_kp = cfg.limits.max_kp[i];
        joint.max_kd = cfg.limits.max_kd[i];
        resolved.joints.push_back(std::move(joint));
    }
    return resolved;
}

// ! ========================= 私 有 类 方 法 实 现 ========================= ! //

} // namespace dm_arm
