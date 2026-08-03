#include "serial_arm/core/joint_actuator_mapper.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace serial_arm {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //



// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

/**
 * @brief 配置关节执行器映射器
 * @param cfg 关节执行器映射配置
 * @return tl::expected<void, JointActuatorMapErr>
 */
tl::expected<void, JointActuatorMapErr> JointActuatorMapper::configure(const JointActuatorMapCfg& cfg) {

    const JointActuatorMapErr err = validate_cfg(cfg);

    if(err != JointActuatorMapErr::OK) {
        configured_ = false;
        return tl::make_unexpected(err);
    }

    cfg_ = cfg;
    configured_ = true;

    return {};
}

/**
 * @brief 将关节控制命令转换为执行器控制命令
 * @param joint_cmd 关节侧完整控制命令
 * @return tl::expected<ActuatorCtrlCmd, JointActuatorMapErr> 执行器控制命令
 */
tl::expected<ActuatorCtrlCmd, JointActuatorMapErr> JointActuatorMapper::to_actuator_cmd(const JointCtrlCmd& joint_cmd) const {
    if(!configured_) {
        return tl::make_unexpected(JointActuatorMapErr::NOT_CONFIGURED);
    }

    const JointActuatorMapErr err = validate_joint_cmd(joint_cmd);

    if(err != JointActuatorMapErr::OK) {
        return tl::make_unexpected(err);
    }

    ActuatorCtrlCmd actuator_cmd;

    actuator_cmd.pos.resize(cfg_.joints_count);
    actuator_cmd.vel.resize(cfg_.joints_count);
    actuator_cmd.tor.resize(cfg_.joints_count);
    actuator_cmd.kp.resize(cfg_.joints_count);
    actuator_cmd.kd.resize(cfg_.joints_count);

    for(std::size_t i = 0; i < cfg_.joints_count; ++i) {
        const double direction = static_cast<double>(cfg_.direction[i]);
        const double pos_ratio = cfg_.pos_ratio[i];
        const double tor_ratio = cfg_.tor_ratio[i];
        const double gain_ratio = pos_ratio * tor_ratio;

        actuator_cmd.pos[i] = cfg_.actuator_zero_offset[i] + direction * pos_ratio * (joint_cmd.pos[i] - cfg_.joint_zero_offset[i]);
        actuator_cmd.vel[i] = direction * pos_ratio * joint_cmd.vel[i];
        actuator_cmd.tor[i] = direction * joint_cmd.tor[i] / tor_ratio;
        actuator_cmd.kp[i] = joint_cmd.kp[i] / gain_ratio;
        actuator_cmd.kd[i] = joint_cmd.kd[i] / gain_ratio;

        if(!std::isfinite(actuator_cmd.pos[i]) || !std::isfinite(actuator_cmd.vel[i]) || !std::isfinite(actuator_cmd.tor[i]) ||
            !std::isfinite(actuator_cmd.kp[i]) || !std::isfinite(actuator_cmd.kd[i])) {
            return tl::make_unexpected(JointActuatorMapErr::INVALID_CONVERSION_VALUE);
        }
    }

    return actuator_cmd;
}

/**
 * @brief 将执行器状态转换为关节状态
 * @param actuator_state 执行器状态
 * @return tl::expected<JointState, JointActuatorMapErr> 关节状态
 */
tl::expected<JointState, JointActuatorMapErr> JointActuatorMapper::to_joint_state(const ActuatorState& actuator_state) const {

    if(!configured_) {
        return tl::make_unexpected(JointActuatorMapErr::NOT_CONFIGURED);
    }

    const JointActuatorMapErr err = validate_actuator_state(actuator_state);

    if(err != JointActuatorMapErr::OK) {
        return tl::make_unexpected(err);
    }

    JointState joint_state;

    joint_state.pos.resize(cfg_.joints_count);
    joint_state.vel.resize(cfg_.joints_count);
    joint_state.tor.resize(cfg_.joints_count);

    for(std::size_t i = 0; i < cfg_.joints_count; ++i) {
        const double direction = static_cast<double>(cfg_.direction[i]);
        const double pos_ratio = cfg_.pos_ratio[i];
        const double tor_ratio = cfg_.tor_ratio[i];

        joint_state.pos[i] = cfg_.joint_zero_offset[i] + direction * (actuator_state.pos[i] - cfg_.actuator_zero_offset[i]) / pos_ratio;
        joint_state.vel[i] = direction * actuator_state.vel[i] / pos_ratio;
        joint_state.tor[i] = direction * tor_ratio * actuator_state.tor[i];

        if(!std::isfinite(joint_state.pos[i]) || !std::isfinite(joint_state.vel[i]) || !std::isfinite(joint_state.tor[i])) {
            return tl::make_unexpected(JointActuatorMapErr::INVALID_CONVERSION_VALUE);
        }
    }

    return joint_state;
}

/**
 * @brief 获取映射数量
 * @return std::size_t 映射数量
 */
std::size_t JointActuatorMapper::size() const noexcept {
    return configured_ ? cfg_.joints_count : 0;
}

// ! ========================= 私 有 类 方 法 实 现 ========================= ! //

/**
 * @brief 验证关节执行器映射配置
 * @param cfg 关节执行器映射配置
 * @return JointActuatorMapErr 验证结果
 */
JointActuatorMapErr JointActuatorMapper::validate_cfg(const JointActuatorMapCfg& cfg) const {

    if(cfg.joints_count == 0) {
        return JointActuatorMapErr::INVALID_CFG;
    }

    if(cfg.pos_ratio.size() != cfg.joints_count || cfg.tor_ratio.size() != cfg.joints_count ||
        cfg.direction.size() != cfg.joints_count || cfg.joint_zero_offset.size() != cfg.joints_count ||
        cfg.actuator_zero_offset.size() != cfg.joints_count) {
        return JointActuatorMapErr::INVALID_CFG;
    }

    if(!is_finite_vector(cfg.pos_ratio) || !is_finite_vector(cfg.tor_ratio) ||
        !is_finite_vector(cfg.joint_zero_offset) || !is_finite_vector(cfg.actuator_zero_offset)) {
        return JointActuatorMapErr::INVALID_CFG;
    }

    for(std::size_t i = 0; i < cfg.joints_count; ++i) {
        if(cfg.pos_ratio[i] <= 0.0 || cfg.tor_ratio[i] <= 0.0) {
            return JointActuatorMapErr::INVALID_CFG;
        }

        if(cfg.direction[i] != 1 && cfg.direction[i] != -1) {
            return JointActuatorMapErr::INVALID_CFG;
        }
    }

    return JointActuatorMapErr::OK;
}

/**
 * @brief 验证关节控制命令
 * @param cmd 关节控制命令
 * @return JointActuatorMapErr 验证结果
 */
JointActuatorMapErr JointActuatorMapper::validate_joint_cmd(const JointCtrlCmd& cmd) const {

    if(cmd.pos.size() != cfg_.joints_count || cmd.vel.size() != cfg_.joints_count || cmd.tor.size() != cfg_.joints_count ||
        cmd.kp.size() != cfg_.joints_count || cmd.kd.size() != cfg_.joints_count) {
        return JointActuatorMapErr::INVALID_JOINT_CMD;
    }

    if(!is_finite_vector(cmd.pos) || !is_finite_vector(cmd.vel) || !is_finite_vector(cmd.tor) ||
        !is_finite_vector(cmd.kp) || !is_finite_vector(cmd.kd)) {
        return JointActuatorMapErr::INVALID_JOINT_CMD;
    }

    const bool kp_valid = std::all_of(cmd.kp.begin(), cmd.kp.end(),
        [](double value) {
            return value >= 0.0;
        });

    const bool kd_valid = std::all_of(cmd.kd.begin(), cmd.kd.end(),
        [](double value) {
            return value >= 0.0;
        });

    if(!kp_valid || !kd_valid) {
        return JointActuatorMapErr::INVALID_JOINT_CMD;
    }

    return JointActuatorMapErr::OK;
}

/**
 * @brief 验证执行器状态
 * @param state 执行器状态
 * @return JointActuatorMapErr 验证结果
 */
JointActuatorMapErr JointActuatorMapper::validate_actuator_state(const ActuatorState& state) const {

    if(state.pos.size() != cfg_.joints_count || state.vel.size() != cfg_.joints_count || state.tor.size() != cfg_.joints_count) {
        return JointActuatorMapErr::INVALID_ACTUATOR_STATE;
    }

    if(!is_finite_vector(state.pos) || !is_finite_vector(state.vel) || !is_finite_vector(state.tor)) {
        return JointActuatorMapErr::INVALID_ACTUATOR_STATE;
    }

    return JointActuatorMapErr::OK;
}

/**
 * @brief 检查向量是否为有限值
 * @param vector 待检查的向量
 * @return true 如果向量中的所有元素都是有限值，否则返回 false
 */
bool JointActuatorMapper::is_finite_vector(const std::vector<double>& vector) const {

    return std::all_of(vector.begin(), vector.end(),
        [](double value) {
            return std::isfinite(value);
        });
}

} // namespace serial_arm
