#include "dm_arm/joints_ctrller.hpp"

#include <cmath>

namespace dm_arm {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //



// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

/**
 * @brief 配置关节控制器
 * @param cfg 关节控制器配置
 * @return tl::expected<void, JointCtrllerErr>
 */
tl::expected<void, JointCtrllerErr> JointCtrller::configure(const JointCtrllerCfg& cfg) {
    const JointCtrllerErr err = validate_cfg(cfg);

    if(err != JointCtrllerErr::OK) {
        is_configured_ = false;
        has_valid_cmd_ = false;

        return tl::make_unexpected(err);
    }

    cfg_ = cfg;

    hold_pos_.assign(cfg_.joints_count, 0.0);

    cur_cmd_ = JointCmd{};
    cur_cmd_.mode = JointCmdMode::HOLD;

    impedance_mode_ = JointImpedanceMode::RIGID_HOLD;

    is_configured_ = true;
    has_valid_cmd_ = false;

    return {};
}

/**
 * @brief 重置关节控制器
 * @param state 关节状态
 * @return tl::expected<void, JointCtrllerErr>
 */
tl::expected<void, JointCtrllerErr> JointCtrller::reset(const JointState& state) {
    if(!is_configured_) {
        return tl::make_unexpected(JointCtrllerErr::NOT_CONFIGURED);
    }

    const JointCtrllerErr err = validate_state(state);

    if(err != JointCtrllerErr::OK) {
        return tl::make_unexpected(err);
    }

    hold_pos_ = state.pos;

    cur_cmd_ = JointCmd{};
    cur_cmd_.mode = JointCmdMode::HOLD;

    impedance_mode_ = JointImpedanceMode::RIGID_HOLD;

    has_valid_cmd_ = true;

    return {};
}

/**
 * @brief 设置关节阻抗模式
 * @param mode 关节阻抗模式
 * @param state 关节状态
 * @return tl::expected<void, JointCtrllerErr>
 */
tl::expected<void, JointCtrllerErr> JointCtrller::set_impedance_mode(JointImpedanceMode mode, const JointState& state) {
    if(!is_configured_) {
        return tl::make_unexpected(JointCtrllerErr::NOT_CONFIGURED);
    }

    const JointCtrllerErr state_err = validate_state(state);

    if(state_err != JointCtrllerErr::OK) {
        return tl::make_unexpected(state_err);
    }

    switch(mode) {
        case JointImpedanceMode::RIGID_HOLD:
        case JointImpedanceMode::COMPLIANT_HOLD:
        case JointImpedanceMode::TRACKING:
        {
            break;
        }

        default:
        {
            return tl::make_unexpected(JointCtrllerErr::INVALID_MODE);
        }
    }

    hold_pos_ = state.pos;
    impedance_mode_ = mode;

    cur_cmd_ = JointCmd{};
    cur_cmd_.mode = JointCmdMode::HOLD;
    has_valid_cmd_ = true;

    return {};
}

/**
 * @brief 设置关节命令
 * @param cmd 关节命令
 * @return tl::expected<void, JointCtrllerErr>
 */
tl::expected<void, JointCtrllerErr> JointCtrller::set_cmd(const JointCmd& cmd) {
    if(!is_configured_) {
        return tl::make_unexpected(JointCtrllerErr::NOT_CONFIGURED);
    }

    const JointCtrllerErr err = validate_cmd(cmd);

    if(err != JointCtrllerErr::OK) {
        return tl::make_unexpected(err);
    }

    cur_cmd_ = cmd;

    if(cmd.mode == JointCmdMode::HOLD) {
        has_valid_cmd_ = false;
    }
    else {
        has_valid_cmd_ = true;
    }

    return {};
}

/**
 * @brief 更新关节控制器
 * @param input 关节控制器输入
 * @return tl::expected<JointCtrllerOutput, JointCtrllerErr> 关节控制器输出
 */
tl::expected<JointCtrllerOutput, JointCtrllerErr> JointCtrller::update(const JointCtrllerInput& input) {
    if(!is_configured_) {
        return tl::make_unexpected(JointCtrllerErr::NOT_CONFIGURED);
    }

    const JointCtrllerErr state_err = validate_state(input.state);

    if(state_err != JointCtrllerErr::OK) {
        return tl::make_unexpected(state_err);
    }

    if(!std::isfinite(input.dt) || input.dt <= 0.0) {
        return tl::make_unexpected(JointCtrllerErr::INVALID_DT);
    }

    if(input.model_feedforward.size() != cfg_.joints_count) {
        return tl::make_unexpected(JointCtrllerErr::INVALID_MODEL_FEEDFORWARD);
    }

    if(!is_finite_vector(input.model_feedforward)) {
        return tl::make_unexpected(JointCtrllerErr::INVALID_MODEL_FEEDFORWARD);
    }

    if(cur_cmd_.mode == JointCmdMode::HOLD && !has_valid_cmd_) {
        hold_pos_ = input.state.pos;
        has_valid_cmd_ = true;
    }

    const std::size_t joints_count = cfg_.joints_count;

    JointCtrllerOutput output;

    output.cmd.pos.assign(joints_count, 0.0);
    output.cmd.vel.assign(joints_count, 0.0);
    output.cmd.tor.assign(joints_count, 0.0);
    output.cmd.kp.assign(joints_count, 0.0);
    output.cmd.kd.assign(joints_count, 0.0);

    const JointImpedanceGains* selected_gains = &cfg_.tracking_gains;

    switch(impedance_mode_) {
        case JointImpedanceMode::RIGID_HOLD:
        {
            selected_gains = &cfg_.rigid_hold_gains;
            break;
        }

        case JointImpedanceMode::COMPLIANT_HOLD:
        {
            selected_gains = &cfg_.compliant_hold_gains;
            break;
        }

        case JointImpedanceMode::TRACKING:
        {
            selected_gains = &cfg_.tracking_gains;
            break;
        }

        default:
        {
            return tl::make_unexpected(JointCtrllerErr::INVALID_MODE);
        }
    }

    if(impedance_mode_ != JointImpedanceMode::TRACKING) {

        output.cmd.pos = hold_pos_;
        output.cmd.vel.assign(joints_count, 0.0);
        output.cmd.tor = input.model_feedforward;
        output.cmd.kp = selected_gains->kp;
        output.cmd.kd = selected_gains->kd;

        return output;
    }

    switch(cur_cmd_.mode) {
        case JointCmdMode::HOLD:
        {
            output.cmd.pos = hold_pos_;
            output.cmd.vel.assign(joints_count, 0.0);
            output.cmd.tor = input.model_feedforward;
            output.cmd.kp = cfg_.tracking_gains.kp;
            output.cmd.kd = cfg_.tracking_gains.kd;

            break;
        }

        case JointCmdMode::POS:
        {
            output.cmd.pos = cur_cmd_.pos.value();
            output.cmd.vel.assign(joints_count, 0.0);
            output.cmd.tor = input.model_feedforward;
            output.cmd.kp = cfg_.tracking_gains.kp;
            output.cmd.kd = cfg_.tracking_gains.kd;

            break;
        }

        case JointCmdMode::VEL:
        {
            output.cmd.pos = input.state.pos;
            output.cmd.vel = cur_cmd_.vel.value();
            output.cmd.tor = input.model_feedforward;
            output.cmd.kp.assign(joints_count, 0.0);
            output.cmd.kd = cfg_.tracking_gains.kd;

            break;
        }

        case JointCmdMode::POS_VEL:
        {
            output.cmd.pos = cur_cmd_.pos.value();
            output.cmd.vel = cur_cmd_.vel.value();
            output.cmd.tor = input.model_feedforward;
            output.cmd.kp = cfg_.tracking_gains.kp;
            output.cmd.kd = cfg_.tracking_gains.kd;

            break;
        }

        case JointCmdMode::TOR:
        {
            output.cmd.pos = input.state.pos;
            output.cmd.vel.assign(joints_count, 0.0);
            output.cmd.tor = cur_cmd_.tor.value();
            output.cmd.kp.assign(joints_count, 0.0);
            output.cmd.kd.assign(joints_count, 0.0);

            break;
        }

        case JointCmdMode::IMPEDANCE:
        {
            output.cmd.pos = cur_cmd_.pos.value();
            output.cmd.vel = cur_cmd_.vel.value();
            output.cmd.kp = cur_cmd_.gains.value().kp;
            output.cmd.kd = cur_cmd_.gains.value().kd;

            for(std::size_t i = 0; i < joints_count; ++i) {
                const double torque = input.model_feedforward[i] + cur_cmd_.tor.value()[i];

                if(!std::isfinite(torque)) {
                    return tl::make_unexpected(JointCtrllerErr::INVALID_TOR);
                }

                output.cmd.tor[i] = torque;
            }

            break;
        }

        default:
        {
            return tl::make_unexpected(JointCtrllerErr::INVALID_MODE);
        }
    }

    return output;
}

// ! ========================= 私 有 类 方 法 实 现 ========================= ! //

/**
 * @brief 检查向量是否为有限值
 * @param vector 待检查的向量
 * @return true 如果向量中的所有元素都是有限值，否则返回 false
 */
bool JointCtrller::is_finite_vector(const JointVector& vector) const {
    return std::all_of(vector.begin(), vector.end(),
        [](double value) {
            return std::isfinite(value);
        });
}

/**
 * @brief 验证关节控制器配置
 * @param cfg 关节控制器配置
 * @return JointCtrllerErr 验证结果
 */
JointCtrllerErr JointCtrller::validate_cfg(const JointCtrllerCfg& cfg) const {
    if(cfg.joints_count == 0) {
        return JointCtrllerErr::INVALID_CFG;
    }

    const auto validate_gains = [this, &cfg](const JointImpedanceGains& gains) -> bool {
        if(gains.kp.size() != cfg.joints_count || gains.kd.size() != cfg.joints_count) {
            return false;
        }

        if(!is_finite_vector(gains.kp) || !is_finite_vector(gains.kd)) {
            return false;
        }

        const bool kp_valid = std::all_of(gains.kp.begin(), gains.kp.end(),
            [](double value) {
                return value >= 0.0;
            });

        const bool kd_valid = std::all_of(gains.kd.begin(), gains.kd.end(),
            [](double value) {
                return value >= 0.0;
            });

        return kp_valid && kd_valid;
        };

    if(!validate_gains(cfg.rigid_hold_gains)) {
        return JointCtrllerErr::INVALID_CFG;
    }

    if(!validate_gains(cfg.compliant_hold_gains)) {
        return JointCtrllerErr::INVALID_CFG;
    }

    if(!validate_gains(cfg.tracking_gains)) {
        return JointCtrllerErr::INVALID_CFG;
    }

    return JointCtrllerErr::OK;
}

/**
 * @brief 验证关节状态
 * @param state 关节状态
 * @return JointCtrllerErr 验证结果
 */
JointCtrllerErr JointCtrller::validate_state(const JointState& state) const {
    if(!is_configured_) {
        return JointCtrllerErr::NOT_CONFIGURED;
    }

    const std::size_t joints_count = cfg_.joints_count;

    if(state.pos.size() != joints_count || state.vel.size() != joints_count || state.tor.size() != joints_count) {
        return JointCtrllerErr::INVALID_STATE;
    }

    if(!is_finite_vector(state.pos) || !is_finite_vector(state.vel) || !is_finite_vector(state.tor)) {
        return JointCtrllerErr::INVALID_STATE;
    }

    return JointCtrllerErr::OK;
}

/**
 * @brief 验证关节命令
 * @param cmd 关节命令
 * @return JointCtrllerErr 验证结果
 */
JointCtrllerErr JointCtrller::validate_cmd(const JointCmd& cmd) const {
    if(!is_configured_) {
        return JointCtrllerErr::NOT_CONFIGURED;
    }

    const std::size_t joints_count = cfg_.joints_count;

    const auto validate_vector = [this, joints_count](const JointVector& vector) -> bool {
        return vector.size() == joints_count &&
            is_finite_vector(vector);
        };

    switch(cmd.mode) {
        case JointCmdMode::HOLD:
        {
            return JointCtrllerErr::OK;
        }

        case JointCmdMode::POS:
        {
            if(!cmd.pos.has_value()) {
                return JointCtrllerErr::MISSING_POS;
            }

            if(!validate_vector(cmd.pos.value())) {
                return JointCtrllerErr::INVALID_POS;
            }

            return JointCtrllerErr::OK;
        }

        case JointCmdMode::VEL:
        {
            if(!cmd.vel.has_value()) {
                return JointCtrllerErr::MISSING_VEL;
            }

            if(!validate_vector(cmd.vel.value())) {
                return JointCtrllerErr::INVALID_VEL;
            }

            return JointCtrllerErr::OK;
        }

        case JointCmdMode::POS_VEL:
        {
            if(!cmd.pos.has_value()) {
                return JointCtrllerErr::MISSING_POS;
            }

            if(!cmd.vel.has_value()) {
                return JointCtrllerErr::MISSING_VEL;
            }

            if(!validate_vector(cmd.pos.value())) {
                return JointCtrllerErr::INVALID_POS;
            }

            if(!validate_vector(cmd.vel.value())) {
                return JointCtrllerErr::INVALID_VEL;
            }

            return JointCtrllerErr::OK;
        }

        case JointCmdMode::TOR:
        {
            if(!cmd.tor.has_value()) {
                return JointCtrllerErr::MISSING_TOR;
            }

            if(!validate_vector(cmd.tor.value())) {
                return JointCtrllerErr::INVALID_TOR;
            }

            return JointCtrllerErr::OK;
        }

        case JointCmdMode::IMPEDANCE:
        {
            if(!cmd.pos.has_value()) {
                return JointCtrllerErr::MISSING_POS;
            }

            if(!cmd.vel.has_value()) {
                return JointCtrllerErr::MISSING_VEL;
            }

            if(!cmd.tor.has_value()) {
                return JointCtrllerErr::MISSING_TOR;
            }

            if(!cmd.gains.has_value()) {
                return JointCtrllerErr::MISSING_GAINS;
            }

            if(!validate_vector(cmd.pos.value())) {
                return JointCtrllerErr::INVALID_POS;
            }

            if(!validate_vector(cmd.vel.value())) {
                return JointCtrllerErr::INVALID_VEL;
            }

            if(!validate_vector(cmd.tor.value())) {
                return JointCtrllerErr::INVALID_TOR;
            }

            const JointImpedanceGains& gains = cmd.gains.value();

            if(!validate_vector(gains.kp) || !validate_vector(gains.kd)) {
                return JointCtrllerErr::INVALID_GAINS;
            }

            const bool kp_valid = std::all_of(gains.kp.begin(), gains.kp.end(),
                [](double value) {
                    return value >= 0.0;
                });

            const bool kd_valid = std::all_of(gains.kd.begin(), gains.kd.end(),
                [](double value) {
                    return value >= 0.0;
                });

            if(!kp_valid || !kd_valid) {
                return JointCtrllerErr::INVALID_GAINS;
            }

            return JointCtrllerErr::OK;
        }

        default:
        {
            return JointCtrllerErr::INVALID_MODE;
        }
    }
}

} // namespace dm_arm
