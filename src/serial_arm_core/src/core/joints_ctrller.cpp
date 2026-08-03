#include "serial_arm/core/joints_ctrller.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <type_traits>

namespace serial_arm {

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
        state_ = JointCtrllerState::UNCONFIGURED;
        has_cmd_ = false;
        has_full_cmd_ = false;

        return tl::make_unexpected(err);
    }

    cfg_ = cfg;

    hold_pos_.assign(cfg_.joints_count, 0.0);
    fallback_pos_.assign(cfg_.joints_count, 0.0);

    full_cmd_.pos.assign(cfg_.joints_count, 0.0);
    full_cmd_.vel.assign(cfg_.joints_count, 0.0);
    full_cmd_.tor.assign(cfg_.joints_count, 0.0);
    full_cmd_.kp.assign(cfg_.joints_count, 0.0);
    full_cmd_.kd.assign(cfg_.joints_count, 0.0);

    cur_cmd_ = JointPosCmd{};

    impedance_mode_ = JointImpedanceMode::RIGID_HOLD;
    state_ = JointCtrllerState::CONFIGURED;

    has_cmd_ = false;
    has_full_cmd_ = false;

    return {};
}

/**
 * @brief 初始化关节控制器
 * @param state 初始关节状态
 * @return tl::expected<void, JointCtrllerErr>
 */
tl::expected<void, JointCtrllerErr> JointCtrller::initialize(const JointState& state) {
    if(state_ == JointCtrllerState::UNCONFIGURED) {
        return tl::make_unexpected(JointCtrllerErr::NOT_CONFIGURED);
    }

    if(state_ == JointCtrllerState::INITIALIZED) {
        return tl::make_unexpected(JointCtrllerErr::ALREADY_INITIALIZED);
    }

    const JointCtrllerErr err = validate_state(state);

    if(err != JointCtrllerErr::OK) {
        return tl::make_unexpected(err);
    }

    hold_pos_ = state.pos;
    fallback_pos_ = state.pos;

    cur_cmd_ = JointPosCmd{};

    impedance_mode_ = JointImpedanceMode::RIGID_HOLD;
    state_ = JointCtrllerState::INITIALIZED;

    has_cmd_ = false;
    has_full_cmd_ = false;

    return {};
}

void JointCtrller::reset() noexcept {
    if(state_ == JointCtrllerState::UNCONFIGURED) {
        return;
    }

    hold_pos_.assign(cfg_.joints_count, 0.0);
    fallback_pos_.assign(cfg_.joints_count, 0.0);
    cur_cmd_ = JointPosCmd{};

    full_cmd_.pos.assign(cfg_.joints_count, 0.0);
    full_cmd_.vel.assign(cfg_.joints_count, 0.0);
    full_cmd_.tor.assign(cfg_.joints_count, 0.0);
    full_cmd_.kp.assign(cfg_.joints_count, 0.0);
    full_cmd_.kd.assign(cfg_.joints_count, 0.0);

    impedance_mode_ = JointImpedanceMode::RIGID_HOLD;
    has_cmd_ = false;
    has_full_cmd_ = false;
    state_ = JointCtrllerState::CONFIGURED;
}

/**
 * @brief 设置关节阻抗模式
 * @param mode 关节阻抗模式
 * @param state 当前关节状态
 * @return tl::expected<void, JointCtrllerErr>
 */
tl::expected<void, JointCtrllerErr> JointCtrller::set_impedance_mode(JointImpedanceMode mode, const JointState& state) {

    if(state_ == JointCtrllerState::UNCONFIGURED) {
        return tl::make_unexpected(JointCtrllerErr::NOT_CONFIGURED);
    }

    if(state_ != JointCtrllerState::INITIALIZED) {
        return tl::make_unexpected(JointCtrllerErr::NOT_INITIALIZED);
    }

    const JointCtrllerErr mode_err = validate_impedance_mode(mode);

    if(mode_err != JointCtrllerErr::OK) {
        return tl::make_unexpected(mode_err);
    }

    const JointCtrllerErr state_err = validate_state(state);

    if(state_err != JointCtrllerErr::OK) {
        return tl::make_unexpected(state_err);
    }

    switch(mode) {
        case JointImpedanceMode::RIGID_HOLD:
        case JointImpedanceMode::COMPLIANT_HOLD:
        {
            hold_pos_ = state.pos;
            fallback_pos_ = state.pos;
            break;
        }

        case JointImpedanceMode::RIGID_TRACKING:
        case JointImpedanceMode::COMPLIANT_TRACKING:
        {
            fallback_pos_ = state.pos;
            break;
        }

        case JointImpedanceMode::COMPLIANT_DRAG:
        {
            fallback_pos_ = state.pos;
            break;
        }

        default:
        {
            return tl::make_unexpected(JointCtrllerErr::INVALID_IMPEDANCE_MODE);
        }
    }

    impedance_mode_ = mode;

    cur_cmd_ = JointPosCmd{};
    has_cmd_ = false;
    has_full_cmd_ = false;

    return {};
}

/**
 * @brief 设置关节参考命令
 * @param cmd 关节参考命令
 * @return tl::expected<void, JointCtrllerErr>
 */
tl::expected<void, JointCtrllerErr> JointCtrller::set_cmd(const JointCmd& cmd) {
    if(state_ == JointCtrllerState::UNCONFIGURED) {
        return tl::make_unexpected(JointCtrllerErr::NOT_CONFIGURED);
    }

    if(state_ != JointCtrllerState::INITIALIZED) {
        return tl::make_unexpected(JointCtrllerErr::NOT_INITIALIZED);
    }

    if(!is_tracking_mode()) {
        return tl::make_unexpected(JointCtrllerErr::CMD_NOT_ALLOWED_IN_MODE);
    }

    const JointCtrllerErr err = validate_cmd(cmd);

    if(err != JointCtrllerErr::OK) {
        return tl::make_unexpected(err);
    }

    cur_cmd_ = cmd;
    has_cmd_ = true;
    has_full_cmd_ = false;

    return {};
}

/**
 * @brief 直接设置关节完整控制命令
 * @param cmd 关节完整控制命令
 * @return tl::expected<void, JointCtrllerErr>
 */
tl::expected<void, JointCtrllerErr> JointCtrller::set_full_cmd(const JointCtrlCmd& cmd) {
    if(state_ == JointCtrllerState::UNCONFIGURED) {
        return tl::make_unexpected(JointCtrllerErr::NOT_CONFIGURED);
    }

    if(state_ != JointCtrllerState::INITIALIZED) {
        return tl::make_unexpected(JointCtrllerErr::NOT_INITIALIZED);
    }

    if(!cfg_.allow_full_cmd) {
        return tl::make_unexpected(JointCtrllerErr::FULL_CMD_NOT_ALLOWED);
    }

    if(!is_tracking_mode()) {
        return tl::make_unexpected(JointCtrllerErr::CMD_NOT_ALLOWED_IN_MODE);
    }

    const JointCtrllerErr err = validate_full_cmd(cmd);

    if(err != JointCtrllerErr::OK) {
        return tl::make_unexpected(err);
    }

    full_cmd_ = cmd;
    has_full_cmd_ = true;
    has_cmd_ = false;

    return {};
}

/**
 * @brief 更新关节控制器
 * @param input 关节控制器输入
 * @return tl::expected<JointCtrllerOutput, JointCtrllerErr> 关节控制器输出
 */
tl::expected<JointCtrllerOutput, JointCtrllerErr> JointCtrller::update(const JointCtrllerInput& input) {
    if(state_ == JointCtrllerState::UNCONFIGURED) {
        return tl::make_unexpected(JointCtrllerErr::NOT_CONFIGURED);
    }

    if(state_ != JointCtrllerState::INITIALIZED) {
        return tl::make_unexpected(JointCtrllerErr::NOT_INITIALIZED);
    }

    const JointCtrllerErr state_err = validate_state(input.state);

    if(state_err != JointCtrllerErr::OK) {
        return tl::make_unexpected(state_err);
    }

    if(!std::isfinite(input.dt) || input.dt <= 0.0) {
        return tl::make_unexpected(JointCtrllerErr::INVALID_DT);
    }

    if(input.model_feedforward.size() != cfg_.joints_count ||
        !is_finite_vector(input.model_feedforward)) {
        return tl::make_unexpected(JointCtrllerErr::INVALID_MODEL_FEEDFORWARD);
    }

    JointCtrllerOutput output;

    switch(impedance_mode_) {
        case JointImpedanceMode::RIGID_HOLD:
        {
            output.cmd = build_hold_cmd(
                hold_pos_,
                cfg_.rigid_hold_gains,
                input.model_feedforward);
            break;
        }

        case JointImpedanceMode::RIGID_TRACKING:
        {
            const auto cmd = build_tracking_cmd(
                cfg_.rigid_tracking_gains,
                input.model_feedforward);

            if(!cmd) {
                return tl::make_unexpected(cmd.error());
            }

            output.cmd = cmd.value();
            break;
        }

        case JointImpedanceMode::COMPLIANT_HOLD:
        {
            output.cmd = build_hold_cmd(
                hold_pos_,
                cfg_.compliant_hold_gains,
                input.model_feedforward);
            break;
        }

        case JointImpedanceMode::COMPLIANT_DRAG:
        {
            output.cmd = build_drag_cmd(
                input.state,
                cfg_.compliant_drag_gains,
                input.model_feedforward);
            break;
        }

        case JointImpedanceMode::COMPLIANT_TRACKING:
        {
            const auto cmd = build_tracking_cmd(
                cfg_.compliant_tracking_gains,
                input.model_feedforward);

            if(!cmd) {
                return tl::make_unexpected(cmd.error());
            }

            output.cmd = cmd.value();
            break;
        }

        default:
        {
            return tl::make_unexpected(JointCtrllerErr::INVALID_IMPEDANCE_MODE);
        }
    }

    return output;
}

/**
 * @brief 获取关节控制器生命周期状态
 * @return JointCtrllerState 关节控制器生命周期状态
 */
JointCtrllerState JointCtrller::get_state() const noexcept {
    return state_;
}

/**
 * @brief 获取当前关节阻抗模式
 * @return JointImpedanceMode 当前关节阻抗模式
 */
JointImpedanceMode JointCtrller::get_impedance_mode() const noexcept {
    return impedance_mode_;
}

// ! ========================= 私 有 类 方 法 实 现 ========================= ! //

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

    if(!validate_gains(cfg.rigid_hold_gains) || !validate_gains(cfg.rigid_tracking_gains) ||
        !validate_gains(cfg.compliant_hold_gains) || !validate_gains(cfg.compliant_drag_gains) ||
        !validate_gains(cfg.compliant_tracking_gains)) {
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
    if(state_ == JointCtrllerState::UNCONFIGURED) {
        return JointCtrllerErr::NOT_CONFIGURED;
    }

    if(state.pos.size() != cfg_.joints_count || state.vel.size() != cfg_.joints_count || state.tor.size() != cfg_.joints_count) {
        return JointCtrllerErr::INVALID_STATE;
    }

    if(!is_finite_vector(state.pos) || !is_finite_vector(state.vel) || !is_finite_vector(state.tor)) {
        return JointCtrllerErr::INVALID_STATE;
    }

    return JointCtrllerErr::OK;
}

/**
 * @brief 验证关节参考命令
 * @param cmd 关节参考命令
 * @return JointCtrllerErr 验证结果
 */
JointCtrllerErr JointCtrller::validate_cmd(const JointCmd& cmd) const {
    if(state_ == JointCtrllerState::UNCONFIGURED) {
        return JointCtrllerErr::NOT_CONFIGURED;
    }

    return std::visit(
        [this](const auto& typed_cmd) -> JointCtrllerErr {
            using CmdType = std::decay_t<decltype(typed_cmd)>;

            if constexpr(std::is_same_v<CmdType, JointPosCmd>) {
                if(typed_cmd.pos.size() != cfg_.joints_count) {
                    return JointCtrllerErr::INVALID_CMD_SIZE;
                }

                if(!is_finite_vector(typed_cmd.pos)) {
                    return JointCtrllerErr::INVALID_CMD_VALUE;
                }
            }
            else if constexpr(std::is_same_v<CmdType, JointPosVelCmd>) {
                if(typed_cmd.pos.size() != cfg_.joints_count ||
                    typed_cmd.vel.size() != cfg_.joints_count) {
                    return JointCtrllerErr::INVALID_CMD_SIZE;
                }

                if(!is_finite_vector(typed_cmd.pos) ||
                    !is_finite_vector(typed_cmd.vel)) {
                    return JointCtrllerErr::INVALID_CMD_VALUE;
                }
            }
            else if constexpr(std::is_same_v<CmdType, JointPosVelTorCmd>) {
                if(typed_cmd.pos.size() != cfg_.joints_count ||
                    typed_cmd.vel.size() != cfg_.joints_count ||
                    typed_cmd.tor.size() != cfg_.joints_count) {
                    return JointCtrllerErr::INVALID_CMD_SIZE;
                }

                if(!is_finite_vector(typed_cmd.pos) ||
                    !is_finite_vector(typed_cmd.vel) ||
                    !is_finite_vector(typed_cmd.tor)) {
                    return JointCtrllerErr::INVALID_CMD_VALUE;
                }
            }

            return JointCtrllerErr::OK;
        },
        cmd);
}

/**
 * @brief 验证关节完整控制命令
 * @param cmd 关节完整控制命令
 * @return JointCtrllerErr 验证结果
 */
JointCtrllerErr JointCtrller::validate_full_cmd(const JointCtrlCmd& cmd) const {
    if(state_ == JointCtrllerState::UNCONFIGURED) {
        return JointCtrllerErr::NOT_CONFIGURED;
    }

    if(cmd.pos.size() != cfg_.joints_count || cmd.vel.size() != cfg_.joints_count || cmd.tor.size() != cfg_.joints_count ||
        cmd.kp.size() != cfg_.joints_count || cmd.kd.size() != cfg_.joints_count) {
        return JointCtrllerErr::INVALID_FULL_CMD;
    }

    if(!is_finite_vector(cmd.pos) || !is_finite_vector(cmd.vel) || !is_finite_vector(cmd.tor) ||
        !is_finite_vector(cmd.kp) || !is_finite_vector(cmd.kd)) {
        return JointCtrllerErr::INVALID_FULL_CMD;
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
        return JointCtrllerErr::INVALID_FULL_CMD;
    }

    return JointCtrllerErr::OK;
}

/**
 * @brief 验证关节阻抗模式
 * @param mode 关节阻抗模式
 * @return JointCtrllerErr 验证结果
 */
JointCtrllerErr JointCtrller::validate_impedance_mode(JointImpedanceMode mode) const {
    switch(mode) {
        case JointImpedanceMode::RIGID_HOLD:
        case JointImpedanceMode::RIGID_TRACKING:
        case JointImpedanceMode::COMPLIANT_HOLD:
        case JointImpedanceMode::COMPLIANT_DRAG:
        case JointImpedanceMode::COMPLIANT_TRACKING:
        {
            return JointCtrllerErr::OK;
        }

        default:
        {
            return JointCtrllerErr::INVALID_IMPEDANCE_MODE;
        }
    }
}

/**
 * @brief 检查当前关节阻抗模式是否为跟踪模式
 * @return true 当前为跟踪模式，否则返回 false
 */
bool JointCtrller::is_tracking_mode() const noexcept {
    return impedance_mode_ == JointImpedanceMode::RIGID_TRACKING || impedance_mode_ == JointImpedanceMode::COMPLIANT_TRACKING;
}

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
 * @brief 生成关节保持 完整控制命令
 * @param hold_pos 关节保持位置
 * @param gains 关节阻抗增益
 * @param model_feedforward 模型前馈力矩
 * @return JointCtrlCmd 关节完整控制命令
 */
JointCtrlCmd JointCtrller::build_hold_cmd(const JointVector& hold_pos, const JointImpedanceGains& gains, const JointVector& model_feedforward) const {

    JointCtrlCmd cmd;

    cmd.pos = hold_pos;
    cmd.vel.assign(cfg_.joints_count, 0.0);
    cmd.tor = model_feedforward;
    cmd.kp = gains.kp;
    cmd.kd = gains.kd;

    return cmd;
}

/**
 * @brief 生成关节拖拽 完整控制命令
 * @param state 当前关节状态
 * @param gains 关节阻抗增益
 * @param model_feedforward 模型前馈力矩
 * @return JointCtrlCmd 关节完整控制命令
 */
JointCtrlCmd JointCtrller::build_drag_cmd(const JointState& state, const JointImpedanceGains& gains, const JointVector& model_feedforward) const {

    JointCtrlCmd cmd;

    cmd.pos = state.pos;
    cmd.vel.assign(cfg_.joints_count, 0.0);
    cmd.tor = model_feedforward;
    cmd.kp = gains.kp;
    cmd.kd = gains.kd;

    return cmd;
}

/**
 * @brief 生成关节跟踪 完整控制命令
 * @param gains 关节阻抗增益
 * @param model_feedforward 模型前馈力矩
 * @return tl::expected<JointCtrlCmd, JointCtrllerErr> 关节完整控制命令
 */
tl::expected<JointCtrlCmd, JointCtrllerErr> JointCtrller::build_tracking_cmd(const JointImpedanceGains& gains, const JointVector& model_feedforward) const {

    if(has_full_cmd_) {
        return full_cmd_;
    }

    if(!has_cmd_) {
        return build_hold_cmd(fallback_pos_, gains, model_feedforward);
    }

    JointCtrlCmd output;

    output.vel.assign(cfg_.joints_count, 0.0);
    output.tor = model_feedforward;
    output.kp = gains.kp;
    output.kd = gains.kd;

    const JointCtrllerErr err = std::visit(
        [this, &output](const auto& typed_cmd) -> JointCtrllerErr {
            using CmdType = std::decay_t<decltype(typed_cmd)>;

            if constexpr(std::is_same_v<CmdType, JointPosCmd>) {
                output.pos = typed_cmd.pos;
            }
            else if constexpr(std::is_same_v<CmdType, JointPosVelCmd>) {
                output.pos = typed_cmd.pos;
                output.vel = typed_cmd.vel;
            }
            else if constexpr(std::is_same_v<CmdType, JointPosVelTorCmd>) {
                output.pos = typed_cmd.pos;
                output.vel = typed_cmd.vel;

                for(std::size_t i = 0; i < cfg_.joints_count; ++i) {
                    const double tor = output.tor[i] + typed_cmd.tor[i];

                    if(!std::isfinite(tor)) {
                        return JointCtrllerErr::INVALID_CMD_VALUE;
                    }

                    output.tor[i] = tor;
                }
            }

            return JointCtrllerErr::OK;
        },
        cur_cmd_);

    if(err != JointCtrllerErr::OK) {
        return tl::make_unexpected(err);
    }

    return output;
}

} // namespace serial_arm
