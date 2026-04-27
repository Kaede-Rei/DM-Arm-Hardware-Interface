#pragma once

#include "dm_control_core/joint_control_types.hpp"
#include "tl/expected.hpp"

#include <cstddef>
#include <vector>

namespace dm_control_core {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief 纯 C++ 关节空间阻抗控制核心
 * @note 该类不包含 ROS 头文件。当前实现对应达妙 MIT 模式语义：
 *       电机侧执行 Kp(q_ref-q)+Kd(dq_ref-dq)+tau_ff
 */
class JointImpedanceController {
public:
    /**
     * @brief 配置关节阻抗控制器
     * @param config 控制器配置
     */
    void configure(const JointImpedanceControllerConfig& config);

    /**
     * @brief 重置控制器状态并锁存当前保持位置
     * @param current_state 当前关节状态
     */
    void reset(const JointState& current_state);

    /**
     * @brief 切换控制模式
     * @param mode 目标控制模式
     * @param current_state 当前关节状态
     * @note 切换到保持模式时会重新锁存当前关节位置，避免回拉旧保持点。
     */
    void set_mode(JointImpedanceMode mode, const JointState& current_state);

    /**
     * @brief 获取当前控制模式
     * @return 当前控制模式
     */
    JointImpedanceMode get_mode() const { return mode_; }

    /**
     * @brief 设置多关节命令
     * @param command 多关节命令
     * @return 命令合法返回空 expected，失败返回错误原因
     */
    tl::expected<void, JointCommandError> set_command(const JointCommand& command);

    /**
     * @brief 执行一次多关节阻抗控制更新
     * @param input 周期输入
     * @return 周期输出命令
     */
    JointImpedanceControllerOutput update(const JointImpedanceControllerInput& input);

private:
    /**
     * @brief 检查配置数组长度
     */
    void validate_config() const;

    /**
     * @brief 检查命令是否满足当前模式的字段需求
     * @param command 多关节命令
     * @return 命令合法返回空 expected，失败返回错误原因
     */
    tl::expected<void, JointCommandError> validate_command(const JointCommand& command) const;

    /**
     * @brief 锁存保持模式目标位置
     * @param current_state 当前关节状态
     */
    void latch_hold_position(const JointState& current_state);

    /**
     * @brief 根据当前模式选择对应关节阻抗参数
     * @param index 关节索引
     * @param kp 当前模式下的位置刚度
     * @param kd 当前模式下的速度阻尼
     */
    void select_gains(std::size_t index, double& kp, double& kd) const;

    /**
     * @brief 过滤非有限数，避免 NaN/Inf 进入控制命令
     * @param value 输入值
     * @param default_value fallback 值
     * @return 有限输入值或 fallback 值
     */
    double sanitize_or_default(double value, double default_value) const;

    /**
     * @brief 对称限幅
     * @param value 输入值
     * @param limit 绝对值上限，非正数或非有限数表示不限制
     * @return 限幅后的值
     */
    double clamp_abs(double value, double limit) const;

    /**
     * @brief 区间限幅
     * @param value 输入值
     * @param lower 下限
     * @param upper 上限
     * @return 限幅后的值
     */
    double clamp_range(double value, double lower, double upper) const;

private:
    JointImpedanceControllerConfig config_;
    JointImpedanceMode mode_{ JointImpedanceMode::RIGID_HOLD };

    std::vector<double> hold_position_;
    JointCommand command_;
    bool command_valid_{ false };
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //



}
