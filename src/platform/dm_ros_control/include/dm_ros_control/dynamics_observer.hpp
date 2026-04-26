#pragma once

#include "dm_ros_control/pinocchio_dynamics_model.hpp"

namespace dm_ros_control {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief 单次动力学观测输出
 */
struct DynamicsObservation {
    bool valid{ false };                         ///< 观测是否成功
    std::vector<double> gravity;                 ///< 重力项
    std::vector<double> nonlinear;               ///< 非线性项
    std::vector<double> active_feedforward;      ///< 当前启用的前馈力矩
    std::vector<double> external_effort;         ///< 扣除前馈后的外部力矩估计
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief 动力学观测器，只负责从关节状态估计重力/非线性项和外力
 */
class DynamicsObserver {
public:
    /**
     * @brief 构建 Pinocchio 动力学模型
     * @param urdf_path 机器人 URDF 路径
     * @param joint_names 需要观测的关节名称
     */
    void configure(const std::string& urdf_path, const std::vector<std::string>& joint_names);

    /**
     * @brief 清理动力学模型
     */
    void cleanup();

    /**
     * @brief 根据当前关节状态计算动力学观测量
     * @param positions 关节位置
     * @param velocities 关节速度
     * @param efforts 关节侧测得力矩
     * @param enable_gravity_feedforward 是否选择重力项作为 active_feedforward
     * @param enable_nonlinear_feedforward 是否选择非线性项作为 active_feedforward
     * @param observation 预分配的动力学观测输出
     * @return 成功返回 true，失败返回 false
     */
    bool observe(
        const std::vector<double>& positions,
        const std::vector<double>& velocities,
        const std::vector<double>& efforts,
        bool enable_gravity_feedforward,
        bool enable_nonlinear_feedforward,
        DynamicsObservation& observation);

private:
    std::shared_ptr<PinocchioDynamicsModel> _dynamics_model_;
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //



}
