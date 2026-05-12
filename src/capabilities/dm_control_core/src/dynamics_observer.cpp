#include "dm_control_core/dynamics_observer.hpp"

namespace dm_control_core {

// ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

/**
 * @brief 配置动力学观测器
 * @param urdf_path 机器人 URDF 路径
 * @param joint_names 受控关节名称列表
 */
void DynamicsObserver::configure(const std::string& urdf_path, const std::vector<std::string>& joint_names) {
    dynamics_model_ = std::make_shared<PinocchioDynamicsModel>(urdf_path, joint_names);
}

/**
 * @brief 清理动力学模型
 */
void DynamicsObserver::cleanup() {
    dynamics_model_.reset();
}

/**
 * @brief 根据关节状态计算动力学观测量
 * @param positions 关节位置
 * @param velocities 关节速度
 * @param efforts 关节侧实测力矩
 * @param enable_gravity_feedforward 是否选择重力项作为 active_feedforward
 * @param enable_nonlinear_feedforward 是否选择非线性项作为 active_feedforward
 * @param observation 预分配观测结果输出
 * @return 成功返回 true，失败返回 false
 */
bool DynamicsObserver::observe(const std::vector<double>& positions, const std::vector<double>& velocities, const std::vector<double>& efforts,
    bool enable_gravity_feedforward, bool enable_nonlinear_feedforward, DynamicsObservation& observation) {

    if(observation.gravity.size() != positions.size()) observation.gravity.assign(positions.size(), 0.0);
    if(observation.nonlinear.size() != positions.size()) observation.nonlinear.assign(positions.size(), 0.0);
    if(observation.active_feedforward.size() != positions.size()) observation.active_feedforward.assign(positions.size(), 0.0);
    if(observation.external_effort.size() != positions.size()) observation.external_effort.assign(positions.size(), 0.0);

    observation.valid = false;
    if(!dynamics_model_) return false;
    if(!dynamics_model_->update(positions, velocities,
        enable_gravity_feedforward, enable_nonlinear_feedforward)) return false;

    dynamics_model_->copy_gravity_to(observation.gravity);
    dynamics_model_->copy_nonlinear_effects_to(observation.nonlinear);
    observation.valid = true;

    for(size_t i = 0; i < positions.size(); ++i) {
        if(enable_nonlinear_feedforward) observation.active_feedforward[i] = observation.nonlinear[i];
        else if(enable_gravity_feedforward) observation.active_feedforward[i] = observation.gravity[i];
        else observation.active_feedforward[i] = 0.0;

        observation.external_effort[i] = efforts[i] - observation.active_feedforward[i];
    }

    return true;
}

}
