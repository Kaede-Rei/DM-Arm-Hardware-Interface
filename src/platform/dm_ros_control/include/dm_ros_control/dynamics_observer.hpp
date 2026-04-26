#pragma once

#include "dm_ros_control/pinocchio_dynamics_model.hpp"

#include <memory>
#include <string>
#include <vector>

namespace dm_ros_control {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

struct DynamicsObservation {
    bool valid{ false };
    std::vector<double> gravity;
    std::vector<double> nonlinear;
    std::vector<double> active_feedforward;
    std::vector<double> external_effort;
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief 动力学观测器，只负责从关节状态估计重力/非线性项和外力
 */
class DynamicsObserver {
public:
    void configure(const std::string& urdf_path, const std::vector<std::string>& joint_names);
    void cleanup();

    DynamicsObservation observe(
        const std::vector<double>& positions,
        const std::vector<double>& velocities,
        const std::vector<double>& efforts,
        bool enable_gravity_feedforward,
        bool enable_nonlinear_feedforward);

private:
    std::shared_ptr<PinocchioDynamicsModel> _dynamics_model_;
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //



}
