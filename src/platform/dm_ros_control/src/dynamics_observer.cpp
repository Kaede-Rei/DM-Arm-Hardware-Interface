#include "dm_ros_control/dynamics_observer.hpp"

namespace dm_ros_control {

// ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

void DynamicsObserver::configure(const std::string& urdf_path, const std::vector<std::string>& joint_names) {
    _dynamics_model_ = std::make_shared<PinocchioDynamicsModel>(urdf_path, joint_names);
}

void DynamicsObserver::cleanup() {
    _dynamics_model_.reset();
}

DynamicsObservation DynamicsObserver::observe(
    const std::vector<double>& positions,
    const std::vector<double>& velocities,
    const std::vector<double>& efforts,
    bool enable_gravity_feedforward,
    bool enable_nonlinear_feedforward) {

    DynamicsObservation observation;
    observation.gravity.assign(positions.size(), 0.0);
    observation.nonlinear.assign(positions.size(), 0.0);
    observation.active_feedforward.assign(positions.size(), 0.0);
    observation.external_effort.assign(positions.size(), 0.0);

    if(!_dynamics_model_) return observation;
    if(!_dynamics_model_->update(positions, velocities)) return observation;

    observation.gravity = _dynamics_model_->get_gravity_std();
    observation.nonlinear = _dynamics_model_->get_nonlinear_effects_std();
    observation.valid = true;

    for(size_t i = 0; i < positions.size(); ++i) {
        if(enable_nonlinear_feedforward) observation.active_feedforward[i] = observation.nonlinear[i];
        else if(enable_gravity_feedforward) observation.active_feedforward[i] = observation.gravity[i];
        else observation.active_feedforward[i] = 0.0;

        observation.external_effort[i] = efforts[i] - observation.active_feedforward[i];
    }

    return observation;
}

}
