#include "dm_ros_control/pinocchio_dynamics_model.hpp"

#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace dm_ros_control {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 函 数 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

PinocchioDynamicsModel::PinocchioDynamicsModel(const std::string& urdf_path, const std::vector<std::string>& joint_names) {
    pinocchio::urdf::buildModel(urdf_path, _model_);
    _data_ = std::make_shared<pinocchio::Data>(_model_);
    _joint_names_ = joint_names;

    _q_ = Eigen::VectorXd::Zero(_model_.nq);
    _dq_ = Eigen::VectorXd::Zero(_model_.nv);
    _g_ = Eigen::VectorXd::Zero(_model_.nv);
}

bool PinocchioDynamicsModel::update(const std::vector<double>& q, const std::vector<double>& dq) {
    if(q.size() != static_cast<size_t>(_model_.nq) || dq.size() != static_cast<size_t>(_model_.nv)) return false;
    _q_ = Eigen::Map<const Eigen::VectorXd>(q.data(), q.size());
    _dq_ = Eigen::Map<const Eigen::VectorXd>(dq.data(), dq.size());

    pinocchio::computeGeneralizedGravity(_model_, *_data_, _q_);
    _g_ = _data_->g;

    pinocchio::nonLinearEffects(_model_, *_data_, _q_, _dq_);
    _nle_ = _data_->nle;

    pinocchio::crba(_model_, *_data_, _q_);
    _m_q_ = _data_->M;

    return true;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //



}
