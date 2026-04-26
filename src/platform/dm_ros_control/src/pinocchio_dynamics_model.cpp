#include "dm_ros_control/pinocchio_dynamics_model.hpp"

#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <sstream>
#include <unordered_set>

namespace dm_ros_control {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 函 数 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

PinocchioDynamicsModel::PinocchioDynamicsModel(const std::string& urdf_path, const std::vector<std::string>& joint_names) {
    _joint_names_ = joint_names;

    pinocchio::Model full_model;
    pinocchio::urdf::buildModel(urdf_path, full_model);

    for(const auto& name : _joint_names_) {
        const pinocchio::JointIndex jid = full_model.getJointId(name);
        if(jid == full_model.joints.size()) {
            std::ostringstream oss;
            oss << "PinocchioDynamicsModel: joint [" << name << "] not found in URDF model.";
            throw std::runtime_error(oss.str());
        }

        if(full_model.nqs[jid] != 1 || full_model.nvs[jid] != 1) {
            std::ostringstream oss;
            oss << "PinocchioDynamicsModel: joint [" << name << "] is not 1-DoF in full model (nq="
                << full_model.nqs[jid] << ", nv=" << full_model.nvs[jid] << ").";
            throw std::runtime_error(oss.str());
        }
    }

    std::unordered_set<std::string> controlled_joint_set(_joint_names_.begin(), _joint_names_.end());
    std::vector<pinocchio::JointIndex> joints_to_lock;
    joints_to_lock.reserve(full_model.njoints);

    for(pinocchio::JointIndex jid = 1; jid < full_model.joints.size(); ++jid) {
        const std::string& joint_name = full_model.names[jid];
        if(controlled_joint_set.find(joint_name) == controlled_joint_set.end()) joints_to_lock.push_back(jid);
    }

    const Eigen::VectorXd q_ref = pinocchio::neutral(full_model);
    _model_ = pinocchio::buildReducedModel(full_model, joints_to_lock, q_ref);
    _data_ = std::make_shared<pinocchio::Data>(_model_);

    if(_model_.nq != static_cast<int>(_joint_names_.size()) || _model_.nv != static_cast<int>(_joint_names_.size())) {
        std::ostringstream oss;
        oss << "PinocchioDynamicsModel: reduced model dimension mismatch. "
            << "Expected nq=nv=" << _joint_names_.size() << ", but got nq=" << _model_.nq << ", nv=" << _model_.nv << ".";
        throw std::runtime_error(oss.str());
    }

    _q_indices_.resize(_joint_names_.size(), -1);
    _v_indices_.resize(_joint_names_.size(), -1);

    for(size_t i = 0; i < _joint_names_.size(); ++i) {
        const pinocchio::JointIndex jid = _model_.getJointId(_joint_names_[i]);
        if(jid == _model_.joints.size()) {
            std::ostringstream oss;
            oss << "PinocchioDynamicsModel: joint [" << _joint_names_[i] << "] not found in reduced model.";
            throw std::runtime_error(oss.str());
        }

        if(_model_.nqs[jid] != 1 || _model_.nvs[jid] != 1) {
            std::ostringstream oss;
            oss << "PinocchioDynamicsModel: joint [" << _joint_names_[i] << "] is not 1-DoF in reduced model (nq=" << _model_.nqs[jid] << ", nv=" << _model_.nvs[jid] << ").";
            throw std::runtime_error(oss.str());
        }

        _q_indices_[i] = _model_.idx_qs[jid];
        _v_indices_[i] = _model_.idx_vs[jid];
    }

    _q_ = Eigen::VectorXd::Zero(_model_.nq);
    _dq_ = Eigen::VectorXd::Zero(_model_.nv);
    _g_ = Eigen::VectorXd::Zero(_model_.nv);
    _nle_ = Eigen::VectorXd::Zero(_model_.nv);
    _m_q_ = Eigen::MatrixXd::Zero(_model_.nv, _model_.nv);
}

bool PinocchioDynamicsModel::update(const std::vector<double>& q, const std::vector<double>& dq) {
    if(q.size() != _joint_names_.size() || dq.size() != _joint_names_.size()) return false;

    _q_.setZero();
    _dq_.setZero();

    for(size_t i = 0; i < _joint_names_.size(); ++i) {
        if(_q_indices_[i] < 0 || _v_indices_[i] < 0) return false;
        _q_[_q_indices_[i]] = q[i];
        _dq_[_v_indices_[i]] = dq[i];
    }

    pinocchio::computeGeneralizedGravity(_model_, *_data_, _q_);
    for(size_t i = 0; i < _joint_names_.size(); ++i) {
        if(_v_indices_[i] < 0) return false;
        _g_[i] = _data_->g[_v_indices_[i]];
    }

    pinocchio::nonLinearEffects(_model_, *_data_, _q_, _dq_);
    for(size_t i = 0; i < _joint_names_.size(); ++i) {
        if(_v_indices_[i] < 0) return false;
        _nle_[i] = _data_->nle[_v_indices_[i]];
    }

    pinocchio::crba(_model_, *_data_, _q_);
    for(size_t i = 0; i < _joint_names_.size(); ++i) {
        if(_v_indices_[i] < 0) return false;
        for(size_t j = 0; j < _joint_names_.size(); ++j) {
            if(_v_indices_[j] < 0) return false;
            _m_q_(i, j) = _data_->M(_v_indices_[i], _v_indices_[j]);
        }
    }

    return true;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //



}
