#include "dm_ros_control/pinocchio_dynamics_model.hpp"

#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <sstream>
#include <stdexcept>
#include <unordered_set>

namespace dm_ros_control {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 函 数 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

/**
 * @brief 构造 Pinocchio reduced model
 * @param urdf_path 机器人 URDF 路径
 * @param joint_names 需要保留的受控关节名称
 *
 * 从完整 URDF 模型中锁定非受控关节，只保留 ros2_control 管理的 1-DoF 关节，
 * 后续动力学输出顺序与 joint_names 保持一致。
 */
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

/**
 * @brief 更新受控关节状态并计算动力学项
 * @param q 受控关节位置
 * @param dq 受控关节速度
 * @return 成功返回 true，输入维度或内部索引异常返回 false
 */
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

/**
 * @brief 拷贝最近一次 update() 得到的重力项
 * @param gravity 输出重力项缓冲
 */
void PinocchioDynamicsModel::copy_gravity_to(std::vector<double>& gravity) const {
    if(gravity.size() != static_cast<size_t>(_g_.size())) gravity.resize(_g_.size());
    for(size_t i = 0; i < gravity.size(); ++i) gravity[i] = _g_[i];
}

/**
 * @brief 拷贝最近一次 update() 得到的非线性项
 * @param nonlinear 输出非线性项缓冲
 */
void PinocchioDynamicsModel::copy_nonlinear_effects_to(std::vector<double>& nonlinear) const {
    if(nonlinear.size() != static_cast<size_t>(_nle_.size())) nonlinear.resize(_nle_.size());
    for(size_t i = 0; i < nonlinear.size(); ++i) nonlinear[i] = _nle_[i];
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //



}
