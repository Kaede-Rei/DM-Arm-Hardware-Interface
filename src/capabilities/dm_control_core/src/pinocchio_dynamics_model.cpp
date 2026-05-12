#include "dm_control_core/pinocchio_dynamics_model.hpp"

#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <unordered_set>

namespace dm_control_core {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 函 数 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

/**
 * @brief 构造 Pinocchio reduced model
 * @param urdf_path 机器人 URDF 路径
 * @param joint_names 需要保留的受控关节名称
 * @note 从完整 URDF 模型中锁定非受控关节，只保留 ros2_control 管理的 1-DoF 关节，
 *       后续动力学输出顺序与 joint_names 保持一致
 */
PinocchioDynamicsModel::PinocchioDynamicsModel(const std::string& urdf_path, const std::vector<std::string>& joint_names) {
    joint_names_ = joint_names;

    pinocchio::Model full_model;
    pinocchio::urdf::buildModel(urdf_path, full_model);

    for(const auto& name : joint_names_) {
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

    std::unordered_set<std::string> controlled_joint_set(joint_names_.begin(), joint_names_.end());
    std::vector<pinocchio::JointIndex> joints_to_lock;
    joints_to_lock.reserve(full_model.njoints);

    for(pinocchio::JointIndex jid = 1; jid < full_model.joints.size(); ++jid) {
        const std::string& joint_name = full_model.names[jid];
        if(controlled_joint_set.find(joint_name) == controlled_joint_set.end()) joints_to_lock.push_back(jid);
    }

    const Eigen::VectorXd q_ref = pinocchio::neutral(full_model);
    model_ = pinocchio::buildReducedModel(full_model, joints_to_lock, q_ref);
    data_ = std::make_shared<pinocchio::Data>(model_);

    if(model_.nq != static_cast<int>(joint_names_.size()) || model_.nv != static_cast<int>(joint_names_.size())) {
        std::ostringstream oss;
        oss << "PinocchioDynamicsModel: reduced model dimension mismatch. "
            << "Expected nq=nv=" << joint_names_.size() << ", but got nq=" << model_.nq << ", nv=" << model_.nv << ".";
        throw std::runtime_error(oss.str());
    }

    q_indices_.resize(joint_names_.size(), -1);
    v_indices_.resize(joint_names_.size(), -1);

    for(size_t i = 0; i < joint_names_.size(); ++i) {
        const pinocchio::JointIndex jid = model_.getJointId(joint_names_[i]);
        if(jid == model_.joints.size()) {
            std::ostringstream oss;
            oss << "PinocchioDynamicsModel: joint [" << joint_names_[i] << "] not found in reduced model.";
            throw std::runtime_error(oss.str());
        }

        if(model_.nqs[jid] != 1 || model_.nvs[jid] != 1) {
            std::ostringstream oss;
            oss << "PinocchioDynamicsModel: joint [" << joint_names_[i] << "] is not 1-DoF in reduced model (nq=" << model_.nqs[jid] << ", nv=" << model_.nvs[jid] << ").";
            throw std::runtime_error(oss.str());
        }

        q_indices_[i] = model_.idx_qs[jid];
        v_indices_[i] = model_.idx_vs[jid];
    }

    q_ = Eigen::VectorXd::Zero(model_.nq);
    dq_ = Eigen::VectorXd::Zero(model_.nv);
    g_ = Eigen::VectorXd::Zero(model_.nv);
    nle_ = Eigen::VectorXd::Zero(model_.nv);
    m_q_ = Eigen::MatrixXd::Zero(model_.nv, model_.nv);
}

/**
 * @brief 更新受控关节状态并计算动力学项
 * @param q 受控关节位置
 * @param dq 受控关节速度
 * @return 成功返回 true，输入维度或内部索引异常返回 false
 */
bool PinocchioDynamicsModel::update(const std::vector<double>& q, const std::vector<double>& dq,
    bool enable_gravity, bool enable_nonlinear, bool enable_mass_matrix) {
    if(q.size() != joint_names_.size() || dq.size() != joint_names_.size()) return false;

    q_.setZero();
    dq_.setZero();

    for(size_t i = 0; i < joint_names_.size(); ++i) {
        if(q_indices_[i] < 0 || v_indices_[i] < 0) return false;
        q_[q_indices_[i]] = q[i];
        dq_[v_indices_[i]] = dq[i];
    }

    if(enable_nonlinear) {
        pinocchio::nonLinearEffects(model_, *data_, q_, dq_);
        for(size_t i = 0; i < joint_names_.size(); ++i) {
            if(v_indices_[i] < 0) return false;
            nle_[i] = data_->nle[v_indices_[i]];
        }
    }
    else {
        nle_.setZero();
    }

    if(enable_gravity) {
        pinocchio::computeGeneralizedGravity(model_, *data_, q_);
        for(size_t i = 0; i < joint_names_.size(); ++i) {
            if(v_indices_[i] < 0) return false;
            g_[i] = data_->g[v_indices_[i]];
        }
    }
    else {
        g_.setZero();
    }

    if(enable_mass_matrix) {
        pinocchio::crba(model_, *data_, q_);
        for(size_t i = 0; i < joint_names_.size(); ++i) {
            if(v_indices_[i] < 0) return false;
            for(size_t j = 0; j < joint_names_.size(); ++j) {
                if(v_indices_[j] < 0) return false;
                m_q_(i, j) = data_->M(v_indices_[i], v_indices_[j]);
            }
        }
    }

    return true;
}

/**
 * @brief 拷贝最近一次 update() 得到的重力项
 * @param gravity 输出重力项缓冲
 */
void PinocchioDynamicsModel::copy_gravity_to(std::vector<double>& gravity) const {
    if(gravity.size() != static_cast<size_t>(g_.size())) gravity.resize(g_.size());
    for(size_t i = 0; i < gravity.size(); ++i) gravity[i] = g_[i];
}

/**
 * @brief 拷贝最近一次 update() 得到的非线性项
 * @param nonlinear 输出非线性项缓冲
 */
void PinocchioDynamicsModel::copy_nonlinear_effects_to(std::vector<double>& nonlinear) const {
    if(nonlinear.size() != static_cast<size_t>(nle_.size())) nonlinear.resize(nle_.size());
    for(size_t i = 0; i < nonlinear.size(); ++i) nonlinear[i] = nle_[i];
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //



}
