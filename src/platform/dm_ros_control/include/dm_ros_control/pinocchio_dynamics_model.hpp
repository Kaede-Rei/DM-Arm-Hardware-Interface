#pragma once

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include <memory>
#include <string>
#include <vector>

namespace dm_ros_control {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

class PinocchioDynamicsModel {
public:
    PinocchioDynamicsModel(const std::string& urdf_path, const std::vector<std::string>& joint_names);
    ~PinocchioDynamicsModel() = default;

    bool update(const std::vector<double>& q, const std::vector<double>& dq);

    std::vector<double> get_gravity_std() const { return std::vector<double>(_g_.data(), _g_.data() + _g_.size()); }
    std::vector<double> get_nonlinear_effects_std() const { return std::vector<double>(_nle_.data(), _nle_.data() + _nle_.size()); }
    const Eigen::MatrixXd& get_mass_matrix() const { return _m_q_; }

private:
    pinocchio::Model _model_;
    std::shared_ptr<pinocchio::Data> _data_;
    std::vector<std::string> _joint_names_;
    std::vector<int> _q_indices_;
    std::vector<int> _v_indices_;

    Eigen::VectorXd _q_;
    Eigen::VectorXd _dq_;
    Eigen::VectorXd _g_;
    Eigen::VectorXd _nle_;
    Eigen::MatrixXd _m_q_;
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //



}
