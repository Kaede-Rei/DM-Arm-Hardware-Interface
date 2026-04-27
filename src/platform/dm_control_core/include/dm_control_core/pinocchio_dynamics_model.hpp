#pragma once

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include <memory>
#include <string>
#include <vector>

namespace dm_control_core {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief Pinocchio 动力学模型封装，用于计算受控关节的动力学项
 */
class PinocchioDynamicsModel {
public:
    /**
     * @brief 构造 Pinocchio reduced model
     * @param urdf_path 机器人 URDF 路径
     * @param joint_names 受控关节名称列表
     */
    PinocchioDynamicsModel(const std::string& urdf_path, const std::vector<std::string>& joint_names);

    /**
     * @brief 默认析构
     */
    ~PinocchioDynamicsModel() = default;

    /**
     * @brief 更新模型状态并计算重力项、非线性项和质量矩阵
     * @param q 受控关节位置
     * @param dq 受控关节速度
     * @return 更新成功返回 true，输入维度或索引异常返回 false
     */
    bool update(const std::vector<double>& q, const std::vector<double>& dq);

    /**
     * @brief 获取重力项拷贝
     * @return 重力项 vector
     */
    std::vector<double> get_gravity_std() const { return std::vector<double>(_g_.data(), _g_.data() + _g_.size()); }

    /**
     * @brief 获取非线性项拷贝
     * @return 非线性项 vector
     */
    std::vector<double> get_nonlinear_effects_std() const { return std::vector<double>(_nle_.data(), _nle_.data() + _nle_.size()); }

    /**
     * @brief 拷贝重力项到预分配缓冲，避免周期路径构造临时 vector
     * @param gravity 输出重力项
     */
    void copy_gravity_to(std::vector<double>& gravity) const;

    /**
     * @brief 拷贝非线性项到预分配缓冲，避免周期路径构造临时 vector
     * @param nonlinear 输出非线性项
     */
    void copy_nonlinear_effects_to(std::vector<double>& nonlinear) const;

    /**
     * @brief 获取质量矩阵引用
     * @return 质量矩阵
     */
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
