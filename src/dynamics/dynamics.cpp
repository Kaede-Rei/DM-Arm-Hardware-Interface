#include "dm_arm/dynamics/dynamics.hpp"

#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <Eigen/Cholesky>

#include <filesystem>
#include <unordered_set>

namespace dm_arm {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //

struct Dynamics::Impl {
    DynamicsCfg cfg;
    DynamicsInfo info;

    pinocchio::Model model;
    std::unique_ptr<pinocchio::Data> data;

    std::vector<int> q_indices;
    std::vector<int> v_indices;

    pinocchio::FrameIndex base_frame_id{ 0 };
    pinocchio::FrameIndex tool_frame_id{ 0 };

    Eigen::VectorXd q_model;
    Eigen::VectorXd dq_model;
    Eigen::VectorXd ddq_model;
    Eigen::VectorXd tau_model;

    Eigen::MatrixXd jacobian_model;

    bool is_configured{ false };
};

namespace {

/**
 * @brief 检查关节向量是否包含有限值
 * @param values 关节向量
 * @return 如果所有值都是有限的，则返回 true，否则返回 false
 */
bool finite_vector(const JointVector& values) {
    return std::all_of(values.begin(), values.end(),
        [](double value) {
            return std::isfinite(value);
        }
    );
}

/**
 * @brief 验证关节向量的大小和有限性
 * @param values 关节向量
 * @param expected_size 期望的关节向量大小
 * @return 成功时返回空值；失败时返回对应的 DynamicsErr
 */
tl::expected<void, DynamicsErr> validate_joint_vector(const JointVector& values, std::size_t expected_size) {
    if(values.size() != expected_size) {
        return tl::make_unexpected(DynamicsErr::INVALID_INPUT_SIZE);
    }

    if(!finite_vector(values)) {
        return tl::make_unexpected(DynamicsErr::NON_FINITE_INPUT);
    }

    return {};
}

/**
 * @brief 将关节向量的值分配到目标向量的指定索引位置
 * @param source 源关节向量
 * @param indices 目标向量中对应的索引位置
 * @param target 目标向量，将在指定索引位置被赋值
 */
void assign_joint_vector(const JointVector& source, const std::vector<int>& indices, Eigen::VectorXd& target) {
    target.setZero();

    for(std::size_t i = 0; i < source.size(); ++i) {
        target[indices[i]] = source[i];
    }
}

/**
 * @brief 从源向量中提取指定索引位置的关节向量
 * @param source 源向量
 * @param indices 需要提取的索引位置
 * @return 提取后的关节向量
 */
JointVector extract_joint_vector(const Eigen::VectorXd& source, const std::vector<int>& indices) {
    JointVector output(indices.size(), 0.0);

    for(std::size_t i = 0; i < indices.size(); ++i) {
        output[i] = source[indices[i]];
    }

    return output;
}

} // namespace

// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

/**
 * @brief 构造一个尚未配置的动力学对象
 */
Dynamics::Dynamics() : impl_(std::make_unique<Impl>()) {

}

/**
 * @brief 析构动力学对象并释放内部模型资源
 */
Dynamics::~Dynamics() = default;

/**
 * @brief 移动构造动力学对象
 * @param other 被移动的动力学对象
 */
Dynamics::Dynamics(Dynamics&& other) noexcept = default;

/**
 * @brief 移动赋值动力学对象
 * @param other 被移动的动力学对象
 * @return 当前对象引用
 */
Dynamics& Dynamics::operator=(Dynamics&& other) noexcept = default;

/**
 * @brief 根据配置加载并初始化动力学模型
 * @param cfg 动力学配置，包括 URDF 路径和受控关节名称等信息
 * @return 成功时返回空值；失败时返回对应的 DynamicsErr
 */
tl::expected<void, DynamicsErr> Dynamics::configure(const DynamicsCfg& cfg) {
    if(impl_->is_configured) {
        return tl::make_unexpected(DynamicsErr::ALREADY_CONFIGURED);
    }

    if(cfg.urdf_path.empty() || cfg.joint_names.empty() ||
        cfg.base_frame.empty() || cfg.tool_frame.empty() ||
        (!cfg.gravity_scale.empty() && cfg.gravity_scale.size() != cfg.joint_names.size()) ||
        !std::isfinite(cfg.gravity[0]) || !std::isfinite(cfg.gravity[1]) || !std::isfinite(cfg.gravity[2])) {
        return tl::make_unexpected(DynamicsErr::INVALID_CFG);
    }

    if(!std::filesystem::exists(cfg.urdf_path)) {
        return tl::make_unexpected(DynamicsErr::URDF_LOAD_FAILED);
    }

    std::unordered_set<std::string> unique_names;
    for(const auto& name : cfg.joint_names) {
        if(name.empty() || !unique_names.insert(name).second) {
            return tl::make_unexpected(DynamicsErr::INVALID_CFG);
        }
    }

    pinocchio::Model full_model;
    try {
        pinocchio::urdf::buildModel(cfg.urdf_path, full_model);
    }
    catch(...) {
        return tl::make_unexpected(DynamicsErr::URDF_LOAD_FAILED);
    }

    for(const auto& name : cfg.joint_names) {
        const pinocchio::JointIndex jid = full_model.getJointId(name);

        if(jid == 0 || (int)jid >= full_model.njoints) {
            return tl::make_unexpected(DynamicsErr::JOINT_NOT_FOUND);
        }

        if(full_model.nqs[jid] != 1 || full_model.nvs[jid] != 1) {
            return tl::make_unexpected(DynamicsErr::JOINT_NOT_1DOF);
        }
    }

    std::unordered_set<std::string> controlled(cfg.joint_names.begin(), cfg.joint_names.end());
    std::vector<pinocchio::JointIndex> joints_to_lock;
    for(pinocchio::JointIndex jid = 1; (int)jid < full_model.njoints; ++jid) {
        const std::string& name = full_model.names[jid];

        if(controlled.find(name) == controlled.end()) {
            joints_to_lock.push_back(jid);
        }
    }

    const Eigen::VectorXd q_ref = pinocchio::neutral(full_model);
    pinocchio::Model reduced_model;
    try {
        reduced_model = pinocchio::buildReducedModel(full_model, joints_to_lock, q_ref);
    }
    catch(...) {
        return tl::make_unexpected(DynamicsErr::URDF_LOAD_FAILED);
    }

    if(reduced_model.nq != 6 || reduced_model.nv != 6) {
        return tl::make_unexpected(
            DynamicsErr::MODEL_SIZE_MISMATCH);
    }

    reduced_model.gravity.linear() = Eigen::Vector3d(cfg.gravity[0], cfg.gravity[1], cfg.gravity[2]);

    std::vector<int> q_indices(cfg.joint_names.size(), -1);
    std::vector<int> v_indices(cfg.joint_names.size(), -1);
    for(std::size_t i = 0; i < cfg.joint_names.size(); ++i) {
        const auto jid = reduced_model.getJointId(cfg.joint_names[i]);

        if(jid == 0 || (int)jid >= reduced_model.njoints) {
            return tl::make_unexpected(DynamicsErr::JOINT_NOT_FOUND);
        }

        if(reduced_model.nqs[jid] != 1 || reduced_model.nvs[jid] != 1) {
            return tl::make_unexpected(DynamicsErr::JOINT_NOT_1DOF);
        }

        q_indices[i] = reduced_model.idx_qs[jid];
        v_indices[i] = reduced_model.idx_vs[jid];
    }

    if(!reduced_model.existFrame(cfg.base_frame) || !reduced_model.existFrame(cfg.tool_frame)) {
        return tl::make_unexpected(DynamicsErr::FRAME_NOT_FOUND);
    }
    const auto base_frame_id = reduced_model.getFrameId(cfg.base_frame);
    const auto tool_frame_id = reduced_model.getFrameId(cfg.tool_frame);

    impl_->cfg = cfg;
    impl_->model = std::move(reduced_model);
    impl_->data = std::make_unique<pinocchio::Data>(impl_->model);
    impl_->q_indices = std::move(q_indices);
    impl_->v_indices = std::move(v_indices);
    impl_->base_frame_id = base_frame_id;
    impl_->tool_frame_id = tool_frame_id;

    impl_->q_model = Eigen::VectorXd::Zero(impl_->model.nq);
    impl_->dq_model = Eigen::VectorXd::Zero(impl_->model.nv);
    impl_->ddq_model = Eigen::VectorXd::Zero(impl_->model.nv);
    impl_->tau_model = Eigen::VectorXd::Zero(impl_->model.nv);
    impl_->jacobian_model = Eigen::MatrixXd::Zero(6, impl_->model.nv);

    impl_->info.joints_count = cfg.joint_names.size();
    impl_->info.nq = impl_->model.nq;
    impl_->info.nv = impl_->model.nv;
    impl_->info.joint_names = cfg.joint_names;
    impl_->info.q_indices = impl_->q_indices;
    impl_->info.v_indices = impl_->v_indices;
    impl_->info.total_mass = pinocchio::computeTotalMass(impl_->model);
    impl_->is_configured = true;

    return {};
}

/**
 * @brief 清理当前动力学模型并恢复未配置状态
 */
void Dynamics::cleanup() {
    impl_ = std::make_unique<Impl>();
}

/**
 * @brief 查询动力学模型是否已经完成配置
 * @return 已成功配置时返回 true，否则返回 false
 */
bool Dynamics::is_configured() const noexcept {
    return impl_ && impl_->is_configured;
}

/**
 * @brief 获取当前动力学模型的基本信息
 * @return DynamicsInfo 的只读引用
 */
const DynamicsInfo& Dynamics::get_info() const noexcept {
    return impl_->info;
}

/**
 * @brief 获取指定坐标系相对于模型根坐标系的位姿
 * @param q 关节位置向量，顺序应与 DynamicsInfo::joint_names 一致
 * @param frame_name 目标坐标系名称
 * @return 成功时返回目标坐标系位姿；失败时返回 DynamicsErr
 */
tl::expected<Eigen::Isometry3d, DynamicsErr> Dynamics::get_frame_pose(const JointVector& q, const std::string& frame_name) const {

}

/**
 * @brief 获取指定坐标系的几何 Jacobian
 * @param q 关节位置向量，顺序应与 DynamicsInfo::joint_names 一致
 * @param frame_name 目标坐标系名称
 * @return 成功时返回 6×N Jacobian；失败时返回 DynamicsErr
 */
tl::expected<Eigen::MatrixXd, DynamicsErr> Dynamics::get_frame_jacobian(const JointVector& q, const std::string& frame_name) const {

}

/**
 * @brief 获取当前关节姿态下的重力广义力
 * @param q 关节位置向量
 * @return 成功时返回重力广义力向量；失败时返回 DynamicsErr
 */
tl::expected<JointVector, DynamicsErr> Dynamics::get_gravity(const JointVector& q) const {

}

/**
 * @brief 获取完整非线性广义力
 * @param q 关节位置向量
 * @param dq 关节速度向量
 * @return 成功时返回非线性广义力向量；失败时返回 DynamicsErr
 */
tl::expected<JointVector, DynamicsErr> Dynamics::get_nonlinear(const JointVector& q, const JointVector& dq) const {

}

/**
 * @brief 获取科氏力和离心力广义力向量
 * @param q 关节位置向量
 * @param dq 关节速度向量
 * @return 成功时返回科氏力和离心力广义力向量；
 *         失败时返回 DynamicsErr
 */
tl::expected<JointVector, DynamicsErr> Dynamics::get_coriolis(const JointVector& q, const JointVector& dq) const {

}

/**
 * @brief 获取关节空间质量矩阵
 * @param q 关节位置向量
 * @return 成功时返回质量矩阵；失败时返回 DynamicsErr
 */
tl::expected<Eigen::MatrixXd, DynamicsErr> Dynamics::get_mass_matrix(const JointVector& q) const {

}

/**
 * @brief 获取关节空间逆动力学
 * @param q 关节位置向量
 * @param dq 关节速度向量
 * @param ddq 关节加速度向量
 * @return 成功时返回关节力矩向量；失败时返回 DynamicsErr
 */
tl::expected<JointVector, DynamicsErr> Dynamics::get_inverse_dynamics(const JointVector& q, const JointVector& dq, const JointVector& ddq) const {

}

/**
 * @brief 获取关节空间正向动力学
 * @param q 关节位置向量
 * @param dq 关节速度向量
 * @param tau 关节驱动力矩向量
 * @return 成功时返回关节加速度向量；失败时返回 DynamicsErr
 */
tl::expected<JointVector, DynamicsErr> Dynamics::get_forward_dynamics(const JointVector& q, const JointVector& dq, const JointVector& tau) const {

}

/**
 * @brief 批量计算指定关节状态下的主要动力学量
 * @param q 关节位置向量
 * @param dq 关节速度向量
 * @param output 用于接收计算结果的输出结构体
 * @return 成功时返回空值；失败时返回 DynamicsErr
 * @post 成功时 output 中所有成员均按照 DynamicsInfo::joint_names 的顺序更新
 */
tl::expected<void, DynamicsErr> Dynamics::compute_state(const JointVector& q, const JointVector& dq, DynamicsState& output) const {

}

// ! ========================= 私 有 类 方 法 实 现 ========================= ! //



} // namespace dm_arm
