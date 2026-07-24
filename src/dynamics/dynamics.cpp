#include "dm_arm/dynamics/dynamics.hpp"

#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <unordered_set>

namespace dm_arm {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //

struct Dynamics::Impl {
    DynamicsCfg cfg;        ///< 动力学配置
    DynamicsInfo info;      ///< 动力学模型信息
    DynamicsState state;    ///< 最近一次成功更新的动力学缓存

    pinocchio::Model model;                 ///< Pinocchio 模型对象
    std::unique_ptr<pinocchio::Data> data;  ///< Pinocchio 数据对象

    std::vector<int> q_indices;     ///< 受控关节位置索引
    std::vector<int> v_indices;     ///< 受控关节速度索引

    pinocchio::FrameIndex base_frame_id{ 0 };   ///< base_frame 索引
    pinocchio::FrameIndex tool_frame_id{ 0 };   ///< tool_frame 索引

    Eigen::VectorXd q_model;        ///< 完整模型位置向量
    Eigen::VectorXd dq_model;       ///< 完整模型速度向量
    Eigen::VectorXd ddq_model;      ///< 完整模型参考加速度向量
    Eigen::VectorXd tau_model;      ///< 完整模型反馈力矩向量

    Eigen::MatrixXd frame_jacobian_model;           ///< 完整模型 Frame Jacobian 临时缓存
    std::vector<Eigen::Isometry3d> frame_poses;     ///< 所有 Frame 位姿缓存
    std::vector<Eigen::MatrixXd> frame_jacobians;   ///< 所有 Frame Jacobian 缓存

    bool is_configured{ false };    ///< 是否已经完成配置
    bool is_updated{ false };       ///< 是否已经完成至少一次更新
};

namespace {

/**
 * @brief 检查关节向量是否包含有限值
 * @param values 关节向量
 * @return 所有值均有限时返回 true，否则返回 false
 */
bool finite_vector(const JointVector& values) {
    return std::all_of(values.begin(), values.end(), [](double value) {
        return std::isfinite(value);
        });
}

/**
 * @brief 验证关节向量的大小和有限性
 * @param values 关节向量
 * @param expected_size 期望大小
 * @return 成功时返回空值；失败时返回 DynamicsErr
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
 * @brief 验证重力补偿缩放系数
 * @param gravity_scale 重力补偿缩放系数
 * @param expected_size 期望大小
 * @return 成功时返回空值；失败时返回 DynamicsErr
 */
tl::expected<void, DynamicsErr> validate_gravity_scale(const JointVector& gravity_scale, std::size_t expected_size) {
    if(gravity_scale.size() != expected_size) {
        return tl::make_unexpected(DynamicsErr::INVALID_INPUT_SIZE);
    }

    const bool finite = std::all_of(gravity_scale.begin(), gravity_scale.end(), [](double value) {
        return std::isfinite(value);
        });
    if(!finite) return tl::make_unexpected(DynamicsErr::NON_FINITE_INPUT);

    const bool in_range = std::all_of(gravity_scale.begin(), gravity_scale.end(), [](double value) {
        return value >= 0.0 && value <= 1.0;
        });
    if(!in_range) return tl::make_unexpected(DynamicsErr::GRAVITY_SCALE_OUT_OF_RANGE);

    return {};
}

/**
 * @brief 将关节向量写入完整模型向量
 * @param source 关节向量
 * @param indices 完整模型索引
 * @param target 完整模型向量
 */
void assign_joint_vector(const JointVector& source, const std::vector<int>& indices, Eigen::VectorXd& target) {
    target.setZero();

    for(std::size_t i = 0; i < source.size(); ++i) {
        target[indices[i]] = source[i];
    }
}

/**
 * @brief 从完整模型向量提取受控关节向量
 * @param source 完整模型向量
 * @param indices 完整模型索引
 * @param target 受控关节向量
 */
void extract_joint_vector(const Eigen::VectorXd& source, const std::vector<int>& indices, JointVector& target) {
    if(target.size() != indices.size()) {
        target.resize(indices.size());
    }

    for(std::size_t i = 0; i < indices.size(); ++i) {
        target[i] = source[indices[i]];
    }
}

/**
 * @brief 从完整质量矩阵提取受控关节子矩阵
 * @param source 完整质量矩阵
 * @param indices 受控关节速度索引
 * @param target 受控关节质量矩阵
 */
void extract_joint_matrix(const Eigen::MatrixXd& source, const std::vector<int>& indices, Eigen::MatrixXd& target) {
    const Eigen::Index size = static_cast<Eigen::Index>(indices.size());
    if(target.rows() != size || target.cols() != size) {
        target.resize(size, size);
    }

    for(std::size_t row = 0; row < indices.size(); ++row) {
        for(std::size_t col = 0; col < indices.size(); ++col) {
            target(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(col)) = source(indices[row], indices[col]);
        }
    }
}

/**
 * @brief 从完整 Frame Jacobian 提取受控关节列
 * @param source 完整 Frame Jacobian
 * @param indices 受控关节速度索引
 * @param target 受控关节 Frame Jacobian
 */
void extract_frame_jacobian(const Eigen::MatrixXd& source, const std::vector<int>& indices, Eigen::MatrixXd& target) {
    const Eigen::Index cols = static_cast<Eigen::Index>(indices.size());
    if(target.rows() != 6 || target.cols() != cols) {
        target.resize(6, cols);
    }

    for(std::size_t col = 0; col < indices.size(); ++col) {
        target.col(static_cast<Eigen::Index>(col)) = source.col(indices[col]);
    }
}

/**
 * @brief 将 Pinocchio SE3 转换为 Eigen Isometry3d
 * @param source Pinocchio 位姿
 * @return Eigen 位姿
 */
Eigen::Isometry3d to_isometry(const pinocchio::SE3& source) {
    Eigen::Isometry3d output = Eigen::Isometry3d::Identity();
    output.linear() = source.rotation();
    output.translation() = source.translation();
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
 * @param cfg 动力学配置
 * @return 成功时返回空值；失败时返回 DynamicsErr
 */
tl::expected<void, DynamicsErr> Dynamics::configure(const DynamicsCfg& cfg) {
    if(impl_->is_configured) {
        return tl::make_unexpected(DynamicsErr::ALREADY_CONFIGURED);
    }

    if(cfg.urdf_path.empty() || cfg.joint_names.empty() || cfg.base_frame.empty() || cfg.tool_frame.empty() ||
        !std::isfinite(cfg.gravity[0]) || !std::isfinite(cfg.gravity[1]) || !std::isfinite(cfg.gravity[2])) {
        return tl::make_unexpected(DynamicsErr::INVALID_CFG);
    }

    JointVector gravity_scale = cfg.gravity_scale;
    if(gravity_scale.empty()) {
        gravity_scale.assign(cfg.joint_names.size(), 0.0);
    }

    const auto gravity_scale_valid = validate_gravity_scale(gravity_scale, cfg.joint_names.size());
    if(!gravity_scale_valid) {
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
        const pinocchio::JointIndex joint_id = full_model.getJointId(name);
        if(joint_id == 0 || static_cast<int>(joint_id) >= full_model.njoints) {
            return tl::make_unexpected(DynamicsErr::JOINT_NOT_FOUND);
        }
        if(full_model.nqs[joint_id] != 1 || full_model.nvs[joint_id] != 1) {
            return tl::make_unexpected(DynamicsErr::JOINT_NOT_1DOF);
        }
    }

    std::unordered_set<std::string> controlled(cfg.joint_names.begin(), cfg.joint_names.end());
    std::vector<pinocchio::JointIndex> joints_to_lock;
    for(pinocchio::JointIndex joint_id = 1; static_cast<int>(joint_id) < full_model.njoints; ++joint_id) {
        if(controlled.find(full_model.names[joint_id]) == controlled.end()) {
            joints_to_lock.push_back(joint_id);
        }
    }

    pinocchio::Model reduced_model;
    try {
        reduced_model = pinocchio::buildReducedModel(full_model, joints_to_lock, pinocchio::neutral(full_model));
    }
    catch(...) {
        return tl::make_unexpected(DynamicsErr::URDF_LOAD_FAILED);
    }

    const int expected_size = static_cast<int>(cfg.joint_names.size());
    if(reduced_model.nq != expected_size || reduced_model.nv != expected_size) {
        return tl::make_unexpected(DynamicsErr::MODEL_SIZE_MISMATCH);
    }

    reduced_model.gravity.linear() = Eigen::Vector3d(cfg.gravity[0], cfg.gravity[1], cfg.gravity[2]);

    std::vector<int> q_indices(cfg.joint_names.size(), -1);
    std::vector<int> v_indices(cfg.joint_names.size(), -1);
    for(std::size_t i = 0; i < cfg.joint_names.size(); ++i) {
        const pinocchio::JointIndex joint_id = reduced_model.getJointId(cfg.joint_names[i]);
        if(joint_id == 0 || static_cast<int>(joint_id) >= reduced_model.njoints) {
            return tl::make_unexpected(DynamicsErr::JOINT_NOT_FOUND);
        }
        if(reduced_model.nqs[joint_id] != 1 || reduced_model.nvs[joint_id] != 1) {
            return tl::make_unexpected(DynamicsErr::JOINT_NOT_1DOF);
        }

        q_indices[i] = reduced_model.idx_qs[joint_id];
        v_indices[i] = reduced_model.idx_vs[joint_id];
    }

    if(!reduced_model.existFrame(cfg.base_frame) || !reduced_model.existFrame(cfg.tool_frame)) {
        return tl::make_unexpected(DynamicsErr::FRAME_NOT_FOUND);
    }

    const pinocchio::FrameIndex base_frame_id = reduced_model.getFrameId(cfg.base_frame);
    const pinocchio::FrameIndex tool_frame_id = reduced_model.getFrameId(cfg.tool_frame);
    if(static_cast<int>(base_frame_id) >= reduced_model.nframes || static_cast<int>(tool_frame_id) >= reduced_model.nframes) {
        return tl::make_unexpected(DynamicsErr::FRAME_NOT_FOUND);
    }

    impl_->cfg = cfg;
    impl_->cfg.gravity_scale = std::move(gravity_scale);
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
    impl_->frame_jacobian_model = Eigen::MatrixXd::Zero(6, impl_->model.nv);

    const std::size_t joints_count = cfg.joint_names.size();
    impl_->state.pos.assign(joints_count, 0.0);
    impl_->state.vel.assign(joints_count, 0.0);
    impl_->state.acc.assign(joints_count, 0.0);
    impl_->state.tor.assign(joints_count, 0.0);
    impl_->state.ref_acc.assign(joints_count, 0.0);
    impl_->state.gravity.assign(joints_count, 0.0);
    impl_->state.gravity_compensation.assign(joints_count, 0.0);
    impl_->state.nonlinear.assign(joints_count, 0.0);
    impl_->state.coriolis.assign(joints_count, 0.0);
    impl_->state.inverse_dynamics.assign(joints_count, 0.0);
    impl_->state.forward_dynamics.assign(joints_count, 0.0);
    impl_->state.mass_matrix = Eigen::MatrixXd::Zero(expected_size, expected_size);
    impl_->state.tool_pose = Eigen::Isometry3d::Identity();
    impl_->state.tool_jacobian = Eigen::MatrixXd::Zero(6, expected_size);

    impl_->frame_poses.assign(impl_->model.nframes, Eigen::Isometry3d::Identity());
    impl_->frame_jacobians.reserve(impl_->model.nframes);
    for(pinocchio::FrameIndex frame_id = 0; static_cast<int>(frame_id) < impl_->model.nframes; ++frame_id) {
        impl_->frame_jacobians.emplace_back(Eigen::MatrixXd::Zero(6, expected_size));
    }

    impl_->info.joints_count = joints_count;
    impl_->info.nq = impl_->model.nq;
    impl_->info.nv = impl_->model.nv;
    impl_->info.total_mass = pinocchio::computeTotalMass(impl_->model);
    impl_->info.joint_names = cfg.joint_names;
    impl_->info.q_indices = impl_->q_indices;
    impl_->info.v_indices = impl_->v_indices;

    impl_->is_configured = true;
    impl_->is_updated = false;
    return {};
}

/**
 * @brief 集中更新当前周期的全部运动学与动力学缓存
 * @param state 当前关节位置、速度和反馈力矩
 * @param acc 当前关节加速度估计
 * @param ref_acc 当前关节参考加速度
 * @return 成功时返回空值；失败时返回 DynamicsErr
 */
tl::expected<void, DynamicsErr> Dynamics::update(const JointState& state, const JointVector& acc, const JointVector& ref_acc) {
    if(!is_configured()) {
        return tl::make_unexpected(DynamicsErr::NOT_CONFIGURED);
    }

    const auto pos_valid = validate_joint_vector(state.pos, impl_->info.joints_count);
    const auto vel_valid = validate_joint_vector(state.vel, impl_->info.joints_count);
    const auto tor_valid = validate_joint_vector(state.tor, impl_->info.joints_count);
    const auto acc_valid = validate_joint_vector(acc, impl_->info.joints_count);
    const auto ref_acc_valid = validate_joint_vector(ref_acc, impl_->info.joints_count);
    if(!pos_valid) return tl::make_unexpected(pos_valid.error());
    if(!vel_valid) return tl::make_unexpected(vel_valid.error());
    if(!tor_valid) return tl::make_unexpected(tor_valid.error());
    if(!acc_valid) return tl::make_unexpected(acc_valid.error());
    if(!ref_acc_valid) return tl::make_unexpected(ref_acc_valid.error());

    assign_joint_vector(state.pos, impl_->q_indices, impl_->q_model);
    assign_joint_vector(state.vel, impl_->v_indices, impl_->dq_model);
    assign_joint_vector(ref_acc, impl_->v_indices, impl_->ddq_model);
    assign_joint_vector(state.tor, impl_->v_indices, impl_->tau_model);

    impl_->is_updated = false;

    try {
        pinocchio::computeAllTerms(impl_->model, *impl_->data, impl_->q_model, impl_->dq_model);
        pinocchio::updateFramePlacements(impl_->model, *impl_->data);

        const pinocchio::SE3 base_pose_inverse = impl_->data->oMf[impl_->base_frame_id].inverse();
        for(pinocchio::FrameIndex frame_id = 0; static_cast<int>(frame_id) < impl_->model.nframes; ++frame_id) {
            impl_->frame_poses[frame_id] = to_isometry(base_pose_inverse * impl_->data->oMf[frame_id]);
            impl_->frame_jacobian_model.setZero();
            pinocchio::getFrameJacobian(impl_->model, *impl_->data, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, impl_->frame_jacobian_model);
            extract_frame_jacobian(impl_->frame_jacobian_model, impl_->v_indices, impl_->frame_jacobians[frame_id]);
        }

        extract_joint_vector(impl_->data->g, impl_->v_indices, impl_->state.gravity);
        extract_joint_vector(impl_->data->nle, impl_->v_indices, impl_->state.nonlinear);
        for(std::size_t i = 0; i < impl_->info.joints_count; ++i) {
            impl_->state.gravity_compensation[i] = impl_->cfg.gravity_scale[i] * impl_->state.gravity[i];
            impl_->state.coriolis[i] = impl_->state.nonlinear[i] - impl_->state.gravity[i];
        }

        impl_->data->M.triangularView<Eigen::StrictlyLower>() = impl_->data->M.transpose().triangularView<Eigen::StrictlyLower>();
        extract_joint_matrix(impl_->data->M, impl_->v_indices, impl_->state.mass_matrix);

        const Eigen::VectorXd& inverse_dynamics = pinocchio::rnea(impl_->model, *impl_->data, impl_->q_model, impl_->dq_model, impl_->ddq_model);
        extract_joint_vector(inverse_dynamics, impl_->v_indices, impl_->state.inverse_dynamics);

        const Eigen::VectorXd& forward_dynamics = pinocchio::aba(impl_->model, *impl_->data, impl_->q_model, impl_->dq_model, impl_->tau_model);
        extract_joint_vector(forward_dynamics, impl_->v_indices, impl_->state.forward_dynamics);
    }
    catch(...) {
        return tl::make_unexpected(DynamicsErr::COMPUTE_FAILED);
    }

    impl_->state.pos = state.pos;
    impl_->state.vel = state.vel;
    impl_->state.acc = acc;
    impl_->state.tor = state.tor;
    impl_->state.ref_acc = ref_acc;
    impl_->state.tool_pose = impl_->frame_poses[impl_->tool_frame_id];
    impl_->state.tool_jacobian = impl_->frame_jacobians[impl_->tool_frame_id];
    impl_->is_updated = true;
    return {};
}

/**
 * @brief 更新重力补偿缩放系数
 * @param gravity_scale 重力补偿缩放系数
 * @return 成功时返回空值；失败时返回 DynamicsErr
 */
tl::expected<void, DynamicsErr> Dynamics::set_gravity_scale(const JointVector& gravity_scale) {
    if(!is_configured()) {
        return tl::make_unexpected(DynamicsErr::NOT_CONFIGURED);
    }

    const auto valid = validate_gravity_scale(gravity_scale, impl_->info.joints_count);
    if(!valid) {
        return tl::make_unexpected(valid.error());
    }

    impl_->cfg.gravity_scale = gravity_scale;
    if(impl_->is_updated) {
        for(std::size_t i = 0; i < impl_->info.joints_count; ++i) {
            impl_->state.gravity_compensation[i] = impl_->cfg.gravity_scale[i] * impl_->state.gravity[i];
        }
    }
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
 */
bool Dynamics::is_configured() const noexcept {
    return impl_ && impl_->is_configured;
}

/**
 * @brief 查询动力学缓存是否已经完成至少一次成功更新
 */
bool Dynamics::is_updated() const noexcept {
    return impl_ && impl_->is_updated;
}

/**
 * @brief 获取当前动力学模型的基本信息
 */
const DynamicsInfo& Dynamics::get_info() const noexcept {
    return impl_->info;
}

/**
 * @brief 获取最近一次 update() 的完整缓存
 */
const DynamicsState& Dynamics::get_state() const noexcept {
    return impl_->state;
}

/**
 * @brief 获取当前重力补偿缩放系数
 */
const JointVector& Dynamics::get_gravity_scale() const noexcept {
    return impl_->cfg.gravity_scale;
}

/**
 * @brief 获取指定坐标系相对于 base_frame 的缓存位姿
 */
tl::expected<Eigen::Isometry3d, DynamicsErr> Dynamics::get_frame_pose(const std::string& frame_name) const {
    if(!is_configured()) return tl::make_unexpected(DynamicsErr::NOT_CONFIGURED);
    if(!impl_->is_updated) return tl::make_unexpected(DynamicsErr::NOT_UPDATED);
    if(frame_name.empty() || !impl_->model.existFrame(frame_name)) return tl::make_unexpected(DynamicsErr::FRAME_NOT_FOUND);

    const pinocchio::FrameIndex frame_id = impl_->model.getFrameId(frame_name);
    if(frame_id >= impl_->frame_poses.size()) return tl::make_unexpected(DynamicsErr::FRAME_NOT_FOUND);
    return impl_->frame_poses[frame_id];
}

/**
 * @brief 获取指定坐标系的缓存几何 Jacobian
 */
tl::expected<Eigen::MatrixXd, DynamicsErr> Dynamics::get_frame_jacobian(const std::string& frame_name) const {
    if(!is_configured()) return tl::make_unexpected(DynamicsErr::NOT_CONFIGURED);
    if(!impl_->is_updated) return tl::make_unexpected(DynamicsErr::NOT_UPDATED);
    if(frame_name.empty() || !impl_->model.existFrame(frame_name)) return tl::make_unexpected(DynamicsErr::FRAME_NOT_FOUND);

    const pinocchio::FrameIndex frame_id = impl_->model.getFrameId(frame_name);
    if(frame_id >= impl_->frame_jacobians.size()) return tl::make_unexpected(DynamicsErr::FRAME_NOT_FOUND);
    return impl_->frame_jacobians[frame_id];
}

/**
 * @brief 获取最近一次 update() 的未缩放重力广义力
 */
const JointVector& Dynamics::get_gravity() const noexcept {
    return impl_->state.gravity;
}

/**
 * @brief 获取最近一次 update() 的缩放后重力补偿
 */
const JointVector& Dynamics::get_gravity_compensation() const noexcept {
    return impl_->state.gravity_compensation;
}

/**
 * @brief 获取最近一次 update() 的完整非线性广义力
 */
const JointVector& Dynamics::get_nonlinear() const noexcept {
    return impl_->state.nonlinear;
}

/**
 * @brief 获取最近一次 update() 的科氏力和离心力广义力
 */
const JointVector& Dynamics::get_coriolis() const noexcept {
    return impl_->state.coriolis;
}

/**
 * @brief 获取最近一次 update() 的关节空间质量矩阵
 */
const Eigen::MatrixXd& Dynamics::get_mass_matrix() const noexcept {
    return impl_->state.mass_matrix;
}

/**
 * @brief 获取最近一次 update() 的逆动力学结果
 */
const JointVector& Dynamics::get_inverse_dynamics() const noexcept {
    return impl_->state.inverse_dynamics;
}

/**
 * @brief 获取最近一次 update() 的正向动力学结果
 */
const JointVector& Dynamics::get_forward_dynamics() const noexcept {
    return impl_->state.forward_dynamics;
}

/**
 * @brief 获取最近一次 update() 的末端位姿
 */
const Eigen::Isometry3d& Dynamics::get_tool_pose() const noexcept {
    return impl_->state.tool_pose;
}

/**
 * @brief 获取最近一次 update() 的末端 Jacobian
 */
const Eigen::MatrixXd& Dynamics::get_tool_jacobian() const noexcept {
    return impl_->state.tool_jacobian;
}

// ! ========================= 私 有 类 方 法 实 现 ========================= ! //



} // namespace dm_arm
