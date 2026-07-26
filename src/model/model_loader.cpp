#include "dm_arm/model/model_loader.hpp"

#include <pinocchio/parsers/urdf.hpp>

#include <cmath>
#include <filesystem>
#include <set>
#include <unordered_map>

namespace dm_arm {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //

namespace {

/**
 * @brief 检查标量是否为有限值
 */
bool finite_positive(double value) {
    return std::isfinite(value) && value > 0.0;
}

} // namespace

// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

/**
 * @brief 加载 URDF 中的受控 Joint 信息
 * @param urdf_path URDF 文件路径
 * @param controlled_joint_names 受控 Joint 名称
 * @return 成功返回模型信息，失败返回 ModelErr
 */
tl::expected<RobotModelInfo, ModelErr> ModelLoader::load(const std::string& urdf_path, const std::vector<std::string>& controlled_joint_names) const {
    if(urdf_path.empty() || !std::filesystem::exists(urdf_path)) return tl::make_unexpected(ModelErr::FILE_OPEN_FAILED);

    std::set<std::string> requested;
    for(const auto& name : controlled_joint_names) {
        if(name.empty() || !requested.insert(name).second) return tl::make_unexpected(ModelErr::DUPLICATE_JOINT);
    }

    pinocchio::Model model;
    try {
        pinocchio::urdf::buildModel(urdf_path, model);
    }
    catch(...) {
        return tl::make_unexpected(ModelErr::URDF_LOAD_FAILED);
    }

    std::unordered_map<std::string, ModelJointLimit> limit_by_name;
    for(pinocchio::JointIndex joint_id = 1; joint_id < static_cast<pinocchio::JointIndex>(model.joints.size()); ++joint_id) {
        const std::string& name = model.names[joint_id];
        if(limit_by_name.find(name) != limit_by_name.end()) return tl::make_unexpected(ModelErr::DUPLICATE_JOINT);

        const int nq = model.joints[joint_id].nq();
        const int nv = model.joints[joint_id].nv();
        if(nv == 0) continue;

        ModelJointLimit limit;
        limit.name = name;
        if(nq == 1 && nv == 1) {
            const int q_index = model.joints[joint_id].idx_q();
            const int v_index = model.joints[joint_id].idx_v();
            limit.min_pos = model.lowerPositionLimit[q_index];
            limit.max_pos = model.upperPositionLimit[q_index];
            limit.max_vel = model.velocityLimit[v_index];
            limit.max_effort = model.effortLimit[v_index];
            limit.has_position_limit = std::isfinite(limit.min_pos) && std::isfinite(limit.max_pos) && limit.min_pos < limit.max_pos;
            if(!finite_positive(limit.max_vel) || !finite_positive(limit.max_effort)) return tl::make_unexpected(ModelErr::INVALID_LIMIT);
        }
        else if(nq == 2 && nv == 1) {
            const int v_index = model.joints[joint_id].idx_v();
            limit.has_position_limit = false;
            limit.max_vel = model.velocityLimit[v_index];
            limit.max_effort = model.effortLimit[v_index];
            if(!finite_positive(limit.max_vel) || !finite_positive(limit.max_effort)) return tl::make_unexpected(ModelErr::INVALID_LIMIT);
        }
        else {
            continue;
        }
        limit_by_name.emplace(name, limit);
    }

    RobotModelInfo info;
    info.urdf_path = std::filesystem::absolute(urdf_path).lexically_normal().string();
    info.joint_names = controlled_joint_names;
    info.joint_limits.reserve(controlled_joint_names.size());
    for(const auto& name : controlled_joint_names) {
        const auto found = limit_by_name.find(name);
        if(found == limit_by_name.end()) return tl::make_unexpected(ModelErr::MISSING_JOINT);
        info.joint_limits.push_back(found->second);
    }
    return info;
}

// ! ========================= 私 有 类 方 法 实 现 ========================= ! //

} // namespace dm_arm
