#pragma once

#include <tl/expected.hpp>

#include <string>
#include <vector>

namespace dm_arm {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief 模型加载错误类型
 */
enum class ModelErr {
    FILE_OPEN_FAILED,       ///< URDF 文件无法打开
    URDF_LOAD_FAILED,       ///< URDF 解析失败
    MISSING_JOINT,          ///< 缺少受控 Joint
    DUPLICATE_JOINT,        ///< Joint 名称重复
    FIXED_JOINT_CONTROLLED, ///< fixed Joint 不能作为受控 Joint
    INVALID_LIMIT,          ///< Joint limit 字段无效
};

/**
 * @brief URDF 中的单个 Joint 限制
 */
struct ModelJointLimit {
    std::string name;                 ///< Joint 名称
    bool has_position_limit{ false }; ///< 是否存在位置限制
    double min_pos{ 0.0 };            ///< 位置下限
    double max_pos{ 0.0 };            ///< 位置上限
    double max_vel{ 0.0 };            ///< 速度上限
    double max_effort{ 0.0 };         ///< 力矩上限
};

/**
 * @brief 机器人模型信息
 */
struct RobotModelInfo {
    std::string urdf_path;                       ///< URDF 文件路径
    std::vector<std::string> joint_names;        ///< 受控 Joint 名称，顺序匹配请求
    std::vector<ModelJointLimit> joint_limits;   ///< 受控 Joint 限制，顺序匹配请求
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief 从 URDF 加载受控 Joint 模型信息
 */
class ModelLoader {
public:
    /**
     * @brief 加载 URDF 中的受控 Joint 信息
     * @param urdf_path URDF 文件路径
     * @param controlled_joint_names 受控 Joint 名称
     * @return 成功返回模型信息，失败返回 ModelErr
     */
    tl::expected<RobotModelInfo, ModelErr> load(const std::string& urdf_path, const std::vector<std::string>& controlled_joint_names) const;
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //

} // namespace dm_arm
