#pragma once

#include <tl/expected.hpp>

#include <string>
#include <vector>

namespace serial_arm {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief Robot Profile 解析错误类型
 */
enum class RobotProfileErr {
    PROFILE_FILE_NOT_FOUND,  ///< 找不到 Profile 文件
    PROFILE_LOAD_FAILED,     ///< Profile YAML 读取或解析失败
    PROFILE_NOT_FOUND,       ///< 指定 profile name 不存在
    MISSING_FIELD,           ///< 缺少必需字段
    RESOURCE_NOT_FOUND,      ///< 无法解析 package/resource 路径
};

/**
 * @brief Robot Profile 解析错误信息
 */
struct RobotProfileErrInfo {
    RobotProfileErr code{ RobotProfileErr::PROFILE_LOAD_FAILED }; ///< 错误码
    std::string message;                                          ///< 明确错误消息
};

/**
 * @brief framework-neutral Robot Profile Core/Hardware 部分
 */
struct RobotProfileCore {
    std::string name;                 ///< Profile 名称
    std::string profile_file;         ///< 使用的 robot_profiles.yaml
    std::string core_config_path;     ///< Core YAML 绝对路径
    std::string hardware_plugin;      ///< Hardware Backend plugin 名称或路径
    std::string hardware_config_path; ///< Hardware YAML 绝对路径
};

/**
 * @brief Robot Profile 搜索选项
 */
struct RobotProfileLoadOptions {
    std::string profile_file;                 ///< 显式 Profile 文件；非空时最高优先级
    std::vector<std::string> resource_paths;  ///< 额外 SerialArm resource roots
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief 加载并解析 framework-neutral Robot Profile
 */
tl::expected<RobotProfileCore, RobotProfileErrInfo> load_robot_profile_core(const std::string& profile_name, const RobotProfileLoadOptions& options = {});

/**
 * @brief 返回当前 SerialArm resource 搜索路径，用于诊断错误
 */
std::vector<std::string> robot_profile_search_paths(const RobotProfileLoadOptions& options = {});

} // namespace serial_arm
