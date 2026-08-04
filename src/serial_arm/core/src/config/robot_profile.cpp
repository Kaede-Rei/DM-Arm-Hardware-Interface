#include "serial_arm/config/robot_profile.hpp"

#include <yaml-cpp/yaml.h>

#include <cstdlib>
#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>

#if defined(__linux__)
#include <unistd.h>
#endif

namespace serial_arm {

namespace {

namespace fs = std::filesystem;

RobotProfileErrInfo err(RobotProfileErr code, const std::string& message) {
    return RobotProfileErrInfo{ code, message };
}

std::vector<std::string> split_paths(const char* value) {
    std::vector<std::string> paths;
    if(!value) return paths;
    std::stringstream stream(value);
    std::string item;
    while(std::getline(stream, item, ':')) {
        if(!item.empty()) paths.push_back(item);
    }
    return paths;
}

std::string current_executable_path() {
#if defined(__linux__)
    std::array<char, 4096> buffer{};
    const ssize_t size = readlink("/proc/self/exe", buffer.data(), buffer.size() - 1);
    if(size > 0) return std::string(buffer.data(), static_cast<std::size_t>(size));
#endif
    return {};
}

void append_if_exists(std::vector<std::string>& paths, const fs::path& path) {
    if(path.empty() || !fs::exists(path)) return;
    const std::string value = fs::absolute(path).lexically_normal().string();
    if(std::find(paths.begin(), paths.end(), value) == paths.end()) paths.push_back(value);
}

std::string read_text_prefix(const fs::path& path, std::size_t max_size = 4096) {
    std::ifstream stream(path);
    if(!stream) return {};
    std::string text(max_size, '\0');
    stream.read(text.data(), static_cast<std::streamsize>(text.size()));
    text.resize(static_cast<std::size_t>(stream.gcount()));
    return text;
}

bool package_xml_has_name(const fs::path& package_xml, const std::string& package) {
    const std::string text = read_text_prefix(package_xml);
    return text.find("<name>" + package + "</name>") != std::string::npos;
}

std::optional<fs::path> find_package_share(const std::vector<std::string>& roots, const std::string& package) {
    for(const auto& root_value : roots) {
        const fs::path root(root_value);
        const std::array<fs::path, 4> direct = {
            root / "share" / package,
            root / package,
            root / "install" / package / "share" / package,
            root / "src" / package,
        };
        for(const auto& candidate : direct) {
            if(fs::exists(candidate / "package.xml") && package_xml_has_name(candidate / "package.xml", package)) {
                return fs::absolute(candidate).lexically_normal();
            }
            if(fs::exists(candidate)) return fs::absolute(candidate).lexically_normal();
        }

        if(!fs::is_directory(root)) continue;
        std::error_code ec;
        for(fs::recursive_directory_iterator it(root, fs::directory_options::skip_permission_denied, ec), end; it != end && !ec; it.increment(ec)) {
            if(it.depth() > 5) {
                it.disable_recursion_pending();
                continue;
            }
            if(!it->is_regular_file(ec) || it->path().filename() != "package.xml") continue;
            if(package_xml_has_name(it->path(), package)) return fs::absolute(it->path().parent_path()).lexically_normal();
        }
    }
    return std::nullopt;
}

std::string resolve_plugin(const std::vector<std::string>& roots, const std::string& plugin) {
    if(plugin.find('/') != std::string::npos) return plugin;
    const std::string library_name = "lib" + plugin + ".so";
    for(const auto& root_value : roots) {
        const fs::path root(root_value);
        const std::array<fs::path, 5> direct = {
            root / library_name,
            root / "lib" / library_name,
            root / "install" / "lib" / library_name,
            root / "install" / plugin / "lib" / library_name,
            root / "build" / plugin / library_name,
        };
        for(const auto& candidate : direct) {
            if(fs::is_regular_file(candidate)) return fs::absolute(candidate).lexically_normal().string();
        }
        if(!fs::is_directory(root)) continue;
        std::error_code ec;
        for(fs::recursive_directory_iterator it(root, fs::directory_options::skip_permission_denied, ec), end; it != end && !ec; it.increment(ec)) {
            if(it.depth() > 5) {
                it.disable_recursion_pending();
                continue;
            }
            if(it->is_regular_file(ec) && it->path().filename() == library_name) return fs::absolute(it->path()).lexically_normal().string();
        }
    }
    return plugin;
}

std::vector<fs::path> profile_file_candidates(const std::vector<std::string>& roots) {
    std::vector<fs::path> candidates;
    for(const auto& root_value : roots) {
        const fs::path root(root_value);
        if(fs::is_regular_file(root)) {
            candidates.push_back(root);
            continue;
        }
        candidates.push_back(root / "robot_profiles.yaml");
        candidates.push_back(root / "config" / "robot_profiles.yaml");
        candidates.push_back(root / "share" / "serial_arm_robot_profiles" / "config" / "robot_profiles.yaml");
        candidates.push_back(root / "install" / "serial_arm_robot_profiles" / "share" / "serial_arm_robot_profiles" / "config" / "robot_profiles.yaml");
        candidates.push_back(root / "src" / "robot_supports" / "profiles" / "config" / "robot_profiles.yaml");
    }
    return candidates;
}

tl::expected<std::string, RobotProfileErrInfo> resolve_package_path(
    const YAML::Node& node,
    const std::string& package_key,
    const std::string& path_key,
    const std::vector<std::string>& roots) {
    if(!node[package_key] || !node[path_key]) {
        return tl::make_unexpected(err(RobotProfileErr::MISSING_FIELD, "profile 缺少 " + package_key + " 或 " + path_key));
    }
    const std::string package = node[package_key].as<std::string>();
    const std::string relative_path = node[path_key].as<std::string>();
    const auto share = find_package_share(roots, package);
    if(!share) {
        return tl::make_unexpected(err(RobotProfileErr::RESOURCE_NOT_FOUND, "无法解析 package '" + package + "'；搜索路径: " + [&roots]() {
            std::ostringstream stream;
            for(std::size_t i = 0; i < roots.size(); ++i) {
                if(i) stream << ", ";
                stream << roots[i];
            }
            return stream.str();
        }()));
    }
    return (*share / relative_path).lexically_normal().string();
}

} // namespace

std::vector<std::string> robot_profile_search_paths(const RobotProfileLoadOptions& options) {
    std::vector<std::string> paths;
    for(const auto& path : options.resource_paths) append_if_exists(paths, path);
    for(const auto& path : split_paths(std::getenv("SERIAL_ARM_RESOURCE_PATH"))) append_if_exists(paths, path);

    const fs::path cwd = fs::current_path();
    append_if_exists(paths, cwd);

    const std::string exe = current_executable_path();
    if(!exe.empty()) {
        fs::path dir = fs::absolute(exe).parent_path();
        for(int i = 0; i < 4 && !dir.empty(); ++i) {
            append_if_exists(paths, dir);
            append_if_exists(paths, dir.parent_path());
            dir = dir.parent_path();
        }
    }

#ifdef SERIAL_ARM_SOURCE_RESOURCE_ROOT
    append_if_exists(paths, SERIAL_ARM_SOURCE_RESOURCE_ROOT);
#endif
#ifdef SERIAL_ARM_INSTALL_RESOURCE_ROOT
    append_if_exists(paths, SERIAL_ARM_INSTALL_RESOURCE_ROOT);
#endif
    return paths;
}

tl::expected<RobotProfileCore, RobotProfileErrInfo> load_robot_profile_core(const std::string& profile_name, const RobotProfileLoadOptions& options) {
    if(profile_name.empty()) {
        return tl::make_unexpected(err(RobotProfileErr::PROFILE_NOT_FOUND, "robot_profile 不能为空"));
    }

    std::vector<std::string> roots = robot_profile_search_paths(options);
    std::vector<fs::path> candidates;
    const bool explicit_profile_file = !options.profile_file.empty();
    if(explicit_profile_file) {
        candidates.push_back(options.profile_file);
    }
    else {
        const auto discovered = profile_file_candidates(roots);
        candidates.insert(candidates.end(), discovered.begin(), discovered.end());
    }

    std::vector<std::string> checked_files;
    for(const auto& candidate : candidates) {
        if(!fs::is_regular_file(candidate)) continue;
        const fs::path profile_file = fs::absolute(candidate).lexically_normal();
        checked_files.push_back(profile_file.string());

        YAML::Node root;
        try {
            root = YAML::LoadFile(profile_file.string());
        }
        catch(const std::exception& error_value) {
            return tl::make_unexpected(err(RobotProfileErr::PROFILE_LOAD_FAILED, std::string("读取 Robot Profile 失败: ") + error_value.what() + "; profile_file=" + profile_file.string()));
        }
        if(!root["profiles"] || !root["profiles"][profile_name]) {
            if(explicit_profile_file) break;
            continue;
        }

        const YAML::Node profile = root["profiles"][profile_name];
        if(!profile["core"] || !profile["hardware"]) {
            return tl::make_unexpected(err(RobotProfileErr::MISSING_FIELD, "robot_profile '" + profile_name + "' 缺少 core 或 hardware"));
        }

        std::vector<std::string> profile_roots = roots;
        profile_roots.push_back(profile_file.parent_path().string());
        profile_roots.push_back(profile_file.parent_path().parent_path().string());
        profile_roots.push_back(profile_file.parent_path().parent_path().parent_path().string());

        auto core_config = resolve_package_path(profile["core"], "package", "config", profile_roots);
        if(!core_config) return tl::make_unexpected(core_config.error());
        const YAML::Node hardware = profile["hardware"];
        if(!hardware["plugin"]) {
            return tl::make_unexpected(err(RobotProfileErr::MISSING_FIELD, "robot_profile '" + profile_name + "' 缺少 hardware.plugin"));
        }
        auto hardware_config = resolve_package_path(hardware, "config_package", "config", profile_roots);
        if(!hardware_config) return tl::make_unexpected(hardware_config.error());

        const std::string hardware_plugin = resolve_plugin(profile_roots, hardware["plugin"].as<std::string>());
        return RobotProfileCore{
            profile_name,
            profile_file.string(),
            *core_config,
            hardware_plugin,
            *hardware_config,
        };
    }
    if(checked_files.empty()) {
        return tl::make_unexpected(err(RobotProfileErr::PROFILE_FILE_NOT_FOUND, "找不到 robot_profiles.yaml；profile='" + profile_name + "'"));
    }
    std::ostringstream message;
    message << "robot_profile '" << profile_name << "' 不存在；checked_profile_files=";
    for(std::size_t i = 0; i < checked_files.size(); ++i) {
        if(i) message << ", ";
        message << checked_files[i];
    }
    return tl::make_unexpected(err(RobotProfileErr::PROFILE_NOT_FOUND, message.str()));
}

} // namespace serial_arm
