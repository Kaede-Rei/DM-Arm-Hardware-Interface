#include "serial_arm/config/config.hpp"

#include "serial_arm/config/limit_resolver.hpp"
#include "serial_arm/hardware/hardware_capability.hpp"
#include "serial_arm/model/model_loader.hpp"

#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <filesystem>
#include <initializer_list>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

namespace serial_arm {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //

namespace {

/**
 * @brief 配置加载异常类
 */
class ConfigLoadException final : public std::runtime_error {
public:
    /**
     * @brief 构造函数
     * @param code 错误码
     * @param message 错误信息
     */
    ConfigLoadException(ConfigErr code, std::string message) : std::runtime_error(std::move(message)), code_(code) {}

    /**
     * @brief 获取错误码
     * @return 错误码
     */
    ConfigErr code() const noexcept { return code_; }

private:
    ConfigErr code_;   ///< 错误码
};

/**
 * @brief 创建配置加载错误
 * @param code 错误码
 * @param message 错误信息
 * @return 配置加载错误
 */
ConfigErrInfo make_err(ConfigErr code, std::string message) {
    return ConfigErrInfo{ code, std::move(message) };
}

/**
 * @brief 创建配置加载错误
 * @param code 错误码
 * @param message 错误信息
 * @return 配置加载错误
 */
tl::expected<void, ConfigErrInfo> fail(ConfigErr code, std::string message) {
    return tl::make_unexpected(make_err(code, std::move(message)));
}

/**
 * @brief 获取 YAML 节点位置的文本描述
 * @param mark YAML 节点标记
 * @return 文本描述
 */
std::string yaml_location(const YAML::Mark& mark) {
    if(mark.line < 0 || mark.column < 0) return {};
    return "line " + std::to_string(mark.line + 1) + ", column " +
        std::to_string(mark.column + 1) + ": ";
}

/**
 * @brief 获取父节点的子节点，并确保其为 map 类型
 * @param parent 父节点
 * @param key 子节点键
 * @param context 上下文信息
 * @return 子节点
 * @throws ConfigLoadException 如果子节点不存在或不是 map 类型
 */
YAML::Node require_map(const YAML::Node& parent, const char* key, const char* context) {
    const YAML::Node node = parent[key];
    if(!node) {
        throw ConfigLoadException(ConfigErr::MISSING_FIELD, std::string(context) + ": missing field '" + key + "'");
    }
    if(!node.IsMap()) {
        throw ConfigLoadException(ConfigErr::INVALID_VALUE, yaml_location(node.Mark()) + std::string(context) + "." + key + " must be a map");
    }
    return node;
}

/**
 * @brief 拒绝 map 中未声明的配置字段
 */
void reject_unknown_keys(const YAML::Node& node, const char* context, std::initializer_list<const char*> allowed_keys) {
    if(!node || !node.IsMap()) return;

    std::set<std::string> allowed;
    for(const char* key : allowed_keys) allowed.insert(key);

    for(const auto& item : node) {
        std::string key;
        try {
            key = item.first.as<std::string>();
        }
        catch(const YAML::BadConversion&) {
            throw ConfigLoadException(ConfigErr::INVALID_VALUE, yaml_location(item.first.Mark()) + std::string(context) + " contains a non-string key");
        }
        if(allowed.find(key) == allowed.end()) {
            throw ConfigLoadException(ConfigErr::INVALID_VALUE, yaml_location(item.first.Mark()) + std::string(context) + " contains unknown field '" + key + "'");
        }
    }
}

/**
 * @brief 拒绝 Joint 名称 map 中未声明的关节字段
 */
void reject_unknown_joint_keys(const YAML::Node& node, const std::vector<std::string>& joint_names, const char* context) {
    if(!node || !node.IsMap()) return;

    std::set<std::string> allowed(joint_names.begin(), joint_names.end());
    for(const auto& item : node) {
        std::string key;
        try {
            key = item.first.as<std::string>();
        }
        catch(const YAML::BadConversion&) {
            throw ConfigLoadException(ConfigErr::INVALID_VALUE, yaml_location(item.first.Mark()) + std::string(context) + " contains a non-string key");
        }
        if(allowed.find(key) == allowed.end()) {
            throw ConfigLoadException(ConfigErr::INVALID_VALUE, yaml_location(item.first.Mark()) + std::string(context) + " contains unknown joint '" + key + "'");
        }
    }
}

/**
 * @brief 获取父节点的子节点，并确保其为 sequence 类型
 * @param parent 父节点
 * @param key 子节点键
 * @param context 上下文信息
 * @return 子节点
 * @throws ConfigLoadException 如果子节点不存在或不是 sequence 类型
 */
YAML::Node require_sequence(const YAML::Node& parent, const char* key, const char* context) {
    const YAML::Node node = parent[key];
    if(!node) {
        throw ConfigLoadException(ConfigErr::MISSING_FIELD, std::string(context) + ": missing field '" + key + "'");
    }
    if(!node.IsSequence()) {
        throw ConfigLoadException(ConfigErr::INVALID_VALUE, yaml_location(node.Mark()) + std::string(context) + "." + key + " must be a sequence");
    }
    return node;
}

/**
 * @brief 从父节点获取子节点的值，并确保其存在且类型正确
 * @tparam T 值的类型
 * @param parent 父节点
 * @param key 子节点键
 * @param context 上下文信息
 * @return 子节点的值
 * @throws ConfigLoadException 如果子节点不存在或类型不正确
 */
template<typename T>
T require_as(const YAML::Node& parent, const char* key, const char* context) {
    const YAML::Node node = parent[key];
    if(!node) {
        throw ConfigLoadException(ConfigErr::MISSING_FIELD, std::string(context) + ": missing field '" + key + "'");
    }

    try {
        return node.as<T>();
    }
    catch(const YAML::BadConversion&) {
        throw ConfigLoadException(ConfigErr::INVALID_VALUE, yaml_location(node.Mark()) + std::string(context) + "." + key + " has an invalid type or value");
    }
}

/**
 * @brief 从控制器配置中加载阻抗增益
 * @param controller 控制器配置节点
 * @param mode_name 模式名称
 * @return 阻抗增益
 * @throws ConfigLoadException 如果增益配置无效
 */
JointImpedanceGains load_gains(const YAML::Node& controller, const char* mode_name) {
    const YAML::Node mode = require_map(controller, mode_name, "controller");
    JointImpedanceGains gains;
    gains.kp = require_as<JointVector>(mode, "kp", mode_name);
    gains.kd = require_as<JointVector>(mode, "kd", mode_name);
    return gains;
}

/**
 * @brief 解析模型前馈策略
 */
ModelFeedforwardMode load_model_feedforward_mode(const YAML::Node& runtime) {
    const std::string value = require_as<std::string>(runtime, "model_feedforward_mode", "runtime");

    if(value == "NONE") return ModelFeedforwardMode::NONE;
    if(value == "GRAVITY") return ModelFeedforwardMode::GRAVITY;
    if(value == "FULL_INVERSE_DYNAMICS") {
        return ModelFeedforwardMode::FULL_INVERSE_DYNAMICS;
    }

    throw ConfigLoadException(ConfigErr::INVALID_VALUE, "runtime.model_feedforward_mode must be NONE, GRAVITY or FULL_INVERSE_DYNAMICS");
}

/**
 * @brief 解析 ros2_control 跟踪阻抗模式
 */
JointImpedanceMode load_tracking_impedance_mode(const YAML::Node& runtime) {
    if(!runtime["tracking_impedance_mode"]) return JointImpedanceMode::RIGID_TRACKING;
    const std::string value = require_as<std::string>(runtime, "tracking_impedance_mode", "runtime");

    if(value == "RIGID_TRACKING") return JointImpedanceMode::RIGID_TRACKING;
    if(value == "COMPLIANT_TRACKING") return JointImpedanceMode::COMPLIANT_TRACKING;

    throw ConfigLoadException(ConfigErr::INVALID_VALUE, "runtime.tracking_impedance_mode must be RIGID_TRACKING or COMPLIANT_TRACKING");
}

/**
 * @brief 解析 FAULT 默认恢复模式
 */
FaultHoldMode load_fault_hold_mode(const YAML::Node& node, const char* context) {
    const std::string value = require_as<std::string>(node, "default_mode", context);
    if(value == "rigid_hold") return FaultHoldMode::RIGID_HOLD;
    if(value == "compliant_recovery") return FaultHoldMode::COMPLIANT_RECOVERY;
    throw ConfigLoadException(ConfigErr::INVALID_VALUE, std::string(context) + ".default_mode must be rigid_hold");
}

/**
 * @brief 检查关节向量是否包含有限值
 * @param values 关节向量
 * @return 如果所有值都是有限的，则返回 true，否则返回 false
 */
bool finite_vector(const JointVector& values) {
    for(const double value : values) {
        if(!std::isfinite(value)) return false;
    }
    return true;
}

/**
 * @brief 解析 YAML 中的 URDF 路径，兼容源码树与 colcon install 空间
 */
std::filesystem::path resolve_urdf_path(const std::filesystem::path& config_path, const std::filesystem::path& raw_path) {
    if(raw_path.is_absolute()) return raw_path.lexically_normal();

    const std::filesystem::path source_tree_path = (config_path.parent_path() / raw_path).lexically_normal();
    if(std::filesystem::exists(source_tree_path)) return source_tree_path;

    const std::string raw_text = raw_path.generic_string();
    const std::string marker = "serial_arm_description/robots/dm_arm/urdf/";
    const std::size_t marker_pos = raw_text.find(marker);
    if(marker_pos != std::string::npos) {
        const std::filesystem::path urdf_relative = raw_text.substr(marker_pos + marker.size());
        for(std::filesystem::path base = config_path.parent_path(); !base.empty(); base = base.parent_path()) {
            const std::filesystem::path install_share_path = (base / "serial_arm_description" / "share" / "serial_arm_description" / "robots" / "dm_arm" / "urdf" / urdf_relative).lexically_normal();
            if(std::filesystem::exists(install_share_path)) return install_share_path;
            if(base == base.parent_path()) break;
        }
    }

    return source_tree_path;
}

/**
 * @brief 获取所有阻抗增益的指针数组
 * @param cfg 控制器配置
 * @return 阻抗增益的指针数组
 */
std::array<const JointImpedanceGains*, 5> all_gains(const JointCtrllerCfg& cfg) {
    return {
        &cfg.rigid_hold_gains,
        &cfg.rigid_tracking_gains,
        &cfg.compliant_hold_gains,
        &cfg.compliant_drag_gains,
        &cfg.compliant_tracking_gains,
    };
}

/**
 * @brief 按 Joint 名称读取 double map
 */
JointVector load_named_joint_vector(const YAML::Node& node, const std::vector<std::string>& joint_names, const char* context) {
    if(!node || !node.IsMap()) throw ConfigLoadException(ConfigErr::MISSING_FIELD, std::string(context) + " must be a map");
    reject_unknown_joint_keys(node, joint_names, context);
    JointVector values;
    values.reserve(joint_names.size());
    for(const auto& joint_name : joint_names) values.push_back(require_as<double>(node, joint_name.c_str(), context));
    return values;
}

/**
 * @brief 从 YAML 读取 FAULT 恢复配置
 */
void load_fault_recovery_cfg(const YAML::Node& node, const std::vector<std::string>& joint_names, SafetyCfg& safety_cfg) {
    if(!node) return;
    if(!node.IsMap()) throw ConfigLoadException(ConfigErr::INVALID_VALUE, yaml_location(node.Mark()) + "fault_recovery must be a map");
    reject_unknown_keys(node, "fault_recovery", { "default_mode", "allow_compliant_recovery", "require_operator_request", "gravity_model_validated", "recovery_timeout_s", "compliant_recovery" });
    safety_cfg.fault_recovery.default_mode = load_fault_hold_mode(node, "fault_recovery");
    safety_cfg.fault_recovery.allow_compliant_recovery = require_as<bool>(node, "allow_compliant_recovery", "fault_recovery");
    safety_cfg.fault_recovery.require_operator_request = require_as<bool>(node, "require_operator_request", "fault_recovery");
    safety_cfg.fault_recovery.gravity_model_validated = require_as<bool>(node, "gravity_model_validated", "fault_recovery");
    safety_cfg.fault_recovery.recovery_timeout_s = require_as<double>(node, "recovery_timeout_s", "fault_recovery");
    const YAML::Node recovery = require_map(node, "compliant_recovery", "fault_recovery");
    reject_unknown_keys(recovery, "fault_recovery.compliant_recovery", { "kp", "kd", "max_vel", "effort_scale" });
    if(recovery["kp"].IsMap()) safety_cfg.fault_recovery.compliant_recovery.kp = load_named_joint_vector(recovery["kp"], joint_names, "fault_recovery.compliant_recovery.kp");
    else safety_cfg.fault_recovery.compliant_recovery.kp = require_as<JointVector>(recovery, "kp", "fault_recovery.compliant_recovery");
    if(recovery["kd"].IsMap()) safety_cfg.fault_recovery.compliant_recovery.kd = load_named_joint_vector(recovery["kd"], joint_names, "fault_recovery.compliant_recovery.kd");
    else safety_cfg.fault_recovery.compliant_recovery.kd = require_as<JointVector>(recovery, "kd", "fault_recovery.compliant_recovery");
    if(recovery["max_vel"].IsMap()) safety_cfg.fault_recovery.compliant_recovery.max_vel = load_named_joint_vector(recovery["max_vel"], joint_names, "fault_recovery.compliant_recovery.max_vel");
    else safety_cfg.fault_recovery.compliant_recovery.max_vel = require_as<JointVector>(recovery, "max_vel", "fault_recovery.compliant_recovery");
    safety_cfg.fault_recovery.compliant_recovery.effort_scale = require_as<double>(recovery, "effort_scale", "fault_recovery.compliant_recovery");
}

} // namespace

// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

/**
 * @brief 使用 yaml-cpp 加载完整机器人配置
 * @param path YAML 文件路径
 * @return 配置加载结果，成功时包含 RobotCfg，失败时包含 ConfigErrInfo
 */
tl::expected<RobotCfg, ConfigErrInfo> load_flat_robot_cfg(const std::string& path) {
    try {
        const YAML::Node root = YAML::LoadFile(path);
        if(!root || !root.IsMap()) {
            return tl::make_unexpected(make_err(ConfigErr::SYNTAX_ERROR, "configuration root must be a YAML map"));
        }

        RobotCfg cfg;

        const YAML::Node joints = require_map(root, "joints", "root");
        cfg.joint_names = require_as<std::vector<std::string>>(joints, "names", "joints");

        const YAML::Node runtime = require_map(root, "runtime", "root");
        cfg.runtime.ctrl_frequency_hz = require_as<double>(runtime, "ctrl_frequency_hz", "runtime");
        cfg.runtime.joint_acc_filter_alpha = runtime["joint_acc_filter_alpha"] ? require_as<double>(runtime, "joint_acc_filter_alpha", "runtime") : 0.2;
        cfg.runtime.write_enabled = require_as<bool>(runtime, "write_enabled", "runtime");
        cfg.runtime.model_feedforward_mode = load_model_feedforward_mode(runtime);
        cfg.runtime.tracking_impedance_mode = load_tracking_impedance_mode(runtime);

        cfg.shutdown.park_pos.assign(cfg.joint_names.size(), 0.0);
        const YAML::Node shutdown = root["shutdown"];
        if(shutdown) {
            if(!shutdown.IsMap()) {
                throw ConfigLoadException(ConfigErr::INVALID_VALUE, yaml_location(shutdown.Mark()) + "root.shutdown must be a map");
            }
            cfg.shutdown.park_before_disable = require_as<bool>(shutdown, "park_before_disable", "shutdown");
            cfg.shutdown.park_pos = require_as<JointVector>(shutdown, "park_pos", "shutdown");
            cfg.shutdown.speed_scale = require_as<double>(shutdown, "speed_scale", "shutdown");
            cfg.shutdown.position_tolerance = require_as<double>(shutdown, "position_tolerance", "shutdown");
            cfg.shutdown.velocity_tolerance = require_as<double>(shutdown, "velocity_tolerance", "shutdown");
            cfg.shutdown.settle_time_s = require_as<double>(shutdown, "settle_time_s", "shutdown");
            cfg.shutdown.relaxed_tolerance_ratio = shutdown["relaxed_tolerance_ratio"] ? require_as<double>(shutdown, "relaxed_tolerance_ratio", "shutdown") : 2.0;
            cfg.shutdown.timeout_s = require_as<double>(shutdown, "timeout_s", "shutdown");
        }

        const YAML::Node safety = require_map(root, "safety", "root");
        cfg.safety.cmd_timeout_s = require_as<double>(safety, "cmd_timeout_s", "safety");
        cfg.safety.state_timeout_s = require_as<double>(safety, "state_timeout_s", "safety");
        cfg.safety.max_dt_s = require_as<double>(safety, "max_dt_s", "safety");
        cfg.safety.numeric_tolerance = require_as<double>(safety, "numeric_tolerance", "safety");
        cfg.safety.state_vel_fault_ratio = safety["state_vel_fault_ratio"] ? require_as<double>(safety, "state_vel_fault_ratio", "safety") : 1.5;
        cfg.safety.require_all_actuators_online = require_as<bool>(safety, "require_all_actuators_online", "safety");
        cfg.safety.require_all_actuators_enabled = require_as<bool>(safety, "require_all_actuators_enabled", "safety");
        cfg.safety.reject_motor_error = safety["reject_motor_error"] ? require_as<bool>(safety, "reject_motor_error", "safety") : true;
        cfg.safety.require_continuous_cmd = safety["require_continuous_cmd"] ? require_as<bool>(safety, "require_continuous_cmd", "safety") : true;
        load_fault_recovery_cfg(safety["fault_recovery"] ? safety["fault_recovery"] : root["fault_recovery"], cfg.joint_names, cfg.safety);

        const YAML::Node limits = require_map(root, "limits", "root");
        cfg.safety.limits.min_pos = require_as<JointVector>(limits, "min_pos", "limits");
        cfg.safety.limits.max_pos = require_as<JointVector>(limits, "max_pos", "limits");
        cfg.safety.limits.max_vel = require_as<JointVector>(limits, "max_vel", "limits");
        cfg.safety.limits.max_acc = require_as<JointVector>(limits, "max_acc", "limits");
        cfg.safety.limits.max_effort = require_as<JointVector>(limits, "max_effort", "limits");
        cfg.safety.limits.max_kp = require_as<JointVector>(limits, "max_kp", "limits");
        cfg.safety.limits.max_kd = require_as<JointVector>(limits, "max_kd", "limits");
        cfg.safety.limits.pos_margin = require_as<JointVector>(limits, "pos_margin", "limits");

        const YAML::Node mapping = require_map(root, "mapping", "root");
        cfg.mapper.pos_ratio = require_as<ActuatorVector>(mapping, "pos_ratio", "mapping");
        cfg.mapper.tor_ratio = require_as<ActuatorVector>(mapping, "tor_ratio", "mapping");
        cfg.mapper.direction = require_as<std::vector<int>>(mapping, "direction", "mapping");
        cfg.mapper.joint_zero_offset = require_as<JointVector>(mapping, "joint_zero_offset", "mapping");
        cfg.mapper.actuator_zero_offset = require_as<ActuatorVector>(mapping, "actuator_zero_offset", "mapping");

        const YAML::Node controller = require_map(root, "controller", "root");
        cfg.ctrller.allow_full_cmd = require_as<bool>(controller, "allow_full_cmd", "controller");
        cfg.ctrller.rigid_hold_gains = load_gains(controller, "rigid_hold");
        cfg.ctrller.rigid_tracking_gains = load_gains(controller, "rigid_tracking");
        cfg.ctrller.compliant_hold_gains = load_gains(controller, "compliant_hold");
        cfg.ctrller.compliant_drag_gains = load_gains(controller, "compliant_drag");
        cfg.ctrller.compliant_tracking_gains = load_gains(controller, "compliant_tracking");


        const YAML::Node dynamics = require_map(root, "dynamics", "root");
        const std::filesystem::path config_path = std::filesystem::absolute(path).lexically_normal();
        std::filesystem::path urdf_path = resolve_urdf_path(config_path, require_as<std::string>(dynamics, "urdf_path", "dynamics"));

        cfg.dynamics.urdf_path = urdf_path.string();
        cfg.dynamics.joint_names = cfg.joint_names;
        cfg.dynamics.base_frame = require_as<std::string>(dynamics, "base_frame", "dynamics");
        cfg.dynamics.tool_frame = require_as<std::string>(dynamics, "tool_frame", "dynamics");

        const JointVector gravity = require_as<JointVector>(dynamics, "gravity", "dynamics");
        if(gravity.size() != 3) {
            throw ConfigLoadException(ConfigErr::INVALID_SIZE, "dynamics.gravity must have length 3");
        }
        cfg.dynamics.gravity = { gravity[0], gravity[1], gravity[2] };
        cfg.dynamics.gravity_scale = require_as<JointVector>(dynamics, "gravity_scale", "dynamics");

        const YAML::Node damiao = require_map(root, "damiao", "root");
        cfg.damiao.serial_port = require_as<std::string>(damiao, "serial_port", "damiao");
        cfg.damiao.baudrate = require_as<int>(damiao, "baudrate", "damiao");
        cfg.damiao.refresh_state_in_read = require_as<bool>(damiao, "refresh_state_in_read", "damiao");
        cfg.damiao.feedback_timeout_s = require_as<double>(damiao, "feedback_timeout_s", "damiao");
        cfg.damiao.activation_retries = damiao["activation_retries"] ? require_as<std::size_t>(damiao, "activation_retries", "damiao") : 3;
        cfg.damiao.startup_read_cycles = require_as<std::size_t>(damiao, "startup_read_cycles", "damiao");
        cfg.damiao.stop_kp = require_as<double>(damiao, "stop_kp", "damiao");
        cfg.damiao.stop_kd = require_as<double>(damiao, "stop_kd", "damiao");
        cfg.damiao.stop_cycles = require_as<std::size_t>(damiao, "stop_cycles", "damiao");

        const YAML::Node actuators = require_sequence(damiao, "actuators", "damiao");
        cfg.damiao.actuators.reserve(actuators.size());
        for(std::size_t i = 0; i < actuators.size(); ++i) {
            const YAML::Node item = actuators[i];
            if(!item.IsMap()) {
                throw ConfigLoadException(ConfigErr::INVALID_VALUE, yaml_location(item.Mark()) + "damiao.actuators[" + std::to_string(i) + "] must be a map");
            }

            DamiaoActuatorCfg actuator;
            const std::string context = "damiao.actuators[" + std::to_string(i) + "]";
            actuator.name = require_as<std::string>(item, "name", context.c_str());
            actuator.joint_name = require_as<std::string>(item, "joint_name", context.c_str());
            actuator.motor_id = require_as<std::uint32_t>(item, "motor_id", context.c_str());
            actuator.master_id = require_as<std::uint32_t>(item, "master_id", context.c_str());
            actuator.motor_type = require_as<std::string>(item, "motor_type", context.c_str());
            cfg.damiao.actuators.push_back(std::move(actuator));
        }

        cfg.ctrller.joints_count = cfg.joint_names.size();
        cfg.mapper.joints_count = cfg.joint_names.size();
        cfg.safety.joints_count = cfg.joint_names.size();

        auto validated = validate_robot_cfg(cfg);
        if(!validated) return tl::make_unexpected(validated.error());
        return cfg;
    }
    catch(const ConfigLoadException& error) {
        return tl::make_unexpected(make_err(error.code(), error.what()));
    }
    catch(const YAML::BadFile&) {
        return tl::make_unexpected(make_err(ConfigErr::FILE_OPEN_FAILED, "failed to open configuration file: " + path));
    }
    catch(const YAML::ParserException& error) {
        return tl::make_unexpected(make_err(ConfigErr::SYNTAX_ERROR, yaml_location(error.mark) + error.msg));
    }
    catch(const YAML::Exception& error) {
        return tl::make_unexpected(make_err(ConfigErr::INVALID_VALUE, yaml_location(error.mark) + error.msg));
    }
    catch(const std::exception& error) {
        return tl::make_unexpected(make_err(ConfigErr::INVALID_VALUE, error.what()));
    }
}

tl::expected<RobotCfg, ConfigErrInfo> load_sectioned_robot_cfg(const std::string& path);

/**
 * @brief 使用 yaml-cpp 加载完整机器人配置
 * @param path YAML 文件路径
 * @return 配置加载结果，成功时包含 RobotCfg，失败时包含 ConfigErrInfo
 */
tl::expected<RobotCfg, ConfigErrInfo> load_robot_cfg(const std::string& path) {
    try {
        const YAML::Node root = YAML::LoadFile(path);
        if(!root || !root.IsMap()) {
            return tl::make_unexpected(make_err(ConfigErr::SYNTAX_ERROR, "configuration root must be a YAML map"));
        }
        if(root["model"] && root["hardware"] && root["calibration"] && root["control"] && root["safety_policy"] && root["shutdown"]) return load_sectioned_robot_cfg(path);
        return load_flat_robot_cfg(path);
    }
    catch(const YAML::BadFile&) {
        return tl::make_unexpected(make_err(ConfigErr::FILE_OPEN_FAILED, "failed to open configuration file: " + path));
    }
    catch(const YAML::Exception& error) {
        return tl::make_unexpected(make_err(ConfigErr::INVALID_VALUE, yaml_location(error.mark) + error.msg));
    }
}

/**
 * @brief 加载分区式单文件配置
 */
tl::expected<RobotCfg, ConfigErrInfo> load_sectioned_robot_cfg(const std::string& path) {
    try {
        const std::filesystem::path config_path = std::filesystem::absolute(path).lexically_normal();
        const YAML::Node root = YAML::LoadFile(path);
        reject_unknown_keys(root, "root", { "model", "hardware", "calibration", "control", "safety_policy", "shutdown" });

        const YAML::Node model_node = require_map(root, "model", "root");
        const YAML::Node hardware_node = require_map(root, "hardware", "root");
        const YAML::Node calibration_node = require_map(root, "calibration", "root");
        const YAML::Node control_node = require_map(root, "control", "root");
        const YAML::Node safety_node = require_map(root, "safety_policy", "root");
        const YAML::Node shutdown_node = require_map(root, "shutdown", "root");
        reject_unknown_keys(model_node, "model", { "urdf_path", "joint_names", "base_frame", "tool_frame", "gravity", "gravity_scale" });
        reject_unknown_keys(hardware_node, "hardware", { "damiao" });
        reject_unknown_keys(calibration_node, "calibration", { "joints" });
        reject_unknown_keys(control_node, "control", { "runtime", "controller" });
        reject_unknown_keys(safety_node, "safety_policy", { "position_margin", "cmd_vel_scale", "state_vel_scale", "max_acc", "max_effort_override", "max_kp_override", "max_kd_override", "max_dt_s", "state_timeout_s", "cmd_timeout_s", "require_all_actuators_online", "require_all_actuators_enabled", "reject_motor_error", "require_continuous_cmd", "fault_recovery" });
        reject_unknown_keys(shutdown_node, "shutdown", { "park_before_disable", "park_pos", "speed_scale", "position_tolerance", "velocity_tolerance", "settle_time_s", "relaxed_tolerance_ratio", "timeout_s" });

        RobotCfg cfg;
        cfg.joint_names = require_as<std::vector<std::string>>(model_node, "joint_names", "model");
        const std::filesystem::path urdf_path = resolve_urdf_path(config_path, require_as<std::string>(model_node, "urdf_path", "model"));
        cfg.dynamics.urdf_path = urdf_path.string();
        cfg.dynamics.joint_names = cfg.joint_names;
        cfg.dynamics.base_frame = require_as<std::string>(model_node, "base_frame", "model");
        cfg.dynamics.tool_frame = require_as<std::string>(model_node, "tool_frame", "model");
        const JointVector gravity = require_as<JointVector>(model_node, "gravity", "model");
        if(gravity.size() != 3) throw ConfigLoadException(ConfigErr::INVALID_SIZE, "model.gravity must have length 3");
        cfg.dynamics.gravity = { gravity[0], gravity[1], gravity[2] };
        cfg.dynamics.gravity_scale = require_as<JointVector>(model_node, "gravity_scale", "model");

        const YAML::Node runtime = require_map(control_node, "runtime", "control");
        reject_unknown_keys(runtime, "control.runtime", { "ctrl_frequency_hz", "joint_acc_filter_alpha", "write_enabled", "model_feedforward_mode", "tracking_impedance_mode" });
        cfg.runtime.ctrl_frequency_hz = require_as<double>(runtime, "ctrl_frequency_hz", "runtime");
        cfg.runtime.joint_acc_filter_alpha = runtime["joint_acc_filter_alpha"] ? require_as<double>(runtime, "joint_acc_filter_alpha", "runtime") : 0.2;
        cfg.runtime.write_enabled = require_as<bool>(runtime, "write_enabled", "runtime");
        cfg.runtime.model_feedforward_mode = load_model_feedforward_mode(runtime);
        cfg.runtime.tracking_impedance_mode = load_tracking_impedance_mode(runtime);
        const YAML::Node controller = require_map(control_node, "controller", "control");
        reject_unknown_keys(controller, "control.controller", { "allow_full_cmd", "rigid_hold", "rigid_tracking", "compliant_hold", "compliant_drag", "compliant_tracking" });
        reject_unknown_keys(require_map(controller, "rigid_hold", "controller"), "controller.rigid_hold", { "kp", "kd" });
        reject_unknown_keys(require_map(controller, "rigid_tracking", "controller"), "controller.rigid_tracking", { "kp", "kd" });
        reject_unknown_keys(require_map(controller, "compliant_hold", "controller"), "controller.compliant_hold", { "kp", "kd" });
        reject_unknown_keys(require_map(controller, "compliant_drag", "controller"), "controller.compliant_drag", { "kp", "kd" });
        reject_unknown_keys(require_map(controller, "compliant_tracking", "controller"), "controller.compliant_tracking", { "kp", "kd" });
        cfg.ctrller.allow_full_cmd = require_as<bool>(controller, "allow_full_cmd", "controller");
        cfg.ctrller.rigid_hold_gains.kp = load_named_joint_vector(require_map(controller, "rigid_hold", "controller")["kp"], cfg.joint_names, "controller.rigid_hold.kp");
        cfg.ctrller.rigid_hold_gains.kd = load_named_joint_vector(require_map(controller, "rigid_hold", "controller")["kd"], cfg.joint_names, "controller.rigid_hold.kd");
        cfg.ctrller.rigid_tracking_gains.kp = load_named_joint_vector(require_map(controller, "rigid_tracking", "controller")["kp"], cfg.joint_names, "controller.rigid_tracking.kp");
        cfg.ctrller.rigid_tracking_gains.kd = load_named_joint_vector(require_map(controller, "rigid_tracking", "controller")["kd"], cfg.joint_names, "controller.rigid_tracking.kd");
        cfg.ctrller.compliant_hold_gains.kp = load_named_joint_vector(require_map(controller, "compliant_hold", "controller")["kp"], cfg.joint_names, "controller.compliant_hold.kp");
        cfg.ctrller.compliant_hold_gains.kd = load_named_joint_vector(require_map(controller, "compliant_hold", "controller")["kd"], cfg.joint_names, "controller.compliant_hold.kd");
        cfg.ctrller.compliant_drag_gains.kp = load_named_joint_vector(require_map(controller, "compliant_drag", "controller")["kp"], cfg.joint_names, "controller.compliant_drag.kp");
        cfg.ctrller.compliant_drag_gains.kd = load_named_joint_vector(require_map(controller, "compliant_drag", "controller")["kd"], cfg.joint_names, "controller.compliant_drag.kd");
        cfg.ctrller.compliant_tracking_gains.kp = load_named_joint_vector(require_map(controller, "compliant_tracking", "controller")["kp"], cfg.joint_names, "controller.compliant_tracking.kp");
        cfg.ctrller.compliant_tracking_gains.kd = load_named_joint_vector(require_map(controller, "compliant_tracking", "controller")["kd"], cfg.joint_names, "controller.compliant_tracking.kd");

        const YAML::Node damiao = require_map(hardware_node, "damiao", "hardware");
        reject_unknown_keys(damiao, "hardware.damiao", { "serial_port", "baudrate", "refresh_state_in_read", "feedback_timeout_s", "activation_retries", "startup_read_cycles", "stop_kp", "stop_kd", "stop_cycles", "actuators" });
        cfg.damiao.serial_port = require_as<std::string>(damiao, "serial_port", "damiao");
        cfg.damiao.baudrate = require_as<int>(damiao, "baudrate", "damiao");
        cfg.damiao.refresh_state_in_read = require_as<bool>(damiao, "refresh_state_in_read", "damiao");
        cfg.damiao.feedback_timeout_s = require_as<double>(damiao, "feedback_timeout_s", "damiao");
        cfg.damiao.activation_retries = require_as<std::size_t>(damiao, "activation_retries", "damiao");
        cfg.damiao.startup_read_cycles = require_as<std::size_t>(damiao, "startup_read_cycles", "damiao");
        cfg.damiao.stop_kp = require_as<double>(damiao, "stop_kp", "damiao");
        cfg.damiao.stop_kd = require_as<double>(damiao, "stop_kd", "damiao");
        cfg.damiao.stop_cycles = require_as<std::size_t>(damiao, "stop_cycles", "damiao");
        const YAML::Node actuators = require_map(damiao, "actuators", "damiao");
        reject_unknown_joint_keys(actuators, cfg.joint_names, "hardware.damiao.actuators");
        for(const auto& joint_name : cfg.joint_names) {
            const YAML::Node item = require_map(actuators, joint_name.c_str(), "damiao.actuators");
            reject_unknown_keys(item, joint_name.c_str(), { "name", "motor_id", "master_id", "motor_type" });
            DamiaoActuatorCfg actuator;
            actuator.name = require_as<std::string>(item, "name", joint_name.c_str());
            actuator.joint_name = joint_name;
            actuator.motor_id = require_as<std::uint32_t>(item, "motor_id", joint_name.c_str());
            actuator.master_id = require_as<std::uint32_t>(item, "master_id", joint_name.c_str());
            actuator.motor_type = require_as<std::string>(item, "motor_type", joint_name.c_str());
            cfg.damiao.actuators.push_back(std::move(actuator));
        }

        const YAML::Node joints = require_map(calibration_node, "joints", "calibration");
        reject_unknown_joint_keys(joints, cfg.joint_names, "calibration.joints");
        for(const auto& joint_name : cfg.joint_names) {
            const YAML::Node item = require_map(joints, joint_name.c_str(), "calibration.joints");
            reject_unknown_keys(item, joint_name.c_str(), { "direction", "pos_ratio", "tor_ratio", "joint_zero_offset", "actuator_zero_offset" });
            cfg.mapper.pos_ratio.push_back(require_as<double>(item, "pos_ratio", joint_name.c_str()));
            cfg.mapper.tor_ratio.push_back(require_as<double>(item, "tor_ratio", joint_name.c_str()));
            cfg.mapper.direction.push_back(static_cast<int>(require_as<double>(item, "direction", joint_name.c_str())));
            cfg.mapper.joint_zero_offset.push_back(require_as<double>(item, "joint_zero_offset", joint_name.c_str()));
            cfg.mapper.actuator_zero_offset.push_back(require_as<double>(item, "actuator_zero_offset", joint_name.c_str()));
        }

        SafetyPolicyCfg policy;
        policy.position_margin = require_as<double>(safety_node, "position_margin", "safety");
        policy.cmd_vel_scale = require_as<double>(safety_node, "cmd_vel_scale", "safety");
        policy.state_vel_scale = require_as<double>(safety_node, "state_vel_scale", "safety");
        policy.max_acc = load_named_joint_vector(require_map(safety_node, "max_acc", "safety"), cfg.joint_names, "safety.max_acc");
        if(safety_node["max_effort_override"]) policy.max_effort_override = load_named_joint_vector(safety_node["max_effort_override"], cfg.joint_names, "safety.max_effort_override");
        if(safety_node["max_kp_override"]) policy.max_kp_override = load_named_joint_vector(safety_node["max_kp_override"], cfg.joint_names, "safety.max_kp_override");
        if(safety_node["max_kd_override"]) policy.max_kd_override = load_named_joint_vector(safety_node["max_kd_override"], cfg.joint_names, "safety.max_kd_override");
        policy.max_dt_s = require_as<double>(safety_node, "max_dt_s", "safety");
        policy.state_timeout_s = require_as<double>(safety_node, "state_timeout_s", "safety");
        policy.cmd_timeout_s = require_as<double>(safety_node, "cmd_timeout_s", "safety");
        policy.require_all_actuators_online = require_as<bool>(safety_node, "require_all_actuators_online", "safety");
        policy.require_all_actuators_enabled = require_as<bool>(safety_node, "require_all_actuators_enabled", "safety");
        policy.reject_motor_error = require_as<bool>(safety_node, "reject_motor_error", "safety");
        policy.require_continuous_cmd = safety_node["require_continuous_cmd"] ? require_as<bool>(safety_node, "require_continuous_cmd", "safety") : true;

        cfg.shutdown.park_before_disable = require_as<bool>(shutdown_node, "park_before_disable", "shutdown");
        cfg.shutdown.park_pos = load_named_joint_vector(require_map(shutdown_node, "park_pos", "shutdown"), cfg.joint_names, "shutdown.park_pos");
        cfg.shutdown.speed_scale = require_as<double>(shutdown_node, "speed_scale", "shutdown");
        cfg.shutdown.position_tolerance = require_as<double>(shutdown_node, "position_tolerance", "shutdown");
        cfg.shutdown.velocity_tolerance = require_as<double>(shutdown_node, "velocity_tolerance", "shutdown");
        cfg.shutdown.settle_time_s = require_as<double>(shutdown_node, "settle_time_s", "shutdown");
        cfg.shutdown.relaxed_tolerance_ratio = require_as<double>(shutdown_node, "relaxed_tolerance_ratio", "shutdown");
        cfg.shutdown.timeout_s = require_as<double>(shutdown_node, "timeout_s", "shutdown");

        cfg.ctrller.joints_count = cfg.joint_names.size();
        cfg.mapper.joints_count = cfg.joint_names.size();
        const auto model_info = ModelLoader{}.load(cfg.dynamics.urdf_path, cfg.joint_names);
        if(!model_info) return tl::make_unexpected(make_err(ConfigErr::INVALID_VALUE, "ModelLoader failed"));
        const auto capabilities = load_damiao_capabilities(cfg.damiao);
        if(!capabilities) return tl::make_unexpected(make_err(ConfigErr::INVALID_VALUE, "Damiao hardware capability failed"));
        const auto resolved = LimitResolver{}.resolve(*model_info, cfg.damiao, cfg.mapper, *capabilities, policy);
        if(!resolved) return tl::make_unexpected(make_err(ConfigErr::INVALID_VALUE, "LimitResolver failed"));
        cfg.safety = to_safety_cfg(*resolved);
        load_fault_recovery_cfg(safety_node["fault_recovery"], cfg.joint_names, cfg.safety);

        auto validated = validate_robot_cfg(cfg);
        if(!validated) return tl::make_unexpected(validated.error());
        return cfg;
    }
    catch(const ConfigLoadException& error) {
        return tl::make_unexpected(make_err(error.code(), error.what()));
    }
    catch(const YAML::BadFile&) {
        return tl::make_unexpected(make_err(ConfigErr::FILE_OPEN_FAILED, "failed to open configuration file: " + path));
    }
    catch(const YAML::Exception& error) {
        return tl::make_unexpected(make_err(ConfigErr::INVALID_VALUE, yaml_location(error.mark) + error.msg));
    }
}

/**
 * @brief 验证 Robot 控制闭环所需的通用配置
 * @param cfg 机器人配置
 * @return 如果配置有效，则返回空值，否则返回 ConfigErrInfo
 */
tl::expected<void, ConfigErrInfo> validate_robot_core_cfg(const RobotCfg& cfg) {
    const std::size_t n = cfg.joint_names.size();
    if(n == 0) {
        return fail(ConfigErr::INVALID_SIZE, "DM-Arm main chain requires at least one joints");
    }

    const auto size_is_n = [n](const auto& values) {
        return values.size() == n;
        };
    const auto& limits = cfg.safety.limits;
    if(!size_is_n(cfg.shutdown.park_pos)) {
        return fail(ConfigErr::INVALID_SIZE, "shutdown.park_pos must have length " + std::to_string(n));
    }
    if(!finite_vector(cfg.shutdown.park_pos) || !std::isfinite(cfg.shutdown.speed_scale) ||
        !std::isfinite(cfg.shutdown.position_tolerance) || !std::isfinite(cfg.shutdown.velocity_tolerance) ||
        !std::isfinite(cfg.shutdown.settle_time_s) || !std::isfinite(cfg.shutdown.relaxed_tolerance_ratio) || !std::isfinite(cfg.shutdown.timeout_s) ||
        cfg.shutdown.speed_scale <= 0.0 || cfg.shutdown.speed_scale > 1.0 ||
        cfg.shutdown.position_tolerance <= 0.0 || cfg.shutdown.velocity_tolerance <= 0.0 ||
        cfg.shutdown.settle_time_s < 0.0 || cfg.shutdown.relaxed_tolerance_ratio < 1.0 || cfg.shutdown.timeout_s <= 0.0) {
        return fail(ConfigErr::INVALID_VALUE, "invalid shutdown configuration");
    }
    if(!size_is_n(limits.min_pos) || !size_is_n(limits.max_pos) ||
        !size_is_n(limits.max_vel) || !size_is_n(limits.max_acc) ||
        !size_is_n(limits.max_effort) || !size_is_n(limits.max_kp) ||
        !size_is_n(limits.max_kd) || !size_is_n(limits.pos_margin)) {
        return fail(ConfigErr::INVALID_SIZE, "all Safety joint arrays must have length " + std::to_string(n));
    }

    if(cfg.ctrller.joints_count != n ||
        cfg.mapper.joints_count != n ||
        cfg.safety.joints_count != n) {
        return fail(ConfigErr::INVALID_SIZE, "controller, mapper and safety joints_count must match joint_names");
    }

    if(!std::isfinite(cfg.runtime.ctrl_frequency_hz) || cfg.runtime.ctrl_frequency_hz <= 0.0) {
        return fail(ConfigErr::INVALID_VALUE, "runtime.ctrl_frequency_hz must be finite and positive");
    }
    if(!std::isfinite(cfg.runtime.joint_acc_filter_alpha) || cfg.runtime.joint_acc_filter_alpha < 0.0 || cfg.runtime.joint_acc_filter_alpha > 1.0) {
        return fail(ConfigErr::INVALID_VALUE, "runtime.joint_acc_filter_alpha must be in [0, 1]");
    }
    if(cfg.runtime.tracking_impedance_mode != JointImpedanceMode::RIGID_TRACKING &&
        cfg.runtime.tracking_impedance_mode != JointImpedanceMode::COMPLIANT_TRACKING) {
        return fail(ConfigErr::INVALID_VALUE, "runtime.tracking_impedance_mode must be a tracking mode");
    }
    const double nominal_dt = 1.0 / cfg.runtime.ctrl_frequency_hz;
    if(cfg.safety.max_dt_s < nominal_dt) {
        return fail(ConfigErr::INVALID_VALUE, "safety.max_dt_s must not be smaller than the nominal control period");
    }

    std::set<std::string> joint_names;
    for(const auto& joint_name : cfg.joint_names) {
        if(joint_name.empty() || !joint_names.insert(joint_name).second) {
            return fail(ConfigErr::DUPLICATE_NAME, "joint names must be non-empty and unique");
        }
    }

    const std::array<const JointVector*, 8> limit_vectors = {
        &limits.min_pos,
        &limits.max_pos,
        &limits.max_vel,
        &limits.max_acc,
        &limits.max_effort,
        &limits.max_kp,
        &limits.max_kd,
        &limits.pos_margin,
    };
    for(const auto* values : limit_vectors) {
        if(!finite_vector(*values)) {
            return fail(ConfigErr::INVALID_VALUE, "Safety joint limits contain NaN or Inf");
        }
    }

    JointActuatorMapper mapper;
    if(!mapper.configure(cfg.mapper)) {
        return fail(ConfigErr::INVALID_VALUE, "invalid Joint/Actuator mapping");
    }

    JointCtrller ctrller;
    if(!ctrller.configure(cfg.ctrller)) {
        return fail(ConfigErr::INVALID_VALUE, "invalid Joint controller gains");
    }

    Safety safety;
    if(!safety.configure(cfg.safety)) {
        return fail(ConfigErr::INVALID_VALUE, "invalid Safety configuration");
    }

    for(const auto* gains : all_gains(cfg.ctrller)) {
        if(gains->kp.size() != n || gains->kd.size() != n) {
            return fail(ConfigErr::INVALID_SIZE, "every impedance gain vector must have length " + std::to_string(n));
        }
    }

    for(std::size_t i = 0; i < n; ++i) {
        for(const auto* gains : all_gains(cfg.ctrller)) {
            if(gains->kp[i] > limits.max_kp[i] || gains->kd[i] > limits.max_kd[i]) {
                return fail(ConfigErr::INVALID_VALUE, "controller gain exceeds configured Safety limit at index " + std::to_string(i));
            }
        }
        const double park_min = limits.min_pos[i] + limits.pos_margin[i];
        const double park_max = limits.max_pos[i] - limits.pos_margin[i];
        if(cfg.shutdown.park_pos[i] < park_min || cfg.shutdown.park_pos[i] > park_max) {
            return fail(ConfigErr::INVALID_VALUE, "shutdown.park_pos is outside the command range at index " + std::to_string(i));
        }
    }

    if(cfg.dynamics.urdf_path.empty() || cfg.dynamics.base_frame.empty() || cfg.dynamics.tool_frame.empty()) {
        return fail(ConfigErr::MISSING_FIELD, "dynamics urdf_path, base_frame and tool_frame must not be empty");
    }
    if(cfg.dynamics.joint_names != cfg.joint_names) {
        return fail(ConfigErr::INVALID_VALUE, "dynamics joint_names must match joint_names");
    }
    if(cfg.dynamics.gravity_scale.size() != n) {
        return fail(ConfigErr::INVALID_SIZE, "dynamics.gravity_scale must have length " + std::to_string(n));
    }
    for(const double value : cfg.dynamics.gravity) {
        if(!std::isfinite(value)) {
            return fail(ConfigErr::INVALID_VALUE, "dynamics.gravity contains NaN or Inf");
        }
    }
    for(const double value : cfg.dynamics.gravity_scale) {
        if(!std::isfinite(value) || value < 0.0 || value > 1.0) {
            return fail(ConfigErr::INVALID_VALUE, "dynamics.gravity_scale values must be in [0, 1]");
        }
    }

    return {};
}

/**
 * @brief 验证完整机器人配置，包括当前 Damiao 后端字段
 * @param cfg 机器人配置
 * @return 如果配置有效，则返回空值，否则返回 ConfigErrInfo
 */
tl::expected<void, ConfigErrInfo> validate_robot_cfg(const RobotCfg& cfg) {
    const auto core_valid = validate_robot_core_cfg(cfg);
    if(!core_valid) {
        return tl::make_unexpected(core_valid.error());
    }

    const std::size_t n = cfg.joint_names.size();
    if(cfg.damiao.actuators.size() != n) {
        return fail(ConfigErr::INVALID_SIZE, "Damiao actuator count must match configured actuator count");
    }

    if(cfg.damiao.serial_port.empty() || cfg.damiao.baudrate <= 0 ||
        cfg.damiao.activation_retries == 0 || cfg.damiao.startup_read_cycles == 0 || cfg.damiao.stop_cycles == 0 ||
        !std::isfinite(cfg.damiao.stop_kp) ||
        !std::isfinite(cfg.damiao.stop_kd) ||
        cfg.damiao.stop_kp < 0.0 || cfg.damiao.stop_kd < 0.0) {
        return fail(ConfigErr::INVALID_VALUE, "invalid Damiao bus or stop configuration");
    }

    std::set<std::string> actuator_names;
    std::set<std::uint32_t> motor_ids;
    for(std::size_t i = 0; i < n; ++i) {
        const auto& actuator = cfg.damiao.actuators[i];
        if(actuator.name.empty() ||
            !actuator_names.insert(actuator.name).second) {
            return fail(ConfigErr::DUPLICATE_NAME, "actuator names must be non-empty and unique");
        }
        if(actuator.joint_name != cfg.joint_names[i]) {
            return fail(ConfigErr::INVALID_VALUE, "actuator order/joint_name must match joint_names at index " + std::to_string(i));
        }
        if(actuator.motor_id == 0 || !motor_ids.insert(actuator.motor_id).second) {
            return fail(ConfigErr::DUPLICATE_MOTOR_ID, "motor IDs must be non-zero and unique");
        }
        if(actuator.motor_type.empty()) {
            return fail(ConfigErr::MISSING_FIELD, "motor_type must not be empty at index " + std::to_string(i));
        }
    }

    return {};
}

/**
 * @brief 只读比较两个配置解析后的最终配置差异
 */
tl::expected<std::vector<std::string>, ConfigErrInfo> compare_robot_cfg(const std::string& lhs_path, const std::string& rhs_path) {
    const auto lhs = load_robot_cfg(lhs_path);
    if(!lhs) return tl::make_unexpected(lhs.error());
    const auto rhs = load_robot_cfg(rhs_path);
    if(!rhs) return tl::make_unexpected(rhs.error());

    std::vector<std::string> diffs;
    const auto add = [&diffs](const std::string& name) {
        diffs.push_back(name);
        };
    if(lhs->joint_names != rhs->joint_names) add("joint_names");
    if(lhs->mapper.pos_ratio != rhs->mapper.pos_ratio || lhs->mapper.tor_ratio != rhs->mapper.tor_ratio ||
        lhs->mapper.direction != rhs->mapper.direction || lhs->mapper.joint_zero_offset != rhs->mapper.joint_zero_offset ||
        lhs->mapper.actuator_zero_offset != rhs->mapper.actuator_zero_offset) add("mapping");
    if(lhs->runtime.ctrl_frequency_hz != rhs->runtime.ctrl_frequency_hz) add("runtime.ctrl_frequency_hz");
    if(lhs->runtime.tracking_impedance_mode != rhs->runtime.tracking_impedance_mode) add("runtime.tracking_impedance_mode");
    if(lhs->ctrller.rigid_hold_gains.kp != rhs->ctrller.rigid_hold_gains.kp || lhs->ctrller.rigid_hold_gains.kd != rhs->ctrller.rigid_hold_gains.kd ||
        lhs->ctrller.rigid_tracking_gains.kp != rhs->ctrller.rigid_tracking_gains.kp || lhs->ctrller.rigid_tracking_gains.kd != rhs->ctrller.rigid_tracking_gains.kd ||
        lhs->ctrller.compliant_hold_gains.kp != rhs->ctrller.compliant_hold_gains.kp || lhs->ctrller.compliant_hold_gains.kd != rhs->ctrller.compliant_hold_gains.kd ||
        lhs->ctrller.compliant_drag_gains.kp != rhs->ctrller.compliant_drag_gains.kp || lhs->ctrller.compliant_drag_gains.kd != rhs->ctrller.compliant_drag_gains.kd ||
        lhs->ctrller.compliant_tracking_gains.kp != rhs->ctrller.compliant_tracking_gains.kp || lhs->ctrller.compliant_tracking_gains.kd != rhs->ctrller.compliant_tracking_gains.kd) add("controller.impedance_gains");
    if(lhs->safety.limits.min_pos != rhs->safety.limits.min_pos || lhs->safety.limits.max_pos != rhs->safety.limits.max_pos) add("limits.position");
    if(lhs->safety.limits.max_vel != rhs->safety.limits.max_vel) add("limits.max_cmd_vel_or_state_base");
    if(lhs->safety.limits.max_acc != rhs->safety.limits.max_acc) add("limits.max_acc");
    if(lhs->safety.limits.max_effort != rhs->safety.limits.max_effort) add("limits.max_effort");
    if(lhs->safety.limits.max_kp != rhs->safety.limits.max_kp) add("limits.max_kp");
    if(lhs->safety.limits.max_kd != rhs->safety.limits.max_kd) add("limits.max_kd");
    if(lhs->safety.cmd_timeout_s != rhs->safety.cmd_timeout_s || lhs->safety.state_timeout_s != rhs->safety.state_timeout_s || lhs->safety.max_dt_s != rhs->safety.max_dt_s) add("safety.timeout");
    if(lhs->damiao.actuators.size() != rhs->damiao.actuators.size()) add("damiao.actuators");
    else {
        for(std::size_t i = 0; i < lhs->damiao.actuators.size(); ++i) {
            if(lhs->damiao.actuators[i].motor_id != rhs->damiao.actuators[i].motor_id || lhs->damiao.actuators[i].motor_type != rhs->damiao.actuators[i].motor_type) {
                add("damiao.motor_id_or_type");
                break;
            }
        }
    }
    if(lhs->dynamics.urdf_path != rhs->dynamics.urdf_path) add("dynamics.urdf_path");
    if(lhs->shutdown.park_before_disable != rhs->shutdown.park_before_disable || lhs->shutdown.park_pos != rhs->shutdown.park_pos ||
        lhs->shutdown.speed_scale != rhs->shutdown.speed_scale || lhs->shutdown.timeout_s != rhs->shutdown.timeout_s) add("shutdown");
    return diffs;
}

// ! ========================= 私 有 类 方 法 实 现 ========================= ! //

} // namespace serial_arm
