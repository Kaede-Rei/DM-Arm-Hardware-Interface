#include "dm_arm/config/config.hpp"

#include <cmath>
#include <fstream>
#include <set>
#include <sstream>

namespace dm_arm {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //

namespace {

/**
 * @brief 解析配置文件行
 */
struct ParsedLine {
    std::size_t number{ 0 };    ///< 行号
    std::size_t indent{ 0 };    ///< 缩进空格数
    std::string text;           ///< 去除注释和首尾空白后的文本
};

/**
 * @brief 达妙电机物理限制
 */
struct MotorLimit {
    double q_max;       ///< 最大位置
    double dq_max;      ///< 最大速度
    double tau_max;     ///< 最大力矩
};

/**
 * @brief 构造配置错误
 * @param code 配置错误码
 * @param message 配置错误描述
 * @return ConfigErr 配置错误信息
 */
ConfigErr make_err(ConfigErrc code, const std::string& message) {
    return ConfigErr{ code, message };
}

/**
 * @brief 构造配置错误返回值
 * @param code 配置错误码
 * @param message 配置错误描述
 * @return tl::expected<void, ConfigErr> 配置错误返回值
 */
tl::expected<void, ConfigErr> fail(ConfigErrc code, const std::string& message) {
    return tl::make_unexpected(make_err(code, message));
}

/**
 * @brief 去除字符串首尾空白字符
 * @param value 输入字符串
 * @return std::string 去除首尾空白后的字符串
 */
std::string trim(const std::string& value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if(first == std::string::npos) return {};
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

/**
 * @brief 去除字符串首尾引号
 * @param value 输入字符串
 * @return std::string 去除引号后的字符串
 */
std::string unquote(std::string value) {
    value = trim(value);
    if(value.size() >= 2 && ((value.front() == '"' && value.back() == '"') ||
        (value.front() == '\'' && value.back() == '\''))) {
        return value.substr(1, value.size() - 2);
    }
    return value;
}

/**
 * @brief 拆分 key: value 配置行
 * @param line 已解析的配置行
 * @return std::pair<std::string, std::string> 键和值
 */
std::pair<std::string, std::string> split_key_value(const ParsedLine& line) {
    const auto pos = line.text.find(':');
    if(pos == std::string::npos) return {};
    return { trim(line.text.substr(0, pos)), trim(line.text.substr(pos + 1)) };
}

/**
 * @brief 读取配置文件有效行
 * @param path 配置文件路径
 * @return tl::expected<std::vector<ParsedLine>, ConfigErr> 已解析行列表
 */
tl::expected<std::vector<ParsedLine>, ConfigErr> read_lines(const std::string& path) {
    std::ifstream input(path);
    if(!input) {
        return tl::make_unexpected(make_err(ConfigErrc::FILE_OPEN_FAILED,
            "cannot open config file: " + path));
    }

    std::vector<ParsedLine> lines;
    std::string raw;
    std::size_t line_number = 0;
    while(std::getline(input, raw)) {
        ++line_number;
        const auto comment = raw.find('#');
        if(comment != std::string::npos) raw.erase(comment);
        if(trim(raw).empty()) continue;

        std::size_t indent = 0;
        while(indent < raw.size() && raw[indent] == ' ') ++indent;
        if(indent % 2 != 0) {
            return tl::make_unexpected(make_err(ConfigErrc::SYNTAX_ERROR,
                "line " + std::to_string(line_number) + ": indentation must use multiples of two spaces"));
        }
        lines.push_back(ParsedLine{ line_number, indent, trim(raw) });
    }
    return lines;
}

template<typename T>
tl::expected<T, ConfigErr> parse_number(const std::string& text, std::size_t line_number);

/**
 * @brief 解析浮点数
 * @param text 待解析文本
 * @param line_number 行号
 * @return tl::expected<double, ConfigErr> 浮点数值
 */
template<>
tl::expected<double, ConfigErr> parse_number<double>(const std::string& text, std::size_t line_number) {
    try {
        std::size_t used = 0;
        const double value = std::stod(trim(text), &used);
        if(used != trim(text).size() || !std::isfinite(value)) throw std::invalid_argument("invalid");
        return value;
    }
    catch(...) {
        return tl::make_unexpected(make_err(ConfigErrc::SYNTAX_ERROR,
            "line " + std::to_string(line_number) + ": invalid floating-point value '" + text + "'"));
    }
}

/**
 * @brief 解析无符号整数
 * @param text 待解析文本
 * @param line_number 行号
 * @return tl::expected<std::size_t, ConfigErr> 无符号整数值
 */
template<>
tl::expected<std::size_t, ConfigErr> parse_number<std::size_t>(const std::string& text, std::size_t line_number) {
    try {
        std::size_t used = 0;
        const unsigned long long value = std::stoull(trim(text), &used, 0);
        if(used != trim(text).size() || value > std::numeric_limits<std::size_t>::max()) throw std::invalid_argument("invalid");
        return static_cast<std::size_t>(value);
    }
    catch(...) {
        return tl::make_unexpected(make_err(ConfigErrc::SYNTAX_ERROR,
            "line " + std::to_string(line_number) + ": invalid unsigned integer '" + text + "'"));
    }
}

/**
 * @brief 解析 32 位无符号整数
 * @param text 待解析文本
 * @param line_number 行号
 * @return tl::expected<std::uint32_t, ConfigErr> 32 位无符号整数值
 */
template<>
tl::expected<std::uint32_t, ConfigErr> parse_number<std::uint32_t>(const std::string& text, std::size_t line_number) {
    try {
        std::size_t used = 0;
        const unsigned long long value = std::stoull(trim(text), &used, 0);
        if(used != trim(text).size() || value > std::numeric_limits<std::uint32_t>::max()) throw std::invalid_argument("invalid");
        return static_cast<std::uint32_t>(value);
    }
    catch(...) {
        return tl::make_unexpected(make_err(ConfigErrc::SYNTAX_ERROR,
            "line " + std::to_string(line_number) + ": invalid uint32 value '" + text + "'"));
    }
}

/**
 * @brief 解析整数
 * @param text 待解析文本
 * @param line_number 行号
 * @return tl::expected<int, ConfigErr> 整数值
 */
tl::expected<int, ConfigErr> parse_int(const std::string& text, std::size_t line_number) {
    try {
        std::size_t used = 0;
        const long value = std::stol(trim(text), &used, 0);
        if(used != trim(text).size() || value < std::numeric_limits<int>::min() || value > std::numeric_limits<int>::max()) {
            throw std::invalid_argument("invalid");
        }
        return static_cast<int>(value);
    }
    catch(...) {
        return tl::make_unexpected(make_err(ConfigErrc::SYNTAX_ERROR,
            "line " + std::to_string(line_number) + ": invalid integer '" + text + "'"));
    }
}

/**
 * @brief 解析布尔值
 * @param text 待解析文本
 * @param line_number 行号
 * @return tl::expected<bool, ConfigErr> 布尔值
 */
tl::expected<bool, ConfigErr> parse_bool(const std::string& text, std::size_t line_number) {
    const auto value = trim(text);
    if(value == "true") return true;
    if(value == "false") return false;
    return tl::make_unexpected(make_err(ConfigErrc::SYNTAX_ERROR,
        "line " + std::to_string(line_number) + ": expected true or false"));
}

/**
 * @brief 拆分内联列表
 * @param text 待解析文本
 * @param line_number 行号
 * @return tl::expected<std::vector<std::string>, ConfigErr> 字符串列表
 */
tl::expected<std::vector<std::string>, ConfigErr> split_list(const std::string& text, std::size_t line_number) {
    const auto value = trim(text);
    if(value.size() < 2 || value.front() != '[' || value.back() != ']') {
        return tl::make_unexpected(make_err(ConfigErrc::SYNTAX_ERROR,
            "line " + std::to_string(line_number) + ": expected inline list [..]"));
    }

    std::vector<std::string> result;
    std::stringstream stream(value.substr(1, value.size() - 2));
    std::string item;
    while(std::getline(stream, item, ',')) {
        item = unquote(item);
        if(item.empty()) {
            return tl::make_unexpected(make_err(ConfigErrc::SYNTAX_ERROR,
                "line " + std::to_string(line_number) + ": empty list item"));
        }
        result.push_back(item);
    }
    return result;
}

/**
 * @brief 解析浮点数列表
 * @param text 待解析文本
 * @param line_number 行号
 * @return tl::expected<JointVector, ConfigErr> 关节向量
 */
tl::expected<JointVector, ConfigErr> parse_double_list(const std::string& text, std::size_t line_number) {
    auto items = split_list(text, line_number);
    if(!items) return tl::make_unexpected(items.error());
    JointVector values;
    values.reserve(items->size());
    for(const auto& item : *items) {
        auto value = parse_number<double>(item, line_number);
        if(!value) return tl::make_unexpected(value.error());
        values.push_back(*value);
    }
    return values;
}

/**
 * @brief 解析整数列表
 * @param text 待解析文本
 * @param line_number 行号
 * @return tl::expected<std::vector<int>, ConfigErr> 整数列表
 */
tl::expected<std::vector<int>, ConfigErr> parse_int_list(const std::string& text, std::size_t line_number) {
    auto items = split_list(text, line_number);
    if(!items) return tl::make_unexpected(items.error());
    std::vector<int> values;
    values.reserve(items->size());
    for(const auto& item : *items) {
        auto value = parse_int(item, line_number);
        if(!value) return tl::make_unexpected(value.error());
        values.push_back(*value);
    }
    return values;
}

/**
 * @brief 检查向量是否为有限值
 * @param values 待检查的向量
 * @return true 如果向量中的所有元素都是有限值，否则返回 false
 */
bool finite_vector(const JointVector& values) {
    return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); });
}

/**
 * @brief 查找达妙电机物理限制
 * @param motor_type 电机型号
 * @return const MotorLimit* 电机物理限制，未找到时返回 nullptr
 */
const MotorLimit* find_motor_limit(const std::string& motor_type) {
    static const std::unordered_map<std::string, MotorLimit> limits = {
        {"DM4310", {12.5, 30.0, 10.0}},
        {"DM4310_48V", {12.5, 50.0, 10.0}},
        {"DM4340", {12.5, 8.0, 28.0}},
        {"DM4340_48V", {12.5, 10.0, 28.0}},
        {"DM6006", {12.5, 45.0, 20.0}},
        {"DM6248P", {12.566, 20.0, 120.0}},
        {"DM8006", {12.5, 45.0, 40.0}},
        {"DM8009", {12.5, 45.0, 54.0}},
        {"DM10010L", {12.5, 25.0, 200.0}},
        {"DM10010", {12.5, 20.0, 200.0}},
        {"DMH3510", {12.5, 28.0, 1.0}},
        {"DMH6215", {12.5, 45.0, 10.0}},
        {"DMG6220", {12.5, 45.0, 10.0}},
        {"DMJH11", {12.5, 10.0, 12.0}},
    };
    const auto it = limits.find(motor_type);
    return it == limits.end() ? nullptr : &it->second;
}

/**
 * @brief 获取所有关节阻抗增益配置
 * @param cfg 关节控制器配置
 * @return const std::vector<const JointImpedanceGains*> 关节阻抗增益配置指针列表
 */
const std::vector<const JointImpedanceGains*> all_gains(const JointCtrllerCfg& cfg) {
    return {
        &cfg.rigid_hold_gains,
        &cfg.rigid_tracking_gains,
        &cfg.compliant_hold_gains,
        &cfg.compliant_drag_gains,
        &cfg.compliant_tracking_gains,
    };
}

} // namespace

// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

/**
 * @brief 加载机器人配置
 * @param path 配置文件路径
 * @return tl::expected<RobotCfg, ConfigErr> 机器人配置
 */
tl::expected<RobotCfg, ConfigErr> load_robot_cfg(const std::string& path) {
    auto parsed_lines = read_lines(path);
    if(!parsed_lines) return tl::make_unexpected(parsed_lines.error());

    RobotCfg cfg;
    std::string section;
    std::string controller_mode;
    bool in_actuators = false;
    DamiaoActuatorCfg* actuator = nullptr;

    for(const auto& line : *parsed_lines) {
        if(line.indent == 0) {
            if(line.text.back() != ':') {
                return tl::make_unexpected(make_err(ConfigErrc::SYNTAX_ERROR,
                    "line " + std::to_string(line.number) + ": top-level section must end with ':'"));
            }
            section = trim(line.text.substr(0, line.text.size() - 1));
            controller_mode.clear();
            in_actuators = false;
            actuator = nullptr;
            continue;
        }

        const auto [key, value] = split_key_value(line);
        if(key.empty()) {
            return tl::make_unexpected(make_err(ConfigErrc::SYNTAX_ERROR,
                "line " + std::to_string(line.number) + ": expected key: value"));
        }

        if(section == "joints" && line.indent == 2 && key == "names") {
            auto values = split_list(value, line.number);
            if(!values) return tl::make_unexpected(values.error());
            cfg.joint_names = *values;
            continue;
        }

        if(section == "runtime" && line.indent == 2) {
            if(key == "ctrl_frequency_hz") {
                auto parsed = parse_number<double>(value, line.number); if(!parsed) return tl::make_unexpected(parsed.error()); cfg.runtime.ctrl_frequency_hz = *parsed;
            }
            else if(key == "cmd_timeout_s") {
                auto parsed = parse_number<double>(value, line.number); if(!parsed) return tl::make_unexpected(parsed.error()); cfg.runtime.cmd_timeout_s = *parsed;
            }
            else if(key == "state_timeout_s") {
                auto parsed = parse_number<double>(value, line.number); if(!parsed) return tl::make_unexpected(parsed.error()); cfg.runtime.state_timeout_s = *parsed;
            }
            else if(key == "startup_read_cycles") {
                auto parsed = parse_number<std::size_t>(value, line.number); if(!parsed) return tl::make_unexpected(parsed.error()); cfg.runtime.startup_read_cycles = *parsed;
            }
            else if(key == "write_enabled") {
                auto parsed = parse_bool(value, line.number); if(!parsed) return tl::make_unexpected(parsed.error()); cfg.runtime.write_enabled = *parsed;
            }
            else if(key == "refresh_state_in_read") {
                auto parsed = parse_bool(value, line.number); if(!parsed) return tl::make_unexpected(parsed.error()); cfg.runtime.refresh_state_in_read = *parsed;
            }
            else return tl::make_unexpected(make_err(ConfigErrc::SYNTAX_ERROR, "line " + std::to_string(line.number) + ": unknown runtime key '" + key + "'"));
            continue;
        }

        if(section == "limits" && line.indent == 2) {
            auto values = parse_double_list(value, line.number);
            if(!values) return tl::make_unexpected(values.error());
            if(key == "min_pos") cfg.limits.min_pos = *values;
            else if(key == "max_pos") cfg.limits.max_pos = *values;
            else if(key == "max_vel") cfg.limits.max_vel = *values;
            else if(key == "max_acc") cfg.limits.max_acc = *values;
            else if(key == "max_effort") cfg.limits.max_effort = *values;
            else if(key == "max_kp") cfg.limits.max_kp = *values;
            else if(key == "max_kd") cfg.limits.max_kd = *values;
            else return tl::make_unexpected(make_err(ConfigErrc::SYNTAX_ERROR, "line " + std::to_string(line.number) + ": unknown limits key '" + key + "'"));
            continue;
        }

        if(section == "mapping" && line.indent == 2) {
            if(key == "direction") {
                auto values = parse_int_list(value, line.number); if(!values) return tl::make_unexpected(values.error()); cfg.mapper.direction = *values;
            }
            else {
                auto values = parse_double_list(value, line.number); if(!values) return tl::make_unexpected(values.error());
                if(key == "pos_ratio") cfg.mapper.pos_ratio = *values;
                else if(key == "tor_ratio") cfg.mapper.tor_ratio = *values;
                else if(key == "joint_zero_offset") cfg.mapper.joint_zero_offset = *values;
                else if(key == "actuator_zero_offset") cfg.mapper.actuator_zero_offset = *values;
                else return tl::make_unexpected(make_err(ConfigErrc::SYNTAX_ERROR, "line " + std::to_string(line.number) + ": unknown mapping key '" + key + "'"));
            }
            continue;
        }

        if(section == "controller") {
            if(line.indent == 2 && key == "allow_full_cmd") {
                auto parsed = parse_bool(value, line.number); if(!parsed) return tl::make_unexpected(parsed.error()); cfg.ctrller.allow_full_cmd = *parsed;
                continue;
            }
            if(line.indent == 2 && value.empty()) {
                controller_mode = key;
                continue;
            }
            if(line.indent == 4 && (key == "kp" || key == "kd")) {
                auto values = parse_double_list(value, line.number); if(!values) return tl::make_unexpected(values.error());
                JointImpedanceGains* gains = nullptr;
                if(controller_mode == "rigid_hold") gains = &cfg.ctrller.rigid_hold_gains;
                else if(controller_mode == "rigid_tracking") gains = &cfg.ctrller.rigid_tracking_gains;
                else if(controller_mode == "compliant_hold") gains = &cfg.ctrller.compliant_hold_gains;
                else if(controller_mode == "compliant_drag") gains = &cfg.ctrller.compliant_drag_gains;
                else if(controller_mode == "compliant_tracking") gains = &cfg.ctrller.compliant_tracking_gains;
                else return tl::make_unexpected(make_err(ConfigErrc::SYNTAX_ERROR, "line " + std::to_string(line.number) + ": unknown controller mode '" + controller_mode + "'"));
                if(key == "kp") gains->kp = *values; else gains->kd = *values;
                continue;
            }
        }

        if(section == "damiao") {
            if(line.indent == 2 && key == "actuators" && value.empty()) {
                in_actuators = true;
                actuator = nullptr;
                continue;
            }
            if(line.indent == 2 && !in_actuators) {
                if(key == "serial_port") cfg.damiao.serial_port = unquote(value);
                else if(key == "baudrate") { auto parsed = parse_int(value, line.number); if(!parsed) return tl::make_unexpected(parsed.error()); cfg.damiao.baudrate = *parsed; }
                else if(key == "stop_kp") { auto parsed = parse_number<double>(value, line.number); if(!parsed) return tl::make_unexpected(parsed.error()); cfg.damiao.stop_kp = *parsed; }
                else if(key == "stop_kd") { auto parsed = parse_number<double>(value, line.number); if(!parsed) return tl::make_unexpected(parsed.error()); cfg.damiao.stop_kd = *parsed; }
                else if(key == "stop_cycles") { auto parsed = parse_number<std::size_t>(value, line.number); if(!parsed) return tl::make_unexpected(parsed.error()); cfg.damiao.stop_cycles = *parsed; }
                else return tl::make_unexpected(make_err(ConfigErrc::SYNTAX_ERROR, "line " + std::to_string(line.number) + ": unknown damiao key '" + key + "'"));
                continue;
            }
            if(in_actuators && line.indent == 4 && line.text.rfind("- ", 0) == 0) {
                ParsedLine item_line = line;
                item_line.text = trim(line.text.substr(2));
                const auto [item_key, item_value] = split_key_value(item_line);
                if(item_key != "name" || item_value.empty()) {
                    return tl::make_unexpected(make_err(ConfigErrc::SYNTAX_ERROR,
                        "line " + std::to_string(line.number) + ": actuator item must start with '- name:'"));
                }
                cfg.damiao.actuators.push_back(DamiaoActuatorCfg{});
                actuator = &cfg.damiao.actuators.back();
                actuator->name = unquote(item_value);
                continue;
            }
            if(in_actuators && line.indent == 6 && actuator != nullptr) {
                if(key == "joint_name") actuator->joint_name = unquote(value);
                else if(key == "motor_id") { auto parsed = parse_number<std::uint32_t>(value, line.number); if(!parsed) return tl::make_unexpected(parsed.error()); actuator->motor_id = *parsed; }
                else if(key == "master_id") { auto parsed = parse_number<std::uint32_t>(value, line.number); if(!parsed) return tl::make_unexpected(parsed.error()); actuator->master_id = *parsed; }
                else if(key == "motor_type") actuator->motor_type = unquote(value);
                else return tl::make_unexpected(make_err(ConfigErrc::SYNTAX_ERROR, "line " + std::to_string(line.number) + ": unknown actuator key '" + key + "'"));
                continue;
            }
        }

        return tl::make_unexpected(make_err(ConfigErrc::SYNTAX_ERROR,
            "line " + std::to_string(line.number) + ": unsupported key or indentation"));
    }

    cfg.ctrller.joints_count = cfg.joint_names.size();
    cfg.mapper.joints_count = cfg.joint_names.size();
    cfg.damiao.startup_read_cycles = cfg.runtime.startup_read_cycles;
    cfg.damiao.refresh_state_in_read = cfg.runtime.refresh_state_in_read;

    auto validated = validate_robot_cfg(cfg);
    if(!validated) return tl::make_unexpected(validated.error());
    return cfg;
}

/**
 * @brief 验证机器人配置
 * @param cfg 机器人配置
 * @return tl::expected<void, ConfigErr>
 */
tl::expected<void, ConfigErr> validate_robot_cfg(const RobotCfg& cfg) {
    const std::size_t n = cfg.joint_names.size();
    if(n != DM_ARM_JOINTS_COUNT) {
        return fail(ConfigErrc::INVALID_SIZE, "DM-Arm main chain requires exactly 6 joints");
    }

    const auto size_is_n = [n](const auto& values) { return values.size() == n; };
    if(!size_is_n(cfg.limits.min_pos) || !size_is_n(cfg.limits.max_pos) ||
        !size_is_n(cfg.limits.max_vel) || !size_is_n(cfg.limits.max_acc) ||
        !size_is_n(cfg.limits.max_effort) || !size_is_n(cfg.limits.max_kp) ||
        !size_is_n(cfg.limits.max_kd) || cfg.damiao.actuators.size() != n) {
        return fail(ConfigErrc::INVALID_SIZE, "all joint arrays and actuator entries must have length 6");
    }

    if(cfg.ctrller.joints_count != n || cfg.mapper.joints_count != n) {
        return fail(ConfigErrc::INVALID_SIZE, "controller and mapper joints_count must match joint_names");
    }

    if(!std::isfinite(cfg.runtime.ctrl_frequency_hz) ||
        !std::isfinite(cfg.runtime.cmd_timeout_s) ||
        !std::isfinite(cfg.runtime.state_timeout_s) ||
        cfg.runtime.ctrl_frequency_hz <= 0.0 || cfg.runtime.cmd_timeout_s <= 0.0 ||
        cfg.runtime.state_timeout_s <= 0.0 || cfg.runtime.startup_read_cycles == 0) {
        return fail(ConfigErrc::INVALID_VALUE, "runtime frequency, timeouts and startup_read_cycles must be finite and positive");
    }

    if(cfg.damiao.serial_port.empty() || cfg.damiao.baudrate <= 0 || cfg.damiao.stop_cycles == 0 ||
        !std::isfinite(cfg.damiao.stop_kp) || !std::isfinite(cfg.damiao.stop_kd) ||
        cfg.damiao.stop_kp < 0.0 || cfg.damiao.stop_kp > 500.0 ||
        cfg.damiao.stop_kd < 0.0 || cfg.damiao.stop_kd > 5.0) {
        return fail(ConfigErrc::INVALID_VALUE, "invalid Damiao port, baudrate or stop parameters");
    }

    std::set<std::string> joint_names;
    std::set<std::string> actuator_names;
    std::set<std::uint32_t> motor_ids;
    for(std::size_t i = 0; i < n; ++i) {
        if(cfg.joint_names[i].empty() || !joint_names.insert(cfg.joint_names[i]).second) {
            return fail(ConfigErrc::DUPLICATE_NAME, "joint names must be non-empty and unique");
        }
        const auto& actuator = cfg.damiao.actuators[i];
        if(actuator.name.empty() || !actuator_names.insert(actuator.name).second) {
            return fail(ConfigErrc::DUPLICATE_NAME, "actuator names must be non-empty and unique");
        }
        if(actuator.joint_name != cfg.joint_names[i]) {
            return fail(ConfigErrc::INVALID_VALUE, "actuator order/joint_name must match joint_names order at index " + std::to_string(i));
        }
        if(actuator.motor_id == 0 || !motor_ids.insert(actuator.motor_id).second) {
            return fail(ConfigErrc::DUPLICATE_MOTOR_ID, "motor IDs must be non-zero and unique");
        }
        if(find_motor_limit(actuator.motor_type) == nullptr) {
            return fail(ConfigErrc::INVALID_MOTOR_TYPE, "unknown motor type: " + actuator.motor_type);
        }
    }

    const std::vector<const JointVector*> limit_vectors = {
        &cfg.limits.min_pos, &cfg.limits.max_pos, &cfg.limits.max_vel,
        &cfg.limits.max_acc, &cfg.limits.max_effort, &cfg.limits.max_kp,
        &cfg.limits.max_kd,
    };
    for(const auto* values : limit_vectors) {
        if(!finite_vector(*values)) return fail(ConfigErrc::INVALID_VALUE, "joint limits contain NaN or Inf");
    }

    JointActuatorMapper mapper;
    if(!mapper.configure(cfg.mapper)) {
        return fail(ConfigErrc::INVALID_VALUE, "invalid joint/actuator mapping");
    }

    JointCtrller ctrller;
    if(!ctrller.configure(cfg.ctrller)) {
        return fail(ConfigErrc::INVALID_VALUE, "invalid joint controller gains");
    }

    for(const auto* gains : all_gains(cfg.ctrller)) {
        if(gains->kp.size() != n || gains->kd.size() != n) {
            return fail(ConfigErrc::INVALID_SIZE, "every impedance gain vector must have length 6");
        }
    }

    for(std::size_t i = 0; i < n; ++i) {
        if(cfg.limits.min_pos[i] >= cfg.limits.max_pos[i] || cfg.limits.max_vel[i] <= 0.0 ||
            cfg.limits.max_acc[i] <= 0.0 || cfg.limits.max_effort[i] <= 0.0 ||
            cfg.limits.max_kp[i] < 0.0 || cfg.limits.max_kp[i] > 500.0 ||
            cfg.limits.max_kd[i] < 0.0 || cfg.limits.max_kd[i] > 5.0) {
            return fail(ConfigErrc::INVALID_VALUE, "invalid joint limit at index " + std::to_string(i));
        }

        for(const auto* gains : all_gains(cfg.ctrller)) {
            if(gains->kp[i] > cfg.limits.max_kp[i] || gains->kd[i] > cfg.limits.max_kd[i]) {
                return fail(ConfigErrc::INVALID_VALUE, "controller gain exceeds configured joint limit at index " + std::to_string(i));
            }
        }

        const auto* motor_limit = find_motor_limit(cfg.damiao.actuators[i].motor_type);
        const double direction = static_cast<double>(cfg.mapper.direction[i]);
        const double pos_ratio = cfg.mapper.pos_ratio[i];
        const double tor_ratio = cfg.mapper.tor_ratio[i];
        const double q_a_min = cfg.mapper.actuator_zero_offset[i] + direction * pos_ratio *
            (cfg.limits.min_pos[i] - cfg.mapper.joint_zero_offset[i]);
        const double q_a_max = cfg.mapper.actuator_zero_offset[i] + direction * pos_ratio *
            (cfg.limits.max_pos[i] - cfg.mapper.joint_zero_offset[i]);
        const double actuator_vel = pos_ratio * cfg.limits.max_vel[i];
        const double actuator_tau = cfg.limits.max_effort[i] / tor_ratio;
        const double actuator_kp = cfg.limits.max_kp[i] / (pos_ratio * tor_ratio);
        const double actuator_kd = cfg.limits.max_kd[i] / (pos_ratio * tor_ratio);

        if(std::max(std::abs(q_a_min), std::abs(q_a_max)) > motor_limit->q_max ||
            actuator_vel > motor_limit->dq_max || actuator_tau > motor_limit->tau_max ||
            actuator_kp > 500.0 || actuator_kd > 5.0) {
            return fail(ConfigErrc::ACTUATOR_LIMIT_EXCEEDED,
                "mapped limit exceeds " + cfg.damiao.actuators[i].motor_type + " range at index " + std::to_string(i));
        }
    }

    return {};
}

// ! ========================= 私 有 类 方 法 实 现 ========================= ! //

} // namespace dm_arm
