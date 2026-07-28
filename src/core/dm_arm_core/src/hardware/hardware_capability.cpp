#include "dm_arm/hardware/hardware_capability.hpp"

#include "dm_hw/damiao.hpp"

#include <set>

namespace dm_arm {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //

namespace {

/**
 * @brief 解析达妙电机型号
 */
tl::expected<damiao::DmMotorType, HardwareCapabilityErr> parse_motor_type(const std::string& value) {
    if(value == "DM4310") return damiao::DM4310;
    if(value == "DM4310_48V") return damiao::DM4310_48V;
    if(value == "DM4340") return damiao::DM4340;
    if(value == "DM4340_48V") return damiao::DM4340_48V;
    if(value == "DM6006") return damiao::DM6006;
    if(value == "DM6248P") return damiao::DM6248P;
    if(value == "DM8006") return damiao::DM8006;
    if(value == "DM8009") return damiao::DM8009;
    if(value == "DM10010L") return damiao::DM10010L;
    if(value == "DM10010") return damiao::DM10010;
    if(value == "DMH3510") return damiao::DMH3510;
    if(value == "DMH6215") return damiao::DMH6215;
    if(value == "DMG6220") return damiao::DMG6220;
    if(value == "DMJH11") return damiao::DMJH11;
    return tl::make_unexpected(HardwareCapabilityErr::UNKNOWN_MOTOR_TYPE);
}

} // namespace

// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

/**
 * @brief 从达妙硬件配置读取执行器物理能力
 */
tl::expected<HardwareCapabilities, HardwareCapabilityErr> load_damiao_capabilities(const DamiaoBusCfg& cfg) {
    std::set<std::string> names;
    HardwareCapabilities output;
    output.reserve(cfg.actuators.size());
    for(const auto& actuator : cfg.actuators) {
        if(actuator.name.empty() || !names.insert(actuator.name).second) return tl::make_unexpected(HardwareCapabilityErr::INVALID_CFG);
        const auto type = parse_motor_type(actuator.motor_type);
        if(!type) return tl::make_unexpected(type.error());
        const damiao::LimitParam limit = damiao::limit_param[*type];

        ActuatorCapability capability;
        capability.actuator_name = actuator.name;
        capability.min_pos = -static_cast<double>(limit.q_max);
        capability.max_pos = static_cast<double>(limit.q_max);
        capability.max_vel = static_cast<double>(limit.dq_max);
        capability.max_effort = static_cast<double>(limit.tau_max);
        capability.max_kp = 500.0;
        capability.max_kd = 5.0;
        output.push_back(std::move(capability));
    }
    return output;
}

// ! ========================= 私 有 类 方 法 实 现 ========================= ! //

} // namespace dm_arm
