#include "dm_arm/core/safety.hpp"

namespace dm_arm {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //



// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

tl::expected<void, SafetyErr> Safety::configure(const SafetyCfg& cfg) {

}

tl::expected<void, SafetyErr> Safety::check(const JointState& joint_state, const ActuatorState& actuator_state) {

}

// ! ========================= 私 有 类 方 法 实 现 ========================= ! //

} // namespace dm_arm
