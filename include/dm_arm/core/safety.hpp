#pragma once

#include <tl/expected.hpp>
#include "types.hpp"

namespace dm_arm {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

enum class SafetyErr {

};

struct SafetyCfg {

};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

class Safety {
public:
    tl::expected<void, SafetyErr> configure(const SafetyCfg& cfg);
    tl::expected<void, SafetyErr> check(const JointState& joint_state, const ActuatorState& actuator_state);
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //

} // namespace dm_arm
