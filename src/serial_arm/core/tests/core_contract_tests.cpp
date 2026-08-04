#include "serial_arm/config/limit_resolver.hpp"
#include "serial_arm/core/joint_actuator_mapper.hpp"
#include "serial_arm/core/safety.hpp"
#include "serial_arm/dynamics/dynamics.hpp"
#include "serial_arm/hardware/motor_bus.hpp"
#include "serial_arm/model/model_loader.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <limits>
#include <string>
#include <vector>

namespace {

using namespace serial_arm;

std::string fixture_path(const std::string& name) {
    return std::string(SERIAL_ARM_TEST_FIXTURE_DIR) + "/" + name;
}

JointState joint_state(std::size_t n) {
    return JointState{
        JointVector(n, 0.0),
        JointVector(n, 0.0),
        JointVector(n, 0.0),
    };
}

ActuatorState actuator_state(std::size_t n) {
    return ActuatorState{
        ActuatorVector(n, 0.0),
        ActuatorVector(n, 0.0),
        ActuatorVector(n, 0.0),
        std::vector<std::uint8_t>(n, 1),
        std::vector<std::uint8_t>(n, 1),
        std::vector<int>(n, 0),
    };
}

JointCtrlCmd joint_cmd(std::size_t n) {
    return JointCtrlCmd{
        JointVector(n, 0.0),
        JointVector(n, 0.0),
        JointVector(n, 0.0),
        JointVector(n, 1.0),
        JointVector(n, 0.1),
    };
}

SafetyCfg safety_cfg(bool continuous) {
    SafetyCfg cfg;
    cfg.joints_count = 1;
    cfg.limits.has_position_limit = { static_cast<std::uint8_t>(continuous ? 0 : 1) };
    cfg.limits.min_pos = { -1.0 };
    cfg.limits.max_pos = { 1.0 };
    cfg.limits.max_vel = { 2.0 };
    cfg.limits.max_acc = { 100.0 };
    cfg.limits.max_effort = { 5.0 };
    cfg.limits.max_kp = { 20.0 };
    cfg.limits.max_kd = { 2.0 };
    cfg.limits.pos_margin = { 0.1 };
    cfg.fault_recovery.compliant_recovery.kp = { 1.0 };
    cfg.fault_recovery.compliant_recovery.kd = { 0.1 };
    cfg.fault_recovery.compliant_recovery.max_vel = { 1.0 };
    return cfg;
}

class FakeMotorBus final : public MotorBus {
public:
    tl::expected<void, MotorBusErr> configure(const std::string&) override { return {}; }
    tl::expected<void, MotorBusErr> connect() override { return {}; }
    tl::expected<ActuatorState, MotorBusErr> read() override { return actuator_state(caps_.size()); }
    tl::expected<void, MotorBusErr> activate() override { return {}; }
    tl::expected<void, MotorBusErr> write(const ActuatorCtrlCmd& cmd) override {
        last_cmd = cmd;
        return {};
    }
    tl::expected<void, MotorBusErr> stop() override { return {}; }
    tl::expected<void, MotorBusErr> deactivate() override { return {}; }
    tl::expected<void, MotorBusErr> recover() override { return {}; }
    const HardwareCapabilities& capabilities() const noexcept override { return caps_; }
    void cleanup() noexcept override {}
    std::size_t size() const noexcept override { return caps_.size(); }

    HardwareCapabilities caps_{ { "actuator1", 10.0, 10.0, 10.0, 100.0, 10.0 } };
    ActuatorCtrlCmd last_cmd;
};

} // namespace

TEST(ContinuousJointSafety, PositionLimitsAreSkippedForContinuousJoints) {
    Safety safety;
    ASSERT_TRUE(safety.configure(safety_cfg(true)));

    JointState state = joint_state(1);
    state.pos[0] = 10.0;
    EXPECT_TRUE(safety.check_state(state, actuator_state(1), 0.0));

    JointCtrlCmd cmd = joint_cmd(1);
    cmd.pos[0] = -10.0;
    cmd.kp[0] = 0.0;
    EXPECT_TRUE(safety.check_joint_cmd(state, cmd, 0.001));
}

TEST(ContinuousJointSafety, VelocityEffortAndFiniteChecksStillApply) {
    Safety safety;
    ASSERT_TRUE(safety.configure(safety_cfg(true)));

    JointState state = joint_state(1);
    state.vel[0] = 4.0;
    auto state_result = safety.check_state(state, actuator_state(1), 0.0);
    ASSERT_FALSE(state_result);
    EXPECT_EQ(state_result.error().code, SafetyErr::JOINT_VEL_LIMIT);

    state = joint_state(1);
    JointCtrlCmd cmd = joint_cmd(1);
    cmd.tor[0] = 6.0;
    auto effort_result = safety.check_joint_cmd(state, cmd, 0.001);
    ASSERT_FALSE(effort_result);
    EXPECT_EQ(effort_result.error().code, SafetyErr::CMD_EFFORT_LIMIT);

    cmd = joint_cmd(1);
    cmd.pos[0] = std::numeric_limits<double>::infinity();
    auto finite_result = safety.check_joint_cmd(state, cmd, 0.001);
    ASSERT_FALSE(finite_result);
    EXPECT_EQ(finite_result.error().code, SafetyErr::NON_FINITE_CMD);
}

TEST(ContinuousJointSafety, RevolutePositionLimitsRemainActive) {
    Safety safety;
    ASSERT_TRUE(safety.configure(safety_cfg(false)));

    JointState state = joint_state(1);
    state.pos[0] = 2.0;
    auto state_result = safety.check_state(state, actuator_state(1), 0.0);
    ASSERT_FALSE(state_result);
    EXPECT_EQ(state_result.error().code, SafetyErr::JOINT_POS_LIMIT);

    state.pos[0] = 0.0;
    JointCtrlCmd cmd = joint_cmd(1);
    cmd.pos[0] = 2.0;
    auto cmd_result = safety.check_joint_cmd(state, cmd, 0.001);
    ASSERT_FALSE(cmd_result);
    EXPECT_EQ(cmd_result.error().code, SafetyErr::CMD_POS_LIMIT);
}

TEST(ContinuousJointSafety, CommandStepUsesExistingPositionRepresentation) {
    Safety safety;
    SafetyCfg cfg = safety_cfg(true);
    cfg.limits.max_acc[0] = 1000.0;
    ASSERT_TRUE(safety.configure(cfg));

    JointState state = joint_state(1);
    state.pos[0] = M_PI - 0.001;
    ASSERT_TRUE(safety.reset_cmd_history(state));

    JointCtrlCmd cmd = joint_cmd(1);
    cmd.pos[0] = -M_PI + 0.001;
    auto result = safety.check_joint_cmd(state, cmd, 0.001);
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error().code, SafetyErr::CMD_POS_STEP_LIMIT);
}

TEST(ModelLoaderLimitResolver, ContinuousJointResolvesWithoutPositionLimits) {
    const std::vector<std::string> names{ "joint1", "joint2", "joint3", "joint4" };
    ModelLoader loader;
    auto model = loader.load(fixture_path("simple_4dof_arm.urdf"), names);
    ASSERT_TRUE(model);
    EXPECT_TRUE(model->joint_limits[0].has_position_limit);
    EXPECT_FALSE(model->joint_limits[1].has_position_limit);

    JointActuatorMapCfg mapper;
    mapper.joints_count = names.size();
    mapper.pos_ratio.assign(names.size(), 1.0);
    mapper.tor_ratio.assign(names.size(), 1.0);
    mapper.direction.assign(names.size(), 1);
    mapper.joint_zero_offset.assign(names.size(), 0.0);
    mapper.actuator_zero_offset.assign(names.size(), 0.0);

    HardwareCapabilities caps;
    for(std::size_t i = 0; i < names.size(); ++i) {
        caps.push_back({ "actuator" + std::to_string(i + 1), 10.0, 10.0, 10.0, 100.0, 10.0 });
    }

    SafetyPolicyCfg policy;
    policy.position_margin = 0.05;
    policy.max_acc.assign(names.size(), 20.0);
    policy.max_dt_s = 0.01;
    policy.state_timeout_s = 0.05;
    policy.cmd_timeout_s = 0.1;

    LimitResolver resolver;
    auto resolved = resolver.resolve(*model, mapper, caps, policy);
    ASSERT_TRUE(resolved);
    EXPECT_FALSE(resolved->joints[1].has_position_limit);

    Safety safety;
    ASSERT_TRUE(safety.configure(to_safety_cfg(*resolved)));
}

TEST(DynamicsMandatory, PlaceholderInertialFixtureComputesFiniteOutputs) {
    const std::vector<std::string> names{ "joint1", "joint2", "joint3", "joint4" };
    ModelLoader loader;
    ASSERT_TRUE(loader.load(fixture_path("simple_4dof_revolute_arm.urdf"), names));

    Dynamics dynamics;
    DynamicsCfg cfg;
    cfg.urdf_path = fixture_path("simple_4dof_revolute_arm.urdf");
    cfg.joint_names = names;
    cfg.base_frame = "base_link";
    cfg.tool_frame = "tool0";
    cfg.gravity_scale.assign(names.size(), 1.0);
    ASSERT_TRUE(dynamics.configure(cfg));

    JointState state = joint_state(names.size());
    JointVector acc(names.size(), 0.0);
    JointVector ref_acc(names.size(), 0.0);
    ASSERT_TRUE(dynamics.update(state, acc, ref_acc));

    EXPECT_EQ(dynamics.get_info().joints_count, names.size());
    EXPECT_EQ(dynamics.get_mass_matrix().rows(), static_cast<int>(names.size()));
    EXPECT_EQ(dynamics.get_mass_matrix().cols(), static_cast<int>(names.size()));
    EXPECT_EQ(dynamics.get_tool_jacobian().rows(), 6);
    EXPECT_EQ(dynamics.get_tool_jacobian().cols(), static_cast<int>(names.size()));
    EXPECT_EQ(dynamics.get_gravity().size(), names.size());
    EXPECT_EQ(dynamics.get_inverse_dynamics().size(), names.size());
    EXPECT_TRUE(dynamics.get_mass_matrix().allFinite());
    EXPECT_TRUE(dynamics.get_tool_jacobian().allFinite());
}

TEST(MitBackendContract, MapperPassesFullCommandToMotorBus) {
    JointActuatorMapCfg mapper_cfg;
    mapper_cfg.joints_count = 1;
    mapper_cfg.pos_ratio = { 2.0 };
    mapper_cfg.tor_ratio = { 3.0 };
    mapper_cfg.direction = { 1 };
    mapper_cfg.joint_zero_offset = { 0.1 };
    mapper_cfg.actuator_zero_offset = { -0.2 };

    JointActuatorMapper mapper;
    ASSERT_TRUE(mapper.configure(mapper_cfg));

    JointCtrlCmd joint;
    joint.pos = { 1.0 };
    joint.vel = { 2.0 };
    joint.tor = { 3.0 };
    joint.kp = { 4.0 };
    joint.kd = { 5.0 };
    auto actuator = mapper.to_actuator_cmd(joint);
    ASSERT_TRUE(actuator);

    FakeMotorBus bus;
    ASSERT_TRUE(bus.write(*actuator));
    EXPECT_DOUBLE_EQ(bus.last_cmd.pos[0], 2.0 * (1.0 - 0.1) - 0.2);
    EXPECT_DOUBLE_EQ(bus.last_cmd.vel[0], 4.0);
    EXPECT_DOUBLE_EQ(bus.last_cmd.tor[0], 1.0);
    EXPECT_DOUBLE_EQ(bus.last_cmd.kp[0], 4.0 / 6.0);
    EXPECT_DOUBLE_EQ(bus.last_cmd.kd[0], 5.0 / 6.0);
}
