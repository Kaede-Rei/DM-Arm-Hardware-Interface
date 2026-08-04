#include "serial_arm/config/limit_resolver.hpp"
#include "serial_arm/config/config.hpp"
#include "serial_arm/config/robot_profile.hpp"
#include "serial_arm/core/joint_actuator_mapper.hpp"
#include "serial_arm/core/safety.hpp"
#include "serial_arm/dynamics/dynamics.hpp"
#include "serial_arm/hardware/motor_bus.hpp"
#include "serial_arm/model/model_loader.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <limits>
#include <fstream>
#include <filesystem>
#include <string>
#include <vector>

namespace {

using namespace serial_arm;
namespace fs = std::filesystem;

std::string fixture_path(const std::string& name) {
    return std::string(SERIAL_ARM_TEST_FIXTURE_DIR) + "/" + name;
}

void write_text(const fs::path& path, const std::string& text) {
    fs::create_directories(path.parent_path());
    std::ofstream stream(path);
    stream << text;
}

void write_test_package(const fs::path& root, const std::string& package) {
    write_text(root / package / "package.xml", "<package><name>" + package + "</name></package>\n");
}

std::string test_profile_yaml(const std::string& profile_name, const std::string& plugin, const std::string& core_package, const std::string& hardware_package) {
    return "profiles:\n  " + profile_name + ":\n    core:\n      package: " + core_package + "\n      config: config/core.yaml\n    hardware:\n      plugin: " + plugin + "\n      config_package: " + hardware_package + "\n      config: config/hardware.yaml\n";
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

RobotCfg robot_cfg_for_validation(bool continuous) {
    RobotCfg cfg;
    cfg.joint_names = { "joint1" };
    cfg.runtime.ctrl_frequency_hz = 100.0;
    cfg.runtime.joint_acc_filter_alpha = 0.2;
    cfg.shutdown.park_before_disable = true;
    cfg.shutdown.park_pos = { continuous ? 100.0 : 0.0 };
    cfg.shutdown.speed_scale = 0.2;
    cfg.shutdown.position_tolerance = 0.01;
    cfg.shutdown.velocity_tolerance = 0.01;
    cfg.shutdown.settle_time_s = 0.0;
    cfg.shutdown.relaxed_tolerance_ratio = 2.0;
    cfg.shutdown.timeout_s = 1.0;
    cfg.ctrller.joints_count = 1;
    cfg.ctrller.allow_full_cmd = true;
    cfg.ctrller.rigid_hold_gains = { { 1.0 }, { 0.1 } };
    cfg.ctrller.rigid_tracking_gains = { { 1.0 }, { 0.1 } };
    cfg.ctrller.compliant_hold_gains = { { 1.0 }, { 0.1 } };
    cfg.ctrller.compliant_drag_gains = { { 0.0 }, { 0.1 } };
    cfg.ctrller.compliant_tracking_gains = { { 1.0 }, { 0.1 } };
    cfg.mapper.joints_count = 1;
    cfg.mapper.pos_ratio = { 1.0 };
    cfg.mapper.tor_ratio = { 1.0 };
    cfg.mapper.direction = { 1 };
    cfg.mapper.joint_zero_offset = { 0.0 };
    cfg.mapper.actuator_zero_offset = { 0.0 };
    cfg.safety = safety_cfg(continuous);
    cfg.safety.max_dt_s = 0.02;
    cfg.dynamics.urdf_path = fixture_path("simple_4dof_revolute_arm.urdf");
    cfg.dynamics.joint_names = { "joint1" };
    cfg.dynamics.base_frame = "base_link";
    cfg.dynamics.tool_frame = "tool0";
    cfg.dynamics.gravity_scale = { 1.0 };
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

    HardwareCapabilities caps_{ { "actuator1", -10.0, 10.0, 10.0, 10.0, 100.0, 10.0 } };
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

TEST(ContinuousJointSafety, GainsAndTimeoutsStillApply) {
    Safety safety;
    ASSERT_TRUE(safety.configure(safety_cfg(true)));

    JointState state = joint_state(1);
    JointCtrlCmd cmd = joint_cmd(1);
    cmd.kp[0] = 21.0;
    auto kp_result = safety.check_joint_cmd(state, cmd, 0.001);
    ASSERT_FALSE(kp_result);
    EXPECT_EQ(kp_result.error().code, SafetyErr::CMD_KP_LIMIT);

    cmd = joint_cmd(1);
    cmd.kd[0] = 3.0;
    auto kd_result = safety.check_joint_cmd(state, cmd, 0.001);
    ASSERT_FALSE(kd_result);
    EXPECT_EQ(kd_result.error().code, SafetyErr::CMD_KD_LIMIT);

    auto state_timeout = safety.check_state(state, actuator_state(1), 1.0);
    ASSERT_FALSE(state_timeout);
    EXPECT_EQ(state_timeout.error().code, SafetyErr::STATE_TIMEOUT);

    auto cmd_timeout = safety.check_cmd_age(1.0);
    ASSERT_FALSE(cmd_timeout);
    EXPECT_EQ(cmd_timeout.error().code, SafetyErr::CMD_TIMEOUT);
}

TEST(ContinuousJointSafety, ParkPositionValidationRespectsPositionLimitFlag) {
    RobotCfg continuous = robot_cfg_for_validation(true);
    EXPECT_TRUE(validate_robot_core_cfg(continuous));

    RobotCfg revolute = robot_cfg_for_validation(false);
    revolute.shutdown.park_pos = { 100.0 };
    auto result = validate_robot_core_cfg(revolute);
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error().code, ConfigErr::INVALID_VALUE);
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
        caps.push_back({ "actuator" + std::to_string(i + 1), -10.0, 10.0, 10.0, 10.0, 100.0, 10.0 });
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

    JointState state = joint_state(names.size());
    ActuatorState actuators = actuator_state(names.size());
    EXPECT_TRUE(safety.check_state(state, actuators, 0.0));
    auto safe_cmd = safety.check_joint_cmd(state, joint_cmd(names.size()), 0.001);
    EXPECT_TRUE(safe_cmd) << static_cast<int>(safe_cmd.error().code) << " index=" << safe_cmd.error().index << " value=" << safe_cmd.error().value << " limit=" << safe_cmd.error().limit;

    JointActuatorMapper mapper_instance;
    ASSERT_TRUE(mapper_instance.configure(mapper));
    auto actuator_cmd = mapper_instance.to_actuator_cmd(joint_cmd(names.size()));
    ASSERT_TRUE(actuator_cmd);
    EXPECT_EQ(actuator_cmd->pos.size(), names.size());
    auto joint_back = mapper_instance.to_joint_state(actuators);
    ASSERT_TRUE(joint_back);
    EXPECT_EQ(joint_back->pos.size(), names.size());
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

TEST(RobotProfile, ResolvesCoreFieldsWithoutRosEnvironment) {
    unsetenv("AMENT_PREFIX_PATH");
    unsetenv("COLCON_PREFIX_PATH");
    unsetenv("SERIAL_ARM_RESOURCE_PATH");

    RobotProfileLoadOptions options;
    options.resource_paths = { std::string(SERIAL_ARM_TEST_REPO_ROOT) };
    auto profile = load_robot_profile_core("dm_arm_gray", options);
    ASSERT_TRUE(profile) << profile.error().message;
    EXPECT_EQ(profile->name, "dm_arm_gray");
    EXPECT_NE(profile->core_config_path.find("config/core/gray.yaml"), std::string::npos);
    EXPECT_NE(profile->hardware_plugin.find("serial_arm_hardware_damiao"), std::string::npos);
    EXPECT_NE(profile->hardware_config_path.find("config/hardware.yaml"), std::string::npos);
}

TEST(RobotProfile, MissingAndInvalidProfilesReportContext) {
    unsetenv("AMENT_PREFIX_PATH");
    unsetenv("COLCON_PREFIX_PATH");

    RobotProfileLoadOptions options;
    options.resource_paths = { std::string(SERIAL_ARM_TEST_REPO_ROOT) };
    auto missing = load_robot_profile_core("missing_profile", options);
    ASSERT_FALSE(missing);
    EXPECT_EQ(missing.error().code, RobotProfileErr::PROFILE_NOT_FOUND);
    EXPECT_NE(missing.error().message.find("missing_profile"), std::string::npos);

    const std::string invalid_file = std::string(SERIAL_ARM_TEST_TMP_DIR) + "/invalid_robot_profiles.yaml";
    {
        std::ofstream stream(invalid_file);
        stream << "profiles:\n  broken:\n    core:\n      package: dm_arm_description\n      config: config/core/gray.yaml\n";
    }
    RobotProfileLoadOptions invalid_options;
    invalid_options.profile_file = invalid_file;
    invalid_options.resource_paths = { std::string(SERIAL_ARM_TEST_REPO_ROOT) };
    auto invalid = load_robot_profile_core("broken", invalid_options);
    ASSERT_FALSE(invalid);
    EXPECT_EQ(invalid.error().code, RobotProfileErr::MISSING_FIELD);
    EXPECT_NE(invalid.error().message.find("broken"), std::string::npos);
}

TEST(RobotProfile, MultipleResourceRootsContinueUntilProfileMatches) {
    const fs::path root = fs::path(SERIAL_ARM_TEST_TMP_DIR) / "multi_resource_roots";
    fs::remove_all(root);
    const fs::path resource_a = root / "resource_a";
    const fs::path resource_b = root / "resource_b";

    write_test_package(resource_a, "robot_a_description");
    write_text(resource_a / "robot_a_description" / "config" / "core.yaml", "robot_a core\n");
    write_text(resource_a / "robot_a_description" / "config" / "hardware.yaml", "robot_a hardware\n");
    write_text(resource_a / "robot_profiles.yaml", test_profile_yaml("robot_a", "serial_arm_hardware_a", "robot_a_description", "robot_a_description"));

    write_test_package(resource_b, "robot_b_description");
    write_text(resource_b / "robot_b_description" / "config" / "core.yaml", "robot_b core\n");
    write_text(resource_b / "robot_b_description" / "config" / "hardware.yaml", "robot_b hardware\n");
    write_text(resource_b / "robot_profiles.yaml", test_profile_yaml("robot_b", "serial_arm_hardware_b", "robot_b_description", "robot_b_description"));

    RobotProfileLoadOptions options;
    options.resource_paths = { resource_a.string(), resource_b.string() };
    auto profile = load_robot_profile_core("robot_b", options);
    ASSERT_TRUE(profile) << profile.error().message;
    EXPECT_NE(profile->profile_file.find("resource_b"), std::string::npos);
    EXPECT_NE(profile->core_config_path.find("robot_b_description"), std::string::npos);

    auto missing = load_robot_profile_core("missing_robot", options);
    ASSERT_FALSE(missing);
    EXPECT_EQ(missing.error().code, RobotProfileErr::PROFILE_NOT_FOUND);
    EXPECT_NE(missing.error().message.find("missing_robot"), std::string::npos);
    EXPECT_NE(missing.error().message.find("resource_a"), std::string::npos);
    EXPECT_NE(missing.error().message.find("resource_b"), std::string::npos);
}

TEST(RobotProfile, ExplicitProfileFileDoesNotFallback) {
    const fs::path root = fs::path(SERIAL_ARM_TEST_TMP_DIR) / "explicit_profile_file";
    fs::remove_all(root);
    const fs::path explicit_root = root / "explicit";
    const fs::path fallback_root = root / "fallback";

    write_test_package(explicit_root, "explicit_description");
    write_text(explicit_root / "explicit_description" / "config" / "core.yaml", "explicit core\n");
    write_text(explicit_root / "explicit_description" / "config" / "hardware.yaml", "explicit hardware\n");
    write_text(explicit_root / "robot_profiles.yaml", test_profile_yaml("same_robot", "serial_arm_hardware_explicit", "explicit_description", "explicit_description"));

    write_test_package(fallback_root, "fallback_description");
    write_text(fallback_root / "fallback_description" / "config" / "core.yaml", "fallback core\n");
    write_text(fallback_root / "fallback_description" / "config" / "hardware.yaml", "fallback hardware\n");
    write_text(fallback_root / "robot_profiles.yaml",
        test_profile_yaml("same_robot", "serial_arm_hardware_fallback", "fallback_description", "fallback_description") +
        "  only_in_fallback:\n"
        "    core:\n"
        "      package: fallback_description\n"
        "      config: config/core.yaml\n"
        "    hardware:\n"
        "      plugin: serial_arm_hardware_fallback\n"
        "      config_package: fallback_description\n"
        "      config: config/hardware.yaml\n");

    RobotProfileLoadOptions options;
    options.profile_file = (explicit_root / "robot_profiles.yaml").string();
    options.resource_paths = { explicit_root.string(), fallback_root.string() };
    auto profile = load_robot_profile_core("same_robot", options);
    ASSERT_TRUE(profile) << profile.error().message;
    EXPECT_NE(profile->core_config_path.find("explicit_description"), std::string::npos);
    EXPECT_EQ(profile->hardware_plugin, "serial_arm_hardware_explicit");

    auto missing = load_robot_profile_core("only_in_fallback", options);
    ASSERT_FALSE(missing);
    EXPECT_EQ(missing.error().code, RobotProfileErr::PROFILE_NOT_FOUND);
    EXPECT_NE(missing.error().message.find("only_in_fallback"), std::string::npos);
}

TEST(RobotProfile, GenericBackendPluginResolutionUsesPluginName) {
    const fs::path root = fs::path(SERIAL_ARM_TEST_TMP_DIR) / "generic_backend_plugin";
    fs::remove_all(root);
    const std::string plugin = "serial_arm_hardware_test";
    write_test_package(root, "test_description");
    write_text(root / "test_description" / "config" / "core.yaml", "test core\n");
    write_text(root / "test_description" / "config" / "hardware.yaml", "test hardware\n");
    write_text(root / "robot_profiles.yaml", test_profile_yaml("test_robot", plugin, "test_description", "test_description"));
    write_text(root / "install" / plugin / "lib" / ("lib" + plugin + ".so"), "");

    RobotProfileLoadOptions options;
    options.resource_paths = { root.string() };
    auto profile = load_robot_profile_core("test_robot", options);
    ASSERT_TRUE(profile) << profile.error().message;
    EXPECT_NE(profile->hardware_plugin.find("libserial_arm_hardware_test.so"), std::string::npos);
    EXPECT_EQ(profile->hardware_plugin.find("serial_arm_hardware_damiao"), std::string::npos);
}

TEST(RobotProfile, StandaloneInstalledResourceLayoutResolvesWithoutRosEnvironment) {
    unsetenv("AMENT_PREFIX_PATH");
    unsetenv("COLCON_PREFIX_PATH");

    const fs::path prefix = fs::path(SERIAL_ARM_TEST_TMP_DIR) / "standalone_install_prefix";
    fs::remove_all(prefix);
    const fs::path profiles_share = prefix / "share" / "serial_arm_robot_profiles";
    const fs::path description_share = prefix / "share" / "test_robot_description";
    const std::string plugin = "serial_arm_hardware_test";

    write_test_package(prefix / "share", "test_robot_description");
    write_text(description_share / "config" / "core.yaml", "test core\n");
    write_text(description_share / "config" / "hardware.yaml", "test hardware\n");
    write_text(profiles_share / "config" / "robot_profiles.yaml", test_profile_yaml("test_robot", plugin, "test_robot_description", "test_robot_description"));
    write_text(prefix / "lib" / ("lib" + plugin + ".so"), "");

    RobotProfileLoadOptions options;
    options.resource_paths = { prefix.string() };
    auto profile = load_robot_profile_core("test_robot", options);
    ASSERT_TRUE(profile) << profile.error().message;
    EXPECT_NE(profile->profile_file.find("share/serial_arm_robot_profiles/config/robot_profiles.yaml"), std::string::npos);
    EXPECT_NE(profile->core_config_path.find("share/test_robot_description/config/core.yaml"), std::string::npos);
    EXPECT_NE(profile->hardware_config_path.find("share/test_robot_description/config/hardware.yaml"), std::string::npos);
    EXPECT_NE(profile->hardware_plugin.find("libserial_arm_hardware_test.so"), std::string::npos);
}
