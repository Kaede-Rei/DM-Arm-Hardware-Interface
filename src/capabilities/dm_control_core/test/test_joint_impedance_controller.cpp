#include "dm_control_core/joint_impedance_controller.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using dm_control_core::JointCommand;
using dm_control_core::JointCommandError;
using dm_control_core::JointCommandMode;
using dm_control_core::JointImpedanceController;
using dm_control_core::JointImpedanceControllerConfig;
using dm_control_core::JointImpedanceControllerInput;
using dm_control_core::JointImpedanceMode;
using dm_control_core::JointState;

JointImpedanceControllerConfig make_config(std::size_t n = 2) {
    JointImpedanceControllerConfig config;

    config.layout.joint_names.reserve(n);
    for(std::size_t i = 0; i < n; ++i) {
        config.layout.joint_names.push_back("joint" + std::to_string(i + 1));
    }

    config.rigid_hold_gains.kp.assign(n, 10.0);
    config.rigid_hold_gains.kd.assign(n, 1.0);
    config.compliant_hold_gains.kp.assign(n, 2.0);
    config.compliant_hold_gains.kd.assign(n, 0.2);
    config.tracking_gains.kp.assign(n, 20.0);
    config.tracking_gains.kd.assign(n, 2.0);

    config.limits.max_velocity.assign(n, 0.0);
    config.limits.max_effort.assign(n, 0.0);
    config.limits.min_kp.assign(n, std::numeric_limits<double>::lowest());
    config.limits.max_kp.assign(n, std::numeric_limits<double>::max());
    config.limits.min_kd.assign(n, std::numeric_limits<double>::lowest());
    config.limits.max_kd.assign(n, std::numeric_limits<double>::max());

    config.use_model_feedforward = true;
    config.use_command_effort = true;
    return config;
}

JointState make_state(
    const std::vector<double>& position = { 0.1, -0.2 },
    const std::vector<double>& velocity = { 0.3, -0.4 },
    const std::vector<double>& effort = { 0.5, -0.6 },
    const std::vector<double>& motor_effort = { 0.05, -0.06 }) {

    JointState state;
    state.position = position;
    state.velocity = velocity;
    state.effort = effort;
    state.motor_effort = motor_effort;
    return state;
}

JointImpedanceControllerInput make_input(
    const JointState& state,
    const std::vector<double>& model_feedforward = { 0.7, -0.8 }) {

    JointImpedanceControllerInput input;
    input.state = state;
    input.model_feedforward = model_feedforward;
    input.dt = 0.005;
    return input;
}

void expect_vector_near(
    const std::vector<double>& actual,
    const std::vector<double>& expected,
    double tolerance = 1e-12) {

    ASSERT_EQ(actual.size(), expected.size());
    for(std::size_t i = 0; i < expected.size(); ++i) {
        EXPECT_NEAR(actual[i], expected[i], tolerance) << "index " << i;
    }
}

}  // namespace

TEST(JointImpedanceControllerTest, RejectsInvalidConfigSizes) {
    JointImpedanceController controller;
    auto config = make_config();
    config.tracking_gains.kp.pop_back();

    EXPECT_THROW(controller.configure(config), std::runtime_error);
}

TEST(JointImpedanceControllerTest, ValidatesRequiredCommandFieldsAndSizes) {
    JointImpedanceController controller;
    controller.configure(make_config());

    JointCommand command;

    command.mode = JointCommandMode::POSITION;
    EXPECT_EQ(controller.set_command(command).error(), JointCommandError::MISSING_POSITION);
    command.position = std::vector<double>{ 1.0 };
    EXPECT_EQ(controller.set_command(command).error(), JointCommandError::INVALID_POSITION_SIZE);

    command = JointCommand{};
    command.mode = JointCommandMode::POSITION_VELOCITY;
    command.position = std::vector<double>{ 1.0, 2.0 };
    EXPECT_EQ(controller.set_command(command).error(), JointCommandError::MISSING_VELOCITY);
    command.velocity = std::vector<double>{ 0.1 };
    EXPECT_EQ(controller.set_command(command).error(), JointCommandError::INVALID_VELOCITY_SIZE);

    command = JointCommand{};
    command.mode = JointCommandMode::IMPEDANCE;
    command.position = std::vector<double>{ 1.0, 2.0 };
    command.velocity = std::vector<double>{ 0.1, 0.2 };
    EXPECT_EQ(controller.set_command(command).error(), JointCommandError::MISSING_EFFORT);
    command.effort = std::vector<double>{ 3.0 };
    EXPECT_EQ(controller.set_command(command).error(), JointCommandError::INVALID_EFFORT_SIZE);

    command = JointCommand{};
    command.mode = JointCommandMode::VELOCITY;
    EXPECT_EQ(controller.set_command(command).error(), JointCommandError::MISSING_VELOCITY);

    command = JointCommand{};
    command.mode = JointCommandMode::TORQUE;
    EXPECT_EQ(controller.set_command(command).error(), JointCommandError::MISSING_EFFORT);
}

TEST(JointImpedanceControllerTest, RigidHoldLatchesCurrentPositionAndUsesRigidGains) {
    JointImpedanceController controller;
    controller.configure(make_config());

    const auto hold_state = make_state({ 0.2, -0.3 });
    controller.reset(hold_state);

    const auto moving_state = make_state({ 1.0, 2.0 }, { 3.0, 4.0 });
    const auto output = controller.update(make_input(moving_state));

    ASSERT_TRUE(output.valid);
    expect_vector_near(output.command.position, { 0.2, -0.3 });
    expect_vector_near(output.command.velocity, { 0.0, 0.0 });
    expect_vector_near(output.command.effort, { 0.7, -0.8 });
    expect_vector_near(output.command.kp, { 10.0, 10.0 });
    expect_vector_near(output.command.kd, { 1.0, 1.0 });
}

TEST(JointImpedanceControllerTest, CompliantHoldLatchesWhenModeIsSelected) {
    JointImpedanceController controller;
    controller.configure(make_config());

    controller.set_mode(JointImpedanceMode::COMPLIANT_HOLD, make_state({ 0.4, -0.5 }));
    const auto output = controller.update(make_input(make_state({ 1.0, 2.0 })));

    ASSERT_TRUE(output.valid);
    expect_vector_near(output.command.position, { 0.4, -0.5 });
    expect_vector_near(output.command.velocity, { 0.0, 0.0 });
    expect_vector_near(output.command.kp, { 2.0, 2.0 });
    expect_vector_near(output.command.kd, { 0.2, 0.2 });
}

TEST(JointImpedanceControllerTest, TrackingPositionCommandUsesPositionAndZeroVelocity) {
    JointImpedanceController controller;
    controller.configure(make_config());
    controller.set_mode(JointImpedanceMode::TRACKING, make_state());

    JointCommand command;
    command.mode = JointCommandMode::POSITION;
    command.position = std::vector<double>{ 1.2, -1.3 };
    ASSERT_TRUE(controller.set_command(command));

    const auto output = controller.update(make_input(make_state()));

    ASSERT_TRUE(output.valid);
    expect_vector_near(output.command.position, { 1.2, -1.3 });
    expect_vector_near(output.command.velocity, { 0.0, 0.0 });
    expect_vector_near(output.command.effort, { 0.7, -0.8 });
    expect_vector_near(output.command.kp, { 20.0, 20.0 });
    expect_vector_near(output.command.kd, { 2.0, 2.0 });
}

TEST(JointImpedanceControllerTest, TrackingPositionVelocityCommandUsesVelocityReference) {
    JointImpedanceController controller;
    controller.configure(make_config());
    controller.set_mode(JointImpedanceMode::TRACKING, make_state());

    JointCommand command;
    command.mode = JointCommandMode::POSITION_VELOCITY;
    command.position = std::vector<double>{ 1.2, -1.3 };
    command.velocity = std::vector<double>{ 0.4, -0.5 };
    ASSERT_TRUE(controller.set_command(command));

    const auto output = controller.update(make_input(make_state()));

    ASSERT_TRUE(output.valid);
    expect_vector_near(output.command.position, { 1.2, -1.3 });
    expect_vector_near(output.command.velocity, { 0.4, -0.5 });
    expect_vector_near(output.command.effort, { 0.7, -0.8 });
}

TEST(JointImpedanceControllerTest, ImpedanceCommandAddsCommandEffortToModelFeedforward) {
    JointImpedanceController controller;
    controller.configure(make_config());
    controller.set_mode(JointImpedanceMode::TRACKING, make_state());

    JointCommand command;
    command.mode = JointCommandMode::IMPEDANCE;
    command.position = std::vector<double>{ 1.2, -1.3 };
    command.velocity = std::vector<double>{ 0.4, -0.5 };
    command.effort = std::vector<double>{ 2.0, -3.0 };
    ASSERT_TRUE(controller.set_command(command));

    const auto output = controller.update(make_input(make_state(), { 0.7, -0.8 }));

    ASSERT_TRUE(output.valid);
    expect_vector_near(output.command.position, { 1.2, -1.3 });
    expect_vector_near(output.command.velocity, { 0.4, -0.5 });
    expect_vector_near(output.command.effort, { 2.7, -3.8 });
}

TEST(JointImpedanceControllerTest, VelocityCommandTracksCurrentPositionAndDisablesKp) {
    JointImpedanceController controller;
    auto config = make_config();
    config.limits.max_velocity = { 0.5, 0.25 };
    controller.configure(config);
    controller.set_mode(JointImpedanceMode::TRACKING, make_state());

    JointCommand command;
    command.mode = JointCommandMode::VELOCITY;
    command.velocity = std::vector<double>{ 1.0, -1.0 };
    ASSERT_TRUE(controller.set_command(command));

    const auto output = controller.update(make_input(make_state({ 0.9, -0.8 })));

    ASSERT_TRUE(output.valid);
    expect_vector_near(output.command.position, { 0.9, -0.8 });
    expect_vector_near(output.command.velocity, { 0.5, -0.25 });
    expect_vector_near(output.command.effort, { 0.7, -0.8 });
    expect_vector_near(output.command.kp, { 0.0, 0.0 });
    expect_vector_near(output.command.kd, { 2.0, 2.0 });
}

TEST(JointImpedanceControllerTest, TorqueCommandDisablesGainsAndIgnoresModelFeedforward) {
    JointImpedanceController controller;
    auto config = make_config();
    config.limits.max_effort = { 1.5, 2.5 };
    controller.configure(config);
    controller.set_mode(JointImpedanceMode::TRACKING, make_state());

    JointCommand command;
    command.mode = JointCommandMode::TORQUE;
    command.effort = std::vector<double>{ 2.0, -3.0 };
    ASSERT_TRUE(controller.set_command(command));

    const auto output = controller.update(make_input(make_state({ 0.9, -0.8 }), { 100.0, -100.0 }));

    ASSERT_TRUE(output.valid);
    expect_vector_near(output.command.position, { 0.9, -0.8 });
    expect_vector_near(output.command.velocity, { 0.0, 0.0 });
    expect_vector_near(output.command.effort, { 1.5, -2.5 });
    expect_vector_near(output.command.kp, { 0.0, 0.0 });
    expect_vector_near(output.command.kd, { 0.0, 0.0 });
}

TEST(JointImpedanceControllerTest, ClampsConfiguredGainsAndEffort) {
    JointImpedanceController controller;
    auto config = make_config();
    config.tracking_gains.kp = { 100.0, -10.0 };
    config.tracking_gains.kd = { 50.0, -5.0 };
    config.limits.max_effort = { 1.0, 2.0 };
    config.limits.min_kp = { 0.0, 0.0 };
    config.limits.max_kp = { 30.0, 30.0 };
    config.limits.min_kd = { 0.0, 0.0 };
    config.limits.max_kd = { 3.0, 3.0 };
    controller.configure(config);
    controller.set_mode(JointImpedanceMode::TRACKING, make_state());

    JointCommand command;
    command.mode = JointCommandMode::IMPEDANCE;
    command.position = std::vector<double>{ 1.0, 2.0 };
    command.velocity = std::vector<double>{ 0.0, 0.0 };
    command.effort = std::vector<double>{ 10.0, -10.0 };
    ASSERT_TRUE(controller.set_command(command));

    const auto output = controller.update(make_input(make_state(), { 0.0, 0.0 }));

    ASSERT_TRUE(output.valid);
    expect_vector_near(output.command.effort, { 1.0, -2.0 });
    expect_vector_near(output.command.kp, { 30.0, 0.0 });
    expect_vector_near(output.command.kd, { 3.0, 0.0 });
}

TEST(JointImpedanceControllerTest, SanitizesNonFiniteCommandAndFeedforwardValues) {
    JointImpedanceController controller;
    controller.configure(make_config());
    controller.set_mode(JointImpedanceMode::TRACKING, make_state());

    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    JointCommand command;
    command.mode = JointCommandMode::IMPEDANCE;
    command.position = std::vector<double>{ nan, 1.2 };
    command.velocity = std::vector<double>{ inf, -inf };
    command.effort = std::vector<double>{ inf, -inf };
    ASSERT_TRUE(controller.set_command(command));

    const auto state = make_state({ 0.4, -0.5 });
    const auto output = controller.update(make_input(state, { nan, inf }));

    ASSERT_TRUE(output.valid);
    expect_vector_near(output.command.position, { 0.4, 1.2 });
    expect_vector_near(output.command.velocity, { 0.0, 0.0 });
    expect_vector_near(output.command.effort, { 0.0, 0.0 });
}

TEST(JointImpedanceControllerTest, ReturnsInvalidOutputForMismatchedStateSizes) {
    JointImpedanceController controller;
    controller.configure(make_config());

    auto input = make_input(make_state());
    input.state.velocity.pop_back();

    const auto output = controller.update(input);
    EXPECT_FALSE(output.valid);
}
