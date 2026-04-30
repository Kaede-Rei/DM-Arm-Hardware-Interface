#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "dm_control_core/dynamics_observer.hpp"
#include "dm_control_core/joint_impedance_controller.hpp"
#include "dm_control_core/joint_control_types.hpp"

#include <stdexcept>
#include <string>
#include <vector>

namespace py = pybind11;

namespace {

using dm_control_core::DynamicsObservation;
using dm_control_core::DynamicsObserver;
using dm_control_core::JointCommand;
using dm_control_core::JointCommandError;
using dm_control_core::JointCommandLimits;
using dm_control_core::JointCommandMode;
using dm_control_core::JointControlLayout;
using dm_control_core::JointImpedanceController;
using dm_control_core::JointImpedanceControllerConfig;
using dm_control_core::JointImpedanceControllerInput;
using dm_control_core::JointImpedanceGains;
using dm_control_core::JointImpedanceMode;
using dm_control_core::JointState;
using dm_control_core::MitJointCommand;

const char* command_error_to_string(JointCommandError error) {
    switch(error) {
        case JointCommandError::MISSING_POSITION:
            return "MISSING_POSITION";
        case JointCommandError::MISSING_VELOCITY:
            return "MISSING_VELOCITY";
        case JointCommandError::MISSING_EFFORT:
            return "MISSING_EFFORT";
        case JointCommandError::INVALID_POSITION_SIZE:
            return "INVALID_POSITION_SIZE";
        case JointCommandError::INVALID_VELOCITY_SIZE:
            return "INVALID_VELOCITY_SIZE";
        case JointCommandError::INVALID_EFFORT_SIZE:
            return "INVALID_EFFORT_SIZE";
        case JointCommandError::INVALID_STATE_SIZE:
            return "INVALID_STATE_SIZE";
    }
    return "UNKNOWN_COMMAND_ERROR";
}

JointState make_joint_state(
    const std::vector<double>& position,
    const std::vector<double>& velocity,
    const std::vector<double>& effort,
    const std::vector<double>& motor_effort) {

    JointState state;
    state.position = position;
    state.velocity = velocity;
    state.effort = effort;
    state.motor_effort = motor_effort;
    return state;
}

std::vector<double> zeros_like_joint_names(const std::vector<std::string>& joint_names) {
    return std::vector<double>(joint_names.size(), 0.0);
}

void ensure_size(
    const std::vector<double>& value,
    std::size_t expected_size,
    const std::string& name) {

    if(value.size() != expected_size) {
        throw std::runtime_error(
            name + " size mismatch: expected " + std::to_string(expected_size)
            + ", got " + std::to_string(value.size()));
    }
}

py::dict mit_command_to_dict(const MitJointCommand& command) {
    py::dict result;
    result["position"] = command.position;
    result["velocity"] = command.velocity;
    result["effort"] = command.effort;
    result["kp"] = command.kp;
    result["kd"] = command.kd;
    return result;
}

py::dict observation_to_dict(const DynamicsObservation& observation) {
    py::dict result;
    result["valid"] = observation.valid;
    result["gravity"] = observation.gravity;
    result["nonlinear"] = observation.nonlinear;
    result["active_feedforward"] = observation.active_feedforward;
    result["external_effort"] = observation.external_effort;
    return result;
}

py::dict output_to_dict(const dm_control_core::JointImpedanceControllerOutput& output) {
    py::dict result;
    result["valid"] = output.valid;
    result["command"] = mit_command_to_dict(output.command);
    return result;
}

class DmControlRuntime {
public:
    DmControlRuntime() = default;

    void configure_dynamics(
        const std::string& urdf_path,
        const std::vector<std::string>& joint_names) {

        if(joint_names.empty()) {
            throw std::runtime_error("configure_dynamics: joint_names is empty");
        }

        joint_names_ = joint_names;
        observer_.configure(urdf_path, joint_names);
        dynamics_configured_ = true;
    }

    void configure_controller(
        const std::vector<std::string>& joint_names,
        const std::vector<double>& rigid_kp,
        const std::vector<double>& rigid_kd,
        const std::vector<double>& compliant_kp,
        const std::vector<double>& compliant_kd,
        const std::vector<double>& tracking_kp,
        const std::vector<double>& tracking_kd,
        const std::vector<double>& max_velocity,
        const std::vector<double>& max_effort,
        const std::vector<double>& min_kp,
        const std::vector<double>& max_kp,
        const std::vector<double>& min_kd,
        const std::vector<double>& max_kd,
        bool use_model_feedforward,
        bool use_command_effort) {

        if(joint_names.empty()) {
            throw std::runtime_error("configure_controller: joint_names is empty");
        }

        const std::size_t n = joint_names.size();

        ensure_size(rigid_kp, n, "rigid_kp");
        ensure_size(rigid_kd, n, "rigid_kd");
        ensure_size(compliant_kp, n, "compliant_kp");
        ensure_size(compliant_kd, n, "compliant_kd");
        ensure_size(tracking_kp, n, "tracking_kp");
        ensure_size(tracking_kd, n, "tracking_kd");
        ensure_size(max_velocity, n, "max_velocity");
        ensure_size(max_effort, n, "max_effort");
        ensure_size(min_kp, n, "min_kp");
        ensure_size(max_kp, n, "max_kp");
        ensure_size(min_kd, n, "min_kd");
        ensure_size(max_kd, n, "max_kd");

        joint_names_ = joint_names;

        JointImpedanceControllerConfig config;
        config.layout.joint_names = joint_names;

        config.rigid_hold_gains.kp = rigid_kp;
        config.rigid_hold_gains.kd = rigid_kd;

        config.compliant_hold_gains.kp = compliant_kp;
        config.compliant_hold_gains.kd = compliant_kd;

        config.tracking_gains.kp = tracking_kp;
        config.tracking_gains.kd = tracking_kd;

        config.limits.max_velocity = max_velocity;
        config.limits.max_effort = max_effort;
        config.limits.min_kp = min_kp;
        config.limits.max_kp = max_kp;
        config.limits.min_kd = min_kd;
        config.limits.max_kd = max_kd;

        config.use_model_feedforward = use_model_feedforward;
        config.use_command_effort = use_command_effort;

        controller_.configure(config);
        controller_configured_ = true;
    }

    void reset(
        const std::vector<double>& position,
        const std::vector<double>& velocity,
        const std::vector<double>& effort,
        const std::vector<double>& motor_effort) {

        require_controller();

        const auto state = checked_state(position, velocity, effort, motor_effort);
        controller_.reset(state);
    }

    void set_mode(
        JointImpedanceMode mode,
        const std::vector<double>& position,
        const std::vector<double>& velocity,
        const std::vector<double>& effort,
        const std::vector<double>& motor_effort) {

        require_controller();

        const auto state = checked_state(position, velocity, effort, motor_effort);
        controller_.set_mode(mode, state);
    }

    py::dict observe(
        const std::vector<double>& position,
        const std::vector<double>& velocity,
        const std::vector<double>& effort,
        bool enable_gravity_feedforward,
        bool enable_nonlinear_feedforward) {

        require_dynamics();

        const std::size_t n = joint_names_.size();
        ensure_size(position, n, "position");
        ensure_size(velocity, n, "velocity");
        ensure_size(effort, n, "effort");

        DynamicsObservation observation;
        const bool ok = observer_.observe(
            position,
            velocity,
            effort,
            enable_gravity_feedforward,
            enable_nonlinear_feedforward,
            observation);

        if(!ok) {
            throw std::runtime_error("DynamicsObserver::observe failed");
        }

        return observation_to_dict(observation);
    }

    py::dict compute_position_command(
        const std::vector<double>& position,
        const std::vector<double>& velocity,
        const std::vector<double>& effort,
        const std::vector<double>& motor_effort,
        const std::vector<double>& q_ref,
        const std::vector<double>& model_feedforward,
        double dt) {

        require_controller();

        const std::size_t n = joint_names_.size();
        ensure_size(q_ref, n, "q_ref");
        ensure_size(model_feedforward, n, "model_feedforward");

        const auto state = checked_state(position, velocity, effort, motor_effort);

        JointCommand command;
        command.mode = JointCommandMode::POSITION;
        command.position = q_ref;

        apply_command(command);

        JointImpedanceControllerInput input;
        input.state = state;
        input.model_feedforward = model_feedforward;
        input.dt = dt;

        const auto output = controller_.update(input);
        return output_to_dict(output);
    }

    py::dict compute_position_velocity_command(
        const std::vector<double>& position,
        const std::vector<double>& velocity,
        const std::vector<double>& effort,
        const std::vector<double>& motor_effort,
        const std::vector<double>& q_ref,
        const std::vector<double>& dq_ref,
        const std::vector<double>& model_feedforward,
        double dt) {

        require_controller();

        const std::size_t n = joint_names_.size();
        ensure_size(q_ref, n, "q_ref");
        ensure_size(dq_ref, n, "dq_ref");
        ensure_size(model_feedforward, n, "model_feedforward");

        const auto state = checked_state(position, velocity, effort, motor_effort);

        JointCommand command;
        command.mode = JointCommandMode::POSITION_VELOCITY;
        command.position = q_ref;
        command.velocity = dq_ref;

        apply_command(command);

        JointImpedanceControllerInput input;
        input.state = state;
        input.model_feedforward = model_feedforward;
        input.dt = dt;

        const auto output = controller_.update(input);
        return output_to_dict(output);
    }

    py::dict compute_impedance_command(
        const std::vector<double>& position,
        const std::vector<double>& velocity,
        const std::vector<double>& effort,
        const std::vector<double>& motor_effort,
        const std::vector<double>& q_ref,
        const std::vector<double>& dq_ref,
        const std::vector<double>& residual_effort,
        const std::vector<double>& model_feedforward,
        double dt) {

        require_controller();

        const std::size_t n = joint_names_.size();
        ensure_size(q_ref, n, "q_ref");
        ensure_size(dq_ref, n, "dq_ref");
        ensure_size(residual_effort, n, "residual_effort");
        ensure_size(model_feedforward, n, "model_feedforward");

        const auto state = checked_state(position, velocity, effort, motor_effort);

        JointCommand command;
        command.mode = JointCommandMode::IMPEDANCE;
        command.position = q_ref;
        command.velocity = dq_ref;
        command.effort = residual_effort;

        apply_command(command);

        JointImpedanceControllerInput input;
        input.state = state;
        input.model_feedforward = model_feedforward;
        input.dt = dt;

        const auto output = controller_.update(input);
        return output_to_dict(output);
    }

    py::dict compute_with_dynamics_position_command(
        const std::vector<double>& position,
        const std::vector<double>& velocity,
        const std::vector<double>& effort,
        const std::vector<double>& motor_effort,
        const std::vector<double>& q_ref,
        double dt,
        bool enable_gravity_feedforward,
        bool enable_nonlinear_feedforward) {

        require_controller();
        require_dynamics();

        const std::size_t n = joint_names_.size();
        ensure_size(q_ref, n, "q_ref");

        DynamicsObservation observation;
        const bool ok = observer_.observe(
            position,
            velocity,
            effort,
            enable_gravity_feedforward,
            enable_nonlinear_feedforward,
            observation);

        if(!ok) {
            throw std::runtime_error("DynamicsObserver::observe failed");
        }

        py::dict command_result = compute_position_command(
            position,
            velocity,
            effort,
            motor_effort,
            q_ref,
            observation.active_feedforward,
            dt);

        py::dict result;
        result["observation"] = observation_to_dict(observation);
        result["control"] = command_result;
        return result;
    }

private:
    void require_controller() const {
        if(!controller_configured_) {
            throw std::runtime_error("DmControlRuntime: controller is not configured");
        }
    }

    void require_dynamics() const {
        if(!dynamics_configured_) {
            throw std::runtime_error("DmControlRuntime: dynamics observer is not configured");
        }
    }

    JointState checked_state(
        const std::vector<double>& position,
        const std::vector<double>& velocity,
        const std::vector<double>& effort,
        const std::vector<double>& motor_effort) const {

        const std::size_t n = joint_names_.size();

        ensure_size(position, n, "position");
        ensure_size(velocity, n, "velocity");
        ensure_size(effort, n, "effort");
        ensure_size(motor_effort, n, "motor_effort");

        return make_joint_state(position, velocity, effort, motor_effort);
    }

    void apply_command(const JointCommand& command) {
        auto result = controller_.set_command(command);
        if(!result) {
            throw std::runtime_error(
                std::string("JointImpedanceController::set_command failed: ")
                + command_error_to_string(result.error()));
        }
    }

private:
    std::vector<std::string> joint_names_;

    DynamicsObserver observer_;
    JointImpedanceController controller_;

    bool dynamics_configured_{ false };
    bool controller_configured_{ false };
};

}  // namespace

PYBIND11_MODULE(dm_impedance, m) {
    m.doc() = "Python binding for dm_control_core";
    m.attr("__version__") = VERSION_INFO;

    py::enum_<JointImpedanceMode>(m, "JointImpedanceMode")
        .value("RIGID_HOLD", JointImpedanceMode::RIGID_HOLD)
        .value("COMPLIANT_HOLD", JointImpedanceMode::COMPLIANT_HOLD)
        .value("TRACKING", JointImpedanceMode::TRACKING)
        .export_values();

    py::enum_<JointCommandMode>(m, "JointCommandMode")
        .value("HOLD", JointCommandMode::HOLD)
        .value("POSITION", JointCommandMode::POSITION)
        .value("POSITION_VELOCITY", JointCommandMode::POSITION_VELOCITY)
        .value("IMPEDANCE", JointCommandMode::IMPEDANCE)
        .value("VELOCITY", JointCommandMode::VELOCITY)
        .value("TORQUE", JointCommandMode::TORQUE)
        .export_values();

    py::enum_<JointCommandError>(m, "JointCommandError")
        .value("MISSING_POSITION", JointCommandError::MISSING_POSITION)
        .value("MISSING_VELOCITY", JointCommandError::MISSING_VELOCITY)
        .value("MISSING_EFFORT", JointCommandError::MISSING_EFFORT)
        .value("INVALID_POSITION_SIZE", JointCommandError::INVALID_POSITION_SIZE)
        .value("INVALID_VELOCITY_SIZE", JointCommandError::INVALID_VELOCITY_SIZE)
        .value("INVALID_EFFORT_SIZE", JointCommandError::INVALID_EFFORT_SIZE)
        .value("INVALID_STATE_SIZE", JointCommandError::INVALID_STATE_SIZE)
        .export_values();

    py::class_<JointState>(m, "JointState")
        .def(py::init<>())
        .def_readwrite("position", &JointState::position)
        .def_readwrite("velocity", &JointState::velocity)
        .def_readwrite("effort", &JointState::effort)
        .def_readwrite("motor_effort", &JointState::motor_effort);

    py::class_<MitJointCommand>(m, "MitJointCommand")
        .def(py::init<>())
        .def_readwrite("position", &MitJointCommand::position)
        .def_readwrite("velocity", &MitJointCommand::velocity)
        .def_readwrite("effort", &MitJointCommand::effort)
        .def_readwrite("kp", &MitJointCommand::kp)
        .def_readwrite("kd", &MitJointCommand::kd);

    py::class_<DynamicsObservation>(m, "DynamicsObservation")
        .def(py::init<>())
        .def_readwrite("valid", &DynamicsObservation::valid)
        .def_readwrite("gravity", &DynamicsObservation::gravity)
        .def_readwrite("nonlinear", &DynamicsObservation::nonlinear)
        .def_readwrite("active_feedforward", &DynamicsObservation::active_feedforward)
        .def_readwrite("external_effort", &DynamicsObservation::external_effort);

    py::class_<DmControlRuntime>(m, "DmControlRuntime")
        .def(py::init<>())

        .def("configure_dynamics",
            &DmControlRuntime::configure_dynamics,
            py::arg("urdf_path"),
            py::arg("joint_names"))

        .def("configure_controller",
            &DmControlRuntime::configure_controller,
            py::arg("joint_names"),
            py::arg("rigid_kp"),
            py::arg("rigid_kd"),
            py::arg("compliant_kp"),
            py::arg("compliant_kd"),
            py::arg("tracking_kp"),
            py::arg("tracking_kd"),
            py::arg("max_velocity"),
            py::arg("max_effort"),
            py::arg("min_kp"),
            py::arg("max_kp"),
            py::arg("min_kd"),
            py::arg("max_kd"),
            py::arg("use_model_feedforward") = true,
            py::arg("use_command_effort") = true)

        .def("reset",
            &DmControlRuntime::reset,
            py::arg("position"),
            py::arg("velocity"),
            py::arg("effort"),
            py::arg("motor_effort"))

        .def("set_mode",
            &DmControlRuntime::set_mode,
            py::arg("mode"),
            py::arg("position"),
            py::arg("velocity"),
            py::arg("effort"),
            py::arg("motor_effort"))

        .def("observe",
            &DmControlRuntime::observe,
            py::arg("position"),
            py::arg("velocity"),
            py::arg("effort"),
            py::arg("enable_gravity_feedforward") = true,
            py::arg("enable_nonlinear_feedforward") = false)

        .def("compute_position_command",
            &DmControlRuntime::compute_position_command,
            py::arg("position"),
            py::arg("velocity"),
            py::arg("effort"),
            py::arg("motor_effort"),
            py::arg("q_ref"),
            py::arg("model_feedforward"),
            py::arg("dt"))

        .def("compute_position_velocity_command",
            &DmControlRuntime::compute_position_velocity_command,
            py::arg("position"),
            py::arg("velocity"),
            py::arg("effort"),
            py::arg("motor_effort"),
            py::arg("q_ref"),
            py::arg("dq_ref"),
            py::arg("model_feedforward"),
            py::arg("dt"))

        .def("compute_impedance_command",
            &DmControlRuntime::compute_impedance_command,
            py::arg("position"),
            py::arg("velocity"),
            py::arg("effort"),
            py::arg("motor_effort"),
            py::arg("q_ref"),
            py::arg("dq_ref"),
            py::arg("residual_effort"),
            py::arg("model_feedforward"),
            py::arg("dt"))

        .def("compute_with_dynamics_position_command",
            &DmControlRuntime::compute_with_dynamics_position_command,
            py::arg("position"),
            py::arg("velocity"),
            py::arg("effort"),
            py::arg("motor_effort"),
            py::arg("q_ref"),
            py::arg("dt"),
            py::arg("enable_gravity_feedforward") = true,
            py::arg("enable_nonlinear_feedforward") = false);
}