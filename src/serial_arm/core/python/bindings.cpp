#include "robot_session.hpp"

#include "serial_arm/config/config.hpp"
#include "serial_arm/core/joint_actuator_mapper.hpp"
#include "serial_arm/core/joints_ctrller.hpp"
#include "serial_arm/core/safety.hpp"
#include "serial_arm/dynamics/dynamics.hpp"
#include "serial_arm/hardware/hardware_loader.hpp"

#include <cmath>
#include <cstring>
#include <string>
#include <utility>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

namespace serial_arm {

namespace python_binding {

using DoubleArray = py::array_t<double, py::array::c_style | py::array::forcecast>;
using Uint8Array = py::array_t<std::uint8_t, py::array::c_style | py::array::forcecast>;
using IntArray = py::array_t<int, py::array::c_style | py::array::forcecast>;

// ! ========================= NumPy 转 换 方 法 实 现 ========================= ! //

/**
 * @brief 将 std::vector 复制为独立 NumPy 一维数组
 * @param values C++ 向量
 * @return 不共享 C++ 内存的 NumPy 数组
 */
template<typename T>
py::array_t<T> vector_to_numpy(const std::vector<T>& values) {
    py::array_t<T> result(values.size());
    if(!values.empty()) {
        std::memcpy(result.mutable_data(), values.data(), values.size() * sizeof(T));
    }
    return result;
}

/**
 * @brief 将 NumPy 一维数组转换为有限 JointVector
 * @param array NumPy 输入数组
 * @param name 参数名称
 * @return 转换后的 JointVector
 */
JointVector numpy_to_joint_vector(const DoubleArray& array, const char* name) {
    if(array.ndim() != 1) {
        throw py::value_error(std::string(name) + " must be a one-dimensional float64 array");
    }

    const std::size_t size = static_cast<std::size_t>(array.shape(0));
    const double* data = array.data();
    JointVector result(data, data + size);
    for(double value : result) {
        if(!std::isfinite(value)) {
            throw py::value_error(std::string(name) + " contains NaN or Inf");
        }
    }
    return result;
}

/**
 * @brief 将 NumPy 一维数组转换为指定长度的有限 JointVector
 * @param array NumPy 输入数组
 * @param expected_size 期望元素数量
 * @param name 参数名称
 * @return 转换后的 JointVector
 */
JointVector numpy_to_joint_vector(const DoubleArray& array, std::size_t expected_size, const char* name) {
    JointVector result = numpy_to_joint_vector(array, name);
    if(result.size() != expected_size) {
        throw py::value_error(std::string(name) + " must have shape (" + std::to_string(expected_size) + ",)");
    }
    return result;
}

std::vector<std::uint8_t> numpy_to_uint8_vector(const Uint8Array& array, const char* name) {
    if(array.ndim() != 1) {
        throw py::value_error(std::string(name) + " must be a one-dimensional uint8 array");
    }
    const std::uint8_t* data = array.data();
    return std::vector<std::uint8_t>(data, data + array.shape(0));
}

std::vector<int> numpy_to_int_vector(const IntArray& array, const char* name) {
    if(array.ndim() != 1) {
        throw py::value_error(std::string(name) + " must be a one-dimensional int array");
    }
    const int* data = array.data();
    return std::vector<int>(data, data + array.shape(0));
}

template<typename Owner, std::vector<double> Owner::* Member>
py::array_t<double> get_double_member(const Owner& self) {
    return vector_to_numpy(self.*Member);
}

template<typename Owner, std::vector<double> Owner::* Member>
void set_double_member(Owner& self, const DoubleArray& values) {
    self.*Member = numpy_to_joint_vector(values, "values");
}

template<typename Owner, std::vector<std::uint8_t> Owner::* Member>
py::array_t<std::uint8_t> get_uint8_member(const Owner& self) {
    return vector_to_numpy(self.*Member);
}

template<typename Owner, std::vector<std::uint8_t> Owner::* Member>
void set_uint8_member(Owner& self, const Uint8Array& values) {
    self.*Member = numpy_to_uint8_vector(values, "values");
}

template<typename Owner, std::vector<int> Owner::* Member>
py::array_t<int> get_int_member(const Owner& self) {
    return vector_to_numpy(self.*Member);
}

template<typename Owner, std::vector<int> Owner::* Member>
void set_int_member(Owner& self, const IntArray& values) {
    self.*Member = numpy_to_int_vector(values, "values");
}

/**
 * @brief 将 Eigen 动态矩阵复制为独立 NumPy 二维数组
 * @param matrix Eigen 矩阵
 * @return 不共享 C++ 内存的 NumPy 数组
 */
py::array_t<double> matrix_to_numpy(const Eigen::MatrixXd& matrix) {
    py::array_t<double> result({ matrix.rows(), matrix.cols() });
    auto view = result.mutable_unchecked<2>();
    for(Eigen::Index row = 0; row < matrix.rows(); ++row) {
        for(Eigen::Index col = 0; col < matrix.cols(); ++col) {
            view(row, col) = matrix(row, col);
        }
    }
    return result;
}

/**
 * @brief 将 Eigen 位姿复制为 4×4 NumPy 齐次变换矩阵
 * @param pose Eigen 位姿
 * @return 4×4 NumPy 数组
 */
py::array_t<double> pose_to_numpy(const Eigen::Isometry3d& pose) {
    const Eigen::Matrix4d matrix = pose.matrix();
    py::array_t<double> result({ 4, 4 });
    auto view = result.mutable_unchecked<2>();
    for(Eigen::Index row = 0; row < 4; ++row) {
        for(Eigen::Index col = 0; col < 4; ++col) {
            view(row, col) = matrix(row, col);
        }
    }
    return result;
}

// ! ========================= 错 误 转 换 方 法 实 现 ========================= ! //

template<typename Error>
std::string enum_error_message(const char* name, Error error) {
    return std::string(name) + "=" + std::to_string(static_cast<int>(error));
}

template<typename Error, typename MessageFn>
void unwrap_void(tl::expected<void, Error>&& result, MessageFn message_fn) {
    if(!result) {
        throw SerialArmPythonError(message_fn(result.error()));
    }
}

template<typename Value, typename Error, typename MessageFn>
Value unwrap_value(tl::expected<Value, Error>&& result, MessageFn message_fn) {
    if(!result) {
        throw SerialArmPythonError(message_fn(result.error()));
    }
    return std::move(result.value());
}

std::string safety_error_message(const SafetyFault& fault) {
    std::string message = "SafetyErr=" + std::to_string(static_cast<int>(fault.code));
    message += "; index=" + std::to_string(fault.index);
    message += "; value=" + std::to_string(fault.value);
    message += "; limit=" + std::to_string(fault.limit);
    return message;
}

void configure_joint_ctrller(JointCtrller& self, const JointCtrllerCfg& cfg) {
    unwrap_void(self.configure(cfg), [](JointCtrllerErr error) { return enum_error_message("JointCtrllerErr", error); });
}

void initialize_joint_ctrller(JointCtrller& self, const JointState& state) {
    unwrap_void(self.initialize(state), [](JointCtrllerErr error) { return enum_error_message("JointCtrllerErr", error); });
}

void set_joint_ctrller_mode(JointCtrller& self, JointImpedanceMode mode, const JointState& state) {
    unwrap_void(self.set_impedance_mode(mode, state), [](JointCtrllerErr error) { return enum_error_message("JointCtrllerErr", error); });
}

void set_joint_pos_cmd(JointCtrller& self, const DoubleArray& pos) {
    unwrap_void(self.set_cmd(JointPosCmd{ numpy_to_joint_vector(pos, "pos") }), [](JointCtrllerErr error) { return enum_error_message("JointCtrllerErr", error); });
}

void set_joint_pos_vel_cmd(JointCtrller& self, const DoubleArray& pos, const DoubleArray& vel) {
    JointPosVelCmd cmd{ numpy_to_joint_vector(pos, "pos"), numpy_to_joint_vector(vel, "vel") };
    unwrap_void(self.set_cmd(cmd), [](JointCtrllerErr error) { return enum_error_message("JointCtrllerErr", error); });
}

void set_joint_pos_vel_tor_cmd(JointCtrller& self, const DoubleArray& pos, const DoubleArray& vel, const DoubleArray& tor) {
    JointPosVelTorCmd cmd{ numpy_to_joint_vector(pos, "pos"), numpy_to_joint_vector(vel, "vel"), numpy_to_joint_vector(tor, "tor") };
    unwrap_void(self.set_cmd(cmd), [](JointCtrllerErr error) { return enum_error_message("JointCtrllerErr", error); });
}

void set_joint_full_cmd(JointCtrller& self, const JointCtrlCmd& cmd) {
    unwrap_void(self.set_full_cmd(cmd), [](JointCtrllerErr error) { return enum_error_message("JointCtrllerErr", error); });
}

JointCtrlCmd update_joint_ctrller(JointCtrller& self, const JointState& state, const DoubleArray& model_feedforward, double dt) {
    JointCtrllerInput input;
    input.state = state;
    input.model_feedforward = numpy_to_joint_vector(model_feedforward, state.pos.size(), "model_feedforward");
    input.dt = dt;
    return unwrap_value(self.update(input), [](JointCtrllerErr error) { return enum_error_message("JointCtrllerErr", error); }).cmd;
}

void configure_mapper(JointActuatorMapper& self, const JointActuatorMapCfg& cfg) {
    unwrap_void(self.configure(cfg), [](JointActuatorMapErr error) { return enum_error_message("JointActuatorMapErr", error); });
}

ActuatorCtrlCmd mapper_to_actuator_cmd(const JointActuatorMapper& self, const JointCtrlCmd& cmd) {
    return unwrap_value(self.to_actuator_cmd(cmd), [](JointActuatorMapErr error) { return enum_error_message("JointActuatorMapErr", error); });
}

JointState mapper_to_joint_state(const JointActuatorMapper& self, const ActuatorState& state) {
    return unwrap_value(self.to_joint_state(state), [](JointActuatorMapErr error) { return enum_error_message("JointActuatorMapErr", error); });
}

void safety_check_state(const Safety& self, const JointState& joint_state, const ActuatorState& actuator_state, double state_age_s) {
    unwrap_void(self.check_state(joint_state, actuator_state, state_age_s), safety_error_message);
}

JointCtrlCmd safety_check_joint_cmd(Safety& self, const JointState& state, const JointCtrlCmd& cmd, double dt) {
    return unwrap_value(self.check_joint_cmd(state, cmd, dt), safety_error_message);
}

void dynamics_set_gravity_scale(Dynamics& self, const DoubleArray& gravity_scale) {
    const JointVector values = numpy_to_joint_vector(gravity_scale, "gravity_scale");
    unwrap_void(self.set_gravity_scale(values), [](DynamicsErr error) { return enum_error_message("DynamicsErr", error); });
}

py::array_t<double> dynamics_frame_pose(const Dynamics& self, const std::string& frame_name) {
    const auto pose = unwrap_value(self.get_frame_pose(frame_name), [](DynamicsErr error) { return enum_error_message("DynamicsErr", error); });
    return pose_to_numpy(pose);
}

py::array_t<double> dynamics_frame_jacobian(const Dynamics& self, const std::string& frame_name) {
    const auto jacobian = unwrap_value(self.get_frame_jacobian(frame_name), [](DynamicsErr error) { return enum_error_message("DynamicsErr", error); });
    return matrix_to_numpy(jacobian);
}

void session_set_gravity_scale(PyRobotSession& self, const DoubleArray& gravity_scale) {
    self.set_gravity_scale(numpy_to_joint_vector(gravity_scale, "gravity_scale"));
}

void session_move_to(PyRobotSession& self, const DoubleArray& pos, double speed_scale) {
    self.move_to(numpy_to_joint_vector(pos, "pos"), speed_scale);
}

// ! ========================= 基 础 类 型 绑 定 方 法 实 现 ========================= ! //

/**
 * @brief 绑定公共错误码、模式和生命周期枚举
 * @param module pybind11 模块
 */
void bind_enums(py::module_& module) {
    py::enum_<JointImpedanceMode>(module, "JointImpedanceMode")
        .value("RIGID_HOLD", JointImpedanceMode::RIGID_HOLD)
        .value("RIGID_TRACKING", JointImpedanceMode::RIGID_TRACKING)
        .value("COMPLIANT_HOLD", JointImpedanceMode::COMPLIANT_HOLD)
        .value("COMPLIANT_DRAG", JointImpedanceMode::COMPLIANT_DRAG)
        .value("COMPLIANT_TRACKING", JointImpedanceMode::COMPLIANT_TRACKING);

    py::enum_<ModelFeedforwardMode>(module, "ModelFeedforwardMode")
        .value("NONE", ModelFeedforwardMode::NONE)
        .value("GRAVITY", ModelFeedforwardMode::GRAVITY)
        .value("FULL_INVERSE_DYNAMICS", ModelFeedforwardMode::FULL_INVERSE_DYNAMICS);

    py::enum_<JointCtrllerState>(module, "JointCtrllerState")
        .value("UNCONFIGURED", JointCtrllerState::UNCONFIGURED)
        .value("CONFIGURED", JointCtrllerState::CONFIGURED)
        .value("INITIALIZED", JointCtrllerState::INITIALIZED);

    py::enum_<SafetyAction>(module, "SafetyAction")
        .value("STOP_HOLD", SafetyAction::STOP_HOLD)
        .value("DISABLE", SafetyAction::DISABLE);

    py::enum_<JointCtrllerErr>(module, "JointCtrllerErr")
        .value("OK", JointCtrllerErr::OK)
        .value("NOT_CONFIGURED", JointCtrllerErr::NOT_CONFIGURED)
        .value("NOT_INITIALIZED", JointCtrllerErr::NOT_INITIALIZED)
        .value("ALREADY_INITIALIZED", JointCtrllerErr::ALREADY_INITIALIZED)
        .value("INVALID_CFG", JointCtrllerErr::INVALID_CFG)
        .value("INVALID_STATE", JointCtrllerErr::INVALID_STATE)
        .value("INVALID_DT", JointCtrllerErr::INVALID_DT)
        .value("INVALID_MODEL_FEEDFORWARD", JointCtrllerErr::INVALID_MODEL_FEEDFORWARD)
        .value("INVALID_IMPEDANCE_MODE", JointCtrllerErr::INVALID_IMPEDANCE_MODE)
        .value("INVALID_CMD_SIZE", JointCtrllerErr::INVALID_CMD_SIZE)
        .value("INVALID_CMD_VALUE", JointCtrllerErr::INVALID_CMD_VALUE)
        .value("INVALID_FULL_CMD", JointCtrllerErr::INVALID_FULL_CMD)
        .value("CMD_NOT_ALLOWED_IN_MODE", JointCtrllerErr::CMD_NOT_ALLOWED_IN_MODE)
        .value("FULL_CMD_NOT_ALLOWED", JointCtrllerErr::FULL_CMD_NOT_ALLOWED);

    py::enum_<JointActuatorMapErr>(module, "JointActuatorMapErr")
        .value("OK", JointActuatorMapErr::OK)
        .value("NOT_CONFIGURED", JointActuatorMapErr::NOT_CONFIGURED)
        .value("INVALID_CFG", JointActuatorMapErr::INVALID_CFG)
        .value("INVALID_JOINT_STATE", JointActuatorMapErr::INVALID_JOINT_STATE)
        .value("INVALID_ACTUATOR_STATE", JointActuatorMapErr::INVALID_ACTUATOR_STATE)
        .value("INVALID_JOINT_CMD", JointActuatorMapErr::INVALID_JOINT_CMD)
        .value("INVALID_ACTUATOR_CMD", JointActuatorMapErr::INVALID_ACTUATOR_CMD)
        .value("INVALID_CONVERSION_VALUE", JointActuatorMapErr::INVALID_CONVERSION_VALUE);

    py::enum_<MotorBusErr>(module, "MotorBusErr")
        .value("NOT_CONFIGURED", MotorBusErr::NOT_CONFIGURED)
        .value("NOT_CONNECTED", MotorBusErr::NOT_CONNECTED)
        .value("NOT_ACTIVE", MotorBusErr::NOT_ACTIVE)
        .value("INVALID_CFG", MotorBusErr::INVALID_CFG)
        .value("OPEN_FAILED", MotorBusErr::OPEN_FAILED)
        .value("READ_FAILED", MotorBusErr::READ_FAILED)
        .value("WRITE_FAILED", MotorBusErr::WRITE_FAILED)
        .value("INVALID_STATE", MotorBusErr::INVALID_STATE)
        .value("INVALID_CMD", MotorBusErr::INVALID_CMD)
        .value("ACTUATOR_OFFLINE", MotorBusErr::ACTUATOR_OFFLINE)
        .value("ACTUATOR_FAULT", MotorBusErr::ACTUATOR_FAULT)
        .value("TIMEOUT", MotorBusErr::TIMEOUT)
        .value("ENABLE_FAILED", MotorBusErr::ENABLE_FAILED)
        .value("MODE_SWITCH_FAILED", MotorBusErr::MODE_SWITCH_FAILED)
        .value("STOP_FAILED", MotorBusErr::STOP_FAILED)
        .value("DISABLE_FAILED", MotorBusErr::DISABLE_FAILED)
        .value("RECOVER_FAILED", MotorBusErr::RECOVER_FAILED);

    py::enum_<ModelFeedforwardErr>(module, "ModelFeedforwardErr")
        .value("NOT_CONFIGURED", ModelFeedforwardErr::NOT_CONFIGURED)
        .value("INVALID_INPUT", ModelFeedforwardErr::INVALID_INPUT)
        .value("INVALID_MODE", ModelFeedforwardErr::INVALID_MODE)
        .value("COMPUTE_FAILED", ModelFeedforwardErr::COMPUTE_FAILED);

    py::enum_<RobotErr>(module, "RobotErr")
        .value("NOT_CONFIGURED", RobotErr::NOT_CONFIGURED)
        .value("ALREADY_CONFIGURED", RobotErr::ALREADY_CONFIGURED)
        .value("INVALID_CFG", RobotErr::INVALID_CFG)
        .value("NULL_MOTOR_BUS", RobotErr::NULL_MOTOR_BUS)
        .value("MOTOR_BUS_SIZE_MISMATCH", RobotErr::MOTOR_BUS_SIZE_MISMATCH)
        .value("WRITE_DISABLED", RobotErr::WRITE_DISABLED)
        .value("NOT_ACTIVE", RobotErr::NOT_ACTIVE)
        .value("NOT_INACTIVE", RobotErr::NOT_INACTIVE)
        .value("ALREADY_ACTIVE", RobotErr::ALREADY_ACTIVE)
        .value("FAULTED", RobotErr::FAULTED)
        .value("NOT_FAULTED", RobotErr::NOT_FAULTED)
        .value("INVALID_TIME", RobotErr::INVALID_TIME)
        .value("MOTOR_BUS_CONNECT_FAILED", RobotErr::MOTOR_BUS_CONNECT_FAILED)
        .value("MOTOR_BUS_ACTIVATE_FAILED", RobotErr::MOTOR_BUS_ACTIVATE_FAILED)
        .value("MOTOR_BUS_READ_FAILED", RobotErr::MOTOR_BUS_READ_FAILED)
        .value("MOTOR_BUS_WRITE_FAILED", RobotErr::MOTOR_BUS_WRITE_FAILED)
        .value("MOTOR_BUS_DEACTIVATE_FAILED", RobotErr::MOTOR_BUS_DEACTIVATE_FAILED)
        .value("MOTOR_BUS_RECOVER_FAILED", RobotErr::MOTOR_BUS_RECOVER_FAILED)
        .value("MAPPER_FAILED", RobotErr::MAPPER_FAILED)
        .value("CTRLLER_FAILED", RobotErr::CTRLLER_FAILED)
        .value("SAFETY_FAILED", RobotErr::SAFETY_FAILED)
        .value("MODEL_FEEDFORWARD_FAILED", RobotErr::MODEL_FEEDFORWARD_FAILED)
        .value("INVALID_MODEL_FEEDFORWARD", RobotErr::INVALID_MODEL_FEEDFORWARD)
        .value("FAULT_RECOVERY_NOT_ALLOWED", RobotErr::FAULT_RECOVERY_NOT_ALLOWED);

    py::enum_<RobotState>(module, "RobotState")
        .value("UNCONFIGURED", RobotState::UNCONFIGURED)
        .value("INACTIVE", RobotState::INACTIVE)
        .value("ACTIVE", RobotState::ACTIVE)
        .value("FAULT", RobotState::FAULT);

    py::enum_<FaultHoldMode>(module, "FaultHoldMode")
        .value("RIGID_HOLD", FaultHoldMode::RIGID_HOLD)
        .value("COMPLIANT_RECOVERY", FaultHoldMode::COMPLIANT_RECOVERY);

    py::enum_<ConfigErr>(module, "ConfigErr")
        .value("FILE_OPEN_FAILED", ConfigErr::FILE_OPEN_FAILED)
        .value("SYNTAX_ERROR", ConfigErr::SYNTAX_ERROR)
        .value("MISSING_FIELD", ConfigErr::MISSING_FIELD)
        .value("INVALID_VALUE", ConfigErr::INVALID_VALUE)
        .value("INVALID_SIZE", ConfigErr::INVALID_SIZE)
        .value("DUPLICATE_NAME", ConfigErr::DUPLICATE_NAME);

    py::enum_<DynamicsErr>(module, "DynamicsErr")
        .value("NOT_CONFIGURED", DynamicsErr::NOT_CONFIGURED)
        .value("ALREADY_CONFIGURED", DynamicsErr::ALREADY_CONFIGURED)
        .value("NOT_UPDATED", DynamicsErr::NOT_UPDATED)
        .value("INVALID_CFG", DynamicsErr::INVALID_CFG)
        .value("URDF_LOAD_FAILED", DynamicsErr::URDF_LOAD_FAILED)
        .value("JOINT_NOT_FOUND", DynamicsErr::JOINT_NOT_FOUND)
        .value("JOINT_NOT_1DOF", DynamicsErr::JOINT_NOT_1DOF)
        .value("MODEL_SIZE_MISMATCH", DynamicsErr::MODEL_SIZE_MISMATCH)
        .value("FRAME_NOT_FOUND", DynamicsErr::FRAME_NOT_FOUND)
        .value("INVALID_INPUT_SIZE", DynamicsErr::INVALID_INPUT_SIZE)
        .value("NON_FINITE_INPUT", DynamicsErr::NON_FINITE_INPUT)
        .value("GRAVITY_SCALE_OUT_OF_RANGE", DynamicsErr::GRAVITY_SCALE_OUT_OF_RANGE)
        .value("COMPUTE_FAILED", DynamicsErr::COMPUTE_FAILED);

    py::enum_<SafetyErr>(module, "SafetyErr")
        .value("NOT_CONFIGURED", SafetyErr::NOT_CONFIGURED)
        .value("INVALID_CFG", SafetyErr::INVALID_CFG)
        .value("INVALID_DT", SafetyErr::INVALID_DT)
        .value("INVALID_STATE_AGE", SafetyErr::INVALID_STATE_AGE)
        .value("INVALID_CMD_AGE", SafetyErr::INVALID_CMD_AGE)
        .value("STATE_TIMEOUT", SafetyErr::STATE_TIMEOUT)
        .value("CMD_TIMEOUT", SafetyErr::CMD_TIMEOUT)
        .value("INVALID_JOINT_STATE_SIZE", SafetyErr::INVALID_JOINT_STATE_SIZE)
        .value("INVALID_ACTUATOR_STATE_SIZE", SafetyErr::INVALID_ACTUATOR_STATE_SIZE)
        .value("NON_FINITE_JOINT_STATE", SafetyErr::NON_FINITE_JOINT_STATE)
        .value("NON_FINITE_ACTUATOR_STATE", SafetyErr::NON_FINITE_ACTUATOR_STATE)
        .value("JOINT_POS_LIMIT", SafetyErr::JOINT_POS_LIMIT)
        .value("JOINT_VEL_LIMIT", SafetyErr::JOINT_VEL_LIMIT)
        .value("ACTUATOR_OFFLINE", SafetyErr::ACTUATOR_OFFLINE)
        .value("ACTUATOR_NOT_ENABLED", SafetyErr::ACTUATOR_NOT_ENABLED)
        .value("ACTUATOR_FAULT", SafetyErr::ACTUATOR_FAULT)
        .value("INVALID_CMD_SIZE", SafetyErr::INVALID_CMD_SIZE)
        .value("NON_FINITE_CMD", SafetyErr::NON_FINITE_CMD)
        .value("CMD_POS_LIMIT", SafetyErr::CMD_POS_LIMIT)
        .value("CMD_VEL_LIMIT", SafetyErr::CMD_VEL_LIMIT)
        .value("CMD_EFFORT_LIMIT", SafetyErr::CMD_EFFORT_LIMIT)
        .value("CMD_KP_LIMIT", SafetyErr::CMD_KP_LIMIT)
        .value("CMD_KD_LIMIT", SafetyErr::CMD_KD_LIMIT)
        .value("CMD_POS_STEP_LIMIT", SafetyErr::CMD_POS_STEP_LIMIT)
        .value("CMD_VEL_STEP_LIMIT", SafetyErr::CMD_VEL_STEP_LIMIT);
}

/**
 * @brief 绑定 Joint、Actuator、故障和周期快照结构
 * @param module pybind11 模块
 */
void bind_state_types(py::module_& module) {
    py::class_<JointState>(module, "JointState")
        .def(py::init<>())
        .def_property("pos", &get_double_member<JointState, &JointState::pos>, &set_double_member<JointState, &JointState::pos>)
        .def_property("vel", &get_double_member<JointState, &JointState::vel>, &set_double_member<JointState, &JointState::vel>)
        .def_property("tor", &get_double_member<JointState, &JointState::tor>, &set_double_member<JointState, &JointState::tor>);

    py::class_<ActuatorState>(module, "ActuatorState")
        .def(py::init<>())
        .def_property("pos", &get_double_member<ActuatorState, &ActuatorState::pos>, &set_double_member<ActuatorState, &ActuatorState::pos>)
        .def_property("vel", &get_double_member<ActuatorState, &ActuatorState::vel>, &set_double_member<ActuatorState, &ActuatorState::vel>)
        .def_property("tor", &get_double_member<ActuatorState, &ActuatorState::tor>, &set_double_member<ActuatorState, &ActuatorState::tor>)
        .def_property("online", &get_uint8_member<ActuatorState, &ActuatorState::online>, &set_uint8_member<ActuatorState, &ActuatorState::online>)
        .def_property("enabled", &get_uint8_member<ActuatorState, &ActuatorState::enabled>, &set_uint8_member<ActuatorState, &ActuatorState::enabled>)
        .def_property("err_code", &get_int_member<ActuatorState, &ActuatorState::err_code>, &set_int_member<ActuatorState, &ActuatorState::err_code>);

    py::class_<JointPosCmd>(module, "JointPosCmd")
        .def(py::init<>())
        .def_property("pos", &get_double_member<JointPosCmd, &JointPosCmd::pos>, &set_double_member<JointPosCmd, &JointPosCmd::pos>);

    py::class_<JointPosVelCmd>(module, "JointPosVelCmd")
        .def(py::init<>())
        .def_property("pos", &get_double_member<JointPosVelCmd, &JointPosVelCmd::pos>, &set_double_member<JointPosVelCmd, &JointPosVelCmd::pos>)
        .def_property("vel", &get_double_member<JointPosVelCmd, &JointPosVelCmd::vel>, &set_double_member<JointPosVelCmd, &JointPosVelCmd::vel>);

    py::class_<JointPosVelTorCmd>(module, "JointPosVelTorCmd")
        .def(py::init<>())
        .def_property("pos", &get_double_member<JointPosVelTorCmd, &JointPosVelTorCmd::pos>, &set_double_member<JointPosVelTorCmd, &JointPosVelTorCmd::pos>)
        .def_property("vel", &get_double_member<JointPosVelTorCmd, &JointPosVelTorCmd::vel>, &set_double_member<JointPosVelTorCmd, &JointPosVelTorCmd::vel>)
        .def_property("tor", &get_double_member<JointPosVelTorCmd, &JointPosVelTorCmd::tor>, &set_double_member<JointPosVelTorCmd, &JointPosVelTorCmd::tor>);

    py::class_<JointCtrlCmd>(module, "JointCtrlCmd")
        .def(py::init<>())
        .def_property("pos", &get_double_member<JointCtrlCmd, &JointCtrlCmd::pos>, &set_double_member<JointCtrlCmd, &JointCtrlCmd::pos>)
        .def_property("vel", &get_double_member<JointCtrlCmd, &JointCtrlCmd::vel>, &set_double_member<JointCtrlCmd, &JointCtrlCmd::vel>)
        .def_property("tor", &get_double_member<JointCtrlCmd, &JointCtrlCmd::tor>, &set_double_member<JointCtrlCmd, &JointCtrlCmd::tor>)
        .def_property("kp", &get_double_member<JointCtrlCmd, &JointCtrlCmd::kp>, &set_double_member<JointCtrlCmd, &JointCtrlCmd::kp>)
        .def_property("kd", &get_double_member<JointCtrlCmd, &JointCtrlCmd::kd>, &set_double_member<JointCtrlCmd, &JointCtrlCmd::kd>);

    py::class_<ActuatorCtrlCmd>(module, "ActuatorCtrlCmd")
        .def(py::init<>())
        .def_property("pos", &get_double_member<ActuatorCtrlCmd, &ActuatorCtrlCmd::pos>, &set_double_member<ActuatorCtrlCmd, &ActuatorCtrlCmd::pos>)
        .def_property("vel", &get_double_member<ActuatorCtrlCmd, &ActuatorCtrlCmd::vel>, &set_double_member<ActuatorCtrlCmd, &ActuatorCtrlCmd::vel>)
        .def_property("tor", &get_double_member<ActuatorCtrlCmd, &ActuatorCtrlCmd::tor>, &set_double_member<ActuatorCtrlCmd, &ActuatorCtrlCmd::tor>)
        .def_property("kp", &get_double_member<ActuatorCtrlCmd, &ActuatorCtrlCmd::kp>, &set_double_member<ActuatorCtrlCmd, &ActuatorCtrlCmd::kp>)
        .def_property("kd", &get_double_member<ActuatorCtrlCmd, &ActuatorCtrlCmd::kd>, &set_double_member<ActuatorCtrlCmd, &ActuatorCtrlCmd::kd>);
}

// ! ========================= 配 置 绑 定 方 法 实 现 ========================= ! //

/**
 * @brief 绑定配置结构、配置加载和配置验证函数
 * @param module pybind11 模块
 */
void bind_config(py::module_& module) {
    py::class_<ConfigErrInfo>(module, "ConfigErrInfo")
        .def(py::init<>())
        .def_readwrite("code", &ConfigErrInfo::code)
        .def_readwrite("message", &ConfigErrInfo::message);

    py::class_<JointImpedanceGains>(module, "JointImpedanceGains")
        .def(py::init<>())
        .def_readwrite("kp", &JointImpedanceGains::kp)
        .def_readwrite("kd", &JointImpedanceGains::kd);

    py::class_<JointCtrllerCfg>(module, "JointCtrllerCfg")
        .def(py::init<>())
        .def_readwrite("joints_count", &JointCtrllerCfg::joints_count)
        .def_readwrite("rigid_hold_gains", &JointCtrllerCfg::rigid_hold_gains)
        .def_readwrite("rigid_tracking_gains", &JointCtrllerCfg::rigid_tracking_gains)
        .def_readwrite("compliant_hold_gains", &JointCtrllerCfg::compliant_hold_gains)
        .def_readwrite("compliant_drag_gains", &JointCtrllerCfg::compliant_drag_gains)
        .def_readwrite("compliant_tracking_gains", &JointCtrllerCfg::compliant_tracking_gains)
        .def_readwrite("allow_full_cmd", &JointCtrllerCfg::allow_full_cmd);

    py::class_<JointActuatorMapCfg>(module, "JointActuatorMapCfg")
        .def(py::init<>())
        .def_readwrite("joints_count", &JointActuatorMapCfg::joints_count)
        .def_readwrite("pos_ratio", &JointActuatorMapCfg::pos_ratio)
        .def_readwrite("tor_ratio", &JointActuatorMapCfg::tor_ratio)
        .def_readwrite("direction", &JointActuatorMapCfg::direction)
        .def_readwrite("joint_zero_offset", &JointActuatorMapCfg::joint_zero_offset)
        .def_readwrite("actuator_zero_offset", &JointActuatorMapCfg::actuator_zero_offset);

    py::class_<JointLimitCfg>(module, "JointLimitCfg")
        .def(py::init<>())
        .def_readwrite("min_pos", &JointLimitCfg::min_pos)
        .def_readwrite("max_pos", &JointLimitCfg::max_pos)
        .def_readwrite("max_vel", &JointLimitCfg::max_vel)
        .def_readwrite("max_acc", &JointLimitCfg::max_acc)
        .def_readwrite("max_effort", &JointLimitCfg::max_effort)
        .def_readwrite("max_kp", &JointLimitCfg::max_kp)
        .def_readwrite("max_kd", &JointLimitCfg::max_kd)
        .def_readwrite("pos_margin", &JointLimitCfg::pos_margin);

    py::class_<FaultCompliantRecoveryCfg>(module, "FaultCompliantRecoveryCfg")
        .def(py::init<>())
        .def_readwrite("kp", &FaultCompliantRecoveryCfg::kp)
        .def_readwrite("kd", &FaultCompliantRecoveryCfg::kd)
        .def_readwrite("max_vel", &FaultCompliantRecoveryCfg::max_vel)
        .def_readwrite("effort_scale", &FaultCompliantRecoveryCfg::effort_scale);

    py::class_<FaultRecoveryCfg>(module, "FaultRecoveryCfg")
        .def(py::init<>())
        .def_readwrite("default_mode", &FaultRecoveryCfg::default_mode)
        .def_readwrite("allow_compliant_recovery", &FaultRecoveryCfg::allow_compliant_recovery)
        .def_readwrite("require_operator_request", &FaultRecoveryCfg::require_operator_request)
        .def_readwrite("gravity_model_validated", &FaultRecoveryCfg::gravity_model_validated)
        .def_readwrite("recovery_timeout_s", &FaultRecoveryCfg::recovery_timeout_s)
        .def_readwrite("compliant_recovery", &FaultRecoveryCfg::compliant_recovery);

    py::class_<SafetyCfg>(module, "SafetyCfg")
        .def(py::init<>())
        .def_readwrite("joints_count", &SafetyCfg::joints_count)
        .def_readwrite("limits", &SafetyCfg::limits)
        .def_readwrite("cmd_timeout_s", &SafetyCfg::cmd_timeout_s)
        .def_readwrite("state_timeout_s", &SafetyCfg::state_timeout_s)
        .def_readwrite("max_dt_s", &SafetyCfg::max_dt_s)
        .def_readwrite("numeric_tolerance", &SafetyCfg::numeric_tolerance)
        .def_readwrite("state_vel_fault_ratio", &SafetyCfg::state_vel_fault_ratio)
        .def_readwrite("require_all_actuators_online", &SafetyCfg::require_all_actuators_online)
        .def_readwrite("require_all_actuators_enabled", &SafetyCfg::require_all_actuators_enabled)
        .def_readwrite("fault_recovery", &SafetyCfg::fault_recovery);

    py::class_<RuntimeCfg>(module, "RuntimeCfg")
        .def(py::init<>())
        .def_readwrite("ctrl_frequency_hz", &RuntimeCfg::ctrl_frequency_hz)
        .def_readwrite("joint_acc_filter_alpha", &RuntimeCfg::joint_acc_filter_alpha)
        .def_readwrite("write_enabled", &RuntimeCfg::write_enabled)
        .def_readwrite("model_feedforward_mode", &RuntimeCfg::model_feedforward_mode)
        .def_readwrite("tracking_impedance_mode", &RuntimeCfg::tracking_impedance_mode);

    py::class_<ShutdownCfg>(module, "ShutdownCfg")
        .def(py::init<>())
        .def_readwrite("park_before_disable", &ShutdownCfg::park_before_disable)
        .def_readwrite("park_pos", &ShutdownCfg::park_pos)
        .def_readwrite("speed_scale", &ShutdownCfg::speed_scale)
        .def_readwrite("position_tolerance", &ShutdownCfg::position_tolerance)
        .def_readwrite("velocity_tolerance", &ShutdownCfg::velocity_tolerance)
        .def_readwrite("settle_time_s", &ShutdownCfg::settle_time_s)
        .def_readwrite("relaxed_tolerance_ratio", &ShutdownCfg::relaxed_tolerance_ratio)
        .def_readwrite("timeout_s", &ShutdownCfg::timeout_s);

    py::class_<DynamicsCfg>(module, "DynamicsCfg")
        .def(py::init<>())
        .def_readwrite("urdf_path", &DynamicsCfg::urdf_path)
        .def_readwrite("joint_names", &DynamicsCfg::joint_names)
        .def_readwrite("base_frame", &DynamicsCfg::base_frame)
        .def_readwrite("tool_frame", &DynamicsCfg::tool_frame)
        .def_readwrite("gravity", &DynamicsCfg::gravity)
        .def_readwrite("gravity_scale", &DynamicsCfg::gravity_scale);

    py::class_<RobotCfg>(module, "RobotCfg")
        .def(py::init<>())
        .def_readwrite("joint_names", &RobotCfg::joint_names)
        .def_readwrite("runtime", &RobotCfg::runtime)
        .def_readwrite("shutdown", &RobotCfg::shutdown)
        .def_readwrite("ctrller", &RobotCfg::ctrller)
        .def_readwrite("mapper", &RobotCfg::mapper)
        .def_readwrite("safety", &RobotCfg::safety)
        .def_readwrite("dynamics", &RobotCfg::dynamics);

    module.def("load_robot_cfg", [](const std::string& path, const std::string& hardware_plugin, const std::string& hardware_config) {
        HardwareLoader loader;
        auto bus = unwrap_value(loader.load(hardware_plugin, hardware_config), [](HardwareLoaderErr error) { return "HardwareLoaderErr=" + std::to_string(static_cast<int>(error)); });
        return unwrap_value(load_robot_cfg(path, bus->capabilities()), [](const ConfigErrInfo& error) { return error.message; });
        });
    module.def("validate_robot_core_cfg", [](const RobotCfg& cfg) {
        unwrap_void(validate_robot_core_cfg(cfg), [](const ConfigErrInfo& error) { return error.message; });
        });
    module.def("validate_robot_cfg", [](const RobotCfg& cfg) {
        unwrap_void(validate_robot_cfg(cfg), [](const ConfigErrInfo& error) { return error.message; });
        });
}

// ! ========================= Core 绑 定 方 法 实 现 ========================= ! //

/**
 * @brief 绑定 JointCtrller、JointActuatorMapper 和 Safety
 * @param module pybind11 模块
 */
void bind_core(py::module_& module) {
    py::class_<JointCtrller>(module, "JointCtrller")
        .def(py::init<>())
        .def("configure", &configure_joint_ctrller)
        .def("initialize", &initialize_joint_ctrller)
        .def("reset", &JointCtrller::reset)
        .def("set_impedance_mode", &set_joint_ctrller_mode)
        .def("set_pos_cmd", &set_joint_pos_cmd)
        .def("set_pos_vel_cmd", &set_joint_pos_vel_cmd)
        .def("set_pos_vel_tor_cmd", &set_joint_pos_vel_tor_cmd)
        .def("set_full_cmd", &set_joint_full_cmd)
        .def("update", &update_joint_ctrller)
        .def_property_readonly("state", &JointCtrller::get_state)
        .def_property_readonly("impedance_mode", &JointCtrller::get_impedance_mode);

    py::class_<JointActuatorMapper>(module, "JointActuatorMapper")
        .def(py::init<>())
        .def("configure", &configure_mapper)
        .def("to_actuator_cmd", &mapper_to_actuator_cmd)
        .def("to_joint_state", &mapper_to_joint_state)
        .def_property_readonly("size", &JointActuatorMapper::size);

    py::class_<SafetyFault>(module, "SafetyFault")
        .def(py::init<>())
        .def_readwrite("code", &SafetyFault::code)
        .def_readwrite("index", &SafetyFault::index)
        .def_readwrite("value", &SafetyFault::value)
        .def_readwrite("limit", &SafetyFault::limit);

    py::class_<Safety>(module, "Safety")
        .def(py::init<>())
        .def("configure", [](Safety& self, const SafetyCfg& cfg) { unwrap_void(self.configure(cfg), safety_error_message); })
        .def("check_state", &safety_check_state)
        .def("check_cmd_age", [](const Safety& self, double cmd_age_s) { unwrap_void(self.check_cmd_age(cmd_age_s), safety_error_message); })
        .def("check_joint_cmd", &safety_check_joint_cmd)
        .def("reset_cmd_history", [](Safety& self, const JointState& state) { unwrap_void(self.reset_cmd_history(state), safety_error_message); })
        .def("clear_cmd_history", &Safety::clear_cmd_history)
        .def("action_for", &Safety::action_for)
        .def_property_readonly("configured", &Safety::is_configured)
        .def_property_readonly("clamp_count", &Safety::clamp_count);
}

// ! ========================= Dynamics 绑 定 方 法 实 现 ========================= ! //

/**
 * @brief 绑定 Dynamics 集中更新接口和只读缓存 getter
 * @param module pybind11 模块
 */
void bind_dynamics(py::module_& module) {
    py::class_<DynamicsInfo>(module, "DynamicsInfo")
        .def(py::init<>())
        .def_readonly("joints_count", &DynamicsInfo::joints_count)
        .def_readonly("nq", &DynamicsInfo::nq)
        .def_readonly("nv", &DynamicsInfo::nv)
        .def_readonly("total_mass", &DynamicsInfo::total_mass)
        .def_readonly("joint_names", &DynamicsInfo::joint_names)
        .def_readonly("q_indices", &DynamicsInfo::q_indices)
        .def_readonly("v_indices", &DynamicsInfo::v_indices);

    py::class_<DynamicsState>(module, "DynamicsState")
        .def(py::init<>())
        .def_property_readonly("pos", [](const DynamicsState& self) { return vector_to_numpy(self.pos); })
        .def_property_readonly("vel", [](const DynamicsState& self) { return vector_to_numpy(self.vel); })
        .def_property_readonly("acc", [](const DynamicsState& self) { return vector_to_numpy(self.acc); })
        .def_property_readonly("tor", [](const DynamicsState& self) { return vector_to_numpy(self.tor); })
        .def_property_readonly("ref_acc", [](const DynamicsState& self) { return vector_to_numpy(self.ref_acc); })
        .def_property_readonly("gravity", [](const DynamicsState& self) { return vector_to_numpy(self.gravity); })
        .def_property_readonly("gravity_compensation", [](const DynamicsState& self) { return vector_to_numpy(self.gravity_compensation); })
        .def_property_readonly("nonlinear", [](const DynamicsState& self) { return vector_to_numpy(self.nonlinear); })
        .def_property_readonly("coriolis", [](const DynamicsState& self) { return vector_to_numpy(self.coriolis); })
        .def_property_readonly("inverse_dynamics", [](const DynamicsState& self) { return vector_to_numpy(self.inverse_dynamics); })
        .def_property_readonly("forward_dynamics", [](const DynamicsState& self) { return vector_to_numpy(self.forward_dynamics); })
        .def_property_readonly("mass_matrix", [](const DynamicsState& self) { return matrix_to_numpy(self.mass_matrix); })
        .def_property_readonly("tool_pose", [](const DynamicsState& self) { return pose_to_numpy(self.tool_pose); })
        .def_property_readonly("tool_jacobian", [](const DynamicsState& self) { return matrix_to_numpy(self.tool_jacobian); });

    py::class_<Dynamics>(module, "Dynamics")
        .def(py::init<>())
        .def("configure", [](Dynamics& self, const DynamicsCfg& cfg) {
        py::gil_scoped_release release;
        unwrap_void(self.configure(cfg), [](DynamicsErr error) { return enum_error_message("DynamicsErr", error); });
            })
        .def("update", [](Dynamics& self, const DoubleArray& pos, const DoubleArray& vel, const DoubleArray& acc, const DoubleArray& tor, const DoubleArray& ref_acc) {
        const std::size_t size = self.get_info().joints_count;
        JointState state;
        state.pos = numpy_to_joint_vector(pos, size, "pos");
        state.vel = numpy_to_joint_vector(vel, size, "vel");
        state.tor = numpy_to_joint_vector(tor, size, "tor");
        JointVector acc_values = numpy_to_joint_vector(acc, size, "acc");
        JointVector ref_acc_values = numpy_to_joint_vector(ref_acc, size, "ref_acc");
        py::gil_scoped_release release;
        unwrap_void(self.update(state, acc_values, ref_acc_values), [](DynamicsErr error) { return enum_error_message("DynamicsErr", error); });
            })
        .def("update_state", [](Dynamics& self, const JointState& state, const DoubleArray& acc, const DoubleArray& ref_acc) {
        JointVector acc_values = numpy_to_joint_vector(acc, state.pos.size(), "acc");
        JointVector ref_acc_values = numpy_to_joint_vector(ref_acc, state.pos.size(), "ref_acc");
        py::gil_scoped_release release;
        unwrap_void(self.update(state, acc_values, ref_acc_values), [](DynamicsErr error) { return enum_error_message("DynamicsErr", error); });
            })
        .def("set_gravity_scale", &dynamics_set_gravity_scale)
        .def("cleanup", &Dynamics::cleanup)
        .def("frame_pose", &dynamics_frame_pose)
        .def("frame_jacobian", &dynamics_frame_jacobian)
        .def_property_readonly("configured", &Dynamics::is_configured)
        .def_property_readonly("updated", &Dynamics::is_updated)
        .def_property_readonly("info", [](const Dynamics& self) { return self.get_info(); })
        .def_property_readonly("state", [](const Dynamics& self) { return self.get_state(); })
        .def_property_readonly("gravity_scale", [](const Dynamics& self) { return vector_to_numpy(self.get_gravity_scale()); })
        .def_property_readonly("gravity", [](const Dynamics& self) { return vector_to_numpy(self.get_gravity()); })
        .def_property_readonly("gravity_compensation", [](const Dynamics& self) { return vector_to_numpy(self.get_gravity_compensation()); })
        .def_property_readonly("nonlinear", [](const Dynamics& self) { return vector_to_numpy(self.get_nonlinear()); })
        .def_property_readonly("coriolis", [](const Dynamics& self) { return vector_to_numpy(self.get_coriolis()); })
        .def_property_readonly("mass_matrix", [](const Dynamics& self) { return matrix_to_numpy(self.get_mass_matrix()); })
        .def_property_readonly("inverse_dynamics", [](const Dynamics& self) { return vector_to_numpy(self.get_inverse_dynamics()); })
        .def_property_readonly("forward_dynamics", [](const Dynamics& self) { return vector_to_numpy(self.get_forward_dynamics()); })
        .def_property_readonly("tool_pose", [](const Dynamics& self) { return pose_to_numpy(self.get_tool_pose()); })
        .def_property_readonly("tool_jacobian", [](const Dynamics& self) { return matrix_to_numpy(self.get_tool_jacobian()); });
}

// ! ========================= RobotSession 绑 定 方 法 实 现 ========================= ! //

/**
 * @brief 绑定由 C++ 工作线程驱动的真机 RobotSession
 * @param module pybind11 模块
 */
void bind_robot_session(py::module_& module) {
    py::class_<RobotFault>(module, "RobotFault")
        .def(py::init<>())
        .def_readonly("code", &RobotFault::code)
        .def_readonly("motor_bus_err", &RobotFault::motor_bus_err)
        .def_readonly("mapper_err", &RobotFault::mapper_err)
        .def_readonly("ctrller_err", &RobotFault::ctrller_err)
        .def_readonly("safety_fault", &RobotFault::safety_fault)
        .def_readonly("model_feedforward_err", &RobotFault::model_feedforward_err);

    py::class_<RobotCycleOutput>(module, "RobotCycleOutput")
        .def(py::init<>())
        .def_readonly("actuator_state", &RobotCycleOutput::actuator_state)
        .def_readonly("joint_state", &RobotCycleOutput::joint_state)
        .def_property_readonly("joint_acc", [](const RobotCycleOutput& self) { return vector_to_numpy(self.joint_acc); })
        .def_property_readonly("joint_ref_acc", [](const RobotCycleOutput& self) { return vector_to_numpy(self.joint_ref_acc); })
        .def_property_readonly("model_feedforward", [](const RobotCycleOutput& self) { return vector_to_numpy(self.model_feedforward); })
        .def_readonly("joint_cmd", &RobotCycleOutput::joint_cmd)
        .def_readonly("actuator_cmd", &RobotCycleOutput::actuator_cmd)
        .def_readonly("dt", &RobotCycleOutput::dt);

    py::class_<RobotSessionActuatorInfo>(module, "ActuatorInfo")
        .def(py::init<>())
        .def_readonly("name", &RobotSessionActuatorInfo::name)
        .def_readonly("joint_name", &RobotSessionActuatorInfo::joint_name)
        .def_readonly("min_pos", &RobotSessionActuatorInfo::min_pos)
        .def_readonly("max_pos", &RobotSessionActuatorInfo::max_pos)
        .def_readonly("max_vel", &RobotSessionActuatorInfo::max_vel)
        .def_readonly("max_effort", &RobotSessionActuatorInfo::max_effort)
        .def_readonly("max_kp", &RobotSessionActuatorInfo::max_kp)
        .def_readonly("max_kd", &RobotSessionActuatorInfo::max_kd);

    py::class_<RobotSessionSnapshot>(module, "RobotSessionSnapshot")
        .def(py::init<>())
        .def_readonly("robot_state", &RobotSessionSnapshot::robot_state)
        .def_readonly("cycle", &RobotSessionSnapshot::cycle)
        .def_readonly("dynamics", &RobotSessionSnapshot::dynamics)
        .def_readonly("valid", &RobotSessionSnapshot::valid)
        .def_readonly("last_error", &RobotSessionSnapshot::last_error);

    py::class_<PyRobotSession>(module, "_RobotSession")
        .def(py::init<>())
        .def("configure", &PyRobotSession::configure, py::call_guard<py::gil_scoped_release>())
        .def("start", &PyRobotSession::start, py::call_guard<py::gil_scoped_release>())
        .def("stop", &PyRobotSession::stop, py::call_guard<py::gil_scoped_release>())
        .def("reset_fault", &PyRobotSession::reset_fault, py::call_guard<py::gil_scoped_release>())
        .def("clear_fault", &PyRobotSession::clear_fault, py::call_guard<py::gil_scoped_release>())
        .def("enter_fault_compliant_recovery", &PyRobotSession::enter_fault_compliant_recovery, py::call_guard<py::gil_scoped_release>())
        .def("return_to_fault_rigid_hold", &PyRobotSession::return_to_fault_rigid_hold, py::call_guard<py::gil_scoped_release>())
        .def("set_impedance_mode", &PyRobotSession::set_impedance_mode)
        .def("set_model_feedforward_mode", &PyRobotSession::set_model_feedforward_mode)
        .def("set_gravity_scale", &session_set_gravity_scale)
        .def("move_to", &session_move_to, py::arg("pos"), py::arg("speed_scale") = 0.3)
        .def("hold_current", &PyRobotSession::hold_current)
        .def_property_readonly("snapshot", &PyRobotSession::get_snapshot)
        .def_property_readonly("state", &PyRobotSession::get_state)
        .def_property_readonly("fault_hold_mode", &PyRobotSession::get_fault_hold_mode)
        .def_property_readonly("configured", &PyRobotSession::is_configured)
        .def_property_readonly("running", &PyRobotSession::is_running)
        .def_property_readonly("config", &PyRobotSession::get_config)
        .def_property_readonly("dynamics_info", &PyRobotSession::get_dynamics_info)
        .def_property_readonly("actuator_info", &PyRobotSession::get_actuator_info);
}

} // namespace python_binding

} // namespace serial_arm

/**
 * @brief 创建 SerialArm Python 扩展模块并注册全部公开绑定
 */
PYBIND11_MODULE(_serial_arm, module) {
    module.doc() = "SerialArm C++17 control, dynamics and hardware bindings";
    module.attr("__version__") = "0.1.0";

    py::register_exception<serial_arm::SerialArmPythonError>(module, "SerialArmError");

    serial_arm::python_binding::bind_enums(module);
    serial_arm::python_binding::bind_state_types(module);
    serial_arm::python_binding::bind_config(module);
    serial_arm::python_binding::bind_core(module);
    serial_arm::python_binding::bind_dynamics(module);
    serial_arm::python_binding::bind_robot_session(module);
}
