# SerialArm-Core API Reference

本文只记录接口级说明；教程和配置流程见 [Tutorial.md](Tutorial.md)

## CMake Targets

安装后，下游纯 C++ 项目可以通过 CMake 导入 SerialArm-Core：

```cmake
find_package(serial_arm_core CONFIG REQUIRED)

add_executable(my_arm_driver main.cpp)
target_link_libraries(my_arm_driver
  PRIVATE
    serial_arm::core
    serial_arm::config
    serial_arm::robot
    serial_arm::dynamics
)
```

导出的 targets：

- `serial_arm::core`
- `serial_arm::config`
- `serial_arm::robot`
- `serial_arm::dynamics`

`serial_arm::dynamics` 是必选 target，依赖 Pinocchio 和 Eigen3；`serial_arm::config` 依赖 yaml-cpp，并包含 ModelLoader、LimitResolver 和 HardwareLoader

## 基础类型

`JointVector` 和 `ActuatorVector` 均为 `std::vector<double>`

### JointState

```cpp
struct JointState {
    JointVector pos;
    JointVector vel;
    JointVector tor;
};
```

单位：`rad`、`rad/s`、`N·m`

### ActuatorState

```cpp
struct ActuatorState {
    ActuatorVector pos;
    ActuatorVector vel;
    ActuatorVector tor;
    std::vector<std::uint8_t> online;
    std::vector<std::uint8_t> enabled;
    std::vector<int> err_code;
};
```

Backend 必须把厂商反馈转换成 SerialArm actuator-side semantics

### JointCtrlCmd

```cpp
struct JointCtrlCmd {
    JointVector pos;
    JointVector vel;
    JointVector tor;
    JointVector kp;
    JointVector kd;
};
```

### ActuatorCtrlCmd

```cpp
struct ActuatorCtrlCmd {
    ActuatorVector pos;
    ActuatorVector vel;
    ActuatorVector tor;
    ActuatorVector kp;
    ActuatorVector kd;
};
```

这是 Core 输出给 `MotorBus::write()` 的完整 MIT / impedance command

## 枚举

### JointImpedanceMode

- `RIGID_HOLD`
- `RIGID_TRACKING`
- `COMPLIANT_HOLD`
- `COMPLIANT_DRAG`
- `COMPLIANT_TRACKING`

五种模式都依赖完整 `position / velocity / torque / kp / kd` 语义

### ModelFeedforwardMode

- `NONE`
- `GRAVITY`
- `FULL_INVERSE_DYNAMICS`

## ModelLoader

```cpp
class ModelLoader {
public:
    tl::expected<RobotModelInfo, ModelErr> load(
        const std::string& urdf_path,
        const std::vector<std::string>& controlled_joint_names) const;
};
```

`RobotModelInfo` 包含 URDF path、joint order 和每个受控 joint 的 `ModelJointLimit`

`ModelJointLimit::has_position_limit=false` 表示 continuous joint 或无绝对位置限位 joint；continuous joint 不伪造 `[-pi, pi]`

## LimitResolver

```cpp
class LimitResolver {
public:
    tl::expected<ResolvedSafetyCfg, LimitResolverErr> resolve(
        const RobotModelInfo& model,
        const JointActuatorMapCfg& mapper,
        const HardwareCapabilities& capabilities,
        const SafetyPolicyCfg& policy) const;
};
```

`LimitResolver` 将 URDF limit、mapping、hardware capability 和 safety policy 收敛为 `ResolvedSafetyCfg`

`to_safety_cfg(const ResolvedSafetyCfg&)` 保留每个 joint 的 `has_position_limit`，供 Safety 决定是否执行位置硬限位和命令位置软限位

## Safety

```cpp
class Safety {
public:
    tl::expected<void, SafetyFault> configure(const SafetyCfg& cfg);
    tl::expected<void, SafetyFault> check_state(
        const JointState& joint_state,
        const ActuatorState& actuator_state,
        double state_age_s) const;
    tl::expected<void, SafetyFault> check_cmd_age(double cmd_age_s) const;
    tl::expected<JointCtrlCmd, SafetyFault> check_joint_cmd(
        const JointState& state,
        const JointCtrlCmd& cmd,
        double dt);
    tl::expected<void, SafetyFault> reset_cmd_history(const JointState& state);
    void clear_cmd_history() noexcept;
    SafetyAction action_for(SafetyErr err) const noexcept;
    bool is_configured() const noexcept;
    std::uint64_t clamp_count() const noexcept;
};
```

`SafetyCfg::limits.has_position_limit[i] == 0` 时，该 joint 跳过：

- `JOINT_POS_LIMIT`
- `CMD_POS_LIMIT`
- `position_margin`

其他 SafetyErr 仍适用，包括 velocity、effort、kp、kd、NaN/Inf、timeout、state validity 和 command step safety

## JointActuatorMapper

```cpp
class JointActuatorMapper {
public:
    tl::expected<void, JointActuatorMapErr> configure(const JointActuatorMapCfg& cfg);
    tl::expected<ActuatorCtrlCmd, JointActuatorMapErr> to_actuator_cmd(
        const JointCtrlCmd& joint_cmd) const;
    tl::expected<JointState, JointActuatorMapErr> to_joint_state(
        const ActuatorState& actuator_state) const;
    std::size_t size() const noexcept;
};
```

Mapper 负责 joint-side 和 actuator-side 的 position、velocity、torque、kp、kd 转换；转换后的 `ActuatorCtrlCmd` 必须完整传给 `MotorBus`

## Dynamics

```cpp
class Dynamics {
public:
    tl::expected<void, DynamicsErr> configure(const DynamicsCfg& cfg);
    tl::expected<void, DynamicsErr> update(
        const JointState& state,
        const JointVector& acc,
        const JointVector& ref_acc);
    tl::expected<void, DynamicsErr> set_gravity_scale(const JointVector& gravity_scale);
    void cleanup();
    bool is_configured() const noexcept;
    bool is_updated() const noexcept;
    const DynamicsInfo& get_info() const noexcept;
    const DynamicsState& get_state() const noexcept;
    tl::expected<Eigen::Isometry3d, DynamicsErr> get_frame_pose(const std::string& frame_name) const;
    tl::expected<Eigen::MatrixXd, DynamicsErr> get_frame_jacobian(const std::string& frame_name) const;
    const JointVector& get_gravity() const noexcept;
    const JointVector& get_gravity_compensation() const noexcept;
    const Eigen::MatrixXd& get_mass_matrix() const noexcept;
    const JointVector& get_inverse_dynamics() const noexcept;
    const JointVector& get_forward_dynamics() const noexcept;
    const Eigen::Isometry3d& get_tool_pose() const noexcept;
    const Eigen::MatrixXd& get_tool_jacobian() const noexcept;
};
```

Dynamics 是 mandatory Core API，依赖 Pinocchio；当前 Dynamics 支持受控 joint 为单自由度且 `nq == 1`、`nv == 1` 的模型路径；continuous joint 的 safety/model limit 支持不等于 Dynamics 已支持 multi-representation continuous joint

## MotorBus Contract

```cpp
class MotorBus {
public:
    virtual tl::expected<void, MotorBusErr> configure(const std::string& config_path) = 0;
    virtual tl::expected<void, MotorBusErr> connect() = 0;
    virtual tl::expected<ActuatorState, MotorBusErr> read() = 0;
    virtual tl::expected<void, MotorBusErr> activate() = 0;
    virtual tl::expected<void, MotorBusErr> write(const ActuatorCtrlCmd& cmd) = 0;
    virtual tl::expected<void, MotorBusErr> stop() = 0;
    virtual tl::expected<void, MotorBusErr> deactivate() = 0;
    virtual tl::expected<void, MotorBusErr> recover() = 0;
    virtual const HardwareCapabilities& capabilities() const noexcept = 0;
    virtual void cleanup() noexcept = 0;
    virtual std::size_t size() const noexcept = 0;
};
```

`MotorBus::write()` 的 `ActuatorCtrlCmd` 必须包含完整 MIT semantics：

- `position`
- `velocity`
- `torque / effort`
- `kp`
- `kd`

SerialArm-Core 不定义 Backend capability switches，不根据 Backend 能力降级；若底层硬件不原生支持 MIT，Backend 自己完成协议映射、模拟或适配；无法实现完整 MIT semantics 的 Backend 不属于完整支持的 SerialArm-Core Backend

## HardwareLoader

```cpp
class HardwareLoader {
public:
    tl::expected<std::unique_ptr<MotorBus>, HardwareLoaderErr> load(
        const std::string& plugin,
        const std::string& config_path);
};
```

Backend shared library 需导出 `create_motor_bus` / `destroy_motor_bus`，并在 `configure()` 中读取自己的 YAML

## Robot

```cpp
class Robot {
public:
    tl::expected<void, RobotFault> configure(
        const RobotCfg& cfg,
        std::unique_ptr<MotorBus> motor_bus,
        ModelFeedforwardFn model_feedforward = {});
    tl::expected<void, RobotFault> connect();
    tl::expected<void, RobotFault> activate();
    tl::expected<RobotCycleOutput, RobotFault> update(Clock::time_point now = Clock::now());
    tl::expected<void, RobotFault> set_impedance_mode(JointImpedanceMode mode, Clock::time_point now = Clock::now());
    tl::expected<void, RobotFault> set_command(const JointCmd& cmd, Clock::time_point now = Clock::now());
    tl::expected<void, RobotFault> stop();
    tl::expected<void, RobotFault> deactivate();
    tl::expected<void, RobotFault> recover();
};
```

Robot 组合 Controller、Safety、Mapper、MotorBus 和可选 model feedforward callback；Robot 不直接实现厂商协议

## Python Binding API

Python package 暴露 `_serial_arm` 绑定和 `serial_arm` Python facade；常用接口包括：

- enum：`JointImpedanceMode`、`ModelFeedforwardMode`
- config / state / command 类型
- offline model / dynamics helper
- `RobotSession`

Python API 与 C++ Core 共享同一语义：joint-side command 进入 Core，actuator-side command 由 Mapper 转换并交给 Backend

## 错误返回

主要接口使用 `tl::expected<T, Err>` 或 `tl::expected<void, Err>`：

- 成功：返回 value 或空成功值
- 失败：返回 enum 或 fault struct

常见错误族：

- `ModelErr`
- `LimitResolverErr`
- `SafetyErr`
- `DynamicsErr`
- `MotorBusErr`
- `HardwareLoaderErr`
- `RobotErr`

调用方应按错误族处理，不应通过字符串解析错误
