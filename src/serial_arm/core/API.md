# SerialArm Core API

命名空间为 `serial_arm`；语言标准为 C++17；本文件只说明公开接口、语义、生命周期和调用约束

## 1. 文档说明

### 1.1. 覆盖范围

本文覆盖

- CMake package 与导出 Targets
- Core 基础类型
- Config、ModelLoader、Hardware Capability 和 LimitResolver
- JointCtrller、JointActuatorMapper 和 Safety
- MotorBus、HardwareLoader 与外部 Hardware Backend
- Dynamics 和 Robot
- Python Binding 与 RobotSession
- Adapter 接入契约

### 1.2. 头文件与命名空间

```cpp
#include <serial_arm/config/config.hpp>
#include <serial_arm/config/limit_resolver.hpp>
#include <serial_arm/core/types.hpp>
#include <serial_arm/core/joints_ctrller.hpp>
#include <serial_arm/core/joint_actuator_mapper.hpp>
#include <serial_arm/core/safety.hpp>
#include <serial_arm/model/model_loader.hpp>
#include <serial_arm/hardware/hardware_capability.hpp>
#include <serial_arm/hardware/motor_bus.hpp>
#include <serial_arm/hardware/hardware_loader.hpp>
#include <serial_arm/dynamics/dynamics.hpp>
#include <serial_arm/robot.hpp>
```

```cpp
namespace serial_arm
```

### 1.3. API 兼容性

当前版本为 `0.1.0`；尚未承诺 ABI 稳定；公共结构体仍可能随 Config、Payload 和多后端需求调整

## 2. CMake Targets

### 2.1. serial_arm::core

包含

- `JointCtrller`
- `JointActuatorMapper`
- `Safety`
- Core 基础类型

```cmake
find_package(serial_arm_core CONFIG REQUIRED)
target_link_libraries(my_target PRIVATE serial_arm::core)
```

### 2.2. serial_arm::config

包含

- YAML 加载与验证
- `ModelLoader`
- `Hardware Capability`
- `LimitResolver`

依赖 `serial_arm::core`、yaml-cpp 和 Pinocchio

### 2.3. serial_arm::robot

包含 Robot 生命周期、周期闭环、故障处理、加速度估计和模型前馈入口

依赖 `serial_arm::core` 与 `serial_arm::config`

### 2.4. serial_arm::dynamics

生成条件

```cmake
-DSERIAL_ARM_ENABLE_DYNAMICS=ON
```

包含 Pinocchio 运动学和刚体动力学

安装后使用 Dynamics 时应确保 Pinocchio CMake package 已进入 `CMAKE_PREFIX_PATH`；下游可显式执行

```cmake
find_package(pinocchio CONFIG REQUIRED)
find_package(serial_arm_core CONFIG REQUIRED)
```

## 3. 通用约定

### 3.1. C++ 标准

所有公开 Targets 使用 C++17

```cmake
target_compile_features(my_target PRIVATE cxx_std_17)
```

### 3.2. Joint 与 Actuator 顺序

所有 `JointVector` 按 `RobotCfg::joint_names` 排列；Actuator 顺序必须与 Joint 顺序和 Hardware Backend `capabilities()` 顺序对应

受控关节顺序由 Core YAML 的 `joint_names` 定义；Core API 不假设固定关节数量或固定关节名

### 3.3. 单位与坐标系

| 数据 | 单位 |
|---|---|
| Joint 位置 | rad |
| Joint 速度 | rad/s |
| Joint 加速度 | rad/s² |
| Joint 力矩 | N·m |
| `kp` | N·m/rad |
| `kd` | N·m·s/rad |
| 平移 | m |
| 质量 | kg |
| 惯性张量 | kg·m² |

Dynamics 位姿表示 `frame` 相对 `base_frame` 的变换；Jacobians 使用 `LOCAL_WORLD_ALIGNED`

### 3.4. tl::expected

主要 C++ 操作使用

```cpp
tl::expected<T, Error>
```

调用者必须检查返回值

```cpp
const auto result = robot.activate();
if(!result) {
    const serial_arm::RobotFault& fault = result.error();
}
```

### 3.5. 所有权与线程安全

- `Robot::configure()` 接收 `std::unique_ptr<MotorBus>` 并独占后端
- `Robot`、`Dynamics`、`MotorBus` 和 Core 控制类不提供通用内部互斥
- 一个 Robot 实例只能由一个控制线程访问
- Python RobotSession 和 ros2_control Adapter 在外层实现线程串行化

### 3.6. 实时与缓存语义

- `Robot::cycle()` 是单周期接口
- `Dynamics::update()` 是集中计算入口
- Dynamics getter 只读取最近一次成功更新的缓存
- Core 不保证硬实时
- 控制周期目标由 `RuntimeCfg::ctrl_frequency_hz` 决定

## 4. 基础类型

头文件

```cpp
#include <serial_arm/core/types.hpp>
```

### 4.1. JointState 与 ActuatorState

```cpp
using JointVector = std::vector<double>;
using ActuatorVector = std::vector<double>;

struct JointState {
    JointVector pos;
    JointVector vel;
    JointVector tor;
};

struct ActuatorState {
    ActuatorVector pos;
    ActuatorVector vel;
    ActuatorVector tor;
    std::vector<std::uint8_t> online;
    std::vector<std::uint8_t> enabled;
    std::vector<int> err_code;
};
```

### 4.2. JointCmd

```cpp
struct JointPosCmd {
    JointVector pos;
};

struct JointPosVelCmd {
    JointVector pos;
    JointVector vel;
};

struct JointPosVelTorCmd {
    JointVector pos;
    JointVector vel;
    JointVector tor;
};

using JointCmd = std::variant<JointPosCmd, JointPosVelCmd, JointPosVelTorCmd>;
```

### 4.3. JointCtrlCmd

```cpp
struct JointCtrlCmd {
    JointVector pos;
    JointVector vel;
    JointVector tor;
    JointVector kp;
    JointVector kd;
};
```

该结构位于 Joint 侧；发送到硬件前必须经过 `JointActuatorMapper`

### 4.4. ActuatorCtrlCmd

```cpp
struct ActuatorCtrlCmd {
    ActuatorVector pos;
    ActuatorVector vel;
    ActuatorVector tor;
    ActuatorVector kp;
    ActuatorVector kd;
};
```

### 4.5. 阻抗与前馈枚举

```cpp
enum class JointImpedanceMode {
    RIGID_HOLD,
    RIGID_TRACKING,
    COMPLIANT_HOLD,
    COMPLIANT_DRAG,
    COMPLIANT_TRACKING,
};

enum class ModelFeedforwardMode {
    NONE,
    GRAVITY,
    FULL_INVERSE_DYNAMICS,
};
```

## 5. Config

头文件

```cpp
#include <serial_arm/config/config.hpp>
```

### 5.1. RobotCfg

```cpp
struct RobotCfg {
    std::vector<std::string> joint_names;
    RuntimeCfg runtime;
    ShutdownCfg shutdown;
    JointCtrllerCfg ctrller;
    JointActuatorMapCfg mapper;
    SafetyCfg safety;
    DynamicsCfg dynamics;
};
```

YAML 采用分区格式；Loader 最终解析为扁平 `RobotCfg`

### 5.2. RuntimeCfg

```cpp
struct RuntimeCfg {
    double ctrl_frequency_hz{ 200.0 };
    double joint_acc_filter_alpha{ 0.2 };
    bool write_enabled{ false };
    ModelFeedforwardMode model_feedforward_mode{ ModelFeedforwardMode::NONE };
};
```

### 5.3. ShutdownCfg

```cpp
struct ShutdownCfg {
    bool park_before_disable{ true };
    JointVector park_pos;
    double speed_scale{ 0.1 };
    double position_tolerance{ 0.03 };
    double velocity_tolerance{ 0.05 };
    double settle_time_s{ 0.25 };
    double relaxed_tolerance_ratio{ 2.0 };
    double timeout_s{ 15.0 };
};
```

该配置供应用层生成停放轨迹；`Robot::deactivate()` 不读取 `park_pos`

### 5.4. JointCtrllerCfg

```cpp
struct JointCtrllerCfg {
    std::size_t joints_count{ 0 };
    JointImpedanceGains rigid_hold_gains;
    JointImpedanceGains rigid_tracking_gains;
    JointImpedanceGains compliant_hold_gains;
    JointImpedanceGains compliant_drag_gains;
    JointImpedanceGains compliant_tracking_gains;
    bool allow_full_cmd{ false };
};
```

### 5.5. JointActuatorMapCfg

```cpp
struct JointActuatorMapCfg {
    std::size_t joints_count{ 0 };
    ActuatorVector pos_ratio;
    ActuatorVector tor_ratio;
    std::vector<int> direction;
    JointVector joint_zero_offset;
    ActuatorVector actuator_zero_offset;
};
```

### 5.6. SafetyCfg

```cpp
struct JointLimitCfg {
    JointVector min_pos;
    JointVector max_pos;
    JointVector max_vel;
    JointVector max_acc;
    JointVector max_effort;
    JointVector max_kp;
    JointVector max_kd;
    JointVector pos_margin;
};

struct SafetyCfg {
    std::size_t joints_count{ 0 };
    JointLimitCfg limits;
    double cmd_timeout_s{ 0.10 };
    double state_timeout_s{ 0.05 };
    double max_dt_s{ 0.02 };
    double numeric_tolerance{ 1.0e-6 };
    double state_vel_fault_ratio{ 1.5 };
    bool require_all_actuators_online{ true };
    bool require_all_actuators_enabled{ true };
    bool reject_motor_error{ true };
    bool require_continuous_cmd{ true };
};
```

### 5.7. DynamicsCfg

```cpp
struct DynamicsCfg {
    std::string urdf_path;
    std::vector<std::string> joint_names;
    std::string base_frame{ "base_link" };
    std::string tool_frame{ "tool0" };
    std::array<double, 3> gravity{ 0.0, 0.0, -9.81 };
    JointVector gravity_scale;
};
```

### 5.8. load_robot_cfg()

```cpp
tl::expected<RobotCfg, ConfigErrInfo> load_robot_cfg(
    const std::string& path,
    const HardwareCapabilities& capabilities);
```

行为

1. 解析 YAML
2. 按 Joint 名称读取分区字段
3. 加载 URDF Model 信息
4. 使用调用方提供的 Hardware Capability
5. 运行 LimitResolver
6. 生成最终 `SafetyCfg`
7. 验证 Core 配置

### 5.10. compare_robot_cfg()

```cpp
tl::expected<std::vector<std::string>, ConfigErrInfo> compare_robot_cfg(const std::string& lhs_path, const std::string& rhs_path);
```

返回两个最终 `RobotCfg` 的字段差异；不连接硬件

其他验证函数

```cpp
tl::expected<void, ConfigErrInfo> validate_robot_core_cfg(const RobotCfg& cfg);
tl::expected<void, ConfigErrInfo> validate_robot_cfg(const RobotCfg& cfg);
```

## 6. ModelLoader

头文件

```cpp
#include <serial_arm/model/model_loader.hpp>
```

### 6.1. RobotModelInfo

```cpp
struct RobotModelInfo {
    std::string urdf_path;
    std::vector<std::string> joint_names;
    std::vector<ModelJointLimit> joint_limits;
};
```

### 6.2. ModelJointLimit

```cpp
struct ModelJointLimit {
    std::string name;
    bool has_position_limit{ false };
    double min_pos{ 0.0 };
    double max_pos{ 0.0 };
    double max_vel{ 0.0 };
    double max_effort{ 0.0 };
};
```

### 6.3. ModelLoader::load()

```cpp
tl::expected<RobotModelInfo, ModelErr> load(const std::string& urdf_path, const std::vector<std::string>& controlled_joint_names) const;
```

输出顺序严格匹配 `controlled_joint_names`

### 6.4. URDF 限位语义

- revolute Joint 必须具有合法 lower、upper、velocity 和 effort
- continuous Joint 的 `has_position_limit` 为 false
- fixed Joint 不能进入受控列表
- 所有数值必须有限

### 6.5. ModelErr

```text
FILE_OPEN_FAILED
URDF_LOAD_FAILED
MISSING_JOINT
DUPLICATE_JOINT
FIXED_JOINT_CONTROLLED
INVALID_LIMIT
```

## 7. Hardware Capability

头文件

```cpp
#include <serial_arm/hardware/hardware_capability.hpp>
```

### 7.1. ActuatorCapability

```cpp
struct ActuatorCapability {
    std::string actuator_name;
    double min_pos{ 0.0 };
    double max_pos{ 0.0 };
    double max_vel{ 0.0 };
    double max_effort{ 0.0 };
    double max_kp{ 0.0 };
    double max_kd{ 0.0 };
};
```

### 7.2. HardwareCapabilities

```cpp
using HardwareCapabilities = std::vector<ActuatorCapability>;
```

### 7.3. 能力来源

`HardwareCapabilities` 由具体 Hardware Backend 在 `configure(config_path)` 后通过 `MotorBus::capabilities()` 提供；Core 只消费这些通用能力值，不解析后端私有配置

## 8. LimitResolver

头文件

```cpp
#include <serial_arm/config/limit_resolver.hpp>
```

### 8.1. SafetyPolicyCfg

```cpp
struct SafetyPolicyCfg {
    double position_margin{ 0.0 };
    double cmd_vel_scale{ 1.0 };
    double state_vel_scale{ 1.0 };
    JointVector max_acc;
    JointVector max_effort_override;
    JointVector max_kp_override;
    JointVector max_kd_override;
    double max_dt_s{ 0.0 };
    double state_timeout_s{ 0.0 };
    double cmd_timeout_s{ 0.0 };
    bool require_all_actuators_online{ true };
    bool require_all_actuators_enabled{ true };
    bool reject_motor_error{ true };
    bool require_continuous_cmd{ true };
};
```

### 8.2. ResolvedJointLimitCfg

```cpp
struct ResolvedJointLimitCfg {
    std::string joint_name;
    bool has_position_limit{ false };
    double hard_min_pos{ 0.0 };
    double hard_max_pos{ 0.0 };
    double cmd_min_pos{ 0.0 };
    double cmd_max_pos{ 0.0 };
    double max_cmd_vel{ 0.0 };
    double max_state_vel{ 0.0 };
    double max_acc{ 0.0 };
    double max_effort{ 0.0 };
    double max_kp{ 0.0 };
    double max_kd{ 0.0 };
};
```

### 8.3. ResolvedSafetyCfg

```cpp
struct ResolvedSafetyCfg {
    std::vector<ResolvedJointLimitCfg> joints;
    double max_dt_s{ 0.0 };
    double state_timeout_s{ 0.0 };
    double cmd_timeout_s{ 0.0 };
    bool require_all_actuators_online{ true };
    bool require_all_actuators_enabled{ true };
    bool reject_motor_error{ true };
    bool require_continuous_cmd{ true };
};
```

### 8.4. resolve()

```cpp
tl::expected<ResolvedSafetyCfg, LimitResolverErr> resolve(
    const RobotModelInfo& model,
    const JointActuatorMapCfg& mapper,
    const HardwareCapabilities& capabilities,
    const SafetyPolicyCfg& policy) const;
```

### 8.5. 限位解析规则

- `hard_min_pos` 和 `hard_max_pos` 来自 URDF
- `cmd_min_pos` 和 `cmd_max_pos` 由硬限位收窄
- `max_cmd_vel = urdf_velocity × cmd_vel_scale`
- `max_state_vel = urdf_velocity × state_vel_scale`
- Joint effort 结合 URDF effort、Actuator capability 和映射比例
- Joint kp、kd 结合 Actuator 上限和映射比例
- Override 只允许收窄

兼容转换

```cpp
SafetyCfg to_safety_cfg(const ResolvedSafetyCfg& resolved);
ResolvedSafetyCfg resolve_from_safety_cfg(const std::vector<std::string>& joint_names, const SafetyCfg& cfg);
```

### 8.6. LimitResolverErr

```text
INVALID_INPUT
MISSING_ACTUATOR
POLICY_WIDENS_LIMIT
```

## 9. JointCtrller

头文件

```cpp
#include <serial_arm/core/joints_ctrller.hpp>
```

### 9.1. configure()

```cpp
tl::expected<void, JointCtrllerErr> configure(const JointCtrllerCfg& cfg);
```

状态从 `UNCONFIGURED` 进入 `CONFIGURED`

### 9.2. initialize()

```cpp
tl::expected<void, JointCtrllerErr> initialize(const JointState& state);
```

使用真实状态初始化保持参考；进入 `INITIALIZED`

### 9.3. set_impedance_mode()

```cpp
tl::expected<void, JointCtrllerErr> set_impedance_mode(JointImpedanceMode mode, const JointState& state);
```

保持和拖拽模式在切换时锁存当前状态

### 9.4. set_cmd()

```cpp
tl::expected<void, JointCtrllerErr> set_cmd(const JointCmd& cmd);
```

仅跟踪模式接受外部参考

### 9.5. set_full_cmd()

```cpp
tl::expected<void, JointCtrllerErr> set_full_cmd(const JointCtrlCmd& cmd);
```

要求 `allow_full_cmd=true`

### 9.6. update()

```cpp
tl::expected<JointCtrllerOutput, JointCtrllerErr> update(const JointCtrllerInput& input);
```

控制器输出 Joint 侧完整命令；模型前馈叠加到 `tor`

Getter

```cpp
JointCtrllerState get_state() const noexcept;
JointImpedanceMode get_impedance_mode() const noexcept;
```

### 9.7. JointCtrllerErr

```text
OK
NOT_CONFIGURED
NOT_INITIALIZED
ALREADY_INITIALIZED
INVALID_CFG
INVALID_STATE
INVALID_DT
INVALID_MODEL_FEEDFORWARD
INVALID_IMPEDANCE_MODE
INVALID_CMD_SIZE
INVALID_CMD_VALUE
INVALID_FULL_CMD
CMD_NOT_ALLOWED_IN_MODE
FULL_CMD_NOT_ALLOWED
```

## 10. JointActuatorMapper

头文件

```cpp
#include <serial_arm/core/joint_actuator_mapper.hpp>
```

### 10.1. configure()

```cpp
tl::expected<void, JointActuatorMapErr> configure(const JointActuatorMapCfg& cfg);
```

### 10.2. to_joint_state()

```cpp
tl::expected<JointState, JointActuatorMapErr> to_joint_state(const ActuatorState& actuator_state) const;
```

### 10.3. to_actuator_cmd()

```cpp
tl::expected<ActuatorCtrlCmd, JointActuatorMapErr> to_actuator_cmd(const JointCtrlCmd& joint_cmd) const;
```

### 10.4. 比例、方向与零位

定义

```text
pos_ratio = 执行器位置变化量 / Joint 位置变化量
tor_ratio = Joint 力矩 / 执行器报告力矩
```

位置映射

```text
q_actuator = direction × pos_ratio × (q_joint - joint_zero_offset) + actuator_zero_offset
```

执行器 SDK 已经输出减速器输出端数据时，`pos_ratio` 和 `tor_ratio` 通常为 1

Getter

```cpp
std::size_t size() const noexcept;
```

### 10.5. JointActuatorMapErr

```text
OK
NOT_CONFIGURED
INVALID_CFG
INVALID_JOINT_STATE
INVALID_ACTUATOR_STATE
INVALID_JOINT_CMD
INVALID_ACTUATOR_CMD
INVALID_CONVERSION_VALUE
```

## 11. Safety

头文件

```cpp
#include <serial_arm/core/safety.hpp>
```

### 11.1. configure()

```cpp
tl::expected<void, SafetyFault> configure(const SafetyCfg& cfg);
```

### 11.2. check_state()

```cpp
tl::expected<void, SafetyFault> check_state(const JointState& joint_state, const ActuatorState& actuator_state, double state_age_s) const;
```

检查数组长度、有限值、Joint 位置、状态速度、Actuator 在线、使能和错误码

### 11.3. check_joint_cmd()

```cpp
tl::expected<JointCtrlCmd, SafetyFault> check_joint_cmd(const JointState& state, const JointCtrlCmd& cmd, double dt);
```

检查位置、速度、effort、kp、kd 和可选连续命令约束；浮点误差级越界可被小范围 clamp

### 11.4. on_timeout()

当前公开 API 将超时拆为

```cpp
tl::expected<void, SafetyFault> check_cmd_age(double cmd_age_s) const;
```

状态超时通过 `check_state(..., state_age_s)` 检查

### 11.5. SafetyFault 与 SafetyAction

```cpp
struct SafetyFault {
    SafetyErr code;
    std::size_t index;
    double value;
    double limit;
};

enum class SafetyAction {
    STOP_HOLD,
    DISABLE,
};
```

```cpp
SafetyAction action_for(SafetyErr err) const noexcept;
```

命令历史

```cpp
tl::expected<void, SafetyFault> reset_cmd_history(const JointState& state);
void clear_cmd_history() noexcept;
```

状态查询

```cpp
bool is_configured() const noexcept;
std::uint64_t clamp_count() const noexcept;
```

### 11.6. SafetyErr

```text
NOT_CONFIGURED
INVALID_CFG
INVALID_DT
INVALID_STATE_AGE
INVALID_CMD_AGE
STATE_TIMEOUT
CMD_TIMEOUT
INVALID_JOINT_STATE_SIZE
INVALID_ACTUATOR_STATE_SIZE
NON_FINITE_JOINT_STATE
NON_FINITE_ACTUATOR_STATE
JOINT_POS_LIMIT
JOINT_VEL_LIMIT
ACTUATOR_OFFLINE
ACTUATOR_NOT_ENABLED
ACTUATOR_FAULT
INVALID_CMD_SIZE
NON_FINITE_CMD
CMD_POS_LIMIT
CMD_VEL_LIMIT
CMD_EFFORT_LIMIT
CMD_KP_LIMIT
CMD_KD_LIMIT
CMD_POS_STEP_LIMIT
CMD_VEL_STEP_LIMIT
```

## 12. MotorBus

头文件

```cpp
#include <serial_arm/hardware/motor_bus.hpp>
```

### 12.1. 接口职责

`MotorBus` 是执行器后端抽象；Robot 通过它读取 `ActuatorState` 并写入 `ActuatorCtrlCmd`

### 12.2. 生命周期

```cpp
virtual tl::expected<void, MotorBusErr> connect() = 0;
virtual tl::expected<ActuatorState, MotorBusErr> read() = 0;
virtual tl::expected<void, MotorBusErr> activate() = 0;
virtual tl::expected<void, MotorBusErr> write(const ActuatorCtrlCmd& cmd) = 0;
virtual tl::expected<void, MotorBusErr> stop() = 0;
virtual tl::expected<void, MotorBusErr> deactivate() = 0;
virtual tl::expected<void, MotorBusErr> recover() = 0;
virtual void cleanup() noexcept = 0;
virtual std::size_t size() const noexcept = 0;
```

### 12.3. read() 与 write()

- `read()` 返回执行器侧状态
- `write()` 接收已经映射完成的 `ActuatorCtrlCmd`
- 后端不应重复实现 Joint Safety

### 12.4. deactivate() 与 recover()

- `stop()` 发送低风险停止保持
- `deactivate()` 停止并失能
- `recover()` 清理旧通信状态并恢复到可重新激活状态

### 12.5. MotorBusErr

```text
NOT_CONFIGURED
NOT_CONNECTED
NOT_ACTIVE
INVALID_CFG
OPEN_FAILED
READ_FAILED
WRITE_FAILED
INVALID_STATE
INVALID_CMD
ACTUATOR_OFFLINE
ACTUATOR_FAULT
TIMEOUT
ENABLE_FAILED
MODE_SWITCH_FAILED
STOP_FAILED
DISABLE_FAILED
RECOVER_FAILED
```

## 13. HardwareLoader 与 Backend ABI

头文件

```cpp
#include <serial_arm/hardware/hardware_loader.hpp>
```

### 13.1. 动态加载

```cpp
serial_arm::HardwareLoader loader;
auto bus = loader.load("serial_arm_hardware_damiao", "hardware.yaml");
```

`load()` 会 `dlopen()` 后端共享库，解析 `create_motor_bus` / `destroy_motor_bus`，创建 `MotorBus` 并调用 `configure(hardware_config)`；调用方必须保证 `HardwareLoader` 的生命周期长于该后端对象

### 13.2. Backend ABI

```cpp
extern "C" serial_arm::MotorBus* create_motor_bus();
extern "C" void destroy_motor_bus(serial_arm::MotorBus* bus);
```

Backend 可包含任意厂商协议和私有配置结构，但不能要求 Core 暴露这些类型

## 14. Dynamics

头文件

```cpp
#include <serial_arm/dynamics/dynamics.hpp>
```

### 14.1. configure()

```cpp
tl::expected<void, DynamicsErr> configure(const DynamicsCfg& cfg);
```

加载 URDF，构造受控 Joint reduced model，解析 base 和 tool Frame

### 14.2. update()

```cpp
tl::expected<void, DynamicsErr> update(const JointState& state, const JointVector& acc, const JointVector& ref_acc);
```

一次性更新

```text
q、dq、ddq、反馈力矩、参考加速度
gravity、gravity_compensation
nonlinear、coriolis
mass_matrix
inverse_dynamics、forward_dynamics
tool_pose、tool_jacobian
```

### 14.3. Gravity 与 Nonlinear

```cpp
const JointVector& get_gravity() const noexcept;
const JointVector& get_gravity_compensation() const noexcept;
const JointVector& get_nonlinear() const noexcept;
const JointVector& get_coriolis() const noexcept;
```

### 14.4. Mass Matrix

```cpp
const Eigen::MatrixXd& get_mass_matrix() const noexcept;
```

### 14.5. Inverse 与 Forward Dynamics

```cpp
const JointVector& get_inverse_dynamics() const noexcept;
const JointVector& get_forward_dynamics() const noexcept;
```

逆动力学使用 `ref_acc`；正动力学使用反馈力矩

### 14.6. Pose 与 Jacobian

```cpp
const Eigen::Isometry3d& get_tool_pose() const noexcept;
const Eigen::MatrixXd& get_tool_jacobian() const noexcept;
tl::expected<Eigen::Isometry3d, DynamicsErr> get_frame_pose(const std::string& frame_name) const;
tl::expected<Eigen::MatrixXd, DynamicsErr> get_frame_jacobian(const std::string& frame_name) const;
```

Frame getter 读取最近一次 `update()` 的缓存

### 14.7. gravity_scale

```cpp
tl::expected<void, DynamicsErr> set_gravity_scale(const JointVector& gravity_scale);
const JointVector& get_gravity_scale() const noexcept;
```

每轴范围为 `[0, 1]`

其他状态

```cpp
bool is_configured() const noexcept;
bool is_updated() const noexcept;
const DynamicsInfo& get_info() const noexcept;
const DynamicsState& get_state() const noexcept;
void cleanup();
```

### 14.8. DynamicsErr

```text
NOT_CONFIGURED
ALREADY_CONFIGURED
NOT_UPDATED
INVALID_CFG
URDF_LOAD_FAILED
JOINT_NOT_FOUND
JOINT_NOT_1DOF
MODEL_SIZE_MISMATCH
FRAME_NOT_FOUND
INVALID_INPUT_SIZE
NON_FINITE_INPUT
GRAVITY_SCALE_OUT_OF_RANGE
COMPUTE_FAILED
```

## 15. Robot

头文件

```cpp
#include <serial_arm/robot.hpp>
```

### 15.1. configure()

```cpp
tl::expected<void, RobotFault> configure(const RobotCfg& cfg, std::unique_ptr<MotorBus> motor_bus, ModelFeedforwardFn model_feedforward = {});
```

Robot 接管 MotorBus；配置 JointCtrller、Mapper 和 Safety；不连接硬件

### 15.2. activate()

```cpp
tl::expected<void, RobotFault> activate();
```

连接和使能硬件，读取真实状态，初始化控制器和 Safety 命令历史

### 15.3. cycle()

```cpp
tl::expected<RobotCycleOutput, RobotFault> cycle(TimePoint now = Clock::now());
```

调用者负责固定频率调度；时间戳必须单调

### 15.4. set_cmd()

```cpp
tl::expected<void, RobotFault> set_cmd(const JointCmd& cmd, TimePoint now = Clock::now());
```

更新跟踪参考和命令时间戳

### 15.5. set_full_cmd()

```cpp
tl::expected<void, RobotFault> set_full_cmd(const JointCtrlCmd& cmd, TimePoint now = Clock::now());
```

要求 Controller 允许完整命令

### 15.6. set_impedance_mode()

```cpp
tl::expected<void, RobotFault> set_impedance_mode(JointImpedanceMode mode, TimePoint now = Clock::now());
```

### 15.7. set_model_feedforward_mode()

```cpp
tl::expected<void, RobotFault> set_model_feedforward_mode(ModelFeedforwardMode mode);
```

仅 INACTIVE 状态可切换

### 15.8. deactivate() 与 force_deactivate()

```cpp
tl::expected<void, RobotFault> deactivate();
tl::expected<void, RobotFault> force_deactivate();
```

`deactivate()` 执行正常停止失能；`force_deactivate()` 允许从 ACTIVE 或 FAULT 强制失能

### 15.9. reset_fault()

```cpp
tl::expected<void, RobotFault> reset_fault();
```

根据当前 FAULT 保持状态恢复到 ACTIVE 或 INACTIVE；调用者应检查最终 `get_state()`

### 15.10. maintain_fault_hold()

```cpp
tl::expected<void, RobotFault> maintain_fault_hold();
```

仅 FAULT 刚性保持有效时调用；必须由外层 Worker 持续刷新

### 15.11. RobotFault 与 RobotErr

```cpp
struct RobotFault {
    RobotErr code;
    MotorBusErr motor_bus_err;
    JointActuatorMapErr mapper_err;
    JointCtrllerErr ctrller_err;
    SafetyFault safety_fault;
    ModelFeedforwardErr model_feedforward_err;
};
```

Getter

```cpp
RobotState get_state() const noexcept;
JointImpedanceMode get_impedance_mode() const noexcept;
ModelFeedforwardMode get_model_feedforward_mode() const noexcept;
const JointState& get_joint_state() const noexcept;
const JointVector& get_joint_acc() const noexcept;
const JointVector& get_joint_ref_acc() const noexcept;
const JointVector& get_model_feedforward() const noexcept;
const ActuatorState& get_actuator_state() const noexcept;
const tl::optional<RobotFault>& get_last_fault() const noexcept;
bool is_fault_holding() const noexcept;
```

RobotErr

```text
NOT_CONFIGURED
ALREADY_CONFIGURED
INVALID_CFG
NULL_MOTOR_BUS
MOTOR_BUS_SIZE_MISMATCH
WRITE_DISABLED
NOT_ACTIVE
NOT_INACTIVE
ALREADY_ACTIVE
FAULTED
NOT_FAULTED
INVALID_TIME
MOTOR_BUS_CONNECT_FAILED
MOTOR_BUS_ACTIVATE_FAILED
MOTOR_BUS_READ_FAILED
MOTOR_BUS_WRITE_FAILED
MOTOR_BUS_DEACTIVATE_FAILED
MOTOR_BUS_RECOVER_FAILED
MAPPER_FAILED
CTRLLER_FAILED
SAFETY_FAILED
MODEL_FEEDFORWARD_FAILED
INVALID_MODEL_FEEDFORWARD
```

## 16. 控制循环边界

### 16.1. Robot::cycle() 单周期语义

Core 实现控制周期内容，不实现固定调度线程

### 16.2. runtime.ctrl_frequency_hz

```cpp
const double target_dt = 1.0 / cfg.runtime.ctrl_frequency_hz;
```

该值是目标频率；实际周期由系统调度、串口、动力学计算和锁竞争共同决定

### 16.3. C++ Terminal Worker

终端 Worker 持有 Robot 外层互斥；菜单线程只提交命令和读取缓存

### 16.4. Python RobotSession Worker

C++ Worker 独占 Robot；Python 方法通过请求和快照与其交互

### 16.5. ros2_control Worker

`SerialArmSystem` Worker 独占 Robot；`read()` 与 `write()` 只复制缓存；不得让 controller_manager 和 Worker 同时访问 Robot

## 17. Python Binding

Python 包

```python
import serial_arm
```

### 17.1. 模块结构

```text
serial_arm/__init__.py
_serial_arm*.so
```

`_serial_arm*.so` 是 CMake 和 pybind11 编译的扩展；`__init__.py` 提供 Python 封装和公开导出

### 17.2. Config Binding

Python 公开

```python
load_robot_cfg(path, hardware_plugin, hardware_config)
validate_robot_core_cfg(cfg)
validate_robot_cfg(cfg)
```

`compare_robot_cfg()` 当前未导出到 Python

### 17.3. Dynamics Binding

```python
dynamics = serial_arm.Dynamics()
dynamics.configure(cfg.dynamics)
dynamics.update(pos, vel, acc, tor, ref_acc)
```

NumPy 输入要求可转换为连续 `float64` 一维数组

### 17.4. RobotSession

```python
session = serial_arm.RobotSession(config_file, hardware_plugin, hardware_config)
```

构造阶段加载配置和构建 C++ 对象，不激活真机

### 17.5. RobotSessionSnapshot

包含

```text
robot_state
cycle
dynamics
valid
last_error
```

快照返回副本；Python 不持有 C++ 实时缓存引用

### 17.6. NumPy 与 GIL 语义

- 数组输入转换为连续 `numpy.float64`
- C++ 长操作在绑定中按需要释放 GIL
- Worker 不持有 Python GIL
- Python 回调不进入 200 Hz 周期

### 17.7. SerialArmError

C++ `tl::expected` 错误在 Python 中转换为统一 `SerialArmError`

RobotSession 主要方法

```python
start()
stop()
reset_fault()
set_impedance_mode(mode)
set_model_feedforward_mode(mode)
set_gravity_scale(scale)
move_to(pos, speed_scale=0.3)
hold_current()
```

属性

```python
snapshot
state
configured
running
config
dynamics_info
actuator_info
```

`stop()` 不生成停放轨迹；应用层应先 `move_to(park_pos)` 和 `hold_current()`

## 18. Adapter 接入契约

### 18.1. Robot 所有权

每个 Adapter 实例必须独占一个 Robot 和一个 MotorBus；禁止多线程直接共享 Robot

### 18.2. Command 缓存

上层命令应先进入固定大小缓存；控制 Worker 每周期读取最近合法命令并调用 `Robot::set_cmd()`

### 18.3. State 快照

Worker 在 `Robot::cycle()` 成功后更新快照；上层只读取快照，不直接访问硬件

### 18.4. 生命周期映射

推荐映射

```text
Adapter configure → Robot::configure
Adapter activate  → Robot::activate + Worker start
Adapter deactivate→ Worker stop + Robot::deactivate
Adapter cleanup   → Robot 和后端析构
```

### 18.5. Safety 边界

Adapter 不得绕过

- `Robot::set_cmd()`
- `Safety::check_state()`
- `Safety::check_joint_cmd()`
- `JointActuatorMapper`

上层规划限制不能替代 Core Safety

## 19. 示例

### 19.1. 加载配置

```cpp
serial_arm::HardwareLoader config_loader;
auto config_bus = config_loader.load("serial_arm_hardware_damiao", "dm_arm_damiao.yaml");
if(!config_bus) return 1;

const auto result = serial_arm::load_robot_cfg(
    "dm_arm_white.yaml",
    config_bus.value()->capabilities());
if(!result) return 1;
const auto cfg = result.value();
```

### 19.2. 离线 Dynamics

```cpp
serial_arm::Dynamics dynamics;
if(!dynamics.configure(cfg.dynamics)) return 1;

serial_arm::JointState state;
state.pos.assign(cfg.joint_names.size(), 0.0);
state.vel.assign(cfg.joint_names.size(), 0.0);
state.tor.assign(cfg.joint_names.size(), 0.0);

serial_arm::JointVector acc(cfg.joint_names.size(), 0.0);
serial_arm::JointVector ref_acc(cfg.joint_names.size(), 0.0);
if(!dynamics.update(state, acc, ref_acc)) return 1;
```

### 19.3. C++ Robot

```cpp
serial_arm::HardwareLoader hardware_loader;
auto bus = hardware_loader.load("serial_arm_hardware_damiao", "dm_arm_damiao.yaml");
if(!bus) return 1;

serial_arm::Dynamics dynamics;
if(!dynamics.configure(cfg.dynamics)) return 1;

serial_arm::ModelFeedforwardFn feedforward = [&dynamics](serial_arm::ModelFeedforwardMode mode, const serial_arm::JointState& state, const serial_arm::JointVector& acc, const serial_arm::JointVector& ref_acc, double) {
    const auto update_result = dynamics.update(state, acc, ref_acc);
    if(!update_result) return tl::expected<serial_arm::JointVector, serial_arm::ModelFeedforwardErr>(tl::make_unexpected(serial_arm::ModelFeedforwardErr::COMPUTE_FAILED));
    if(mode == serial_arm::ModelFeedforwardMode::GRAVITY) return tl::expected<serial_arm::JointVector, serial_arm::ModelFeedforwardErr>(dynamics.get_gravity_compensation());
    if(mode == serial_arm::ModelFeedforwardMode::FULL_INVERSE_DYNAMICS) return tl::expected<serial_arm::JointVector, serial_arm::ModelFeedforwardErr>(dynamics.get_inverse_dynamics());
    return tl::expected<serial_arm::JointVector, serial_arm::ModelFeedforwardErr>(serial_arm::JointVector(state.pos.size(), 0.0));
};

serial_arm::Robot robot;
if(!robot.configure(cfg, std::move(bus), std::move(feedforward))) return 1;
```

### 19.4. Python RobotSession

```python
import serial_arm
import numpy as np

session = serial_arm.RobotSession(
    "dm_arm_white.yaml",
    "serial_arm_hardware_damiao",
    "dm_arm_damiao.yaml",
)
session.set_model_feedforward_mode(serial_arm.ModelFeedforwardMode.GRAVITY)
session.start()
session.move_to(np.zeros(6), speed_scale=0.1)
```

### 19.5. 下游 CMake

```cmake
cmake_minimum_required(VERSION 3.20)
project(dm_arm_app LANGUAGES CXX)

find_package(pinocchio CONFIG REQUIRED)
find_package(serial_arm_core CONFIG REQUIRED)

add_executable(dm_arm_app main.cpp)
target_link_libraries(dm_arm_app PRIVATE
    serial_arm::config
    serial_arm::robot
    serial_arm::dynamics
)
target_compile_features(dm_arm_app PRIVATE cxx_std_17)
```

## 20. 错误码索引

### 20.1. ConfigErr

```text
FILE_OPEN_FAILED
SYNTAX_ERROR
MISSING_FIELD
INVALID_VALUE
INVALID_SIZE
DUPLICATE_NAME
```

### 20.2. ModelErr

```text
FILE_OPEN_FAILED
URDF_LOAD_FAILED
MISSING_JOINT
DUPLICATE_JOINT
FIXED_JOINT_CONTROLLED
INVALID_LIMIT
```

### 20.3. LimitResolverErr

```text
INVALID_INPUT
MISSING_ACTUATOR
POLICY_WIDENS_LIMIT
```

### 20.5. JointCtrllerErr

见 `9.7. JointCtrllerErr`

### 20.6. JointActuatorMapErr

见 `10.5. JointActuatorMapErr`

### 20.7. SafetyErr

见 `11.6. SafetyErr`

### 20.8. MotorBusErr

见 `12.5. MotorBusErr`

### 20.9. DynamicsErr

见 `14.8. DynamicsErr`

### 20.10. RobotErr

见 `15.11. RobotFault 与 RobotErr`
