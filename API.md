# DM-Arm API 文档

命名空间为 `dm_arm`；语言标准为 C++17

## 1. CMake targets

### `dm_arm::core`

头文件

```cpp
#include <dm_arm/core/types.hpp>
#include <dm_arm/core/joints_ctrller.hpp>
#include <dm_arm/core/joint_actuator_mapper.hpp>
#include <dm_arm/core/safety.hpp>
```

能力

- Joint 和 Actuator 数据类型
- 阻抗模式
- 关节控制器
- Joint 与 Actuator 映射
- Safety

### `dm_arm::config`

头文件

```cpp
#include <dm_arm/config/config.hpp>
```

能力

- `RuntimeCfg`
- `DamiaoBusCfg`
- `DynamicsCfg`
- `RobotCfg`
- YAML 加载
- 配置验证

依赖

```text
dm_arm::core
yaml-cpp
```

### `dm_arm::robot`

头文件

```cpp
#include <dm_arm/robot.hpp>
```

能力

- 生命周期
- 周期闭环
- 故障锁存
- 关节加速度估计
- 参考加速度估计
- 模型前馈回调

依赖

```text
dm_arm::core
dm_arm::config
```

### `dm_arm::damiao`

头文件

```cpp
#include <dm_arm/hardware/damiao_motor_bus.hpp>
```

生成条件

```cmake
-DM_ARM_BUILD_DAMIAO=ON
```

能力

- 达妙串口连接
- MIT 控制
- 状态读取
- 使能和失能
- stop 和 recover
- 执行器静态信息

### `dm_arm::dynamics`

头文件

```cpp
#include <dm_arm/dynamics/dynamics.hpp>
```

生成条件

```cmake
-DDM_ARM_ENABLE_DYNAMICS=ON
```

能力

- URDF reduced model
- 集中式 `update()`
- Frame 位姿和 Jacobian
- 重力项
- 非线性项
- 科氏和离心项
- 质量矩阵
- RNEA
- ABA

## 2. 通用约定

### 2.1. 命名空间

```cpp
namespace dm_arm
```

### 2.2. Joint 顺序

当前系统固定六轴；所有向量按 `RobotCfg::joint_names` 顺序排列

默认配置

```text
joint1, joint2, joint3, joint4, joint5, joint6
```

### 2.3. 单位

| 数据 | Joint 侧单位 |
|---|---|
| 位置 | rad |
| 速度 | rad/s |
| 加速度 | rad/s² |
| 力矩 | N·m |
| `kp` | Joint 侧位置增益 |
| `kd` | Joint 侧速度增益 |

Actuator 侧语义由具体 MotorBus 定义；达妙后端使用 SDK MIT 语义

### 2.4. 错误返回

主要操作使用

```cpp
tl::expected<T, Error>
```

示例

```cpp
const auto result = robot.activate();
if(!result) {
    const dm_arm::RobotFault& fault = result.error();
}
```

### 2.5. 所有权

`Robot::configure()` 接收 `std::unique_ptr<MotorBus>`；成功后 Robot 独占硬件后端

### 2.6. 线程安全

当前公开类不提供内部互斥锁；生命周期、命令和 `cycle()` 应由应用层串行化

### 2.7. Dynamics 缓存语义

`Dynamics::update()` 负责计算；getter 只读取最近一次成功更新的缓存

```cpp
const auto update_result = dynamics.update(state, acc, ref_acc);
if(!update_result) {
    return;
}

const auto& gravity = dynamics.get_gravity();
const auto& mass_matrix = dynamics.get_mass_matrix();
```

## 3. 基础类型

头文件

```cpp
#include <dm_arm/core/types.hpp>
```

### 3.1. 向量

```cpp
using JointVector = std::vector<double>;
using ActuatorVector = std::vector<double>;
```

### 3.2. 阻抗模式

```cpp
enum class JointImpedanceMode {
    RIGID_HOLD,
    RIGID_TRACKING,
    COMPLIANT_HOLD,
    COMPLIANT_DRAG,
    COMPLIANT_TRACKING,
};
```

| 模式 | 外部命令 | 语义 |
|---|---:|---|
| `RIGID_HOLD` | 否 | 切换时位置刚性保持 |
| `RIGID_TRACKING` | 是 | 刚性跟踪外部参考 |
| `COMPLIANT_HOLD` | 否 | 切换时位置柔性保持 |
| `COMPLIANT_DRAG` | 否 | 当前实际位置和低阻尼拖拽 |
| `COMPLIANT_TRACKING` | 是 | 柔性跟踪外部参考 |

### 3.3. 模型前馈模式

```cpp
enum class ModelFeedforwardMode {
    NONE,
    GRAVITY,
    FULL_INVERSE_DYNAMICS,
};
```

### 3.4. 状态结构

```cpp
struct JointState {
    JointVector pos;
    JointVector vel;
    JointVector tor;
};
```

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

### 3.5. 参考命令

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
```

```cpp
using JointCmd = std::variant<JointPosCmd, JointPosVelCmd, JointPosVelTorCmd>;
```

### 3.6. 完整命令

```cpp
struct JointCtrlCmd {
    JointVector pos;
    JointVector vel;
    JointVector tor;
    JointVector kp;
    JointVector kd;
};
```

```cpp
struct ActuatorMitCmd {
    ActuatorVector pos;
    ActuatorVector vel;
    ActuatorVector tor;
    ActuatorVector kp;
    ActuatorVector kd;
};
```

## 4. 配置 API

头文件

```cpp
#include <dm_arm/config/config.hpp>
```

### 4.1. `RuntimeCfg`

```cpp
struct RuntimeCfg {
    double ctrl_frequency_hz{ 200.0 };
    double joint_acc_filter_alpha{ 0.2 };
    bool write_enabled{ false };
    ModelFeedforwardMode model_feedforward_mode{ ModelFeedforwardMode::NONE };
};
```

### 4.2. `DamiaoActuatorCfg`

```cpp
struct ShutdownCfg {
    bool park_before_disable{ true };
    JointVector park_pos;
    double speed_scale{ 0.1 };
    double position_tolerance{ 0.02 };
    double velocity_tolerance{ 0.03 };
    double settle_time_s{ 0.5 };
    double timeout_s{ 15.0 };
};

struct DamiaoActuatorCfg {
    std::string name;
    std::string joint_name;
    std::uint32_t motor_id{ 0 };
    std::uint32_t master_id{ 0 };
    std::string motor_type;
};
```

### 4.3. `DamiaoBusCfg`

```cpp
struct DamiaoBusCfg {
    std::string serial_port{ "/dev/ttyACM0" };
    int baudrate{ 921600 };
    bool refresh_state_in_read{ false };
    double feedback_timeout_s{ 0.05 };
    std::size_t startup_read_cycles{ 5 };
    double stop_kp{ 3.0 };
    double stop_kd{ 0.1 };
    std::size_t stop_cycles{ 5 };
    std::vector<DamiaoActuatorCfg> actuators;
};
```

### 4.4. `DynamicsCfg`

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

### 4.5. `RobotCfg`

```cpp
struct RobotCfg {
    std::vector<std::string> joint_names;
    RuntimeCfg runtime;
    ShutdownCfg shutdown;
    JointCtrllerCfg ctrller;
    JointActuatorMapCfg mapper;
    SafetyCfg safety;
    DamiaoBusCfg damiao;
    DynamicsCfg dynamics;
};
```

### 4.6. 加载函数

```cpp
tl::expected<RobotCfg, ConfigErrInfo> load_robot_cfg(const std::string& path);
tl::expected<void, ConfigErrInfo> validate_robot_core_cfg(const RobotCfg& cfg);
tl::expected<void, ConfigErrInfo> validate_robot_cfg(const RobotCfg& cfg);
```

示例

```cpp
const auto cfg_result = dm_arm::load_robot_cfg("config/dm_arm.yaml");
if(!cfg_result) {
    std::cerr << cfg_result.error().message << '\n';
    return 1;
}

dm_arm::RobotCfg cfg = cfg_result.value();
```

## 5. JointCtrller

头文件

```cpp
#include <dm_arm/core/joints_ctrller.hpp>
```

### 5.1. 配置

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

### 5.2. 主要接口

```cpp
tl::expected<void, JointCtrllerErr> configure(const JointCtrllerCfg& cfg);
tl::expected<void, JointCtrllerErr> initialize(const JointState& state);
void reset() noexcept;
tl::expected<void, JointCtrllerErr> set_impedance_mode(JointImpedanceMode mode, const JointState& state);
tl::expected<void, JointCtrllerErr> set_cmd(const JointCmd& cmd);
tl::expected<void, JointCtrllerErr> set_full_cmd(const JointCtrlCmd& cmd);
tl::expected<JointCtrllerOutput, JointCtrllerErr> update(const JointCtrllerInput& input);
JointCtrllerState get_state() const noexcept;
JointImpedanceMode get_impedance_mode() const noexcept;
```

调用约束

- 必须先 `configure()`
- 激活时使用真实 JointState 调用 `initialize()`
- 跟踪命令只允许在跟踪模式使用
- `set_full_cmd()` 需要配置允许
- `update()` 的模型前馈长度必须匹配关节数量

## 6. JointActuatorMapper

头文件

```cpp
#include <dm_arm/core/joint_actuator_mapper.hpp>
```

### 6.1. 配置

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

### 6.2. 接口

```cpp
tl::expected<void, JointActuatorMapErr> configure(const JointActuatorMapCfg& cfg);
tl::expected<ActuatorMitCmd, JointActuatorMapErr> to_actuator_cmd(const JointCtrlCmd& joint_cmd) const;
tl::expected<JointState, JointActuatorMapErr> to_joint_state(const ActuatorState& actuator_state) const;
std::size_t size() const noexcept;
```

## 7. Safety

头文件

```cpp
#include <dm_arm/core/safety.hpp>
```

### 7.1. 配置

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
```

```cpp
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
};
```

### 7.2. 接口

```cpp
tl::expected<void, SafetyFault> configure(const SafetyCfg& cfg);
tl::expected<void, SafetyFault> check_state(const JointState& joint_state, const ActuatorState& actuator_state, double state_age_s) const;
tl::expected<void, SafetyFault> check_cmd_age(double cmd_age_s) const;
tl::expected<JointCtrlCmd, SafetyFault> check_joint_cmd(const JointState& state, const JointCtrlCmd& cmd, double dt);
tl::expected<void, SafetyFault> reset_cmd_history(const JointState& state);
void clear_cmd_history() noexcept;
SafetyAction action_for(SafetyErr err) const noexcept;
std::uint64_t clamp_count() const noexcept;
bool is_configured() const noexcept;
```

SafetyAction

```cpp
enum class SafetyAction {
    STOP_HOLD,
    DISABLE,
};
```

状态故障通常对应 `DISABLE`；命令故障通常对应 `STOP_HOLD`

## 8. MotorBus

头文件

```cpp
#include <dm_arm/hardware/motor_bus.hpp>
```

接口

```cpp
class MotorBus {
public:
    virtual ~MotorBus() = default;
    virtual tl::expected<void, MotorBusErr> connect() = 0;
    virtual tl::expected<ActuatorState, MotorBusErr> read() = 0;
    virtual tl::expected<void, MotorBusErr> activate() = 0;
    virtual tl::expected<void, MotorBusErr> write(const ActuatorMitCmd& cmd) = 0;
    virtual tl::expected<void, MotorBusErr> stop() = 0;
    virtual tl::expected<void, MotorBusErr> deactivate() = 0;
    virtual tl::expected<void, MotorBusErr> recover() = 0;
    virtual void cleanup() noexcept = 0;
    virtual std::size_t size() const noexcept = 0;
};
```

后端要求

- `read()` 返回固定长度 ActuatorState
- `write()` 接受执行器侧 MIT 命令
- `stop()` 优先执行可控保持
- `deactivate()` 立即停止并失能硬件；回停放姿态属于终端或上层会话职责
- `recover()` 清理故障残留并恢复到可再次激活的状态
- 周期路径不读取 YAML

## 9. DamiaoMotorBus

头文件

```cpp
#include <dm_arm/hardware/damiao_motor_bus.hpp>
```

### 9.1. 配置和生命周期

```cpp
tl::expected<void, MotorBusErr> configure(const DamiaoBusCfg& cfg);
tl::expected<void, MotorBusErr> connect() override;
tl::expected<ActuatorState, MotorBusErr> read() override;
tl::expected<void, MotorBusErr> activate() override;
tl::expected<void, MotorBusErr> write(const ActuatorMitCmd& cmd) override;
tl::expected<void, MotorBusErr> stop() override;
tl::expected<void, MotorBusErr> deactivate() override;
tl::expected<void, MotorBusErr> recover() override;
void cleanup() noexcept override;
std::size_t size() const noexcept override;
```

### 9.2. 静态信息

```cpp
struct DamiaoActuatorInfo {
    std::string name;
    std::string joint_name;
    std::uint32_t motor_id{ 0 };
    std::uint32_t master_id{ 0 };
    std::string motor_type;
    double q_max{ 0.0 };
    double dq_max{ 0.0 };
    double tau_max{ 0.0 };
};
```

```cpp
const std::vector<DamiaoActuatorInfo>& get_actuator_info() const noexcept;
```

## 10. Dynamics

头文件

```cpp
#include <dm_arm/dynamics/dynamics.hpp>
```

### 10.1. 错误码

```cpp
enum class DynamicsErr {
    NOT_CONFIGURED,
    ALREADY_CONFIGURED,
    NOT_UPDATED,
    INVALID_CFG,
    URDF_LOAD_FAILED,
    JOINT_NOT_FOUND,
    JOINT_NOT_1DOF,
    MODEL_SIZE_MISMATCH,
    FRAME_NOT_FOUND,
    INVALID_INPUT_SIZE,
    NON_FINITE_INPUT,
    COMPUTE_FAILED,
};
```

### 10.2. 模型信息

```cpp
struct DynamicsInfo {
    std::size_t joints_count{ 0 };
    int nq{ 0 };
    int nv{ 0 };
    double total_mass{ 0.0 };
    std::vector<std::string> joint_names;
    std::vector<int> q_indices;
    std::vector<int> v_indices;
};
```

### 10.3. 周期缓存

```cpp
struct DynamicsState {
    JointVector pos;
    JointVector vel;
    JointVector acc;
    JointVector tor;
    JointVector ref_acc;
    JointVector gravity;
    JointVector gravity_compensation;
    JointVector nonlinear;
    JointVector coriolis;
    JointVector inverse_dynamics;
    JointVector forward_dynamics;
    Eigen::MatrixXd mass_matrix;
    Eigen::Isometry3d tool_pose{ Eigen::Isometry3d::Identity() };
    Eigen::MatrixXd tool_jacobian;
};
```

### 10.4. 主要接口

```cpp
Dynamics();
~Dynamics();
Dynamics(Dynamics&& other) noexcept;
Dynamics& operator=(Dynamics&& other) noexcept;

tl::expected<void, DynamicsErr> configure(const DynamicsCfg& cfg);
tl::expected<void, DynamicsErr> update(const JointState& state, const JointVector& acc, const JointVector& ref_acc);
tl::expected<void, DynamicsErr> set_gravity_scale(const JointVector& gravity_scale);
void cleanup();

bool is_configured() const noexcept;
bool is_updated() const noexcept;
const DynamicsInfo& get_info() const noexcept;
const DynamicsState& get_state() const noexcept;
const JointVector& get_gravity_scale() const noexcept;

tl::expected<Eigen::Isometry3d, DynamicsErr> get_frame_pose(const std::string& frame_name) const;
tl::expected<Eigen::MatrixXd, DynamicsErr> get_frame_jacobian(const std::string& frame_name) const;

const JointVector& get_gravity() const noexcept;
const JointVector& get_gravity_compensation() const noexcept;
const JointVector& get_nonlinear() const noexcept;
const JointVector& get_coriolis() const noexcept;
const Eigen::MatrixXd& get_mass_matrix() const noexcept;
const JointVector& get_inverse_dynamics() const noexcept;
const JointVector& get_forward_dynamics() const noexcept;
const Eigen::Isometry3d& get_tool_pose() const noexcept;
const Eigen::MatrixXd& get_tool_jacobian() const noexcept;
```

### 10.5. 更新语义

`update()` 同一周期集中执行

- JointVector 到 Pinocchio 模型向量映射
- `computeAllTerms()`
- Frame placements
- 全部 Frame Jacobian 缓存
- `g(q)`
- `nle(q, dq)`
- `c(q, dq)`
- `M(q)`
- `RNEA(q, dq, ref_acc)`
- `ABA(q, dq, tau_feedback)`

### 10.6. 标准用法

```cpp
dm_arm::Dynamics dynamics;

const auto configure_result = dynamics.configure(cfg.dynamics);
if(!configure_result) {
    return 1;
}

const auto update_result = dynamics.update(joint_state, joint_acc, joint_ref_acc);
if(!update_result) {
    return 1;
}

const dm_arm::DynamicsState& model_state = dynamics.get_state();
```

### 10.7. Frame getter

```cpp
const auto pose = dynamics.get_frame_pose("tool0");
const auto jacobian = dynamics.get_frame_jacobian("tool0");
```

这两个 getter 只读取缓存；首次 `update()` 前返回 `DynamicsErr::NOT_UPDATED`

## 11. Robot

头文件

```cpp
#include <dm_arm/robot.hpp>
```

### 11.1. 生命周期

```cpp
enum class RobotState {
    UNCONFIGURED,
    INACTIVE,
    ACTIVE,
    FAULT,
};
```

```text
UNCONFIGURED
    ↓ configure
INACTIVE
    ↓ activate
ACTIVE
    ↓ deactivate
INACTIVE

ACTIVE
    ↓ fault
FAULT
    ↓ reset_fault
INACTIVE
```

### 11.2. 模型前馈函数

```cpp
using ModelFeedforwardFn = std::function<tl::expected<JointVector, ModelFeedforwardErr>(ModelFeedforwardMode, const JointState&, const JointVector&, const JointVector&, double)>;
```

参数顺序

```text
mode
JointState
joint_acc
joint_ref_acc
dt
```

### 11.3. 配置和生命周期接口

```cpp
tl::expected<void, RobotFault> configure(const RobotCfg& cfg, std::unique_ptr<MotorBus> motor_bus, ModelFeedforwardFn model_feedforward = {});
tl::expected<void, RobotFault> activate();
tl::expected<void, RobotFault> deactivate();
tl::expected<void, RobotFault> force_deactivate();
tl::expected<void, RobotFault> reset_fault();
tl::expected<void, RobotFault> maintain_fault_hold();
```

### 11.4. 命令接口

```cpp
tl::expected<void, RobotFault> set_cmd(const JointCmd& cmd, TimePoint now = Clock::now());
tl::expected<void, RobotFault> set_full_cmd(const JointCtrlCmd& cmd, TimePoint now = Clock::now());
tl::expected<void, RobotFault> set_impedance_mode(JointImpedanceMode mode, TimePoint now = Clock::now());
tl::expected<void, RobotFault> set_model_feedforward_mode(ModelFeedforwardMode mode);
```

`set_model_feedforward_mode()` 仅允许在 `INACTIVE` 使用

### 11.5. 周期接口

```cpp
tl::expected<RobotCycleOutput, RobotFault> cycle(TimePoint now = Clock::now());
bool is_fault_holding() const noexcept;
```

周期输出

```cpp
struct RobotCycleOutput {
    ActuatorState actuator_state;
    JointState joint_state;
    JointVector joint_acc;
    JointVector joint_ref_acc;
    JointVector model_feedforward;
    JointCtrlCmd joint_cmd;
    ActuatorMitCmd actuator_cmd;
    double dt{ 0.0 };
};
```

### 11.6. Getter

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
```

### 11.7. Dynamics 回调示例

```cpp
auto model_feedforward = [&dynamics](dm_arm::ModelFeedforwardMode mode, const dm_arm::JointState& state, const dm_arm::JointVector& acc, const dm_arm::JointVector& ref_acc, double) -> tl::expected<dm_arm::JointVector, dm_arm::ModelFeedforwardErr> {
    const auto result = dynamics.update(state, acc, ref_acc);
    if(!result) return tl::make_unexpected(dm_arm::ModelFeedforwardErr::COMPUTE_FAILED);

    if(mode == dm_arm::ModelFeedforwardMode::NONE) return dm_arm::JointVector(state.pos.size(), 0.0);
    if(mode == dm_arm::ModelFeedforwardMode::GRAVITY) return dynamics.get_gravity_compensation();
    if(mode == dm_arm::ModelFeedforwardMode::FULL_INVERSE_DYNAMICS) return dynamics.get_inverse_dynamics();
    return tl::make_unexpected(dm_arm::ModelFeedforwardErr::INVALID_MODE);
};
```

## 12. 完整应用骨架

```cpp
#include <dm_arm/config/config.hpp>
#include <dm_arm/dynamics/dynamics.hpp>
#include <dm_arm/hardware/damiao_motor_bus.hpp>
#include <dm_arm/robot.hpp>

int main() {
    const auto cfg_result = dm_arm::load_robot_cfg("config/dm_arm.yaml");
    if(!cfg_result) return 1;

    dm_arm::RobotCfg cfg = cfg_result.value();
    dm_arm::Dynamics dynamics;
    if(!dynamics.configure(cfg.dynamics)) return 1;

    auto bus = std::make_unique<dm_arm::DamiaoMotorBus>();
    if(!bus->configure(cfg.damiao)) return 1;

    auto model_feedforward = [&dynamics](dm_arm::ModelFeedforwardMode mode, const dm_arm::JointState& state, const dm_arm::JointVector& acc, const dm_arm::JointVector& ref_acc, double) -> tl::expected<dm_arm::JointVector, dm_arm::ModelFeedforwardErr> {
        if(!dynamics.update(state, acc, ref_acc)) return tl::make_unexpected(dm_arm::ModelFeedforwardErr::COMPUTE_FAILED);
        if(mode == dm_arm::ModelFeedforwardMode::NONE) return dm_arm::JointVector(state.pos.size(), 0.0);
        if(mode == dm_arm::ModelFeedforwardMode::GRAVITY) return dynamics.get_gravity_compensation();
        return dynamics.get_inverse_dynamics();
    };

    dm_arm::Robot robot;
    if(!robot.configure(cfg, std::move(bus), model_feedforward)) return 1;
    if(!robot.activate()) return 1;

    while(robot.get_state() == dm_arm::RobotState::ACTIVE) {
        const auto cycle_result = robot.cycle();
        if(!cycle_result) break;
    }

    robot.deactivate();
    return 0;
}
```

## 13. 调用顺序

### 13.1. 启动

```text
load_robot_cfg
→ Dynamics::configure
→ DamiaoMotorBus::configure
→ Robot::configure
→ Robot::activate
```

### 13.2. ACTIVE 周期

```text
set_cmd 可选
→ Robot::cycle
→ 读取 RobotCycleOutput
→ 读取 Dynamics 缓存
```

### 13.3. 正常停机

```text
Robot::deactivate
```

### 13.4. FAULT 恢复

```text
Robot::reset_fault
→ Robot::activate
```

## 14. 实时约束

周期路径应避免

- YAML 读取
- 重建 Pinocchio 模型
- 重建 Motor 对象
- 大量日志
- 长时间锁等待
- 动态切换后端

当前仍存在动态容器和 `tl::expected`；项目尚未宣称硬实时

## 15. Python API

Python 包名为 `dm_arm`；底层扩展模块名为 `_dm_arm`

### 15.1. 构建入口

根 CMake 开关

```cmake
-DM_ARM_BUILD_PYTHON=ON
```

该开关要求

```text
DM_ARM_ENABLE_DYNAMICS=ON
Python Interpreter
Python Development.Module
pybind11
Threads
```

`DM_ARM_BUILD_DAMIAO=ON` 时编译可用的真机 `RobotSession`；关闭 Damiao 时 Config、Core 和 Dynamics 仍可绑定，但真机会话会在 configure 阶段明确拒绝

wheel 构建入口

```bash
cd python
python -m build --wheel
```

Python 包说明见 [`../python/README.md`](../python/README.md)

### 15.2. 异常

所有 `tl::expected` 错误统一转换为

```python
dm_arm.DmArmError
```

配置、控制、映射、Safety、Dynamics 和 Robot 错误枚举仍然可以用于诊断

```python
try:
    cfg = dm_arm.load_robot_cfg("missing.yaml")
except dm_arm.DmArmError as error:
    print(error)
```

### 15.3. NumPy 规则

输入向量接受 C contiguous 或可转换为 C contiguous 的数组；内部统一转换为 `float64`

```text
JointVector shape = (N,)
六轴 q、dq、ddq、tau shape = (6,)
质量矩阵 shape = (6, 6)
Jacobian shape = (6, 6)
位姿 shape = (4, 4)
```

所有 C++ 向量、矩阵和位姿 getter 都返回 Python 独立副本；修改返回数组不会修改 C++ 缓存

### 15.4. 配置 API

公开类型

```text
RuntimeCfg
JointCtrllerCfg
JointActuatorMapCfg
JointLimitCfg
SafetyCfg
DamiaoActuatorCfg
DamiaoBusCfg
DynamicsCfg
RobotCfg
ConfigErr
ConfigErrInfo
```

公开函数

```python
cfg = dm_arm.load_robot_cfg("config/dm_arm.yaml")
dm_arm.validate_robot_core_cfg(cfg)
dm_arm.validate_robot_cfg(cfg)
```

### 15.5. 状态和命令类型

公开类型

```text
JointState
ActuatorState
JointPosCmd
JointPosVelCmd
JointPosVelTorCmd
JointCtrlCmd
ActuatorMitCmd
```

向量成员通过 NumPy 属性读写

```python
state = dm_arm.JointState()
state.pos = np.zeros(6)
state.vel = np.zeros(6)
state.tor = np.zeros(6)
```

### 15.6. JointCtrller

```python
ctrller = dm_arm.JointCtrller()
ctrller.configure(cfg.ctrller)
ctrller.initialize(state)
ctrller.set_impedance_mode(dm_arm.JointImpedanceMode.RIGID_TRACKING, state)
ctrller.set_pos_vel_cmd(pos, vel)
cmd = ctrller.update(state, model_feedforward, 0.005)
```

公开方法

```text
configure
initialize
reset
set_impedance_mode
set_pos_cmd
set_pos_vel_cmd
set_pos_vel_tor_cmd
set_full_cmd
update
```

### 15.7. JointActuatorMapper

```python
mapper = dm_arm.JointActuatorMapper()
mapper.configure(cfg.mapper)
actuator_cmd = mapper.to_actuator_cmd(joint_cmd)
joint_state = mapper.to_joint_state(actuator_state)
```

### 15.8. Safety

```python
safety = dm_arm.Safety()
safety.configure(cfg.safety)
safety.check_state(joint_state, actuator_state, state_age_s=0.005)
safe_cmd = safety.check_joint_cmd(joint_state, joint_cmd, dt=0.005)
```

公开诊断

```text
configured
clamp_count
action_for
```

### 15.9. Dynamics

```python
dynamics = dm_arm.Dynamics()
dynamics.configure(cfg.dynamics)
dynamics.update(q, dq, ddq, tau, ddq_ref)
```

更新后读取缓存

```python
state = dynamics.state
gravity = dynamics.gravity
gravity_compensation = dynamics.gravity_compensation
nonlinear = dynamics.nonlinear
coriolis = dynamics.coriolis
mass_matrix = dynamics.mass_matrix
inverse_dynamics = dynamics.inverse_dynamics
forward_dynamics = dynamics.forward_dynamics
tool_pose = dynamics.tool_pose
tool_jacobian = dynamics.tool_jacobian
```

指定 Frame

```python
pose = dynamics.frame_pose("tool0")
jacobian = dynamics.frame_jacobian("tool0")
```

Dynamics 计算期间释放 GIL；NumPy 转换和返回值复制在持有 GIL 时完成

### 15.10. RobotSession

Python 不直接绑定由调用方驱动的 `Robot::cycle()` 真机循环；公开的 `RobotSession` 在 C++ 内部创建工作线程并独占 Robot、DamiaoMotorBus 和 Dynamics

构造

```python
session = dm_arm.RobotSession("config/dm_arm.yaml", allow_hardware=True)
```

生命周期

```python
session.set_model_feedforward_mode(dm_arm.ModelFeedforwardMode.GRAVITY)
session.start()
session.stop()
session.reset_fault()
```

`set_model_feedforward_mode()` 只允许 INACTIVE；运行期间调用会抛出 `DmArmError`

模式和目标

```python
session.set_impedance_mode(dm_arm.JointImpedanceMode.RIGID_TRACKING)
session.set_gravity_scale(np.array([0.0, 0.1, 0.2, 0.0, 0.0, 0.0]))
session.move_to(np.array([0.0, 0.2, 0.2, 0.0, 0.0, 0.0]), speed_scale=0.2)
session.hold_current()
```

状态

```python
snapshot = session.snapshot
print(snapshot.robot_state)
print(snapshot.valid)
print(snapshot.last_error)
print(snapshot.cycle.joint_state.pos)
print(snapshot.cycle.actuator_state.pos)
print(snapshot.cycle.joint_acc)
print(snapshot.dynamics.gravity)
```

静态信息

```python
print(session.config)
print(session.dynamics_info)
print(session.actuator_info)
```

### 15.11. RobotSession 安全语义

真机启动必须同时满足

```text
Python 构造 allow_hardware=True
YAML runtime.write_enabled=true
CMake DM_ARM_BUILD_DAMIAO=ON
```

`start()` 负责激活和启动 C++ 周期线程；`stop()` 停止线程并在 ACTIVE 时调用 `Robot::deactivate()`；工作线程错误写入 `snapshot.last_error` 并停止周期；FAULT 必须显式调用 `reset_fault()`

`move_to()` 使用 C++ 梯形位置速度参考；目标必须位于 Safety 软限位内；调用前必须请求 `RIGID_TRACKING` 或 `COMPLIANT_TRACKING`

### 15.12. 上下文管理器

```python
with dm_arm.RobotSession("config/dm_arm.yaml", allow_hardware=True) as session:
    session.set_impedance_mode(dm_arm.JointImpedanceMode.COMPLIANT_DRAG)
    snapshot = session.snapshot
```

退出上下文时自动调用 `stop()`

## 16 未实现 API

当前尚未实现

- ROS 2 / ros2_control

开启 `DM_ARM_BUILD_ROS2=ON` 时会主动报错
