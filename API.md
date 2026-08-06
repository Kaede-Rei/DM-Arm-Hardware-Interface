# SerialArm-Core API Reference

本文档面向需要编写 SerialArm-Core 应用代码的开发者

每个可调用公共接口按以下顺序说明

```text
Doxygen 语义
参数
返回值
具体使用示例
使用注意
```

ros2_control Adapter 的公共生命周期接口也纳入本文档

[]

---

## 1. API 使用入口

SerialArm-Core 有三种主要调用层级

| 使用目标 | 推荐入口 |
| --- | --- |
| C++ 真机控制闭环 | `Robot` |
| Python 真机控制与实验 | `RobotSession` |
| 独立运动学和动力学计算 | `Dynamics` |
| 新 Hardware Backend | `MotorBus` + `HardwareLoader` |
| 调试控制器内部行为 | `JointCtrller` |
| 调试 Joint 与 Actuator 映射 | `JointActuatorMapper` |
| 独立验证安全策略 | `Safety` |

普通应用不需要手工把 `JointCtrller`、`Mapper` 和 `Safety` 串起来

`Robot` 已经负责这些模块的组合

### 公共头文件索引

| 头文件 | 主要公共接口 |
| --- | --- |
| `serial_arm/core/types.hpp` | Joint 与 Actuator 状态、命令、控制模式 |
| `serial_arm/config/config.hpp` | `RobotCfg`、配置加载与验证 |
| `serial_arm/config/robot_profile.hpp` | Robot Profile 解析 |
| `serial_arm/config/limit_resolver.hpp` | 最终 Safety limit 解析 |
| `serial_arm/model/model_loader.hpp` | URDF Joint limit 读取 |
| `serial_arm/core/joint_actuator_mapper.hpp` | Joint 与 Actuator 双向映射 |
| `serial_arm/core/joints_ctrller.hpp` | 五种阻抗模式控制器 |
| `serial_arm/core/safety.hpp` | 状态与命令安全检查 |
| `serial_arm/dynamics/dynamics.hpp` | Pinocchio Dynamics |
| `serial_arm/hardware/hardware_capability.hpp` | 执行器物理能力 |
| `serial_arm/hardware/motor_bus.hpp` | Hardware Backend contract |
| `serial_arm/hardware/hardware_loader.hpp` | Backend shared library loader |
| `serial_arm/robot.hpp` | 顶层 C++ 控制闭环 |

---

## 2. CMake Targets

安装 `serial_arm_core` 后可以使用

```cmake
find_package(serial_arm_core CONFIG REQUIRED)
```

公共 target

```text
serial_arm::core
serial_arm::config
serial_arm::robot
serial_arm::dynamics
```

最常见应用

```cmake
add_executable(my_robot_app main.cpp)

target_link_libraries(my_robot_app
  PRIVATE
    serial_arm::core
    serial_arm::config
    serial_arm::robot
    serial_arm::dynamics
)
```

如果只做离线 Dynamics

```cmake
target_link_libraries(my_dynamics_app
  PRIVATE
    serial_arm::dynamics
    serial_arm::config
)
```

---

# Part I 基础类型

## 3. `JointVector`

头文件

```cpp
#include "serial_arm/core/types.hpp"
```

定义

```cpp
using JointVector = std::vector<double>;
```

用途

所有 Joint 侧位置、速度、力矩、增益和加速度都使用 `JointVector`

约定

```text
位置   rad
速度   rad/s
力矩   N*m
kp     N*m/rad
kd     N*m*s/rad
```

所有 JointVector 的顺序必须与 `RobotCfg::joint_names` 一致

示例

```cpp
JointVector target{
    0.0,
    0.2,
    -0.3,
    0.0,
    0.1,
    0.0,
};
```

---

## 4. `ActuatorVector`

定义

```cpp
using ActuatorVector = std::vector<double>;
```

用途

用于 Backend 与 Core 之间的 Actuator 侧统一语义

即使底层厂商 SDK 使用编码器计数、电流或 RPM，进入 `ActuatorVector` 前也必须完成单位转换

### 具体使用示例

Backend 内部通常先使用厂商 SDK 原始值，再转换到 `ActuatorVector`

```cpp
serial_arm::ActuatorVector actuator_pos(6, 0.0);
serial_arm::ActuatorVector actuator_vel(6, 0.0);
serial_arm::ActuatorVector actuator_tor(6, 0.0);

for(std::size_t i = 0; i < 6; ++i) {
    const auto raw = driver.read(i);
    actuator_pos[i] = raw.position_rad;
    actuator_vel[i] = raw.velocity_rad_s;
    actuator_tor[i] = raw.torque_nm;
}
```

不要把 encoder count、RPM 或电流直接写入 `ActuatorVector`

---

## 5. `JointState`

定义

```cpp
struct JointState {
    JointVector pos;
    JointVector vel;
    JointVector tor;
};
```

字段

| 字段 | 单位 | 含义 |
| --- | --- | --- |
| `pos` | rad | Joint 实测位置 |
| `vel` | rad/s | Joint 实测速度 |
| `tor` | N*m | Joint 实测或估算力矩 |

示例

```cpp
JointState state;

state.pos = {0.0, 0.1, 0.2};
state.vel = {0.0, 0.0, 0.0};
state.tor = {0.0, 0.0, 0.0};
```

要求

- 三个向量长度一致
- 长度等于受控 Joint 数量
- 不允许 NaN 或 Inf

---

## 6. `ActuatorState`

定义

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

字段

| 字段 | 含义 |
| --- | --- |
| `pos` | 执行器侧位置，rad |
| `vel` | 执行器侧速度，rad/s |
| `tor` | 执行器侧力矩，N*m |
| `online` | 执行器是否在线 |
| `enabled` | 执行器是否使能 |
| `err_code` | Backend 提供的执行器错误码 |

`online`、`enabled` 和 `err_code` 会被 Safety 使用

Backend 不应把厂商原始数据直接填入 `pos`、`vel` 和 `tor`

### 具体使用示例

Backend `read()` 的典型返回结构

```cpp
serial_arm::ActuatorState state;
state.pos = {0.0, 0.1, -0.2};
state.vel = {0.0, 0.0, 0.0};
state.tor = {0.3, 0.5, 0.2};
state.online = {1, 1, 1};
state.enabled = {1, 1, 1};
state.err_code = {0, 0, 0};

return state;
```

`Safety::check_state()` 会进一步检查 `online`、`enabled` 和 `err_code`

---

## 7. `JointImpedanceGains`

定义

```cpp
struct JointImpedanceGains {
    JointVector kp;
    JointVector kd;
};
```

单位

```text
kp N*m/rad
kd N*m*s/rad
```

用途

为五种阻抗模式分别提供 Joint 侧刚度与阻尼

约束

- `kp` 与 `kd` 长度等于 Joint 数量
- 所有值必须有限
- 所有值必须非负

### 具体使用示例

为一个三关节测试控制器创建低刚度拖拽参数

```cpp
serial_arm::JointImpedanceGains drag_gains;
drag_gains.kp = {0.0, 0.0, 0.0};
drag_gains.kd = {0.10, 0.12, 0.08};

serial_arm::JointCtrllerCfg cfg;
cfg.joints_count = 3;
cfg.compliant_drag_gains = drag_gains;
```

实际 Robot 配置通常通过 YAML 加载，不需要在业务代码里逐项手工构造

---

## 8. `JointPosCmd`

定义

```cpp
struct JointPosCmd {
    JointVector pos;
};
```

用途

只提供位置参考

示例

```cpp
JointPosCmd cmd;
cmd.pos = target;

robot.set_cmd(cmd);
```

最终控制命令中的速度参考为 0

最终力矩会保留模型前馈项

---

## 9. `JointPosVelCmd`

定义

```cpp
struct JointPosVelCmd {
    JointVector pos;
    JointVector vel;
};
```

用途

上层轨迹生成器同时提供位置和速度参考

示例

```cpp
JointPosVelCmd cmd;

cmd.pos = target_pos;
cmd.vel = target_vel;

robot.set_cmd(cmd);
```

---

## 10. `JointPosVelTorCmd`

定义

```cpp
struct JointPosVelTorCmd {
    JointVector pos;
    JointVector vel;
    JointVector tor;
};
```

用途

上层除了位置和速度外还要附加关节力矩

示例

```cpp
JointPosVelTorCmd cmd;

cmd.pos = target_pos;
cmd.vel = target_vel;
cmd.tor = extra_torque;

robot.set_cmd(cmd);
```

如果 Robot 同时启用了 Dynamics 前馈

最终力矩语义为

```text
extra_torque + model_feedforward
```

因此不要重复加入同一份重力补偿

---

## 11. `JointCmd`

定义

```cpp
using JointCmd = std::variant<
    JointPosCmd,
    JointPosVelCmd,
    JointPosVelTorCmd
>;
```

通常不需要手工创建 `std::variant`

直接把三种具体命令传给 `Robot::set_cmd()` 即可

### 具体使用示例

`JointCmd` 是 variant，因此同一个变量可以承载三种 reference

```cpp
serial_arm::JointCmd cmd = serial_arm::JointPosCmd{
    {0.0, 0.2, -0.1}
};

robot.set_cmd(cmd);

cmd = serial_arm::JointPosVelCmd{
    {0.0, 0.25, -0.1},
    {0.0, 0.05, 0.0}
};

robot.set_cmd(cmd);
```

普通代码更推荐直接把具体命令类型传给 `set_cmd()`，不必显式声明 `JointCmd`

---

## 12. `JointCtrlCmd`

定义

```cpp
struct JointCtrlCmd {
    JointVector pos;
    JointVector vel;
    JointVector tor;
    JointVector kp;
    JointVector kd;
};
```

这是完整 Joint 侧 MIT 风格命令

普通上层控制优先使用 `JointPosCmd`、`JointPosVelCmd` 或 `JointPosVelTorCmd`

只有需要直接控制 `kp` 和 `kd` 时才使用 `JointCtrlCmd`

调用 `Robot::set_full_cmd()` 前必须满足

```yaml
control:
  controller:
    allow_full_cmd: true
```

并且 Robot 当前必须处于 tracking mode

示例

```cpp
JointCtrlCmd cmd;

cmd.pos = target_pos;
cmd.vel = target_vel;
cmd.tor = feedforward;
cmd.kp = kp;
cmd.kd = kd;

auto result = robot.set_full_cmd(cmd);
```

---

## 13. `ActuatorCtrlCmd`

定义

```cpp
struct ActuatorCtrlCmd {
    ActuatorVector pos;
    ActuatorVector vel;
    ActuatorVector tor;
    ActuatorVector kp;
    ActuatorVector kd;
};
```

这是 Mapper 之后、Backend 之前的完整执行器命令

应用层通常不会直接创建它

Backend 必须完整解释这五个字段

### 具体使用示例

`ActuatorCtrlCmd` 最常见的消费者是 Hardware Backend

```cpp
tl::expected<void, serial_arm::MotorBusErr>
MyBus::write(const serial_arm::ActuatorCtrlCmd& cmd) {
    for(std::size_t i = 0; i < cmd.pos.size(); ++i) {
        driver_.send_mit(
            i,
            cmd.pos[i],
            cmd.vel[i],
            cmd.kp[i],
            cmd.kd[i],
            cmd.tor[i]);
    }
    return {};
}
```

应用层通常只会在 `RobotCycleOutput::actuator_cmd` 中读取它用于诊断

---

# Part II 控制模式

## 14. `JointImpedanceMode`

定义

```cpp
enum class JointImpedanceMode {
    RIGID_HOLD,
    RIGID_TRACKING,
    COMPLIANT_HOLD,
    COMPLIANT_DRAG,
    COMPLIANT_TRACKING,
};
```

### `RIGID_HOLD`

进入模式时记录当前实测位置

之后保持该位置并使用 `rigid_hold_gains`

不接受 `set_cmd()`

典型调用

```cpp
robot.set_impedance_mode(
    JointImpedanceMode::RIGID_HOLD);
```

### `RIGID_TRACKING`

使用上层 `set_cmd()` 目标并使用 `rigid_tracking_gains`

典型调用

```cpp
robot.set_impedance_mode(
    JointImpedanceMode::RIGID_TRACKING);

robot.set_cmd(
    JointPosCmd{target});
```

### `COMPLIANT_HOLD`

进入模式时记录当前实测位置

使用低刚度 `compliant_hold_gains`

不接受 `set_cmd()`

### `COMPLIANT_DRAG`

每个控制周期都把当前实测位置作为位置参考

使用 `compliant_drag_gains`

不接受 `set_cmd()`

典型用途

- 手动拖拽
- 示教
- 低阻力关节移动

### `COMPLIANT_TRACKING`

接受 `set_cmd()`

使用 `compliant_tracking_gains`

适合带柔顺性的轨迹跟踪

### 具体使用示例

```cpp
robot.set_impedance_mode(serial_arm::JointImpedanceMode::RIGID_HOLD);
robot.set_impedance_mode(serial_arm::JointImpedanceMode::RIGID_TRACKING);
robot.set_impedance_mode(serial_arm::JointImpedanceMode::COMPLIANT_HOLD);
robot.set_impedance_mode(serial_arm::JointImpedanceMode::COMPLIANT_DRAG);
robot.set_impedance_mode(serial_arm::JointImpedanceMode::COMPLIANT_TRACKING);
```

只有 `RIGID_TRACKING` 和 `COMPLIANT_TRACKING` 接受外部 `set_cmd()` reference

---

## 15. `ModelFeedforwardMode`

定义

```cpp
enum class ModelFeedforwardMode {
    NONE,
    GRAVITY,
    FULL_INVERSE_DYNAMICS,
};
```

### `NONE`

不加入模型前馈

### `GRAVITY`

使用 Dynamics 的 `gravity_compensation`

### `FULL_INVERSE_DYNAMICS`

使用 Dynamics 的 `inverse_dynamics`

完整逆动力学依赖

- 真实模型质量与惯量
- 当前 `q`
- 当前 `dq`
- 参考 `ddq`

模型参数没有验证时不要直接把 FULL_INVERSE_DYNAMICS 用于真机

### 具体使用示例

Robot configure 完成后、activate 前选择重力补偿

```cpp
robot.set_model_feedforward_mode(
    serial_arm::ModelFeedforwardMode::GRAVITY);

robot.activate();
```

离线 Dynamics 验证阶段可以先使用 `NONE`，确认真实惯性参数后再切换到 `GRAVITY` 或 `FULL_INVERSE_DYNAMICS`

---

# Part III Robot Profile 与配置

## 16. `RobotProfileCore`

头文件

```cpp
#include "serial_arm/config/robot_profile.hpp"
```

定义

```cpp
struct RobotProfileCore {
    std::string name;
    std::string profile_file;
    std::string core_config_path;
    std::string hardware_plugin;
    std::string hardware_config_path;
};
```

用途

把 profile name 解析成 C++ Core 真正需要的资源

典型结果

```text
dm_arm_gray
    ->
core_config_path
hardware_plugin
hardware_config_path
```

### 具体使用示例

```cpp
auto profile_result = serial_arm::load_robot_profile_core("dm_arm_gray");
if(!profile_result) {
    return 1;
}

const serial_arm::RobotProfileCore profile = profile_result.value();

std::cout << profile.name << "\n";
std::cout << profile.core_config_path << "\n";
std::cout << profile.hardware_plugin << "\n";
std::cout << profile.hardware_config_path << "\n";
```

`RobotProfileCore` 只包含 framework-neutral Core/Hardware 部分，ROS 2 Description、Controllers 和 MoveIt 信息由 Adapter 层继续读取

---

## 17. `RobotProfileLoadOptions`

定义

```cpp
struct RobotProfileLoadOptions {
    std::string profile_file;
    std::vector<std::string> resource_paths;
};
```

### `profile_file`

显式指定 `robot_profiles.yaml`

非空时优先级最高

### `resource_paths`

额外资源根目录

适合 standalone 安装或测试目录

### 具体使用示例

```cpp
serial_arm::RobotProfileLoadOptions options;
options.profile_file = "/tmp/test_robot_profiles.yaml";
options.resource_paths = {
    "/tmp/test_install",
    "/opt/serial_arm",
};

auto profile = serial_arm::load_robot_profile_core(
    "test_arm",
    options);
```

测试代码常使用显式 `profile_file`，正常安装环境更常只依赖 resource path

---

## 18. `load_robot_profile_core()`

签名

```cpp
tl::expected<
    RobotProfileCore,
    RobotProfileErrInfo
>
load_robot_profile_core(
    const std::string& profile_name,
    const RobotProfileLoadOptions& options = {});
```

最常见用法

```cpp
auto result =
    load_robot_profile_core("dm_arm_gray");

if(!result) {
    std::cerr
        << result.error().message
        << "\n";
    return 1;
}

RobotProfileCore profile =
    result.value();
```

显式 Profile 文件

```cpp
RobotProfileLoadOptions options;

options.profile_file =
    "/path/to/robot_profiles.yaml";

auto result =
    load_robot_profile_core(
        "my_arm",
        options);
```

常见错误

- `PROFILE_FILE_NOT_FOUND`
- `PROFILE_LOAD_FAILED`
- `PROFILE_NOT_FOUND`
- `MISSING_FIELD`
- `RESOURCE_NOT_FOUND`

### Doxygen 语义展开

将一个 Robot Profile 名称解析为 Core 与 Hardware 真正需要的绝对资源路径

参数

- `profile_name` 为 `robot_profiles.yaml` 中的 profile 名称
- `options.profile_file` 非空时只使用该文件
- `options.resource_paths` 用于追加 standalone resource root

返回值

成功返回 `RobotProfileCore`，失败返回 `RobotProfileErrInfo`，错误信息应优先打印 `message`

具体使用示例

```cpp
#include "serial_arm/config/robot_profile.hpp"
#include <iostream>

serial_arm::RobotProfileLoadOptions options;
options.resource_paths = {
    "/opt/serial_arm",
    "/home/user/serialarm_install",
};

auto profile_result = serial_arm::load_robot_profile_core(
    "dm_arm_gray",
    options);

if(!profile_result) {
    std::cerr << profile_result.error().message << "\n";
    return 1;
}

const auto& profile = profile_result.value();
std::cout << "core config: " << profile.core_config_path << "\n";
std::cout << "hardware plugin: " << profile.hardware_plugin << "\n";
std::cout << "hardware config: " << profile.hardware_config_path << "\n";
```

使用注意

- 不要把解析结果继续当相对路径拼接
- 显式 `profile_file` 适合测试和固定部署环境，普通安装优先依赖 resource resolver

---

## 19. `robot_profile_search_paths()`

签名

```cpp
std::vector<std::string>
robot_profile_search_paths(
    const RobotProfileLoadOptions& options = {});
```

用途

诊断 Profile resolver 到底搜索了哪些目录

示例

```cpp
for(const auto& path :
    robot_profile_search_paths())
{
    std::cout << path << "\n";
}
```

Profile 找不到时优先调用这个函数，而不是在代码里硬编码更多路径

### Doxygen 语义展开

返回 Robot Profile resolver 当前会搜索的资源根目录，用于定位 profile 找不到的问题

参数

- `options` 与 `load_robot_profile_core()` 使用同一套搜索规则

返回值

返回按搜索优先级排列的路径数组

具体使用示例

```cpp
serial_arm::RobotProfileLoadOptions options;
options.resource_paths.push_back("/tmp/serial_arm_test_install");

for(const auto& path : serial_arm::robot_profile_search_paths(options)) {
    std::cout << path << "\n";
}
```

使用注意

- 这个接口用于诊断，不建议把返回路径再写回配置

---

## 20. `RuntimeCfg`

定义

```cpp
struct RuntimeCfg {
    double ctrl_frequency_hz;
    double joint_acc_filter_alpha;
    bool write_enabled;
    ModelFeedforwardMode model_feedforward_mode;
    JointImpedanceMode tracking_impedance_mode;
};
```

### `ctrl_frequency_hz`

Robot 目标控制频率

`cycle()` 第一个周期使用

```text
1 / ctrl_frequency_hz
```

作为 nominal dt

### `joint_acc_filter_alpha`

Robot 内部关节加速度估计低通系数

### `write_enabled`

是否允许 `Robot::activate()`

`false` 时 `activate()` 返回 `RobotErr::WRITE_DISABLED`

### `model_feedforward_mode`

模型前馈策略

### `tracking_impedance_mode`

配置层默认 tracking mode

具体 Adapter 或 Session 可以读取它作为默认策略

### 具体使用示例

```cpp
serial_arm::RuntimeCfg runtime;
runtime.ctrl_frequency_hz = 200.0;
runtime.joint_acc_filter_alpha = 0.2;
runtime.write_enabled = false;
runtime.model_feedforward_mode = serial_arm::ModelFeedforwardMode::GRAVITY;
runtime.tracking_impedance_mode = serial_arm::JointImpedanceMode::COMPLIANT_TRACKING;
```

真机首次检查阶段应保持 `write_enabled=false`，完成配置与模型验证后再显式开启

---

## 21. `ShutdownCfg`

定义

```cpp
struct ShutdownCfg {
    bool park_before_disable;
    JointVector park_pos;
    double speed_scale;
    double position_tolerance;
    double velocity_tolerance;
    double settle_time_s;
    double relaxed_tolerance_ratio;
    double timeout_s;
};
```

用途

保存上层 Session、Terminal 或 Adapter 可以采用的停放姿态参数

当前 `Robot::deactivate()` 本身直接调用 Backend deactivate，不会在 `Robot` 内部自动执行 park trajectory

其中 `velocity_tolerance` 会被 `Robot::clear_fault()` 用作低速度恢复判据

### 具体使用示例

```cpp
serial_arm::ShutdownCfg shutdown;
shutdown.park_before_disable = true;
shutdown.park_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
shutdown.speed_scale = 0.10;
shutdown.position_tolerance = 0.05;
shutdown.velocity_tolerance = 0.05;
shutdown.settle_time_s = 0.25;
shutdown.relaxed_tolerance_ratio = 2.0;
shutdown.timeout_s = 15.0;
```

`Robot::deactivate()` 当前不会自己执行 park trajectory，这组参数主要由 Session、Terminal 或 Adapter 使用

---

## 22. `DynamicsCfg`

定义

```cpp
struct DynamicsCfg {
    std::string urdf_path;
    std::vector<std::string> joint_names;
    std::string base_frame;
    std::string tool_frame;
    std::array<double, 3> gravity;
    JointVector gravity_scale;
};
```

典型配置

```yaml
model:
  urdf_path: ../../model/gray/urdf/dm_arm_no_gripper.urdf
  joint_names:
    - joint1
    - joint2
    - joint3
    - joint4
    - joint5
    - joint6
  base_frame: base_link
  tool_frame: tool0
  gravity: [0.0, 0.0, -9.81]
  gravity_scale: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
```

### 具体使用示例

```cpp
serial_arm::DynamicsCfg dynamics_cfg;
dynamics_cfg.urdf_path = "/path/to/robot.urdf";
dynamics_cfg.joint_names = {
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
};
dynamics_cfg.base_frame = "base_link";
dynamics_cfg.tool_frame = "tool0";
dynamics_cfg.gravity = {0.0, 0.0, -9.81};
dynamics_cfg.gravity_scale = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

serial_arm::Dynamics dynamics;
dynamics.configure(dynamics_cfg);
```

---

## 23. `RobotCfg`

定义

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

这是 Core 完整静态配置

正常情况下不要在应用中手工拼装

优先通过 `load_robot_cfg()` 从 YAML 生成

### 具体使用示例

正常使用时从 YAML 获得完整 `RobotCfg`

```cpp
auto cfg_result = serial_arm::load_robot_cfg(
    profile.core_config_path,
    bus->capabilities());

if(!cfg_result) {
    return 1;
}

serial_arm::RobotCfg cfg = cfg_result.value();
std::cout << cfg.joint_names.size() << "\n";
std::cout << cfg.runtime.ctrl_frequency_hz << "\n";
```

只有测试 fixture 或配置生成工具才建议手工拼完整 `RobotCfg`

---

## 24. `load_robot_cfg()`

C++ 签名

```cpp
tl::expected<
    RobotCfg,
    ConfigErrInfo
>
load_robot_cfg(
    const std::string& path,
    const HardwareCapabilities& capabilities);
```

为什么必须传 `HardwareCapabilities`

最终 Safety limit 不是只读取 YAML

它还需要结合

```text
URDF
HardwareCapabilities
Calibration
Safety policy
```

完整示例

```cpp
HardwareLoader loader;

auto bus_result = loader.load(
    profile.hardware_plugin,
    profile.hardware_config_path);

if(!bus_result) {
    return 1;
}

auto bus =
    std::move(bus_result.value());

auto cfg_result = load_robot_cfg(
    profile.core_config_path,
    bus->capabilities());

if(!cfg_result) {
    std::cerr
        << cfg_result.error().message
        << "\n";
    return 1;
}

RobotCfg cfg =
    cfg_result.value();
```

### Doxygen 语义展开

使用 yaml-cpp 加载完整机器人配置，并结合执行器能力生成最终 Safety 配置

参数

- `path` 为 Core YAML 路径
- `capabilities` 为 Backend 按执行器顺序提供的物理能力

返回值

成功返回 `RobotCfg`，失败返回带明确 `message` 的 `ConfigErrInfo`

具体使用示例

```cpp
serial_arm::HardwareLoader hardware_loader;
auto bus_result = hardware_loader.load(
    profile.hardware_plugin,
    profile.hardware_config_path);

if(!bus_result) {
    std::cerr << "hardware loader failed\n";
    return 1;
}

auto bus = std::move(bus_result.value());

auto cfg_result = serial_arm::load_robot_cfg(
    profile.core_config_path,
    bus->capabilities());

if(!cfg_result) {
    std::cerr << cfg_result.error().message << "\n";
    return 1;
}

serial_arm::RobotCfg cfg = cfg_result.value();
std::cout << "joint count = " << cfg.joint_names.size() << "\n";
```

使用注意

- 不要自己用硬件 YAML 猜 `HardwareCapabilities`
- 配置加载成功只代表静态配置合法，不代表真机已经连接

---

## 25. `validate_robot_core_cfg()`

签名

```cpp
tl::expected<void, ConfigErrInfo>
validate_robot_core_cfg(
    const RobotCfg& cfg);
```

用途

验证 Robot 闭环真正需要的配置一致性

适合配置经过程序修改后再次检查

示例

```cpp
cfg.runtime.ctrl_frequency_hz = 250.0;

auto result =
    validate_robot_core_cfg(cfg);

if(!result) {
    std::cerr
        << result.error().message
        << "\n";
}
```

### Doxygen 语义展开

验证 `Robot` 控制闭环真正依赖的通用配置一致性

参数

- `cfg` 为已经解析或被程序修改后的 `RobotCfg`

返回值

成功返回空 `expected`，失败返回 `ConfigErrInfo`

具体使用示例

```cpp
serial_arm::RobotCfg cfg = cfg_result.value();
cfg.runtime.ctrl_frequency_hz = 250.0;

const auto valid = serial_arm::validate_robot_core_cfg(cfg);
if(!valid) {
    std::cerr << valid.error().message << "\n";
    return 1;
}
```

使用注意

- 程序运行时改配置后应重新验证
- 不要把这个函数当作硬件连通性检查

---

## 26. `validate_robot_cfg()`

签名

```cpp
tl::expected<void, ConfigErrInfo>
validate_robot_cfg(
    const RobotCfg& cfg);
```

用途

验证完整配置

如果只想验证 Robot 闭环最小契约，可使用 `validate_robot_core_cfg()`

### Doxygen 语义展开

验证完整 `RobotCfg`，用于配置工具、测试和 release 前检查

参数

- `cfg` 为待验证的完整配置

返回值

成功返回空 `expected`，失败返回 `ConfigErrInfo`

具体使用示例

```cpp
const auto valid = serial_arm::validate_robot_cfg(cfg);
if(!valid) {
    std::cerr << "invalid config: " << valid.error().message << "\n";
    return 1;
}
```

使用注意

- 业务控制代码通常在 `load_robot_cfg()` 后直接使用结果即可
- 配置生成工具更适合显式调用完整验证

---

## 27. `compare_robot_cfg()`

签名

```cpp
tl::expected<
    std::vector<std::string>,
    ConfigErrInfo
>
compare_robot_cfg(
    const std::string& lhs_path,
    const std::string& rhs_path);
```

带 HardwareCapabilities 的重载

```cpp
compare_robot_cfg(
    lhs_path,
    rhs_path,
    capabilities);
```

用途

比较两个 YAML 解析后的最终配置差异

适合

- gray 与 white variant 对比
- 调参前后对比
- release 前检查配置漂移

### Doxygen 语义展开

比较两个 YAML 解析后的最终配置，而不是比较原始文本差异

参数

- `lhs_path` 与 `rhs_path` 为两个 Core YAML
- `capabilities` 重载用于需要真实硬件约束参与解析的比较

返回值

成功返回差异字符串数组，数组为空表示解析后的配置一致

具体使用示例

```cpp
auto diff_result = serial_arm::compare_robot_cfg(
    "config/core/gray.yaml",
    "config/core/white.yaml",
    bus->capabilities());

if(!diff_result) {
    std::cerr << diff_result.error().message << "\n";
    return 1;
}

for(const auto& diff : diff_result.value()) {
    std::cout << diff << "\n";
}
```

使用注意

- 比较结果适合调参审计，不要用它判断机器人当前运行状态

---

# Part IV Model 与 Safety limit 解析

## 28. `ModelLoader`

头文件

```cpp
#include "serial_arm/model/model_loader.hpp"
```

签名

```cpp
tl::expected<
    RobotModelInfo,
    ModelErr
>
ModelLoader::load(
    const std::string& urdf_path,
    const std::vector<std::string>& controlled_joint_names) const;
```

用途

从 URDF 提取受控 Joint 和 limit 信息

示例

```cpp
ModelLoader loader;

auto model_result = loader.load(
    "/path/to/robot.urdf",
    {
        "joint1",
        "joint2",
        "joint3",
    });

if(!model_result) {
    return 1;
}

const RobotModelInfo& model =
    model_result.value();
```

常见错误

- `FILE_OPEN_FAILED`
- `URDF_LOAD_FAILED`
- `MISSING_JOINT`
- `DUPLICATE_JOINT`
- `FIXED_JOINT_CONTROLLED`
- `INVALID_LIMIT`

### Doxygen 语义展开

从 URDF 中读取指定受控 Joint 的类型和 limit 信息

参数

- `urdf_path` 为 URDF 文件
- `controlled_joint_names` 决定返回顺序并限制只解析这些 Joint

返回值

成功返回 `RobotModelInfo`，失败返回 `ModelErr`

具体使用示例

```cpp
serial_arm::ModelLoader loader;

auto model_result = loader.load(
    cfg.dynamics.urdf_path,
    cfg.joint_names);

if(!model_result) {
    std::cerr << "ModelLoader error="
              << static_cast<int>(model_result.error()) << "\n";
    return 1;
}

for(const auto& limit : model_result->joint_limits) {
    std::cout << limit.name
              << " max_vel=" << limit.max_vel
              << " max_effort=" << limit.max_effort
              << "\n";
}
```

使用注意

- continuous Joint 没有绝对位置上下限是合法情况
- 受控 Joint 名称不存在时应修 URDF 或配置，不要在运行时跳过

---

## 29. `RobotModelInfo`

定义

```cpp
struct RobotModelInfo {
    std::string urdf_path;
    std::vector<std::string> joint_names;
    std::vector<ModelJointLimit> joint_limits;
};
```

`joint_names` 与 `joint_limits` 保持请求顺序

### 具体使用示例

```cpp
const serial_arm::RobotModelInfo model = model_result.value();

std::cout << "URDF: " << model.urdf_path << "\n";
for(std::size_t i = 0; i < model.joint_names.size(); ++i) {
    std::cout << model.joint_names[i]
              << " max_vel=" << model.joint_limits[i].max_vel
              << "\n";
}
```

---

## 30. `ModelJointLimit`

定义

```cpp
struct ModelJointLimit {
    std::string name;
    bool has_position_limit;
    double min_pos;
    double max_pos;
    double max_vel;
    double max_effort;
};
```

continuous joint 可以没有绝对 position limit

不要人为给 continuous joint 添加 `[-pi, pi]`

### 具体使用示例

```cpp
for(const serial_arm::ModelJointLimit& limit : model.joint_limits) {
    if(limit.has_position_limit) {
        std::cout << limit.name
                  << " range=[" << limit.min_pos
                  << ", " << limit.max_pos << "]\n";
    }
    else {
        std::cout << limit.name << " has no absolute position limit\n";
    }
}
```

continuous Joint 常见 `has_position_limit == false`

---

## 31. `ActuatorCapability`

头文件

```cpp
#include "serial_arm/hardware/hardware_capability.hpp"
```

定义

```cpp
struct ActuatorCapability {
    std::string actuator_name;
    double min_pos;
    double max_pos;
    double max_vel;
    double max_effort;
    double max_kp;
    double max_kd;
};
```

Backend 必须根据实际执行器型号提供这些物理能力

### 具体使用示例

Backend 可以按执行器型号生成 capability

```cpp
serial_arm::ActuatorCapability cap;
cap.actuator_name = "joint1_motor";
cap.min_pos = -3.14;
cap.max_pos = 3.14;
cap.max_vel = 20.0;
cap.max_effort = 28.0;
cap.max_kp = 500.0;
cap.max_kd = 5.0;
```

这些值应来源于执行器真实物理能力或 Backend 明确的保守约束

---

## 32. `HardwareCapabilities`

定义

```cpp
using HardwareCapabilities =
    std::vector<ActuatorCapability>;
```

顺序必须与 Backend 管理的 Actuator 顺序一致

### 具体使用示例

```cpp
serial_arm::HardwareCapabilities capabilities;
capabilities.push_back(joint1_capability);
capabilities.push_back(joint2_capability);
capabilities.push_back(joint3_capability);

assert(capabilities.size() == 3);
```

数组顺序必须与 Backend 的 Actuator 顺序保持一致

---

## 33. `SafetyPolicyCfg`

定义

```cpp
struct SafetyPolicyCfg {
    double position_margin;
    double cmd_vel_scale;
    double state_vel_scale;
    JointVector max_acc;
    JointVector max_effort_override;
    JointVector max_kp_override;
    JointVector max_kd_override;
    double max_dt_s;
    double state_timeout_s;
    double cmd_timeout_s;
    bool require_all_actuators_online;
    bool require_all_actuators_enabled;
    bool reject_motor_error;
    bool require_continuous_cmd;
};
```

它表达的是用户希望施加在物理能力之上的运行策略

`override` 和 scale 只能收窄最终限制

### 具体使用示例

```cpp
serial_arm::SafetyPolicyCfg policy;
policy.position_margin = 0.05;
policy.cmd_vel_scale = 0.5;
policy.state_vel_scale = 0.8;
policy.max_acc.assign(6, 5.0);
policy.max_effort_override.assign(6, 10.0);
policy.max_kp_override.assign(6, 50.0);
policy.max_kd_override.assign(6, 1.0);
policy.max_dt_s = 0.02;
policy.state_timeout_s = 0.05;
policy.cmd_timeout_s = 0.10;
policy.require_all_actuators_online = true;
policy.require_all_actuators_enabled = true;
policy.reject_motor_error = true;
policy.require_continuous_cmd = true;
```

这些限制会与 URDF 和 HardwareCapabilities 取交集，不能用于放宽物理限制

---

## 34. `ResolvedJointLimitCfg`

定义

```cpp
struct ResolvedJointLimitCfg {
    std::string joint_name;
    bool has_position_limit;
    double hard_min_pos;
    double hard_max_pos;
    double cmd_min_pos;
    double cmd_max_pos;
    double max_cmd_vel;
    double max_state_vel;
    double max_acc;
    double max_effort;
    double max_kp;
    double max_kd;
};
```

它适合用于打印最终生效限制，而不是作为用户直接输入配置

### 具体使用示例

```cpp
for(const auto& limit : resolved.joints) {
    std::cout << limit.joint_name
              << " hard=[" << limit.hard_min_pos << ", " << limit.hard_max_pos << "]"
              << " cmd=[" << limit.cmd_min_pos << ", " << limit.cmd_max_pos << "]"
              << " max_vel=" << limit.max_cmd_vel
              << " max_effort=" << limit.max_effort
              << "\n";
}
```

它更适合配置诊断输出，不应作为用户手工输入结构

---

## 35. `ResolvedSafetyCfg`

定义

```cpp
struct ResolvedSafetyCfg {
    std::vector<ResolvedJointLimitCfg> joints;
    double max_dt_s;
    double state_timeout_s;
    double cmd_timeout_s;
    bool require_all_actuators_online;
    bool require_all_actuators_enabled;
    bool reject_motor_error;
    bool require_continuous_cmd;
};
```

它是 `LimitResolver` 的最终输出

### 具体使用示例

```cpp
serial_arm::ResolvedSafetyCfg resolved = resolved_result.value();

std::cout << "max dt=" << resolved.max_dt_s << "\n";
std::cout << "state timeout=" << resolved.state_timeout_s << "\n";
std::cout << "command timeout=" << resolved.cmd_timeout_s << "\n";

serial_arm::SafetyCfg safety_cfg = serial_arm::to_safety_cfg(resolved);
```

---

## 36. `LimitResolver`

头文件

```cpp
#include "serial_arm/config/limit_resolver.hpp"
```

签名

```cpp
tl::expected<
    ResolvedSafetyCfg,
    LimitResolverErr
>
resolve(
    const RobotModelInfo& model,
    const JointActuatorMapCfg& mapper,
    const HardwareCapabilities& capabilities,
    const SafetyPolicyCfg& policy) const;
```

作用

把多个限制来源收敛成最终 Joint 侧 Safety limit

逻辑可以理解为

```text
Resolved limit
    =
URDF
    ∩
mapped hardware capability
    ∩
Safety policy
```

Safety policy 只能收窄，不能放宽底层能力

### Doxygen 语义展开

把 URDF limit、Joint/Actuator 映射、硬件能力和 Safety policy 收敛成最终 Joint 侧限制

参数

- `model` 来自 `ModelLoader`
- `mapper` 为关节执行器映射
- `capabilities` 来自 Backend
- `policy` 为用户希望施加的额外收窄策略

返回值

成功返回 `ResolvedSafetyCfg`，失败返回 `LimitResolverErr`

具体使用示例

```cpp
serial_arm::LimitResolver resolver;

auto resolved_result = resolver.resolve(
    model_result.value(),
    cfg.mapper,
    bus->capabilities(),
    policy);

if(!resolved_result) {
    std::cerr << "LimitResolverErr="
              << static_cast<int>(resolved_result.error()) << "\n";
    return 1;
}

for(const auto& joint : resolved_result->joints) {
    std::cout << joint.joint_name
              << " cmd_vel=" << joint.max_cmd_vel
              << " effort=" << joint.max_effort
              << "\n";
}
```

使用注意

- Safety policy 只能收窄能力，试图放宽硬件或 URDF 限制会失败

---

## 37. `to_safety_cfg()`

签名

```cpp
SafetyCfg
to_safety_cfg(
    const ResolvedSafetyCfg& resolved);
```

用途

把 resolver 输出转换为 `Safety` 可以直接使用的配置

### Doxygen 语义展开

把便于诊断展示的 `ResolvedSafetyCfg` 转成 `Safety` 真正消费的 `SafetyCfg`

参数

- `resolved` 为 `LimitResolver::resolve()` 的结果

返回值

返回可以直接传给 `Safety::configure()` 的配置

具体使用示例

```cpp
const serial_arm::ResolvedSafetyCfg resolved = resolved_result.value();
serial_arm::SafetyCfg safety_cfg = serial_arm::to_safety_cfg(resolved);

serial_arm::Safety safety;
auto configured = safety.configure(safety_cfg);
if(!configured) {
    std::cerr << "Safety configure failed\n";
}
```

---

## 38. `resolve_from_safety_cfg()`

签名

```cpp
ResolvedSafetyCfg
resolve_from_safety_cfg(
    const std::vector<std::string>& joint_names,
    const SafetyCfg& cfg);
```

用途

从已经生成的 SafetyCfg 重建便于展示或诊断的 resolved 结构

### Doxygen 语义展开

从已经生成的 `SafetyCfg` 重建带 Joint 名称的 resolved 结构，主要用于日志和诊断工具

参数

- `joint_names` 顺序必须与 `cfg.limits` 一致
- `cfg` 为最终 Safety 配置

返回值

返回 `ResolvedSafetyCfg`

具体使用示例

```cpp
const auto resolved = serial_arm::resolve_from_safety_cfg(
    cfg.joint_names,
    cfg.safety);

for(const auto& joint : resolved.joints) {
    std::cout << joint.joint_name
              << " [" << joint.cmd_min_pos
              << ", " << joint.cmd_max_pos << "]\n";
}
```

---

# Part V JointActuatorMapper

## 39. `JointActuatorMapCfg`

头文件

```cpp
#include "serial_arm/core/joint_actuator_mapper.hpp"
```

定义

```cpp
struct JointActuatorMapCfg {
    std::size_t joints_count;

    ActuatorVector pos_ratio;
    ActuatorVector tor_ratio;
    std::vector<int> direction;

    JointVector joint_zero_offset;
    ActuatorVector actuator_zero_offset;
};
```

约束

- `joints_count > 0`
- 所有数组长度等于 `joints_count`
- `pos_ratio > 0`
- `tor_ratio > 0`
- `direction` 只能为 `1` 或 `-1`
- 所有 double 必须为有限值

### 具体使用示例

```cpp
serial_arm::JointActuatorMapCfg map;
map.joints_count = 3;
map.pos_ratio = {1.0, 6.0, 6.0};
map.tor_ratio = {1.0, 6.0, 6.0};
map.direction = {1, -1, 1};
map.joint_zero_offset = {0.0, 0.0, 0.0};
map.actuator_zero_offset = {0.0, 0.0, 0.0};

serial_arm::JointActuatorMapper mapper;
mapper.configure(map);
```

如果厂商 SDK 已经报告减速器输出端状态，`pos_ratio` 和 `tor_ratio` 通常设为 1

---

## 40. `JointActuatorMapper::configure()`

签名

```cpp
tl::expected<void, JointActuatorMapErr>
configure(
    const JointActuatorMapCfg& cfg);
```

必须在任何转换前调用

示例

```cpp
JointActuatorMapper mapper;

auto result =
    mapper.configure(cfg.mapper);

if(!result) {
    return 1;
}
```

### Doxygen 语义展开

保存 Joint 与 Actuator 的方向、比例和零位关系，使后续双向转换具有固定语义

参数

- `cfg` 为 `JointActuatorMapCfg`，所有数组长度必须等于 `joints_count`

返回值

成功返回空 `expected`，配置非法返回 `JointActuatorMapErr`

具体使用示例

```cpp
serial_arm::JointActuatorMapper mapper;

auto result = mapper.configure(cfg.mapper);
if(!result) {
    std::cerr << "mapper configure error="
              << static_cast<int>(result.error()) << "\n";
    return 1;
}
```

---

## 41. `JointActuatorMapper::to_joint_state()`

签名

```cpp
tl::expected<
    JointState,
    JointActuatorMapErr
>
to_joint_state(
    const ActuatorState& actuator_state) const;
```

逐轴映射关系

```text
joint_pos
=
joint_zero_offset
+
direction
*
(actuator_pos - actuator_zero_offset)
/
pos_ratio
```

```text
joint_vel
=
direction
*
actuator_vel
/
pos_ratio
```

```text
joint_tor
=
direction
*
tor_ratio
*
actuator_tor
```

示例

```cpp
auto state_result =
    mapper.to_joint_state(
        actuator_state);

if(!state_result) {
    return 1;
}

JointState joint_state =
    state_result.value();
```

### Doxygen 语义展开

把 Backend 返回的统一 Actuator 状态映射到 Joint 侧状态

参数

- `actuator_state` 必须使用 rad、rad/s、N*m 且数量匹配

返回值

成功返回 `JointState`，失败返回 `JointActuatorMapErr`

具体使用示例

```cpp
auto actuator_result = bus->read();
if(!actuator_result) {
    return 1;
}

auto joint_result = mapper.to_joint_state(actuator_result.value());
if(!joint_result) {
    return 1;
}

const auto& joint_state = joint_result.value();
for(std::size_t i = 0; i < joint_state.pos.size(); ++i) {
    std::cout << "joint " << i
              << " q=" << joint_state.pos[i]
              << " dq=" << joint_state.vel[i]
              << " tau=" << joint_state.tor[i]
              << "\n";
}
```

---

## 42. `JointActuatorMapper::to_actuator_cmd()`

签名

```cpp
tl::expected<
    ActuatorCtrlCmd,
    JointActuatorMapErr
>
to_actuator_cmd(
    const JointCtrlCmd& joint_cmd) const;
```

逐轴核心映射

```text
actuator_pos
=
actuator_zero_offset
+
direction
*
pos_ratio
*
(joint_pos - joint_zero_offset)
```

```text
actuator_vel
=
direction
*
pos_ratio
*
joint_vel
```

```text
actuator_tor
=
direction
*
joint_tor
/
tor_ratio
```

增益使用

```text
gain_ratio
=
pos_ratio * tor_ratio
```

```text
actuator_kp
=
joint_kp / gain_ratio
```

```text
actuator_kd
=
joint_kd / gain_ratio
```

### Doxygen 语义展开

把 Joint 侧完整 MIT 风格命令转换为 Backend 可以发送的 Actuator 侧命令

参数

- `joint_cmd` 必须包含相同长度的 pos、vel、tor、kp、kd

返回值

成功返回 `ActuatorCtrlCmd`，失败返回 `JointActuatorMapErr`

具体使用示例

```cpp
serial_arm::JointCtrlCmd joint_cmd;
joint_cmd.pos = target_pos;
joint_cmd.vel.assign(target_pos.size(), 0.0);
joint_cmd.tor.assign(target_pos.size(), 0.0);
joint_cmd.kp = cfg.ctrller.rigid_tracking_gains.kp;
joint_cmd.kd = cfg.ctrller.rigid_tracking_gains.kd;

auto actuator_cmd = mapper.to_actuator_cmd(joint_cmd);
if(!actuator_cmd) {
    return 1;
}

const auto& cmd = actuator_cmd.value();
std::cout << "actuator1 target=" << cmd.pos[0]
          << " kp=" << cmd.kp[0] << "\n";
```

使用注意

- 普通业务代码不应绕过 `Robot` 手工执行这条链路
- 该示例更适合 Mapper 单元测试或 Backend 联调

---

## 43. `JointActuatorMapper::size()`

签名

```cpp
std::size_t size() const noexcept;
```

未配置时返回 `0`

配置成功后返回 `joints_count`

### Doxygen 语义展开

返回当前 Mapper 管理的 Joint/Actuator 数量

返回值

未配置返回 `0`，配置成功后返回 `joints_count`

具体使用示例

```cpp
serial_arm::JointActuatorMapper mapper;
std::cout << mapper.size() << "\n";  // 0

mapper.configure(cfg.mapper);
std::cout << mapper.size() << "\n";  // 受控关节数量
```

---

# Part VI JointCtrller

## 44. `JointCtrller`

头文件

```cpp
#include "serial_arm/core/joints_ctrller.hpp"
```

当前公共类型名的拼写就是 `JointCtrller`

应用代码需要使用这个精确名称

普通应用优先使用 `Robot`

只有在控制器单元测试、算法调试或不连接 Hardware 时才直接使用 `JointCtrller`

### 具体使用示例

直接使用控制器通常只用于离线测试

```cpp
serial_arm::JointCtrller ctrller;
ctrller.configure(cfg.ctrller);
ctrller.initialize(measured_state);
ctrller.set_impedance_mode(
    serial_arm::JointImpedanceMode::RIGID_TRACKING,
    measured_state);
ctrller.set_cmd(serial_arm::JointPosCmd{target});

serial_arm::JointCtrllerInput input;
input.state = measured_state;
input.model_feedforward.assign(target.size(), 0.0);
input.dt = 0.005;

auto output = ctrller.update(input);
```

正常真机应用优先使用 `Robot`

---

## 45. `JointCtrllerCfg`

定义

```cpp
struct JointCtrllerCfg {
    std::size_t joints_count;

    JointImpedanceGains rigid_hold_gains;
    JointImpedanceGains rigid_tracking_gains;
    JointImpedanceGains compliant_hold_gains;
    JointImpedanceGains compliant_drag_gains;
    JointImpedanceGains compliant_tracking_gains;

    bool allow_full_cmd;
};
```

每套 `kp` 和 `kd` 长度必须等于 `joints_count`

所有增益必须有限且非负

### 具体使用示例

```cpp
serial_arm::JointCtrllerCfg controller_cfg;
controller_cfg.joints_count = 6;
controller_cfg.rigid_hold_gains = cfg.ctrller.rigid_hold_gains;
controller_cfg.rigid_tracking_gains = cfg.ctrller.rigid_tracking_gains;
controller_cfg.compliant_hold_gains = cfg.ctrller.compliant_hold_gains;
controller_cfg.compliant_drag_gains = cfg.ctrller.compliant_drag_gains;
controller_cfg.compliant_tracking_gains = cfg.ctrller.compliant_tracking_gains;
controller_cfg.allow_full_cmd = false;
```

五套 gains 都必须完整提供，不能只配置当前计划使用的一种模式

---

## 46. `JointCtrller::configure()`

签名

```cpp
tl::expected<void, JointCtrllerErr>
configure(
    const JointCtrllerCfg& cfg);
```

成功后状态变为

```text
CONFIGURED
```

此时还不能 `update()`

### Doxygen 语义展开

加载五套阻抗增益和完整命令权限，完成控制器静态配置

参数

- `cfg` 为 `JointCtrllerCfg`

返回值

成功后状态变为 `CONFIGURED`，失败返回 `JointCtrllerErr`

具体使用示例

```cpp
serial_arm::JointCtrller ctrller;

auto result = ctrller.configure(cfg.ctrller);
if(!result) {
    std::cerr << "JointCtrllerErr="
              << static_cast<int>(result.error()) << "\n";
    return 1;
}

assert(ctrller.get_state() == serial_arm::JointCtrllerState::CONFIGURED);
```

---

## 47. `JointCtrller::initialize()`

签名

```cpp
tl::expected<void, JointCtrllerErr>
initialize(
    const JointState& state);
```

用途

使用真实初始关节状态初始化控制器

成功后

```text
state = INITIALIZED
mode  = RIGID_HOLD
hold position = current measured position
```

示例

```cpp
JointCtrller ctrller;

ctrller.configure(cfg.ctrller);

ctrller.initialize(current_state);
```

### Doxygen 语义展开

使用真实初始关节状态设置 hold reference，并进入可 update 的 `INITIALIZED` 状态

参数

- `state` 为当前真实 `JointState`

返回值

成功返回空 `expected`，失败返回 `JointCtrllerErr`

具体使用示例

```cpp
auto initialized = ctrller.initialize(measured_state);
if(!initialized) {
    return 1;
}

assert(ctrller.get_state() == serial_arm::JointCtrllerState::INITIALIZED);
assert(ctrller.get_impedance_mode() == serial_arm::JointImpedanceMode::RIGID_HOLD);
```

使用注意

- 不要用全零假状态代替真机当前状态，否则初始保持参考可能发生跳变

---

## 48. `JointCtrller::reset()`

签名

```cpp
void reset() noexcept;
```

作用

- 清除当前命令
- 清除 hold reference
- 回到 `CONFIGURED`
- 不重新读取静态配置

调用后必须重新 `initialize()`

### Doxygen 语义展开

清除运行时 reference 和 command，但保留静态控制器配置

返回值

无返回值，调用后状态回到 `CONFIGURED`

具体使用示例

```cpp
ctrller.reset();

if(ctrller.get_state() == serial_arm::JointCtrllerState::CONFIGURED) {
    ctrller.initialize(new_measured_state);
}
```

使用注意

- reset 后不能直接 update，必须重新 initialize

---

## 49. `JointCtrller::set_impedance_mode()`

签名

```cpp
tl::expected<void, JointCtrllerErr>
set_impedance_mode(
    JointImpedanceMode mode,
    const JointState& state);
```

`state` 用于把模式切换参考重新对齐到当前实测状态

切换 mode 会清除已有 tracking command

### Doxygen 语义展开

切换五种关节阻抗模式，并用当前实测状态重新对齐 hold 或 fallback reference

参数

- `mode` 为目标阻抗模式
- `state` 为切换瞬间当前实测关节状态

返回值

成功返回空 `expected`，失败返回 `JointCtrllerErr`

具体使用示例

```cpp
auto mode_result = ctrller.set_impedance_mode(
    serial_arm::JointImpedanceMode::COMPLIANT_TRACKING,
    measured_state);

if(!mode_result) {
    return 1;
}

serial_arm::JointPosCmd target;
target.pos = measured_state.pos;
target.pos[0] += 0.05;

ctrller.set_cmd(target);
```

使用注意

- 模式切换会清除之前的 tracking command，切换到 tracking 后要重新 set_cmd

---

## 50. `JointCtrller::set_cmd()`

签名

```cpp
tl::expected<void, JointCtrllerErr>
set_cmd(
    const JointCmd& cmd);
```

前置条件

- 已 configure
- 已 initialize
- 当前 mode 是 `RIGID_TRACKING` 或 `COMPLIANT_TRACKING`

否则返回

```text
CMD_NOT_ALLOWED_IN_MODE
```

### Doxygen 语义展开

设置位置、位置速度或位置速度力矩三类 tracking reference

参数

- `cmd` 为 `JointCmd` variant 中任意一种参考命令

返回值

成功返回空 `expected`，非 tracking mode 返回 `CMD_NOT_ALLOWED_IN_MODE`

具体使用示例

```cpp
ctrller.set_impedance_mode(
    serial_arm::JointImpedanceMode::RIGID_TRACKING,
    measured_state);

serial_arm::JointPosVelCmd cmd;
cmd.pos = measured_state.pos;
cmd.vel.assign(cmd.pos.size(), 0.0);
cmd.pos[1] += 0.03;

const auto result = ctrller.set_cmd(cmd);
if(!result) {
    std::cerr << static_cast<int>(result.error()) << "\n";
}
```

---

## 51. `JointCtrller::set_full_cmd()`

签名

```cpp
tl::expected<void, JointCtrllerErr>
set_full_cmd(
    const JointCtrlCmd& cmd);
```

额外前置条件

```text
allow_full_cmd = true
```

否则返回

```text
FULL_CMD_NOT_ALLOWED
```

### Doxygen 语义展开

直接设置完整 Joint 侧 pos、vel、tor、kp、kd 命令

参数

- `cmd` 为完整 `JointCtrlCmd`

返回值

成功返回空 `expected`，未开启 `allow_full_cmd` 返回 `FULL_CMD_NOT_ALLOWED`

具体使用示例

```cpp
serial_arm::JointCtrlCmd cmd;
cmd.pos = measured_state.pos;
cmd.vel.assign(n, 0.0);
cmd.tor.assign(n, 0.0);
cmd.kp.assign(n, 5.0);
cmd.kd.assign(n, 0.1);

const auto result = ctrller.set_full_cmd(cmd);
if(!result) {
    std::cerr << "set_full_cmd failed: "
              << static_cast<int>(result.error()) << "\n";
}
```

使用注意

- 只在确实需要逐周期覆盖 kp/kd 时开启 full command

---

## 52. `JointCtrller::update()`

输入

```cpp
struct JointCtrllerInput {
    JointState state;
    JointVector model_feedforward;
    double dt;
};
```

签名

```cpp
tl::expected<
    JointCtrllerOutput,
    JointCtrllerErr
>
update(
    const JointCtrllerInput& input);
```

输出

```cpp
struct JointCtrllerOutput {
    JointCtrlCmd cmd;
};
```

独立控制器示例

```cpp
JointCtrllerInput input;

input.state = measured_state;
input.model_feedforward =
    JointVector(n, 0.0);
input.dt = 0.005;

auto result =
    ctrller.update(input);

if(result) {
    const JointCtrlCmd& cmd =
        result->cmd;
}
```

### Doxygen 语义展开

根据当前 Joint 状态、模型前馈和 dt 生成一帧完整 Joint 控制命令

参数

- `input.state` 为当前关节状态
- `input.model_feedforward` 为 Joint 侧前馈力矩
- `input.dt` 为正有限控制周期

返回值

成功返回 `JointCtrllerOutput`，其中 `cmd` 是完整 `JointCtrlCmd`

具体使用示例

```cpp
serial_arm::JointCtrllerInput input;
input.state = measured_state;
input.model_feedforward.assign(n, 0.0);
input.dt = 0.005;

auto output = ctrller.update(input);
if(!output) {
    return 1;
}

const serial_arm::JointCtrlCmd& joint_cmd = output->cmd;
std::cout << "joint1 kp=" << joint_cmd.kp[0]
          << " target=" << joint_cmd.pos[0] << "\n";
```

---

## 53. `JointCtrller::get_state()`

```cpp
JointCtrllerState
get_state() const noexcept;
```

返回

```text
UNCONFIGURED
CONFIGURED
INITIALIZED
```

### Doxygen 语义展开

读取 `JointCtrller` 当前生命周期状态

返回值

返回 `UNCONFIGURED`、`CONFIGURED` 或 `INITIALIZED`

具体使用示例

```cpp
switch(ctrller.get_state()) {
case serial_arm::JointCtrllerState::UNCONFIGURED:
    std::cout << "need configure\n";
    break;
case serial_arm::JointCtrllerState::CONFIGURED:
    std::cout << "need initialize\n";
    break;
case serial_arm::JointCtrllerState::INITIALIZED:
    std::cout << "ready to update\n";
    break;
}
```

---

## 54. `JointCtrller::get_impedance_mode()`

```cpp
JointImpedanceMode
get_impedance_mode() const noexcept;
```

返回当前模式

### Doxygen 语义展开

读取当前阻抗模式，常用于调试、状态显示和测试断言

返回值

返回 `JointImpedanceMode`

具体使用示例

```cpp
if(ctrller.get_impedance_mode() ==
   serial_arm::JointImpedanceMode::COMPLIANT_DRAG) {
    std::cout << "manual drag mode\n";
}
```

---

# Part VII Safety

## 55. `JointLimitCfg`

定义

```cpp
struct JointLimitCfg {
    std::vector<std::uint8_t> has_position_limit;
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

这是已经解析到 Joint 侧的最终限制

continuous joint 可以让 `has_position_limit[i] == 0`

此时不执行绝对位置上下限检查，但速度、加速度、effort、kp、kd 和超时检查仍然有效

### 具体使用示例

```cpp
const serial_arm::JointLimitCfg& limits = cfg.safety.limits;

for(std::size_t i = 0; i < cfg.joint_names.size(); ++i) {
    std::cout << cfg.joint_names[i]
              << " max_vel=" << limits.max_vel[i]
              << " max_acc=" << limits.max_acc[i]
              << " max_effort=" << limits.max_effort[i]
              << "\n";
}
```

`has_position_limit[i] == 0` 时不要访问该 Joint 的绝对位置限制去做业务判断

---

## 56. `FaultCompliantRecoveryCfg`

定义

```cpp
struct FaultCompliantRecoveryCfg {
    JointVector kp;
    JointVector kd;
    JointVector max_vel;
    double effort_scale;
};
```

用于 FAULT 内部受限柔性恢复

`effort_scale` 用于限制恢复阶段保留的模型补偿力矩比例

### 具体使用示例

```cpp
serial_arm::FaultCompliantRecoveryCfg recovery;
recovery.kp = {8.0, 20.0, 12.0, 5.0, 3.0, 1.0};
recovery.kd = {0.08, 0.20, 0.12, 0.05, 0.03, 0.01};
recovery.max_vel = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
recovery.effort_scale = 0.5;
```

这组参数只用于 FAULT 内部的受限柔性恢复，不等同于正常 `COMPLIANT_DRAG` gains

---

## 57. `FaultRecoveryCfg`

定义

```cpp
struct FaultRecoveryCfg {
    FaultHoldMode default_mode;
    bool allow_compliant_recovery;
    bool require_operator_request;
    bool gravity_model_validated;
    double recovery_timeout_s;
    FaultCompliantRecoveryCfg compliant_recovery;
};
```

正常策略应保持

```text
default_mode = RIGID_HOLD
```

柔性恢复不应自动进入

### 具体使用示例

```cpp
serial_arm::FaultRecoveryCfg recovery_cfg;
recovery_cfg.default_mode = serial_arm::FaultHoldMode::RIGID_HOLD;
recovery_cfg.allow_compliant_recovery = true;
recovery_cfg.require_operator_request = true;
// 只有完成真实重力模型验证后才能设置为 true
recovery_cfg.gravity_model_validated = true;
recovery_cfg.recovery_timeout_s = 30.0;
recovery_cfg.compliant_recovery = recovery;
```

`default_mode` 应保持 `RIGID_HOLD`，柔性恢复必须由操作员显式请求

---

## 58. `SafetyAction`

定义

```cpp
enum class SafetyAction {
    STOP_HOLD,
    DISABLE,
};
```

`STOP_HOLD` 表示状态仍可信，Robot 可以拒绝危险命令并尝试低风险保持

`DISABLE` 表示状态或硬件可信度不足，应优先失能

### 具体使用示例

```cpp
const auto action = safety.action_for(fault.code);

switch(action) {
case serial_arm::SafetyAction::STOP_HOLD:
    bus->stop();
    break;
case serial_arm::SafetyAction::DISABLE:
    bus->deactivate();
    break;
}
```

---

## 59. `FaultHoldMode`

定义

```cpp
enum class FaultHoldMode {
    RIGID_HOLD,
    COMPLIANT_RECOVERY,
};
```

它描述的是 Robot 已经进入 FAULT 后的内部保持状态

不要与正常运行时的 `JointImpedanceMode` 混淆

### 具体使用示例

```cpp
if(robot.get_state() == serial_arm::RobotState::FAULT) {
    if(robot.get_fault_hold_mode() == serial_arm::FaultHoldMode::RIGID_HOLD) {
        std::cout << "fault rigid hold\n";
    }
    else {
        std::cout << "fault compliant recovery\n";
    }
}
```

不要用 `FaultHoldMode` 表示正常 ACTIVE 下的阻抗模式

---

## 60. `SafetyCfg`

头文件

```cpp
#include "serial_arm/core/safety.hpp"
```

主要字段

```cpp
struct SafetyCfg {
    std::size_t joints_count;
    JointLimitCfg limits;

    double cmd_timeout_s;
    double state_timeout_s;
    double max_dt_s;
    double numeric_tolerance;
    double state_vel_fault_ratio;

    bool require_all_actuators_online;
    bool require_all_actuators_enabled;
    bool reject_motor_error;
    bool require_continuous_cmd;

    FaultRecoveryCfg fault_recovery;
};
```

正常应用通过 `load_robot_cfg()` 自动生成

不建议业务代码直接修改 SafetyCfg 后跳过验证

### 具体使用示例

正常代码从 `RobotCfg` 读取最终 Safety 配置

```cpp
serial_arm::Safety safety;

const serial_arm::SafetyCfg& safety_cfg = cfg.safety;
auto result = safety.configure(safety_cfg);
if(!result) {
    std::cerr << "invalid Safety config\n";
}
```

业务层不要绕过 `load_robot_cfg()` 自己放宽 `SafetyCfg`

---

## 61. `SafetyFault`

定义

```cpp
struct SafetyFault {
    SafetyErr code;
    std::size_t index;
    double value;
    double limit;
};
```

示例

如果第 2 个 Joint 超出速度限制，fault 可以表达

```text
code  = JOINT_VEL_LIMIT
index = 1
value = measured velocity
limit = allowed velocity
```

---

## 62. `Safety::configure()`

签名

```cpp
tl::expected<void, SafetyFault>
configure(
    const SafetyCfg& cfg);
```

必须在检查状态和命令前调用

### Doxygen 语义展开

加载最终 Joint/Actuator 安全限制和故障恢复策略

参数

- `cfg` 为已经解析完成的 `SafetyCfg`

返回值

成功返回空 `expected`，失败返回包含 index/value/limit 的 `SafetyFault`

具体使用示例

```cpp
serial_arm::Safety safety;

auto result = safety.configure(cfg.safety);
if(!result) {
    const auto& fault = result.error();
    std::cerr << "SafetyErr=" << static_cast<int>(fault.code)
              << " index=" << fault.index
              << " value=" << fault.value
              << " limit=" << fault.limit << "\n";
    return 1;
}
```

---

## 63. `Safety::check_state()`

签名

```cpp
tl::expected<void, SafetyFault>
check_state(
    const JointState& joint_state,
    const ActuatorState& actuator_state,
    double state_age_s) const;
```

检查内容包括

- JointState 维度
- ActuatorState 维度
- NaN 和 Inf
- state timeout
- Joint position
- Joint velocity
- actuator online
- actuator enabled
- actuator error

示例

```cpp
auto result =
    safety.check_state(
        joint_state,
        actuator_state,
        state_age_s);

if(!result) {
    const SafetyFault fault =
        result.error();
}
```

### Doxygen 语义展开

在 ACTIVE 控制周期中联合检查 Joint 状态与 Actuator 健康状态

参数

- `joint_state` 为映射后的 Joint 状态
- `actuator_state` 为 Backend 状态
- `state_age_s` 为距离上一帧合法状态的时间

返回值

合法返回空 `expected`，非法返回 `SafetyFault`

具体使用示例

```cpp
const double state_age_s = 0.004;

auto result = safety.check_state(
    joint_state,
    actuator_state,
    state_age_s);

if(!result) {
    const auto action = safety.action_for(result.error().code);
    if(action == serial_arm::SafetyAction::DISABLE) {
        bus->deactivate();
    }
}
```

---

## 64. `Safety::check_cmd_age()`

签名

```cpp
tl::expected<void, SafetyFault>
check_cmd_age(
    double cmd_age_s) const;
```

用于 tracking command timeout

如果 `cmd_age_s > cmd_timeout_s`，返回 `CMD_TIMEOUT`

### Doxygen 语义展开

检查 tracking reference 是否已经超过允许的命令刷新时间

参数

- `cmd_age_s` 为当前时间距离最后一帧合法外部命令的秒数

返回值

合法返回空 `expected`，超时通常返回 `CMD_TIMEOUT`

具体使用示例

```cpp
const double cmd_age_s = 0.12;
const auto result = safety.check_cmd_age(cmd_age_s);

if(!result && result.error().code == serial_arm::SafetyErr::CMD_TIMEOUT) {
    std::cerr << "tracking command timeout\n";
}
```

---

## 65. `Safety::check_joint_cmd()`

签名

```cpp
tl::expected<
    JointCtrlCmd,
    SafetyFault
>
check_joint_cmd(
    const JointState& state,
    const JointCtrlCmd& cmd,
    double dt);
```

它不是只返回 true 或 false

对于浮点误差级轻微越界，Safety 可以执行 clamp，并返回安全命令

因此应该使用返回值继续下游流程

正确

```cpp
auto safe_result =
    safety.check_joint_cmd(
        state,
        cmd,
        dt);

if(!safe_result) {
    return;
}

JointCtrlCmd safe_cmd =
    safe_result.value();
```

不要检查通过后继续使用原来的 `cmd`

### Doxygen 语义展开

检查一帧 Joint 控制命令是否满足位置、速度、力矩、增益和连续性限制，并返回真正应该下发的安全命令

参数

- `state` 为当前 Joint 状态
- `cmd` 为控制器输出
- `dt` 为当前控制周期

返回值

成功返回可能经过浮点误差级 clamp 的 `JointCtrlCmd`，失败返回 `SafetyFault`

具体使用示例

```cpp
auto safe_result = safety.check_joint_cmd(
    joint_state,
    controller_output.cmd,
    dt);

if(!safe_result) {
    const auto& fault = safe_result.error();
    std::cerr << "rejected command at joint " << fault.index << "\n";
    return 1;
}

const serial_arm::JointCtrlCmd safe_cmd = safe_result.value();
auto actuator_cmd = mapper.to_actuator_cmd(safe_cmd);
```

使用注意

- 通过检查后要使用返回的 `safe_cmd`，不要继续下发原始 `cmd`

---

## 66. `Safety::reset_cmd_history()`

签名

```cpp
tl::expected<void, SafetyFault>
reset_cmd_history(
    const JointState& state);
```

作用

把命令连续性历史重置到当前实测位置和零速度

典型使用场景

- activate
- impedance mode 切换
- fault recovery
- reference reset

### Doxygen 语义展开

把 command continuity 历史重置到当前实测位置和零目标速度

参数

- `state` 为当前 Joint 状态

返回值

成功返回空 `expected`，失败返回 `SafetyFault`

具体使用示例

```cpp
const auto reset_result = safety.reset_cmd_history(joint_state);
if(!reset_result) {
    return 1;
}

// 之后第一帧命令会从当前实测位置作为连续性基准
```

使用注意

- activate、模式切换和 fault recovery 后都适合调用

---

## 67. `Safety::clear_cmd_history()`

```cpp
void clear_cmd_history() noexcept;
```

完全清除命令历史

典型用于 deactivate

### Doxygen 语义展开

完全清空命令连续性历史

返回值

无返回值

具体使用示例

```cpp
bus->deactivate();
safety.clear_cmd_history();
```

使用注意

- 典型用于退出 ACTIVE，不等同于 reset_cmd_history

---

## 68. `Safety::action_for()`

签名

```cpp
SafetyAction
action_for(
    SafetyErr err) const noexcept;
```

返回

```text
STOP_HOLD
DISABLE
```

`Robot` 使用它决定故障时是尝试安全保持还是直接失能

### Doxygen 语义展开

把具体 `SafetyErr` 映射为 Robot 应采取的故障动作

参数

- `err` 为 Safety 错误码

返回值

返回 `STOP_HOLD` 或 `DISABLE`

具体使用示例

```cpp
const serial_arm::SafetyAction action =
    safety.action_for(serial_arm::SafetyErr::ACTUATOR_OFFLINE);

if(action == serial_arm::SafetyAction::DISABLE) {
    bus->deactivate();
}
```

---

## 69. `Safety::is_configured()`

```cpp
bool is_configured() const noexcept;
```

用于诊断和测试

### Doxygen 语义展开

检查 Safety 是否已经成功 configure

返回值

已配置返回 true

具体使用示例

```cpp
if(!safety.is_configured()) {
    std::cerr << "Safety is not ready\n";
}
```

---

## 70. `Safety::clamp_count()`

```cpp
std::uint64_t clamp_count() const noexcept;
```

返回 Safety 已执行的浮点误差级 clamp 次数

如果该值持续快速增加，说明命令或边界配置值得检查

### Doxygen 语义展开

读取 Safety 对轻微数值越界执行 clamp 的累计次数

返回值

返回累计计数

具体使用示例

```cpp
const auto before = safety.clamp_count();
const auto safe_result = safety.check_joint_cmd(state, cmd, dt);
const auto after = safety.clamp_count();

if(after > before) {
    std::cout << "command was numerically clamped\n";
}
```

---

# Part VIII Dynamics

## 71. `Dynamics`

头文件

```cpp
#include "serial_arm/dynamics/dynamics.hpp"
```

Dynamics 使用 Pinocchio 维护完整运动学和动力学缓存

使用模式是

```text
configure once
    ->
update every state
    ->
read cached outputs many times
```

不要在每个周期重复 configure

### Doxygen 语义展开

构造并持有一套 Pinocchio 运动学动力学模型，推荐一个机器人实例对应一个长期存活的 Dynamics 对象

返回值

对象默认处于未配置状态，析构时释放内部模型资源

具体使用示例

```cpp
serial_arm::Dynamics dynamics;

if(!dynamics.is_configured()) {
    auto result = dynamics.configure(cfg.dynamics);
    if(!result) {
        return 1;
    }
}

// Dynamics 支持移动，不支持复制
serial_arm::Dynamics moved = std::move(dynamics);
```

---

## 72. `Dynamics::configure()`

签名

```cpp
tl::expected<void, DynamicsErr>
configure(
    const DynamicsCfg& cfg);
```

成功后

```cpp
dynamics.is_configured() == true
```

但此时还没有运行状态缓存

```cpp
dynamics.is_updated() == false
```

### Doxygen 语义展开

根据 `DynamicsCfg` 加载 URDF、受控 Joint、base/tool frame 和重力配置

参数

- `cfg` 为动力学配置

返回值

成功返回空 `expected`，失败返回 `DynamicsErr`

具体使用示例

```cpp
serial_arm::Dynamics dynamics;
const auto result = dynamics.configure(cfg.dynamics);

if(!result) {
    std::cerr << "DynamicsErr="
              << static_cast<int>(result.error()) << "\n";
    return 1;
}

const auto& info = dynamics.get_info();
std::cout << "model mass=" << info.total_mass << "\n";
```

---

## 73. `Dynamics::update()`

签名

```cpp
tl::expected<void, DynamicsErr>
update(
    const JointState& state,
    const JointVector& acc,
    const JointVector& ref_acc);
```

输入

| 参数 | 含义 |
| --- | --- |
| `state.pos` | 当前 q |
| `state.vel` | 当前 dq |
| `state.tor` | 当前反馈关节力矩 |
| `acc` | 当前估计 ddq |
| `ref_acc` | 参考关节加速度 |

一次 update 会集中计算并缓存

- gravity
- gravity compensation
- nonlinear term
- coriolis
- mass matrix
- inverse dynamics
- forward dynamics
- tool pose
- tool Jacobian

示例

```cpp
auto result =
    dynamics.update(
        state,
        joint_acc,
        joint_ref_acc);

if(!result) {
    return 1;
}
```

### Doxygen 语义展开

用当前 Joint 状态和加速度信息集中刷新所有运动学动力学缓存

参数

- `state` 提供 q、dq 和反馈 tau
- `acc` 为当前估计 ddq
- `ref_acc` 为参考 ddq

返回值

成功返回空 `expected`，之后所有缓存 Getter 都对应这一帧

具体使用示例

```cpp
const std::size_t n = cfg.joint_names.size();
serial_arm::JointVector acc(n, 0.0);
serial_arm::JointVector ref_acc(n, 0.0);

const auto update_result = dynamics.update(
    joint_state,
    acc,
    ref_acc);

if(!update_result) {
    return 1;
}

std::cout << "gravity joint2="
          << dynamics.get_gravity()[1] << "\n";
```

---

## 74. `Dynamics::set_gravity_scale()`

签名

```cpp
tl::expected<void, DynamicsErr>
set_gravity_scale(
    const JointVector& gravity_scale);
```

每个值有效范围

```text
[0, 1]
```

示例

```cpp
dynamics.set_gravity_scale({
    1.0,
    0.95,
    0.85,
    1.0,
    1.0,
    1.0,
});
```

超出范围返回

```text
GRAVITY_SCALE_OUT_OF_RANGE
```

### Doxygen 语义展开

在线修改各 Joint 的重力补偿比例，不改变 URDF 模型本身

参数

- `gravity_scale` 长度必须等于 Joint 数量，每项范围为 [0, 1]

返回值

成功返回空 `expected`，越界返回 `GRAVITY_SCALE_OUT_OF_RANGE`

具体使用示例

```cpp
serial_arm::JointVector scale = dynamics.get_gravity_scale();
scale[1] = 0.90;
scale[2] = 0.85;

const auto result = dynamics.set_gravity_scale(scale);
if(!result) {
    std::cerr << "invalid gravity scale\n";
}
```

---

## 75. `Dynamics::cleanup()`

```cpp
void cleanup();
```

释放当前模型并恢复未配置状态

### Doxygen 语义展开

释放当前 Pinocchio 模型并恢复未配置状态

返回值

无返回值

具体使用示例

```cpp
dynamics.cleanup();

assert(!dynamics.is_configured());
assert(!dynamics.is_updated());

// 如果需要换模型，可以重新 configure
dynamics.configure(other_cfg);
```

---

## 76. `Dynamics::is_configured()`

```cpp
bool is_configured() const noexcept;
```

用于确认模型是否加载成功

### Doxygen 语义展开

查询是否已经成功完成 Dynamics configure

返回值

已配置返回 true

具体使用示例

```cpp
if(!dynamics.is_configured()) {
    dynamics.configure(cfg.dynamics);
}
```

---

## 77. `Dynamics::is_updated()`

```cpp
bool is_updated() const noexcept;
```

只有至少一次 `update()` 成功后才为 true

### Doxygen 语义展开

查询是否至少成功执行过一次 update

返回值

存在有效运行时缓存时返回 true

具体使用示例

```cpp
if(dynamics.is_configured() && !dynamics.is_updated()) {
    std::cout << "model ready but no state has been updated\n";
}
```

---

## 78. `Dynamics::get_info()`

```cpp
const DynamicsInfo&
get_info() const noexcept;
```

`DynamicsInfo`

```cpp
struct DynamicsInfo {
    std::size_t joints_count;
    int nq;
    int nv;
    double total_mass;
    std::vector<std::string> joint_names;
    std::vector<int> q_indices;
    std::vector<int> v_indices;
};
```

示例

```cpp
const auto& info =
    dynamics.get_info();

std::cout
    << info.total_mass
    << "\n";
```

### Doxygen 语义展开

读取模型静态信息，包括受控 Joint、Pinocchio nq/nv 和总质量

返回值

返回内部 `DynamicsInfo` 只读引用

具体使用示例

```cpp
const serial_arm::DynamicsInfo& info = dynamics.get_info();

std::cout << "controlled joints=" << info.joints_count << "\n";
for(std::size_t i = 0; i < info.joint_names.size(); ++i) {
    std::cout << info.joint_names[i]
              << " q_index=" << info.q_indices[i]
              << " v_index=" << info.v_indices[i]
              << "\n";
}
```

---

## 79. `Dynamics::get_state()`

```cpp
const DynamicsState&
get_state() const noexcept;
```

一次获取完整缓存

适合记录日志

```cpp
const DynamicsState& s =
    dynamics.get_state();
```

`DynamicsState` 主要字段

```text
pos
vel
acc
tor
ref_acc
gravity
gravity_compensation
nonlinear
coriolis
inverse_dynamics
forward_dynamics
mass_matrix
tool_pose
tool_jacobian
```

这些字段都对应最近一次成功 `update()`

### Doxygen 语义展开

一次性读取最近一次 update 生成的完整动力学缓存

返回值

返回内部 `DynamicsState` 只读引用

具体使用示例

```cpp
if(dynamics.is_updated()) {
    const auto& s = dynamics.get_state();
    std::cout << "q0=" << s.pos[0]
              << " gravity0=" << s.gravity[0]
              << " M=" << s.mass_matrix.rows() << "x" << s.mass_matrix.cols()
              << " J=" << s.tool_jacobian.rows() << "x" << s.tool_jacobian.cols()
              << "\n";
}
```

使用注意

- 返回的是只读引用，不要跨越 Dynamics 生命周期保存引用

---

## 80. `Dynamics::get_gravity_scale()`

```cpp
const JointVector&
get_gravity_scale() const noexcept;
```

返回当前各 Joint 重力补偿缩放值

### Doxygen 语义展开

读取当前生效的重力补偿缩放数组

返回值

返回 `JointVector` 只读引用

具体使用示例

```cpp
for(const double scale : dynamics.get_gravity_scale()) {
    std::cout << scale << " ";
}
std::cout << "\n";
```

---

## 81. `Dynamics::get_frame_pose()`

签名

```cpp
tl::expected<
    Eigen::Isometry3d,
    DynamicsErr
>
get_frame_pose(
    const std::string& frame_name) const;
```

要求先成功执行 `update()`

示例

```cpp
auto pose_result =
    dynamics.get_frame_pose(
        "tool0");

if(pose_result) {
    Eigen::Matrix4d T =
        pose_result->matrix();
}
```

frame 不存在返回

```text
FRAME_NOT_FOUND
```

### Doxygen 语义展开

读取任意已存在 frame 相对 `base_frame` 的缓存位姿

参数

- `frame_name` 为 URDF/Pinocchio frame 名称

返回值

成功返回 `Eigen::Isometry3d`，frame 不存在或尚未 update 时返回 `DynamicsErr`

具体使用示例

```cpp
auto pose_result = dynamics.get_frame_pose("tool0");
if(!pose_result) {
    return 1;
}

const Eigen::Vector3d p = pose_result->translation();
std::cout << "tool xyz = "
          << p.x() << " " << p.y() << " " << p.z() << "\n";
```

---

## 82. `Dynamics::get_frame_jacobian()`

签名

```cpp
tl::expected<
    Eigen::MatrixXd,
    DynamicsErr
>
get_frame_jacobian(
    const std::string& frame_name) const;
```

返回 `6 x N` Jacobian

示例

```cpp
auto J_result =
    dynamics.get_frame_jacobian(
        "tool0");

if(J_result) {
    const Eigen::MatrixXd J =
        J_result.value();
}
```

### Doxygen 语义展开

读取任意 frame 的 LOCAL_WORLD_ALIGNED 几何 Jacobian

参数

- `frame_name` 为目标 frame 名称

返回值

成功返回 `6 x N` `Eigen::MatrixXd`，失败返回 `DynamicsErr`

具体使用示例

```cpp
auto jacobian_result = dynamics.get_frame_jacobian("tool0");
if(!jacobian_result) {
    return 1;
}

const Eigen::MatrixXd J = jacobian_result.value();
Eigen::VectorXd dq = Eigen::Map<const Eigen::VectorXd>(
    joint_state.vel.data(),
    joint_state.vel.size());

const Eigen::VectorXd twist = J * dq;
std::cout << "vx=" << twist[0] << " vy=" << twist[1] << " vz=" << twist[2] << "\n";
```

---

## 83. Dynamics 缓存 Getter

以下接口都读取最近一次成功 `update()` 的缓存

```cpp
const JointVector&
get_gravity() const noexcept;

const JointVector&
get_gravity_compensation() const noexcept;

const JointVector&
get_nonlinear() const noexcept;

const JointVector&
get_coriolis() const noexcept;

const Eigen::MatrixXd&
get_mass_matrix() const noexcept;

const JointVector&
get_inverse_dynamics() const noexcept;

const JointVector&
get_forward_dynamics() const noexcept;

const Eigen::Isometry3d&
get_tool_pose() const noexcept;

const Eigen::MatrixXd&
get_tool_jacobian() const noexcept;
```

典型用法

```cpp
dynamics.update(
    state,
    acc,
    ref_acc);

const JointVector& gravity =
    dynamics.get_gravity();

const Eigen::MatrixXd& M =
    dynamics.get_mass_matrix();

const Eigen::Isometry3d& T =
    dynamics.get_tool_pose();

const Eigen::MatrixXd& J =
    dynamics.get_tool_jacobian();
```

### Doxygen 语义展开

这些 Getter 都读取最近一次成功 `update()` 的缓存，适合在一个控制周期内重复读取而不重复计算 Pinocchio

返回值

每个 Getter 返回对应缓存的只读引用

具体使用示例

```cpp
dynamics.update(joint_state, joint_acc, joint_ref_acc);

const auto& g = dynamics.get_gravity();
const auto& g_comp = dynamics.get_gravity_compensation();
const auto& nle = dynamics.get_nonlinear();
const auto& coriolis = dynamics.get_coriolis();
const auto& M = dynamics.get_mass_matrix();
const auto& tau_id = dynamics.get_inverse_dynamics();
const auto& ddq_fd = dynamics.get_forward_dynamics();
const auto& T_tool = dynamics.get_tool_pose();
const auto& J_tool = dynamics.get_tool_jacobian();

std::cout << "g0=" << g[0]
          << " g_comp0=" << g_comp[0]
          << " tau_id0=" << tau_id[0]
          << "\n";
```

使用注意

- 调用这些 Getter 前应保证 `is_updated()==true`

---

# Part IX MotorBus 与 HardwareLoader

## 84. `MotorBus`

头文件

```cpp
#include "serial_arm/hardware/motor_bus.hpp"
```

`MotorBus` 是 Hardware Backend 必须实现的抽象接口

应用层通常只持有

```cpp
std::unique_ptr<MotorBus>
```

### 具体使用示例

开发新 Backend 时通过继承实现 Hardware Contract

```cpp
class MyBus final : public serial_arm::MotorBus {
public:
    tl::expected<void, serial_arm::MotorBusErr>
    configure(const std::string& path) override;

    tl::expected<void, serial_arm::MotorBusErr> connect() override;
    tl::expected<serial_arm::ActuatorState, serial_arm::MotorBusErr> read() override;
    tl::expected<void, serial_arm::MotorBusErr> activate() override;
    tl::expected<void, serial_arm::MotorBusErr>
    write(const serial_arm::ActuatorCtrlCmd& cmd) override;
    tl::expected<void, serial_arm::MotorBusErr> stop() override;
    tl::expected<void, serial_arm::MotorBusErr> deactivate() override;
    tl::expected<void, serial_arm::MotorBusErr> recover() override;
    const serial_arm::HardwareCapabilities& capabilities() const noexcept override;
    void cleanup() noexcept override;
    std::size_t size() const noexcept override;
};
```

实现完成后再通过 `create_motor_bus()` 和 `destroy_motor_bus()` 导出给 `HardwareLoader`

---

## 85. `MotorBus::configure()`

```cpp
virtual tl::expected<void, MotorBusErr>
configure(
    const std::string& config_path) = 0;
```

职责

- 读取 Backend YAML
- 校验 actuator 数量
- 校验 ID、型号和通信参数
- 建立内部配置对象

不应在这里使能执行器或发送运动命令

### Doxygen 语义展开

Backend 读取并校验自己的 YAML，建立配置但不驱动真实执行器运动

参数

- `config_path` 为 Backend 专属 YAML

返回值

实现应返回 `tl::expected<void, MotorBusErr>`

具体使用示例

```cpp
tl::expected<void, serial_arm::MotorBusErr>
MyBus::configure(const std::string& config_path) {
    if(config_path.empty()) {
        return tl::make_unexpected(serial_arm::MotorBusErr::INVALID_CFG);
    }

    cfg_ = load_my_backend_yaml(config_path);
    capabilities_ = build_capabilities(cfg_);
    configured_ = true;
    return {};
}
```

使用注意

- 这里可以解析配置和准备静态 capability，但不要使能电机

---

## 86. `MotorBus::connect()`

```cpp
virtual tl::expected<void, MotorBusErr>
connect() = 0;
```

职责

打开串口、CAN、EtherCAT 或其他底层设备

connect 成功不等于执行器已使能

### Doxygen 语义展开

Backend 打开底层串口、CAN、EtherCAT 或其他设备连接

返回值

连接成功返回空 `expected`，打开失败返回 `OPEN_FAILED` 或更具体错误

具体使用示例

```cpp
tl::expected<void, serial_arm::MotorBusErr> MyBus::connect() {
    if(!configured_) {
        return tl::make_unexpected(serial_arm::MotorBusErr::NOT_CONFIGURED);
    }

    if(!transport_.open(cfg_.device)) {
        return tl::make_unexpected(serial_arm::MotorBusErr::OPEN_FAILED);
    }

    connected_ = true;
    return {};
}
```

---

## 87. `MotorBus::read()`

```cpp
virtual tl::expected<
    ActuatorState,
    MotorBusErr
>
read() = 0;
```

返回值必须已经转换为 SerialArm 统一单位

```text
pos rad
vel rad/s
tor N*m
```

### Doxygen 语义展开

读取执行器状态并转换成 SerialArm Hardware Contract 的统一单位

返回值

成功返回 `ActuatorState`，通信或状态失败返回 `MotorBusErr`

具体使用示例

```cpp
tl::expected<serial_arm::ActuatorState, serial_arm::MotorBusErr>
MyBus::read() {
    if(!connected_) {
        return tl::make_unexpected(serial_arm::MotorBusErr::NOT_CONNECTED);
    }

    serial_arm::ActuatorState state;
    state.pos.resize(size());
    state.vel.resize(size());
    state.tor.resize(size());
    state.online.assign(size(), 1);
    state.enabled.assign(size(), 1);
    state.err_code.assign(size(), 0);

    for(std::size_t i = 0; i < size(); ++i) {
        const auto raw = driver_.read_motor(i);
        state.pos[i] = raw.position_rad;
        state.vel[i] = raw.velocity_rad_s;
        state.tor[i] = raw.torque_nm;
    }

    return state;
}
```

使用注意

- 如果厂商只给电流，应在 Backend 内完成电流到 N*m 的换算

---

## 88. `MotorBus::activate()`

```cpp
virtual tl::expected<void, MotorBusErr>
activate() = 0;
```

典型职责

- 使能执行器
- 切换控制模式
- 确认反馈
- 进入可写状态

### Doxygen 语义展开

使 Backend 进入可写控制状态，通常在这里完成电机使能和控制模式切换

返回值

成功返回空 `expected`，使能或模式切换失败返回对应错误

具体使用示例

```cpp
tl::expected<void, serial_arm::MotorBusErr> MyBus::activate() {
    if(!connected_) {
        return tl::make_unexpected(serial_arm::MotorBusErr::NOT_CONNECTED);
    }

    for(std::size_t i = 0; i < size(); ++i) {
        if(!driver_.enable(i)) {
            return tl::make_unexpected(serial_arm::MotorBusErr::ENABLE_FAILED);
        }
        if(!driver_.switch_to_mit(i)) {
            return tl::make_unexpected(serial_arm::MotorBusErr::MODE_SWITCH_FAILED);
        }
    }

    active_ = true;
    return {};
}
```

---

## 89. `MotorBus::write()`

```cpp
virtual tl::expected<void, MotorBusErr>
write(
    const ActuatorCtrlCmd& cmd) = 0;
```

Backend 必须接受完整

```text
pos
vel
tor
kp
kd
```

如果厂商协议的数据单位不同，在这里转换

### Doxygen 语义展开

把 Core 给出的完整 Actuator MIT 语义转换为厂商协议并发送

参数

- `cmd` 包含 pos、vel、tor、kp、kd

返回值

成功返回空 `expected`，非法命令或通信失败返回 `MotorBusErr`

具体使用示例

```cpp
tl::expected<void, serial_arm::MotorBusErr>
MyBus::write(const serial_arm::ActuatorCtrlCmd& cmd) {
    if(!active_) {
        return tl::make_unexpected(serial_arm::MotorBusErr::NOT_ACTIVE);
    }
    if(cmd.pos.size() != size()) {
        return tl::make_unexpected(serial_arm::MotorBusErr::INVALID_CMD);
    }

    for(std::size_t i = 0; i < size(); ++i) {
        const bool ok = driver_.send_mit(
            i,
            cmd.pos[i],
            cmd.vel[i],
            cmd.kp[i],
            cmd.kd[i],
            cmd.tor[i]);
        if(!ok) {
            return tl::make_unexpected(serial_arm::MotorBusErr::WRITE_FAILED);
        }
    }

    return {};
}
```

---

## 90. `MotorBus::stop()`

```cpp
virtual tl::expected<void, MotorBusErr>
stop() = 0;
```

用于故障处理或安全停止

Backend 可以实现为

- 刹停
- 当前位置保持
- 多次刷新低风险停止命令

具体策略由 Backend 决定，但必须符合 Hardware Contract

### Doxygen 语义展开

请求 Backend 停止当前运动或持续刷新安全保持命令

返回值

成功返回空 `expected`，停止失败返回 `STOP_FAILED`

具体使用示例

```cpp
tl::expected<void, serial_arm::MotorBusErr> MyBus::stop() {
    for(std::size_t i = 0; i < size(); ++i) {
        if(!driver_.hold_current(i, stop_kp_, stop_kd_)) {
            return tl::make_unexpected(serial_arm::MotorBusErr::STOP_FAILED);
        }
    }
    return {};
}
```

使用注意

- stop 的具体物理策略由 Backend 决定，但必须是低风险行为

---

## 91. `MotorBus::deactivate()`

```cpp
virtual tl::expected<void, MotorBusErr>
deactivate() = 0;
```

退出可写状态并失能执行器

### Doxygen 语义展开

退出可写状态并失能执行器

返回值

成功返回空 `expected`，失能失败返回 `DISABLE_FAILED`

具体使用示例

```cpp
tl::expected<void, serial_arm::MotorBusErr> MyBus::deactivate() {
    for(std::size_t i = 0; i < size(); ++i) {
        if(!driver_.disable(i)) {
            return tl::make_unexpected(serial_arm::MotorBusErr::DISABLE_FAILED);
        }
    }
    active_ = false;
    return {};
}
```

---

## 92. `MotorBus::recover()`

```cpp
virtual tl::expected<void, MotorBusErr>
recover() = 0;
```

用于尝试清理 Backend 或执行器错误状态

它不等于 Robot 的 `clear_fault()`

Robot fault 和 hardware fault 是两个层级

### Doxygen 语义展开

尝试清理硬件错误并让 Backend 恢复到可以重新使用的状态

返回值

成功返回空 `expected`，恢复失败返回 `RECOVER_FAILED`

具体使用示例

```cpp
tl::expected<void, serial_arm::MotorBusErr> MyBus::recover() {
    for(std::size_t i = 0; i < size(); ++i) {
        if(!driver_.clear_fault(i)) {
            return tl::make_unexpected(serial_arm::MotorBusErr::RECOVER_FAILED);
        }
    }
    return {};
}
```

使用注意

- Hardware recover 不等于 Robot clear_fault，两层状态机必须分别满足条件

---

## 93. `MotorBus::capabilities()`

```cpp
virtual const HardwareCapabilities&
capabilities() const noexcept = 0;
```

必须在 Core 配置解析阶段可用

因此 Backend 的 capability 信息不应依赖 Robot 已经 ACTIVE

### Doxygen 语义展开

返回 Core Safety 解析需要的执行器物理能力

返回值

返回 `HardwareCapabilities` 只读引用

具体使用示例

```cpp
const serial_arm::HardwareCapabilities& MyBus::capabilities() const noexcept {
    return capabilities_;
}

// 调用侧
for(const auto& capability : bus->capabilities()) {
    std::cout << capability.actuator_name
              << " max_torque=" << capability.max_effort << "\n";
}
```

使用注意

- capabilities 必须在 Robot ACTIVE 之前就可用

---

## 94. `MotorBus::cleanup()`

```cpp
virtual void cleanup() noexcept = 0;
```

要求

- noexcept
- 可以重复调用
- 释放通信资源
- 不抛异常

### Doxygen 语义展开

无异常释放 Backend 持有的底层资源，并允许重复调用

返回值

无返回值且必须 `noexcept`

具体使用示例

```cpp
void MyBus::cleanup() noexcept {
    active_ = false;
    connected_ = false;
    transport_.close_noexcept();
}
```

使用注意

- 析构和失败路径都可能调用 cleanup，因此必须幂等

---

## 95. `MotorBus::size()`

```cpp
virtual std::size_t
size() const noexcept = 0;
```

必须等于 Robot 受控 Joint 数量

否则 `Robot::configure()` 返回

```text
MOTOR_BUS_SIZE_MISMATCH
```

### Doxygen 语义展开

返回 Backend 管理的执行器数量

返回值

返回值必须与 Robot 受控 Joint 数量一致

具体使用示例

```cpp
std::size_t MyBus::size() const noexcept {
    return actuators_.size();
}

if(bus->size() != cfg.joint_names.size()) {
    std::cerr << "joint/actuator count mismatch\n";
}
```

---

## 96. `HardwareLoader`

头文件

```cpp
#include "serial_arm/hardware/hardware_loader.hpp"
```

签名

```cpp
tl::expected<
    std::unique_ptr<MotorBus>,
    HardwareLoaderErr
>
load(
    const std::string& plugin,
    const std::string& config_path);
```

作用

- `dlopen()` Backend shared library
- 查找 `create_motor_bus`
- 查找 `destroy_motor_bus`
- 创建 MotorBus
- 调用 `MotorBus::configure()`
- 把 Backend 对象生命周期与 DSO 生命周期绑定

示例

```cpp
HardwareLoader loader;

auto result = loader.load(
    "serial_arm_hardware_damiao",
    "/path/to/hardware.yaml");

if(!result) {
    return 1;
}

std::unique_ptr<MotorBus> bus =
    std::move(result.value());
```

如果 `plugin` 不包含路径分隔符，Loader 还会尝试

```text
lib<plugin>.so
```

### Doxygen 语义展开

动态加载 Backend shared library，创建 MotorBus 并自动调用其 configure

参数

- `plugin` 为共享库路径或插件名
- `config_path` 为 Backend YAML

返回值

成功返回拥有 Backend 和 DSO 生命周期的 `std::unique_ptr<MotorBus>`，失败返回 `HardwareLoaderErr`

具体使用示例

```cpp
serial_arm::HardwareLoader loader;

auto result = loader.load(
    "serial_arm_hardware_damiao",
    "/opt/serial_arm/share/dm_arm_description/config/hardware.yaml");

if(!result) {
    std::cerr << "HardwareLoaderErr="
              << static_cast<int>(result.error()) << "\n";
    return 1;
}

std::unique_ptr<serial_arm::MotorBus> bus =
    std::move(result.value());

std::cout << "actuator count=" << bus->size() << "\n";
```

使用注意

- 返回的 `MotorBus` wrapper 自己绑定 DSO 生命周期，不需要调用者手工 dlclose

---

# Part X Robot

## 97. `Robot`

头文件

```cpp
#include "serial_arm/robot.hpp"
```

这是 C++ 应用的主要控制入口

生命周期

```text
UNCONFIGURED
    |
    | configure
    v
INACTIVE
    |
    | activate
    v
ACTIVE
    |
    | cycle
    v
ACTIVE
    |
    | deactivate
    v
INACTIVE
```

任意关键错误可能进入

```text
FAULT
```

### 构造与析构使用示例

`Robot` 默认构造后处于 `UNCONFIGURED`，析构时如果仍持有 MotorBus 会尽力安全失能并调用 Backend cleanup

```cpp
{
    serial_arm::Robot robot;
    assert(robot.get_state() == serial_arm::RobotState::UNCONFIGURED);

    auto result = robot.configure(cfg, std::move(bus), model_feedforward);
    if(!result) {
        return 1;
    }

    // 后续即使函数提前 return，Robot 析构仍会尽力释放硬件资源
}
```

不要用析构代替正常 `deactivate()`，正常退出仍应显式调用生命周期接口

---

## 98. `Robot::configure()`

签名

```cpp
tl::expected<void, RobotFault>
configure(
    const RobotCfg& cfg,
    std::unique_ptr<MotorBus> motor_bus,
    ModelFeedforwardFn model_feedforward = {});
```

职责

- 验证 Core 配置
- 检查 MotorBus 数量
- 配置 JointCtrller
- 配置 Mapper
- 配置 Safety
- 接管 MotorBus 所有权
- 保存模型前馈函数
- 进入 `INACTIVE`

注意

如果

```text
cfg.runtime.model_feedforward_mode != NONE
```

但没有传 `model_feedforward`

configure 会失败

完整示例见文档末尾的功能示例 A

### Doxygen 语义展开

配置顶层 Robot 闭环并接管 MotorBus 所有权，不连接也不使能硬件

参数

- `cfg` 为完整 Robot 配置
- `motor_bus` 为已被 HardwareLoader configure 的 Backend
- `model_feedforward` 为可选模型前馈回调

返回值

成功后 Robot 进入 `INACTIVE`，失败返回含子错误的 `RobotFault`

具体使用示例

```cpp
serial_arm::Robot robot;

auto configure_result = robot.configure(
    cfg,
    std::move(bus),
    model_feedforward);

if(!configure_result) {
    const auto& fault = configure_result.error();
    std::cerr << "RobotErr=" << static_cast<int>(fault.code) << "\n";
    return 1;
}

assert(robot.get_state() == serial_arm::RobotState::INACTIVE);
```

使用注意

- configure 会接管 `motor_bus`，调用后不要继续使用原 unique_ptr
- 非 NONE 前馈模式必须同时提供有效 `model_feedforward`

---

## 99. `Robot::activate()`

签名

```cpp
tl::expected<void, RobotFault>
activate();
```

前置条件

- 已 configure
- 当前为 INACTIVE
- `write_enabled=true`

内部流程

```text
MotorBus::connect
    ->
MotorBus::activate
    ->
MotorBus::read
    ->
Mapper::to_joint_state
    ->
Safety::check_state
    ->
JointCtrller::initialize
    ->
Safety::reset_cmd_history
    ->
ACTIVE
```

`write_enabled=false` 时返回

```text
WRITE_DISABLED
```

这是保护行为，不是 Backend 故障

### Doxygen 语义展开

连接和使能真实 Backend，读取第一帧状态，并用真实状态初始化 Mapper、Safety command history 和 JointCtrller

返回值

成功进入 `ACTIVE`，失败返回 `RobotFault` 并可能进入 `FAULT`

具体使用示例

```cpp
if(robot.get_state() != serial_arm::RobotState::INACTIVE) {
    return 1;
}

auto result = robot.activate();
if(!result) {
    std::cerr << "activate RobotErr="
              << static_cast<int>(result.error().code) << "\n";
    return 1;
}

const auto& q0 = robot.get_joint_state().pos;
std::cout << "activated at q0=" << q0[0] << "\n";
```

使用注意

- `runtime.write_enabled=false` 时该接口明确返回 `WRITE_DISABLED`
- activate 成功后的默认控制器模式是按当前实测位置初始化的保持状态

---

## 100. `Robot::set_cmd()`

签名

```cpp
tl::expected<void, RobotFault>
set_cmd(
    const JointCmd& cmd,
    TimePoint now = Clock::now());
```

前置条件

- Robot 是 ACTIVE
- 当前 JointCtrller mode 是 tracking mode
- 时间戳不回退

正确流程

```cpp
robot.set_impedance_mode(
    JointImpedanceMode::RIGID_TRACKING);

while(running) {
    const auto now =
        Robot::Clock::now();

    robot.set_cmd(
        JointPosCmd{target},
        now);

    robot.cycle(now);
}
```

为什么持续刷新

tracking command 会受 `cmd_timeout_s` 检查

### Doxygen 语义展开

向 tracking 模式提交一帧上层参考命令，并记录命令时间用于 timeout 检查

参数

- `cmd` 为 `JointPosCmd`、`JointPosVelCmd` 或 `JointPosVelTorCmd`
- `now` 必须不早于最近 cycle 和上一帧 command 时间

返回值

成功返回空 `expected`，失败返回 `RobotFault`

具体使用示例

```cpp
robot.set_impedance_mode(serial_arm::JointImpedanceMode::RIGID_TRACKING);

while(running) {
    const auto now = serial_arm::Robot::Clock::now();

    serial_arm::JointPosCmd cmd;
    cmd.pos = target;

    auto command_result = robot.set_cmd(cmd, now);
    if(!command_result) {
        break;
    }

    auto cycle_result = robot.cycle(now);
    if(!cycle_result) {
        break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
}
```

使用注意

- tracking 模式下必须按小于 `cmd_timeout_s` 的间隔持续刷新命令

---

## 101. `Robot::set_full_cmd()`

签名

```cpp
tl::expected<void, RobotFault>
set_full_cmd(
    const JointCtrlCmd& cmd,
    TimePoint now = Clock::now());
```

前置条件

- ACTIVE
- tracking mode
- `allow_full_cmd=true`

用途

需要直接控制

```text
pos
vel
tor
kp
kd
```

的高级控制器

### Doxygen 语义展开

向 Robot 直接提交完整 Joint MIT 命令，使上层可以逐周期覆盖 pos、vel、tor、kp、kd

参数

- `cmd` 为完整 `JointCtrlCmd`
- `now` 为当前单调时钟时间

返回值

成功返回空 `expected`，控制器未允许 full command 时会通过 `CTRLLER_FAILED` 暴露子错误

具体使用示例

```cpp
robot.set_impedance_mode(serial_arm::JointImpedanceMode::RIGID_TRACKING);

serial_arm::JointCtrlCmd cmd;
cmd.pos = robot.get_joint_state().pos;
cmd.vel.assign(n, 0.0);
cmd.tor.assign(n, 0.0);
cmd.kp.assign(n, 8.0);
cmd.kd.assign(n, 0.2);

const auto now = serial_arm::Robot::Clock::now();
auto result = robot.set_full_cmd(cmd, now);
if(result) {
    robot.cycle(now);
}
```

使用注意

- 必须先在配置中设置 `allow_full_cmd=true`

---

## 102. `Robot::set_impedance_mode()`

签名

```cpp
tl::expected<void, RobotFault>
set_impedance_mode(
    JointImpedanceMode mode,
    TimePoint now = Clock::now());
```

要求

- ACTIVE
- 已有合法 state
- 时间戳不回退

切换模式后 Robot 会

- 用当前实测状态重建 reference
- 重置 Safety command history
- 清除 external command 状态
- 重置 joint reference acceleration

因此切换到 tracking 后应重新发送目标

### Doxygen 语义展开

切换 Robot 正常运行时的阻抗模式，并自动把 reference 与 Safety history 对齐到当前实测状态

参数

- `mode` 为目标模式
- `now` 为单调时间点

返回值

成功返回空 `expected`，非 ACTIVE 或时间回退返回 `RobotFault`

具体使用示例

```cpp
const auto mode_result = robot.set_impedance_mode(
    serial_arm::JointImpedanceMode::COMPLIANT_DRAG);

if(!mode_result) {
    return 1;
}

while(running) {
    auto cycle_result = robot.cycle();
    if(!cycle_result) {
        break;
    }
}
```

使用注意

- `COMPLIANT_DRAG` 不接受 set_cmd
- 切换到 tracking 后旧命令会被清除，必须重新提交目标

---

## 103. `Robot::set_model_feedforward_mode()`

签名

```cpp
tl::expected<void, RobotFault>
set_model_feedforward_mode(
    ModelFeedforwardMode mode);
```

只允许在

```text
INACTIVE
```

调用

ACTIVE 时返回

```text
NOT_INACTIVE
```

如果切换到非 NONE 且没有配置 model feedforward callback，返回模型前馈配置错误

### Doxygen 语义展开

修改 Robot 使用的模型前馈策略

参数

- `mode` 为 `NONE`、`GRAVITY` 或 `FULL_INVERSE_DYNAMICS`

返回值

只允许在 `INACTIVE` 调用，成功返回空 `expected`

具体使用示例

```cpp
// Robot 已 configure，但尚未 activate
auto result = robot.set_model_feedforward_mode(
    serial_arm::ModelFeedforwardMode::GRAVITY);

if(!result) {
    std::cerr << "cannot change feedforward mode\n";
    return 1;
}

robot.activate();
```

使用注意

- 非 NONE 模式要求 configure 时已经提供 model feedforward callback

---

## 104. `Robot::cycle()`

签名

```cpp
tl::expected<
    RobotCycleOutput,
    RobotFault
>
cycle(
    TimePoint now = Clock::now());
```

一次完整周期包括

```text
read ActuatorState
    ->
map to JointState
    ->
Safety state check
    ->
command timeout check
    ->
estimate joint acceleration
    ->
JointCtrller update
    ->
estimate reference acceleration
    ->
model feedforward
    ->
Safety command check
    ->
map to ActuatorCtrlCmd
    ->
MotorBus write
```

调用者负责循环频率

Robot 本身不会创建控制线程

### Doxygen 语义展开

执行一帧完整控制闭环，包括读状态、映射、Safety、控制器、模型前馈和硬件写入

参数

- `now` 为本周期时间点，必须单调不回退

返回值

成功返回完整 `RobotCycleOutput`，任一环节失败返回 `RobotFault` 并可能触发 FAULT

具体使用示例

```cpp
const auto period = std::chrono::microseconds(5000);

auto next = serial_arm::Robot::Clock::now();
while(running) {
    next += period;
    const auto now = serial_arm::Robot::Clock::now();

    auto output = robot.cycle(now);
    if(!output) {
        std::cerr << "cycle RobotErr="
                  << static_cast<int>(output.error().code) << "\n";
        break;
    }

    std::cout << "dt=" << output->dt
              << " q0=" << output->joint_state.pos[0]
              << " tau_ff0=" << output->model_feedforward[0]
              << "\n";

    std::this_thread::sleep_until(next);
}
```

使用注意

- Robot 不自己创建控制线程，C++ 调用者必须维护周期

---

## 105. `RobotCycleOutput`

定义

```cpp
struct RobotCycleOutput {
    ActuatorState actuator_state;
    JointState joint_state;
    JointVector joint_acc;
    JointVector joint_ref_acc;
    JointVector model_feedforward;
    JointCtrlCmd joint_cmd;
    ActuatorCtrlCmd actuator_cmd;
    double dt;
};
```

用途

一次获取本周期从状态到最终命令的全部关键数据

非常适合

- 日志
- 数据集采集
- 控制调试
- Safety 诊断
- Backend 对比

### 具体使用示例

```cpp
auto cycle_result = robot.cycle();
if(!cycle_result) {
    return 1;
}

const serial_arm::RobotCycleOutput& out = cycle_result.value();

std::cout << "dt=" << out.dt << "\n";
std::cout << "q0=" << out.joint_state.pos[0] << "\n";
std::cout << "actuator0=" << out.actuator_state.pos[0] << "\n";
std::cout << "joint_acc0=" << out.joint_acc[0] << "\n";
std::cout << "ref_acc0=" << out.joint_ref_acc[0] << "\n";
std::cout << "feedforward0=" << out.model_feedforward[0] << "\n";
std::cout << "safe kp0=" << out.joint_cmd.kp[0] << "\n";
std::cout << "sent actuator kp0=" << out.actuator_cmd.kp[0] << "\n";
```

如果要定位一帧控制为何异常，应优先同时记录 `joint_state`、`joint_cmd`、`actuator_cmd` 和 `model_feedforward`

---

## 106. `Robot::deactivate()`

签名

```cpp
tl::expected<void, RobotFault>
deactivate();
```

正常退出 ACTIVE

成功后状态为 INACTIVE

如果当前已经 INACTIVE，直接返回成功

如果当前 FAULT，使用 `force_deactivate()` 或 fault recovery 接口

### Doxygen 语义展开

正常停止 Robot，调用 Backend deactivate，清空控制器与 Safety 运行时状态并回到 INACTIVE

返回值

成功返回空 `expected`，失败返回 `RobotFault`

具体使用示例

```cpp
if(robot.get_state() == serial_arm::RobotState::ACTIVE) {
    auto result = robot.deactivate();
    if(!result) {
        std::cerr << "deactivate failed\n";
    }
}
```

使用注意

- 当前 Robot::deactivate 本身不会执行 shutdown park trajectory

---

## 107. `Robot::force_deactivate()`

签名

```cpp
tl::expected<void, RobotFault>
force_deactivate();
```

允许从

```text
ACTIVE
FAULT
```

强制退出到 INACTIVE

用于

- 应用异常退出
- fault recovery 不适用
- 需要优先失能硬件

### Doxygen 语义展开

无条件尝试从 ACTIVE 或 FAULT 失能硬件并回到 INACTIVE

返回值

成功返回空 `expected`，真正的 Backend deactivate 失败仍返回 `RobotFault`

具体使用示例

```cpp
if(robot.get_state() == serial_arm::RobotState::FAULT) {
    auto result = robot.force_deactivate();
    if(!result) {
        std::cerr << "hardware could not be disabled\n";
    }
}
```

使用注意

- 这是退出故障现场的强制失能接口，不等同于恢复后继续 ACTIVE

---

## 108. `Robot::reset_fault()`

签名

```cpp
tl::expected<void, RobotFault>
reset_fault();
```

当前实现是

```text
clear_fault()
```

的兼容别名

不要把它理解成无条件清故障

### Doxygen 语义展开

兼容旧代码的故障复位入口，当前实现直接调用 `clear_fault()`

返回值

与 `clear_fault()` 返回相同结果

具体使用示例

```cpp
if(robot.get_state() == serial_arm::RobotState::FAULT) {
    while(robot.is_fault_holding()) {
        robot.maintain_fault_hold();
        if(auto result = robot.reset_fault(); result) {
            break;
        }
    }
}
```

使用注意

- 新代码优先直接使用 `clear_fault()` 表达语义

---

## 109. `Robot::enter_fault_compliant_recovery()`

签名

```cpp
tl::expected<void, RobotFault>
enter_fault_compliant_recovery();
```

仅在 FAULT 且当前 fault hold 有效时使用

还要求配置允许

```text
allow_compliant_recovery
require_operator_request
gravity_model_validated
```

并且当前 fault 类型允许柔性恢复

不满足时返回

```text
FAULT_RECOVERY_NOT_ALLOWED
```

### Doxygen 语义展开

在 Robot 已处于 FAULT 且状态仍可信时，由操作员显式请求低增益柔性恢复

返回值

允许恢复时返回空 `expected`，策略或故障类型不允许时返回 `FAULT_RECOVERY_NOT_ALLOWED`

具体使用示例

```cpp
if(robot.get_state() == serial_arm::RobotState::FAULT &&
   robot.is_fault_holding()) {
    auto result = robot.enter_fault_compliant_recovery();
    if(result) {
        std::cout << "fault compliant recovery active\n";
    }
}
```

使用注意

- 需要配置允许柔性恢复并且重力模型已验证
- 该模式仍属于 FAULT，不代表恢复正常 ACTIVE

---

## 110. `Robot::return_to_fault_rigid_hold()`

签名

```cpp
tl::expected<void, RobotFault>
return_to_fault_rigid_hold();
```

把 FAULT 内部恢复模式重新切回刚性保持

### Doxygen 语义展开

从 FAULT 柔性恢复模式重新回到 FAULT 刚性保持

返回值

成功返回空 `expected`，非 FAULT 状态返回错误

具体使用示例

```cpp
if(robot.get_fault_hold_mode() ==
   serial_arm::FaultHoldMode::COMPLIANT_RECOVERY) {
    robot.return_to_fault_rigid_hold();
}
```

---

## 111. `Robot::clear_fault()`

签名

```cpp
tl::expected<void, RobotFault>
clear_fault();
```

目标

从 FAULT 恢复到

```text
ACTIVE + RIGID_HOLD
```

不是简单清变量

当前实现要求

- fault hold 有效
- Backend 可以读到合法状态
- Safety state check 通过
- 所有关节速度不超过 `shutdown.velocity_tolerance`
- 已积累至少 3 个合法恢复周期

不满足时返回错误

### Doxygen 语义展开

在满足状态合法、低速度和连续有效周期等条件后清除 FAULT，并以当前实测位置进入 ACTIVE + RIGID_HOLD

返回值

成功返回空 `expected`，恢复条件不足返回 `FAULT_RECOVERY_NOT_ALLOWED` 或具体子错误

具体使用示例

```cpp
if(robot.get_state() == serial_arm::RobotState::FAULT) {
    for(int i = 0; i < 10 && robot.is_fault_holding(); ++i) {
        robot.maintain_fault_hold();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    auto result = robot.clear_fault();
    if(result) {
        std::cout << "recovered to ACTIVE + RIGID_HOLD\n";
    }
}
```

使用注意

- clear_fault 不是无条件清错，必须先满足恢复判据

---

## 112. `Robot::maintain_fault_hold()`

签名

```cpp
tl::expected<void, RobotFault>
maintain_fault_hold();
```

FAULT 状态下用于持续刷新 fault hold

如果应用控制线程仍在运行，进入 FAULT 后应进入 fault maintenance 分支，而不是继续调用正常 `cycle()`

### Doxygen 语义展开

Robot 在 FAULT 且 fault hold 有效时持续刷新当前故障保持命令

返回值

成功返回空 `expected`，非 FAULT 或没有可用 fault hold 时返回错误

具体使用示例

```cpp
while(robot.get_state() == serial_arm::RobotState::FAULT &&
      robot.is_fault_holding()) {
    auto result = robot.maintain_fault_hold();
    if(!result) {
        break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
}
```

使用注意

- FAULT 保持需要持续刷新时，应按受控周期调用而不是只调用一次

---

## 113. Robot Getter API

### `get_state()`

```cpp
RobotState
get_state() const noexcept;
```

### `get_impedance_mode()`

```cpp
JointImpedanceMode
get_impedance_mode() const noexcept;
```

### `get_model_feedforward_mode()`

```cpp
ModelFeedforwardMode
get_model_feedforward_mode() const noexcept;
```

### `get_joint_state()`

```cpp
const JointState&
get_joint_state() const noexcept;
```

### `get_joint_acc()`

```cpp
const JointVector&
get_joint_acc() const noexcept;
```

### `get_joint_ref_acc()`

```cpp
const JointVector&
get_joint_ref_acc() const noexcept;
```

### `get_model_feedforward()`

```cpp
const JointVector&
get_model_feedforward() const noexcept;
```

### `get_actuator_state()`

```cpp
const ActuatorState&
get_actuator_state() const noexcept;
```

### `get_last_fault()`

```cpp
const tl::optional<RobotFault>&
get_last_fault() const noexcept;
```

### `is_fault_holding()`

```cpp
bool
is_fault_holding() const noexcept;
```

### `get_fault_hold_mode()`

```cpp
FaultHoldMode
get_fault_hold_mode() const noexcept;
```

这些 Getter 都只读取最近一次缓存

不会主动访问硬件

### Doxygen 语义展开

Robot Getter 用于读取最近一次合法缓存和生命周期信息，不触发额外硬件读取

返回值

返回值都对应 Robot 当前缓存状态，其中状态和向量 Getter 多数返回只读引用

具体使用示例

```cpp
std::cout << "robot state="
          << static_cast<int>(robot.get_state()) << "\n";
std::cout << "impedance mode="
          << static_cast<int>(robot.get_impedance_mode()) << "\n";
std::cout << "feedforward mode="
          << static_cast<int>(robot.get_model_feedforward_mode()) << "\n";

const auto& joint = robot.get_joint_state();
const auto& actuator = robot.get_actuator_state();
const auto& acc = robot.get_joint_acc();
const auto& ref_acc = robot.get_joint_ref_acc();
const auto& ff = robot.get_model_feedforward();

if(const auto& last_fault = robot.get_last_fault(); last_fault) {
    std::cerr << "last RobotErr="
              << static_cast<int>(last_fault->code) << "\n";
}

if(robot.is_fault_holding()) {
    std::cout << "fault hold mode="
              << static_cast<int>(robot.get_fault_hold_mode()) << "\n";
}
```

使用注意

- 这些 Getter 不会主动调用 `MotorBus::read()`，需要新数据时仍应先成功执行 cycle 或对应 fault reaction

---

# Part XI ModelFeedforwardFn

## 114. `ModelFeedforwardFn`

定义

```cpp
using ModelFeedforwardFn =
    std::function<
        tl::expected<
            JointVector,
            ModelFeedforwardErr
        >(
            ModelFeedforwardMode,
            const JointState&,
            const JointVector&,
            const JointVector&,
            double
        )
    >;
```

参数依次为

```text
mode
current JointState
estimated joint acceleration
reference joint acceleration
dt
```

一个标准 Dynamics 实现

```cpp
ModelFeedforwardFn feedforward =
    [&dynamics, joints_count](
        ModelFeedforwardMode mode,
        const JointState& state,
        const JointVector& acc,
        const JointVector& ref_acc,
        double)
        -> tl::expected<
            JointVector,
            ModelFeedforwardErr>
    {
        if(mode ==
            ModelFeedforwardMode::NONE)
        {
            return JointVector(
                joints_count,
                0.0);
        }

        auto result =
            dynamics.update(
                state,
                acc,
                ref_acc);

        if(!result) {
            return tl::make_unexpected(
                ModelFeedforwardErr::COMPUTE_FAILED);
        }

        if(mode ==
            ModelFeedforwardMode::GRAVITY)
        {
            return
                dynamics.get_gravity_compensation();
        }

        if(mode ==
            ModelFeedforwardMode::FULL_INVERSE_DYNAMICS)
        {
            return
                dynamics.get_inverse_dynamics();
        }

        return tl::make_unexpected(
            ModelFeedforwardErr::INVALID_MODE);
    };
```

### Doxygen 语义展开

把外部 Dynamics 或其他模型计算器接入 Robot，使 Robot 根据当前前馈模式请求一帧 Joint 侧模型力矩

参数

- `mode` 为当前前馈模式
- `state` 为当前 Joint 状态
- `joint_acc` 为估计加速度
- `joint_ref_acc` 为参考加速度
- `dt` 为当前控制周期

返回值

成功返回与 Joint 数量一致的前馈力矩，失败返回 `ModelFeedforwardErr`

具体使用示例

```cpp
serial_arm::Dynamics dynamics;
dynamics.configure(cfg.dynamics);

serial_arm::ModelFeedforwardFn model_feedforward =
    [&dynamics](
        serial_arm::ModelFeedforwardMode mode,
        const serial_arm::JointState& state,
        const serial_arm::JointVector& joint_acc,
        const serial_arm::JointVector& joint_ref_acc,
        double /*dt*/)
        -> tl::expected<serial_arm::JointVector, serial_arm::ModelFeedforwardErr>
    {
        if(mode == serial_arm::ModelFeedforwardMode::NONE) {
            return serial_arm::JointVector(state.pos.size(), 0.0);
        }

        auto updated = dynamics.update(state, joint_acc, joint_ref_acc);
        if(!updated) {
            return tl::make_unexpected(
                serial_arm::ModelFeedforwardErr::COMPUTE_FAILED);
        }

        if(mode == serial_arm::ModelFeedforwardMode::GRAVITY) {
            return dynamics.get_gravity_compensation();
        }

        if(mode == serial_arm::ModelFeedforwardMode::FULL_INVERSE_DYNAMICS) {
            return dynamics.get_inverse_dynamics();
        }

        return tl::make_unexpected(
            serial_arm::ModelFeedforwardErr::INVALID_MODE);
    };
```

使用注意

- 回调捕获的 Dynamics 生命周期必须覆盖 Robot 使用回调的整个时间

---

# Part XII Python API

## 115. Python 错误模型

Python Binding 把 C++ `tl::expected` 错误转换为

```python
serial_arm.SerialArmError
```

因此 Python 推荐使用

```python
try:
    ...
except serial_arm.SerialArmError as exc:
    print(exc)
```

NumPy 输入维度或 NaN/Inf 等 Python 侧参数错误可能抛出 `ValueError`

### 具体使用示例

```python
import serial_arm

try:
    cfg = serial_arm.load_robot_cfg(core_yaml, plugin, hardware_yaml)
except serial_arm.SerialArmError as exc:
    print("SerialArm core error:", exc)

try:
    bad = np.array([[0.0, 1.0]], dtype=np.float64)
    dynamics.set_gravity_scale(bad)
except ValueError as exc:
    print("Python input error:", exc)
```

`SerialArmError` 表示 C++ expected 错误已经被绑定层转换，`ValueError` 更常表示 Python/NumPy 参数形状或数值非法

---

## 116. Python `load_robot_cfg()`

签名语义

```python
serial_arm.load_robot_cfg(
    path,
    hardware_plugin,
    hardware_config,
)
```

Python wrapper 内部会

```text
HardwareLoader::load
    ->
MotorBus::capabilities
    ->
C++ load_robot_cfg
```

示例

```python
cfg = serial_arm.load_robot_cfg(
    "config/core/gray.yaml",
    "serial_arm_hardware_damiao",
    "config/hardware.yaml",
)
```

返回

```python
serial_arm.RobotCfg
```

### Doxygen 语义展开

Python 版本先通过 HardwareLoader 获取真实 Backend capability，再调用 C++ `load_robot_cfg()`

参数

- `path` 为 Core YAML
- `hardware_plugin` 为 Backend 插件
- `hardware_config` 为 Backend YAML

返回值

成功返回 `RobotCfg`，C++ expected 错误会转换为 `SerialArmError`

具体使用示例

```python
import serial_arm

try:
    cfg = serial_arm.load_robot_cfg(
        "/opt/serial_arm/share/dm_arm_description/config/core/gray.yaml",
        "serial_arm_hardware_damiao",
        "/opt/serial_arm/share/dm_arm_description/config/hardware.yaml",
    )
except serial_arm.SerialArmError as exc:
    raise SystemExit(f"config load failed: {exc}")

print(cfg.joint_names)
print(cfg.runtime.ctrl_frequency_hz)
```

Python Binding 还直接暴露两种配置验证接口

```python
serial_arm.validate_robot_core_cfg(cfg)
serial_arm.validate_robot_cfg(cfg)
```

典型用法

```python
cfg.runtime.ctrl_frequency_hz = 250.0

try:
    serial_arm.validate_robot_core_cfg(cfg)
except serial_arm.SerialArmError as exc:
    print("invalid core config:", exc)
```

---

## 117. Python `load_robot_profile_core()`

使用

```python
profile = serial_arm.load_robot_profile_core(
    "dm_arm_gray"
)
```

显式文件

```python
profile = serial_arm.load_robot_profile_core(
    "dm_arm_gray",
    "/path/to/robot_profiles.yaml",
)
```

返回 `RobotProfileCore`

可读取

```python
profile.name
profile.profile_file
profile.core_config_path
profile.hardware_plugin
profile.hardware_config_path
```

### Doxygen 语义展开

Python 版本解析 framework-neutral profile，直接返回 Python 可读写的 `RobotProfileCore`

参数

- `profile_name` 为 profile 名称
- `profile_file` 可选显式路径

返回值

成功返回 `RobotProfileCore`，失败抛 `SerialArmError`

具体使用示例

```python
import serial_arm

profile = serial_arm.load_robot_profile_core("dm_arm_gray")

cfg = serial_arm.load_robot_cfg(
    profile.core_config_path,
    profile.hardware_plugin,
    profile.hardware_config_path,
)

print(profile.name)
print(profile.core_config_path)
```

---

## 118. Python `Dynamics`

创建

```python
dynamics = serial_arm.Dynamics()
```

配置

```python
dynamics.configure(cfg.dynamics)
```

### `update()`

Python 版 `update()` 接受五个 NumPy 数组

```python
dynamics.update(
    pos,
    vel,
    acc,
    tor,
    ref_acc,
)
```

示例

```python
n = len(cfg.joint_names)

q = np.zeros(n, dtype=np.float64)
dq = np.zeros(n, dtype=np.float64)
ddq = np.zeros(n, dtype=np.float64)
tau = np.zeros(n, dtype=np.float64)
ddq_ref = np.zeros(n, dtype=np.float64)

dynamics.update(
    q,
    dq,
    ddq,
    tau,
    ddq_ref,
)
```

### `update_state()`

如果已经有 `JointState`

```python
dynamics.update_state(
    state,
    acc,
    ref_acc,
)
```

### `set_gravity_scale()`

```python
dynamics.set_gravity_scale(
    np.array(
        [1.0, 0.9, 0.8, 1.0, 1.0, 1.0],
        dtype=np.float64,
    )
)
```

### `frame_pose()`

```python
T = dynamics.frame_pose("tool0")
```

返回 `4 x 4` NumPy 数组

### `frame_jacobian()`

```python
J = dynamics.frame_jacobian("tool0")
```

返回 `6 x N` NumPy 数组

### Dynamics 只读属性

```python
dynamics.configured
dynamics.updated
dynamics.info
dynamics.state
dynamics.gravity_scale
dynamics.gravity
dynamics.gravity_compensation
dynamics.nonlinear
dynamics.coriolis
dynamics.mass_matrix
dynamics.inverse_dynamics
dynamics.forward_dynamics
dynamics.tool_pose
dynamics.tool_jacobian
```

### Doxygen 语义展开

Python `Dynamics` 是 C++ Dynamics 的直接计算接口，适合离线验证模型、重力、Jacobian 和逆动力学

参数

- 所有 Joint 向量输入使用一维 `numpy.float64` 数组
- 数组长度必须等于 `dynamics.info.joints_count`

返回值

C++ DynamicsErr 转换为 `SerialArmError`，NumPy 维度或 NaN/Inf 错误可能抛 `ValueError`

具体使用示例

```python
import numpy as np
import serial_arm

cfg = serial_arm.load_robot_cfg(core_yaml, plugin, hardware_yaml)
dynamics = serial_arm.Dynamics()
dynamics.configure(cfg.dynamics)

n = len(cfg.joint_names)
q = np.zeros(n, dtype=np.float64)
dq = np.zeros(n, dtype=np.float64)
ddq = np.zeros(n, dtype=np.float64)
tau = np.zeros(n, dtype=np.float64)
ddq_ref = np.zeros(n, dtype=np.float64)

dynamics.update(q, dq, ddq, tau, ddq_ref)

print("gravity", dynamics.gravity)
print("tool pose\n", dynamics.tool_pose)
print("tool jacobian shape", dynamics.tool_jacobian.shape)
```

---

## 119. Python `JointCtrller`

主要用于离线控制器测试

创建

```python
ctrller = serial_arm.JointCtrller()
```

公共方法

```python
ctrller.configure(cfg)
ctrller.initialize(state)
ctrller.reset()

ctrller.set_impedance_mode(
    mode,
    state,
)

ctrller.set_pos_cmd(pos)

ctrller.set_pos_vel_cmd(
    pos,
    vel,
)

ctrller.set_pos_vel_tor_cmd(
    pos,
    vel,
    tor,
)

ctrller.set_full_cmd(cmd)

output_cmd = ctrller.update(
    state,
    model_feedforward,
    dt,
)
```

只读属性

```python
ctrller.state
ctrller.impedance_mode
```

### Doxygen 语义展开

Python `JointCtrller` 暴露 C++ 控制器用于离线单元测试，不负责 hardware、mapping 或 Safety

参数

- `configure()` 接收 `JointCtrllerCfg`
- `initialize()` 接收 `JointState`
- `update()` 接收当前状态、模型前馈数组和 dt

返回值

失败统一抛 `SerialArmError`

具体使用示例

```python
import numpy as np
import serial_arm

ctrller = serial_arm.JointCtrller()
ctrller.configure(cfg.ctrller)

state = serial_arm.JointState()
state.pos = np.zeros(6, dtype=np.float64)
state.vel = np.zeros(6, dtype=np.float64)
state.tor = np.zeros(6, dtype=np.float64)
ctrller.initialize(state)

ctrller.set_impedance_mode(
    serial_arm.JointImpedanceMode.RIGID_TRACKING,
    state,
)

target = state.pos.copy()
target[0] += 0.05
ctrller.set_pos_cmd(target)

joint_cmd = ctrller.update(
    state,
    np.zeros(6, dtype=np.float64),
    0.005,
)

print(joint_cmd.pos)
print(joint_cmd.kp)
```

---

## 120. Python `JointActuatorMapper`

```python
mapper = serial_arm.JointActuatorMapper()

mapper.configure(cfg.mapper)

joint_state =
    mapper.to_joint_state(
        actuator_state
    )

actuator_cmd =
    mapper.to_actuator_cmd(
        joint_cmd
    )

print(mapper.size)
```

### Doxygen 语义展开

Python Mapper 用于离线验证方向、零位、减速比和力矩比例

参数

- `configure()` 接收 `JointActuatorMapCfg`
- `to_joint_state()` 接收 `ActuatorState`
- `to_actuator_cmd()` 接收 `JointCtrlCmd`

返回值

失败抛 `SerialArmError`

具体使用示例

```python
import numpy as np
import serial_arm

mapper = serial_arm.JointActuatorMapper()
mapper.configure(cfg.mapper)

act = serial_arm.ActuatorState()
act.pos = np.zeros(6, dtype=np.float64)
act.vel = np.zeros(6, dtype=np.float64)
act.tor = np.zeros(6, dtype=np.float64)
act.online = np.ones(6, dtype=np.uint8)
act.enabled = np.ones(6, dtype=np.uint8)
act.err_code = np.zeros(6, dtype=np.int32)

joint = mapper.to_joint_state(act)
print(joint.pos)
print("mapper size", mapper.size)
```

---

## 121. Python `Safety`

```python
safety = serial_arm.Safety()

safety.configure(cfg.safety)

safety.check_state(
    joint_state,
    actuator_state,
    state_age_s,
)

safe_cmd =
    safety.check_joint_cmd(
        joint_state,
        joint_cmd,
        dt,
    )
```

其他 API

```python
safety.check_cmd_age(cmd_age_s)
safety.reset_cmd_history(joint_state)
safety.clear_cmd_history()
safety.action_for(error)
safety.configured
safety.clamp_count
```

### Doxygen 语义展开

Python Safety 用于离线验证状态、命令和超时策略，返回的安全命令可能已经发生轻微 clamp

参数

- `check_state()` 需要 JointState、ActuatorState 和 state age
- `check_joint_cmd()` 需要当前 JointState、完整 JointCtrlCmd 和 dt

返回值

失败抛 `SerialArmError`

具体使用示例

```python
import serial_arm

safety = serial_arm.Safety()
safety.configure(cfg.safety)
safety.reset_cmd_history(joint_state)

try:
    safety.check_state(joint_state, actuator_state, 0.005)
    safe_cmd = safety.check_joint_cmd(joint_state, joint_cmd, 0.005)
except serial_arm.SerialArmError as exc:
    print("rejected by Safety:", exc)
else:
    print("safe target", safe_cmd.pos)
    print("clamp count", safety.clamp_count)
```

---

## 122. Python `RobotSession`

创建

```python
session = serial_arm.RobotSession(
    config_file,
    hardware_plugin,
    hardware_config,
)
```

它与 C++ `Robot` 的主要差异

```text
C++ Robot
  调用者自己维护 cycle 频率
  write_enabled=false 时 activate 返回 WRITE_DISABLED

Python RobotSession
  C++ 工作线程维护控制周期
  Python 只提交高层请求并读取 snapshot
  write_enabled=false 时使用内部 MockMotorBus 运行离线闭环
```

RobotSession 配置阶段仍会加载真实 Hardware Backend 来读取 `HardwareCapabilities`

只有实际控制 Bus 在 `write_enabled=false` 时被替换为 mock

### Doxygen 语义展开

Python `RobotSession` 把 Robot、Dynamics、HardwareLoader 和固定频率 C++ 工作线程组合成高层会话

参数

- 构造参数分别是 Core config、Hardware plugin、Hardware config

返回值

构造只完成配置不激活，推荐通过上下文管理器保证 stop

具体使用示例

```python
import serial_arm

session = serial_arm.RobotSession(
    core_yaml,
    "serial_arm_hardware_damiao",
    hardware_yaml,
)

session.set_model_feedforward_mode(
    serial_arm.ModelFeedforwardMode.GRAVITY
)

with session:
    print("running", session.running)
    print("state", session.state)
    print("joints", session.config.joint_names)
```

---

## 123. `RobotSession.start()`

```python
session.start()
```

作用

- 激活底层会话
- 启动 C++ 控制线程

真机前确认 `write_enabled`

### Doxygen 语义展开

激活底层 Robot 并启动 C++ 固定频率控制线程

返回值

成功无返回值，失败抛 `SerialArmError`

具体使用示例

```python
session = serial_arm.RobotSession(core_yaml, plugin, hardware_yaml)

try:
    session.start()
    print(session.running)
finally:
    session.stop()
```

---

## 124. `RobotSession.stop()`

```python
session.stop()
```

停止工作线程并安全退出

推荐始终在 `finally` 中调用，或者使用上下文管理器

### Doxygen 语义展开

请求控制线程退出并在 Robot 仍 ACTIVE 时安全 deactivate

返回值

成功无返回值，deactivate 失败抛 `SerialArmError`

具体使用示例

```python
session.start()
try:
    run_experiment(session)
finally:
    session.stop()

assert not session.running
```

---

## 125. `RobotSession.set_impedance_mode()`

```python
session.set_impedance_mode(
    serial_arm.JointImpedanceMode.COMPLIANT_DRAG
)
```

请求由 C++ 工作线程串行应用

### Doxygen 语义展开

向 C++ 工作线程提交阻抗模式切换请求，由工作线程在控制序列中串行应用

参数

- `mode` 为 `JointImpedanceMode`

返回值

会话未运行时抛 `SerialArmError`

具体使用示例

```python
session.start()

session.set_impedance_mode(
    serial_arm.JointImpedanceMode.COMPLIANT_DRAG
)

# C++ worker 继续维持控制周期
for _ in range(10):
    snap = session.snapshot
    if snap.valid:
        print(snap.cycle.joint_state.pos)
```

---

## 126. `RobotSession.set_model_feedforward_mode()`

```python
session.set_model_feedforward_mode(
    serial_arm.ModelFeedforwardMode.GRAVITY
)
```

底层 Robot 只允许在 INACTIVE 修改 model feedforward mode

因此建议在 `start()` 前设置

### Doxygen 语义展开

修改底层 Robot 的模型前馈模式

参数

- `mode` 为 `ModelFeedforwardMode`

返回值

只允许配置完成且会话处于 INACTIVE 时调用

具体使用示例

```python
session = serial_arm.RobotSession(core_yaml, plugin, hardware_yaml)

session.set_model_feedforward_mode(
    serial_arm.ModelFeedforwardMode.GRAVITY
)

session.start()
```

使用注意

- 不要在 `session.running == True` 时调用

---

## 127. `RobotSession.set_gravity_scale()`

```python
session.set_gravity_scale(
    np.array(
        [1.0, 0.9, 0.85, 1.0, 1.0, 1.0],
        dtype=np.float64,
    )
)
```

运行期间由 C++ 工作线程应用

### Doxygen 语义展开

设置各 Joint 重力补偿比例，运行时请求会由 C++ worker 串行应用

参数

- `gravity_scale` 为一维 float64 数组，每项范围 [0, 1]

返回值

成功无返回值，维度或范围非法时抛异常

具体使用示例

```python
scale = np.asarray(
    session.config.dynamics.gravity_scale,
    dtype=np.float64,
).copy()
scale[1] = 0.90
scale[2] = 0.85

session.set_gravity_scale(scale)
print("requested gravity scale", scale)
```

---

## 128. `RobotSession.move_to()`

签名

```python
session.move_to(
    pos,
    speed_scale=0.3,
)
```

要求

- `pos` 为一维 float64 NumPy 数组
- 长度等于受控 Joint 数
- `speed_scale` 由 Session 轨迹逻辑使用

示例

```python
target = session.snapshot.cycle.joint_state.pos.copy()

target[0] += 0.05

session.move_to(
    target,
    speed_scale=0.15,
)
```

### Doxygen 语义展开

提交一个绝对 Joint 位置目标，由 C++ worker 生成连续梯形位置速度 reference

参数

- `pos` 为目标 Joint 位置
- `speed_scale` 有效范围 (0, 1]

返回值

要求会话运行且当前请求模式为 RIGID_TRACKING 或 COMPLIANT_TRACKING

具体使用示例

```python
session.start()
session.set_impedance_mode(
    serial_arm.JointImpedanceMode.RIGID_TRACKING
)

target = session.snapshot.cycle.joint_state.pos.copy()
target[0] += 0.05

session.move_to(target, speed_scale=0.15)

while True:
    snap = session.snapshot
    if snap.valid:
        error = abs(snap.cycle.joint_state.pos[0] - target[0])
        if error < 0.01:
            break
```

使用注意

- 目标会在 Session 侧先检查 Safety soft position limit

---

## 129. `RobotSession.hold_current()`

```python
session.hold_current()
```

取消当前位置目标并请求当前位置刚性保持

### Doxygen 语义展开

取消当前 move_to 目标并请求切换到当前位置 `RIGID_HOLD`

返回值

要求会话正在运行

具体使用示例

```python
session.hold_current()

snap = session.snapshot
print("hold requested at", snap.cycle.joint_state.pos)
```

---

## 130. RobotSession fault API

### `reset_fault()`

```python
session.reset_fault()
```

兼容接口，底层等价于 clear fault

### `clear_fault()`

```python
session.clear_fault()
```

尝试满足条件后恢复 ACTIVE + RIGID_HOLD

### `enter_fault_compliant_recovery()`

```python
session.enter_fault_compliant_recovery()
```

人工请求受限柔性恢复

### `return_to_fault_rigid_hold()`

```python
session.return_to_fault_rigid_hold()
```

返回 FAULT 刚性保持

### Doxygen 语义展开

RobotSession 暴露与 C++ Robot 对应的 FAULT 恢复操作，并在需要时停止 worker 后执行

返回值

失败抛 `SerialArmError`

具体使用示例

```python
if session.state == serial_arm.RobotState.FAULT:
    print("fault mode", session.fault_hold_mode)

    try:
        session.enter_fault_compliant_recovery()
        # 操作员完成低风险处理后
        session.return_to_fault_rigid_hold()
        session.clear_fault()
    except serial_arm.SerialArmError as exc:
        print("fault recovery rejected:", exc)
```

使用注意

- `reset_fault()` 只是 `clear_fault()` 的兼容别名

---

## 131. RobotSession 属性

### `snapshot`

```python
snapshot = session.snapshot
```

`RobotSessionSnapshot` 包含

```python
snapshot.robot_state
snapshot.cycle
snapshot.dynamics
snapshot.valid
snapshot.last_error
```

### `state`

```python
session.state
```

返回 `RobotState`

### `fault_hold_mode`

```python
session.fault_hold_mode
```

### `configured`

```python
session.configured
```

### `running`

```python
session.running
```

### `config`

```python
session.config
```

### `dynamics_info`

```python
session.dynamics_info
```

### `actuator_info`

```python
session.actuator_info
```

### Doxygen 语义展开

这些属性返回线程安全复制出的会话状态、配置和最近一次快照，用于上层监控和实验脚本

返回值

属性读取不会改变控制状态

具体使用示例

```python
snap = session.snapshot

if snap.valid:
    print("robot state", snap.robot_state)
    print("q", snap.cycle.joint_state.pos)
    print("tau ff", snap.cycle.model_feedforward)
    print("tool pose", snap.dynamics.tool_pose)

print("configured", session.configured)
print("running", session.running)
print("fault hold mode", session.fault_hold_mode)
print("joint names", session.config.joint_names)
print("model mass", session.dynamics_info.total_mass)

for actuator in session.actuator_info:
    print(actuator.joint_name, actuator.name, actuator.max_effort)
```

---


# Part XIII ros2_control Adapter API

这一部分直接对应 `src/serial_arm/bringup/ros2_control/include/serial_arm_ros2_control/serial_arm_system.hpp` 的公开 Doxygen 接口

`SerialArmSystem` 是 ros2_control 的 `SystemInterface` 插件，正常用户不会在业务代码中手工调用其生命周期函数，而是由 `controller_manager` 按生命周期调用

## R0. `SerialArmSystem` 生命周期对象

Doxygen 语义

`SerialArmSystem` 析构时会停止后台线程并尽力释放真机硬件

正常使用方式不是手工 `new SerialArmSystem`，而是让 pluginlib 和 controller_manager 管理对象生命周期

```xml
<plugin>serial_arm_ros2_control/SerialArmSystem</plugin>
```

因此退出 controller_manager、卸载 hardware component 或对象析构时，都必须保证后台 worker 不再访问已经释放的 Robot、Dynamics 或 Backend

---

## R1. `SerialArmSystem::on_init()`

Doxygen 语义

初始化 ros2_control 硬件信息并校验 Core 配置一致性

参数

- `info` 为 ros2_control 从 URDF `<ros2_control>` 解析得到的 `HardwareInfo`

返回值

- 初始化成功返回 `hardware_interface::CallbackReturn::SUCCESS`
- 参数缺失、Joint 或接口不一致时返回 `ERROR`

框架调用场景

```text
controller_manager 加载 hardware plugin
    ->
SerialArmSystem::on_init(info)
```

具体配置示例

```xml
<ros2_control name="SerialArmSystem" type="system">
  <hardware>
    <plugin>serial_arm_ros2_control/SerialArmSystem</plugin>
    <param name="config_file">/path/to/core.yaml</param>
    <param name="hardware_plugin">serial_arm_hardware_damiao</param>
    <param name="hardware_config">/path/to/hardware.yaml</param>
  </hardware>

  <joint name="joint1">
    <command_interface name="position"/>
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
```

`on_init()` 应验证 ros2_control joint 名称和接口是否与 Core YAML 的 joint 顺序一致

---

## R2. `SerialArmSystem::on_configure()`

Doxygen 语义

配置 Dynamics、Hardware Backend 与 Robot，但不连接或使能真机

参数

- `previous_state` 为生命周期切换前状态

返回值

- 配置成功返回 `SUCCESS`
- Backend、配置、URDF、Dynamics 或 Robot 配置失败返回 `ERROR`

框架调用场景

```bash
ros2 control set_hardware_component_state dm_arm inactive
```

其内部目标状态可理解为

```text
Robot Profile resolved
    ->
HardwareLoader::load
    ->
load_robot_cfg
    ->
Dynamics::configure
    ->
Robot::configure
    ->
RobotState::INACTIVE
```

如果这里失败，应优先检查 profile、Core YAML、Backend YAML、URDF 和 joint/interface 一致性，而不是检查运动控制器参数

---

## R3. `SerialArmSystem::on_activate()`

Doxygen 语义

显式授权后连接真机、初始化命令缓存并启动控制线程

参数

- `previous_state` 为生命周期切换前状态

返回值

- 激活成功返回 `SUCCESS`
- Robot activate 或 worker 启动失败返回 `ERROR`

框架使用示例

```bash
ros2 control set_hardware_component_state dm_arm active
```

典型内部流程

```text
Robot::activate
    ->
读取当前 Joint 状态
    ->
初始化 command cache 为当前位置
    ->
启动 worker thread
```

真机第一次 activate 前应保持目标机械臂处于安全姿态并完成方向、零位和限位检查

---

## R4. `SerialArmSystem::on_deactivate()`

Doxygen 语义

停止后台线程并请求刚性保持后失能真机

参数

- `previous_state` 为生命周期切换前状态

返回值

- 正常失能返回 `SUCCESS`
- park 或 Robot deactivate 失败时按实现返回 `ERROR`

框架使用示例

```bash
ros2 control set_hardware_component_state dm_arm inactive
```

如果配置启用了 shutdown park，上层 Adapter 可以在真正失能前执行停放轨迹

---

## R5. `SerialArmSystem::export_state_interfaces()`

Doxygen 语义

向 ros2_control 导出每个关节的 position、velocity、effort 状态接口

返回值

- 返回 `std::vector<hardware_interface::StateInterface>`

控制器侧可通过标准 ros2_control 状态接口读取

```text
joint1/position
joint1/velocity
joint1/effort
joint2/position
joint2/velocity
joint2/effort
...
```

这些值来自后台 worker 写入的最近一次合法 `StateFrame`，其 `pos / vel / effort` 最终来源于 `RobotCycleOutput`

---

## R6. `SerialArmSystem::export_command_interfaces()`

Doxygen 语义

向 ros2_control 导出每个关节的 position、velocity 命令接口

返回值

- 返回 `std::vector<hardware_interface::CommandInterface>`

例如 JointTrajectoryController 最终写入

```text
joint1/position
joint1/velocity
joint2/position
joint2/velocity
...
```

当前 Adapter 不向 ros2_control 上层直接暴露 effort、kp、kd 命令接口，因此完整 MIT 自定义控制仍应走 Core API 或后续专用 controller

---

## R7. `SerialArmSystem::read()`

Doxygen 语义

将后台控制线程最近一次合法状态复制到 ros2_control state interface

参数

- `time` 为 controller_manager 当前时间
- `period` 为 controller_manager 本次 update 周期

返回值

- 成功复制最近合法缓存返回 `hardware_interface::return_type::OK`

框架会周期调用

```text
controller_manager update
    ->
SerialArmSystem::read
    ->
controller update
    ->
SerialArmSystem::write
```

状态消费示例

```bash
ros2 topic echo /joint_states
```

`read()` 本身不应在 controller_manager 线程里重新执行完整 Hardware Backend 读写闭环，真实闭环由后台 worker 维护

---

## R8. `SerialArmSystem::write()`

Doxygen 语义

将 ros2_control command interface 的 position、velocity 复制到后台线程命令缓存

参数

- `time` 为 controller_manager 当前时间
- `period` 为 controller_manager 本次 update 周期

返回值

- 正常写入缓存返回 `OK`
- 后台错误已经锁存时返回 `ERROR`

最常见上层来源是 JointTrajectoryController

```bash
ros2 action send_goal \
  /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint1, joint2], points: [{positions: [0.1, 0.2], time_from_start: {sec: 2}}]}}"
```

`write()` 的职责是复制上层 reference，不应在 controller_manager 实时线程中直接执行耗时的串口/CAN 事务

---

## R9. `CommandFrame`

Doxygen 语义

ros2_control Adapter 内部使用的命令帧结构体

典型语义

```cpp
serial_arm_ros2_control::CommandFrame frame;
frame.pos = command_position;
frame.vel = command_velocity;
frame.sequence = ++command_sequence;
```

它用于把 controller_manager 线程产生的 command 安全交给后台 worker

---

## R10. `StateFrame`

Doxygen 语义

ros2_control Adapter 内部使用的状态帧结构体

典型消费方式

```cpp
serial_arm_ros2_control::StateFrame frame = latest_state_frame;

if(frame.valid) {
    joint_position = frame.pos;
    joint_velocity = frame.vel;
    joint_effort = frame.effort;
    model_feedforward = frame.model_feedforward;
}
```

它用于把后台 worker 的最近合法状态复制回 ros2_control `read()` 路径

---

# Part XIV 错误处理

## 132. C++ `tl::expected`

SerialArm-Core 不依赖异常完成 C++ 正常错误传播

典型形式

```cpp
auto result = robot.activate();

if(!result) {
    const RobotFault& fault =
        result.error();

    std::cerr
        << "RobotErr="
        << static_cast<int>(fault.code)
        << "\n";

    return 1;
}
```

不要直接写

```cpp
robot.activate().value();
```

除非当前程序明确允许错误直接终止

### 具体使用示例

```cpp
auto result = robot.activate();

if(!result) {
    const serial_arm::RobotFault& fault = result.error();
    std::cerr << "RobotErr=" << static_cast<int>(fault.code) << "\n";
    return 1;
}

// 只有 result 为 true 时才读取 value
```

不要在没有先判断 `result` 的情况下直接调用 `value()`

---

## 133. `RobotFault`

定义

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

先读

```cpp
fault.code
```

再根据顶层错误决定查看哪个子字段

例如

```text
MOTOR_BUS_READ_FAILED
    ->
motor_bus_err

MAPPER_FAILED
    ->
mapper_err

CTRLLER_FAILED
    ->
ctrller_err

SAFETY_FAILED
    ->
safety_fault

MODEL_FEEDFORWARD_FAILED
    ->
model_feedforward_err
```

### 具体使用示例

```cpp
auto cycle_result = robot.cycle();
if(!cycle_result) {
    const serial_arm::RobotFault& fault = cycle_result.error();

    switch(fault.code) {
    case serial_arm::RobotErr::MOTOR_BUS_READ_FAILED:
    case serial_arm::RobotErr::MOTOR_BUS_WRITE_FAILED:
        std::cerr << "MotorBusErr="
                  << static_cast<int>(fault.motor_bus_err) << "\n";
        break;

    case serial_arm::RobotErr::SAFETY_FAILED:
        std::cerr << "SafetyErr="
                  << static_cast<int>(fault.safety_fault.code)
                  << " joint=" << fault.safety_fault.index
                  << " value=" << fault.safety_fault.value
                  << " limit=" << fault.safety_fault.limit
                  << "\n";
        break;

    default:
        std::cerr << "RobotErr=" << static_cast<int>(fault.code) << "\n";
        break;
    }
}
```

先看顶层 `code`，再读取对应的子错误字段

---

# Part XV 功能参考示例

## 134. 功能 A：通过 Robot Profile 创建一个 Robot

```cpp
#include <iostream>
#include <memory>

#include "serial_arm/config/config.hpp"
#include "serial_arm/config/robot_profile.hpp"
#include "serial_arm/dynamics/dynamics.hpp"
#include "serial_arm/hardware/hardware_loader.hpp"
#include "serial_arm/robot.hpp"

using namespace serial_arm;

int main() {
    auto profile_result =
        load_robot_profile_core(
            "dm_arm_gray");

    if(!profile_result) {
        std::cerr
            << profile_result.error().message
            << "\n";
        return 1;
    }

    RobotProfileCore profile =
        profile_result.value();

    HardwareLoader loader;

    auto bus_result = loader.load(
        profile.hardware_plugin,
        profile.hardware_config_path);

    if(!bus_result) {
        return 1;
    }

    std::unique_ptr<MotorBus> bus =
        std::move(bus_result.value());

    auto cfg_result = load_robot_cfg(
        profile.core_config_path,
        bus->capabilities());

    if(!cfg_result) {
        std::cerr
            << cfg_result.error().message
            << "\n";
        return 1;
    }

    RobotCfg cfg =
        cfg_result.value();

    Dynamics dynamics;

    if(auto result =
        dynamics.configure(cfg.dynamics);
        !result)
    {
        return 1;
    }

    const std::size_t n =
        cfg.joint_names.size();

    ModelFeedforwardFn model_ff =
        [&dynamics, n](
            ModelFeedforwardMode mode,
            const JointState& state,
            const JointVector& acc,
            const JointVector& ref_acc,
            double)
            -> tl::expected<
                JointVector,
                ModelFeedforwardErr>
        {
            if(mode ==
                ModelFeedforwardMode::NONE)
            {
                return JointVector(
                    n,
                    0.0);
            }

            if(auto result =
                dynamics.update(
                    state,
                    acc,
                    ref_acc);
                !result)
            {
                return tl::make_unexpected(
                    ModelFeedforwardErr::COMPUTE_FAILED);
            }

            if(mode ==
                ModelFeedforwardMode::GRAVITY)
            {
                return
                    dynamics.get_gravity_compensation();
            }

            if(mode ==
                ModelFeedforwardMode::FULL_INVERSE_DYNAMICS)
            {
                return
                    dynamics.get_inverse_dynamics();
            }

            return tl::make_unexpected(
                ModelFeedforwardErr::INVALID_MODE);
        };

    Robot robot;

    auto robot_result =
        robot.configure(
            cfg,
            std::move(bus),
            model_ff);

    if(!robot_result) {
        return 1;
    }

    std::cout
        << "Robot configured\n";

    return 0;
}
```

---

## 135. 功能 B：实现刚性位置跟踪

前提

```text
Robot ACTIVE
```

代码

```cpp
auto mode_result =
    robot.set_impedance_mode(
        JointImpedanceMode::RIGID_TRACKING);

if(!mode_result) {
    return;
}

JointVector target =
    robot.get_joint_state().pos;

target[0] += 0.05;

while(running) {
    const auto now =
        Robot::Clock::now();

    auto cmd_result =
        robot.set_cmd(
            JointPosCmd{target},
            now);

    if(!cmd_result) {
        break;
    }

    auto cycle_result =
        robot.cycle(now);

    if(!cycle_result) {
        break;
    }
}
```

重点

- tracking 前先切 mode
- mode 切换会清除旧命令
- tracking 期间持续刷新 command
- target 长度必须等于 Joint 数量

---

## 136. 功能 C：实现柔性位置跟踪

只需要切换 gains 所属 mode

```cpp
robot.set_impedance_mode(
    JointImpedanceMode::COMPLIANT_TRACKING);

while(running) {
    const auto now =
        Robot::Clock::now();

    robot.set_cmd(
        JointPosCmd{target},
        now);

    auto result =
        robot.cycle(now);

    if(!result) {
        break;
    }
}
```

柔性程度主要来自

```yaml
compliant_tracking:
  kp:
  kd:
```

不是来自不同的命令结构

---

## 137. 功能 D：实现手动拖拽

```cpp
auto result =
    robot.set_impedance_mode(
        JointImpedanceMode::COMPLIANT_DRAG);

if(!result) {
    return;
}

while(running) {
    auto cycle_result =
        robot.cycle();

    if(!cycle_result) {
        break;
    }
}
```

不要在该模式持续调用 `set_cmd()`

它会被控制器拒绝

---

## 138. 功能 E：实现重力补偿拖拽

配置 Robot 前提供 Dynamics callback

然后确保模型前馈模式为 GRAVITY

```cpp
robot.set_model_feedforward_mode(
    ModelFeedforwardMode::GRAVITY);
```

该函数只能在 INACTIVE 调用

之后

```cpp
robot.activate();

robot.set_impedance_mode(
    JointImpedanceMode::COMPLIANT_DRAG);

while(running) {
    auto result =
        robot.cycle();

    if(!result) {
        break;
    }
}
```

若手感异常，先查看

```cpp
robot.get_model_feedforward();
```

再独立检查

```cpp
dynamics.get_gravity();
dynamics.get_gravity_compensation();
```

---

## 139. 功能 F：读取末端位姿和 Jacobian

```cpp
Dynamics dynamics;

dynamics.configure(
    cfg.dynamics);

dynamics.update(
    state,
    acc,
    ref_acc);

const Eigen::Isometry3d& T =
    dynamics.get_tool_pose();

const Eigen::MatrixXd& J =
    dynamics.get_tool_jacobian();

std::cout
    << T.matrix()
    << "\n";

std::cout
    << J
    << "\n";
```

任意 frame

```cpp
auto camera_pose =
    dynamics.get_frame_pose(
        "camera_link");

auto camera_jacobian =
    dynamics.get_frame_jacobian(
        "camera_link");
```

---

## 140. 功能 G：计算笛卡尔末端速度

已知

```text
dq
J
```

则

```cpp
Eigen::VectorXd dq =
    Eigen::Map<const Eigen::VectorXd>(
        state.vel.data(),
        state.vel.size());

Eigen::VectorXd twist =
    dynamics.get_tool_jacobian()
    * dq;
```

`twist` 为 6 维

具体线速度和角速度排列应与当前 Pinocchio Jacobian convention 保持一致

不要在没有确认 frame convention 的情况下直接把它与相机或世界坐标速度混用

---

## 141. 功能 H：记录一个完整控制周期

```cpp
auto result =
    robot.cycle();

if(!result) {
    return;
}

const RobotCycleOutput& cycle =
    result.value();

log(cycle.dt);

log(cycle.joint_state.pos);
log(cycle.joint_state.vel);
log(cycle.joint_state.tor);

log(cycle.joint_acc);
log(cycle.joint_ref_acc);
log(cycle.model_feedforward);

log(cycle.joint_cmd.pos);
log(cycle.joint_cmd.vel);
log(cycle.joint_cmd.tor);
log(cycle.joint_cmd.kp);
log(cycle.joint_cmd.kd);

log(cycle.actuator_cmd.pos);
log(cycle.actuator_cmd.vel);
log(cycle.actuator_cmd.tor);
```

这比只记录 `/joint_states` 更适合分析控制闭环

因为可以同时看到

```text
measured state
reference
model feedforward
Safety 后 Joint command
映射后的 Actuator command
```

---

## 142. 功能 I：处理 Robot FAULT

```cpp
auto cycle_result =
    robot.cycle();

if(!cycle_result) {
    const auto& fault_opt =
        robot.get_last_fault();

    if(fault_opt) {
        const RobotFault& fault =
            fault_opt.value();

        std::cerr
            << "RobotErr="
            << static_cast<int>(
                fault.code)
            << "\n";
    }
}
```

如果

```cpp
robot.is_fault_holding()
```

为 true

控制线程进入

```cpp
while(robot.get_state() ==
      RobotState::FAULT)
{
    auto hold_result =
        robot.maintain_fault_hold();

    if(!hold_result) {
        break;
    }

    if(operator_requests_clear) {
        auto clear_result =
            robot.clear_fault();

        if(clear_result) {
            break;
        }
    }
}
```

如果不希望继续保持

```cpp
robot.force_deactivate();
```

---

## 143. 功能 J：Python 完成一次位置移动

```python
from pathlib import Path

import numpy as np
import serial_arm

config_file = Path(
    "install/standalone/share/dm_arm_description/config/core/gray.yaml"
)

hardware_plugin = (
    "install/standalone/lib/libserial_arm_hardware_damiao.so"
)

hardware_config = Path(
    "install/standalone/share/dm_arm_description/config/hardware.yaml"
)

with serial_arm.RobotSession(
    config_file,
    hardware_plugin,
    hardware_config,
) as arm:
    arm.set_impedance_mode(
        serial_arm.JointImpedanceMode.RIGID_TRACKING
    )

    snapshot = arm.snapshot

    if not snapshot.valid:
        raise RuntimeError(
            snapshot.last_error
        )

    target = (
        snapshot.cycle.joint_state.pos.copy()
    )

    target[0] += 0.05

    arm.move_to(
        target,
        speed_scale=0.15,
    )
```

业务脚本不需要自己创建 200 Hz Python 循环

---

## 144. 功能 K：Python 独立计算 Dynamics

```python
import numpy as np
import serial_arm

cfg = serial_arm.load_robot_cfg(
    "config/core/gray.yaml",
    "serial_arm_hardware_damiao",
    "config/hardware.yaml",
)

dynamics = serial_arm.Dynamics()

dynamics.configure(
    cfg.dynamics
)

n = len(cfg.joint_names)

q = np.zeros(n, dtype=np.float64)
dq = np.zeros(n, dtype=np.float64)
ddq = np.zeros(n, dtype=np.float64)
tau = np.zeros(n, dtype=np.float64)
ddq_ref = np.zeros(n, dtype=np.float64)

dynamics.update(
    q,
    dq,
    ddq,
    tau,
    ddq_ref,
)

print(
    dynamics.gravity_compensation
)

print(
    dynamics.mass_matrix
)

print(
    dynamics.tool_pose
)

print(
    dynamics.tool_jacobian
)
```

---

## 145. 功能 L：新增 Backend 的最小骨架

```cpp
#include "serial_arm/hardware/motor_bus.hpp"

class MyMotorBus final
    : public serial_arm::MotorBus
{
public:
    tl::expected<
        void,
        serial_arm::MotorBusErr>
    configure(
        const std::string& path) override
    {
        // 解析 Backend YAML
        // 构造 capabilities
        return {};
    }

    tl::expected<
        void,
        serial_arm::MotorBusErr>
    connect() override
    {
        // 打开总线
        return {};
    }

    tl::expected<
        serial_arm::ActuatorState,
        serial_arm::MotorBusErr>
    read() override
    {
        serial_arm::ActuatorState state;

        // 厂商反馈
        // -> rad
        // -> rad/s
        // -> N*m

        return state;
    }

    tl::expected<
        void,
        serial_arm::MotorBusErr>
    activate() override
    {
        // enable
        // mode switch
        return {};
    }

    tl::expected<
        void,
        serial_arm::MotorBusErr>
    write(
        const serial_arm::ActuatorCtrlCmd& cmd)
        override
    {
        // cmd.pos
        // cmd.vel
        // cmd.tor
        // cmd.kp
        // cmd.kd
        // -> 厂商协议

        return {};
    }

    tl::expected<
        void,
        serial_arm::MotorBusErr>
    stop() override
    {
        return {};
    }

    tl::expected<
        void,
        serial_arm::MotorBusErr>
    deactivate() override
    {
        return {};
    }

    tl::expected<
        void,
        serial_arm::MotorBusErr>
    recover() override
    {
        return {};
    }

    const serial_arm::HardwareCapabilities&
    capabilities() const noexcept override
    {
        return capabilities_;
    }

    void cleanup() noexcept override
    {
    }

    std::size_t
    size() const noexcept override
    {
        return capabilities_.size();
    }

private:
    serial_arm::HardwareCapabilities
        capabilities_;
};
```

Backend shared library还需要按 HardwareLoader contract 导出创建和销毁入口

具体命名必须与当前 `HardwareLoader` 约定保持一致

---

# Part XVI 错误码索引

错误码用于定位失败发生在哪一层

不要只记录整数值

日志中应同时记录错误类型名、顶层错误和有意义的子错误

## `ConfigErr`

| 错误 | 含义 |
| --- | --- |
| `FILE_OPEN_FAILED` | 配置文件无法打开 |
| `SYNTAX_ERROR` | YAML 语法错误 |
| `MISSING_FIELD` | 缺少必需字段 |
| `INVALID_VALUE` | 字段值非法 |
| `INVALID_SIZE` | 数组长度不一致 |
| `DUPLICATE_NAME` | Joint 或 Actuator 名称重复 |

`ConfigErrInfo` 额外提供 `message`

优先把 `message` 直接写入日志

## `RobotProfileErr`

| 错误 | 含义 |
| --- | --- |
| `PROFILE_FILE_NOT_FOUND` | 找不到 Profile 文件 |
| `PROFILE_LOAD_FAILED` | Profile 读取或解析失败 |
| `PROFILE_NOT_FOUND` | 请求的 profile name 不存在 |
| `MISSING_FIELD` | Profile 缺少必需字段 |
| `RESOURCE_NOT_FOUND` | package 或 resource 路径无法解析 |

`RobotProfileErrInfo` 同样提供明确 `message`

## `ModelErr`

| 错误 | 含义 |
| --- | --- |
| `FILE_OPEN_FAILED` | URDF 无法打开 |
| `URDF_LOAD_FAILED` | URDF 解析失败 |
| `MISSING_JOINT` | 请求的受控 Joint 不存在 |
| `DUPLICATE_JOINT` | 受控 Joint 名称重复 |
| `FIXED_JOINT_CONTROLLED` | fixed Joint 被错误加入控制列表 |
| `INVALID_LIMIT` | URDF limit 无效 |

## `LimitResolverErr`

| 错误 | 含义 |
| --- | --- |
| `INVALID_INPUT` | Model、Mapper、Capability 或 Policy 输入无效 |
| `MISSING_ACTUATOR` | 缺少对应 Actuator capability |
| `POLICY_WIDENS_LIMIT` | Safety policy 尝试放宽底层限制 |

## `JointActuatorMapErr`

| 错误 | 含义 |
| --- | --- |
| `OK` | 映射正常 |
| `NOT_CONFIGURED` | Mapper 尚未配置 |
| `INVALID_CFG` | 映射配置无效 |
| `INVALID_JOINT_STATE` | JointState 无效 |
| `INVALID_ACTUATOR_STATE` | ActuatorState 无效 |
| `INVALID_JOINT_CMD` | JointCtrlCmd 无效 |
| `INVALID_ACTUATOR_CMD` | ActuatorCtrlCmd 无效 |
| `INVALID_CONVERSION_VALUE` | 映射结果出现非法数值 |

## `JointCtrllerErr`

| 错误 | 含义 |
| --- | --- |
| `OK` | 控制器正常 |
| `NOT_CONFIGURED` | 尚未 configure |
| `NOT_INITIALIZED` | 尚未 initialize |
| `ALREADY_INITIALIZED` | 重复 initialize |
| `INVALID_CFG` | 控制器配置无效 |
| `INVALID_STATE` | JointState 无效 |
| `INVALID_DT` | dt 非法 |
| `INVALID_MODEL_FEEDFORWARD` | 模型前馈维度或数值非法 |
| `INVALID_IMPEDANCE_MODE` | 阻抗模式非法 |
| `INVALID_CMD_SIZE` | 参考命令长度错误 |
| `INVALID_CMD_VALUE` | 参考命令包含非法数值 |
| `INVALID_FULL_CMD` | 完整命令非法 |
| `CMD_NOT_ALLOWED_IN_MODE` | 当前不是 tracking mode |
| `FULL_CMD_NOT_ALLOWED` | `allow_full_cmd=false` |

## `SafetyErr`

| 错误 | 含义 |
| --- | --- |
| `NOT_CONFIGURED` | Safety 尚未配置 |
| `INVALID_CFG` | Safety 配置无效 |
| `INVALID_DT` | 控制周期非法 |
| `INVALID_STATE_AGE` | state age 非法 |
| `INVALID_CMD_AGE` | command age 非法 |
| `STATE_TIMEOUT` | 状态超时 |
| `CMD_TIMEOUT` | 跟踪命令超时 |
| `INVALID_JOINT_STATE_SIZE` | JointState 长度错误 |
| `INVALID_ACTUATOR_STATE_SIZE` | ActuatorState 长度错误 |
| `NON_FINITE_JOINT_STATE` | JointState 含 NaN 或 Inf |
| `NON_FINITE_ACTUATOR_STATE` | ActuatorState 含 NaN 或 Inf |
| `JOINT_POS_LIMIT` | 实测 Joint 位置越界 |
| `JOINT_VEL_LIMIT` | 实测 Joint 速度越界 |
| `ACTUATOR_OFFLINE` | 执行器离线 |
| `ACTUATOR_NOT_ENABLED` | 执行器未使能 |
| `ACTUATOR_FAULT` | 执行器报告故障 |
| `INVALID_CMD_SIZE` | JointCtrlCmd 长度错误 |
| `NON_FINITE_CMD` | 命令含 NaN 或 Inf |
| `CMD_POS_LIMIT` | 目标位置越界 |
| `CMD_VEL_LIMIT` | 目标速度越界 |
| `CMD_EFFORT_LIMIT` | 前馈力矩越界 |
| `CMD_KP_LIMIT` | kp 越界 |
| `CMD_KD_LIMIT` | kd 越界 |
| `CMD_POS_STEP_LIMIT` | 相邻命令位置跳变过大 |
| `CMD_VEL_STEP_LIMIT` | 相邻命令速度跳变过大 |

## `DynamicsErr`

| 错误 | 含义 |
| --- | --- |
| `NOT_CONFIGURED` | Dynamics 尚未配置 |
| `ALREADY_CONFIGURED` | 重复配置 |
| `NOT_UPDATED` | 尚无合法 Dynamics cache |
| `INVALID_CFG` | DynamicsCfg 无效 |
| `URDF_LOAD_FAILED` | URDF 模型构建失败 |
| `JOINT_NOT_FOUND` | 配置 Joint 不存在 |
| `JOINT_NOT_1DOF` | 受控 Joint 不是支持的单自由度 Joint |
| `MODEL_SIZE_MISMATCH` | Pinocchio 模型维度与 Joint 映射不一致 |
| `FRAME_NOT_FOUND` | 请求 frame 不存在 |
| `INVALID_INPUT_SIZE` | 输入向量长度错误 |
| `NON_FINITE_INPUT` | 输入含 NaN 或 Inf |
| `GRAVITY_SCALE_OUT_OF_RANGE` | gravity scale 超出 `[0, 1]` |
| `COMPUTE_FAILED` | 底层动力学计算失败 |

## `MotorBusErr`

| 错误 | 含义 |
| --- | --- |
| `NOT_CONFIGURED` | Backend 尚未配置 |
| `NOT_CONNECTED` | 底层设备尚未连接 |
| `NOT_ACTIVE` | Backend 尚未进入可写状态 |
| `INVALID_CFG` | Hardware YAML 无效 |
| `OPEN_FAILED` | 打开设备失败 |
| `READ_FAILED` | 读取状态失败 |
| `WRITE_FAILED` | 写命令失败 |
| `INVALID_STATE` | 当前生命周期不允许该操作 |
| `INVALID_CMD` | ActuatorCtrlCmd 无效 |
| `ACTUATOR_OFFLINE` | 执行器离线 |
| `ACTUATOR_FAULT` | 执行器故障 |
| `TIMEOUT` | 通信超时 |
| `ENABLE_FAILED` | 使能失败 |
| `MODE_SWITCH_FAILED` | 控制模式切换失败 |
| `STOP_FAILED` | 安全停止失败 |
| `DISABLE_FAILED` | 失能失败 |
| `RECOVER_FAILED` | 恢复失败 |

## `HardwareLoaderErr`

| 错误 | 含义 |
| --- | --- |
| `OPEN_FAILED` | shared library 无法打开 |
| `SYMBOL_FAILED` | 缺少 create 或 destroy symbol |
| `CREATE_FAILED` | Backend 实例创建失败 |
| `CONFIGURE_FAILED` | `MotorBus::configure()` 失败 |

## `ModelFeedforwardErr`

| 错误 | 含义 |
| --- | --- |
| `NOT_CONFIGURED` | 非 NONE 模式缺少 callback |
| `INVALID_INPUT` | 输入状态无效 |
| `INVALID_MODE` | 模式无效 |
| `COMPUTE_FAILED` | Dynamics 或自定义前馈计算失败 |

## `RobotErr`

| 错误 | 含义 |
| --- | --- |
| `NOT_CONFIGURED` | Robot 尚未 configure |
| `ALREADY_CONFIGURED` | 重复 configure |
| `INVALID_CFG` | RobotCfg 无效 |
| `NULL_MOTOR_BUS` | MotorBus 为空 |
| `MOTOR_BUS_SIZE_MISMATCH` | Backend 数量与 Joint 数量不一致 |
| `WRITE_DISABLED` | `write_enabled=false` |
| `NOT_ACTIVE` | 当前不是 ACTIVE |
| `NOT_INACTIVE` | 当前不是 INACTIVE |
| `ALREADY_ACTIVE` | 重复 activate |
| `FAULTED` | 当前处于 FAULT |
| `NOT_FAULTED` | 当前不在 FAULT |
| `INVALID_TIME` | 时间戳回退或非法 |
| `MOTOR_BUS_CONNECT_FAILED` | Backend connect 失败 |
| `MOTOR_BUS_ACTIVATE_FAILED` | Backend activate 失败 |
| `MOTOR_BUS_READ_FAILED` | Backend read 失败 |
| `MOTOR_BUS_WRITE_FAILED` | Backend write 失败 |
| `MOTOR_BUS_DEACTIVATE_FAILED` | Backend deactivate 失败 |
| `MOTOR_BUS_RECOVER_FAILED` | Backend recover 失败 |
| `MAPPER_FAILED` | JointActuatorMapper 失败 |
| `CTRLLER_FAILED` | JointCtrller 失败 |
| `SAFETY_FAILED` | Safety 检查失败 |
| `MODEL_FEEDFORWARD_FAILED` | 模型前馈计算失败 |
| `INVALID_MODEL_FEEDFORWARD` | 模型前馈结果非法 |
| `FAULT_RECOVERY_NOT_ALLOWED` | 当前条件不允许 fault recovery |

---

# Part XVII 快速索引

## 想控制机械臂位置

使用

```text
Robot
set_impedance_mode(RIGID_TRACKING)
set_cmd(JointPosCmd)
cycle
```

Python 使用

```text
RobotSession
set_impedance_mode
move_to
```

## 想做阻抗拖拽

使用

```text
COMPLIANT_DRAG
Dynamics gravity compensation
cycle
```

## 想做柔性轨迹跟踪

使用

```text
COMPLIANT_TRACKING
set_cmd
cycle
```

## 想读取关节状态

使用

```text
RobotCycleOutput::joint_state
Robot::get_joint_state
RobotSessionSnapshot::cycle
```

## 想读取执行器原始统一状态

使用

```text
RobotCycleOutput::actuator_state
Robot::get_actuator_state
```

这里的状态仍然已经被 Backend 转换为 SerialArm 单位，不是厂商原始字节

## 想计算重力补偿

使用

```text
Dynamics::update
Dynamics::get_gravity_compensation
```

## 想计算末端位姿

使用

```text
Dynamics::update
Dynamics::get_tool_pose
```

## 想计算任意 frame 位姿

使用

```text
Dynamics::get_frame_pose
```

## 想计算 Jacobian

使用

```text
Dynamics::get_tool_jacobian
Dynamics::get_frame_jacobian
```

## 想开发新电机驱动

实现

```text
MotorBus
HardwareCapabilities
HardwareLoader plugin contract
```

## 想排查方向和减速比

使用

```text
JointActuatorMapCfg
JointActuatorMapper
```

## 想排查限位

使用

```text
ModelLoader
LimitResolver
Safety
```

## 想排查控制故障

先看

```text
RobotFault::code
```

再看对应子错误

```text
motor_bus_err
mapper_err
ctrller_err
safety_fault
model_feedforward_err
```