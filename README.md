# DM-Arm Hardware Interface

> 面向 DM-Arm 六轴机械臂的独立 C++17 控制平台；项目将关节控制、Joint/Actuator 映射、安全检查、Robot 生命周期和硬件后端解耦，并为后续 Pinocchio 动力学、ROS 2 / ros2_control 与 Python binding 提供统一接口

---

## 目录

- [项目目标](#项目目标)
- [安全警告](#安全警告)
- [当前实现状态](#当前实现状态)
- [系统架构](#系统架构)
- [模块说明](#模块说明)
- [目录结构](#目录结构)
- [依赖与平台](#依赖与平台)
- [构建与安装](#构建与安装)
- [在其他 CMake 项目中使用](#在其他-cmake-项目中使用)
- [快速开始](#快速开始)
- [控制模式与命令](#控制模式与命令)
- [Robot 生命周期](#robot-生命周期)
- [Safety 行为](#safety-行为)
- [配置文件](#配置文件)
- [接入自定义 MotorBus](#接入自定义-motorbus)
- [模型前馈接口](#模型前馈接口)
- [达妙后端](#达妙后端)
- [CMake 选项](#cmake-选项)
- [已知限制](#已知限制)
- [开发路线](#开发路线)
- [贡献](#贡献)
- [许可证与第三方代码](#许可证与第三方代码)

---

## 项目目标

DM-Arm 不只是一个串口驱动，也不只是一个 ROS 2 硬件插件；项目目标是提供一套可被不同上层复用的机械臂平台核心：

```text
关节命令 / 上层规划
        ↓
JointCtrller
        ↓
Safety
        ↓
JointActuatorMapper
        ↓
Robot
        ↓
MotorBus
        ↓
Damiao / Fake / 其他硬件后端
```

设计重点：

- **Joint 与 Actuator 分离**：控制算法工作在关节侧，硬件协议工作在执行器侧；
- **硬件后端可替换**：`Robot` 只依赖 `MotorBus` 接口；
- **错误显式返回**：使用 `tl::expected`，避免用异常表达正常运行错误；
- **安全逻辑集中**：状态、命令、超时和跳变检查统一由 `Safety` 处理；
- **生命周期统一**：连接、使能、运行、故障和复位由 `Robot` 管理；
- **动力学是一等能力**：模型前馈已经在 `Robot` 控制周期中预留正式接口；
- **适配层不复制核心逻辑**：未来 ROS 2 和 Python 层只调用公共 C++ API

---

## 安全警告

> [!WARNING]
> 本项目会控制真实机械臂；错误的电机 ID、方向、零位、减速比、限位、增益或负载参数都可能导致机械臂突然运动、碰撞、过流或坠落

真机运行前至少完成：

1. 设置独立急停并确认可以切断执行器动力；
2. 将机械臂放置在支撑工装或安全姿态；
3. 逐轴确认电机型号、CAN ID、master ID 和反馈方向；
4. 标定 `direction`、位置比例和零位偏移；
5. 从低 `kp`、低速度、低力矩限制开始；
6. 确认 `stop()` 和 `deactivate()` 在当前机构上是安全的；
7. 在完成以上检查前保持：

```yaml
runtime:
  write_enabled: false
```

`write_enabled: false` 时，`Robot::activate()` 会返回 `RobotErr::WRITE_DISABLED`；这是故意设置的真机门禁

本项目仍处于早期开发阶段，不能替代工业安全控制器、硬限位、制动器、急停回路或风险评估

---

## 当前实现状态

| 主线 / 模块 | 状态 | 当前能力 |
|---|---:|---|
| 基础类型 `types.hpp` | 已实现 | Joint/Actuator 状态、命令、阻抗模式、模型前馈模式 |
| `JointCtrller` | 已实现 | 五种阻抗行为；位置、位置速度、位置速度力矩命令 |
| `JointActuatorMapper` | 已实现 | 方向、比例、关节零位和执行器零位映射 |
| `Safety` | 已实现 | 状态、在线、使能、超时、限位、增益、跳变和 NaN/Inf 检查 |
| `Robot` | 已实现 | 生命周期、统一控制周期、FAULT 锁存、错误分流和模型前馈入口 |
| YAML 配置 | 已实现 | 使用 `yaml-cpp` 加载并验证固定六轴配置 |
| `MotorBus` | 已实现 | 通用执行器后端接口 |
| `DamiaoMotorBus` | 可选构建 | 串口连接、达妙 MIT 控制、状态读取、使能、停止和失能 |
| Pinocchio 动力学 | 规划中 | 尚未生成 `dm_arm::dynamics` target |
| ROS 2 / ros2_control | 规划中 | CMake 选项已预留，但尚无适配包或插件 |
| Python binding | 规划中 | CMake 选项已预留，但尚无 pybind11 模块 |
| 自动化测试与诊断工具 | 暂缓 | 将在核心接口稳定后加入 |

当前对外 CMake targets：

```text
dm_arm::core      基础类型、JointCtrller、Mapper、Safety
dm_arm::config    yaml-cpp 配置加载与验证
dm_arm::robot     Robot 生命周期与完整控制周期
dm_arm::damiao    可选达妙后端
```

---

## 系统架构

### 控制周期

`Robot::cycle()` 的固定执行顺序：

```text
MotorBus::read()
        ↓
ActuatorState
        ↓
JointActuatorMapper::to_joint_state()
        ↓
Safety::check_state()
        ↓
Safety::check_cmd_age()        仅跟踪模式且收到过外部命令时
        ↓
ModelFeedforwardFn             NONE / GRAVITY / FULL_INVERSE_DYNAMICS
        ↓
JointCtrller::update()
        ↓
Safety::check_joint_cmd()
        ↓
JointActuatorMapper::to_actuator_cmd()
        ↓
MotorBus::write()
```

未经 `Safety` 检查的关节命令不会进入执行器映射

### 依赖关系

```text
                         dm_arm::core
                         /          \
                        /            \
              dm_arm::config      dm_arm::damiao
                     │                  │
                     └── dm_arm::robot  │
                              ▲         │
                              └─ 应用层注入具体 MotorBus

      dynamics（规划）/ ROS 2（规划）/ Python（规划）
                 复用 Robot 与 Core 公共接口
```

`DamiaoMotorBus` 依赖达妙通信代码是正常的；通用控制模块不会包含 `dm_hw/damiao.hpp`

---

## 模块说明

### `dm_arm::core`

包含：

```text
include/dm_arm/core/types.hpp
include/dm_arm/core/joints_ctrller.hpp
include/dm_arm/core/joint_actuator_mapper.hpp
include/dm_arm/core/safety.hpp
```

#### `JointCtrller`

负责根据当前阻抗模式、外部参考和模型前馈生成关节侧完整命令：

```cpp
struct JointCtrlCmd {
    JointVector pos;
    JointVector vel;
    JointVector tor;
    JointVector kp;
    JointVector kd;
};
```

它不负责：

- 电机 ID 和型号；
- 串口或 CAN 通信；
- Joint/Actuator 比例；
- 软件限位；
- URDF 和动力学模型；
- ROS 消息

#### `JointActuatorMapper`

负责：

```text
JointCtrlCmd  → ActuatorMitCmd
ActuatorState → JointState
```

映射配置包括：

- `pos_ratio`：位置比例；
- `tor_ratio`：力矩比例；
- `direction`：方向；
- `joint_zero_offset`：关节侧零位；
- `actuator_zero_offset`：执行器侧零位

#### `Safety`

负责关节状态、执行器健康、命令范围、超时和单周期变化检查；`Safety` 只返回安全结果和建议动作，不直接操作硬件

### `dm_arm::config`

使用 `yaml-cpp`：

- 读取 `config/dm_arm.yaml`；
- 构造 `RobotCfg`；
- 校验字段类型、数组长度、名称、ID 和模块基础配置；
- 返回带消息的 `ConfigErrInfo`

配置文件只应在启动或重新配置阶段读取，不应在控制周期内读取

### `dm_arm::robot`

`Robot` 组合：

```text
JointCtrller
JointActuatorMapper
Safety
MotorBus
ModelFeedforwardFn
```

它负责：

- 接管 `MotorBus` 所有权；
- 生命周期管理；
- 硬件连接和激活；
- 完整控制周期；
- 命令时间戳；
- 状态时间戳；
- `FAULT` 锁存；
- 安全动作执行；
- 模型前馈调用

### `dm_arm::damiao`

可选达妙执行器后端，负责：

- Linux 串口打开；
- 创建和注册达妙 Motor；
- 电机使能和失能；
- 切换 MIT 模式；
- 编码执行器侧 MIT 命令；
- 读取位置、速度和力矩；
- 使用 SDK 自带电机范围校验执行器命令；
- 低增益停止保持和最终失能

它不负责 Joint 零位、减速比、Joint 限位、阻抗模式或动力学

---

## 目录结构

```text
dm_arm_hardware_interface/
├── CMakeLists.txt
├── package.xml
├── README.md
├── LICENSE
│
├── cmake/
│   └── dm_arm_hardware_interfaceConfig.cmake.in
│
├── config/
│   └── dm_arm.yaml
│
├── include/dm_arm/
│   ├── config/
│   │   └── config.hpp
│   ├── core/
│   │   ├── types.hpp
│   │   ├── joints_ctrller.hpp
│   │   ├── joint_actuator_mapper.hpp
│   │   └── safety.hpp
│   ├── hardware/
│   │   ├── motor_bus.hpp
│   │   └── damiao_motor_bus.hpp
│   └── robot.hpp
│
├── src/
│   ├── config/config.cpp
│   ├── core/
│   │   ├── joints_ctrller.cpp
│   │   ├── joint_actuator_mapper.cpp
│   │   └── safety.cpp
│   ├── hardware/damiao_motor_bus.cpp
│   └── robot.cpp
│
├── description/
│   ├── urdf/dm_arm.urdf
│   └── meshes/
│
├── docs/
│   └── 开发路线与详细设计文档
│
└── third_lib/
    ├── tl/
    └── damiao/
```

当前仓库按要求暂不包含 `tests/` 和 `tools/`

---

## 依赖与平台

### 必需依赖

- CMake `>= 3.20`；
- 支持 C++17 的编译器；
- `yaml-cpp`；
- Linux/Unix 风格构建环境

Ubuntu 22.04：

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  libyaml-cpp-dev
```

### 达妙后端附加要求

- Linux `termios` 串口接口；
- 与 `third_lib/damiao` 协议兼容的串口 USB-CAN 设备；
- 串口读写权限；
- 正确的电机 ID、master ID、型号和接线

常见串口权限处理方式：

```bash
sudo usermod -aG dialout "$USER"
```

修改用户组后需要重新登录

---

## 构建与安装

### 仅构建 Core、Config 和 Robot

即使关闭达妙后端，当前配置模块仍依赖 `yaml-cpp`

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DDM_ARM_BUILD_DAMIAO=OFF

cmake --build build -j
```

安装到项目内目录：

```bash
cmake --install build --prefix "$PWD/install"
```

### 构建达妙后端

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DDM_ARM_BUILD_DAMIAO=ON

cmake --build build -j
cmake --install build --prefix "$PWD/install"
```

### 清理构建

```bash
rm -rf build install
```

### 使用 colcon 构建

`package.xml` 将包声明为普通 CMake 包，因此可以放入 ROS 2 工作空间后由 `colcon` 调用 CMake：

```bash
colcon build \
  --packages-select dm_arm_hardware_interface \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DDM_ARM_BUILD_DAMIAO=ON
```

这不等于当前已经实现 `ros2_control` 插件这里只是使用 `colcon` 组织普通 CMake 包

---

## 在其他 CMake 项目中使用

安装后，将安装前缀加入 `CMAKE_PREFIX_PATH`：

```bash
export CMAKE_PREFIX_PATH="$PWD/install:$CMAKE_PREFIX_PATH"
```

下游项目：

```cmake
cmake_minimum_required(VERSION 3.20)
project(dm_arm_example LANGUAGES CXX)

find_package(dm_arm_hardware_interface REQUIRED CONFIG)

add_executable(dm_arm_example main.cpp)
target_compile_features(dm_arm_example PRIVATE cxx_std_17)

target_link_libraries(dm_arm_example
    PRIVATE
        dm_arm::robot
        dm_arm::damiao   # 只有构建并安装达妙后端时才可链接
)
```

如果只实现自己的 `MotorBus`，通常只需要：

```cmake
target_link_libraries(your_target PRIVATE dm_arm::robot)
```

可用 targets：

| Target | 是否始终生成 | 主要头文件 |
|---|---:|---|
| `dm_arm::core` | 是 | `dm_arm/core/*.hpp` |
| `dm_arm::config` | 是 | `dm_arm/config/config.hpp` |
| `dm_arm::robot` | 是 | `dm_arm/robot.hpp` |
| `dm_arm::damiao` | `DM_ARM_BUILD_DAMIAO=ON` | `dm_arm/hardware/damiao_motor_bus.hpp` |

---

## 快速开始

下面示例展示完整的配置、达妙后端注入和 Robot 生命周期；它会真实使能执行器，使用前必须完成真机检查并在 YAML 中明确设置 `write_enabled: true`

```cpp
#include <dm_arm/config/config.hpp>
#include <dm_arm/hardware/damiao_motor_bus.hpp>
#include <dm_arm/robot.hpp>

#include <chrono>
#include <cstddef>
#include <iostream>
#include <memory>
#include <thread>
#include <utility>

int main() {
    using dm_arm::Robot;

    // 1. 加载 YAML
    auto cfg_result = dm_arm::load_robot_cfg("config/dm_arm.yaml");
    if(!cfg_result) {
        std::cerr << "Failed to load config: "
                  << cfg_result.error().message << '\n';
        return 1;
    }
    dm_arm::RobotCfg cfg = std::move(cfg_result.value());

    // 默认配置会阻止真实写入
    if(!cfg.runtime.write_enabled) {
        std::cerr
            << "Hardware write is disabled. Complete commissioning first, "
            << "then set runtime.write_enabled to true.\n";
        return 2;
    }

    // 2. 配置具体硬件后端
    auto bus = std::make_unique<dm_arm::DamiaoMotorBus>();
    auto bus_cfg_result = bus->configure(cfg.damiao);
    if(!bus_cfg_result) {
        std::cerr << "Failed to configure Damiao bus, error="
                  << static_cast<int>(bus_cfg_result.error()) << '\n';
        return 3;
    }

    // 3. Robot 接管 MotorBus 所有权
    Robot robot;
    auto robot_cfg_result = robot.configure(cfg, std::move(bus));
    if(!robot_cfg_result) {
        std::cerr << "Failed to configure robot, error="
                  << static_cast<int>(robot_cfg_result.error().code) << '\n';
        return 4;
    }

    // 4. connect → enable → read initial state → initialize controller
    auto activate_result = robot.activate();
    if(!activate_result) {
        std::cerr << "Failed to activate robot, error="
                  << static_cast<int>(activate_result.error().code) << '\n';
        return 5;
    }

    const auto period = std::chrono::duration_cast<Robot::Clock::duration>(
        std::chrono::duration<double>(1.0 / cfg.runtime.ctrl_frequency_hz));
    auto next_tick = Robot::Clock::now();

    // 示例只运行固定周期；真实程序应接入退出信号和上层命令源
    for(std::size_t cycle_index = 0;
        cycle_index < 2000 && robot.get_state() == dm_arm::RobotState::ACTIVE;
        ++cycle_index) {

        const auto cycle_result = robot.cycle(Robot::Clock::now());
        if(!cycle_result) {
            std::cerr << "Robot cycle failed, error="
                      << static_cast<int>(cycle_result.error().code) << '\n';
            break;
        }

        next_tick += period;
        std::this_thread::sleep_until(next_tick);
    }

    // 5. 正常退出时显式失能
    if(robot.get_state() == dm_arm::RobotState::ACTIVE) {
        const auto deactivate_result = robot.deactivate();
        if(!deactivate_result) {
            std::cerr << "Failed to deactivate robot, error="
                      << static_cast<int>(deactivate_result.error().code)
                      << '\n';
            return 6;
        }
    }

    // FAULT 状态下应检查 get_last_fault()，排除原因后再 reset_fault()
    return robot.get_state() == dm_arm::RobotState::FAULT ? 7 : 0;
}
```

### 重要调度说明

`Robot` 不创建后台线程，也不会自动睡眠到 200 Hz；`ctrl_frequency_hz` 用于配置校验和首周期 nominal `dt`；调用方必须负责：

- 按目标频率调用 `cycle()`；
- 使用 `std::chrono::steady_clock`；
- 避免时间戳倒退；
- 避免在控制线程中读取 YAML、分配大型资源或打印大量日志

---

## 控制模式与命令

### 阻抗模式

```cpp
enum class JointImpedanceMode {
    RIGID_HOLD,
    RIGID_TRACKING,
    COMPLIANT_HOLD,
    COMPLIANT_DRAG,
    COMPLIANT_TRACKING,
};
```

| 模式 | 行为 | 是否需要持续外部命令 |
|---|---|---:|
| `RIGID_HOLD` | 以刚性增益保持进入模式时的位置 | 否 |
| `RIGID_TRACKING` | 以刚性增益跟踪参考 | 是 |
| `COMPLIANT_HOLD` | 以较低增益保持位置 | 否 |
| `COMPLIANT_DRAG` | 低/零位置刚度并保留阻尼，服务拖拽 | 否 |
| `COMPLIANT_TRACKING` | 以柔性增益跟踪参考 | 是 |

切换模式：

```cpp
auto result = robot.set_impedance_mode(
    dm_arm::JointImpedanceMode::RIGID_TRACKING);
```

### 参考命令

项目只接受以下三类常规 Joint 命令：

```cpp
dm_arm::JointPosCmd

dm_arm::JointPosVelCmd

dm_arm::JointPosVelTorCmd
```

示例：

```cpp
dm_arm::JointPosVelCmd cmd;
cmd.pos = {0.0, 1.0, 1.5, 0.0, 0.0, 0.0};
cmd.vel = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

auto result = robot.set_cmd(cmd);
```

`set_full_cmd()` 只有在：

```yaml
controller:
  allow_full_cmd: true
```

时才允许直接设置 `pos/vel/tor/kp/kd` 全部字段

### 不要直接发送远距离阶跃目标

`Safety` 会检查相邻合法命令之间的位置和速度变化；直接将目标从当前位置跳到数十度之外，通常会触发：

```text
CMD_POS_STEP_LIMIT
或
CMD_VEL_STEP_LIMIT
```

上层必须先使用轨迹生成器、JointTrajectoryController 或限速插值器，把目标离散成连续的 200 Hz 命令序列

---

## Robot 生命周期

```text
UNCONFIGURED
      │ configure(cfg, bus)
      ▼
  INACTIVE
      │ activate()
      ▼
   ACTIVE ────────────────┐
      │                   │ 任何锁存故障
      │ deactivate()      ▼
      ▼                 FAULT
  INACTIVE                 │ reset_fault()
                           ▼
                       INACTIVE
```

### `configure()`

- 验证通用 Robot 配置；
- 检查 `MotorBus::size()` 与关节数量一致；
- 配置 Controller、Mapper 和 Safety；
- 接管 `MotorBus` 所有权；
- 非 `NONE` 前馈模式必须提供 `ModelFeedforwardFn`

### `activate()`

执行：

```text
检查 write_enabled
→ MotorBus::connect()
→ MotorBus::activate()
→ 读取真实执行器状态
→ 映射到 JointState
→ Safety 状态检查
→ 用真实状态初始化 JointCtrller
→ 初始化 Safety 命令历史
→ ACTIVE
```

### `cycle()`

只能在 `ACTIVE` 调用；任何失败都会返回 `RobotFault`；部分错误还会使 Robot 进入锁存 `FAULT`

### `deactivate()`

正常停止并失能，成功后回到 `INACTIVE`

### `reset_fault()`

只在已经排除故障原因后调用；它会再次尝试失能硬件；失能失败时保持 `FAULT`，不会错误地宣称设备安全

---

## Safety 行为

### 状态检查

`Safety::check_state()` 检查：

- JointState 与 ActuatorState 数组长度；
- NaN/Inf；
- 状态年龄；
- Joint 位置硬限位；
- Joint 速度限制；
- 每个执行器的 `online`；
- 每个执行器的 `enabled`；
- 每个执行器的 `err_code`

### 命令检查

`Safety::check_joint_cmd()` 检查：

- `dt`；
- 命令数组长度；
- NaN/Inf；
- 位置软限位；
- 速度、力矩、`kp`、`kd` 上限；
- 相邻合法命令的位置变化；
- 相邻合法命令的速度变化

轻微浮点误差级越界允许 clamp 并增加计数；明显错误不会被静默截断

### 超时

- 状态超时：状态不再可信，建议 `DISABLE`；
- 命令超时：只对收到过外部命令的跟踪模式生效，建议 `STOP_HOLD`；
- 保持和拖拽模式不会因为没有持续外部命令而超时

### 故障动作

| 故障类别 | SafetyAction | Robot 行为 |
|---|---|---|
| 命令非法、命令超时、模型前馈失败 | `STOP_HOLD` | 尝试 `MotorBus::stop()`；失败后降级失能 |
| 状态超时、离线、硬件错误、状态非法 | `DISABLE` | 调用 `MotorBus::deactivate()` |

发生故障后本周期立即终止，不会继续发送原命令

---

## 配置文件

默认配置：

```text
config/dm_arm.yaml
```

### 顶层结构

```yaml
joints:      # 固定 Joint 名称与顺序
runtime:     # 控制频率、写门禁、模型前馈模式
safety:      # timeout、dt、在线和使能要求
limits:      # Joint 侧位置、速度、加速度、力矩和增益限制
mapping:     # Joint/Actuator 比例、方向和零位
controller:  # 五种阻抗模式增益
damiao:      # 达妙串口和执行器配置
```

实际 YAML 键为小写 `damiao`

### 固定 Joint 顺序

当前主链固定为六轴：

```yaml
joints:
  names: [joint1, joint2, joint3, joint4, joint5, joint6]
```

所有数组、执行器列表和上层命令必须使用同一顺序

### `runtime`

```yaml
runtime:
  ctrl_frequency_hz: 200.0
  write_enabled: false
  model_feedforward_mode: NONE
```

`model_feedforward_mode` 可选：

```text
NONE
GRAVITY
FULL_INVERSE_DYNAMICS
```

当前仓库尚未实现内置动力学对象；选择后两种模式时，调用 `Robot::configure()` 必须传入模型前馈回调

### `limits`

所有限制都在 Joint 侧定义：

- 位置：rad；
- 速度：rad/s；
- 加速度：rad/s²；
- 力矩：N·m；
- `kp`：N·m/rad；
- `kd`：N·m·s/rad

`min_pos/max_pos` 是状态硬边界；命令位置会再加上 `pos_margin`，形成更保守的软边界

### `mapping`

配置值必须来自真实机构和标定结果；默认 `1.0/0.0` 只能表示尚未标定，不能证明机械关系正确

### `controller`

五种模式分别设置 `kp/kd`；仓库中的数值是联调初值，不是适用于所有机械臂和负载的稳定参数

### `damiao`

当前支持的文本型号名称：

```text
DM4310
DM4310_48V
DM4340
DM4340_48V
DM6006
DM6248P
DM8006
DM8009
DM10010L
DM10010
DMH3510
DMH6215
DMG6220
DMJH11
```

当前串口波特率实现支持：

```text
115200
460800    系统 termios 提供 B460800 时
921600    系统 termios 提供 B921600 时
```

执行器侧 `q/dq/tau` 范围直接读取达妙 SDK 的 `Motor::get_limit_param()`，配置模块不维护第二份限制表

---

## 接入自定义 MotorBus

非达妙硬件只需要实现：

```cpp
class MotorBus {
public:
    virtual ~MotorBus() = default;

    virtual tl::expected<void, MotorBusErr> connect() = 0;
    virtual tl::expected<ActuatorState, MotorBusErr> read() = 0;
    virtual tl::expected<void, MotorBusErr> activate() = 0;
    virtual tl::expected<void, MotorBusErr> write(
        const ActuatorMitCmd& cmd) = 0;
    virtual tl::expected<void, MotorBusErr> stop() = 0;
    virtual tl::expected<void, MotorBusErr> deactivate() = 0;

    virtual void cleanup() noexcept = 0;
    virtual std::size_t size() const noexcept = 0;
};
```

后端实现原则：

- `configure()` 在注入 Robot 前由具体后端自行完成；
- `connect()` 打开通信资源，但不要自动下发运动命令；
- `activate()` 执行使能和模式切换；
- `read()` 返回固定长度的 `ActuatorState`；
- `write()` 只接收执行器侧命令；
- `stop()` 尽力进入当前硬件支持的安全保持状态；
- `deactivate()` 失能执行器；
- `cleanup()` 必须 `noexcept`；
- 周期路径不要读取 YAML、重新打开设备或创建电机对象；
- 不要在周期内打印大量日志

`Robot::configure()` 会检查：

```text
motor_bus->size() == cfg.joint_names.size()
```

---

## 模型前馈接口

A 主线已经提供动力学接入点：

```cpp
using ModelFeedforwardFn = std::function<
    tl::expected<JointVector, ModelFeedforwardErr>(
        ModelFeedforwardMode,
        const JointState&,
        double)>;
```

配置为 `NONE` 时，Robot 自动使用六维零向量

配置为 `GRAVITY` 或 `FULL_INVERSE_DYNAMICS` 时，需要在 `configure()` 中传入回调：

```cpp
Robot robot;

auto feedforward = [/* dynamics */](
    dm_arm::ModelFeedforwardMode mode,
    const dm_arm::JointState& state,
    double dt)
    -> tl::expected<dm_arm::JointVector, dm_arm::ModelFeedforwardErr> {

    (void)dt;

    if(mode == dm_arm::ModelFeedforwardMode::GRAVITY) {
        // return dynamics.gravity(state.pos);
    }

    if(mode == dm_arm::ModelFeedforwardMode::FULL_INVERSE_DYNAMICS) {
        // return dynamics.inverse_dynamics(...);
    }

    return tl::make_unexpected(
        dm_arm::ModelFeedforwardErr::COMPUTE_FAILED);
};

robot.configure(cfg, std::move(bus), std::move(feedforward));
```

内置 `dm_arm::dynamics` 尚未实现；未来会使用 Pinocchio 提供：

- URDF 模型加载；
- FK；
- Frame pose；
- Jacobian；
- 质量矩阵；
- 重力项；
- 科氏/离心项；
- RNEA；
- ABA

动力学输出会在 `JointCtrller::update()` 前加入 `model_feedforward`，随后再经过 `Safety` 的完整力矩限制检查

---

## 达妙后端

### 数据边界

```text
Joint 方向、比例、零位
        │
        └── JointActuatorMapper

电机型号、ID、串口、MIT 编解码
        │
        └── DamiaoMotorBus
```

### 激活流程

`DamiaoMotorBus::activate()` 当前负责逐轴使能、切换 MIT 模式并读取启动状态任何一步失败都应由 `Robot` 进入 `FAULT` 并失能已经激活的执行器

### 命令范围

在发送前检查：

```text
0 <= kp <= 500
0 <= kd <= 5
abs(pos) <= motor.q_max
abs(vel) <= motor.dq_max
abs(tor) <= motor.tau_max
```

其中 `q_max/dq_max/tau_max` 来自 `third_lib/damiao` 的电机型号参数

### `stop()`

当前后端会基于最近一次执行器状态发送低增益当前位置保持命令，并在必要时由 Robot 降级执行 `deactivate()`

对于无抱闸机械臂，直接失能可能造成关节坠落；真机使用前必须结合重力方向、支撑结构和制动能力验证停止策略

---

## CMake 选项

| 选项 | 默认值 | 当前状态 |
|---|---:|---|
| `DM_ARM_BUILD_DAMIAO` | `OFF` | 已实现；生成 `dm_arm::damiao` |
| `DM_ARM_ENABLE_DYNAMICS` | `OFF` | 仅预留；当前不会生成 dynamics target |
| `DM_ARM_BUILD_ROS2` | `OFF` | 仅预留；当前不会生成 ROS 2 插件 |
| `DM_ARM_BUILD_PYTHON` | `OFF` | 仅预留；当前不会生成 Python 模块 |

不要仅根据 option 名称判断功能已经完成；以 [当前实现状态](#当前实现状态) 和实际 CMake targets 为准

---

## 已知限制

当前版本仍有以下限制：

1. **API 尚未冻结**：项目处于 `0.1.0-alpha`，类型和生命周期接口仍可能调整；
2. **仅固定六轴主链**：`DM_ARM_JOINTS_COUNT == 6`，夹爪尚未纳入统一控制闭环；
3. **无内置动力学**：只有模型前馈回调，尚无 Pinocchio target；
4. **无 ROS 2 插件**：暂不能直接作为 `ros2_control::SystemInterface` 加载；
5. **无 Python binding**；
6. **暂无仓库内 tests/tools**：接口稳定后再补充；
7. **不提供硬实时保证**：当前使用 `std::vector`、`std::function` 等通用容器，尚未完成实时内存审计；
8. **不自带调度线程**：调用方负责周期；
9. **无通用 dry-run 后端**：默认写门禁只阻止激活，不等价于完整仿真；
10. **反馈力矩仍需真机标定**：不能直接把所有反馈力矩异常解释为真实外力；
11. **默认参数不是标定结果**：mapping 和增益必须按真实机械臂调整；
12. **达妙后端面向当前串口 SDK**：不是通用 SocketCAN 后端

---

## 开发路线

### A：控制、安全与 Robot

当前已完成最小闭环，后续重点：

- 真机分级验证；
- 错误信息可读化；
- 接口稳定后增加 FakeBus 和自动化测试；
- 实时内存和周期抖动分析

### B：Pinocchio 运动学与动力学

优先级最高的下一主线：

1. URDF 加载和 Joint index 映射；
2. FK 与 Frame pose；
3. Jacobian；
4. `gravity(q)`；
5. 质量矩阵、非线性项、RNEA 和 ABA；
6. 重力补偿接入 `ModelFeedforwardFn`；
7. 负载与工具模型

### C：硬件后端

- 完成达妙后端真机验证；
- 明确兼容 USB-CAN 设备和协议；
- 视需求增加 SocketCAN、EtherCAT 或其他执行器后端

### D：ROS 2 / ros2_control

- 单独 ROS 2 适配包；
- `hardware_interface::SystemInterface`；
- Joint state/command interfaces；
- 与 JointTrajectoryController 对接；
- 生命周期和错误状态映射

### E：Python binding

- pybind11；
- NumPy 六维向量契约；
- Controller、Mapper、Dynamics 和 Robot 非实时接口；
- wheel 打包；
- 用于标定、数据分析和算法原型

详细路线与设计说明见 [`docs/`](docs/)

---

## 贡献

项目仍处于快速演进阶段，提交修改前建议先确认模块边界

### 开发约定

- C++17；
- 保持现有命名风格，例如 `Ctrller`；
- 周期路径使用明确错误码，不把常见运行错误改成异常；
- Core 不包含 ROS 2 消息或具体硬件 SDK 类型；
- Damiao 专用代码只放在对应后端；
- 不在多个模块重复维护电机型号或限制表；
- YAML 只在 configure 阶段读取；
- 不通过静默 clamp 掩盖明显错误；
- 新功能同步更新 README 和详细设计文档

### 建议提交范围

一个提交只处理一个主题，例如：

```text
feat: add Pinocchio model loading
fix: reject reversed Robot timestamps
docs: document Safety fault actions
refactor: remove duplicated Damiao limits
```

在测试体系加入前，Pull Request 至少应提供：

- 使用的编译器和系统版本；
- CMake 配置命令；
- 是否启用达妙后端；
- 构建结果；
- 真机相关修改的风险说明和验证步骤

---

## 许可证与第三方代码

项目自身代码采用 [MIT License](LICENSE)

仓库包含第三方代码：

- `third_lib/tl`：包含其自身 `COPYING` 文件；
- `third_lib/damiao`：达妙通信适配代码

项目主许可证不会自动覆盖第三方代码；公开发布、二次分发或商业使用前，应分别确认所有第三方组件的来源、许可证和再分发条件当前 `third_lib/damiao` 目录未发现独立许可证文件，这是正式公开发布前需要补充确认的事项

---

## 维护状态

当前仓库主要服务于 DM-Arm 平台重构和真实机械臂研发欢迎针对以下方向提交可复现的问题：

- 配置加载；
- Joint/Actuator 映射；
- Safety 边界；
- Robot 生命周期；
- 达妙通信兼容性；
- 动力学接口设计；
- ROS 2 和 Python 适配

报告真机问题时，请避免上传密钥或敏感设备信息，并至少提供：系统版本、编译器、CMake 参数、电机型号、通信设备类型、错误码和最小复现步骤
