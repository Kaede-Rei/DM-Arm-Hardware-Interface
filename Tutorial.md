# SerialArm-Core Tutorial

Tutorial 主要说明如何从一个刚克隆仓库的环境开始，逐步完成配置检查、离线验证、真机启动、阻抗控制、动力学前馈、ROS 2 接入以及新机械臂和新 Backend 的扩展

如果需要逐项查找类、函数、参数、返回值和错误码，请阅读 [API.md](API.md)

[toc]

---

## 1. SerialArm-Core 的使用层级

SerialArm-Core 的主要使用入口为 `Robot`，典型控制链如下

```text
Robot Profile
    |
    +-- Core YAML
    +-- Hardware Backend
    +-- Hardware YAML
    |
    v
HardwareLoader
    |
    v
MotorBus
    |
    +-------------------------------+
    |                               |
    v                               v
load_robot_cfg()                 Dynamics
    |                               |
    +---------------+---------------+
                    |
                    v
                  Robot
                    |
        +-----------+-----------+
        |           |           |
        v           v           v
      Safety      Mapper    JointCtrller
        |           |           |
        +-----------+-----------+
                    |
                    v
                 MotorBus
```

对于大多数应用开发，建议只直接操作以下对象

- `Robot Profile` 用于选择一套完整机器人实例
- `Robot` 用于 C++ 控制闭环
- `RobotSession` 用于 Python 控制闭环
- `Dynamics` 用于独立运动学和动力学计算
- ROS 2 Adapter 用于 ros2_control 和 MoveIt 2

只有开发新 Backend、调试 Safety 或验证关节映射时，才需要直接操作底层类

---

## 2. 准备环境

### 2.1. 基础依赖

Core 使用 C++17，当前必需依赖包括

- CMake 3.20+
- C++17 compiler
- yaml-cpp
- Eigen3
- Pinocchio
- GTest，用于构建测试

Python Binding 额外需要

- Python 3.10+
- pybind11
- NumPy
- scikit-build-core

ROS 2 Adapter 当前面向 ROS 2 Humble，并额外依赖 ros2_control、controller_manager、xacro、robot_state_publisher 等组件

MoveIt 2 只在运行 MoveIt 路径时需要

### 2.2. 克隆仓库

```bash
git clone https://github.com/Kaede-Rei/SerialArm-Core.git
cd SerialArm-Core
```

后续命令默认都在仓库根目录执行

---

## 3. 第一步：只构建 Core 并运行测试

第一次接触仓库时不要直接连接真机

先确认 Core、配置系统、Dynamics、Safety 和映射模块能够正常编译并通过契约测试

```bash
cmake -S src/serial_arm/core -B build/serial_arm_core \
  -DCMAKE_BUILD_TYPE=Release \
  -DSERIAL_ARM_BUILD_PYTHON=OFF \
  -DSERIAL_ARM_BUILD_TERMINAL=OFF

cmake --build build/serial_arm_core -j

ctest \
  --test-dir build/serial_arm_core \
  --output-on-failure
```

预期结果

```text
100% tests passed
```

如果这一步失败，不要继续真机流程

优先解决依赖、Pinocchio、yaml-cpp、Eigen3 或编译器问题

---

## 4. 第二步：构建 standalone 安装目录

Standalone 模式不依赖 ROS 2 runtime，适合优先验证 Core、Robot Profile、Hardware Backend、Dynamics 和配置文件

### 4.1. 构建 Core 与 Terminal

```bash
cmake -S src/serial_arm/core -B build/serial_arm_core \
  -DCMAKE_BUILD_TYPE=Release \
  -DSERIAL_ARM_BUILD_PYTHON=OFF \
  -DSERIAL_ARM_BUILD_TERMINAL=ON

cmake --build build/serial_arm_core -j

cmake --install build/serial_arm_core \
  --prefix install/standalone
```

### 4.2. 构建 Damiao Backend

```bash
cmake -S src/robot_supports/hardware/damiao \
  -B build/serial_arm_hardware_damiao \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$PWD/install/standalone"

cmake --build build/serial_arm_hardware_damiao -j

cmake --install build/serial_arm_hardware_damiao \
  --prefix install/standalone
```

### 4.3. 安装 Robot Profiles

```bash
cmake -S src/robot_supports/profiles \
  -B build/serial_arm_robot_profiles

cmake --install build/serial_arm_robot_profiles \
  --prefix install/standalone
```

### 4.4. 安装 DM-Arm resources

```bash
cmake -S src/robot_supports/robots/dm_arm/description \
  -B build/dm_arm_description

cmake --install build/dm_arm_description \
  --prefix install/standalone
```

### 4.5. 配置资源搜索路径

```bash
export SERIAL_ARM_RESOURCE_PATH="$PWD/install/standalone"
export LD_LIBRARY_PATH="$PWD/install/standalone/lib:/opt/openrobots/lib:${LD_LIBRARY_PATH:-}"
```

安装完成后至少应能找到以下文件

```text
install/standalone/bin/serial_arm_terminal
install/standalone/lib/libserial_arm_hardware_damiao.so
install/standalone/share/serial_arm_robot_profiles/config/robot_profiles.yaml
install/standalone/share/dm_arm_description/config/core/gray.yaml
install/standalone/share/dm_arm_description/config/hardware.yaml
install/standalone/share/dm_arm_description/model/...
```

---

## 5. 第三步：理解 Robot Profile

Robot Profile 的作用是给一个完整机器人实例命名，并把 Core 配置、Hardware Backend、Hardware 配置、URDF、Controllers 和可选 MoveIt 配置关联起来

当前 `dm_arm_gray` 的核心结构如下

```yaml
profiles:
  dm_arm_gray:
    core:
      package: dm_arm_description
      config: config/core/gray.yaml

    hardware:
      plugin: serial_arm_hardware_damiao
      config_package: dm_arm_description
      config: config/hardware.yaml

    description:
      package: dm_arm_description
      urdf: model/gray/urdf/dm_arm_no_gripper.urdf
      ros2_control_xacro: model/gray/urdf/dm_arm.ros2_control.xacro

    controllers:
      package: dm_arm_description
      config: config/ros2_controllers.yaml

    moveit:
      package: dm_arm_no_gripper
```

这里最重要的是先理解前三项

```text
core.config
hardware.plugin
hardware.config
```

它们最终解析为

```text
Core YAML
Hardware Backend shared library
Hardware Backend YAML
```

Standalone C++、Python 和 ROS 2 Adapter 都复用这套 Profile

Robot Profile 不属于 ROS 2，也不要求 ROS 2 runtime

---

## 6. 第四步：检查 Core YAML

以 `dm_arm_gray` 为例，Core YAML 主要由以下部分组成

```text
model
calibration
control
safety_policy
shutdown
```

### 6.1. 检查 model

```yaml
model:
  urdf_path: ../../model/gray/urdf/dm_arm_no_gripper.urdf
  joint_names: [joint1, joint2, joint3, joint4, joint5, joint6]
  base_frame: base_link
  tool_frame: tool0
  gravity: [0.0, 0.0, -9.81]
  gravity_scale: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
```

需要确认

- `joint_names` 顺序与实际受控关节顺序一致
- URDF 中存在这些 Joint
- `base_frame` 和 `tool_frame` 在 URDF 中存在
- URDF 中的 joint axis 正确
- URDF 中的位置、速度和 effort limit 合理
- Dynamics 所需 inertial 参数存在

如果惯性参数暂时没有完成辨识，可以先使用合法的 placeholder inertial 完成软件接入

此时 FK 和 Jacobian 仍可用于软件联调，但 gravity、mass matrix、inverse dynamics 等结果不能视为真实动力学结果

### 6.2. 检查 calibration

```yaml
calibration:
  joints:
    joint1:
      direction: 1.0
      pos_ratio: 1.0
      tor_ratio: 1.0
      joint_zero_offset: 0.0
      actuator_zero_offset: 0.0
```

参数含义

```text
direction
  Joint 与 Actuator 的方向关系，只允许 1 或 -1

pos_ratio
  actuator position delta / joint position delta

tor_ratio
  joint torque / actuator reported torque

joint_zero_offset
  Joint 侧零位偏置

actuator_zero_offset
  Actuator 侧零位偏置
```

如果电机 SDK 已经返回减速器输出端位置和力矩，`pos_ratio` 与 `tor_ratio` 通常可以保持 `1.0`

不要为了修正方向错误直接修改 URDF joint axis 和 Backend 单位转换

先明确问题属于机械模型、关节映射还是厂商协议层

### 6.3. 检查 runtime

```yaml
control:
  runtime:
    ctrl_frequency_hz: 200.0
    joint_acc_filter_alpha: 0.2
    write_enabled: false
    model_feedforward_mode: GRAVITY
    tracking_impedance_mode: COMPLIANT_TRACKING
```

第一次调试必须优先保持

```yaml
write_enabled: false
```

`Robot::activate()` 在 `write_enabled=false` 时会拒绝进入 ACTIVE，因此不会通过 `Robot` 真机闭环写执行器

在完成配置、方向、零位和限位检查前不要改成 `true`

---

## 7. 第五步：构建 Python Binding 并做配置检查

Python Binding 适合快速检查配置和进行离线工具开发

### 7.1. 构建 wheel

```bash
cd src/serial_arm/core/python

python -m pip install build

python -m build --wheel

python -m pip install --force-reinstall dist/serial_arm-*.whl

cd ../../../..
```

### 7.2. 运行 Python Terminal 的 check-only

```bash
export SERIAL_ARM_RESOURCE_PATH="$PWD/install/standalone"
export LD_LIBRARY_PATH="$PWD/install/standalone/lib:/opt/openrobots/lib:${LD_LIBRARY_PATH:-}"

python src/serial_arm/core/app/serial_arm_terminal.py \
  --robot-profile dm_arm_gray \
  --check-only
```

这一步应该用于检查

- Profile 是否可解析
- Core YAML 是否存在
- Hardware Backend 是否能加载
- Hardware YAML 是否可读取
- HardwareCapabilities 是否能参与 Safety limit 解析
- Dynamics 配置是否有效

如果 check-only 失败，优先修复配置，不要绕过错误进入真机

---

## 8. 第六步：使用 Python Dynamics

这一节完全不需要进入机械臂 ACTIVE 状态

目标是验证 URDF、Joint 顺序、FK、Jacobian 和重力项

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

cfg = serial_arm.load_robot_cfg(
    str(config_file),
    str(hardware_plugin),
    str(hardware_config),
)

dynamics = serial_arm.Dynamics()
dynamics.configure(cfg.dynamics)

n = len(cfg.joint_names)

state = serial_arm.JointState()
state.pos = np.zeros(n, dtype=np.float64)
state.vel = np.zeros(n, dtype=np.float64)
state.tor = np.zeros(n, dtype=np.float64)

acc = np.zeros(n, dtype=np.float64)
ref_acc = np.zeros(n, dtype=np.float64)

dynamics.update_state(state, acc, ref_acc)

print("gravity =", dynamics.gravity)
print("gravity_compensation =", dynamics.gravity_compensation)
print("mass_matrix =")
print(dynamics.mass_matrix)
print("tool_pose =")
print(dynamics.tool_pose)
print("tool_jacobian =")
print(dynamics.tool_jacobian)
```

预期结果

- `gravity` 长度等于关节数量
- `mass_matrix` 维度为 `N x N`
- `tool_pose` 为 `4 x 4`
- `tool_jacobian` 为 `6 x N`
- 所有结果应为有限值

如果 `FRAME_NOT_FOUND`，检查 `base_frame` 和 `tool_frame`

如果 `JOINT_NOT_FOUND`，检查 `joint_names`

如果结果数值明显异常，先检查 URDF joint axis、origin、mass 和 inertia

---

## 9. 第七步：理解五种阻抗模式

不要把五种模式理解成五套完全独立的控制器

它们共享统一的 MIT 语义

```text
position
velocity
torque
kp
kd
```

差异主要在于参考位置如何产生，以及使用哪套 `kp` 和 `kd`

| 模式 | 参考位置 | 是否接受 `set_cmd` | 典型用途 |
| --- | --- | --- | --- |
| `RIGID_HOLD` | 切换模式时的当前实测位置 | 否 | 高刚度定点保持 |
| `RIGID_TRACKING` | 上层命令 | 是 | 常规刚性轨迹跟踪 |
| `COMPLIANT_HOLD` | 切换模式时的当前实测位置 | 否 | 低刚度定点保持 |
| `COMPLIANT_DRAG` | 每周期当前实测位置 | 否 | 手动拖拽与柔性示教 |
| `COMPLIANT_TRACKING` | 上层命令 | 是 | 柔性轨迹跟踪 |

模式切换后，Core 会清除原来的 tracking command，并以当前实测状态重新建立参考和 Safety 命令历史

因此正确流程是

```text
set_impedance_mode(TRACKING)
        |
        v
持续发送 set_cmd(...)
        |
        v
持续执行 cycle()
```

不要先发送 `set_cmd()` 再切换 tracking mode

---

## 10. 第八步：写 C++ Robot 控制闭环

下面示例展示真正的 Core 使用路径

它不是 ROS 2 节点，也不会自动创建控制线程

调用者负责按 `ctrl_frequency_hz` 调用 `cycle()`

### 10.1. 最小 CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.20)
project(serial_arm_example LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

find_package(serial_arm_core CONFIG REQUIRED)

add_executable(robot_example main.cpp)

target_link_libraries(robot_example
  PRIVATE
    serial_arm::core
    serial_arm::config
    serial_arm::robot
    serial_arm::dynamics
)
```

### 10.2. 完整 main.cpp

```cpp
#include <chrono>
#include <iostream>
#include <thread>

#include <tl/expected.hpp>

#include "serial_arm/config/config.hpp"
#include "serial_arm/config/robot_profile.hpp"
#include "serial_arm/dynamics/dynamics.hpp"
#include "serial_arm/hardware/hardware_loader.hpp"
#include "serial_arm/robot.hpp"

using namespace serial_arm;

int main() {
    auto profile_result = load_robot_profile_core("dm_arm_gray");

    if(!profile_result) {
        std::cerr << "load_robot_profile_core failed\n";
        return 1;
    }

    const RobotProfileCore profile = profile_result.value();

    HardwareLoader hardware_loader;

    auto bus_result = hardware_loader.load(
        profile.hardware_plugin,
        profile.hardware_config_path);

    if(!bus_result) {
        std::cerr << "HardwareLoader::load failed\n";
        return 1;
    }

    std::unique_ptr<MotorBus> bus = std::move(bus_result.value());

    auto cfg_result = load_robot_cfg(
        profile.core_config_path,
        bus->capabilities());

    if(!cfg_result) {
        std::cerr << "load_robot_cfg failed: "
                  << cfg_result.error().message
                  << "\n";
        return 1;
    }

    RobotCfg cfg = cfg_result.value();

    Dynamics dynamics;

    if(auto result = dynamics.configure(cfg.dynamics); !result) {
        std::cerr << "Dynamics::configure failed\n";
        return 1;
    }

    const std::size_t joints_count = cfg.joint_names.size();

    ModelFeedforwardFn feedforward =
        [&dynamics, joints_count](
            ModelFeedforwardMode mode,
            const JointState& state,
            const JointVector& acc,
            const JointVector& ref_acc,
            double) -> tl::expected<JointVector, ModelFeedforwardErr>
        {
            if(mode == ModelFeedforwardMode::NONE) {
                return JointVector(joints_count, 0.0);
            }

            auto update_result = dynamics.update(
                state,
                acc,
                ref_acc);

            if(!update_result) {
                return tl::make_unexpected(
                    ModelFeedforwardErr::COMPUTE_FAILED);
            }

            switch(mode) {
                case ModelFeedforwardMode::GRAVITY:
                    return dynamics.get_gravity_compensation();

                case ModelFeedforwardMode::FULL_INVERSE_DYNAMICS:
                    return dynamics.get_inverse_dynamics();

                case ModelFeedforwardMode::NONE:
                    return JointVector(joints_count, 0.0);
            }

            return tl::make_unexpected(
                ModelFeedforwardErr::INVALID_MODE);
        };

    Robot robot;

    if(auto result = robot.configure(
        cfg,
        std::move(bus),
        feedforward); !result)
    {
        std::cerr << "Robot::configure failed\n";
        return 1;
    }

    if(!cfg.runtime.write_enabled) {
        std::cout
            << "Configuration is valid, but write_enabled=false\n";
        return 0;
    }

    if(auto result = robot.activate(); !result) {
        std::cerr << "Robot::activate failed\n";
        return 1;
    }

    if(auto result = robot.set_impedance_mode(
        JointImpedanceMode::RIGID_TRACKING); !result)
    {
        std::cerr << "set_impedance_mode failed\n";
        robot.force_deactivate();
        return 1;
    }

    JointVector target = robot.get_joint_state().pos;

    target[0] += 0.05;

    const auto period = std::chrono::duration<double>(
        1.0 / cfg.runtime.ctrl_frequency_hz);

    for(int i = 0; i < 400; ++i) {
        const auto now = Robot::Clock::now();

        if(auto result = robot.set_cmd(
            JointPosCmd{target},
            now); !result)
        {
            std::cerr << "Robot::set_cmd failed\n";
            robot.force_deactivate();
            return 1;
        }

        auto cycle_result = robot.cycle(now);

        if(!cycle_result) {
            std::cerr << "Robot::cycle failed\n";
            robot.force_deactivate();
            return 1;
        }

        const RobotCycleOutput& output =
            cycle_result.value();

        std::cout
            << "q0=" << output.joint_state.pos[0]
            << " tau_ff0=" << output.model_feedforward[0]
            << " dt=" << output.dt
            << "\n";

        std::this_thread::sleep_for(period);
    }

    if(auto result = robot.deactivate(); !result) {
        std::cerr << "Robot::deactivate failed\n";
        return 1;
    }

    return 0;
}
```

### 10.3. 为什么 tracking 示例每周期都调用 set_cmd

`cmd_timeout_s` 是独立的 Safety 约束

只调用一次 `set_cmd()` 后长时间持续 `cycle()` 会触发命令超时

对于持续 tracking，应让上层规划器、轨迹生成器或控制任务以稳定频率刷新目标

---

## 11. 第九步：实现三种常见跟踪命令

### 11.1. 只给位置

```cpp
JointPosCmd cmd;
cmd.pos = target;

robot.set_cmd(cmd);
```

Core 最终构造

```text
pos = target
vel = 0
tor = model feedforward
kp  = 当前 tracking mode 对应 kp
kd  = 当前 tracking mode 对应 kd
```

### 11.2. 给位置和速度

```cpp
JointPosVelCmd cmd;
cmd.pos = target_pos;
cmd.vel = target_vel;

robot.set_cmd(cmd);
```

适合轨迹生成器已经提供关节速度参考的情况

### 11.3. 给位置、速度和附加力矩

```cpp
JointPosVelTorCmd cmd;
cmd.pos = target_pos;
cmd.vel = target_vel;
cmd.tor = extra_torque;

robot.set_cmd(cmd);
```

这里的 `tor` 是附加关节力矩

如果同时启用了模型前馈，最终关节力矩为

```text
command torque + model feedforward torque
```

不要在上层已经手动加入重力补偿后又启用 `GRAVITY`，否则会发生重复补偿

---

## 12. 第十步：实现柔性拖拽

柔性拖拽使用

```cpp
robot.set_impedance_mode(
    JointImpedanceMode::COMPLIANT_DRAG);
```

进入该模式后不再发送 `set_cmd()`

控制循环只需要持续执行

```cpp
while(running) {
    auto result = robot.cycle();

    if(!result) {
        break;
    }
}
```

`COMPLIANT_DRAG` 每周期把当前实测位置作为新的参考位置

因此其核心效果不是追踪一个固定位置，而是避免建立明显的位置回拉误差

实际拖拽手感主要由以下参数决定

```yaml
control:
  controller:
    compliant_drag:
      kp:
        joint1: 0.0
      kd:
        joint1: 0.08
```

以及

```yaml
control:
  runtime:
    model_feedforward_mode: GRAVITY
```

建议调试顺序

1. 先验证执行器反馈方向与关节方向
2. 再启用低 `kp` 和低 `kd`
3. 再验证重力模型方向
4. 最后逐轴调整 `gravity_scale`

不要用大 `kp` 的所谓拖拽模式抵消重力模型问题

---

## 13. 第十一步：调试重力补偿

### 13.1. 开启 GRAVITY

Core YAML

```yaml
control:
  runtime:
    model_feedforward_mode: GRAVITY
```

Dynamics 配置

```yaml
model:
  gravity_scale:
    - 1.0
    - 1.0
    - 1.0
    - 1.0
    - 1.0
    - 1.0
```

### 13.2. 运行时修改 gravity_scale

C++

```cpp
Dynamics dynamics;

dynamics.configure(cfg.dynamics);

dynamics.set_gravity_scale({
    1.0,
    0.9,
    0.85,
    1.0,
    1.0,
    1.0,
});
```

Python `RobotSession`

```python
arm.set_gravity_scale(
    np.array(
        [1.0, 0.9, 0.85, 1.0, 1.0, 1.0],
        dtype=np.float64,
    )
)
```

### 13.3. 推荐验证方式

不要一开始就在整机拖拽下调整所有关节

建议逐轴进行

```text
固定其余关节
    |
    v
降低目标关节 kp
    |
    v
观察自然下坠方向
    |
    v
开启 gravity compensation
    |
    v
检查补偿方向是否正确
    |
    v
从较小 gravity_scale 向上增加
```

如果补偿方向错误，优先检查

- URDF joint axis
- `direction`
- 电机力矩正方向
- `tor_ratio`
- 重力坐标方向
- inertial 参数

不要先靠负 `gravity_scale` 修正体系错误

当前 `gravity_scale` 的有效范围为 `[0, 1]`

---

## 14. 第十二步：使用 Python RobotSession

`RobotSession` 适合任务层、数据采集工具和快速实验

控制周期由 C++ 工作线程维护，Python 不需要自己写 200 Hz `cycle()` 循环

### 14.1. 创建会话

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

arm = serial_arm.RobotSession(
    config_file,
    hardware_plugin,
    hardware_config,
)
```

### 14.2. `write_enabled` 在 RobotSession 中的行为

`RobotSession` 会先加载真实 Backend 以获得 `HardwareCapabilities` 并完成 Core 配置解析

当 Core YAML 为

```yaml
write_enabled: false
```

Session 会把实际控制 Backend 替换成内部 `MockMotorBus`，因此 `start()` 可以用于离线控制链验证，不会使能真实执行器

当 Core YAML 为

```yaml
write_enabled: true
```

Session 才会把真实 Hardware Backend 交给 `Robot`

真机模式必须完成方向、零位、限位、Backend 和 Safety 验证后再调用

```python
arm.start()
```

### 14.3. 切换刚性跟踪并移动

```python
arm.set_impedance_mode(
    serial_arm.JointImpedanceMode.RIGID_TRACKING
)

snapshot = arm.snapshot

if not snapshot.valid:
    raise RuntimeError(snapshot.last_error)

target = snapshot.cycle.joint_state.pos.copy()

target[0] += 0.05

arm.move_to(
    target,
    speed_scale=0.15,
)
```

### 14.4. 保持当前位置

```python
arm.hold_current()
```

### 14.5. 柔性拖拽

```python
arm.set_impedance_mode(
    serial_arm.JointImpedanceMode.COMPLIANT_DRAG
)
```

### 14.6. 安全退出

```python
arm.stop()
```

推荐使用上下文管理器

```python
with serial_arm.RobotSession(
    config_file,
    hardware_plugin,
    hardware_config,
) as arm:
    arm.set_impedance_mode(
        serial_arm.JointImpedanceMode.COMPLIANT_DRAG
    )

    snapshot = arm.snapshot

    print(snapshot.cycle.joint_state.pos)
```

---

## 15. 第十三步：真机 Bringup

在把 `write_enabled` 改成 `true` 前完成以下检查

### 15.1. 机械侧

- 机械臂安装可靠
- 无人员位于危险运动区域
- 末端负载已知
- 急停或硬件断电路径可用
- 初始姿态不会在使能瞬间发生机构干涉

### 15.2. 模型侧

- URDF joint axis 正确
- joint order 正确
- position limit 正确
- velocity limit 正确
- effort limit 正确
- base 和 tool frame 正确

### 15.3. 映射侧

- direction 正确
- pos_ratio 正确
- tor_ratio 正确
- joint zero 正确
- actuator zero 正确

### 15.4. Backend 侧

当前 DM-Arm 示例配置

```yaml
damiao:
  serial_port: /dev/ttyACM0
  baudrate: 921600
  feedback_timeout_s: 0.05
  activation_retries: 3
  startup_read_cycles: 5
```

还要逐轴核对

```yaml
actuators:
  joint1:
    name: actuator1
    motor_id: 1
    master_id: 0
    motor_type: DM4340
```

### 15.5. 小幅单关节验证

建议先用

```text
0.02 rad
0.05 rad
0.10 rad
```

逐步验证方向、速度、跟踪误差和停止行为

---

## 16. 第十四步：ROS 2 display

当 standalone 配置和 URDF 已经通过检查后，再进入 ROS 2

```bash
colcon build --symlink-install

source install/setup.bash
```

只显示模型

```bash
ros2 launch serial_arm_ros2_control \
  display.launch.py \
  robot_profile:=dm_arm_gray
```

这一阶段重点检查

- mesh 是否正确
- Joint 方向是否正确
- TF 树是否正确
- base 和 tool frame 是否正确
- 机器人模型是否与实物一致

display 正确不代表真机映射一定正确

URDF joint axis 和 actuator direction 属于不同层

---

## 17. 第十五步：ROS 2 hardware

模型验证完成后再启动 ros2_control hardware

```bash
ros2 launch serial_arm_ros2_control \
  hardware.launch.py \
  robot_profile:=dm_arm_gray
```

该路径主要建立

```text
robot_state_publisher
controller_manager
joint_state_broadcaster
joint_trajectory_controller
SerialArm SystemInterface
```

它不依赖 MoveIt

先确认

```bash
ros2 control list_hardware_interfaces
```

再确认

```bash
ros2 control list_controllers
```

最后才发送小幅 JointTrajectoryController 目标

---

## 18. 第十六步：MoveIt 2

MoveIt 属于上层规划能力，不属于 Core 必选依赖

只有 Profile 定义了

```yaml
moveit:
  package: dm_arm_no_gripper
```

才运行

```bash
ros2 launch serial_arm_ros2_control \
  moveit.launch.py \
  robot_profile:=dm_arm_gray
```

推荐顺序始终保持

```text
Core test
    ->
Profile / config
    ->
URDF / Dynamics
    ->
Backend
    ->
小幅 Joint control
    ->
ros2_control
    ->
MoveIt
```

不要用 MoveIt 来替代底层方向和限位验证

---

## 19. 第十七步：新增一台机械臂

新增机械臂时不要复制整个仓库并修改 Core

新增内容应主要进入 `robot_supports`

推荐结构

```text
src/
├── serial_arm/
│   ├── core/
│   └── bringup/
│
└── robot_supports/
    ├── hardware/
    │   └── <backend>/
    │
    ├── profiles/
    │   └── config/
    │       └── robot_profiles.yaml
    │
    └── robots/
        └── <robot_name>/
            ├── description/
            │   ├── config/
            │   │   ├── core/
            │   │   │   └── default.yaml
            │   │   ├── hardware.yaml
            │   │   └── ros2_controllers.yaml
            │   └── model/
            │       └── ...
            │
            └── moveit_config/
```

### 19.1. 先放 URDF

至少保证

- 受控 Joint 名称稳定
- joint axis 正确
- joint limit 合法
- link inertial 合法
- base frame 确定
- tool frame 确定

### 19.2. 创建 Core YAML

复制结构，不复制数值

必须重新确认

- joint names
- calibration
- gains
- Safety policy
- shutdown pose
- dynamics frames
- gravity scale

### 19.3. 创建 Profile

```yaml
profiles:
  my_arm:
    core:
      package: my_arm_description
      config: config/core/default.yaml

    hardware:
      plugin: my_hardware_backend
      config_package: my_arm_description
      config: config/hardware.yaml

    description:
      package: my_arm_description
      urdf: model/my_arm.urdf
      ros2_control_xacro: model/my_arm.ros2_control.xacro

    controllers:
      package: my_arm_description
      config: config/ros2_controllers.yaml
```

MoveIt 可以后加

不要把 MoveIt 当作新机械臂接入 Core 的前置条件

---

## 20. 第十八步：新增 Hardware Backend

只有厂商协议发生变化时才应该新增 Backend

Core 不应该感知 Damiao、RealMan 或其他厂商协议

新 Backend 必须实现 `MotorBus`

```cpp
class MyMotorBus final : public serial_arm::MotorBus {
public:
    tl::expected<void, MotorBusErr>
    configure(const std::string& config_path) override;

    tl::expected<void, MotorBusErr>
    connect() override;

    tl::expected<ActuatorState, MotorBusErr>
    read() override;

    tl::expected<void, MotorBusErr>
    activate() override;

    tl::expected<void, MotorBusErr>
    write(const ActuatorCtrlCmd& cmd) override;

    tl::expected<void, MotorBusErr>
    stop() override;

    tl::expected<void, MotorBusErr>
    deactivate() override;

    tl::expected<void, MotorBusErr>
    recover() override;

    const HardwareCapabilities&
    capabilities() const noexcept override;

    void cleanup() noexcept override;

    std::size_t size() const noexcept override;
};
```

Backend 对 Core 的状态输出必须统一为

```text
position -> rad
velocity -> rad/s
torque   -> N*m
```

Backend 从 Core 接收完整控制语义

```text
position
velocity
torque
kp
kd
```

如果厂商 SDK 使用编码器计数、电流、转速 RPM 或其他单位，应在 Backend 内转换

不要让这些厂商单位进入 Core

---

## 21. 第十九步：调试 Safety

Safety 不是最后才加的保护层

它参与每个 ACTIVE 控制周期

重点参数

```yaml
safety_policy:
  position_margin: 0.0
  cmd_vel_scale: 1.0
  state_vel_scale: 1.0
  max_dt_s: 0.02
  state_timeout_s: 0.05
  cmd_timeout_s: 0.10
  require_all_actuators_online: true
  require_all_actuators_enabled: true
  reject_motor_error: true
  require_continuous_cmd: false
```

建议首次真机时主动收窄，而不是放宽限制

例如

```yaml
cmd_vel_scale: 0.2
state_vel_scale: 0.5
```

前提是这些值满足当前配置校验规则并且不会试图放宽底层物理限制

Safety 最终限制来自多个来源的交集

```text
URDF limits
    ∩
HardwareCapabilities
    ∩
Calibration mapping
    ∩
Safety policy
```

Safety policy 只能收窄限制，不能扩张物理能力

---

## 22. 第二十步：处理 FAULT

Robot 进入 `FAULT` 后不要继续发送正常 tracking command

先读取

```cpp
const auto& fault = robot.get_last_fault();
```

确认故障来源

```text
RobotErr
MotorBusErr
JointActuatorMapErr
JointCtrllerErr
SafetyFault
ModelFeedforwardErr
```

如果 Robot 正在执行 fault hold，应持续调用

```cpp
robot.maintain_fault_hold();
```

恢复流程应按故障性质选择

### 22.1. 返回故障刚性保持

```cpp
robot.return_to_fault_rigid_hold();
```

### 22.2. 人工进入受限柔性恢复

只有配置允许、重力模型已验证并且当前故障类型允许时才能使用

```cpp
robot.enter_fault_compliant_recovery();
```

### 22.3. 清除故障

```cpp
robot.clear_fault();
```

`clear_fault()` 不是无条件复位

它要求当前状态重新可信，关节速度足够低，并且已经积累连续合法的恢复周期

如果条件不满足，可能返回 `FAULT_RECOVERY_NOT_ALLOWED`

如果目标只是无条件退出到失能状态，使用

```cpp
robot.force_deactivate();
```

---

## 23. 推荐的完整开发顺序

新机械臂或新 Backend 建议严格按以下顺序推进

```text
1. Core 编译
2. Core tests
3. URDF
4. Robot Profile
5. Core YAML
6. Hardware YAML
7. HardwareCapabilities
8. config check
9. Dynamics FK / Jacobian
10. Safety limit
11. Joint / Actuator mapping
12. write_enabled=true
13. 单关节小幅运动
14. RIGID_HOLD
15. RIGID_TRACKING
16. COMPLIANT_HOLD
17. COMPLIANT_DRAG
18. gravity compensation
19. COMPLIANT_TRACKING
20. ros2_control
21. MoveIt
22. 上层任务
```

这个顺序的目的不是增加流程，而是让每个错误都能在最小范围内定位

---

## 24. 常见问题定位

### Profile 找不到

检查

```bash
echo "$SERIAL_ARM_RESOURCE_PATH"
```

确认

```text
install/standalone/share/serial_arm_robot_profiles/config/robot_profiles.yaml
```

存在

如果显式传入 `--profile-file`，resolver 只使用该文件，不会自动 fallback 到其他 Profile 文件

### Hardware Backend 找不到

检查

```bash
ls install/standalone/lib/libserial_arm_hardware_damiao.so
```

再检查

```bash
echo "$LD_LIBRARY_PATH"
```

### `Robot::activate()` 返回 `WRITE_DISABLED`

这是预期保护行为

修改 Core YAML

```yaml
write_enabled: true
```

前提是已经完成真机前检查

### `set_cmd()` 返回 `CMD_NOT_ALLOWED_IN_MODE`

当前模式不是 tracking mode

切换为

```cpp
robot.set_impedance_mode(
    JointImpedanceMode::RIGID_TRACKING);
```

或者

```cpp
robot.set_impedance_mode(
    JointImpedanceMode::COMPLIANT_TRACKING);
```

### tracking 运行一段时间后进入 FAULT

优先检查 `cmd_timeout_s`

持续 tracking 必须持续刷新 `set_cmd()`

### `Dynamics::get_frame_pose()` 返回 `FRAME_NOT_FOUND`

检查请求 frame 是否存在于 URDF

并检查 `base_frame` 是否正确

### 重力补偿方向错误

不要直接放大增益

依次检查

```text
URDF joint axis
direction
tor_ratio
电机力矩正方向
gravity vector
inertial
```

### 拖拽仍然很硬

检查

```yaml
compliant_drag.kp
compliant_drag.kd
```

再检查是否存在机械摩擦、减速器阻力或电机自身控制器刚度

不要把所有问题都归因于 Core 阻抗参数

---

## 25. 下一步应该读什么

需要写 C++ 控制程序时，直接阅读 [API.md](API.md) 中的 `Robot`、`JointCmd` 和错误处理章节

需要写 Python 实验脚本时，直接阅读 [API.md](API.md) 中的 `RobotSession` 章节

需要开发运动学或控制算法时，直接阅读 [API.md](API.md) 中的 `Dynamics` 章节

需要开发新电机驱动时，直接阅读 [API.md](API.md) 中的 `MotorBus` 与 `HardwareLoader` 章节

需要接 ROS 2 时，先完成本教程的 standalone 与真机小幅验证，再进入 `serial_arm_ros2_control`
