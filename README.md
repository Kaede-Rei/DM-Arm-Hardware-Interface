# DM-Arm Hardware Interface

DM-Arm 的共享 C++ 平台：

```text
JointCtrller
+ JointActuatorMapper
+ Safety
+ Robot lifecycle/cycle
+ MotorBus
+ DamiaoMotorBus
+ yaml-cpp config
```

动力学、ROS 2 和 Python binding 按独立主线继续接入同一个 `Robot`

## 当前 targets

```text
dm_arm::core      types / JointCtrller / Mapper / Safety
dm_arm::config    yaml-cpp 配置加载
dm_arm::robot     生命周期、统一 cycle、FAULT 处理
dm_arm::damiao    可选达妙后端
```

## 依赖

Ubuntu 22.04：

```bash
sudo apt update
sudo apt install libyaml-cpp-dev
```

## Core + Robot 构建

```bash
cmake -S . -B build-core \
  -DCMAKE_BUILD_TYPE=Release \
  -DDM_ARM_BUILD_DAMIAO=OFF

cmake --build build-core -j
cmake --install build-core --prefix "$PWD/install-core"
```

外部项目：

```cmake
find_package(dm_arm_hardware_interface REQUIRED CONFIG)

target_link_libraries(your_target PRIVATE
    dm_arm::robot
)
```

## Damiao 构建

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DDM_ARM_BUILD_DAMIAO=ON

cmake --build build -j
cmake --install build --prefix "$PWD/install"
```

外部项目按需链接：

```cmake
target_link_libraries(your_target PRIVATE
    dm_arm::robot
    dm_arm::damiao
)
```

## 使用顺序

```cpp
auto cfg_result = dm_arm::load_robot_cfg("config/dm_arm.yaml");

auto bus = std::make_unique<dm_arm::DamiaoMotorBus>();
bus->configure(cfg_result->damiao);

dm_arm::Robot robot;
robot.configure(cfg_result.value(), std::move(bus));
robot.activate();

while(robot.get_state() == dm_arm::RobotState::ACTIVE) {
    auto output = robot.cycle();
    if(!output) break;
}

robot.deactivate();
```

`MotorBus` 后端必须在注入 `Robot` 前完成其专用 `configure()`

## 真机门禁

默认配置：

```yaml
runtime:
  write_enabled: false
  model_feedforward_mode: NONE
```

没有确认电机 ID、型号、方向、零位、限位和急停条件前，不要改为 `true`

