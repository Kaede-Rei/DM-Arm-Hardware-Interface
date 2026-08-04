# SerialArm-Core Tutorial

本文是 SerialArm-Core 的完整教程；项目首页见 [README.md](README.md)，接口级说明见 [API.md](API.md)

## 环境与依赖

SerialArm-Core 以 C++17 为核心，当前主要在 ROS 2 Humble 环境中使用；Core 依赖：

- CMake
- C++17 compiler
- yaml-cpp
- Eigen3
- Pinocchio
- tl::expected vendored header

ROS 2 Adapter 额外依赖：

- rclcpp / rclcpp_lifecycle
- hardware_interface
- controller_manager
- pluginlib
- realtime_tools
- xacro
- robot_state_publisher
- joint_state_broadcaster
- JointTrajectoryController

MoveIt 2 只在 `moveit.launch.py` 或明确使用 MoveIt 的路径中需要

## 编译

### ROS 2 workspace

常规 ROS 2 workspace：

```bash
colcon build --symlink-install
source install/setup.bash
```

### 只构建 Core

不使用 ROS 2 时，可以只构建 Core 并安装为普通 CMake package：

```bash
cmake -S src/serial_arm/core -B build/serial_arm_core \
  -DSERIAL_ARM_BUILD_PYTHON=OFF \
  -DSERIAL_ARM_BUILD_TERMINAL=OFF
cmake --build build/serial_arm_core
ctest --test-dir build/serial_arm_core --output-on-failure
cmake --install build/serial_arm_core --prefix install/serial_arm_core
```

下游项目导入：

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

运行下游项目前，确保 `serial_arm_coreConfig.cmake` 所在路径进入 `CMAKE_PREFIX_PATH`，并且 Pinocchio、yaml-cpp、Eigen3 可被 CMake 找到

### Standalone C++ 使用

终端工具是纯 C++ 调试入口，不依赖 ROS 2 runtime；使用 `--robot-profile dm_arm_gray` 时，仍需要安装 Core、Damiao Backend、Robot Profiles 和 DM-Arm resources；即使 `runtime.write_enabled: false`，Core 也会加载 Backend library 获取 `HardwareCapabilities`

```bash
cmake -S src/serial_arm/core -B build/serial_arm_core \
  -DCMAKE_BUILD_TYPE=Release \
  -DSERIAL_ARM_BUILD_PYTHON=OFF \
  -DSERIAL_ARM_BUILD_TERMINAL=ON
cmake --build build/serial_arm_core -j
cmake --install build/serial_arm_core --prefix install/standalone

cmake -S src/robot_supports/hardware/damiao -B build/serial_arm_hardware_damiao \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$PWD/install/standalone"
cmake --build build/serial_arm_hardware_damiao -j
cmake --install build/serial_arm_hardware_damiao --prefix install/standalone

cmake -S src/robot_supports/profiles -B build/serial_arm_robot_profiles
cmake --install build/serial_arm_robot_profiles --prefix install/standalone

cmake -S src/robot_supports/robots/dm_arm/description -B build/dm_arm_description
cmake --install build/dm_arm_description --prefix install/standalone

export SERIAL_ARM_RESOURCE_PATH="$PWD/install/standalone"
export LD_LIBRARY_PATH="$PWD/install/standalone/lib:/opt/openrobots/lib:${LD_LIBRARY_PATH:-}"
./install/standalone/bin/serial_arm_terminal --robot-profile dm_arm_gray
```

安装后的关键布局：

```text
install/standalone/bin/serial_arm_terminal
install/standalone/lib/libserial_arm_hardware_damiao.so
install/standalone/share/serial_arm_robot_profiles/config/robot_profiles.yaml
install/standalone/share/dm_arm_description/config/core/gray.yaml
install/standalone/share/dm_arm_description/config/hardware.yaml
install/standalone/share/dm_arm_description/model/...
```

它适合验证 Core config、ModelLoader、Dynamics、Safety、Joint / Actuator Mapping、HardwareLoader 和 Hardware Backend lifecycle；真机模式仍需要设备权限；Standalone 流程不需要 `source /opt/ros/...` 或 `source install/setup.bash`

如果不使用 profile，也可以显式传入 Core config、Backend plugin 和 hardware config：

```bash
./build/serial_arm_core/serial_arm_terminal \
  --config install/dm_arm_description/share/dm_arm_description/config/core/gray.yaml \
  --hardware-plugin serial_arm_hardware_damiao \
  --hardware-config install/dm_arm_description/share/dm_arm_description/config/hardware.yaml
```

ROS 2 / colcon 场景单独使用 workspace overlay：

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch serial_arm_ros2_control display.launch.py robot_profile:=dm_arm_gray
```

### Python

Python Binding 提供 `RobotSession` 和 Python terminal；它同样不要求 ROS 2 控制链路，profile 模式复用 C++ Core resolver：

```bash
source install/setup.bash
cd src/serial_arm/core/python
python -m build --wheel
python -m pip install --force-reinstall dist/serial_arm-*.whl
```

无真机检查：

```bash
python ../app/serial_arm_terminal.py --robot-profile dm_arm_gray --check-only
```

启动 Python terminal：

```bash
python ../app/serial_arm_terminal.py --robot-profile dm_arm_gray
```

显式路径模式：

```bash
python ../app/serial_arm_terminal.py \
  --config ../../../install/dm_arm_description/share/dm_arm_description/config/core/gray.yaml \
  --hardware-plugin serial_arm_hardware_damiao \
  --hardware-config ../../../install/dm_arm_description/share/dm_arm_description/config/hardware.yaml
```

## Robot Profile

Robot Profile 把 Core config、Hardware config、Description、Controllers 和可选 MoveIt config 聚合为一个可启动实例

## Robot Profile 与 ROS 2 的关系

Robot Profile 属于 SerialArm-Core，不是 ROS 2 Profile、ros2_control Profile 或 MoveIt Profile；Native C++、Python 和 ROS 2 Adapter 共享同一份 Core / Hardware profile contract：

- `core.package + core.config`
- `hardware.plugin`
- `hardware.config_package + hardware.config`

ROS 2 Adapter 只是在同一个 profile 上额外读取：

- `description`
- `controllers`
- optional `moveit`

因此 `serial_arm_terminal --robot-profile dm_arm_gray` 和 Python terminal 的 `--robot-profile dm_arm_gray` 不要求 ROS 2 runtime；不需要 `rclcpp`、`controller_manager`、ros2_control、MoveIt 或 ament index；但具体 profile 仍需要对应 Robot resources 和 Hardware Backend library 可用；例如 `dm_arm_gray` 需要 SerialArm Core、DM-Arm Robot Resources、`serial_arm_hardware_damiao` shared library 和对应 config；即使 `runtime.write_enabled: false` 不连接、不使能、不向真实执行器写入，Core 仍需要 Backend 提供 `HardwareCapabilities`

Core resolver 的搜索入口包括显式 `--profile-file`、`SERIAL_ARM_RESOURCE_PATH`、当前工作目录、可执行文件附近路径以及编译/安装时记录的 resource root；Linux 下 `SERIAL_ARM_RESOURCE_PATH` 使用 `:` 分隔多个 resource root：

```bash
export SERIAL_ARM_RESOURCE_PATH=/path/to/resource_a:/path/to/resource_b
```

多个 resource root 可以各自提供 `robot_profiles.yaml`；SerialArm 会按搜索顺序查找请求的 profile；第一份存在但不包含目标 profile 的文件不会阻止后续 resource root 继续搜索；显式 `--profile-file <path>` 只使用该文件，不会 fallback 到其他资源目录

最小 profile：

```yaml
profiles:
  my_arm:
    core:
      package: my_arm_description
      config: config/core/default.yaml
    hardware:
      plugin: serial_arm_hardware_damiao
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

`moveit` 字段是 optional；没有 `moveit` 时：

- profile loader 成功
- display launch 可用
- hardware launch 可用
- ros2_control `SystemInterface` 可用

只有 `moveit.launch.py` 会要求：

```yaml
moveit:
  package: my_arm_moveit_config
```

## Dynamics

Dynamics 是 Core 强制能力；每个 Robot Core config 都应提供：

- `dynamics.urdf_path`
- `dynamics.joint_names`
- `dynamics.base_frame`
- `dynamics.tool_frame`
- `dynamics.gravity`
- `dynamics.gravity_scale`

如果暂时没有真实惯性参数，应在 URDF 中放合法 placeholder inertial，而不是跳过 Dynamics 初始化；当前测试 fixture 使用很小但正数的 mass 和 inertia，因为这更容易被 URDF / Pinocchio 稳定接受

placeholder inertial 可以用于：

- 软件接入
- URDF 验证
- FK
- Jacobian
- 控制链路
- 接口开发

但此时以下输出不具备真实机械臂动力学意义：

- gravity
- mass matrix
- inverse dynamics
- model feedforward

后续应使用 CAD 或实测惯性参数替换

## Safety

Safety 检查：

- NaN / Inf
- state timeout
- command timeout
- joint state size
- actuator state size
- actuator online / enabled / fault
- velocity limit
- effort limit
- kp / kd limit
- command continuity / step safety
- revolute / prismatic 位置限制

continuous joint 没有绝对位置上下限，不伪造 `[-pi, pi]`，因此不执行：

- `JOINT_POS_LIMIT`
- `CMD_POS_LIMIT`
- `position_margin`

continuous joint 仍执行：

- NaN / Inf
- velocity limit
- effort limit
- kp limit
- kd limit
- timeout
- state validity
- command continuity / step safety
- actuator hardware state checks

当前 Core 的命令 step safety 使用现有 position representation 的差值；若内部位置是 unwrapped / multi-turn，则不会做 modulo；若后续某个 Backend 改为 wrapped representation，需要在明确协议后处理 `+pi` / `-pi` 的 shortest angular distance

## Hardware Backend

完整支持的 Backend 必须实现 `MotorBus`，并接收完整 MIT semantics：

- `position`
- `velocity`
- `torque`
- `kp`
- `kd`

不要添加 `supports_position`、`supports_velocity`、`supports_torque`、`supports_impedance` 之类开关；Core 不根据 Backend 能力降级；不支持完整 MIT semantics 的硬件适配不属于完整支持的 SerialArm-Core Backend

Backend 需要负责：

- 读取 Backend YAML
- 打开总线或设备
- 将厂商反馈转换为 `ActuatorState`
- 将 `ActuatorCtrlCmd` 转换为厂商协议
- 处理 enable / stop / disable / recover
- 暴露 `HardwareCapabilities`

## ros2_control

`serial_arm_ros2_control` 负责把 ROS 2 control lifecycle 映射到 Core：

- `on_init` 加载参数和 profile
- `on_configure` 构造 Dynamics、MotorBus、Robot
- `on_activate` 激活控制链路
- `read` 读取硬件状态并映射
- `write` 下发 JointTrajectoryController 输出
- `on_deactivate` 停止或失能

`hardware.launch.py` 使用：

- `robot_state_publisher`
- `controller_manager`
- `joint_state_broadcaster`
- `joint_trajectory_controller`

这些不依赖 MoveIt

## MoveIt

MoveIt 是 ROS 2 上层可选能力；只有运行：

```bash
ros2 launch serial_arm_ros2_control moveit.launch.py robot_profile:=dm_arm_gray
```

或直接使用 MoveIt config 时，才要求 profile 定义 `moveit.package`

如果 profile 没有 MoveIt support，错误应明确为：

```text
Robot profile '<name>' does not define MoveIt support
```

## Python Binding

Python Binding 暴露 Core 的离线计算和真机 session 封装；典型用途：

- 加载 robot config
- 查询 joint names
- 运行 model / dynamics 工具
- 用 Python 终端调试 RobotSession

构建 Python 绑定需要 pybind11、Python development module 和 Pinocchio 环境；接口细节见 [API.md](API.md)

## 真机 Bringup

建议顺序：

1. `display.launch.py` 验证 URDF、joint order、mesh、frame
2. 用 fake/mock Backend 验证 ros2_control 生命周期
3. 检查 hardware config：串口/CAN、actuator id、型号、方向、零点
4. 低增益 `COMPLIANT_HOLD` 或保守 `RIGID_HOLD` 验证 enable/disable
5. 小幅度单关节命令验证 direction、ratio、limit
6. 再接 JointTrajectoryController / MoveIt

不要在 Safety、mapping、direction 未验证前直接运行大幅度轨迹

## 调参

优先顺序：

1. 确认 URDF joint axis、limit、frame
2. 确认 `mapping.pos_ratio`、`mapping.tor_ratio`、`direction`、zero offset
3. 确认 hardware capability 与 motor model 一致
4. 收窄 Safety policy
5. 从低 kp/kd 开始验证 impedance mode
6. 确认 gravity model 后再启用有物理意义的 model feedforward

## Troubleshooting

`profile loader` 找不到 profile：先检查 `--profile-file`、`SERIAL_ARM_RESOURCE_PATH`、standalone install prefix、`serial_arm_robot_profiles/config/robot_profiles.yaml` 和 Robot resource package 是否存在；如果是在 ROS 2 / colcon 环境，再检查是否 `source install/setup.bash`

Core config 或 hardware config 找不到：这是 SerialArm-Core 的 resource resolution 问题，检查 `core.package + core.config`、`hardware.config_package + hardware.config` 是否能在资源根下解析到对应文件；ROS 2 package discovery 只影响 launch / adapter 层

`display.launch.py` 失败：检查 `description.package`、`description.urdf` 和 URDF 本身

`hardware.launch.py` 失败：检查 ros2_control xacro、controllers YAML、Backend plugin 名称、hardware config 路径

`moveit.launch.py` 报未定义 MoveIt support：该 profile 没有 `moveit.package`；这是合法 profile；只在需要 MoveIt 时补配置

Dynamics configure 失败：检查 URDF 是否可被 Pinocchio 加载、受控 joint 是否都是当前 Dynamics 支持的单自由度 joint、base/tool frame 是否存在、inertial 是否合法

Safety 报 position limit：revolute/prismatic 检查 URDF limit 和 `position_margin`；continuous joint 不应触发 `JOINT_POS_LIMIT` 或 `CMD_POS_LIMIT`
