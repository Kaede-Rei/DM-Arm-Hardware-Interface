# DM-Arm-Hardware-Interface

面向达妙电机机械臂的 ROS 2 Humble 工作区；仓库包含机械臂描述、达妙底层通信、ROS 2 control 硬件插件、关节阻抗控制核心、Pinocchio 动力学前馈、假硬件联调和真机启动入口

当前控制链路以 `ros2_control` 为主干，上层使用 `joint_trajectory_controller/JointTrajectoryController` 输出关节位置和速度目标，硬件接口将目标转换为达妙 MIT 或 POS_VEL 命令

## 包结构

```text
DM-Arm-Hardware-Interface/
├── cmake/
│   └── dm_control_core.cmake          # 非 ROS 工程复用 dm_control_core 的 CMake 入口
├── src/
│   ├── infra/
│   │   ├── tl/                        # tl::optional / tl::expected
│   │   └── tsid/                      # TSID 源码树，后续高级控制实验使用
│   └── platform/
│       ├── dm_arm_description/        # 机械臂 URDF、mesh、可视化模型
│       ├── dm_control_core/           # ROS 无关控制核心与动力学封装
│       ├── dm_hw/                     # 达妙串口、Motor、MotorControl 底层封装
│       └── dm_ros_control/            # ros2_control SystemInterface 插件和 launch
├── README.md
├── Tutorial.md
└── LICENSE
```

核心包职责：

| 包 | 作用 |
|---|---|
| `dm_arm_description` | 机械臂模型真源，提供 URDF 和网格资源 |
| `dm_hw` | 达妙电机通信、控制模式、参数读写 |
| `dm_control_core` | 纯 C++ 控制核心，包含阻抗控制、动力学观测、电机总线抽象 |
| `dm_ros_control` | ROS 2 control 硬件插件，连接控制器、控制核心和真机 |
| `tl` | `optional` / `expected` 基础依赖 |
| `tsid` | 后续 inverse dynamics / TSID 研究基础 |

## 控制架构

```text
JointTrajectoryController
        │ position / velocity command
        ▼
dm_ros_control/DmHardwareInterface
        │
        ├── JointImpedanceController
        ├── DynamicsObserver / PinocchioDynamicsModel
        └── DmMotorBus
        ▼
dm_hw::MotorControl
        ▼
SerialPort / USB-CAN
        ▼
Damiao motors
```

MIT 模式下可近似理解为：

```text
tau = kp * (q_ref - q) + kd * (dq_ref - dq) + tau_ff
```

其中：

- `q_ref / dq_ref` 来自 `JointTrajectoryController` 或硬件接口的命令模式转换
- `kp / kd` 默认来自 `dm_ros_control/config/pd_config.yaml`
- `tau_ff` 可由 Pinocchio 重力项或非线性项生成
- 真机通信由 `DmMotorBus` 通过 `dm_hw` 完成

## 环境依赖

推荐环境：

- Ubuntu 22.04
- ROS 2 Humble
- `colcon`
- `rosdep`

基础依赖：

```bash
sudo apt update
sudo apt install -y \
  git \
  build-essential \
  cmake \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool
```

ROS 2 control 和可视化依赖：

```bash
sudo apt install -y \
  ros-humble-xacro \
  ros-humble-rviz2 \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-joint-trajectory-controller
```

动力学依赖：

```bash
sudo apt install -y \
  ros-humble-pinocchio \
  ros-humble-hpp-fcl \
  ros-humble-eigenpy \
  libeigen3-dev
```

## 构建

完整构建：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

日常联调推荐只构建核心包：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  dm_hw \
  dm_control_core \
  dm_arm_description \
  dm_ros_control
source install/setup.bash
```

说明：`src/infra/tsid` 体量较大，当前只做机械臂硬件接口联调时不必优先构建

## 快速启动

### 假硬件联调

适合在不上真机时检查 URDF、控制器、话题和 RViz：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch dm_ros_control dm_ros_control.launch.py \
  use_fake_hardware:=true \
  use_rviz:=true
```

假硬件模式使用 `mock_components/GenericSystem`，不会加载 `DmHardwareInterface`，也不会读取串口或运行 `dm_control_core` 的真机路径

检查控制器：

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states
```

下发一条小幅轨迹：

```bash
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper_left'],
  points: [
    {
      positions: [0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.0],
      velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      time_from_start: {sec: 2}
    }
  ]
}"
```

### 真机启动

真机启动会加载 `dm_ros_control/DmHardwareInterface`，打开串口，使能电机，并按配置下发控制命令

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch dm_ros_control dm_ros_control.launch.py \
  use_fake_hardware:=false \
  serial_port:=/dev/ttyACM0 \
  baudrate:=921600 \
  enable_write:=true \
  command_mode:=impedance \
  enable_dynamics:=true \
  enable_gravity_feedforward:=true \
  enable_nonlinear_feedforward:=false
```

上真机前建议先确认：

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states
```

## 启动参数

`dm_ros_control.launch.py` 常用参数：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `use_fake_hardware` | `false` | `true` 时使用 `mock_components/GenericSystem`，不接触真机 |
| `use_rviz` | `false` | 是否启动 RViz |
| `serial_port` | `/dev/ttyACM0` | 真机串口设备 |
| `baudrate` | `921600` | 串口波特率 |
| `enable_write` | `true` | `false` 时 `write()` 直接返回，不向电机写命令 |
| `refresh_state_in_read` | `false` | `read()` 中是否主动刷新电机状态 |
| `startup_read_cycles` | `5` | 激活阶段初始状态读取平均次数 |
| `legacy_feedforward_enabled` | `true` | 是否把模型前馈叠加到 MIT 命令 |
| `legacy_pd_fallback` | `true` | 是否从 `pd_config.yaml` 加载 `kp/kd` |
| `command_mode` | `impedance` | `hold`、`position`、`position_velocity`、`impedance`、`velocity`、`torque` |
| `enable_dynamics` | `true` | 是否启用 Pinocchio 动力学观测 |
| `enable_gravity_feedforward` | `true` | 是否选择重力项作为模型前馈 |
| `enable_nonlinear_feedforward` | `false` | 是否选择完整非线性项作为模型前馈 |

`kp/kd` 暂时使用默认增益；默认 PD 增益来自：

```text
src/platform/dm_ros_control/config/pd_config.yaml
```

运行时读取的是安装后的：

```text
install/dm_ros_control/share/dm_ros_control/config/pd_config.yaml
```

使用 `--symlink-install` 时，源码配置修改后通常可直接生效；否则需要重新构建安装

## ros2_control 接口

当前受控关节：

```text
joint1
joint2
joint3
joint4
joint5
joint6
gripper_left
```

真机 URDF 中每个关节导出命令接口：

- `position`
- `velocity`
- `effort`
- `kp`
- `kd`

当前 `arm_controller` 只使用：

- `position`
- `velocity`

硬件插件内部会根据 `command_mode` 将这些命令转换为 `JointCommand`，再交给 `JointImpedanceController` 生成 MIT 命令；`kp/kd` 命令接口目前主要保留为过渡接口，实际控制增益来自控制核心配置

状态接口包括：

- `position`
- `velocity`
- `effort`
- `motor_effort`
- `gravity_effort`
- `nonlinear_effort`
- `model_feedforward_effort`
- `feedforward_effort`
- `external_effort`

## dm_control_core

`dm_control_core` 是 ROS 无关的控制核心；主要接口文档见：

```text
src/platform/dm_control_core/README.md
```

核心类：

| 类 | 作用 |
|---|---|
| `JointImpedanceController` | 将上层关节命令转换为 MIT 位置、速度、力矩、`kp/kd` |
| `DynamicsObserver` | 封装动力学观测，输出重力项、非线性项、当前 active feedforward |
| `PinocchioDynamicsModel` | 构建 reduced model，计算 `g(q)`、`nle(q,dq)`、`M(q)` |
| `DmMotorBus` | 达妙电机总线管理和关节/电机单位换算 |

运行控制核心测试：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select dm_control_core
colcon test --packages-select dm_control_core --event-handlers console_direct+
```

## 动力学前馈

动力学路径在 `enable_dynamics:=true` 时启用；`DynamicsObserver` 每个控制周期使用当前关节位置、速度、实测力矩计算：

- `gravity`：重力项
- `nonlinear`：非线性项
- `active_feedforward`：当前被选中的前馈项
- `external_effort`：测得力矩扣除 active feedforward 后的估计外力

选择规则：

```text
enable_nonlinear_feedforward=true  -> active_feedforward = nonlinear
enable_gravity_feedforward=true    -> active_feedforward = gravity
otherwise                          -> active_feedforward = 0
```

`PinocchioDynamicsModel` 会从完整 URDF 构建 reduced model，只保留 `ros2_control` 管理的 1-DoF 关节，并锁定非受控关节；这样可以避免完整模型维度和控制维度不一致导致的前馈失效

## 测试策略

推荐按风险从低到高测试：

1. `dm_control_core` 单元测试：验证命令语义、限幅、前馈叠加和非法输入处理
2. `use_fake_hardware:=true`：验证 URDF、控制器、话题、RViz，不碰真机
3. 真机 `enable_write:=false`：验证串口打开、状态读取、动力学观测，但不下发周期写命令
4. 真机小幅轨迹：低 `kp/kd`、低速度、小范围单关节，再扩展到多关节
5. 动力学前馈：先重力项，再评估是否需要非线性项

注意：`use_fake_hardware:=true` 不覆盖 `DmHardwareInterface` 和 `dm_control_core` 真机路径如果要在完整硬件插件内做闭环仿真，需要后续增加 `SimDmMotorBus` 或等价仿真后端

## 常用命令

构建核心包：

```bash
colcon build --symlink-install --packages-select dm_hw dm_control_core dm_arm_description dm_ros_control
```

运行控制核心测试：

```bash
colcon test --packages-select dm_control_core --event-handlers console_direct+
```

查看控制器：

```bash
ros2 control list_controllers
```

查看硬件接口：

```bash
ros2 control list_hardware_interfaces
```

查看关节状态：

```bash
ros2 topic echo /joint_states
```

查看机器人描述：

```bash
ros2 param get /robot_state_publisher robot_description
```

底层电机测试：

```bash
ros2 run dm_hw test_damiao
```

## 故障排查

### 控制器无法启动

检查：

- `source install/setup.bash` 是否执行
- `robot_description` 是否生成成功
- `dm_ros_control_plugins.xml` 是否安装
- `controller_manager` 是否加载了 `joint_state_broadcaster` 和 `arm_controller`
- `controllers.yaml` 中关节名是否和 URDF 完全一致

### 真机无反馈

检查：

- `serial_port` 是否正确
- 当前用户是否在 `dialout` 组
- 波特率是否匹配
- 电机 ID 和 xacro 中的 `motor_id` 是否一致
- USB-CAN 或串口链路是否稳定

权限配置：

```bash
sudo usermod -aG dialout $USER
```

重新登录后生效

### 机械臂响应发软或发飘

检查：

- `pd_config.yaml` 中 `kp/kd` 是否过低
- 控制频率是否稳定在 `controllers.yaml` 的 `update_rate`
- `enable_gravity_feedforward` 是否真的生效
- `model_feedforward_effort` 状态接口是否接近非零
- 轨迹目标速度是否过大

### 重力补偿没有体感

检查：

- `enable_dynamics:=true`
- `enable_gravity_feedforward:=true`
- `enable_nonlinear_feedforward:=false` 或按需开启
- URDF 中受控关节是否都能被 Pinocchio 找到
- reduced model 维度是否等于受控关节数量

### 轨迹命令被拒绝

检查：

- `joint_names` 必须完整包含 7 个关节
- 当前 `allow_partial_joints_goal: false`
- `position` 数组长度必须等于关节数量
- 如使用 velocity，也应给出相同长度的数组

## 参考

- `Tutorial.md`
- `src/platform/dm_control_core/README.md`
- `src/platform/dm_hw/README.md`
- ROS 2 Humble
- ros2_control
- Pinocchio
- TSID

## 许可证

MIT License，详见 `LICENSE`
