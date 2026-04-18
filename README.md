<div align="center">

# DM-Arm-Hardware-Interface

一个面向达妙电机机械臂的 ROS 2 硬件接口工作区，覆盖机械臂描述、底层串口控制、`ros2_control` 硬件插件、`JointTrajectoryController` 轨迹执行，以及基于 Pinocchio 的重力/非线性前馈能力

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](./LICENSE)
[![ROS 2: Humble](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Control: ros2_control](https://img.shields.io/badge/Control-ros2__control-green.svg)](https://control.ros.org/)
[![Dynamics: Pinocchio](https://img.shields.io/badge/Dynamics-Pinocchio-orange.svg)](https://github.com/stack-of-tasks/pinocchio)
[![Hardware: DM Motor](https://img.shields.io/badge/Hardware-Damiao-red.svg)](#)

</div>

---

## 项目简介

本仓库用于构建一套可运行在 ROS 2 环境下的达妙机械臂控制基础设施，目标不是单纯提供一份 URDF 或一个示例脚本，而是形成一条完整、可扩展、便于继续做动力学与高级控制研究的控制链：

- `dm_arm_description`：机械臂模型、网格、URDF/Xacro 资源
- `dm_hw`：达妙电机串口通信与控制模式封装
- `dm_ros_control`：`hardware_interface::SystemInterface` 插件实现
- `JointTrajectoryController`：轨迹控制器
- Pinocchio：重力补偿 / 非线性前馈 / 质量矩阵计算
- RViz / robot_state_publisher：可视化与状态发布

当前工作区适合以下场景：

- 达妙机械臂真机接入与关节状态读取
- 基于 `ros2_control` 的标准控制器框架集成
- 基于 `JointTrajectoryController` 的单点/多点轨迹执行
- MIT 模式下的阻抗式关节保持、柔顺测试与动力学前馈实验
- 后续扩展 Pinocchio / TSID / inverse dynamics / whole-body 控制研究

---

## 系统架构

### 硬件配置

| 项目 | 规格 | 说明 |
|---|---|---|
| 机械臂 | DM Arm | 6 个旋转关节 + 夹爪驱动 |
| 电机通信 | USB 转 CAN / 串口链路 | 当前接口层按串口设备接入 |
| 主机系统 | Ubuntu 22.04 | 推荐 ROS 2 Humble 环境 |
| 可视化 | RViz 2 | 支持假硬件与真机状态显示 |

### 软件栈

- 操作系统：Ubuntu 22.04 LTS
- ROS 版本：ROS 2 Humble
- 控制框架：`ros2_control`
- 轨迹控制器：`joint_trajectory_controller/JointTrajectoryController`
- 硬件接口：`dm_ros_control/DmHardwareInterface`
- 动力学库：Pinocchio
- 机械臂描述：`dm_arm_description`
- 底层通信：`dm_hw`
- 可视化：`robot_state_publisher` + RViz 2 + `joint_state_publisher_gui`

### 当前控制链路

```text
JointTrajectoryController
        │
        │  position / velocity command
        ▼
DmHardwareInterface (ros2_control plugin)
        │
        ├── joint state feedback
        ├── MIT / POS_VEL command dispatch
        └── dynamics feedforward (Pinocchio)
        ▼
       dm_hw
        │
        ▼
 SerialPort / USB-CAN bridge
        │
        ▼
   Damiao Motors
```

### 控制语义说明

当前工程的主控制路径为：

- 上层控制器输出：`q_des`、`dq_des`
- 硬件反馈输入：`q`、`dq`
- 动力学前馈：`τ_ff`
- 达妙 MIT 模式控制输入：`kp`、`kd`、`q_des`、`dq_des`、`τ_ff`

可近似理解为：

```text
τ = Kp · (q_des - q) + Kd · (dq_des - dq) + τ_ff
```

这意味着本项目既可以作为标准轨迹执行接口使用，也可以继续向“关节空间阻抗控制 + 动力学补偿”的方向扩展

---

## 功能特性

### 1. `dm_hw`：达妙底层硬件封装

`dm_hw` 是一个轻量 C++ 库/ROS 2 包，负责完成达妙电机的底层通信与控制模式封装，当前已覆盖：

- 电机对象建模（电机型号、ID、状态、参数缓存）
- 串口通信收发
- 电机使能 / 失能 / 回零
- 多种控制模式封装
  - MIT 模式
  - 位置速度模式
  - 纯速度模式
  - 力位混合模式
  - CSP 系列模式
- 寄存器参数读写与保存
- `test_damiao` 最小测试程序

适合单电机联调、总线检查、寄存器读写与底层通信验证

### 2. `dm_arm_description`：机械臂描述包

用于集中维护机械臂的：

- URDF / Xacro
- 网格与碰撞模型
- 关节与连杆层级
- 末端执行器相关几何与碰撞表达

该包本身不承担控制逻辑，而是作为整个工作区的统一模型真源。

### 3. `dm_ros_control`：ROS 2 硬件接口主包

`dm_ros_control` 是本仓库的核心运行包，提供：

- `hardware_interface::SystemInterface` 插件实现
- `ros2_control_node` 集成入口
- `JointTrajectoryController` 配置
- 真机 / 假硬件两套启动方式
- Pinocchio 动力学模型更新
- 重力前馈与非线性前馈开关
- RViz 联调启动文件

### 4. 动力学前馈能力

当前工程已经为后续动力学控制留出清晰接口，支持：

- `computeGeneralizedGravity`：重力补偿
- `nonLinearEffects`：非线性项前馈
- `crba`：质量矩阵计算

适用场景包括：

- 静态姿态保持
- 降低关节“塌陷”倾向
- MIT 模式下的柔顺保持与被动跟随实验
- 后续扩展 TSID / inverse dynamics / torque feedforward

---

## 工作区结构

```text
DM-Arm-Hardware-Interface/
├── README.md
├── Tutorial.md
├── LICENSE
└── src/
    ├── infra/
    │   ├── tl/                    # TartanLlama optional / expected 相关基础依赖封装
    │   └── tsid/                  # TSID 源码树与构建资源（用于后续动力学/任务空间控制实验）
    └── platform/
        ├── dm_arm_description/    # 机械臂描述包（URDF / meshes / model resources）
        ├── dm_hw/                 # 达妙底层串口通信与电机控制封装
        └── dm_ros_control/        # ros2_control 硬件接口、控制器配置、launch、动力学集成
```

### `dm_ros_control` 包内结构

```text
src/platform/dm_ros_control/
├── CMakeLists.txt
├── package.xml
├── dm_ros_control_plugins.xml
├── config/
│   └── controllers.yaml
├── launch/
│   ├── dm_ros_control.launch.py
│   └── dm_ros_control_rviz.launch.py
├── src/
│   ├── dm_hardware_interface.cpp
│   └── pinocchio_dynamics_model.cpp
└── urdf/
    └── dm_arm_ros_control.urdf.xacro
```

---

## 环境准备

## 1. 基础环境

推荐环境：

- Ubuntu 22.04 LTS
- ROS 2 Humble
- `colcon`
- `rosdep`
- `xacro`
- `ros2_control`
- `ros2_controllers`

可参考如下基础安装流程：

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

若尚未安装 ROS 2 Humble，请先完成官方安装

## 2. ROS 2 相关依赖

建议至少具备以下运行依赖：

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

## 3. Pinocchio / 动力学相关依赖

如果需要启用动力学前馈，建议安装：

```bash
sudo apt install -y \
  ros-humble-pinocchio \
  ros-humble-hpp-fcl \
  ros-humble-eigenpy \
  libeigen3-dev
```

更完整的 Pinocchio / TSID 安装与编译建议，可参考仓库内文档：

- `Tutorial.md`

---

## 源码编译

### 1. 获取源码

```bash
cd ~/ros2_ws/src
git clone https://github.com/Kaede-Rei/DM-Arm-Hardware-Interface.git
cd ..
```

### 2. 安装依赖

```bash
source /opt/ros/humble/setup.bash
sudo rosdep init || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

> 说明：若 `src/infra/tsid` 参与构建，编译时间与依赖体量会显著增加；如当前只需硬件接口联调，可先聚焦 `dm_hw`、`dm_arm_description`、`dm_ros_control` 三个核心包

### 3. 编译工作区

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 4. 按包编译（推荐联调时使用）

```bash
colcon build --symlink-install --packages-select \
  dm_hw \
  dm_arm_description \
  dm_ros_control
```

---

## 快速开始

## 1. 假硬件 + RViz 联调

适用于：

- 检查 URDF / TF / 控制器是否正常加载
- 检查 `ros2_control` 启动链路
- 在无真机时验证控制话题和 RViz 状态变化

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch dm_ros_control dm_ros_control_rviz.launch.py \
  use_fake_hardware:=true
```

启动后可完成：

- `robot_state_publisher` 发布模型
- `ros2_control_node` 启动控制器管理器
- `joint_state_broadcaster` 启动
- `arm_controller` 启动
- RViz 打开模型显示
- `joint_state_publisher_gui` 提供关节交互滑条

## 2. 真机启动

适用于：

- 达妙电机真机联调
- 状态读取
- 轨迹执行
- MIT 模式与动力学前馈测试

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch dm_ros_control dm_ros_control.launch.py \
  use_fake_hardware:=false \
  serial_port:=/dev/ttyACM0 \
  baudrate:=921600 \
  kp:=38.0 \
  kd:=1.5 \
  enable_dynamics:=true \
  enable_gravity_feedforward:=true \
  enable_nonlinear_feedforward:=false
```

## 3. 查看控制器状态

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

预期至少应看到：

- `joint_state_broadcaster`
- `arm_controller`

## 4. 查看关节状态

```bash
ros2 topic echo /joint_states
```

## 5. 下发一条轨迹命令

```bash
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper_left'],
  points: [
    {
      positions: [0.3, 0.2, 0.2, 0.2, 0.2, 0.2, -0.01],
      time_from_start: {sec: 2}
    }
  ]
}"
```

建议先在假硬件环境验证，再进行真机测试

---

## 启动文件说明

## 1. `dm_ros_control.launch.py`

标准控制启动入口，负责：

- 加载 `dm_arm_ros_control.urdf.xacro`
- 启动 `robot_state_publisher`
- 启动 `ros2_control_node`
- 启动 `joint_state_broadcaster`
- 启动 `arm_controller`

### 常用参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `use_fake_hardware` | `false` | 是否使用假硬件 |
| `serial_port` | `/dev/ttyACM0` | 真机串口设备 |
| `baudrate` | `921600` | 串口波特率 |
| `kp` | `10.0` | MIT 模式位置刚度参数 |
| `kd` | `0.02` | MIT 模式速度阻尼参数 |
| `enable_write` | `true` | 是否允许向电机下发命令 |
| `refresh_state_in_read` | `false` | 是否在 `read()` 中主动刷新电机状态 |
| `startup_read_cycles` | `5` | 启动阶段初始读取轮数 |
| `enable_dynamics` | `true` | 是否启用动力学模块 |
| `enable_gravity_feedforward` | `true` | 是否启用重力前馈 |
| `enable_nonlinear_feedforward` | `false` | 是否启用完整非线性前馈 |

## 2. `dm_ros_control_rviz.launch.py`

在标准控制启动基础上额外拉起：

- RViz 2
- `joint_state_publisher_gui`

适合模型检查、联调与演示。

---

## 控制器配置说明

当前 `controllers.yaml` 使用：

- `joint_state_broadcaster/JointStateBroadcaster`
- `joint_trajectory_controller/JointTrajectoryController`

当前轨迹控制器配置特征：

- 控制关节：`joint1 ~ joint6 + gripper_left`
- 命令接口：`position` + `velocity`
- 状态接口：`position` + `velocity`
- `allow_partial_joints_goal: false`
- `interpolate_from_desired_state: false`

这意味着当前工程已经不是“只给位置目标、底层自己猜速度”的简单位置控制模式，而是标准的：

- 上层控制器生成连续轨迹
- 硬件接口直接消费 `q_des / dq_des`
- MIT 模式叠加动力学前馈

---

## `dm_arm_ros_control.urdf.xacro` 说明

该文件是整个控制系统的桥接文件，负责将：

- `dm_arm_description` 中的机械臂模型
- `ros2_control` 硬件资源定义
- 启动参数（串口、kp/kd、动力学开关）

组合到同一份运行时 `robot_description` 中

当前定义的关节包括：

- `joint1`
- `joint2`
- `joint3`
- `joint4`
- `joint5`
- `joint6`
- `gripper_left`

所有这些关节当前都导出：

- `position` 命令接口
- `velocity` 命令接口
- `position` 状态接口
- `velocity` 状态接口

并在真机模式下使用：

- `dm_ros_control/DmHardwareInterface`

作为底层 `hardware_interface::SystemInterface` 插件

---

## 动力学模块说明

## 1. 功能定位

`pinocchio_dynamics_model.cpp` 负责根据当前关节状态在线计算：

- 重力项 `g(q)`
- 非线性项 `nle(q, dq)`
- 质量矩阵 `M(q)`

其输出用于 `dm_hardware_interface.cpp` 中的前馈项生成

## 2. 当前用途

在当前控制链路中，动力学模块的首要目标不是直接替代控制器，而是：

- 提供重力补偿
- 降低静态保持时的下垂倾向
- 改善 MIT 模式下的姿态支撑能力
- 为更进一步的逆动力学 / TSID 扩展保留接口

## 3. 关于完整 URDF 与受控关节维度不一致的问题

在机械臂工程中，完整 URDF 往往会包含：

- 仅用于碰撞检测的被动关节
- mimic 关节
- 仅用于几何表达的附属关节

而 `hardware interface` 实际可读写的控制状态维度，未必与完整模型 `nq / nv` 一致

对于这类情况，动力学侧应保证：

- 控制状态空间与动力学模型维度严格一致
- 对非受控关节使用锁定或 reduced model 处理
- 不应直接将 7 维控制状态强行喂给 8 维完整模型

否则会导致：

- 动力学更新失败
- 重力前馈长期无效
- 开关重力补偿几乎没有体感差异

---

## 参数整定建议

本项目当前主参数入口为：

- `kp`
- `kd`
- `enable_gravity_feedforward`
- `enable_nonlinear_feedforward`

### `kp` 的作用

`kp` 决定位置误差对应的“虚拟刚度”。

- 数值大：更硬，更不容易被外力推离目标位姿
- 数值小：更软，更容易产生被动跟随感

### `kd` 的作用

`kd` 决定速度误差对应的“虚拟阻尼”。

- 数值大：回位更稳，但会更黏、更迟缓
- 数值小：更灵活，但更容易发飘或回弹

### 推荐调参顺序

1. 先保证动力学前馈确实生效
2. 先固定 `kp`，逐步提高 `kd`
3. 再根据软硬程度微调 `kp`
4. 最后再判断是否需要启用完整 `nonlinear feedforward`

### 柔性保持的典型目标

若目标是：

- 无外力时能保持目标位姿
- 受外力时允许一定被动偏离
- 松手后平稳回归

一般更适合采用：

- 低到中等 `kp`
- 中等 `kd`
- 开启 `gravity feedforward`

而不是一味增大 `kp`

---

## 常用开发与调试命令

### 编译单个包

```bash
colcon build --symlink-install --packages-select dm_ros_control
```

### 查看控制器

```bash
ros2 control list_controllers
```

### 查看硬件接口

```bash
ros2 control list_hardware_interfaces
```

### 查看机器人描述

```bash
ros2 param get /robot_state_publisher robot_description
```

### 查看控制器命令话题

```bash
ros2 topic list | grep arm_controller
```

### 底层电机测试

```bash
ros2 run dm_hw test_damiao
```

---

## 故障排查

### 1. 控制器无法启动

优先检查：

- `robot_description` 是否成功加载
- `dm_ros_control_plugins.xml` 是否正确安装
- `pluginlib` 是否找到 `dm_ros_control/DmHardwareInterface`
- `controllers.yaml` 中控制器名称与启动脚本是否一致

### 2. 真机无反馈或反馈异常

优先检查：

- 串口设备路径是否正确
- 波特率是否匹配
- USB 转 CAN / 串口链路是否稳定
- 电机 ID 是否与 Xacro 中配置一致
- 用户是否具备串口访问权限

可通过以下方式检查权限：

```bash
groups $USER
sudo usermod -aG dialout $USER
```

重新登录后生效。

### 3. 机械臂可以动，但响应迟缓或发软

优先检查：

- 实际控制频率是否足够
- `read()` / `write()` 是否被通信阻塞拉慢
- `kp` / `kd` 是否过于保守
- 重力前馈是否确实生效

### 4. 开关重力补偿几乎没区别

优先检查：

- Pinocchio 模型维度是否与实际受控关节维度一致
- 动力学更新函数是否返回成功
- 当前 `_gravity_feedforward_` 是否真的为非零
- 完整 URDF 中是否包含未受控的 mimic / 从动关节

### 5. 轨迹控制异常

优先检查：

- 下发的 `joint_names` 是否与控制器配置完全一致
- `allow_partial_joints_goal` 当前为 `false`，必须给全关节目标
- 起始状态与当前真实状态是否已经同步
- 是否在真机未稳态时连续插入新轨迹

---

## 参考资料

- ROS 2 Humble 文档
- ros2_control 文档
- Pinocchio 文档
- TSID 项目文档
- 仓库内文档：`Tutorial.md`
- 包内说明：`src/platform/dm_hw/README.md`

---

## 致谢

本工作区在开发与实现过程中参考或使用了以下开源项目，特此致谢：

- [TartanLlama/expected](https://github.com/TartanLlama/expected)
- [TartanLlama/optional](https://github.com/TartanLlama/optional)
- [robot-learning-co/trlc-dk1](https://github.com/robot-learning-co/trlc-dk1)
- [stack-of-tasks/Pinocchio](https://github.com/stack-of-tasks/Pinocchio)
- [stack-of-tasks/TSID](https://github.com/stack-of-tasks/TSID)

---

## 许可证

MIT License，详见 `LICENSE`

## 贡献

欢迎提交 Issue / PR

