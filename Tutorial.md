# DM-Arm-Hardware-Interface：Pinocchio 动力学前馈教程

> 本文不是项目 README，也不是仓库导航文档。本文只讨论当前项目中 **Pinocchio 动力学前馈链路** 的实现与使用，重点是：
>
> 1. 如何在 `dm_ros_control` 中接入 Pinocchio
> 2. 如何完成 **重力补偿** `g(q)`
> 3. 如何完成 **非线性项补偿** `nle(q, dq)`
> 4. 如何理解当前控制律里 `kp / kd / tau_ff` 的分工
> 5. 如何在当前项目里验证补偿是否真的生效

---

## 0. 教程范围与当前实现边界

当前项目里的动力学前馈，不是完整逆动力学控制器，也不是 TSID 任务空间控制器。

当前实际实现属于：

- 上层：`joint_trajectory_controller/JointTrajectoryController`
- 中层：`ros2_control` 的 `position + velocity` 命令接口
- 底层：达妙电机 MIT 模式
- 前馈：Pinocchio 计算的 `g(q)` 或 `nle(q, dq)`

当前控制律可以近似写成：

```text
τ = Kp · (q_des - q) + Kd · (dq_des - dq) + τ_ff
```

其中：

- `q_des, dq_des`：来自 JTC 的轨迹采样结果
- `q, dq`：来自电机反馈，经 `joint_to_motor_scale` 换算后的关节状态
- `τ_ff`：来自 Pinocchio 动力学模型
- `Kp, Kd`：MIT 模式中的刚度和阻尼参数

因此，本文讨论的是 **关节空间阻抗控制 + 动力学前馈补偿**。

---

## 1. 当前项目中与动力学相关的文件

本教程对应的核心文件如下：

```text
src/platform/dm_ros_control/
├── include/dm_ros_control/
│   ├── dm_hardware_interface.hpp
│   └── pinocchio_dynamics_model.hpp
├── src/
│   ├── dm_hardware_interface.cpp
│   └── pinocchio_dynamics_model.cpp
├── urdf/
│   └── dm_arm_ros_control.urdf.xacro
├── config/
│   └── controllers.yaml
└── launch/
    ├── dm_ros_control.launch.py
    └── dm_ros_control_rviz.launch.py
```

职责划分如下：

- `pinocchio_dynamics_model.*`
  - 负责从 URDF 建模
  - 负责 reduced model 构造
  - 负责周期更新 `q / dq`
  - 负责输出 `g(q)`、`nle(q, dq)`、`M(q)`

- `dm_hardware_interface.*`
  - 负责解析 `ros2_control` 参数
  - 负责读取电机反馈状态
  - 负责在 `read()` 中更新动力学模型
  - 负责在 `write()` 中把前馈扭矩叠加到 MIT 控制命令里

- `dm_arm_ros_control.urdf.xacro`
  - 负责把硬件参数和动力学开关传给 `DmHardwareInterface`

- `controllers.yaml`
  - 负责配置 `JointTrajectoryController`

- `launch/*.launch.py`
  - 负责启动 `robot_state_publisher + ros2_control_node + spawner`

---

## 2. 环境准备

当前项目至少需要 ROS 2 Humble、Pinocchio、Eigen、基础构建工具。

如果只做当前已经实现好的 **重力补偿 / 非线性项补偿**，实际上并不强依赖 TSID；
但如果后续准备继续做完整逆动力学、任务空间控制、约束控制，提前把 TSID 环境配好是合理的。

### 2.1 安装 Pinocchio 与基础依赖

```bash
sudo apt update

# Pinocchio + FCL + Eigen + eigenpy
sudo apt install -y \
  ros-humble-pinocchio \
  ros-humble-hpp-fcl \
  ros-humble-eigenpy \
  libeigen3-dev \
  libboost-python-dev

# 可选：用于一些可视化实验
pip install meshcat
```

### 2.2 可选：安装 TSID

如果工作空间里还没有 TSID，可按下面方式准备。当前项目主链路不直接依赖 TSID 做补偿，但后续扩展可直接复用。

```bash
cd ~/your_ws/src
git clone --recursive https://github.com/stack-of-tasks/tsid.git
cd ..
rosdep install --from-paths src -iry
```

### 2.3 大工作空间构建时的内存问题

如果工作空间较大，或者同时启用了 Python 接口、FCL、额外第三方库，构建过程可能比较吃内存。

必要时可以临时扩大 swap：

```bash
sudo swapoff /swapfile
sudo dd if=/dev/zero of=/swapfile bs=1M count=16384 status=progress
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
free -h
```

### 2.4 编译工作空间

```bash
cd ~/your_ws
source /opt/ros/humble/setup.bash

colcon build --executor sequential \
  --symlink-install \
  --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_PYTHON_INTERFACE=ON \
  -DPINOCCHIO_USE_HPP_FCL=ON

source install/setup.bash
```

说明：

- `--executor sequential`：降低并行编译带来的内存压力
- `-DBUILD_PYTHON_INTERFACE=ON`：如果工作空间里包含 TSID / Pinocchio Python 接口需求，可保留
- `-DPINOCCHIO_USE_HPP_FCL=ON`：允许 Pinocchio 配合 FCL 进行相关支持

---

## 3. 为什么当前项目需要动力学前馈

如果 MIT 模式只使用：

```text
τ = Kp · (q_des - q) + Kd · (dq_des - dq)
```

那么在静止保持某个姿态时，电机需要靠位置误差来“顶住重力”。

这会带来三个直接问题：

### 3.1 保持依赖误差建立

系统不是“本来就能托住”，而是要先产生一个位置偏差，才能由 `Kp · e` 产生恢复力矩。

于是容易出现：

- 轨迹起步时先轻微下垂
- 目标位姿附近显得发软
- 为了托住机械臂，不得不把 `kp` 调大

### 3.2 刚度和补偿耦合

如果没有前馈，`kp` 同时承担：

- 保持目标位姿
- 抵消重力
- 抵抗外扰

这样会导致参数很难调。

### 3.3 柔顺控制空间被压缩

如果想实现“无外力时可保持，受外力时可被动跟随”，就需要让 `kp` 不至于过高。

但如果 `kp` 低了，单靠误差项又托不住大关节。

因此，需要引入动力学前馈，把“托住机械臂”这件事从 `kp` 身上剥离一部分。

---

## 4. 重力补偿与非线性项补偿的区别

### 4.1 重力补偿 `g(q)`

重力补偿只考虑当前姿态下，为了抵消重力所需的广义力：

```text
g(q)
```

它与速度无关，只和当前姿态有关。

适用场景：

- 静态保持
- 慢速轨迹跟踪
- 首先验证动力学模型方向是否正确
- 优先改善“软、塌陷、需要高 kp 才能托住”的问题

### 4.2 非线性项补偿 `nle(q, dq)`

非线性项一般包含：

```text
nle(q, dq) = C(q, dq)·dq + g(q)
```

也就是：

- 重力项
- 科氏项 / 离心项
- 其他速度相关非线性项

适用场景：

- 轨迹速度较高
- 加速减速明显
- 需要改善动态跟踪品质

### 4.3 当前项目建议的启用顺序

建议顺序始终是：

1. 先验证 **仅重力补偿**
2. 确认 `g(q)` 方向和量级正确
3. 再切换到 **非线性项补偿**

不要在动力学链路刚接通时就直接上 `nle(q, dq)`，否则问题定位会更困难。

---

## 5. 当前项目中的动力学实现链路

这一部分是本文核心。

---

## 6. `PinocchioDynamicsModel`：完整模型到 reduced model

### 6.1 为什么不能直接拿完整 URDF 求解

你当前机械臂的描述模型里，除了 `joint1 ~ joint6 + gripper_left` 之外，还有一个额外的 `gripper_right`，它主要用于：

- 碰撞模型完整性
- 几何显示语义完整性

但硬件接口实际只能提供 7 维受控状态：

- `joint1`
- `joint2`
- `joint3`
- `joint4`
- `joint5`
- `joint6`
- `gripper_left`

如果 Pinocchio 按完整 URDF 建出的是 8 维模型，而 `DmHardwareInterface` 每周期只传 7 维 `q / dq`，那动力学更新就会直接失败。

这就是之前“开关重力补偿几乎没差别”的核心原因之一：

- `update()` 一直失败
- `_gravity_feedforward_` 始终没被正确更新
- 最终喂给 MIT 的 `tau_ff` 实际上接近零

### 6.2 当前项目的实际做法：reduced model

当前实现不是手工删改 URDF，而是：

1. 先用完整 URDF 建 full model
2. 找出所有 **不在 `joint_names` 中的关节**
3. 将这些关节统一锁死
4. 基于 `pinocchio::buildReducedModel(...)` 构造 reduced model

这样做的优点：

- 碰撞 / 显示仍然保留完整 URDF
- 动力学模型维度严格与受控关节一致
- 不需要维护两套结构不同的 URDF

### 6.3 构造函数流程

当前 `PinocchioDynamicsModel` 的构造函数逻辑可概括为：

```cpp
pinocchio::Model full_model;
pinocchio::urdf::buildModel(urdf_path, full_model);

// 校验受控关节是否都存在且为 1 自由度
// ...

// 锁死所有不在 joint_names 中的关节
const Eigen::VectorXd q_ref = pinocchio::neutral(full_model);
_model_ = pinocchio::buildReducedModel(full_model, joints_to_lock, q_ref);
_data_ = std::make_shared<pinocchio::Data>(_model_);
```

之后又会建立：

- `_q_indices_`
- `_v_indices_`

用于把 `hardware interface` 顺序下的关节向量，映射到 Pinocchio 模型内部顺序。

### 6.4 这一层的工程意义

这一层解决的不是“补偿公式”，而是 **模型表达空间与控制状态空间对齐** 的问题。

如果这一步不对，后续的：

- `computeGeneralizedGravity(...)`
- `nonLinearEffects(...)`
- `crba(...)`

都会失去实际意义。

---

## 7. `update(q, dq)`：周期更新动力学量

当前 `update()` 的核心逻辑是：

```cpp
_q_.setZero();
_dq_.setZero();

for (...) {
    _q_[_q_indices_[i]] = q[i];
    _dq_[_v_indices_[i]] = dq[i];
}

pinocchio::computeGeneralizedGravity(_model_, *_data_, _q_);
_g_ = _data_->g;

pinocchio::nonLinearEffects(_model_, *_data_, _q_, _dq_);
_nle_ = _data_->nle;

pinocchio::crba(_model_, *_data_, _q_);
_m_q_ = _data_->M;
```

### 7.1 `computeGeneralizedGravity`

得到的是：

```text
g(q)
```

含义是：

- 当前姿态下
- 如果希望系统静态平衡
- 每个关节需要提供多少广义力矩去抵消重力

### 7.2 `nonLinearEffects`

得到的是：

```text
nle(q, dq)
```

它包含了：

- `g(q)`
- 速度相关非线性项

### 7.3 `crba`

得到质量矩阵：

```text
M(q)
```

当前项目虽然还没有在控制律中直接使用质量矩阵，但保留它是合理的，因为后续扩展会用到：

- 逆动力学
- 操作空间控制
- 惯量整形
- 更精细的阻抗调参

---

## 8. `DmHardwareInterface::read()`：状态更新与动力学更新

当前 `read()` 的流程可以理解成两步。

### 8.1 第一步：从电机读真实状态

```cpp
if(_refresh_state_in_read_) {
    _motor_controller_->refresh_motor_status(*_motors_[i]);
}

_hw_positions_[i] = _motors_[i]->get_position() / scale;
_hw_velocities_[i] = _motors_[i]->get_velocity() / scale;
```

含义是：

- 电机侧位置 / 速度反馈
- 经 `joint_to_motor_scale` 还原到关节空间
- 存入 `_hw_positions_ / _hw_velocities_`

所以，从 `JointTrajectoryController` 的角度看，它接触到的始终是 **关节空间状态**，不是电机编码器原始量。

### 8.2 第二步：用当前关节状态更新动力学模型

```cpp
if(_enable_dynamics_ && _dynamics_model_) {
    const bool ok = _dynamics_model_->update(_hw_positions_, _hw_velocities_);
    if(ok) {
        _gravity_feedforward_ = _dynamics_model_->get_gravity_std();
        _nonlinear_feedforward_ = _dynamics_model_->get_nonlinear_effects_std();
    }
}
```

这一步的意义是：

- `read()` 不仅刷新真实状态
- 还顺便把当前状态下的动力学量更新好
- 后续 `write()` 可以直接使用最新的 `tau_ff`

### 8.3 为什么放在 `read()` 而不是 `write()`

因为当前周期真正的真实状态来自 `read()`。

如果把动力学计算放到 `write()`，可能会混入：

- 上一个周期的状态
- 尚未刷新完成的状态缓存

把动力学更新放在 `read()`，语义更清晰：

> 先拿到真实状态，再基于真实状态计算前馈。

---

## 9. `DmHardwareInterface::write()`：如何把前馈叠加到 MIT

当前 `write()` 中，核心部分是：

```cpp
double tau_feedforward = 0.0;
if(_enable_dynamics_) {
    if(_enable_nonlinear_feedforward_) {
        tau_feedforward = _nonlinear_feedforward_[i];
    } else if(_enable_gravity_feedforward_) {
        tau_feedforward = _gravity_feedforward_[i];
    }
}
```

然后在 MIT 分支中调用：

```cpp
_motor_controller_->control_mit(
    *_motors_[i],
    static_cast<float>(_kp_),
    static_cast<float>(_kd_),
    static_cast<float>(cmd_motor),
    static_cast<float>(cmd_vel_motor),
    static_cast<float>(tau_feedforward));
```

### 9.1 这里的物理含义

MIT 控制命令本质上包含三部分：

1. 位置误差项
2. 速度误差项
3. 前馈扭矩项

当前控制器不是“只靠误差纠正”，而是：

- 先由 `tau_feedforward` 提供一部分理论所需扭矩
- 再由 `kp / kd` 去处理模型误差、剩余误差和外扰

### 9.2 为什么前馈只在 MIT 模式下更自然

因为 MIT 模式天然接受：

- `q_des`
- `dq_des`
- `tau_ff`

而 `POS_VEL` 更像一个较高封装的内置位置速度控制模式，对前馈扭矩支持并不天然。

所以当前项目中：

- 大部分动力学补偿讨论都围绕 MIT 模式
- `POS_VEL` 模式主要作为另一种控制模式或调试手段

---

## 10. 启动参数与控制器配置

---

## 10.1 `dm_arm_ros_control.urdf.xacro`

当前 Xacro 中，与动力学直接相关的参数有：

```xml
<xacro:arg name="enable_dynamics" default="true"/>
<xacro:arg name="enable_gravity_feedforward" default="true"/>
<xacro:arg name="enable_nonlinear_feedforward" default="false"/>
<xacro:arg name="urdf_path" default="$(find dm_arm_description)/urdf/DM-Arm-Description.urdf"/>
```

在 `<hardware>` 中会传递给插件：

```xml
<param name="enable_dynamics">$(arg enable_dynamics)</param>
<param name="enable_gravity_feedforward">$(arg enable_gravity_feedforward)</param>
<param name="enable_nonlinear_feedforward">$(arg enable_nonlinear_feedforward)</param>
<param name="urdf_path">$(arg urdf_path)</param>
```

这说明当前项目里：

- 动力学是否启用，是 launch/xacro 级别的开关
- 重力补偿和非线性项补偿，是互斥优先级选择
- Pinocchio 建模直接使用 `dm_arm_description` 中的 URDF

---

## 10.2 `controllers.yaml`

当前控制器配置是：

- `joint_state_broadcaster`
- `joint_trajectory_controller/JointTrajectoryController`

命令接口为：

```yaml
command_interfaces:
  - position
  - velocity
```

状态接口为：

```yaml
state_interfaces:
  - position
  - velocity
```

这意味着上层 JTC 下发的不是单独位置，而是：

- `q_des`
- `dq_des`

这与 MIT 模式的输入语义是匹配的。

---

## 10.3 launch 文件

当前 `dm_ros_control.launch.py` / `dm_ros_control_rviz.launch.py` 会将以下参数传入 Xacro：

- `serial_port`
- `baudrate`
- `kp`
- `kd`
- `enable_write`
- `refresh_state_in_read`
- `startup_read_cycles`
- `enable_dynamics`
- `enable_gravity_feedforward`
- `enable_nonlinear_feedforward`

所以，在不改代码的情况下，切换动力学模式只需要改 launch 参数即可。

---

## 11. 如何运行当前项目中的重力补偿

### 11.1 编译

```bash
cd ~/your_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select dm_hw dm_arm_description dm_ros_control
source install/setup.bash
```

### 11.2 启动真机：只开重力补偿

```bash
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

如果要连 RViz 一起起：

```bash
ros2 launch dm_ros_control dm_ros_control_rviz.launch.py \
  use_fake_hardware:=false \
  serial_port:=/dev/ttyACM0 \
  baudrate:=921600 \
  kp:=38.0 \
  kd:=1.5 \
  enable_dynamics:=true \
  enable_gravity_feedforward:=true \
  enable_nonlinear_feedforward:=false
```

### 11.3 发送测试轨迹

```bash
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper_left'],
  points: [
    {
      positions: [0.5, 0.4, 0.4, 0.4, 0.4, 0.4, -0.02],
      time_from_start: {sec: 2}
    }
  ]
}"
```

### 11.4 推荐先做的验证

第一轮先不要看“手感”，而是看以下现象：

- 新目标起步时，肩肘关节是否比无补偿更不容易先下垂
- 停在某个中间姿态时，是否比无补偿更能稳住
- 在相同 `kp / kd` 下，是否不再必须把 `kp` 调得特别高才能托住机械臂

---

## 12. 如何运行当前项目中的非线性项补偿

在确认重力补偿方向正确以后，再切换到 `nle(q, dq)`。

```bash
ros2 launch dm_ros_control dm_ros_control.launch.py \
  use_fake_hardware:=false \
  serial_port:=/dev/ttyACM0 \
  baudrate:=921600 \
  kp:=38.0 \
  kd:=1.5 \
  enable_dynamics:=true \
  enable_gravity_feedforward:=false \
  enable_nonlinear_feedforward:=true
```

这一模式下，前馈不仅包含重力，还包含与速度相关的非线性项。

适合观察：

- 运动中速度变化较大时，跟踪是否更顺
- 轨迹执行中是否减少额外拖拽感
- 加减速过程是否更自然

---

## 13. 如何判断补偿有没有真正生效

这一部分非常重要。

不要只靠主观感觉判断“好像有一点效果”。

### 13.1 先打印前馈数值

建议在 `read()` 中临时打印大关节的前馈量：

```cpp
RCLCPP_INFO_THROTTLE(
    rclcpp::get_logger("DmHardwareInterface"),
    *rclcpp::get_clock(),
    1000,
    "g_ff: [%.3f, %.3f, %.3f]",
    _gravity_feedforward_[0],
    _gravity_feedforward_[1],
    _gravity_feedforward_[2]);
```

如果在不同姿态下：

- 数值始终接近零
- 几乎不变化
- 或明显方向反了

那就说明链路仍然有问题。

### 13.2 做 A/B 对比

建议保持同一组 `kp / kd`，分别测试：

1. `enable_dynamics=false`
2. `enable_dynamics=true + gravity=true`
3. `enable_dynamics=true + nonlinear=true`

比较以下项目：

- 中间姿态保持能力
- 起步是否先塌
- 轨迹中途是否更稳
- 大关节是否更容易托住

### 13.3 “开关几乎没差别”通常意味着什么

如果开关补偿几乎没差别，优先排查：

1. `update()` 是否失败
2. reduced model 是否仍有维度不匹配
3. `tau_feedforward` 是否实际传到了 MIT 命令
4. 扭矩语义是否与电机控制语义一致
5. 模型质量、惯量、质心是否明显不准

---

## 14. `kp / kd / tau_ff` 在当前项目中的分工

### 14.1 开启前馈前

如果没有前馈：

- `kp` 既要负责保持目标
- 又要负责顶住重力
- 还要负责抵抗外扰

这会导致：

- 为了托住机械臂，`kp` 不得不调很大
- 系统会更硬
- 柔顺控制空间被压缩

### 14.2 开启重力补偿后

如果 `g(q)` 基本正确，那么“托住手臂”主要由：

```text
τ_ff ≈ g(q)
```

来承担。

这时 `kp` 的职责就变成：

- 约束目标位姿
- 处理模型误差
- 在受扰动后把关节拉回目标附近

这就是为什么开启补偿后，通常可以在更低 `kp` 下获得更好的保持效果。

### 14.3 `kd` 的作用

`kd` 本质上负责阻尼：

- 太低：像低阻尼弹簧，容易飘、容易回摆
- 太高：像被黏住，拖泥带水

因此，当前项目里合理的关系通常是：

- `tau_ff` 负责先把系统“托起来”
- `kp` 决定“有多硬”
- `kd` 决定“有多稳”

---

## 15. 建议的调试顺序

### 第一轮：只验证模型链路

目标：确认动力学真的在更新，不讨论手感。

步骤：

1. 开 `enable_dynamics=true`
2. 只开 `enable_gravity_feedforward=true`
3. 打印前馈值
4. 检查不同姿态下 `g(q)` 是否变化合理

### 第二轮：只验证静态保持

目标：确认重力补偿是否真的改善了保持。

步骤：

1. 让机械臂停在一个非零中间姿态
2. 比较有无补偿时的保持效果
3. 不做快速轨迹，不做大动作

### 第三轮：再验证动态轨迹

目标：确认 `nle(q, dq)` 是否改善动态跟踪。

步骤：

1. 先确保重力补偿正确
2. 切 `enable_nonlinear_feedforward=true`
3. 跑中等速度轨迹
4. 观察轨迹跟踪、加减速与平顺性
