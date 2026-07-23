# DM-Arm 构建、真机联调与动力学调参教程

教程目标

- 完成依赖安装和构建
- 理解配置加载和运行链路
- 完成无前馈真机检查
- 读取 Joint、Actuator 和 Dynamics 全部状态
- 逐轴完成重力补偿调参
- 判断位置误差来自控制增益、映射还是模型
- 为 ROS 2 和 Python 适配准备稳定基线

## 1. 运行链路

当前真机控制周期

```text
DamiaoMotorBus::read()
        ↓
ActuatorState
        ↓
JointActuatorMapper::to_joint_state()
        ↓
JointState
        ↓
Safety::check_state()
        ↓
Robot 估计 joint_acc
        ↓
JointCtrller 生成初始 JointCtrlCmd
        ↓
Robot 估计 joint_ref_acc
        ↓
Dynamics::update(state, joint_acc, joint_ref_acc)
        ↓
NONE / GRAVITY / FULL_INVERSE_DYNAMICS
        ↓
JointCtrller::update()
        ↓
Safety::check_joint_cmd()
        ↓
JointActuatorMapper::to_actuator_cmd()
        ↓
DamiaoMotorBus::write()
```

动力学 getter 只读取最近一次成功 `update()` 的缓存；不会在查看状态时重新计算模型

## 2. 环境准备

### 2.1. 基础依赖

```bash
sudo apt update
sudo apt install -y build-essential cmake libyaml-cpp-dev libeigen3-dev
```

### 2.2. Pinocchio 环境

当前工程通过 `/opt/openrobots` 查找 Pinocchio

```bash
export PATH=/opt/openrobots/bin:$PATH
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
```

检查

```bash
ls /opt/openrobots/lib/libpinocchio_default.so
ls /opt/openrobots/lib/cmake/pinocchio/pinocchioConfig.cmake
```

长期使用可写入 `~/.bashrc`

```bash
cat >> ~/.bashrc <<'EOF_BASHRC'
export PATH=/opt/openrobots/bin:$PATH
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
EOF_BASHRC

source ~/.bashrc
```

### 2.3. 串口权限

检查设备

```bash
ls -l /dev/ttyACM0
```

将当前用户加入 `dialout`

```bash
sudo usermod -aG dialout "$USER"
```

重新登录后检查

```bash
groups
```

## 3. 构建

### 3.1. 清理旧构建

公共接口或 Pinocchio 环境变化后建议清理

```bash
rm -rf build
```

### 3.2. Debug 构建

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Debug \
  -DDM_ARM_BUILD_TERMINAL=ON \
  -DDM_ARM_BUILD_DAMIAO=ON \
  -DDM_ARM_ENABLE_DYNAMICS=ON \
  -DDM_ARM_BUILD_PYTHON=OFF \
  -DDM_ARM_BUILD_ROS2=OFF

cmake --build build -j"$(nproc)"
```

CMake 摘要应包含

```text
DM_ARM_BUILD_TERMINAL = ON
DM_ARM_BUILD_DAMIAO   = ON
DM_ARM_ENABLE_DYNAMICS= ON
DM_ARM_BUILD_PYTHON   = OFF
DM_ARM_BUILD_ROS2     = OFF
```

### 3.3. 链接检查

```bash
ldd build/dm_arm_terminal | grep -E "pinocchio|coal|yaml"
```

### 3.4. 安装检查

```bash
cmake --install build --prefix install
find install -maxdepth 4 -type f | sort
```

## 4. 配置文件说明

主配置文件

```text
config/dm_arm.yaml
```

### 4.1. Joint 顺序

```yaml
joints:
  names: [joint1, joint2, joint3, joint4, joint5, joint6]
```

该顺序必须与以下内容一致

- 控制器增益
- Safety 限制
- Mapping
- Dynamics joint names
- Damiao actuator 顺序
- 所有 JointVector

### 4.2. Runtime

```yaml
runtime:
  ctrl_frequency_hz: 200.0
  joint_acc_filter_alpha: 0.2
  write_enabled: true
  model_feedforward_mode: NONE
```

参数含义

| 参数 | 含义 | 调参建议 |
|---|---|---|
| `ctrl_frequency_hz` | Robot 目标周期频率 | 当前使用 200 Hz |
| `joint_acc_filter_alpha` | 加速度估计一阶低通系数 | 噪声大时减小；响应慢时增大 |
| `write_enabled` | 真机写入门禁 | 未确认安全前设为 false |
| `model_feedforward_mode` | 启动时模型前馈模式 | 首次测试使用 NONE |

加速度滤波形式

```text
acc_filtered = alpha × acc_raw + (1 - alpha) × acc_previous
```

### 4.3. Safety

```yaml
safety:
  cmd_timeout_s: 0.10
  state_timeout_s: 0.05
  max_dt_s: 0.02
  numeric_tolerance: 1.0e-6
  state_vel_fault_ratio: 1.5
  require_all_actuators_online: true
  require_all_actuators_enabled: true
```

Safety 不应作为轨迹生成器；明显非法命令应被拒绝，正常参考应由终端或未来轨迹控制器生成连续位置、速度和加速度

### 4.4. Joint 限制

```yaml
limits:
  min_pos: [...]
  max_pos: [...]
  max_vel: [...]
  max_acc: [...]
  max_effort: [...]
  max_kp: [...]
  max_kd: [...]
  pos_margin: [...]
```

注意

- `min_pos` 和 `max_pos` 是 Joint 状态硬边界
- 命令位置范围会叠加 `pos_margin`
- `max_vel` 和 `max_acc` 应是机械臂允许的 Joint 侧运行值
- 达妙协议最大速度不能直接作为日常机械臂运行速度
- `max_effort` 必须与 Joint 侧力矩映射一致

### 4.5. Mapping

```yaml
mapping:
  pos_ratio: [1, 1, 1, 1, 1, 1]
  tor_ratio: [1, 1, 1, 1, 1, 1]
  direction: [1, 1, 1, 1, 1, 1]
  joint_zero_offset: [0, 0, 0, 0, 0, 0]
  actuator_zero_offset: [0, 0, 0, 0, 0, 0]
```

位置映射必须逐轴确认

```text
q_actuator = direction × pos_ratio × (q_joint - joint_zero_offset) + actuator_zero_offset
```

力矩映射必须确认符号和比例；重力补偿方向错误时优先检查 `direction` 和 `tor_ratio`

### 4.6. Controller

控制器提供五组增益

- `rigid_hold`
- `rigid_tracking`
- `compliant_hold`
- `compliant_drag`
- `compliant_tracking`

没有重力补偿时，较大关节需要依靠位置误差产生托举力矩；表现为能够抬起但存在静态误差

加入重力补偿后，位置误差项主要负责跟踪和抗扰；重力项主要负责托举

### 4.7. Dynamics

```yaml
dynamics:
  urdf_path: ../description/urdf/dm_arm.urdf
  base_frame: base_link
  tool_frame: tool0
  gravity: [0.0, 0.0, -9.81]
  gravity_scale: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

`gravity_scale` 使用逐轴比例

```text
gravity_compensation[i] = gravity_scale[i] × gravity[i]
```

首次真机测试必须从 0 开始

### 4.8. Damiao

```yaml
damiao:
  serial_port: /dev/ttyACM0
  baudrate: 921600
  refresh_state_in_read: false
  startup_read_cycles: 5
  stop_kp: 3.0
  stop_kd: 0.10
  stop_cycles: 5
```

执行器列表必须确认

- motor ID
- master ID
- motor type
- Joint 对应关系

SDK 中的 `q_max`、`dq_max` 和 `tau_max` 是执行器侧唯一物理限制真源

## 5. 启动终端

```bash
./build/dm_arm_terminal --config config/dm_arm.yaml --allow-hardware
```

终端只支持真实达妙后端；没有 Fake 选项

首次启动前建议

```yaml
runtime:
  model_feedforward_mode: NONE

dynamics:
  gravity_scale: [0, 0, 0, 0, 0, 0]
```

## 6. 无动力学前馈基线检查

### 6.1. 配置检查

使用终端菜单查看完整配置摘要；确认

- 六个 Joint 名称正确
- 六个执行器 ID 正确
- 电机型号正确
- Joint 限位覆盖当前姿态
- `write_enabled` 状态符合计划
- Dynamics URDF 路径正确
- base Frame 和 tool Frame 正确

### 6.2. 激活

执行 `activate()`；成功后后台周期自动运行

若激活失败

- `JOINT_POS_LIMIT` 表示当前实测位置超过 Joint 硬限位
- `ACTUATOR_OFFLINE` 表示目标执行器没有有效反馈
- `ENABLE_FAILED` 表示达妙使能失败
- `MODE_SWITCH_FAILED` 表示 MIT 模式切换失败
- `STATE_TIMEOUT` 表示状态周期超时

### 6.3. 查看全部状态

使用终端状态菜单检查

Joint 侧

- `q`
- `dq`
- `ddq_est`
- `tau_feedback`
- `q_ref`
- `dq_ref`
- `ddq_ref`
- `model_feedforward`
- `kp`
- `kd`

Actuator 侧

- 位置
- 速度
- 力矩
- 在线状态
- 使能状态
- 错误码
- MIT 目标位置
- MIT 目标速度
- MIT 前馈力矩
- MIT `kp`
- MIT `kd`

### 6.4. 小幅运动

推荐顺序

1. 切换 `RIGID_TRACKING`
2. 输入小于 0.05 rad 的单轴目标
3. 速度比例使用 0.1
4. 观察 Joint 命令和实测状态
5. 确认方向正确后再扩大范围

判断方式

| 现象 | 优先检查 |
|---|---|
| `cmd.pos` 不变化 | 参考生成或模式 |
| `cmd.pos` 变化而 `actuator_cmd.pos` 不合理 | Mapping |
| `actuator_cmd.pos` 正确而实测反向 | direction 或零位 |
| 实测运动正确但静态误差大 | kp、重力补偿或摩擦 |
| 关节抖动 | kp 过高、kd 过低、反馈噪声或周期抖动 |
| 关节迟钝 | kp 过低、速度限制低、力矩不足或重力负载大 |

## 7. 动力学缓存检查

Dynamics 配置成功后，即使前馈模式为 `NONE`，Robot 仍会周期调用 `Dynamics::update()`

查看动力学菜单；确认

- `gravity` 为六维有限向量
- `nonlinear` 为六维有限向量
- `coriolis = nonlinear - gravity`
- `mass_matrix` 为 6×6
- `inverse_dynamics` 为六维有限向量
- `forward_dynamics` 为六维有限向量
- tool pose 为有限齐次矩阵
- tool Jacobian 为 6×6

静止状态下应近似满足

```text
dq ≈ 0
ddq_ref ≈ 0
nonlinear ≈ gravity
inverse_dynamics ≈ gravity
```

质量矩阵检查

```text
M ≈ Mᵀ
对角元素大于 0
```

RNEA 与 ABA 一致性检查

```text
tau = RNEA(q, dq, ddq_ref)
ddq_check = ABA(q, dq, tau)
ddq_check ≈ ddq_ref
```

## 8. 重力补偿调参

### 8.1. 调参目标

重力补偿目标不是让机械臂自动运动；目标是在静止或低速状态下减少维持姿态所需的位置误差

没有重力补偿

```text
tau_cmd = kp × position_error + kd × velocity_error
```

加入重力补偿

```text
tau_cmd = kp × position_error + kd × velocity_error + gravity_scale ⊙ g(q)
```

### 8.2. 调参前确认

必须确认

- URDF 重力方向正确
- Joint 顺序正确
- Joint 位置方向正确
- Joint 力矩方向正确
- 当前姿态处于安全范围
- `max_effort` 不会过早截断所需补偿
- 急停和支撑可用

### 8.3. 只观察不下发

保持

```yaml
runtime:
  model_feedforward_mode: NONE

dynamics:
  gravity_scale: [0, 0, 0, 0, 0, 0]
```

激活后查看 `gravity`

重点观察 joint2 和 joint3

- 姿态变化时数值应连续
- 同一姿态重复读取应稳定
- 机械臂越接近水平伸展，承担重力的关节力矩通常越大
- 关节方向翻转后，重力力矩符号可能改变

### 8.4. 单轴低比例测试

先停机并回到 `INACTIVE`；使用终端设置

```text
[0, 0, 0.05, 0, 0, 0]
```

切换前馈模式为 `GRAVITY`；重新激活

观察

- joint3 是否向正确方向获得托举
- `model_feedforward[2]` 是否等于 `0.05 × gravity[2]`
- 关节是否突然加速
- 电机是否出现异常声音或错误码

方向错误时立即失能；不要通过增加比例继续试错

### 8.5. 比例递增

建议序列

```text
0.05
0.10
0.15
0.20
0.30
0.40
0.50
```

每个比例记录

- 目标位置
- 实际位置
- 静态误差
- `gravity`
- `gravity_compensation`
- 最终 Joint 前馈力矩
- Actuator MIT 前馈力矩
- 电机温度
- 是否振荡
- 是否下沉

### 8.6. joint3 判据

假设固定目标 `q_ref`

```text
error_none = |q_ref - q_actual| when mode = NONE
error_gravity = |q_ref - q_actual| when mode = GRAVITY
```

有效补偿应满足

```text
error_gravity < error_none
```

同时不应出现

- 高频振荡
- 静态持续加速
- 力矩饱和
- 明显过补偿
- 关节越过目标后继续抬升

### 8.7. joint2 与 joint3 联合调参

joint3 单轴方向确认后再加入 joint2

```text
[0, 0.05, 0.10, 0, 0, 0]
```

联合调参时注意耦合；改变 joint2 会改变 joint3 后续连杆的空间姿态和重力力矩

### 8.8. 降低 kp 验证解耦效果

重力补偿稳定后逐步降低 `rigid_hold` 或 `compliant_hold` 的 kp

判据

- 静态误差仍可接受
- 外力下更柔顺
- 释放外力后能够回到目标附近
- 不依赖极高 kp 才能托住

### 8.9. COMPLIANT_HOLD

推荐顺序

1. `GRAVITY` 模式
2. `RIGID_HOLD` 确认补偿稳定
3. 切换 `COMPLIANT_HOLD`
4. 观察下沉和回位
5. 调整 compliant kp 和 kd

### 8.10. COMPLIANT_DRAG

`COMPLIANT_DRAG` 的 kp 为 0 时不提供位置恢复；重力补偿负责托举，kd 负责阻尼

调参重点

- 重力比例不足时会下沉
- 重力比例过大时会自行抬升
- kd 太低时拖拽后摆动
- kd 太高时拖拽阻力大

## 9. FULL_INVERSE_DYNAMICS

当前 Robot 通过命令速度差分估计 `joint_ref_acc`

```text
joint_ref_acc = (dq_ref_current - dq_ref_previous) / dt
```

`FULL_INVERSE_DYNAMICS` 使用

```text
tau_ff = RNEA(q, dq, joint_ref_acc)
```

启用前必须先完成

- 重力补偿方向确认
- URDF 惯量审计
- 参考加速度噪声检查
- `max_effort` 检查
- 低速轨迹验证

首次测试建议

- 单轴
- 小幅目标
- 低速度比例
- 低加速度
- 记录 `joint_ref_acc` 和 RNEA 输出

若 RNEA 输出尖峰明显，优先降低参考加速度噪声；不要直接扩大力矩限制

## 10. URDF 动力学参数审计

每个运动 link 应检查

- 质量单位为 kg
- 质心相对于 link Frame 的方向正确
- 惯量单位为 kg·m²
- 惯量矩阵对称
- 主对角元素大于 0
- 惯量满足物理可实现条件
- 末端工具、夹爪和相机质量没有遗漏或重复

推荐表格

| Link | CAD 质量 | URDF 质量 | CAD 质心 | URDF 质心 | 惯量来源 | 状态 |
|---|---:|---:|---|---|---|---|
| link1 |  |  |  |  |  |  |
| link2 |  |  |  |  |  |  |
| link3 |  |  |  |  |  |  |
| link4 |  |  |  |  |  |  |
| link5 |  |  |  |  |  |  |
| link6 |  |  |  |  |  |  |
| tool |  |  |  |  |  |  |

## 11. 常见问题

### 11.1. joint3 能抬起但到不了目标

可能原因

- kp 仍不足
- 重力补偿未启用
- `gravity_scale` 太低
- URDF 质量或质心偏小
- 前馈力矩方向错误
- `max_effort` 限制过低
- `tor_ratio` 不正确
- 机械摩擦或结构卡滞

排查顺序

1. 比较 `q_ref` 和 `q`
2. 查看 `gravity[2]`
3. 查看 `gravity_compensation[2]`
4. 查看最终 `joint_cmd.tor[2]`
5. 查看 `actuator_cmd.tor[2]`
6. 检查 `tor_ratio` 和方向
7. 逐步增加补偿比例
8. 最后再调整 kp

### 11.2. 重力补偿后关节向错误方向运动

优先检查

- Joint 位置方向
- Joint 力矩方向
- `direction`
- `tor_ratio`
- URDF 关节轴方向
- 世界重力方向

### 11.3. `JOINT_POS_LIMIT`

实测位置超过硬限位；应先确认零位和实际机械范围，不应通过 `numeric_tolerance` 掩盖明显偏差

### 11.4. `CMD_POS_STEP_LIMIT`

相邻命令位置变化超过允许值；检查轨迹参考是否连续、目标重规划是否从上一帧参考开始、周期 dt 是否一致

### 11.5. `CMD_VEL_STEP_LIMIT`

相邻命令速度变化超过 `max_acc × dt`；检查轨迹末端是否突然清零速度、参考切换是否保留上一帧速度

### 11.6. `STATE_TIMEOUT`

检查

- 串口读取时间
- `state_timeout_s`
- 周期是否被终端输入阻塞
- 激活后的首帧时间基准
- `refresh_state_in_read`

### 11.7. `MOTOR_BUS_ACTIVATE_FAILED`

检查子错误

- `ENABLE_FAILED`
- `MODE_SWITCH_FAILED`
- `WRITE_FAILED`
- `ACTUATOR_OFFLINE`

FAULT 后必须先执行 `reset_fault()`；恢复流程会重建串口和电机对象

## 12. 数据记录建议

每次真机调参记录

```text
timestamp
mode
q_ref
q
dq
ddq_est
tau_feedback
model_feedforward
gravity
gravity_scale
joint_cmd_tor
actuator_cmd_tor
kp
kd
motor_err_code
```

建议为每个姿态保存

- 配置文件副本
- URDF commit
- 软件 commit
- 机械负载
- 工具质量
- 测试视频
- 急停和支撑方式

## 13. Python binding 构建与使用

### 13.1. 创建虚拟环境

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install scikit-build-core pybind11 numpy build
```

虚拟环境目录名称由 `python3 -m venv` 的最后一个参数决定；`.venv` 只是当前仓库推荐名称

### 13.2. 构建 wheel

```bash
cd python
python -m build --wheel
python -m pip install --force-reinstall dist/dm_arm-*.whl
python -c "import dm_arm; print(dm_arm.__version__)"
```

构建前必须保留 Pinocchio 的 `/opt/openrobots` 环境变量；wheel 默认启用 Dynamics 和 Damiao

### 13.3. 离线 Dynamics 检查

```python
from pathlib import Path

import numpy as np

import dm_arm

cfg = dm_arm.load_robot_cfg(Path("../config/dm_arm.yaml"))
dynamics = dm_arm.Dynamics()
dynamics.configure(cfg.dynamics)

zero = np.zeros(6, dtype=np.float64)
dynamics.update(zero, zero, zero, zero, zero)

print(dynamics.gravity)
print(dynamics.mass_matrix)
print(dynamics.tool_pose)
```

检查结果

- `gravity.shape == (6,)`
- `mass_matrix.shape == (6, 6)`
- `tool_pose.shape == (4, 4)`
- 修改返回数组不会改变 C++ Dynamics 缓存

### 13.4. 真机 RobotSession

```python
from pathlib import Path

import numpy as np

import dm_arm

session = dm_arm.RobotSession(Path("../config/dm_arm.yaml"), allow_hardware=True)
session.set_model_feedforward_mode(dm_arm.ModelFeedforwardMode.GRAVITY)
session.set_gravity_scale(np.array([0.0, 0.1, 0.2, 0.0, 0.0, 0.0]))

with session:
    session.set_impedance_mode(dm_arm.JointImpedanceMode.RIGID_TRACKING)
    session.move_to(np.array([0.0, 0.2, 0.2, 0.0, 0.0, 0.0]), speed_scale=0.2)
    print(session.snapshot.cycle.joint_state.pos)
```

真机启动必须同时满足

```text
allow_hardware=True
runtime.write_enabled=true
DM_ARM_BUILD_DAMIAO=ON
```

`set_model_feedforward_mode()` 必须在 `start()` 前调用；运行期间通过 `snapshot.last_error` 检查工作线程错误；FAULT 复位前先确保工作线程已经停止
