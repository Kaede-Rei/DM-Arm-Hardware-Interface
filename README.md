<div align="center">

# DM-Arm Hardware Interface

DM-Arm Hardware Interface 是面向六轴 DM-Arm 机械臂的独立 C++17 控制平台；仓库提供关节控制、Joint 与 Actuator 映射、安全检查、Robot 生命周期、达妙硬件后端、Pinocchio 运动学与动力学、重力补偿和真机终端

</div>

---

## 特性

- 五种关节阻抗模式
- 三种受约束关节参考命令
- Joint 与 Actuator 双向映射
- 状态、命令、超时、限位和突变检查
- Robot 生命周期、FAULT 锁存和故障刚性保持
- 达妙 MIT 串口后端
- Pinocchio reduced model
- 集中式动力学 `update()` 和只读缓存 getter
- 重力项、非线性项、科氏离心项和质量矩阵
- RNEA 逆动力学和 ABA 正动力学
- Frame 位姿、末端位姿和 Jacobian
- 关节加速度估计和参考加速度估计
- 真机交互式终端
- CMake 安装和 package 导出
- pybind11 Python 离线 API
- NumPy 状态、命令、矩阵和位姿转换
- C++ 后台线程真机 `RobotSession`
- scikit-build-core wheel 构建

---

## 当前状态

| 模块 | 状态 | 当前能力 |
|---|---|---|
| `dm_arm::core` | 已实现 | 类型、阻抗控制、映射和 Safety |
| `dm_arm::config` | 已实现 | 使用 yaml-cpp 加载和验证配置 |
| `dm_arm::robot` | 已实现 | 生命周期、周期闭环、故障处理和模型前馈 |
| `dm_arm::damiao` | 已实现 | 串口、MIT、状态读取、使能、停止、失能和恢复 |
| `dm_arm::dynamics` | 已实现 | FK、Frame、Jacobian、重力、质量矩阵、RNEA 和 ABA |
| `dm_arm_terminal` | 已实现 | 真机控制、状态观测、动力学观测和补偿调参 |
| ROS 2 / ros2_control | 未实现 | 构建开关存在；开启时会主动报错 |
| Python binding | 已实现 | Config、Core、Dynamics、NumPy API、真机 RobotSession 和 wheel |
| 自动化测试 | 未实现 | 待所有完成再统一测试 |
| 独立诊断工具 | 未加入 | 当前主要通过终端完成联调 |

动力学代码已经完成；当前仍需完成 URDF 惯量审计、Joint 与 Actuator 力矩映射确认、`gravity_scale` 调参、真机补偿验证和长期运行验证

---

## 架构

```text
上层命令或终端
      ↓
    Robot
    ├── MotorBus::read()
    ├── JointActuatorMapper::to_joint_state()
    ├── Safety::check_state()
    ├── 关节加速度与参考加速度估计
    ├── Dynamics::update()
    ├── JointCtrller::update()
    ├── Safety::check_joint_cmd()
    ├── JointActuatorMapper::to_actuator_cmd()
    └── MotorBus::write()
```

模块依赖关系

```text
                         dm_arm::core
                       /      |      \
                      /       |       \
          dm_arm::config  dm_arm::robot  dm_arm::damiao
                |             |
                |       dm_arm::dynamics
                |             |
                ├──── dm_arm_terminal
                └──── Python _dm_arm
```

核心边界

- `JointCtrller` 只处理 Joint 侧控制语义
- `JointActuatorMapper` 只处理比例、方向和零位映射
- `Safety` 只检查状态和命令并给出动作建议
- `Robot` 统一组织生命周期和周期闭环
- `DamiaoMotorBus` 只处理达妙通信和执行器侧限制
- `Dynamics` 只处理模型计算和缓存
- 终端只负责交互、参考生成和观测
- Python 离线 API 只包装公开 C++ 接口
- Python 真机会话由 C++ 工作线程独占 Robot 和串口；Python 线程只提交请求和读取快照

---

## 安全警告

> [!WARNING]
> 本项目能够向真实机械臂发送 MIT 命令；错误的零位、方向、比例、限位、增益、惯量、重力方向或电机型号可能导致突然运动、碰撞、过流、坠落或机构损坏

真机运行前必须完成

1. 准备独立急停和可靠支撑
2. 确认六个电机 ID、master ID 和型号
3. 逐轴确认位置与速度反馈方向
4. 标定 `direction`、`pos_ratio`、`tor_ratio` 和零位偏移
5. 依据真实机械范围设置 Joint 硬限位和命令边距
6. 从低速度、低补偿比例和安全姿态开始测试
7. 确认停放姿态、立即失能、故障停止和 `recover()` 的真实行为
8. 确认 `shutdown.park_pos` 无碰撞且失能后能够被机械结构承托
9. 在未确认真机安全前保持 `write_enabled: false`

本项目不能替代工业安全控制器、硬限位、制动器、急停回路和正式风险评估

---

## 目录结构

```text
.
├── app/
│   └── dm_arm_terminal.cpp
├── cmake/
│   └── dm_arm_coreConfig.cmake.in
├── config/
│   └── dm_arm.yaml
├── description/
│   └── urdf/dm_arm.urdf
├── include/dm_arm/
│   ├── config/config.hpp
│   ├── core/
│   │   ├── joint_actuator_mapper.hpp
│   │   ├── joints_ctrller.hpp
│   │   ├── safety.hpp
│   │   └── types.hpp
│   ├── dynamics/dynamics.hpp
│   ├── hardware/
│   │   ├── damiao_motor_bus.hpp
│   │   └── motor_bus.hpp
│   └── robot.hpp
├── python/
│   ├── bindings.cpp
│   ├── robot_session.hpp
│   ├── robot_session.cpp
│   ├── dm_arm/__init__.py
│   ├── README.md
│   └── pyproject.toml
├── src/
│   ├── config/config.cpp
│   ├── core/
│   ├── dynamics/dynamics.cpp
│   ├── hardware/damiao_motor_bus.cpp
│   └── robot.cpp
├── third_lib/
│   ├── damiao/
│   └── tl/
├── CMakeLists.txt
├── package.xml
├── README.md
└── Tutorial.md
```

---

## 平台与依赖

当前开发目标平台

- Ubuntu 22.04
- GCC 11 或兼容编译器
- CMake 3.20 及以上
- C++17

基础依赖

* yaml-cpp
* Eigen3
* Pinocchio
* pthread
* Linux termios

其中 pthread 和 termios 由 Linux 基础工具链及 C 标准库提供，无需安装独立开发包

### Ubuntu 基础依赖

```bash
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    pkg-config \
    lsb-release \
    curl \
    libyaml-cpp-dev \
    libeigen3-dev
```

### 安装 Pinocchio

Ubuntu 22.04 推荐通过 Pinocchio 官方使用的 robotpkg 软件源安装；该方式会自动安装 Pinocchio 及其所需依赖，并将软件安装到 `/opt/openrobots`

首先安装 robotpkg 软件源所需工具

```bash
sudo apt install -y lsb-release curl
```

创建 APT 密钥目录并导入 robotpkg 仓库密钥

```bash
sudo mkdir -p /etc/apt/keyrings

curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
    | sudo tee /etc/apt/keyrings/robotpkg.asc > /dev/null
```

添加与当前 Ubuntu 发行版匹配的 robotpkg 软件源

```bash
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
    | sudo tee /etc/apt/sources.list.d/robotpkg.list
```

更新软件源并安装 Pinocchio

```bash
sudo apt update
sudo apt install -y robotpkg-py3*-pinocchio
```

Pinocchio 官方为 Ubuntu 20.04、22.04 和 24.04 提供 robotpkg 二进制包；安装位置默认为 `/opt/openrobots`

### 配置 Pinocchio 环境

当前项目只使用 Pinocchio C++ 接口，至少需要配置以下环境变量

```bash
export PATH=/opt/openrobots/bin:$PATH
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
```

需要使用 Pinocchio Python 接口时，Ubuntu 22.04 还可增加

```bash
export PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH
```

官方文档同样要求将 `/opt/openrobots` 下的可执行文件、动态库、pkg-config 文件、CMake 包配置和可选 Python 模块加入对应环境变量

为了使配置永久生效，可将环境变量写入 `~/.bashrc`

```bash
cat >> ~/.bashrc <<'EOF'

# Pinocchio and robotpkg
export PATH=/opt/openrobots/bin:$PATH
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
EOF

source ~/.bashrc
```

如需使用 Python 接口，再单独追加

```bash
echo 'export PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH' \
    >> ~/.bashrc

source ~/.bashrc
```

### 验证 Pinocchio 安装

检查 pkg-config 是否能够找到 Pinocchio

```bash
pkg-config --modversion pinocchio
```

检查 CMake 包配置文件

```bash
find /opt/openrobots -path "*pinocchio*Config.cmake" \
    -o -path "*pinocchio-config.cmake"
```

---

## 构建

### 完整真机版本

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DDM_ARM_BUILD_TERMINAL=ON \
  -DDM_ARM_BUILD_DAMIAO=ON \
  -DDM_ARM_ENABLE_DYNAMICS=ON \
  -DDM_ARM_BUILD_PYTHON=OFF \

cmake --build build -j"$(nproc)"
```

### Debug 版本

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j"$(nproc)"
```

### 安装

```bash
cmake --install build --prefix install
```

---

## CMake 选项

| 选项 | 默认值 | 状态 |
|---|---:|---|
| `DM_ARM_BUILD_TERMINAL` | `ON` | 已实现；要求 Damiao 和 Dynamics 同时开启 |
| `DM_ARM_BUILD_DAMIAO` | `ON` | 已实现 |
| `DM_ARM_ENABLE_DYNAMICS` | `ON` | 已实现 |
| `DM_ARM_BUILD_PYTHON` | `ON` | 已实现；要求 Dynamics，Damiao 开启时额外提供真机 RobotSession |

---

## 正常停机

正常停机由终端应用层执行连续停放轨迹；底层 `Robot::deactivate()` 和 `MotorBus::deactivate()` 仍保持立即停止并失能语义

```yaml
shutdown:
  park_before_disable: true
  park_pos: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  speed_scale: 0.10
  position_tolerance: 0.03
  velocity_tolerance: 0.05
  settle_time_s: 0.25
  relaxed_tolerance_ratio: 2.0
  timeout_s: 15.0
```

严格判据超时但实测位置和速度满足宽松判据时，终端会打印最差关节并继续执行保持与失能；如果宽松判据仍不满足，终端保持 ACTIVE 和 `RIGID_HOLD`，不会自动失能

`park_pos` 必须根据真实机械结构确认；没有抱闸的机械臂只有在停放姿态能够被机械结构承托时，失能后才不会继续下落

---

## 运行终端

终端只支持真实达妙后端；启动命令

```bash
./build/dm_arm_terminal --config config/dm_arm.yaml --allow-hardware
```

`--allow-hardware` 只表示用户明确允许程序访问真实硬件；YAML 中仍需设置

```yaml
runtime:
  write_enabled: true
```

推荐首次运行顺序

1. 将 `gravity_scale` 全部设为 0
2. 保持 `model_feedforward_mode: NONE`
3. 启动终端并查看配置摘要
4. 执行 `activate()`
5. 查看 Joint 和 Actuator 状态
6. 查看动力学向量和矩阵
7. 执行小幅跟踪运动
8. 使用菜单 3 回到停放姿态并失能
9. 逐轴增加重力补偿比例
10. 切换到 `GRAVITY` 后重新测试

---

## 终端功能

当前菜单包含

- Robot 状态和 getter 输出
- `activate()`、回停放姿态失能、立即失能和 `reset_fault()`
- 五种阻抗模式切换
- 三种模型前馈模式切换
- 绝对位置和相对位置梯形参考
- `JointPosCmd`、`JointPosVelCmd` 和 `JointPosVelTorCmd`
- `set_full_cmd()`
- Joint 全部周期状态
- Actuator 全部周期状态
- 动力学向量和末端位姿
- 质量矩阵和末端 Jacobian
- 达妙执行器静态参数
- 运行时重力补偿比例调整
- 指定 Frame 的缓存位姿和 Jacobian

---

## 配置

主配置文件为 `config/dm_arm.yaml`

主要分区

```yaml
joints:
runtime:
safety:
limits:
mapping:
controller:
dynamics:
damiao:
```

重要原则

- `joints.names` 是六轴顺序真源
- 所有 JointVector 必须保持相同顺序
- `limits.max_vel` 和 `limits.max_acc` 是 Joint 侧运行限制；不能直接照搬电机协议最大值
- `mapping` 必须通过真机逐轴标定
- `dynamics.gravity_scale` 应从 0 开始逐轴调整
- 达妙电机物理限制以 SDK 为唯一真源
- YAML 只在启动阶段读取；周期中不读取文件

完整配置说明见 [Tutorial.md](Tutorial.md)

---

## 控制模式

| 模式 | 外部命令 | 目标位置语义 | 典型用途 |
|---|---:|---|---|
| `RIGID_HOLD` | 否 | 切换时实际位置 | 刚性保持 |
| `RIGID_TRACKING` | 是 | 外部参考 | 常规轨迹跟踪 |
| `COMPLIANT_HOLD` | 否 | 切换时实际位置 | 低刚度保持 |
| `COMPLIANT_DRAG` | 否 | 每周期实际位置 | 手动拖拽 |
| `COMPLIANT_TRACKING` | 是 | 外部参考 | 柔顺跟踪 |

模型前馈模式

| 模式 | 前馈输出 |
|---|---|
| `NONE` | 零向量；Dynamics 仍可更新缓存 |
| `GRAVITY` | `gravity_scale ⊙ g(q)` |
| `FULL_INVERSE_DYNAMICS` | `RNEA(q, dq, ddq_ref)` |

---

## 动力学接口

标准周期调用

```cpp
const auto result = dynamics.update(joint_state, joint_acc, joint_ref_acc);
if(!result) {
    return;
}

const auto& state = dynamics.get_state();
const auto& gravity = dynamics.get_gravity();
const auto& mass_matrix = dynamics.get_mass_matrix();
```

`update()` 集中计算并刷新同一周期缓存；getter 只返回最近一次成功更新的只读缓存，不重复执行 Pinocchio 算法

当前缓存包括

- `q`、`dq`、`ddq_est`、反馈力矩和参考加速度
- `g(q)`
- 缩放后的重力补偿
- `nle(q, dq)`
- 科氏和离心项
- `M(q)`
- RNEA 输出
- ABA 输出
- 所有 Frame 位姿和 Jacobian
- tool Frame 位姿和 Jacobian

---

## Python binding

Python 模块由 `_dm_arm` C++ 扩展和 `dm_arm` 纯 Python 包装层组成

离线接口包括

- `load_robot_cfg()` 和完整配置结构
- `JointCtrller`
- `JointActuatorMapper`
- `Safety`
- `Dynamics`
- NumPy 状态、命令、质量矩阵、Jacobian 和位姿

真机接口使用 `RobotSession`；200 Hz 周期在 C++ 工作线程中执行，Python 不直接调度 `Robot::cycle()`

创建虚拟环境并构建 wheel

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install scikit-build-core pybind11 numpy build

cd python
python -m build --wheel
python -m pip install dist/dm_arm-*.whl
```

离线动力学示例

```python
from pathlib import Path

import numpy as np

import dm_arm

repo = Path.cwd()
cfg = dm_arm.load_robot_cfg(repo / "config" / "dm_arm.yaml")
dynamics = dm_arm.Dynamics()
dynamics.configure(cfg.dynamics)

zero = np.zeros(6, dtype=np.float64)
dynamics.update(zero, zero, zero, zero, zero)

print(dynamics.gravity)
print(dynamics.mass_matrix)
print(dynamics.tool_pose)
```

真机会话示例

```python
from pathlib import Path

import numpy as np

import dm_arm

config_file = Path("config/dm_arm.yaml")
session = dm_arm.RobotSession(config_file, allow_hardware=True)
session.set_model_feedforward_mode(dm_arm.ModelFeedforwardMode.GRAVITY)
session.set_gravity_scale(np.array([0.0, 0.1, 0.2, 0.0, 0.0, 0.0]))

with session:
    session.set_impedance_mode(dm_arm.JointImpedanceMode.RIGID_TRACKING)
    session.move_to(np.array([0.0, 0.3, 0.3, 0.0, 0.0, 0.0]), speed_scale=0.2)
    snapshot = session.snapshot
    print(snapshot.cycle.joint_state.pos)
```

真机启动同时要求构造参数 `allow_hardware=True` 和 YAML 中 `runtime.write_enabled: true`；模型前馈模式只能在会话处于 INACTIVE 时修改

## 在其他 CMake 项目中使用

安装后

```cmake
find_package(dm_arm_core REQUIRED)

target_link_libraries(your_app PRIVATE
    dm_arm::core
    dm_arm::config
    dm_arm::robot
    dm_arm::damiao
    dm_arm::dynamics
)
```

可用 targets

```text
dm_arm::core
dm_arm::config
dm_arm::robot
dm_arm::damiao
dm_arm::dynamics
```

详细 API 见 [API.md](API.md)
Python 构建与使用见 [python/README.md](python/README.md)

---

## 已知限制

- 当前固定六个受控旋转关节
- ROS 2 / ros2_control 尚未实现
- 当前尚无 Python、C++ 和硬件在环自动化测试
- 当前终端仅面向真实达妙后端
- URDF 动力学参数尚需系统审计和标定
- `gravity_scale` 尚需逐轴真机调参
- 关节反馈加速度来自速度差分和低通滤波
- 参考加速度来自命令速度差分
- `FULL_INVERSE_DYNAMICS` 的效果依赖参考加速度质量
- 当前控制循环没有实时线程优先级和 CPU 亲和性配置

---

## 贡献

提交修改前建议完成

- 保持 C++17
- 保持现有命名和格式规范
- 函数定义参数不拆行
- 短函数调用不随意拆行
- 不在周期路径读取 YAML
- 不在 getter 中重复计算动力学
- 不复制达妙 SDK 的电机限制表
- 新接口同步更新 README、Tutorial 和 API 文档
- 真机相关修改说明测试姿态、速度、增益和安全措施

推荐流程

```bash
git checkout -b feature/your-change
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j"$(nproc)"
git status
git diff
```

---

## 许可证

项目使用 MIT License；详见 [LICENSE](LICENSE)
