# SerialArm Python Binding

SerialArm Python binding 由 `_serial_arm` pybind11 扩展和 `serial_arm` 纯 Python 包装层组成；离线接口直接复用 C++ Config、Core、Safety 和 Dynamics；真机接口通过 C++ `RobotSession` 工作线程维持控制周期

## 当前能力

- YAML 配置加载和验证
- Joint、Actuator、命令和故障结构
- `JointCtrller`
- `JointActuatorMapper`
- `Safety`
- `Dynamics` 集中更新和只读缓存
- NumPy 向量、矩阵、Jacobian 和位姿转换
- C++ 工作线程真机 `RobotSession`
- scikit-build-core wheel 构建

## 安全边界

> [!WARNING]
> `RobotSession` 能够通过 Hardware Backend 连接真实机械臂并发送执行器命令；使用前必须确认后端配置、零位、方向、限位、力矩映射、控制增益和动力学参数

真机运行必须满足

```text
runtime.write_enabled=true
已构建并安装对应 Hardware Backend
```

`runtime.write_enabled=false` 时 `RobotSession` 使用离线 mock 后端，不连接或写入真实硬件

Python 线程不直接执行 `Robot::cycle()`；C++ 工作线程是 Robot、Dynamics 和串口的唯一周期访问者

## 环境准备

```bash
export PATH=/opt/openrobots/bin:$PATH
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH

python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install scikit-build-core pybind11 numpy build
```

## 构建 wheel

从仓库的 `python/` 目录执行

```bash
cd python
python -m build --wheel
python -m pip install --force-reinstall dist/serial_arm-*.whl
python -c "import serial_arm; print(serial_arm.__version__)"
```

wheel 默认启用 Dynamics；真机运行还要求目标系统能够加载对应 Hardware Backend 共享库，以及 Pinocchio、coal、yaml-cpp 和对应 C++ 运行库

## 离线动力学

```python
from pathlib import Path

import numpy as np

import serial_arm

cfg = serial_arm.load_robot_cfg(
    "src/robot_supports/robots/dm_arm/description/config/dm_arm_gray.yaml",
    "serial_arm_hardware_damiao",
    "src/robot_supports/robots/dm_arm/description/config/dm_arm_damiao.yaml",
)
dynamics = serial_arm.Dynamics()
dynamics.configure(cfg.dynamics)

zero = np.zeros(len(cfg.joint_names), dtype=np.float64)
dynamics.update(zero, zero, zero, zero, zero)

print(dynamics.gravity)
print(dynamics.mass_matrix)
print(dynamics.tool_pose)
```

`Dynamics.update()` 集中刷新同一周期的运动学和动力学缓存；`gravity`、`mass_matrix`、`tool_pose` 等属性只返回独立 NumPy 副本

## 真机会话

```python
from pathlib import Path

import numpy as np

import serial_arm

session = serial_arm.RobotSession(
    "src/robot_supports/robots/dm_arm/description/config/dm_arm_gray.yaml",
    hardware_plugin="serial_arm_hardware_damiao",
    hardware_config="src/robot_supports/robots/dm_arm/description/config/dm_arm_damiao.yaml",
)
session.set_model_feedforward_mode(serial_arm.ModelFeedforwardMode.GRAVITY)
session.set_gravity_scale(np.array([0.0, 0.1, 0.2, 0.0, 0.0, 0.0]))

with session:
    session.set_impedance_mode(serial_arm.JointImpedanceMode.RIGID_TRACKING)
    session.move_to(np.array([0.0, 0.2, 0.2, 0.0, 0.0, 0.0]), speed_scale=0.2)
    snapshot = session.snapshot
    print(snapshot.cycle.joint_state.pos)
    print(snapshot.dynamics.gravity)
```

`set_model_feedforward_mode()` 只允许在 INACTIVE 状态调用；工作线程错误写入 `snapshot.last_error`；FAULT 必须在停止线程后显式调用 `reset_fault()`

## RobotSession 属性

| 属性 | 含义 |
|---|---|
| `snapshot` | 最近一次完整周期快照副本 |
| `state` | 当前 Robot 生命周期状态 |
| `configured` | 是否已经完成配置 |
| `running` | C++ 工作线程是否正在运行 |
| `config` | 静态配置副本 |
| `dynamics_info` | reduced model 静态信息 |
| `actuator_info` | Hardware Backend 提供的执行器能力 |

## 当前限制

- 当前固定六个受控关节
- Python API 仍处于 0.x 阶段
- 当前仓库没有 Python 自动化测试
- wheel 尚需在目标 Ubuntu、Python 和 Pinocchio 环境完成安装验证
- Python 真机会话尚需完成 30 min、1 h 和长期稳定性验证
- ROS 2 适配尚未实现

完整 C++ 和 Python API 见 [`../docs/API.md`](../docs/API.md)
