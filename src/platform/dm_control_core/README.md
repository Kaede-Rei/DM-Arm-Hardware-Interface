# dm_control_core 接口文档

`dm_control_core` 是机械臂控制链中的 ROS 无关核心库；它不依赖 `rclcpp`、`hardware_interface` 或 ROS 消息，目标是把关节空间控制语义、动力学观测和达妙总线换算从 `dm_ros_control` 中分离出来，便于单元测试和后续仿真后端复用

## 模块组成

```text
dm_control_core/
├── include/dm_control_core/
│   ├── joint_control_types.hpp
│   ├── joint_impedance_controller.hpp
│   ├── dynamics_observer.hpp
│   ├── pinocchio_dynamics_model.hpp
│   └── dm_motor_bus.hpp
├── src/
│   ├── joint_impedance_controller.cpp
│   ├── dynamics_observer.cpp
│   ├── pinocchio_dynamics_model.cpp
│   └── dm_motor_bus.cpp
└── test/
    └── test_joint_impedance_controller.cpp
```

主要类：

| 类 | 头文件 | 作用 |
|---|---|---|
| `JointImpedanceController` | `joint_impedance_controller.hpp` | 上层关节命令到 MIT 命令的纯 C++ 转换 |
| `DynamicsObserver` | `dynamics_observer.hpp` | 基于动力学模型生成重力、非线性和外力观测 |
| `PinocchioDynamicsModel` | `pinocchio_dynamics_model.hpp` | 从 URDF 构建 reduced model 并计算动力学项 |
| `DmMotorBus` | `dm_motor_bus.hpp` | 达妙串口总线管理、状态读取、命令写入和单位换算 |

## 数据类型

### `JointState`

关节侧观测状态，所有数组按 `JointControlLayout::joint_names` 对齐

```cpp
struct JointState {
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> effort;
    std::vector<double> motor_effort;
};
```

- `position`：关节位置，单位 rad 或 m，取决于关节类型
- `velocity`：关节速度
- `effort`：关节侧力矩或力
- `motor_effort`：电机侧原始力矩，无法提供时可为 0

### `JointCommand`

上层命令语义；`optional` 用来区分“未提供”和“提供了 0”

```cpp
struct JointCommand {
    JointCommandMode mode{JointCommandMode::HOLD};
    tl::optional<std::vector<double>> position;
    tl::optional<std::vector<double>> velocity;
    tl::optional<std::vector<double>> effort;
};
```

命令模式：

| 模式 | 必需字段 | 输出语义 |
|---|---|---|
| `HOLD` | 无 | 保持锁存位置，速度为 0 |
| `POSITION` | `position` | 跟踪位置，速度目标为 0 |
| `POSITION_VELOCITY` | `position`、`velocity` | 跟踪位置和速度 |
| `IMPEDANCE` | `position`、`velocity`、`effort` | 跟踪位置速度，并叠加残差力矩 |
| `VELOCITY` | `velocity` | 当前位置作为位置参考，`kp=0` |
| `TORQUE` | `effort` | 当前位置作为参考，`kp=0`、`kd=0`，不叠加模型前馈 |

### `MitJointCommand`

发送到达妙 MIT 模式的关节侧命令

```cpp
struct MitJointCommand {
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> effort;
    std::vector<double> kp;
    std::vector<double> kd;
};
```

`DmMotorBus` 在写入时会把关节侧位置、速度、力矩转换到电机侧：

```text
motor_position = joint_position * joint_to_motor_scale
motor_velocity = joint_velocity * joint_to_motor_scale
motor_tau      = joint_tau / joint_to_motor_scale
```

## JointImpedanceController

### 配置

```cpp
JointImpedanceController controller;

JointImpedanceControllerConfig config;
config.layout.joint_names = {"joint1", "joint2"};
config.tracking_gains.kp = {10.0, 10.0};
config.tracking_gains.kd = {0.1, 0.1};
config.rigid_hold_gains = config.tracking_gains;
config.compliant_hold_gains = config.tracking_gains;

config.limits.max_velocity = {0.0, 0.0};  // 非正数表示不限制
config.limits.max_effort = {0.0, 0.0};
config.limits.min_kp = {-std::numeric_limits<double>::max(), -std::numeric_limits<double>::max()};
config.limits.max_kp = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
config.limits.min_kd = {-std::numeric_limits<double>::max(), -std::numeric_limits<double>::max()};
config.limits.max_kd = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};

config.use_model_feedforward = true;
config.use_command_effort = true;

controller.configure(config);
```

配置数组长度必须和 `joint_names.size()` 一致，否则 `configure()` 抛出 `std::runtime_error`

### 调用顺序

典型周期调用：

```cpp
JointState startup_state;
startup_state.position = {0.0, 0.0};
startup_state.velocity = {0.0, 0.0};
startup_state.effort = {0.0, 0.0};
startup_state.motor_effort = {0.0, 0.0};

controller.reset(startup_state);
controller.set_mode(JointImpedanceMode::TRACKING, startup_state);

JointCommand command;
command.mode = JointCommandMode::IMPEDANCE;
command.position = std::vector<double>{0.2, -0.1};
command.velocity = std::vector<double>{0.0, 0.0};
command.effort = std::vector<double>{0.0, 0.0};

auto command_result = controller.set_command(command);
if(!command_result) {
    // command_result.error() 是 JointCommandError
}

JointImpedanceControllerInput input;
input.state = startup_state;
input.model_feedforward = {0.3, -0.2};
input.dt = 0.005;

auto output = controller.update(input);
if(output.valid) {
    const MitJointCommand& mit = output.command;
}
```

### 运行规则

- `set_mode(RIGID_HOLD/COMPLIANT_HOLD)` 会锁存当前关节位置作为保持点
- `TRACKING` 下如果还没有有效命令，输出以当前位置为参考
- `TORQUE` 模式下不叠加 `model_feedforward`，避免纯力矩命令被模型项污染
- `NaN/Inf` 会被替换为安全默认值
- `max_velocity` 和 `max_effort` 为非正数时表示不限幅
- `update()` 输入状态数组维度不匹配时返回 `valid=false`

## DynamicsObserver

`DynamicsObserver` 是对 `PinocchioDynamicsModel` 的周期路径封装

```cpp
DynamicsObserver observer;
observer.configure("/path/to/robot.urdf", joint_names);

DynamicsObservation observation;
bool ok = observer.observe(
    positions,
    velocities,
    efforts,
    true,   // enable_gravity_feedforward
    false,  // enable_nonlinear_feedforward
    observation);
```

输出：

| 字段 | 说明 |
|---|---|
| `valid` | 本次观测是否成功 |
| `gravity` | `g(q)` |
| `nonlinear` | `nle(q,dq)` |
| `active_feedforward` | 当前被选择的模型前馈 |
| `external_effort` | `efforts - active_feedforward` |

前馈选择优先级：

```text
enable_nonlinear_feedforward=true  -> active_feedforward = nonlinear
enable_gravity_feedforward=true    -> active_feedforward = gravity
otherwise                          -> active_feedforward = 0
```

## PinocchioDynamicsModel

构造函数从完整 URDF 中构建 reduced model：

```cpp
PinocchioDynamicsModel model(urdf_path, joint_names);
```

约束：

- `joint_names` 中每个关节必须存在于 URDF
- 每个受控关节必须是 1-DoF
- 非受控关节会被锁定
- reduced model 的 `nq` 和 `nv` 必须等于 `joint_names.size()`

更新：

```cpp
bool ok = model.update(q, dq, true, true, true);
```

参数：

| 参数 | 说明 |
|---|---|
| `q` | 受控关节位置 |
| `dq` | 受控关节速度 |
| `enable_gravity` | 是否计算重力项 |
| `enable_nonlinear` | 是否计算非线性项 |
| `enable_mass_matrix` | 是否计算质量矩阵 |

读取结果：

```cpp
std::vector<double> gravity = model.get_gravity_std();
std::vector<double> nonlinear = model.get_nonlinear_effects_std();
const Eigen::MatrixXd& mass = model.get_mass_matrix();
```

周期路径建议使用：

```cpp
model.copy_gravity_to(gravity_buffer);
model.copy_nonlinear_effects_to(nonlinear_buffer);
```

## DmMotorBus

`DmMotorBus` 负责真实达妙硬件通信

```cpp
std::vector<DmMotorConfig> configs;
configs.push_back({
    "joint1",
    1,
    damiao::DM4310,
    1.0,
    ControlMode::MIT
});

DmMotorBus bus;
bus.configure("/dev/ttyACM0", B921600, configs);

JointState startup_state;
bus.activate(5, startup_state);

JointState state = startup_state;
bool read_ok = bus.read(false, state);

MitJointCommand command;
// 填充 command.position / velocity / effort / kp / kd
bool write_ok = bus.write(0, command);

bus.deactivate();
bus.cleanup();
```

注意：

- `activate()` 会使能电机并切换控制模式
- `deactivate()` 会先切到 POS_VEL 并发送零位置、低速度命令，然后失能
- 周期路径 `read()` 和 `write()` 捕获异常并返回 `false`
- 当前类直接依赖真实 `SerialPort` 和 `damiao::MotorControl`，还不是仿真后端接口

## 单元测试

当前测试覆盖 `JointImpedanceController`：

- 配置尺寸校验
- 命令必需字段和长度校验
- `RIGID_HOLD`
- `COMPLIANT_HOLD`
- `POSITION`
- `POSITION_VELOCITY`
- `IMPEDANCE`
- `VELOCITY`
- `TORQUE`
- 前馈叠加
- 速度、力矩、`kp/kd` 限幅
- `NaN/Inf` 清洗
- 输入状态维度错误

运行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select dm_control_core
colcon test --packages-select dm_control_core --event-handlers console_direct+
```
