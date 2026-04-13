# dm_hw

`dm_hw` 是一个基于串口（USB 转 CAN）与达妙电机通信的轻量 C++ 库/ROS2 包，提供：

- 电机对象建模（型号、ID、状态、参数缓存）
- 电机使能/失能/回零
- 多种控制模式（MIT、位置速度、速度、力位混合、CSP）
- 寄存器参数读写与保存

## 目录

- [包内容](#包内容)
- [快速开始](#快速开始)
- [核心 API](#核心-api)
- [最小示例](#最小示例)
- [注意事项](#注意事项)

## 包内容

- 头文件
  - `include/dm_hw/damiao.hpp`
  - `include/dm_hw/serial_port.hpp`
- 示例程序
  - `src/test_damiao.cpp`
- 构建配置
  - `CMakeLists.txt`
  - `package.xml`

## 快速开始

### 1) 编译 `dm_hw`

在 ROS2 工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select dm_hw
source install/setup.bash
```

### 2) 连接硬件

默认串口设备为 `/dev/ttyACM0`，默认波特率为 `B921600`。

如果你使用其他设备节点，请在代码里创建 `SerialPort` 时替换为实际路径。

### 3) 运行示例

```bash
ros2 run dm_hw test_damiao
```

示例程序会演示：

- 创建串口与控制器
- 添加电机到控制器
- 切换控制模式
- 控制/刷新电机状态并打印位置、速度、电流（扭矩）

## 核心 API

以下 API 均来自 `include/dm_hw/damiao.hpp` 和 `include/dm_hw/serial_port.hpp`。

### 1) 类型与枚举

- `damiao::MotorId`：电机 ID 类型（`uint32_t`）
- `damiao::DmMotorType`：电机型号枚举（如 `DM4310`、`DMH3510`）
- `damiao::DmControlMode`：控制模式枚举
  - `MIT_MODE`
  - `POS_VEL_MODE`
  - `VEL_MODE`
  - `POS_FORCE_MODE`
  - `POS_VEL_CSP_MODE`
  - `VEL_CSP_MODE`
  - `TORQUE_CSP_MODE`
- `damiao::DmReg`：寄存器枚举（如 `CTRL_MODE`、`PMAX`、`UV_Value`）

### 2) `damiao::Motor`

用于描述单个电机并缓存反馈/参数。

常用接口：

- 构造
  - `Motor(DmMotorType motor_type, MotorId slave_id, MotorId master_id)`
  - `Motor()`
- 状态读写
  - `void receive_data(float q, float dq, float tau)`
  - `float get_position() const`
  - `float get_velocity() const`
  - `float get_tau() const`
- 标识信息
  - `MotorId get_slave_id() const`
  - `MotorId get_master_id() const`
  - `DmMotorType get_motor_type() const`
- 参数缓存
  - `set_param(...)`
  - `get_param_as_float(...)`
  - `get_param_as_uint32(...)`
  - `has_param(...)`
  - `clear_param(...)`
  - `clear_all_params()`

### 3) `damiao::MotorControl`

用于发送控制命令、接收反馈、读写寄存器。

常用接口：

- 初始化
  - `MotorControl(SerialPort::SharedPtr serial = nullptr)`
  - `void add_motor(Motor* motor)`
- 设备控制
  - `void enable(const Motor& motor)`
  - `void disable(const Motor& motor)`
  - `void set_zero_position(const Motor& motor)`
  - `void refresh_motor_status(const Motor& motor)`
- 控制模式
  - `bool switch_control_mode(Motor& motor, DmControlMode mode)`
- 运动控制
  - `void control_mit(Motor& motor, float kp, float kd, float q, float dq, float tau)`
  - `void control_pos_vel(Motor& motor, float pos, float vel)`
  - `void control_vel(Motor& motor, float vel)`
  - `void control_pos_force(Motor& motor, float pos, uint16_t vel, uint16_t i)`
  - `void control_pos_vel_csp(Motor& motor, float pos, float vel)`
  - `void control_vel_csp(Motor& motor, float vel)`
  - `void control_tor_csp(Motor& motor, float tor)`
- 寄存器
  - `float read_motor_param(Motor& motor, uint8_t reg_id)`
  - `bool change_motor_param(Motor& motor, uint8_t reg_id, float data)`
  - `void save_motor_param(Motor& motor)`

### 4) `SerialPort`

`SerialPort` 在 `include/dm_hw/serial_port.hpp` 中定义。

常用接口：

- `SerialPort(std::string port, speed_t baudrate, int timeout_ms = 2)`
- `ssize_t send(const uint8_t* data, size_t len)`
- `ssize_t recv(uint8_t* data, size_t len)`
- `bool recv_frame(uint8_t* data, uint8_t head, ssize_t len)`
- `void set_timeout(int timeout_ms)`

## 最小示例

```cpp
#include "dm_hw/damiao.hpp"
#include <memory>

int main() {
    auto serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    damiao::MotorControl control(serial);

    damiao::Motor motor(damiao::DMH3510, 0x01, 0x00);
    control.add_motor(&motor);

    control.disable(motor);
    control.enable(motor);

    control.switch_control_mode(motor, damiao::VEL_MODE);
    control.control_vel(motor, 1.0f);

    control.refresh_motor_status(motor);
    return 0;
}
```

## 注意事项

- 控制前必须先 `add_motor`，否则会触发 “id not found” 异常。
- 默认串口是 `/dev/ttyACM0`；设备路径变化时需要同步修改。
- `Motor` 的速度/位置/扭矩反馈解码依赖对应型号的限幅参数。
- 若反馈异常，优先检查：电源、总线接线、ID 冲突、串口权限。
