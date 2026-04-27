#pragma once

#include "dm_hw/damiao.hpp"
#include "dm_control_core/joint_control_types.hpp"

#include <cstdint>
#include <cstddef>
#include <memory>
#include <string>
#include <termios.h>
#include <vector>

class SerialPort;

namespace damiao {
class Motor;
class MotorControl;
}

namespace dm_control_core {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief 控制模式枚举
 */
enum class ControlMode {
    MIT,        ///< MIT 模式
    POS_VEL     ///< 位置+速度模式
};

/**
 * @brief 单个关节对应的达妙电机配置
 */
struct DmMotorConfig {
    std::string joint_name;                ///< ROS 关节名称
    uint32_t motor_id;                     ///< 达妙电机 CAN ID
    damiao::DmMotorType motor_type;        ///< 达妙电机型号
    double joint_to_motor_scale;           ///< 关节侧到电机侧的位置/速度比例
    ControlMode control_mode;              ///< 电机控制模式
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief DmMotorBus 封装达妙电机串口/CAN 通信和关节-电机单位换算
 */
class DmMotorBus {
public:
    /**
     * @brief 配置串口、MotorControl 和电机对象
     * @param serial_port 串口设备路径
     * @param baudrate termios 波特率
     * @param configs 关节-电机配置表
     */
    void configure(const std::string& serial_port, speed_t baudrate, const std::vector<DmMotorConfig>& configs);

    /**
     * @brief 使能电机、切换控制模式，并读取启动初始状态
     * @param startup_read_cycles 启动时读取并平均的次数
     * @param state 输出启动后的关节状态
     */
    void activate(int startup_read_cycles, JointState& state);

    /**
     * @brief 停用电机，先切回位置速度模式再失能
     */
    void deactivate();

    /**
     * @brief 清理串口、控制器和电机对象
     */
    void cleanup();

    /**
     * @brief 读取所有电机状态
     * @param refresh_state 是否主动请求刷新电机状态
     * @param state 预分配的状态缓冲，大小必须等于电机数量
     * @return 成功返回 true，失败返回 false；周期路径不向外抛异常
     */
    bool read(bool refresh_state, JointState& state) noexcept;

    /**
     * @brief 写入单个关节命令到电机
     * @param index 电机/关节索引
     * @param command 关节侧命令
     * @return 成功返回 true，失败返回 false；周期路径不向外抛异常
     */
    bool write(std::size_t index, const MitJointCommand& command) noexcept;

    /**
     * @brief 获取当前 bus 管理的电机数量
     * @return 电机数量
     */
    std::size_t size() const { return configs_.size(); }

private:
    /**
     * @brief 将硬件接口控制模式转换为达妙控制模式
     * @param mode 硬件接口控制模式
     * @param dm_mode 输出达妙控制模式
     * @return 成功返回 true，未知模式返回 false
     */
    bool to_dm_control_mode(ControlMode mode, damiao::DmControlMode& dm_mode) const noexcept;

    /**
     * @brief 读取单个电机状态并换算到关节侧
     * @param index 电机/关节索引
     * @param refresh_state 是否主动刷新电机状态
     * @param state 输出关节侧状态
     * @return 成功返回 true，失败返回 false
     */
    bool read_one(std::size_t index, bool refresh_state, JointState& state) noexcept;

private:
    std::shared_ptr<SerialPort> serial_;
    std::shared_ptr<damiao::MotorControl> motor_controller_;
    std::vector<std::shared_ptr<damiao::Motor>> motors_;
    std::vector<DmMotorConfig> configs_;
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //



}
