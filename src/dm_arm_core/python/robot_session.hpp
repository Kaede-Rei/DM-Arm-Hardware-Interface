#pragma once

#include "dm_arm/config/config.hpp"
#include "dm_arm/dynamics/dynamics.hpp"
#include "dm_arm/robot.hpp"

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace dm_arm {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief Python binding 统一异常
 */
class DmArmPythonError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

/**
 * @brief Python 真机会话执行器静态信息
 */
struct RobotSessionActuatorInfo {
    std::string name;              ///< 执行器名称
    std::string joint_name;        ///< 关联关节名称
    std::uint32_t motor_id{ 0 };   ///< 电机 ID
    std::uint32_t master_id{ 0 };  ///< 主站 ID
    std::string motor_type;        ///< 电机型号名称
    double q_max{ 0.0 };           ///< 执行器最大位置绝对值
    double dq_max{ 0.0 };          ///< 执行器最大速度绝对值
    double tau_max{ 0.0 };         ///< 执行器最大力矩绝对值
};

/**
 * @brief Python 真机会话快照
 */
struct RobotSessionSnapshot {
    RobotState robot_state{ RobotState::UNCONFIGURED };  ///< Robot 生命周期状态
    RobotCycleOutput cycle;                              ///< 最近一次完整控制周期输出
    DynamicsState dynamics;                              ///< 最近一次动力学缓存
    bool valid{ false };                                 ///< 是否已经存在合法周期快照
    std::string last_error;                              ///< 最近一次工作线程错误
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief Python 真机安全会话
 *
 * RobotSession 使用独立 C++ 线程维持控制周期；Python 线程只提交模式、目标和调参请求
 */
class PyRobotSession {
public:
    PyRobotSession() = default;
    ~PyRobotSession();

    PyRobotSession(const PyRobotSession&) = delete;
    PyRobotSession& operator=(const PyRobotSession&) = delete;
    PyRobotSession(PyRobotSession&&) = delete;
    PyRobotSession& operator=(PyRobotSession&&) = delete;

    /**
     * @brief 加载配置并构建 Dynamics、DamiaoMotorBus 和 Robot
     * @param config_file YAML 配置文件路径
     */
    void configure(const std::string& config_file);
    /**
     * @brief 激活真机并启动 C++ 控制线程
     * @param allow_hardware 是否显式允许真机写入
     */
    void start(bool allow_hardware);
    /**
     * @brief 停止控制线程并安全失能
     */
    void stop();
    /**
     * @brief 在控制线程停止后清除 Robot FAULT
     */
    void reset_fault();
    /**
     * @brief 在控制线程停止后清除 Robot FAULT
     */
    void clear_fault();
    /**
     * @brief 请求进入 FAULT 受限柔性恢复
     */
    void enter_fault_compliant_recovery();
    /**
     * @brief 请求返回 FAULT 刚性保持
     */
    void return_to_fault_rigid_hold();

    /**
     * @brief 请求切换阻抗模式
     * @param mode 目标阻抗模式
     */
    void set_impedance_mode(JointImpedanceMode mode);
    /**
     * @brief 请求切换模型前馈模式
     * @param mode 目标模型前馈模式
     */
    void set_model_feedforward_mode(ModelFeedforwardMode mode);
    /**
     * @brief 请求更新重力补偿比例
     * @param gravity_scale 六轴重力补偿比例
     */
    void set_gravity_scale(const JointVector& gravity_scale);
    /**
     * @brief 设置连续梯形参考的绝对位置目标
     * @param pos 六轴绝对位置目标
     * @param speed_scale 速度比例，范围为 (0, 1]
     */
    void move_to(const JointVector& pos, double speed_scale);
    /**
     * @brief 取消位置目标并切换到当前位置保持
     */
    void hold_current();

    /**
     * @brief 获取最近一次完整会话快照
     */
    RobotSessionSnapshot get_snapshot() const;
    /**
     * @brief 获取当前 Robot 生命周期状态
     */
    RobotState get_state() const;
    /**
     * @brief 获取当前 FAULT 保持模式
     */
    FaultHoldMode get_fault_hold_mode() const;
    /**
     * @brief 获取当前会话是否已经完成配置
     */
    bool is_configured() const noexcept;
    /**
     * @brief 获取 C++ 控制线程是否正在运行
     */
    bool is_running() const noexcept;
    /**
     * @brief 获取当前静态配置副本
     */
    RobotCfg get_config() const;
    /**
     * @brief 获取动力学模型信息副本
     */
    DynamicsInfo get_dynamics_info() const;
    /**
     * @brief 获取达妙执行器静态信息副本
     */
    std::vector<RobotSessionActuatorInfo> get_actuator_info() const;

private:
    /**
     * @brief C++ 周期线程入口
     */
    void loop() noexcept;
    /**
     * @brief 根据当前目标生成一帧连续位置速度参考
     * @param dt 当前参考生成周期
     */
    void update_reference(double dt);
    /**
     * @brief 校验关节向量大小和有限性
     * @param values 待检查向量
     * @param name 参数名称
     */
    void validate_joint_vector(const JointVector& values, const char* name) const;
    /**
     * @brief 将工作线程错误写入快照
     * @param message 错误消息
     */
    void set_worker_error(const std::string& message) noexcept;
    /**
     * @brief 连接 Dynamics 与 Robot 的模型前馈回调
     */
    ModelFeedforwardFn make_model_feedforward();

private:
    RobotCfg cfg_;                                      ///< 完整静态配置
    std::unique_ptr<Dynamics> dynamics_;                ///< 动力学模型
    std::unique_ptr<Robot> robot_;                      ///< Robot 闭环
    std::vector<RobotSessionActuatorInfo> actuator_info_;     ///< 达妙执行器静态信息

    mutable std::mutex mutex_;                          ///< 请求与快照互斥锁
    RobotSessionSnapshot snapshot_;                     ///< 最近一次完整快照
    JointVector goal_pos_;                              ///< 当前绝对位置目标
    JointVector ref_pos_;                               ///< 当前连续位置参考
    JointVector ref_vel_;                               ///< 当前连续速度参考
    JointVector requested_gravity_scale_;               ///< 待应用重力补偿比例

    JointImpedanceMode requested_impedance_mode_{ JointImpedanceMode::RIGID_HOLD };       ///< 待应用阻抗模式
    double speed_scale_{ 0.3 };                         ///< 梯形参考速度比例

    std::uint64_t impedance_sequence_{ 0 };             ///< 阻抗模式请求序号
    std::uint64_t gravity_sequence_{ 0 };               ///< 重力比例请求序号
    bool has_goal_{ false };                            ///< 是否存在位置目标
    bool configured_{ false };                          ///< 会话是否已完成配置

    std::thread worker_;                                ///< C++ 控制线程
    std::atomic<bool> running_{ false };                ///< 工作线程运行标志
};

} // namespace dm_arm
