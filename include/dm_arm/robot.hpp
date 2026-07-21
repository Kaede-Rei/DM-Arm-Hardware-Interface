#pragma once

#include <tl/expected.hpp>

#include "dm_arm/config/config.hpp"
#include "dm_arm/core/joint_actuator_mapper.hpp"
#include "dm_arm/core/joints_ctrller.hpp"
#include "dm_arm/core/safety.hpp"
#include "dm_arm/hardware/motor_bus.hpp"

#include <chrono>
#include <memory>
#include <tl/optional.hpp>

namespace dm_arm {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief Robot 生命周期状态
 */
enum class RobotState {
    UNCONFIGURED, ///< 尚未完成模块配置
    INACTIVE,     ///< 已配置但未使能硬件
    ACTIVE,       ///< 正常控制周期运行中
    FAULT,        ///< 已触发故障，必须显式 reset_fault()
};

/**
 * @brief 模型前馈计算错误
 */
enum class ModelFeedforwardErr {
    NOT_CONFIGURED, ///< 非 NONE 模式未提供计算函数
    INVALID_INPUT,  ///< 输入状态无效
    COMPUTE_FAILED, ///< 动力学计算失败
};

/**
 * @brief Robot 顶层错误类型
 */
enum class RobotErr {
    NOT_CONFIGURED,            ///< 尚未完成 configure()
    ALREADY_CONFIGURED,        ///< 已经完成配置，禁止重复配置
    INVALID_CFG,               ///< 配置内容非法
    NULL_MOTOR_BUS,            ///< 传入的 MotorBus 为空
    MOTOR_BUS_SIZE_MISMATCH,   ///< MotorBus 数量与关节数量不一致
    WRITE_DISABLED,            ///< 配置禁止写入执行器
    NOT_ACTIVE,                ///< 当前状态不是 ACTIVE
    ALREADY_ACTIVE,            ///< 当前已经处于 ACTIVE
    FAULTED,                   ///< 当前处于 FAULT
    NOT_FAULTED,               ///< 当前并未处于 FAULT
    INVALID_TIME,              ///< 时间戳回退或不合法

    MOTOR_BUS_CONNECT_FAILED,   ///< MotorBus connect() 失败
    MOTOR_BUS_ACTIVATE_FAILED,   ///< MotorBus activate() 失败
    MOTOR_BUS_READ_FAILED,       ///< MotorBus read() 失败
    MOTOR_BUS_WRITE_FAILED,      ///< MotorBus write() 失败
    MOTOR_BUS_DEACTIVATE_FAILED, ///< MotorBus deactivate() 失败
    MOTOR_BUS_RECOVER_FAILED,    ///< MotorBus recover() 失败

    MAPPER_FAILED,              ///< 关节/执行器映射失败
    CTRLLER_FAILED,             ///< 控制器失败
    SAFETY_FAILED,              ///< 安全检查失败
    MODEL_FEEDFORWARD_FAILED,   ///< 模型前馈计算失败
    INVALID_MODEL_FEEDFORWARD,  ///< 模型前馈结果非法
};

/**
 * @brief Robot 故障详情
 */
struct RobotFault {
    RobotErr code{ RobotErr::INVALID_CFG };                             ///< 顶层故障码
    MotorBusErr motor_bus_err{ MotorBusErr::INVALID_CFG };              ///< MotorBus 子错误
    JointActuatorMapErr mapper_err{ JointActuatorMapErr::INVALID_CFG }; ///< 映射器子错误
    JointCtrllerErr ctrller_err{ JointCtrllerErr::INVALID_CFG };        ///< 控制器子错误
    SafetyFault safety_fault{};                                         ///< 安全检查子错误详情
    ModelFeedforwardErr model_feedforward_err{
        ModelFeedforwardErr::COMPUTE_FAILED
    };                                                                  ///< 模型前馈子错误
};

/**
 * @brief 单次 Robot 控制周期输出
 */
struct RobotCycleOutput {
    ActuatorState actuator_state; ///< 本周期执行器状态
    JointState joint_state;       ///< 本周期关节状态
    JointCtrlCmd joint_cmd;       ///< Safety 检查后的关节命令
    ActuatorMitCmd actuator_cmd;  ///< 实际发送的执行器命令
    double dt{ 0.0 };             ///< 本周期使用的时间步长
};

/**
 * @brief 接入 Robot 的模型前馈函数
 */
using ModelFeedforwardFn = std::function<tl::expected<JointVector, ModelFeedforwardErr>(ModelFeedforwardMode, const JointState&, double)>;

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief DM-Arm 顶层控制闭环
 */
class Robot {
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    Robot() = default;
    /**
     * @brief 释放 Robot 持有的硬件资源并尽力安全失能
     */
    ~Robot();

    Robot(const Robot&) = delete;
    Robot& operator=(const Robot&) = delete;
    Robot(Robot&&) = delete;
    Robot& operator=(Robot&&) = delete;

    /**
     * @brief 配置 Robot 并接管 MotorBus 所有权
     * @param cfg Robot 完整配置
     * @param motor_bus 待接管的 MotorBus 实例
     * @param model_feedforward 可选模型前馈函数
     * @return 配置成功返回空 expected，失败返回 RobotFault
     */
    tl::expected<void, RobotFault> configure(const RobotCfg& cfg, std::unique_ptr<MotorBus> motor_bus, ModelFeedforwardFn model_feedforward = {});

    /**
     * @brief 连接、使能并用真实状态初始化控制器
     * @return 成功返回空 expected，失败返回 RobotFault
     */
    tl::expected<void, RobotFault> activate();

    /**
     * @brief 设置跟踪参考命令
     * @param cmd 跟踪命令
     * @param now 当前时间点
     * @return 成功返回空 expected，失败返回 RobotFault
     */
    tl::expected<void, RobotFault> set_cmd(const JointCmd& cmd, TimePoint now = Clock::now());

    /**
     * @brief 设置完整 Joint 控制命令
     * @param cmd 完整关节控制命令
     * @param now 当前时间点
     * @return 成功返回空 expected，失败返回 RobotFault
     */
    tl::expected<void, RobotFault> set_full_cmd(const JointCtrlCmd& cmd, TimePoint now = Clock::now());

    /**
     * @brief 切换阻抗模式
     * @param mode 目标阻抗模式
     * @param now 当前时间点
     * @return 成功返回空 expected，失败返回 RobotFault
     */
    tl::expected<void, RobotFault> set_impedance_mode(JointImpedanceMode mode, TimePoint now = Clock::now());

    /**
     * @brief 执行一次完整控制周期
     * @param now 当前时间点
     * @return 成功返回周期输出，失败返回 RobotFault
     */
    tl::expected<RobotCycleOutput, RobotFault> cycle(TimePoint now = Clock::now());

    /**
     * @brief 安全停止并失能，回到 INACTIVE
     * @return 成功返回空 expected，失败返回 RobotFault
     */
    tl::expected<void, RobotFault> deactivate();

    /**
     * @brief 清除 FAULT 锁存并回到 INACTIVE
     * @return 成功返回空 expected，失败返回 RobotFault
     */
    tl::expected<void, RobotFault> reset_fault();

    /**
     * @brief 获取当前 Robot 生命周期状态
     * @return 当前 RobotState
     */
    RobotState get_state() const noexcept;

    /**
     * @brief 获取当前控制器阻抗模式
     * @return 当前 JointImpedanceMode
     */
    JointImpedanceMode get_impedance_mode() const noexcept;

    /**
     * @brief 获取当前模型前馈模式
     * @return 当前 ModelFeedforwardMode
     */
    ModelFeedforwardMode get_model_feedforward_mode() const noexcept;

    /**
     * @brief 获取最近一次合法的关节状态
     * @return 最近一次合法的 JointState
     */
    const JointState& get_joint_state() const noexcept;

    /**
     * @brief 获取最近一次合法的执行器状态
     * @return 最近一次合法的 ActuatorState
     */
    const ActuatorState& get_actuator_state() const noexcept;

    /**
     * @brief 获取最近一次锁存的故障信息
     * @return 最近一次故障信息，若无故障则为空
     */
    const tl::optional<RobotFault>& get_last_fault() const noexcept;

private:
    /**
     * @brief 计算当前周期的模型前馈项
     * @param state 当前关节状态
     * @param dt 当前周期时间步长
     * @return 成功返回前馈向量，失败返回 RobotFault
     */
    tl::expected<JointVector, RobotFault> compute_model_feedforward(const JointState& state, double dt) const;

    /**
     * @brief 构造仅包含通用错误码的故障对象
     * @param code 顶层错误码
     * @return 构造后的 RobotFault
     */
    RobotFault make_fault(RobotErr code) const noexcept;

    /**
     * @brief 构造包含 MotorBus 子错误的故障对象
     * @param code 顶层错误码
     * @param err MotorBus 子错误
     * @return 构造后的 RobotFault
     */
    RobotFault make_bus_fault(RobotErr code, MotorBusErr err) const noexcept;

    /**
     * @brief 构造包含映射器子错误的故障对象
     * @param err 映射器子错误
     * @return 构造后的 RobotFault
     */
    RobotFault make_mapper_fault(JointActuatorMapErr err) const noexcept;

    /**
     * @brief 构造包含控制器子错误的故障对象
     * @param err 控制器子错误
     * @return 构造后的 RobotFault
     */
    RobotFault make_ctrller_fault(JointCtrllerErr err) const noexcept;

    /**
     * @brief 构造包含安全检查子错误的故障对象
     * @param fault 安全检查子错误详情
     * @return 构造后的 RobotFault
     */
    RobotFault make_safety_fault(const SafetyFault& fault) const noexcept;

    /**
     * @brief 构造包含模型前馈子错误的故障对象
     * @param err 模型前馈子错误
     * @return 构造后的 RobotFault
     */
    RobotFault make_model_fault(ModelFeedforwardErr err) const noexcept;

    /**
     * @brief 进入 FAULT 状态并执行对应安全动作
     * @param fault 触发故障
     * @param action 对应的安全动作
     */
    void enter_fault(const RobotFault& fault, SafetyAction action) noexcept;

    /**
     * @brief 尝试停机，失败后降级为失能
     */
    void stop_or_disable_noexcept() noexcept;

    /**
     * @brief 直接失能硬件，忽略错误
     */
    void disable_noexcept() noexcept;

    /**
     * @brief 清空运行时缓存状态
     */
    void clear_runtime_state() noexcept;

    /**
     * @brief 判断当前控制模式是否属于跟踪模式
        * @return 当前是否处于跟踪模式
     */
    bool is_tracking_mode() const noexcept;

    /**
     * @brief 计算两个时间点之间的秒数差
        * @param newer 较新的时间点
        * @param older 较旧的时间点
        * @return 时间差秒数
     */
    double seconds_between(TimePoint newer, TimePoint older) const noexcept;

private:
    RobotCfg cfg_;                                  ///< 完整 Robot 配置
    JointCtrller ctrller_;                          ///< Joint 控制器
    JointActuatorMapper mapper_;                    ///< Joint/Actuator 映射
    Safety safety_;                                 ///< 安全检查器
    std::unique_ptr<MotorBus> motor_bus_;           ///< 执行器后端
    ModelFeedforwardFn model_feedforward_;          ///< 可选动力学前馈入口

    RobotState state_{ RobotState::UNCONFIGURED };  ///< Robot 生命周期状态
    JointState joint_state_;                        ///< 最近一次合法 JointState
    ActuatorState actuator_state_;                  ///< 最近一次合法 ActuatorState
    tl::optional<RobotFault> last_fault_;           ///< 锁存的最近故障

    TimePoint last_cycle_time_{};               ///< 上一成功周期时间
    TimePoint last_state_time_{};               ///< 上一合法状态时间
    TimePoint last_cmd_time_{};                 ///< 上一外部命令时间

    bool has_state_{ false };                   ///< 是否已有合法状态
    bool has_completed_cycle_{ false };         ///< 是否已经完成至少一个周期
    bool has_external_cmd_{ false };            ///< 跟踪模式是否收到过外部命令
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //

} // namespace dm_arm
