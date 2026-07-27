#pragma once

#include <memory>

#include <tl/expected.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "dm_arm/config/config.hpp"
#include "dm_arm/core/types.hpp"

namespace dm_arm {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief 动力学模块错误类型
 */
enum class DynamicsErr {
    NOT_CONFIGURED,                 ///< 动力学模型尚未完成配置
    ALREADY_CONFIGURED,             ///< 动力学模型已经配置，不能重复配置
    NOT_UPDATED,                    ///< 动力学缓存尚未完成首次更新
    INVALID_CFG,                    ///< 动力学配置内容无效
    URDF_LOAD_FAILED,               ///< URDF 文件读取或模型构建失败
    JOINT_NOT_FOUND,                ///< 配置指定的关节在模型中不存在
    JOINT_NOT_1DOF,                 ///< 配置指定的关节不是受支持的单自由度关节
    MODEL_SIZE_MISMATCH,            ///< 模型维度、关节数量或索引映射不一致
    FRAME_NOT_FOUND,                ///< 请求的坐标系在模型中不存在
    INVALID_INPUT_SIZE,             ///< 输入关节向量长度与配置的关节数量不一致
    NON_FINITE_INPUT,               ///< 输入包含 NaN 或无穷值
    GRAVITY_SCALE_OUT_OF_RANGE,     ///< 重力补偿缩放系数超出 [0, 1]
    COMPUTE_FAILED,                 ///< 底层运动学或动力学计算失败
};

/**
 * @brief 动力学模型基本信息
 */
struct DynamicsInfo {
    std::size_t joints_count{ 0 };              ///< 受控关节数量
    int nq{ 0 };                                ///< Pinocchio 模型位置空间维数
    int nv{ 0 };                                ///< Pinocchio 模型速度空间维数
    double total_mass{ 0.0 };                   ///< URDF 模型总质量
    std::vector<std::string> joint_names;       ///< 受控关节名称
    std::vector<int> q_indices;                 ///< 受控关节位置索引
    std::vector<int> v_indices;                 ///< 受控关节速度索引
};

/**
 * @brief 最近一次 update() 得到的完整运动学与动力学缓存
 */
struct DynamicsState {
    JointVector pos;                    ///< 当前关节位置
    JointVector vel;                    ///< 当前关节速度
    JointVector acc;                    ///< 当前关节加速度估计
    JointVector tor;                    ///< 当前关节反馈力矩
    JointVector ref_acc;                ///< 当前关节参考加速度

    JointVector gravity;                ///< 未缩放重力广义力
    JointVector gravity_compensation;   ///< 缩放后的重力补偿
    JointVector nonlinear;              ///< 非线性广义力
    JointVector coriolis;               ///< 科氏力和离心力广义力
    JointVector inverse_dynamics;       ///< 使用 ref_acc 计算的逆动力学结果
    JointVector forward_dynamics;       ///< 使用反馈力矩计算的正向动力学结果

    Eigen::MatrixXd mass_matrix;        ///< 关节空间质量矩阵
    Eigen::Isometry3d tool_pose{ Eigen::Isometry3d::Identity() };  ///< tool_frame 相对于 base_frame 的位姿
    Eigen::MatrixXd tool_jacobian;      ///< tool_frame 的 LOCAL_WORLD_ALIGNED Jacobian
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief 机械臂运动学与刚体动力学计算接口
 */
class Dynamics {
public:
    /**
     * @brief 构造一个尚未配置的动力学对象
     */
    Dynamics();
    /**
     * @brief 析构动力学对象并释放内部模型资源
     */
    ~Dynamics();

    Dynamics(const Dynamics&) = delete;
    Dynamics& operator=(const Dynamics&) = delete;
    /**
     * @brief 移动构造动力学对象
     * @param other 被移动的动力学对象
     */
    Dynamics(Dynamics&& other) noexcept;
    /**
     * @brief 移动赋值动力学对象
     * @param other 被移动的动力学对象
     * @return 当前对象引用
     */
    Dynamics& operator=(Dynamics&& other) noexcept;

    /**
     * @brief 根据配置加载并初始化动力学模型
     * @param cfg 动力学配置
     * @return 成功时返回空值；失败时返回 DynamicsErr
     */
    tl::expected<void, DynamicsErr> configure(const DynamicsCfg& cfg);
    /**
     * @brief 集中更新当前周期的全部运动学与动力学缓存
     * @param state 当前关节位置、速度和反馈力矩
     * @param acc 当前关节加速度估计
     * @param ref_acc 当前关节参考加速度
     * @return 成功时返回空值；失败时返回 DynamicsErr
     */
    tl::expected<void, DynamicsErr> update(const JointState& state, const JointVector& acc, const JointVector& ref_acc);
    /**
     * @brief 更新重力补偿缩放系数
     * @param gravity_scale 重力补偿缩放系数
     * @return 成功时返回空值；失败时返回 DynamicsErr
     */
    tl::expected<void, DynamicsErr> set_gravity_scale(const JointVector& gravity_scale);
    /**
     * @brief 清理当前动力学模型并恢复未配置状态
     */
    void cleanup();

    /**
     * @brief 查询动力学模型是否已经完成配置
     * @return 已成功配置时返回 true，否则返回 false
     */
    bool is_configured() const noexcept;
    /**
     * @brief 查询动力学缓存是否已经完成至少一次成功更新
     * @return update() 成功执行过至少一次时返回 true，否则返回 false
     */
    bool is_updated() const noexcept;

    /**
     * @brief 获取当前动力学模型的基本信息
     * @return DynamicsInfo 的只读引用
     */
    const DynamicsInfo& get_info() const noexcept;
    /**
     * @brief 获取最近一次 update() 的完整缓存
     * @return DynamicsState 的只读引用
     */
    const DynamicsState& get_state() const noexcept;
    /**
     * @brief 获取当前重力补偿缩放系数
     * @return 重力补偿缩放系数的只读引用
     */
    const JointVector& get_gravity_scale() const noexcept;
    /**
     * @brief 获取指定坐标系相对于 base_frame 的缓存位姿
     * @param frame_name 目标坐标系名称
     * @return 成功时返回目标坐标系位姿副本；失败时返回 DynamicsErr
     */
    tl::expected<Eigen::Isometry3d, DynamicsErr> get_frame_pose(const std::string& frame_name) const;
    /**
     * @brief 获取指定坐标系的缓存几何 Jacobian
     * @param frame_name 目标坐标系名称
     * @return 成功时返回 6×N Jacobian 副本；失败时返回 DynamicsErr
     */
    tl::expected<Eigen::MatrixXd, DynamicsErr> get_frame_jacobian(const std::string& frame_name) const;
    /**
     * @brief 获取最近一次 update() 的未缩放重力广义力
     * @return 重力广义力缓存的只读引用
     */
    const JointVector& get_gravity() const noexcept;
    /**
     * @brief 获取最近一次 update() 的缩放后重力补偿
     * @return 重力补偿缓存的只读引用
     */
    const JointVector& get_gravity_compensation() const noexcept;
    /**
     * @brief 获取最近一次 update() 的完整非线性广义力
     * @return 非线性广义力缓存的只读引用
     */
    const JointVector& get_nonlinear() const noexcept;
    /**
     * @brief 获取最近一次 update() 的科氏力和离心力广义力
     * @return 科氏力和离心力缓存的只读引用
     */
    const JointVector& get_coriolis() const noexcept;
    /**
     * @brief 获取最近一次 update() 的关节空间质量矩阵
     * @return 质量矩阵缓存的只读引用
     */
    const Eigen::MatrixXd& get_mass_matrix() const noexcept;
    /**
     * @brief 获取最近一次 update() 的逆动力学结果
     * @return 逆动力学缓存的只读引用
     */
    const JointVector& get_inverse_dynamics() const noexcept;
    /**
     * @brief 获取最近一次 update() 的正向动力学结果
     * @return 正向动力学缓存的只读引用
     */
    const JointVector& get_forward_dynamics() const noexcept;
    /**
     * @brief 获取最近一次 update() 的末端位姿
     * @return tool_frame 位姿缓存的只读引用
     */
    const Eigen::Isometry3d& get_tool_pose() const noexcept;
    /**
     * @brief 获取最近一次 update() 的末端 Jacobian
     * @return tool_frame Jacobian 缓存的只读引用
     */
    const Eigen::MatrixXd& get_tool_jacobian() const noexcept;

private:
    struct Impl;                    ///< 动力学模块内部实现
    std::unique_ptr<Impl> impl_;    ///< 动力学内部实现对象
};

} // namespace dm_arm
