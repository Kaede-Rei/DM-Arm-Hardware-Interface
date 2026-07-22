#pragma once

#include <memory>

#include <tl/expected.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "dm_arm/core/types.hpp"
#include "dm_arm/config/config.hpp"

namespace dm_arm {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief 动力学模块错误类型
 */
enum class DynamicsErr {
    NOT_CONFIGURED,      ///< 动力学模型尚未完成配置
    ALREADY_CONFIGURED,  ///< 动力学模型已经配置，不能重复配置
    INVALID_CFG,         ///< 动力学配置内容无效
    URDF_LOAD_FAILED,    ///< URDF 文件读取或模型构建失败
    JOINT_NOT_FOUND,     ///< 配置指定的关节在模型中不存在
    JOINT_NOT_1DOF,      ///< 配置指定的关节不是受支持的单自由度关节
    MODEL_SIZE_MISMATCH, ///< 模型维度、关节数量或索引映射不一致
    FRAME_NOT_FOUND,     ///< 请求的坐标系在模型中不存在
    INVALID_INPUT_SIZE,  ///< 输入关节向量长度与配置的关节数量不一致
    NON_FINITE_INPUT,    ///< 输入包含 NaN 或无穷值
    COMPUTE_FAILED,      ///< 底层运动学或动力学计算失败
};

/**
 * @brief 动力学模型基本信息
 */
struct DynamicsInfo {
    std::size_t joints_count{ 0 };  ///< 受控关节数量，等于 joint_names.size()

    int nq{ 0 };        ///< Pinocchio 模型位置空间维数
    int nv{ 0 };        ///< Pinocchio 模型速度空间维数

    double total_mass{ 0.0 };       ///< URDF 模型中所有刚体的总质量

    std::vector<std::string> joint_names;   ///< 受控关节名称，顺序与 JointVector 一致
    std::vector<int> q_indices;             ///< 各受控关节在完整模型位置向量中的起始索引
    std::vector<int> v_indices;             ///< 各受控关节在完整模型速度向量中的起始索引
};

/**
 * @brief 指定关节状态下的主要动力学计算结果
 */
struct DynamicsState {
    JointVector gravity;            ///< 重力广义力向量
    JointVector nonlinear;          ///< 完整非线性广义力向量
    JointVector coriolis;           ///< 科氏力和离心力广义力向量
    Eigen::MatrixXd mass_matrix;    ///< 关节空间质量矩阵
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
     * @param cfg 动力学配置，包括 URDF 路径和受控关节名称等信息
     * @return 成功时返回空值；失败时返回对应的 DynamicsErr
     */
    tl::expected<void, DynamicsErr> configure(const DynamicsCfg& cfg);
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
     * @brief 获取当前动力学模型的基本信息
     * @return DynamicsInfo 的只读引用
     */
    const DynamicsInfo& get_info() const noexcept;
    /**
     * @brief 获取指定坐标系相对于模型根坐标系的位姿
     * @param q 关节位置向量，顺序应与 DynamicsInfo::joint_names 一致
     * @param frame_name 目标坐标系名称
     * @return 成功时返回目标坐标系位姿；失败时返回 DynamicsErr
     */
    tl::expected<Eigen::Isometry3d, DynamicsErr> get_frame_pose(const JointVector& q, const std::string& frame_name) const;
    /**
     * @brief 获取指定坐标系的几何 Jacobian
     * @param q 关节位置向量，顺序应与 DynamicsInfo::joint_names 一致
     * @param frame_name 目标坐标系名称
     * @return 成功时返回 6×N Jacobian；失败时返回 DynamicsErr
     */
    tl::expected<Eigen::MatrixXd, DynamicsErr> get_frame_jacobian(const JointVector& q, const std::string& frame_name) const;
    /**
     * @brief 获取当前关节姿态下的重力广义力
     * @param q 关节位置向量
     * @return 成功时返回重力广义力向量；失败时返回 DynamicsErr
     */
    tl::expected<JointVector, DynamicsErr> get_gravity(const JointVector& q) const;
    /**
     * @brief 获取完整非线性广义力
     * @param q 关节位置向量
     * @param dq 关节速度向量
     * @return 成功时返回非线性广义力向量；失败时返回 DynamicsErr
     */
    tl::expected<JointVector, DynamicsErr> get_nonlinear(const JointVector& q, const JointVector& dq) const;
    /**
     * @brief 获取科氏力和离心力广义力向量
     * @param q 关节位置向量
     * @param dq 关节速度向量
     * @return 成功时返回科氏力和离心力广义力向量；
     *         失败时返回 DynamicsErr
     */
    tl::expected<JointVector, DynamicsErr> get_coriolis(const JointVector& q, const JointVector& dq) const;
    /**
     * @brief 获取关节空间质量矩阵
     * @param q 关节位置向量
     * @return 成功时返回质量矩阵；失败时返回 DynamicsErr
     */
    tl::expected<Eigen::MatrixXd, DynamicsErr> get_mass_matrix(const JointVector& q) const;
    /**
     * @brief 获取关节空间逆动力学
     * @param q 关节位置向量
     * @param dq 关节速度向量
     * @param ddq 关节加速度向量
     * @return 成功时返回关节力矩向量；失败时返回 DynamicsErr
     */
    tl::expected<JointVector, DynamicsErr> get_inverse_dynamics(const JointVector& q, const JointVector& dq, const JointVector& ddq) const;
    /**
     * @brief 获取关节空间正向动力学
     * @param q 关节位置向量
     * @param dq 关节速度向量
     * @param tau 关节驱动力矩向量
     * @return 成功时返回关节加速度向量；失败时返回 DynamicsErr
     */
    tl::expected<JointVector, DynamicsErr> get_forward_dynamics(const JointVector& q, const JointVector& dq, const JointVector& tau) const;

    /**
     * @brief 批量计算指定关节状态下的主要动力学量
     * @param q 关节位置向量
     * @param dq 关节速度向量
     * @param output 用于接收计算结果的输出结构体
     * @return 成功时返回空值；失败时返回 DynamicsErr
     * @post 成功时 output 中所有成员均按照 DynamicsInfo::joint_names 的顺序更新
     */
    tl::expected<void, DynamicsErr> compute_state(const JointVector& q, const JointVector& dq, DynamicsState& output) const;

private:
    struct Impl;                    ///< 动力学模块内部实现
    std::unique_ptr<Impl> impl_;    ///< 动力学内部实现对象
};

} // namespace dm_arm