#pragma once

#include "serial_arm/transport/can.hpp"
#include "tl/expected.hpp"

#include <chrono>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace serial_arm::transport {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

class CanChannel;

/**
 * @brief 同进程共享 CAN 总线抽象
 */
class CanBus : public std::enable_shared_from_this<CanBus> {
public:
    using SharedPtr = std::shared_ptr<CanBus>;

    virtual ~CanBus() = default;

    /**
     * @brief 打开底层 CAN 总线
     * @return 成功时返回空结果，否则返回 CAN 错误
     */
    virtual tl::expected<void, CanErr> open() = 0;

    /**
     * @brief 关闭底层 CAN 总线
     */
    virtual void close() noexcept = 0;

    /**
     * @brief 查询底层 CAN 总线是否已打开
     * @return 已打开时返回 true
     */
    virtual bool is_open() const noexcept = 0;

    /**
     * @brief 向底层 CAN 总线发送一帧
     * @param frame 待发送 CAN 帧
     * @return 成功时返回空结果，否则返回 CAN 错误
     */
    virtual tl::expected<void, CanErr> send(const CanFrame& frame) = 0;

    /**
     * @brief 从底层 CAN 总线接收一帧
     * @param timeout 物理接收超时时间
     * @return 成功时返回 CAN 帧，否则返回 CAN 错误
     */
    virtual tl::expected<CanFrame, CanErr> receive(std::chrono::milliseconds timeout) = 0;

    /**
     * @brief 清空底层 CAN 总线缓冲区
     */
    virtual void flush() noexcept = 0;

    /**
     * @brief 创建 CAN 通道
     * @param filters CAN ID 过滤规则，空列表表示接收所有 CAN 帧
     * @return CAN 通道
     * @note 具体 CanBus 必须由 std::shared_ptr 持有后才能调用本函数
     */
    std::shared_ptr<CanChannel> create_channel(std::vector<CanFilter> filters = {});

private:
    friend class CanChannel;

    /**
     * @brief 在已经持有 rx_mutex_ 时执行一次物理接收和逻辑分发
     * @param timeout 本次物理接收超时时间
     * @return 物理接收成功或未匹配通道时返回空结果，否则返回 CAN 错误
     */
    tl::expected<void, CanErr> dispatch_once_locked(std::chrono::milliseconds timeout);

    /**
     * @brief 注册逻辑 CAN 通道
     * @param channel 待注册通道
     */
    void register_channel(const std::shared_ptr<CanChannel>& channel);

private:
    std::vector<std::weak_ptr<CanChannel>> channels_;   ///< 已注册 CAN 通道
    std::mutex channels_mutex_;                         ///< 通道列表互斥锁
    std::mutex tx_mutex_;                               ///< 物理 CAN 发送互斥锁
    std::mutex rx_mutex_;                               ///< 物理 CAN 接收互斥锁
};

/**
 * @brief 共享物理 CAN 总线的逻辑通道
 */
class CanChannel {
public:
    /**
     * @brief 发送 CAN 帧
     * @param frame CAN 帧
     * @return 如果发送成功，则返回空结果，否则返回错误类型
     */
    tl::expected<void, CanErr> send(const CanFrame& frame);

    /**
     * @brief 接收属于本通道的 CAN 帧
     * @param timeout 等待本通道 CAN 帧的总超时时间
     * @return 如果接收成功，则返回 CAN 帧，否则返回错误类型
     */
    tl::expected<CanFrame, CanErr> receive(std::chrono::milliseconds timeout);

    /**
     * @brief 清空本通道待读队列
     */
    void flush() noexcept;

    /**
     * @brief 判断 CAN 帧是否匹配本通道过滤规则
     * @param frame CAN 帧
     * @return 匹配时返回 true
     */
    bool accepts(const CanFrame& frame) const noexcept;

private:
    friend class CanBus;

    /**
     * @brief 创建逻辑 CAN 通道
     * @param bus 所属共享 CAN 总线
     * @param filters CAN ID 过滤规则
     */
    CanChannel(std::shared_ptr<CanBus> bus, std::vector<CanFilter> filters);

    /**
     * @brief 弹出本通道已分发的待读帧
     * @return 有待读帧时返回 CAN 帧，否则返回 TIMEOUT
     */
    tl::expected<CanFrame, CanErr> pop_pending();

    /**
     * @brief 将已匹配的 CAN 帧加入本通道待读队列
     * @param frame 已匹配 CAN 帧
     */
    void enqueue(const CanFrame& frame);

private:
    std::shared_ptr<CanBus> bus_;           ///< 所属 CAN 总线
    std::vector<CanFilter> filters_;        ///< CAN ID 过滤规则
    std::deque<CanFrame> pending_;          ///< 已分发到本通道的待读帧
    std::mutex mutex_;                      ///< 通道待读队列互斥锁
    std::condition_variable pending_cv_;    ///< 通道待读队列通知
};

/**
 * @brief 同进程 CAN 总线共享池
 */
class BusPool final {
public:
    using BusCreator = std::function<std::shared_ptr<CanBus>()>;

    /**
     * @brief 原子获取或创建共享 CAN 总线
     * @param name 总线名称
     * @param creator 总线创建函数
     * @return 如果存在则返回已有总线，否则创建、保存并返回新总线
     */
    static std::shared_ptr<CanBus> get_or_create(const std::string& name, const BusCreator& creator);

private:
    static std::unordered_map<std::string, std::weak_ptr<CanBus>>& buses();
    static std::mutex& mutex();
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //

} // namespace serial_arm::transport
