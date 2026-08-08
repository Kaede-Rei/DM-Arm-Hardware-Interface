#include "serial_arm/transport/bus.hpp"

#include <algorithm>

namespace serial_arm::transport {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //

namespace {

constexpr auto DISPATCH_SLICE = std::chrono::milliseconds(2);

std::chrono::milliseconds min_timeout(std::chrono::milliseconds lhs, std::chrono::milliseconds rhs) {
    return lhs < rhs ? lhs : rhs;
}

} // namespace


// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

/**
 * @brief 创建 CAN 通道
 */
std::shared_ptr<CanChannel> CanBus::create_channel(std::vector<CanFilter> filters) {
    std::shared_ptr<CanChannel> channel(new CanChannel(shared_from_this(), std::move(filters)));
    register_channel(channel);
    return channel;
}

/**
 * @brief 执行一次物理接收并分发到匹配通道
 */
tl::expected<void, CanErr> CanBus::dispatch_once_locked(std::chrono::milliseconds timeout) {
    auto frame = receive(timeout);
    if(!frame) return tl::make_unexpected(frame.error());

    std::lock_guard<std::mutex> channels_lock(channels_mutex_);
    channels_.erase(std::remove_if(channels_.begin(), channels_.end(),
        [](const std::weak_ptr<CanChannel>& channel) {
            return channel.expired();
        }), channels_.end());

    for(const auto& weak_channel : channels_) {
        auto channel = weak_channel.lock();
        if(channel && channel->accepts(*frame)) {
            channel->enqueue(*frame);
        }
    }

    return {};
}

/**
 * @brief 注册逻辑 CAN 通道
 */
void CanBus::register_channel(const std::shared_ptr<CanChannel>& channel) {
    std::lock_guard<std::mutex> lock(channels_mutex_);
    channels_.push_back(channel);
}

/**
 * @brief 创建逻辑 CAN 通道
 */
CanChannel::CanChannel(std::shared_ptr<CanBus> bus, std::vector<CanFilter> filters)
    : bus_(std::move(bus)), filters_(std::move(filters)) {}

/**
 * @brief 发送 CAN 帧
 */
tl::expected<void, CanErr> CanChannel::send(const CanFrame& frame) {
    if(!bus_) return tl::make_unexpected(CanErr::NOT_OPEN);
    std::lock_guard<std::mutex> lock(bus_->tx_mutex_);
    return bus_->send(frame);
}

/**
 * @brief 接收属于本通道的 CAN 帧
 */
tl::expected<CanFrame, CanErr> CanChannel::receive(std::chrono::milliseconds timeout) {
    if(!bus_) return tl::make_unexpected(CanErr::NOT_OPEN);

    auto pending = pop_pending();
    if(pending) return pending;

    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while(true) {
        const auto now = std::chrono::steady_clock::now();
        if(now >= deadline) return tl::make_unexpected(CanErr::TIMEOUT);

        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
        const auto physical_timeout = min_timeout(remaining, DISPATCH_SLICE);

        if(bus_->rx_mutex_.try_lock()) {
            std::unique_lock<std::mutex> rx_lock(bus_->rx_mutex_, std::adopt_lock);
            pending = pop_pending();
            if(pending) return pending;
            auto dispatched = bus_->dispatch_once_locked(physical_timeout);
            rx_lock.unlock();

            if(!dispatched && dispatched.error() != CanErr::TIMEOUT) return tl::make_unexpected(dispatched.error());
        }
        else {
            std::unique_lock<std::mutex> lock(mutex_);
            pending_cv_.wait_for(lock, physical_timeout, [this]() {
                return !pending_.empty();
                });
        }

        pending = pop_pending();
        if(pending) return pending;
    }
}

/**
 * @brief 清空本通道待读队列
 */
void CanChannel::flush() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_.clear();
}

/**
 * @brief 判断 CAN 帧是否匹配本通道过滤规则
 */
bool CanChannel::accepts(const CanFrame& frame) const noexcept {
    if(filters_.empty()) return true;
    for(const auto& filter : filters_) {
        if((frame.id & filter.mask) == (filter.id & filter.mask)) return true;
    }
    return false;
}

// ! ========================= 私 有 类 方 法 实 现 ========================= ! //

/**
 * @brief 弹出本通道已分发的待读帧
 */
tl::expected<CanFrame, CanErr> CanChannel::pop_pending() {
    std::lock_guard<std::mutex> lock(mutex_);
    if(pending_.empty()) return tl::make_unexpected(CanErr::TIMEOUT);
    CanFrame frame = pending_.front();
    pending_.pop_front();
    return frame;
}

/**
 * @brief 将匹配帧加入本通道待读队列
 */
void CanChannel::enqueue(const CanFrame& frame) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        pending_.push_back(frame);
    }
    pending_cv_.notify_one();
}

/**
 * @brief 原子获取或创建共享 CAN 总线
 */
std::shared_ptr<CanBus> BusPool::get_or_create(const std::string& name, const BusCreator& creator) {
    std::lock_guard<std::mutex> lock(mutex());
    auto& pool = buses();
    auto it = pool.find(name);
    if(it != pool.end()) {
        auto bus = it->second.lock();
        if(bus) return bus;
        pool.erase(it);
    }

    auto bus = creator();
    if(bus) pool[name] = bus;
    return bus;
}

/**
 * @brief 获取进程内总线弱引用表
 */
std::unordered_map<std::string, std::weak_ptr<CanBus>>& BusPool::buses() {
    static std::unordered_map<std::string, std::weak_ptr<CanBus>> instance;
    return instance;
}

/**
 * @brief 获取总线共享池互斥锁
 */
std::mutex& BusPool::mutex() {
    static std::mutex instance;
    return instance;
}

} // namespace serial_arm::transport
