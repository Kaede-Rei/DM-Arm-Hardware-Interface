#include "serial_arm/transport/bus.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

namespace {

using serial_arm::transport::CanBus;
using serial_arm::transport::CanErr;
using serial_arm::transport::CanFilter;
using serial_arm::transport::CanFrame;
using serial_arm::transport::BusPool;

class MockCanBus final : public CanBus {
public:
    tl::expected<void, CanErr> open() override {
        open_ = true;
        return {};
    }

    void close() noexcept override {
        open_ = false;
    }

    bool is_open() const noexcept override {
        return open_;
    }

    tl::expected<void, CanErr> send(const CanFrame& frame) override {
        std::lock_guard<std::mutex> lock(mutex_);
        tx_.push_back(frame);
        return {};
    }

    tl::expected<CanFrame, CanErr> receive(std::chrono::milliseconds) override {
        std::lock_guard<std::mutex> lock(mutex_);
        if(rx_.empty()) return tl::make_unexpected(CanErr::TIMEOUT);
        CanFrame frame = rx_.front();
        rx_.pop_front();
        return frame;
    }

    void flush() noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        rx_.clear();
    }

    void push_rx(const CanFrame& frame) {
        std::lock_guard<std::mutex> lock(mutex_);
        rx_.push_back(frame);
    }

    std::size_t tx_size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return tx_.size();
    }

private:
    bool open_{ false };
    mutable std::mutex mutex_;
    std::deque<CanFrame> rx_;
    std::deque<CanFrame> tx_;
};

class BlockingMockCanBus final : public CanBus {
public:
    tl::expected<void, CanErr> open() override {
        open_ = true;
        return {};
    }

    void close() noexcept override {
        open_ = false;
    }

    bool is_open() const noexcept override {
        return open_;
    }

    tl::expected<void, CanErr> send(const CanFrame& frame) override {
        std::lock_guard<std::mutex> lock(mutex_);
        tx_.push_back(frame);
        return {};
    }

    tl::expected<CanFrame, CanErr> receive(std::chrono::milliseconds timeout) override {
        std::this_thread::sleep_for(timeout);
        std::lock_guard<std::mutex> lock(mutex_);
        if(rx_.empty()) return tl::make_unexpected(CanErr::TIMEOUT);
        CanFrame frame = rx_.front();
        rx_.pop_front();
        return frame;
    }

    void flush() noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        rx_.clear();
    }

    void push_rx(const CanFrame& frame) {
        std::lock_guard<std::mutex> lock(mutex_);
        rx_.push_back(frame);
    }

    std::size_t tx_size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return tx_.size();
    }

private:
    bool open_{ false };
    mutable std::mutex mutex_;
    std::deque<CanFrame> rx_;
    std::deque<CanFrame> tx_;
};

CanFrame frame(std::uint32_t id) {
    CanFrame value;
    value.id = id;
    value.size = 1;
    value.data[0] = static_cast<std::uint8_t>(id);
    return value;
}

} // namespace

TEST(TransportTests, SingleChannelReceivesMatchingFrame) {
    auto bus = std::make_shared<MockCanBus>();
    ASSERT_TRUE(bus->open());
    auto channel = bus->create_channel({ CanFilter{ 0x01, 0x7FF } });
    bus->push_rx(frame(0x01));

    auto received = channel->receive(std::chrono::milliseconds(5));
    ASSERT_TRUE(received);
    EXPECT_EQ(received->id, 0x01u);
}

TEST(TransportTests, ChannelDoesNotDropOtherChannelFrame) {
    auto bus = std::make_shared<MockCanBus>();
    ASSERT_TRUE(bus->open());
    auto arm = bus->create_channel({ CanFilter{ 0x01, 0x7FF } });
    auto eef = bus->create_channel({ CanFilter{ 0x20, 0x7FF } });
    bus->push_rx(frame(0x20));
    bus->push_rx(frame(0x01));

    auto arm_frame = arm->receive(std::chrono::milliseconds(5));
    ASSERT_TRUE(arm_frame);
    EXPECT_EQ(arm_frame->id, 0x01u);

    auto eef_frame = eef->receive(std::chrono::milliseconds(5));
    ASSERT_TRUE(eef_frame);
    EXPECT_EQ(eef_frame->id, 0x20u);
}

TEST(TransportTests, ChannelFlushDoesNotClearOtherChannelPendingFrame) {
    auto bus = std::make_shared<MockCanBus>();
    ASSERT_TRUE(bus->open());
    auto arm = bus->create_channel({ CanFilter{ 0x01, 0x7FF } });
    auto eef = bus->create_channel({ CanFilter{ 0x20, 0x7FF } });
    bus->push_rx(frame(0x20));
    bus->push_rx(frame(0x01));

    auto arm_frame = arm->receive(std::chrono::milliseconds(5));
    ASSERT_TRUE(arm_frame);
    arm->flush();

    auto eef_frame = eef->receive(std::chrono::milliseconds(5));
    ASSERT_TRUE(eef_frame);
    EXPECT_EQ(eef_frame->id, 0x20u);
}

TEST(TransportTests, UnknownFrameDoesNotEnterOtherChannels) {
    auto bus = std::make_shared<MockCanBus>();
    ASSERT_TRUE(bus->open());
    auto arm = bus->create_channel({ CanFilter{ 0x01, 0x7FF } });
    bus->push_rx(frame(0x30));

    auto received = arm->receive(std::chrono::milliseconds(5));
    ASSERT_FALSE(received);
    EXPECT_EQ(received.error(), CanErr::TIMEOUT);
}

TEST(TransportTests, ConcurrentTxRxDoesNotCrossChannels) {
    auto bus = std::make_shared<MockCanBus>();
    ASSERT_TRUE(bus->open());
    auto arm = bus->create_channel({ CanFilter{ 0x01, 0x7FF } });
    auto eef = bus->create_channel({ CanFilter{ 0x20, 0x7FF } });

    for(int i = 0; i < 20; ++i) {
        bus->push_rx(frame(i % 2 == 0 ? 0x01 : 0x20));
    }

    std::size_t arm_count = 0;
    std::size_t eef_count = 0;
    std::thread arm_thread([&]() {
        for(int i = 0; i < 10; ++i) {
            auto received = arm->receive(std::chrono::milliseconds(20));
            if(received && received->id == 0x01) ++arm_count;
            (void)arm->send(frame(0x01));
        }
    });
    std::thread eef_thread([&]() {
        for(int i = 0; i < 10; ++i) {
            auto received = eef->receive(std::chrono::milliseconds(20));
            if(received && received->id == 0x20) ++eef_count;
            (void)eef->send(frame(0x20));
        }
    });

    arm_thread.join();
    eef_thread.join();

    EXPECT_EQ(arm_count, 10u);
    EXPECT_EQ(eef_count, 10u);
    EXPECT_EQ(bus->tx_size(), 20u);
}

TEST(TransportTests, BusPoolGetOrCreateIsAtomic) {
    constexpr int thread_count = 20;
    std::atomic<int> create_count{ 0 };
    std::vector<std::shared_ptr<CanBus>> buses(thread_count);
    std::vector<std::thread> threads;
    threads.reserve(thread_count);

    for(int i = 0; i < thread_count; ++i) {
        threads.emplace_back([&, i]() {
            buses[i] = BusPool::get_or_create("transport_test_atomic", [&]() {
                create_count.fetch_add(1);
                auto bus = std::make_shared<MockCanBus>();
                (void)bus->open();
                return bus;
            });
        });
    }

    for(auto& thread : threads) thread.join();

    ASSERT_TRUE(buses.front());
    for(const auto& bus : buses) {
        EXPECT_EQ(bus.get(), buses.front().get());
    }
    EXPECT_EQ(create_count.load(), 1);
}

TEST(TransportTests, LongReceiveDoesNotBlockSend) {
    auto bus = std::make_shared<BlockingMockCanBus>();
    ASSERT_TRUE(bus->open());
    auto arm = bus->create_channel({ CanFilter{ 0x01, 0x7FF } });
    auto eef = bus->create_channel({ CanFilter{ 0x20, 0x7FF } });

    std::atomic<bool> receive_started{ false };
    std::thread receiver([&]() {
        receive_started = true;
        (void)eef->receive(std::chrono::milliseconds(100));
    });

    while(!receive_started) std::this_thread::yield();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    const auto start = std::chrono::steady_clock::now();
    auto sent = arm->send(frame(0x01));
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start);

    receiver.join();
    ASSERT_TRUE(sent);
    EXPECT_LT(elapsed.count(), 10);
    EXPECT_EQ(bus->tx_size(), 1u);
}

TEST(TransportTests, LongEefReceiveDoesNotMonopolizeArmReceive) {
    auto bus = std::make_shared<BlockingMockCanBus>();
    ASSERT_TRUE(bus->open());
    auto arm = bus->create_channel({ CanFilter{ 0x01, 0x7FF } });
    auto eef = bus->create_channel({ CanFilter{ 0x20, 0x7FF } });

    std::atomic<bool> eef_started{ false };
    std::thread eef_thread([&]() {
        eef_started = true;
        (void)eef->receive(std::chrono::milliseconds(100));
    });

    while(!eef_started) std::this_thread::yield();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    bus->push_rx(frame(0x01));

    const auto start = std::chrono::steady_clock::now();
    auto received = arm->receive(std::chrono::milliseconds(80));
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start);

    eef_thread.join();
    ASSERT_TRUE(received);
    EXPECT_EQ(received->id, 0x01u);
    EXPECT_LT(elapsed.count(), 20);
}
