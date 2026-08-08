#include "serial_arm_protocol_damiao_usb2can/bus.hpp"

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <fcntl.h>
#include <stdexcept>
#include <string>
#include <sys/select.h>
#include <thread>
#include <unistd.h>

namespace {

namespace damiao_usb2can = serial_arm::protocol::damiao_usb2can;
using serial_arm::transport::CanFilter;
using serial_arm::transport::CanFrame;
using serial_arm::transport::CanErr;

class Pty {
public:
    Pty() {
        master_ = ::posix_openpt(O_RDWR | O_NOCTTY);
        if(master_ < 0) throw std::runtime_error("posix_openpt failed");
        if(::grantpt(master_) != 0 || ::unlockpt(master_) != 0) throw std::runtime_error("pty setup failed");
        const char* name = ::ptsname(master_);
        if(name == nullptr) throw std::runtime_error("ptsname failed");
        slave_ = name;
    }

    ~Pty() {
        if(master_ >= 0) (void)::close(master_);
    }

    int master() const noexcept { return master_; }
    const std::string& slave() const noexcept { return slave_; }

private:
    int master_{ -1 };
    std::string slave_;
};

damiao_usb2can::Config config(const Pty& pty) {
    damiao_usb2can::Config cfg;
    cfg.serial_port = pty.slave();
    cfg.baudrate = 115200;
    return cfg;
}

CanFrame frame(std::uint32_t id) {
    CanFrame value;
    value.id = id;
    value.size = 8;
    for(std::uint8_t i = 0; i < value.size; ++i) value.data[i] = static_cast<std::uint8_t>(i + 1);
    return value;
}

std::array<std::uint8_t, 16> rx_packet(std::uint32_t id, std::uint8_t first_data) {
    std::array<std::uint8_t, 16> packet{};
    packet[0] = 0xAA;
    packet[1] = 0x11;
    packet[2] = 8;
    packet[3] = static_cast<std::uint8_t>(id & 0xFF);
    packet[4] = static_cast<std::uint8_t>((id >> 8) & 0xFF);
    packet[5] = static_cast<std::uint8_t>((id >> 16) & 0xFF);
    packet[6] = static_cast<std::uint8_t>((id >> 24) & 0xFF);
    for(std::uint8_t i = 0; i < 8; ++i) packet[7 + i] = static_cast<std::uint8_t>(first_data + i);
    packet[15] = 0x55;
    return packet;
}

std::size_t read_available(int fd, std::uint8_t* data, std::size_t size) {
    fd_set rset;
    FD_ZERO(&rset);
    FD_SET(fd, &rset);
    timeval tv{};
    tv.tv_sec = 0;
    tv.tv_usec = 20000;
    if(::select(fd + 1, &rset, nullptr, nullptr, &tv) <= 0) return 0;
    const ssize_t n = ::read(fd, data, size);
    return n > 0 ? static_cast<std::size_t>(n) : 0;
}

} // namespace

TEST(DamiaoUsbCanBusTests, FlushBeforeOpenDoesNotThrow) {
    damiao_usb2can::DamiaoUsbCanBus bus(damiao_usb2can::Config{});
    EXPECT_NO_THROW(bus.flush());
}

TEST(DamiaoUsbCanBusTests, OpensWithPtyAndSendsStandardCanFrame) {
    Pty pty;
    auto channel = damiao_usb2can::acquire_channel("protocol_send", config(pty), { CanFilter{ 0x01, 0x7FF } });
    ASSERT_TRUE(channel);

    ASSERT_TRUE((*channel)->send(frame(0x123)));
    std::array<std::uint8_t, 64> bytes{};
    const std::size_t n = read_available(pty.master(), bytes.data(), bytes.size());
    ASSERT_GE(n, 30u);
    EXPECT_EQ(bytes[0], 0x55);
    EXPECT_EQ(bytes[1], 0xAA);
    EXPECT_EQ(bytes[12], 0x00);
    EXPECT_EQ(bytes[13], 0x23);
    EXPECT_EQ(bytes[14], 0x01);
}

TEST(DamiaoUsbCanBusTests, RejectsExtendedCanId) {
    Pty pty;
    auto channel = damiao_usb2can::acquire_channel("protocol_invalid_id", config(pty), {});
    ASSERT_TRUE(channel);

    auto result = (*channel)->send(frame(0x800));
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), CanErr::INVALID_FRAME);
}

TEST(DamiaoUsbCanBusTests, ReceivesTwoBufferedPacketsWithoutSecondDelay) {
    Pty pty;
    auto channel = damiao_usb2can::acquire_channel("protocol_buffered_rx", config(pty), {});
    ASSERT_TRUE(channel);
    const auto first = rx_packet(0x01, 10);
    const auto second = rx_packet(0x02, 20);
    ASSERT_EQ(::write(pty.master(), first.data(), first.size()), static_cast<ssize_t>(first.size()));
    ASSERT_EQ(::write(pty.master(), second.data(), second.size()), static_cast<ssize_t>(second.size()));

    auto first_frame = (*channel)->receive(std::chrono::milliseconds(20));
    ASSERT_TRUE(first_frame);
    EXPECT_EQ(first_frame->id, 0x01u);

    const auto start = std::chrono::steady_clock::now();
    auto second_frame = (*channel)->receive(std::chrono::milliseconds(20));
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start);
    ASSERT_TRUE(second_frame);
    EXPECT_EQ(second_frame->id, 0x02u);
    EXPECT_LT(elapsed.count(), 10);
}

TEST(DamiaoUsbCanBusTests, ReceivesFragmentedPacketWithinTimeout) {
    Pty pty;
    damiao_usb2can::DamiaoUsbCanBus bus(config(pty));
    ASSERT_TRUE(bus.open());
    const auto packet = rx_packet(0x321, 30);

    std::thread writer([&]() {
        ASSERT_EQ(::write(pty.master(), packet.data(), 8), 8);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        ASSERT_EQ(::write(pty.master(), packet.data() + 8, packet.size() - 8),
            static_cast<ssize_t>(packet.size() - 8));
    });

    auto received = bus.receive(std::chrono::milliseconds(20));
    writer.join();

    ASSERT_TRUE(received);
    EXPECT_EQ(received->id, 0x321u);
    EXPECT_EQ(received->size, 8u);
    EXPECT_EQ(received->data[0], 30u);
}

TEST(DamiaoUsbCanBusTests, FragmentedPacketTimesOutOnlyAfterBudget) {
    Pty pty;
    damiao_usb2can::DamiaoUsbCanBus bus(config(pty));
    ASSERT_TRUE(bus.open());
    const auto packet = rx_packet(0x321, 30);
    ASSERT_EQ(::write(pty.master(), packet.data(), 8), 8);

    const auto start = std::chrono::steady_clock::now();
    auto received = bus.receive(std::chrono::milliseconds(20));
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start);

    ASSERT_FALSE(received);
    EXPECT_EQ(received.error(), CanErr::TIMEOUT);
    EXPECT_GE(elapsed.count(), 15);
}

TEST(DamiaoUsbCanBusTests, InvalidPacketResyncsToNextHeader) {
    Pty pty;
    auto channel = damiao_usb2can::acquire_channel("protocol_resync", config(pty), {});
    ASSERT_TRUE(channel);

    auto invalid = rx_packet(0x01, 1);
    invalid[15] = 0x00;
    const auto valid = rx_packet(0x02, 2);
    ASSERT_EQ(::write(pty.master(), invalid.data(), invalid.size()), static_cast<ssize_t>(invalid.size()));
    ASSERT_EQ(::write(pty.master(), valid.data(), valid.size()), static_cast<ssize_t>(valid.size()));

    auto received = (*channel)->receive(std::chrono::milliseconds(20));
    ASSERT_TRUE(received);
    EXPECT_EQ(received->id, 0x02u);
}

TEST(DamiaoUsbCanBusTests, DetectsConfigMismatchForSharedBusName) {
    Pty pty;
    auto first = damiao_usb2can::acquire_channel("protocol_config_mismatch", config(pty), {});
    ASSERT_TRUE(first);

    auto bad_config = config(pty);
    bad_config.serial_port = "/dev/does-not-match";
    auto second = damiao_usb2can::acquire_channel("protocol_config_mismatch", bad_config, {});
    ASSERT_FALSE(second);
    EXPECT_EQ(second.error(), damiao_usb2can::Err::CONFIG_CONFLICT);
}

TEST(DamiaoUsbCanBusTests, ConcurrentAcquireSameBusSucceeds) {
    Pty pty;
    constexpr int thread_count = 20;
    std::array<bool, thread_count> ok{};
    std::array<std::thread, thread_count> threads;

    for(int i = 0; i < thread_count; ++i) {
        threads[i] = std::thread([&, i]() {
            auto channel = damiao_usb2can::acquire_channel("protocol_concurrent_acquire", config(pty), {});
            ok[i] = channel.has_value();
        });
    }
    for(auto& thread : threads) thread.join();
    for(bool value : ok) EXPECT_TRUE(value);
}
