#include "serial_arm/transport/serial_port.hpp"

#include <gtest/gtest.h>

#include <array>
#include <fcntl.h>
#include <stdexcept>
#include <string>
#include <unistd.h>

namespace {

using serial_arm::transport::SerialPort;

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

    Pty(const Pty&) = delete;
    Pty& operator=(const Pty&) = delete;

    int master() const noexcept { return master_; }
    const std::string& slave() const noexcept { return slave_; }

private:
    int master_{ -1 };
    std::string slave_;
};

} // namespace

TEST(SerialPortTests, RejectsInvalidConfiguration) {
    SerialPort::Config config;
    config.baud_rate = 12345;
    EXPECT_THROW(SerialPort("/dev/null", config), std::invalid_argument);

    config = SerialPort::Config{};
    config.data_bits = 9;
    EXPECT_THROW(SerialPort("/dev/null", config), std::invalid_argument);

    config = SerialPort::Config{};
    config.read_timeout = std::chrono::milliseconds(-1);
    EXPECT_THROW(SerialPort("/dev/null", config), std::invalid_argument);

    config = SerialPort::Config{};
    config.write_timeout = std::chrono::milliseconds(-1);
    EXPECT_THROW(SerialPort("/dev/null", config), std::invalid_argument);
}

TEST(SerialPortTests, OpensAndReopensWithoutDroppingOldFdOnFailure) {
    Pty first;
    Pty second;
    SerialPort port(first.slave());
    ASSERT_TRUE(port.is_open());

    const std::array<std::uint8_t, 1> old_byte{ 0x41 };
    ASSERT_EQ(::write(first.master(), old_byte.data(), old_byte.size()), 1);
    std::array<std::uint8_t, 1> read_buf{};
    EXPECT_EQ(port.read_exact(read_buf.data(), read_buf.size()), 1);
    EXPECT_EQ(read_buf[0], old_byte[0]);

    SerialPort::Config invalid;
    invalid.baud_rate = 12345;
    EXPECT_THROW(port.open(second.slave(), invalid), std::invalid_argument);
    ASSERT_TRUE(port.is_open());
    const std::array<std::uint8_t, 1> still_old{ 0x42 };
    ASSERT_EQ(::write(first.master(), still_old.data(), still_old.size()), 1);
    EXPECT_EQ(port.read_exact(read_buf.data(), read_buf.size()), 1);
    EXPECT_EQ(read_buf[0], still_old[0]);

    port.open(second.slave());
    const std::array<std::uint8_t, 1> new_byte{ 0x43 };
    ASSERT_EQ(::write(second.master(), new_byte.data(), new_byte.size()), 1);
    EXPECT_EQ(port.read_exact(read_buf.data(), read_buf.size()), 1);
    EXPECT_EQ(read_buf[0], new_byte[0]);
}

TEST(SerialPortTests, ReadWriteTimeoutFlushDrainAndMove) {
    Pty pty;
    SerialPort::Config config;
    config.read_timeout = std::chrono::milliseconds(5);
    config.write_timeout = std::chrono::milliseconds(50);
    SerialPort port(pty.slave(), config);
    EXPECT_TRUE(port.is_open());
    EXPECT_EQ(port.config().read_timeout, std::chrono::milliseconds(5));
    EXPECT_EQ(port.config().write_timeout, std::chrono::milliseconds(50));

    const std::array<std::uint8_t, 3> out{ 1, 2, 3 };
    EXPECT_EQ(port.write(out.data(), out.size()), out.size());
    std::array<std::uint8_t, 3> from_port{};
    ASSERT_EQ(::read(pty.master(), from_port.data(), from_port.size()), 3);
    EXPECT_EQ(from_port, out);
    EXPECT_NO_THROW(port.drain());

    std::array<std::uint8_t, 2> in{ 4, 5 };
    ASSERT_EQ(::write(pty.master(), in.data(), in.size()), 2);
    EXPECT_GE(port.available(), 0u);
    std::array<std::uint8_t, 2> to_port{};
    EXPECT_EQ(port.read(to_port.data(), to_port.size()), to_port.size());
    EXPECT_EQ(to_port, in);
    EXPECT_EQ(port.read(to_port.data(), to_port.size()), 0u);

    port.flush();
    SerialPort moved(std::move(port));
    EXPECT_TRUE(moved.is_open());
    EXPECT_FALSE(port.is_open());
    SerialPort assigned;
    assigned = std::move(moved);
    EXPECT_TRUE(assigned.is_open());
    EXPECT_FALSE(moved.is_open());
}

TEST(SerialPortTests, BufferApisAndIndependentTimeoutSetters) {
    Pty pty;
    SerialPort port(pty.slave());
    port.set_read_timeout(std::chrono::milliseconds(3));
    port.set_write_timeout(std::chrono::milliseconds(40));
    EXPECT_EQ(port.config().read_timeout, std::chrono::milliseconds(3));
    EXPECT_EQ(port.config().write_timeout, std::chrono::milliseconds(40));

    EXPECT_EQ(port.write({ 7, 8 }), 2u);
    std::array<std::uint8_t, 2> from_port{};
    ASSERT_EQ(::read(pty.master(), from_port.data(), from_port.size()), 2);
    EXPECT_EQ(from_port[0], 7);
    EXPECT_EQ(from_port[1], 8);

    ASSERT_EQ(::write(pty.master(), from_port.data(), from_port.size()), 2);
    SerialPort::Buffer buffer;
    EXPECT_EQ(port.read(buffer, 2), 2u);
    EXPECT_EQ(buffer.size(), 2u);
}
