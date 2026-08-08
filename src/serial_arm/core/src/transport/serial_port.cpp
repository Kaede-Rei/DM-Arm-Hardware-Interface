#include "serial_arm/transport/serial_port.hpp"

#include <cerrno>
#include <limits>
#include <stdexcept>
#include <system_error>
#include <utility>

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace serial_arm::transport {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //



// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

/**
 * @brief 使用默认配置打开串口
 */
SerialPort::SerialPort(std::string port) {
    open(std::move(port));
}

/**
 * @brief 使用指定配置打开串口
 */
SerialPort::SerialPort(std::string port, const Config& config) {
    open(std::move(port), config);
}

/**
 * @brief 关闭串口
 */
SerialPort::~SerialPort() noexcept {
    close();
}

/**
 * @brief 移动构造并接管文件描述符
 */
SerialPort::SerialPort(SerialPort&& other) noexcept
    : fd_(std::exchange(other.fd_, -1)), port_(std::move(other.port_)), config_(other.config_) {
}

/**
 * @brief 移动赋值并接管文件描述符
 */
SerialPort& SerialPort::operator=(SerialPort&& other) noexcept {
    if(this != &other) {
        close();
        fd_ = std::exchange(other.fd_, -1);
        port_ = std::move(other.port_);
        config_ = other.config_;
    }
    return *this;
}

/**
 * @brief 使用当前配置打开串口
 */
void SerialPort::open(std::string port) {
    open(std::move(port), config_);
}

/**
 * @brief 使用指定配置打开串口
 */
void SerialPort::open(std::string port, const Config& config) {
    validate_config(config);

    int new_fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if(new_fd < 0) throw_system_error("open(" + port + ")");

    try {
        configure_fd(new_fd, config);
        if(config.flush_on_open) flush_fd(new_fd, FlushDirection::Both);
    }
    catch(...) {
        (void)::close(new_fd);
        throw;
    }

    close();
    fd_ = new_fd;
    port_ = std::move(port);
    config_ = config;
}

/**
 * @brief 关闭串口文件描述符
 */
void SerialPort::close() noexcept {
    if(fd_ >= 0) {
        (void)::close(fd_);
        fd_ = -1;
    }
    port_.clear();
}

/**
 * @brief 查询串口是否已打开
 */
bool SerialPort::is_open() const noexcept {
    return fd_ >= 0;
}

/**
 * @brief 获取底层文件描述符
 */
int SerialPort::native_handle() const noexcept {
    return fd_;
}

/**
 * @brief 获取串口设备路径
 */
const std::string& SerialPort::port() const noexcept {
    return port_;
}

/**
 * @brief 获取当前串口配置
 */
const SerialPort::Config& SerialPort::config() const noexcept {
    return config_;
}

/**
 * @brief 更新串口配置
 */
void SerialPort::set_config(const Config& config) {
    validate_config(config);
    if(is_open()) configure_fd(fd_, config);
    config_ = config;
}

/**
 * @brief 设置读取超时时间
 */
void SerialPort::set_read_timeout(std::chrono::milliseconds timeout) {
    validate_timeout(timeout, "read_timeout");
    config_.read_timeout = timeout;
}

/**
 * @brief 设置写入超时时间
 */
void SerialPort::set_write_timeout(std::chrono::milliseconds timeout) {
    validate_timeout(timeout, "write_timeout");
    config_.write_timeout = timeout;
}

/**
 * @brief 最多读取指定字节数
 */
std::size_t SerialPort::read(Byte* data, std::size_t len) {
    ensure_open();
    validate_buffer(data, len);
    if(len == 0) return 0;
    if(!wait_ready(POLLIN, config_.read_timeout)) return 0;

    for(;;) {
        const ssize_t ret = ::read(fd_, data, len);
        if(ret >= 0) return static_cast<std::size_t>(ret);
        if(errno == EINTR) continue;
        if(errno == EAGAIN || errno == EWOULDBLOCK) return 0;
        throw_system_error("read(" + port_ + ")");
    }
}

/**
 * @brief 最多读取指定字节数并返回缓冲区
 */
SerialPort::Buffer SerialPort::read(std::size_t max_bytes) {
    Buffer buffer(max_bytes);
    const std::size_t received = read(buffer.data(), buffer.size());
    buffer.resize(received);
    return buffer;
}

/**
 * @brief 最多读取指定字节数到目标缓冲区
 */
std::size_t SerialPort::read(Buffer& buffer, std::size_t max_bytes) {
    buffer.resize(max_bytes);
    const std::size_t received = read(buffer.data(), buffer.size());
    buffer.resize(received);
    return received;
}

/**
 * @brief 在读取超时时间内尽量读满指定长度
 */
std::size_t SerialPort::read_exact(Byte* data, std::size_t len) {
    ensure_open();
    validate_buffer(data, len);
    if(len == 0) return 0;

    const auto deadline = std::chrono::steady_clock::now() + config_.read_timeout;
    std::size_t total = 0;
    while(total < len) {
        const auto remaining = remaining_time(deadline);
        if(!wait_ready(POLLIN, remaining)) break;

        const ssize_t ret = ::read(fd_, data + total, len - total);
        if(ret > 0) {
            total += static_cast<std::size_t>(ret);
            continue;
        }
        if(ret == 0) break;
        if(errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) continue;
        throw_system_error("read(" + port_ + ")");
    }
    return total;
}

/**
 * @brief 在读取超时时间内尽量读满指定长度并返回缓冲区
 */
SerialPort::Buffer SerialPort::read_exact(std::size_t len) {
    Buffer buffer(len);
    const std::size_t received = read_exact(buffer.data(), buffer.size());
    buffer.resize(received);
    return buffer;
}

/**
 * @brief 在读取超时时间内尽量读满指定长度到目标缓冲区
 */
std::size_t SerialPort::read_exact(Buffer& buffer, std::size_t len) {
    buffer.resize(len);
    const std::size_t received = read_exact(buffer.data(), buffer.size());
    buffer.resize(received);
    return received;
}

/**
 * @brief 在写入超时时间内尽量写出指定长度
 */
std::size_t SerialPort::write(const Byte* data, std::size_t len) {
    ensure_open();
    validate_buffer(data, len);
    if(len == 0) return 0;

    const auto deadline = std::chrono::steady_clock::now() + config_.write_timeout;
    std::size_t total = 0;
    while(total < len) {
        const auto remaining = remaining_time(deadline);
        if(!wait_ready(POLLOUT, remaining)) break;

        const ssize_t ret = ::write(fd_, data + total, len - total);
        if(ret > 0) {
            total += static_cast<std::size_t>(ret);
            continue;
        }
        if(ret == 0) break;
        if(errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) continue;
        throw_system_error("write(" + port_ + ")");
    }
    return total;
}

/**
 * @brief 写出字节缓冲区
 */
std::size_t SerialPort::write(const Buffer& data) {
    return write(data.data(), data.size());
}

/**
 * @brief 写出初始化列表
 */
std::size_t SerialPort::write(std::initializer_list<Byte> data) {
    return write(data.begin(), data.size());
}

/**
 * @brief 等待输出队列发送完成
 */
void SerialPort::drain() {
    ensure_open();
    int ret;
    do {
        ret = ::tcdrain(fd_);
    }
    while(ret < 0 && errno == EINTR);
    if(ret < 0) throw_system_error("tcdrain(" + port_ + ")");
}

/**
 * @brief 清空串口缓冲区
 */
void SerialPort::flush(FlushDirection direction) {
    ensure_open();
    flush_fd(fd_, direction);
}

/**
 * @brief 查询内核输入缓冲区可读字节数
 */
std::size_t SerialPort::available() const {
    ensure_open();
    int bytes = 0;
    if(::ioctl(fd_, FIONREAD, &bytes) < 0) throw_system_error("ioctl(FIONREAD, " + port_ + ")");
    return bytes > 0 ? static_cast<std::size_t>(bytes) : 0U;
}

// ! ========================= 私 有 类 方 法 实 现 ========================= ! //

/**
 * @brief 校验非空缓冲区
 */
void SerialPort::validate_buffer(const void* data, std::size_t len) {
    if(data == nullptr && len != 0) {
        throw std::invalid_argument("SerialPort buffer is null while len != 0");
    }
}

/**
 * @brief 校验超时时间非负
 */
void SerialPort::validate_timeout(std::chrono::milliseconds timeout, const char* name) {
    if(timeout.count() < 0) {
        throw std::invalid_argument(std::string("SerialPort ") + name + " must be >= 0 ms");
    }
}

/**
 * @brief 校验串口配置
 */
void SerialPort::validate_config(const Config& config) {
    if(config.data_bits < 5 || config.data_bits > 8) {
        throw std::invalid_argument("SerialPort data_bits must be in [5, 8]");
    }
    if(config.baud_rate == 0) {
        throw std::invalid_argument("SerialPort baud_rate must be > 0");
    }
    validate_timeout(config.read_timeout, "read_timeout");
    validate_timeout(config.write_timeout, "write_timeout");
    (void)baud_to_speed(config.baud_rate);
}

/**
 * @brief 转换 termios 波特率
 */
speed_t SerialPort::baud_to_speed(std::uint32_t baud_rate) {
    switch(baud_rate) {
        case 50: return B50;
        case 75: return B75;
        case 110: return B110;
        case 134: return B134;
        case 150: return B150;
        case 200: return B200;
        case 300: return B300;
        case 600: return B600;
        case 1200: return B1200;
        case 1800: return B1800;
        case 2400: return B2400;
        case 4800: return B4800;
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
#ifdef B57600
        case 57600: return B57600;
#endif
#ifdef B115200
        case 115200: return B115200;
#endif
#ifdef B230400
        case 230400: return B230400;
#endif
#ifdef B460800
        case 460800: return B460800;
#endif
#ifdef B500000
        case 500000: return B500000;
#endif
#ifdef B576000
        case 576000: return B576000;
#endif
#ifdef B921600
        case 921600: return B921600;
#endif
#ifdef B1000000
        case 1000000: return B1000000;
#endif
#ifdef B1152000
        case 1152000: return B1152000;
#endif
#ifdef B1500000
        case 1500000: return B1500000;
#endif
#ifdef B2000000
        case 2000000: return B2000000;
#endif
#ifdef B2500000
        case 2500000: return B2500000;
#endif
#ifdef B3000000
        case 3000000: return B3000000;
#endif
#ifdef B3500000
        case 3500000: return B3500000;
#endif
#ifdef B4000000
        case 4000000: return B4000000;
#endif
        default:
            throw std::invalid_argument("SerialPort baud_rate is not supported by this termios platform: " + std::to_string(baud_rate));
    }
}

/**
 * @brief 配置串口文件描述符
 */
void SerialPort::configure_fd(int fd, const Config& config) {
    termios tty{};
    if(::tcgetattr(fd, &tty) < 0) throw_system_error("tcgetattr");
    ::cfmakeraw(&tty);

    const speed_t speed = baud_to_speed(config.baud_rate);
    if(::cfsetispeed(&tty, speed) < 0 || ::cfsetospeed(&tty, speed) < 0) {
        throw_system_error("cfsetispeed/cfsetospeed");
    }

    tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    switch(config.data_bits) {
        case 5: tty.c_cflag |= CS5; break;
        case 6: tty.c_cflag |= CS6; break;
        case 7: tty.c_cflag |= CS7; break;
        case 8: tty.c_cflag |= CS8; break;
        default: break;
    }

    tty.c_cflag &= static_cast<tcflag_t>(~(PARENB | PARODD));
    tty.c_iflag &= static_cast<tcflag_t>(~INPCK);
    if(config.parity == Parity::Even) {
        tty.c_cflag |= PARENB;
        tty.c_iflag |= INPCK;
    }
    else if(config.parity == Parity::Odd) {
        tty.c_cflag |= static_cast<tcflag_t>(PARENB | PARODD);
        tty.c_iflag |= INPCK;
    }

    if(config.stop_bits == StopBits::Two) tty.c_cflag |= CSTOPB;
    else tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);

    tty.c_iflag &= static_cast<tcflag_t>(~(IXON | IXOFF | IXANY));
#ifdef CRTSCTS
    tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
#endif
    if(config.flow_control == FlowControl::Software) {
        tty.c_iflag |= static_cast<tcflag_t>(IXON | IXOFF);
    }
    else if(config.flow_control == FlowControl::Hardware) {
#ifdef CRTSCTS
        tty.c_cflag |= CRTSCTS;
#else
        throw std::invalid_argument("SerialPort hardware flow control is unsupported on this platform");
#endif
    }

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    if(::tcsetattr(fd, TCSANOW, &tty) < 0) throw_system_error("tcsetattr");
}

/**
 * @brief 清空指定串口文件描述符缓冲区
 */
void SerialPort::flush_fd(int fd, FlushDirection direction) {
    int queue = TCIOFLUSH;
    switch(direction) {
        case FlushDirection::Input: queue = TCIFLUSH; break;
        case FlushDirection::Output: queue = TCOFLUSH; break;
        case FlushDirection::Both: queue = TCIOFLUSH; break;
    }

    int ret;
    do {
        ret = ::tcflush(fd, queue);
    }
    while(ret < 0 && errno == EINTR);
    if(ret < 0) throw_system_error("tcflush");
}

/**
 * @brief 等待串口读写事件就绪
 */
bool SerialPort::wait_ready(short events, std::chrono::milliseconds timeout) const {
    ensure_open();
    validate_timeout(timeout, "I/O timeout");
    const auto deadline = std::chrono::steady_clock::now() + timeout;

    for(;;) {
        pollfd pfd{};
        pfd.fd = fd_;
        pfd.events = events;
        const int ret = ::poll(&pfd, 1, to_poll_timeout(remaining_time(deadline)));
        if(ret > 0) {
            if((pfd.revents & POLLNVAL) != 0) {
                throw std::system_error(EBADF, std::generic_category(), "poll(" + port_ + ")");
            }
            if((pfd.revents & POLLERR) != 0) throw std::runtime_error("SerialPort poll error on " + port_);
            if((pfd.revents & events) != 0) return true;
            if((pfd.revents & POLLHUP) != 0) throw std::runtime_error("SerialPort disconnected: " + port_);
            continue;
        }
        if(ret == 0) return false;
        if(errno == EINTR) {
            if(std::chrono::steady_clock::now() >= deadline) return false;
            continue;
        }
        throw_system_error("poll(" + port_ + ")");
    }
}

/**
 * @brief 计算剩余超时时间
 */
std::chrono::milliseconds SerialPort::remaining_time(const std::chrono::steady_clock::time_point& deadline) {
    const auto now = std::chrono::steady_clock::now();
    if(now >= deadline) return std::chrono::milliseconds{ 0 };
    const auto remaining = deadline - now;
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(remaining);
    if(ms.count() == 0 && remaining > std::chrono::steady_clock::duration::zero()) {
        ms = std::chrono::milliseconds{ 1 };
    }
    return ms;
}

/**
 * @brief 转换 poll 超时时间
 */
int SerialPort::to_poll_timeout(std::chrono::milliseconds timeout) {
    if(timeout.count() < 0) return 0;
    if(timeout.count() > std::numeric_limits<int>::max()) return std::numeric_limits<int>::max();
    return static_cast<int>(timeout.count());
}

/**
 * @brief 确认串口已经打开
 */
void SerialPort::ensure_open() const {
    if(fd_ < 0) throw std::logic_error("SerialPort is not open");
}

/**
 * @brief 按 errno 抛出 system_error
 */
void SerialPort::throw_system_error(const std::string& operation) {
    const int error = errno;
    throw std::system_error(error, std::generic_category(), operation);
}

} // namespace serial_arm::transport
