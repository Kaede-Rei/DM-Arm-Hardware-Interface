#include "serial_arm_protocol_damiao_usb2can/bus.hpp"

#include <algorithm>
#include <cstring>
#include <memory>

namespace serial_arm::protocol::damiao_usb2can {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //

namespace {

/**
 * @brief 计算距离 deadline 的剩余毫秒数
 */
std::chrono::milliseconds remaining_time(const std::chrono::steady_clock::time_point& deadline) {
    const auto now = std::chrono::steady_clock::now();
    if(now >= deadline) return std::chrono::milliseconds{ 0 };
    const auto remaining = deadline - now;
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(remaining);
    if(ms.count() == 0 && remaining > std::chrono::steady_clock::duration::zero()) {
        ms = std::chrono::milliseconds{ 1 };
    }
    return ms;
}

#pragma pack(push, 1)
/**
 * @brief 达妙官方 USB2CAN 模块接收报文
 */
struct RxPacket {
    std::uint8_t frame_header;
    std::uint8_t cmd;
    std::uint8_t can_data_len : 6;
    std::uint8_t can_ide : 1;
    std::uint8_t can_rtr : 1;
    std::uint32_t can_id;
    std::uint8_t can_data[8];
    std::uint8_t frame_end;
};

/**
 * @brief 达妙官方 USB2CAN 模块发送报文
 */
struct TxPacket {
    std::uint8_t frame_header[2] = { 0x55, 0xAA };
    std::uint8_t frame_len = 0x1e;
    std::uint8_t cmd = 0x03;
    std::uint32_t send_times = 1;
    std::uint32_t time_interval = 10;
    std::uint8_t id_type = 0;
    std::uint32_t can_id = 0x01;
    std::uint8_t frame_type = 0;
    std::uint8_t len = 0x08;
    std::uint8_t id_acc = 0;
    std::uint8_t data_acc = 0;
    std::uint8_t data[8] = { 0 };
    std::uint8_t crc = 0;
};
#pragma pack(pop)

} // namespace

// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

/**
 * @brief 创建达妙 USB2CAN 总线对象
 */
DamiaoUsbCanBus::DamiaoUsbCanBus(Config config)
    : config_(std::move(config)) {
}

/**
 * @brief 打开底层串口
 */
tl::expected<void, transport::CanErr> DamiaoUsbCanBus::open() {
    try {
        transport::SerialPort::Config config;
        config.baud_rate = static_cast<std::uint32_t>(config_.baudrate);
        config.read_timeout = std::chrono::milliseconds(2);
        config.write_timeout = std::chrono::milliseconds(100);
        serial_.open(config_.serial_port, config);
        return {};
    }
    catch(...) {
        return tl::make_unexpected(transport::CanErr::OPEN_FAILED);
    }
}

/**
 * @brief 关闭底层串口并清空缓存
 */
void DamiaoUsbCanBus::close() noexcept {
    serial_.close();
    rx_queue_.clear();
}

/**
 * @brief 查询底层串口是否已打开
 */
bool DamiaoUsbCanBus::is_open() const noexcept {
    return serial_.is_open();
}

/**
 * @brief 编码并发送 CAN 帧
 */
tl::expected<void, transport::CanErr> DamiaoUsbCanBus::send(const transport::CanFrame& frame) {
    if(!serial_.is_open()) return tl::make_unexpected(transport::CanErr::NOT_OPEN);
    if(frame.size > 8 || frame.id > 0x7FF) return tl::make_unexpected(transport::CanErr::INVALID_FRAME);

    TxPacket wire;
    wire.can_id = frame.id;
    wire.len = frame.size;
    std::copy(frame.data.begin(), frame.data.end(), wire.data);
    try {
        const auto sent = serial_.write(reinterpret_cast<const std::uint8_t*>(&wire), sizeof(wire));
        if(sent != sizeof(wire)) return tl::make_unexpected(transport::CanErr::TIMEOUT);
        return {};
    }
    catch(...) {
        return tl::make_unexpected(transport::CanErr::WRITE_FAILED);
    }
}

/**
 * @brief 在总 timeout 内接收并解析完整 CAN 帧
 */
tl::expected<transport::CanFrame, transport::CanErr> DamiaoUsbCanBus::receive(std::chrono::milliseconds timeout) {
    if(!serial_.is_open()) return tl::make_unexpected(transport::CanErr::NOT_OPEN);

    transport::CanFrame frame;
    try {
        if(try_parse_frame(frame)) return frame;

        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while(std::chrono::steady_clock::now() < deadline) {
            if(receive_wire_frame(frame, remaining_time(deadline))) return frame;
        }
        return tl::make_unexpected(transport::CanErr::TIMEOUT);
    }
    catch(...) {
        return tl::make_unexpected(transport::CanErr::READ_FAILED);
    }
}

/**
 * @brief 清空协议缓存和串口缓冲区
 */
void DamiaoUsbCanBus::flush() noexcept {
    rx_queue_.clear();
    if(!serial_.is_open()) return;
    try {
        serial_.flush();
    }
    catch(...) {
    }
}

/**
 * @brief 比较共享总线配置
 */
bool DamiaoUsbCanBus::config_matches(const Config& config) const noexcept {
    return config_.serial_port == config.serial_port &&
        config_.baudrate == config.baudrate;
}

/**
 * @brief 获取或创建达妙 USB2CAN 逻辑通道
 */
tl::expected<std::shared_ptr<transport::CanChannel>, Err> acquire_channel(
    const std::string& name,
    const Config& config,
    std::vector<transport::CanFilter> filters) {
    Err create_error = Err::OPEN_FAILED;
    bool created = false;
    auto bus = transport::BusPool::get_or_create(name, [&]() -> std::shared_ptr<transport::CanBus> {
        auto candidate = std::make_shared<DamiaoUsbCanBus>(config);
        const auto opened = candidate->open();
        if(!opened) {
            create_error = Err::OPEN_FAILED;
            return nullptr;
        }
        created = true;
        return candidate;
    });

    if(!bus) return tl::make_unexpected(create_error);
    auto damiao_bus = std::dynamic_pointer_cast<DamiaoUsbCanBus>(bus);
    if(!damiao_bus) return tl::make_unexpected(Err::TYPE_MISMATCH);
    if(!created && !damiao_bus->config_matches(config)) {
        return tl::make_unexpected(Err::CONFIG_CONFLICT);
    }
    return bus->create_channel(std::move(filters));
}

// ! ========================= 私 有 类 方 法 实 现 ========================= ! //

/**
 * @brief 从串口补充字节并尝试解析完整帧
 */
bool DamiaoUsbCanBus::receive_wire_frame(transport::CanFrame& frame, std::chrono::milliseconds timeout) {
    if(try_parse_frame(frame)) return true;

    serial_.set_read_timeout(timeout);
    const auto n = serial_.read(rx_buf_.data(), rx_buf_.size());
    for(std::size_t i = 0; i < n; ++i) rx_queue_.push_back(rx_buf_[i]);
    return try_parse_frame(frame);
}

/**
 * @brief 从缓存中解析达妙 USB2CAN 接收报文
 */
bool DamiaoUsbCanBus::try_parse_frame(transport::CanFrame& frame) {
    while(rx_queue_.size() >= sizeof(RxPacket)) {
        if(rx_queue_.front() != 0xAA) {
            rx_queue_.pop_front();
            continue;
        }

        std::array<std::uint8_t, sizeof(RxPacket)> raw{};
        for(std::size_t i = 0; i < sizeof(RxPacket); ++i) {
            raw[i] = rx_queue_.front();
            rx_queue_.pop_front();
        }
        RxPacket wire{};
        std::memcpy(&wire, raw.data(), raw.size());
        if(wire.cmd != 0x11 || wire.frame_end != 0x55 || wire.can_data_len > 8 ||
            wire.can_ide != 0 || wire.can_rtr != 0 || wire.can_id > 0x7FF) {
            for(std::size_t i = raw.size() - 1; i > 0; --i) rx_queue_.push_front(raw[i]);
            continue;
        }

        frame.id = wire.can_id;
        frame.size = wire.can_data_len;
        std::copy(std::begin(wire.can_data), std::end(wire.can_data), frame.data.begin());
        return true;
    }
    return false;
}

} // namespace serial_arm::protocol::damiao_usb2can
