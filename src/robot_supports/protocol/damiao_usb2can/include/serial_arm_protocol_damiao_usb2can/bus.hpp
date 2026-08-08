#pragma once

#include "serial_arm/transport/bus.hpp"
#include "serial_arm/transport/serial_port.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

namespace serial_arm::protocol::damiao_usb2can {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief 达妙官方 USB2CAN 模块运行配置
 */
struct Config {
    std::string serial_port{ "/dev/ttyACM0" };      ///< 串口设备路径
    int baudrate{ 921600 };                         ///< 串口波特率
};

/**
 * @brief 达妙官方 USB2CAN 模块错误类型
 */
enum class Err {
    OPEN_FAILED,        ///< 打开失败
    CONFIG_CONFLICT,   ///< 同名共享总线配置冲突
    TYPE_MISMATCH,     ///< 同名共享总线不是达妙官方 USB2CAN 模块
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief 达妙官方 USB2CAN 模块 CAN 总线实现
 *
 * 将 SerialArm 通用 CanFrame 与达妙官方 USB2CAN 模块的
 * 私有串口报文进行双向转换
 */
class DamiaoUsbCanBus final : public transport::CanBus {
public:
    /**
     * @brief 创建达妙官方 USB2CAN 总线对象
     * @param config USB2CAN 串口配置
     */
    explicit DamiaoUsbCanBus(Config config);

    /**
     * @brief 关闭串口并释放资源
     */
    ~DamiaoUsbCanBus() override { close(); }

    /**
     * @brief 打开底层 USB2CAN 串口
     * @return 成功时返回空结果，否则返回通用 CAN 错误
     */
    tl::expected<void, transport::CanErr> open() override;

    /**
     * @brief 关闭底层 USB2CAN 串口并清空接收缓存
     */
    void close() noexcept override;

    /**
     * @brief 查询底层 USB2CAN 串口是否已打开
     * @return 已打开时返回 true
     */
    bool is_open() const noexcept override;

    /**
     * @brief 将通用 CAN 帧编码为达妙 USB2CAN 串口报文并发送
     * @param frame 通用 CAN 帧
     * @return 成功时返回空结果，否则返回通用 CAN 错误
     */
    tl::expected<void, transport::CanErr> send(const transport::CanFrame& frame) override;

    /**
     * @brief 从达妙 USB2CAN 串口报文解析一个通用 CAN 帧
     * @param timeout 物理串口读取超时时间
     * @return 成功时返回 CAN 帧，否则返回通用 CAN 错误
     */
    tl::expected<transport::CanFrame, transport::CanErr> receive(std::chrono::milliseconds timeout) override;

    /**
     * @brief 清空协议缓存和串口缓冲区
     */
    void flush() noexcept override;

    /**
     * @brief 检查配置是否与当前总线一致
     * @param config 待比较配置
     * @return 完全一致时返回 true
     */
    bool config_matches(const Config& config) const noexcept;

private:
    /**
     * @brief 尝试从缓存中解析完整 USB2CAN 接收报文
     * @param frame 解析成功时写入的 CAN 帧
     * @return 成功解析时返回 true
     */
    bool try_parse_frame(transport::CanFrame& frame);

    /**
     * @brief 从串口补充字节并尝试解析一个 CAN 帧
     * @param frame 解析成功时写入的 CAN 帧
     * @return 成功解析时返回 true
     */
    bool receive_wire_frame(transport::CanFrame& frame, std::chrono::milliseconds timeout);

private:
    Config config_;                                     ///< 达妙官方 USB2CAN 模块配置
    transport::SerialPort serial_;                      ///< 底层串口
    std::deque<std::uint8_t> rx_queue_;                 ///< 串口接收拼帧缓存
    std::array<std::uint8_t, 1024> rx_buf_{};            ///< 串口临时读取缓冲区
};

/**
 * @brief 获取达妙官方 USB2CAN 模块 CAN 通道
 * @param name 共享总线名称
 * @param config 达妙官方 USB2CAN 模块配置
 * @param filters CAN 通道过滤规则
 * @return 如果成功，则返回 CAN 通道，否则返回错误类型
 */
tl::expected<std::shared_ptr<transport::CanChannel>, Err> acquire_channel(
    const std::string& name,
    const Config& config,
    std::vector<transport::CanFilter> filters);

// ! ========================= 模 版 方 法 实 现 ========================= ! //

} // namespace serial_arm::protocol::damiao_usb2can
