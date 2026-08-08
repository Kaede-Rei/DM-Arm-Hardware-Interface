#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <string>
#include <vector>

#include <termios.h>

namespace serial_arm::transport {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief Linux/POSIX 串口传输层封装
 */
class SerialPort {
public:
    using Byte = std::uint8_t;          ///< 串口字节类型
    using Buffer = std::vector<Byte>;   ///< 串口字节缓冲区

    /**
     * @brief 奇偶校验方式
     */
    enum class Parity {
        None,   ///< 不使用奇偶校验
        Even,   ///< 偶校验
        Odd,    ///< 奇校验
    };

    /**
     * @brief 停止位数量
     */
    enum class StopBits {
        One,    ///< 一个停止位
        Two,    ///< 两个停止位
    };

    /**
     * @brief 流控制方式
     */
    enum class FlowControl {
        None,       ///< 不使用流控制
        Software,   ///< 软件流控制
        Hardware,   ///< 硬件流控制
    };

    /**
     * @brief 串口缓冲区清空方向
     */
    enum class FlushDirection {
        Input,   ///< 清空输入缓冲区
        Output,  ///< 清空输出缓冲区
        Both,    ///< 同时清空输入和输出缓冲区
    };

    /**
     * @brief 串口运行配置
     */
    struct Config {
        std::uint32_t baud_rate{ 115200 };                         ///< 波特率
        std::uint8_t data_bits{ 8 };                               ///< 数据位数量
        Parity parity{ Parity::None };                             ///< 奇偶校验方式
        StopBits stop_bits{ StopBits::One };                       ///< 停止位数量
        FlowControl flow_control{ FlowControl::None };             ///< 流控制方式
        std::chrono::milliseconds read_timeout{ 2 };               ///< 读取超时时间
        std::chrono::milliseconds write_timeout{ 100 };            ///< 写入超时时间
        bool flush_on_open{ true };                                ///< 打开后是否清空缓冲区
    };

public:
    /**
     * @brief 创建未打开的串口对象
     */
    SerialPort() = default;

    /**
     * @brief 使用默认配置打开串口
     * @param port 串口设备路径
     */
    explicit SerialPort(std::string port);

    /**
     * @brief 使用指定配置打开串口
     * @param port 串口设备路径
     * @param config 串口配置
     */
    SerialPort(std::string port, const Config& config);

    /**
     * @brief 关闭串口并释放文件描述符
     */
    ~SerialPort() noexcept;

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    /**
     * @brief 移动构造，接管已有串口文件描述符
     * @param other 被移动的串口对象
     */
    SerialPort(SerialPort&& other) noexcept;

    /**
     * @brief 移动赋值，关闭当前串口并接管对方文件描述符
     * @param other 被移动的串口对象
     * @return 当前对象引用
     */
    SerialPort& operator=(SerialPort&& other) noexcept;

    /**
     * @brief 使用当前配置打开串口
     * @param port 串口设备路径
     */
    void open(std::string port);

    /**
     * @brief 使用指定配置打开串口
     * @param port 串口设备路径
     * @param config 串口配置
     */
    void open(std::string port, const Config& config);

    /**
     * @brief 关闭串口
     */
    void close() noexcept;

    /**
     * @brief 查询串口是否已经打开
     * @return 已打开时返回 true
     */
    bool is_open() const noexcept;

    /**
     * @brief 获取底层文件描述符
     * @return 文件描述符，未打开时返回负值
     */
    int native_handle() const noexcept;

    /**
     * @brief 获取串口设备路径
     * @return 串口设备路径
     */
    const std::string& port() const noexcept;

    /**
     * @brief 获取当前串口配置
     * @return 串口配置
     */
    const Config& config() const noexcept;

    /**
     * @brief 更新串口配置
     * @param config 新串口配置
     */
    void set_config(const Config& config);

    /**
     * @brief 设置读取超时时间
     * @param timeout 读取超时时间
     */
    void set_read_timeout(std::chrono::milliseconds timeout);

    /**
     * @brief 设置写入超时时间
     * @param timeout 写入超时时间
     */
    void set_write_timeout(std::chrono::milliseconds timeout);

    /**
     * @brief 最多读取 len 字节
     * @param data 目标缓冲区
     * @param len 目标缓冲区长度
     * @return 实际读取字节数，超时时返回 0
     */
    std::size_t read(Byte* data, std::size_t len);

    /**
     * @brief 最多读取 max_bytes 字节并返回新缓冲区
     * @param max_bytes 最大读取字节数
     * @return 实际读取到的字节
     */
    Buffer read(std::size_t max_bytes);

    /**
     * @brief 最多读取 max_bytes 字节到指定缓冲区
     * @param buffer 目标缓冲区，函数会调整其大小
     * @param max_bytes 最大读取字节数
     * @return 实际读取字节数
     */
    std::size_t read(Buffer& buffer, std::size_t max_bytes);

    /**
     * @brief 在读取超时时间内尽量读取 len 字节
     * @param data 目标缓冲区
     * @param len 期望读取字节数
     * @return 实际读取字节数，可能小于 len
     */
    std::size_t read_exact(Byte* data, std::size_t len);

    /**
     * @brief 在读取超时时间内尽量读取 len 字节并返回新缓冲区
     * @param len 期望读取字节数
     * @return 实际读取到的字节
     */
    Buffer read_exact(std::size_t len);

    /**
     * @brief 在读取超时时间内尽量读取 len 字节到指定缓冲区
     * @param buffer 目标缓冲区，函数会调整其大小
     * @param len 期望读取字节数
     * @return 实际读取字节数
     */
    std::size_t read_exact(Buffer& buffer, std::size_t len);

    /**
     * @brief 在写入超时时间内尽量写出 len 字节
     * @param data 待写缓冲区
     * @param len 待写字节数
     * @return 实际写出字节数，可能小于 len
     */
    std::size_t write(const Byte* data, std::size_t len);

    /**
     * @brief 写出缓冲区全部内容
     * @param data 待写缓冲区
     * @return 实际写出字节数
     */
    std::size_t write(const Buffer& data);

    /**
     * @brief 写出初始化列表内容
     * @param data 待写字节列表
     * @return 实际写出字节数
     */
    std::size_t write(std::initializer_list<Byte> data);

    /**
     * @brief 等待输出队列发送完成
     */
    void drain();

    /**
     * @brief 清空串口系统缓冲区
     * @param direction 清空方向
     */
    void flush(FlushDirection direction = FlushDirection::Both);

    /**
     * @brief 查询当前可读字节数
     * @return 内核输入缓冲区中的可读字节数
     */
    std::size_t available() const;

private:
    /**
     * @brief 校验非空缓冲区参数
     */
    static void validate_buffer(const void* data, std::size_t len);

    /**
     * @brief 校验超时时间非负
     */
    static void validate_timeout(std::chrono::milliseconds timeout, const char* name);

    /**
     * @brief 校验串口配置合法性
     */
    static void validate_config(const Config& config);

    /**
     * @brief 将整数波特率转换为 termios 常量
     */
    static speed_t baud_to_speed(std::uint32_t baud_rate);

    /**
     * @brief 将串口配置应用到底层文件描述符
     */
    static void configure_fd(int fd, const Config& config);

    /**
     * @brief 清空指定文件描述符的串口缓冲区
     */
    static void flush_fd(int fd, FlushDirection direction);

    /**
     * @brief 等待串口文件描述符可读或可写
     */
    bool wait_ready(short events, std::chrono::milliseconds timeout) const;

    /**
     * @brief 计算距离 deadline 的剩余毫秒数
     */
    static std::chrono::milliseconds remaining_time(const std::chrono::steady_clock::time_point& deadline);

    /**
     * @brief 转换为 poll 使用的毫秒超时值
     */
    static int to_poll_timeout(std::chrono::milliseconds timeout);

    /**
     * @brief 确认串口已经打开
     */
    void ensure_open() const;

    /**
     * @brief 按当前 errno 抛出 system_error
     */
    [[noreturn]] static void throw_system_error(const std::string& operation);

private:
    int fd_{ -1 };             ///< 串口文件描述符
    std::string port_;         ///< 当前串口设备路径
    Config config_{};          ///< 当前串口配置
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //

} // namespace serial_arm::transport
