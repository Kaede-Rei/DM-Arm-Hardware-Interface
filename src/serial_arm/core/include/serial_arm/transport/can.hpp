#pragma once

#include <array>
#include <cstdint>

namespace serial_arm::transport {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief CAN 数据帧
 */
struct CanFrame {
    std::uint32_t id{ 0 };                  ///< CAN ID
    std::array<std::uint8_t, 8> data{};     ///< CAN 数据
    std::uint8_t size{ 0 };                 ///< CAN 数据长度
};

/**
 * @brief CAN ID 过滤规则
 */
struct CanFilter {
    std::uint32_t id{ 0 };                  ///< 目标 CAN ID
    std::uint32_t mask{ 0x7FF };            ///< CAN ID 掩码
};

/**
 * @brief CAN 传输错误类型
 */
enum class CanErr {
    NOT_OPEN,       ///< 总线未打开
    OPEN_FAILED,    ///< 打开失败
    READ_FAILED,    ///< 读取失败
    WRITE_FAILED,   ///< 写入失败
    TIMEOUT,        ///< 操作超时
    INVALID_FRAME,  ///< 非法 CAN 帧
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //



// ! ========================= 模 版 方 法 实 现 ========================= ! //

} // namespace serial_arm::transport
