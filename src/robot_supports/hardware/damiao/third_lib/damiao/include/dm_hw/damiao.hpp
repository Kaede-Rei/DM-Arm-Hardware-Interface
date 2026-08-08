#pragma once

#include "serial_arm/transport/bus.hpp"

#include <memory>
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <iostream> // IWYU pragma: keep
#include <unistd.h>

#define POS_MODE 0x100
#define SPEED_MODE 0x200
#define POSI_MODE 0x300

#define POS_CSP_MODE 0x400
#define SPEED_CSP_MODE 0x500
#define TOR_CSP_MODE 0x600

namespace damiao {
#pragma pack(1)
using MotorId = uint32_t;

constexpr uint8_t MAX_RETRIES = 20;
constexpr useconds_t RETRY_INTERVAL_US = 50000;
constexpr uint8_t PARAM_READ_CMD = 51;
constexpr uint8_t PARAM_WRITE_CMD = 85;
constexpr uint8_t PARAM_SAVE_CMD = 170;

/**
 * @brief Motor Type 电机类型
 */
enum DmMotorType {
    DM4310,
    DM4310_48V,
    DM4340,
    DM4340_48V,
    DM6006,
    DM6248P,
    DM8006,
    DM8009,
    DM10010L,
    DM10010,
    DMH3510,
    DMH6215,
    DMG6220,
    DMJH11,
    Num_Of_Motor
};

/**
 * @brief 电机控制模式
 * @note 这是改控制模式对应的编码
 */
enum DmControlMode {
    MIT_MODE = 1,
    POS_VEL_MODE = 2,
    VEL_MODE = 3,
    POS_FORCE_MODE = 4,

    POS_VEL_CSP_MODE = 5,
    VEL_CSP_MODE = 6,
    TORQUE_CSP_MODE = 7,
};

/**
 * @brief 寄存器列表 具体参考达妙手册
 */
enum DmReg {
    UV_Value = 0,
    KT_Value = 1,
    OT_Value = 2,
    OC_Value = 3,
    ACC = 4,
    DEC = 5,
    MAX_SPD = 6,
    MST_ID = 7,
    ESC_ID = 8,
    TIMEOUT = 9,
    CTRL_MODE = 10,
    Damp = 11,
    Inertia = 12,
    hw_ver = 13,
    sw_ver = 14,
    SN = 15,
    NPP = 16,
    Rs = 17,
    LS = 18,
    Flux = 19,
    Gr = 20,
    PMAX = 21,
    VMAX = 22,
    TMAX = 23,
    I_BW = 24,
    KP_ASR = 25,
    KI_ASR = 26,
    KP_APR = 27,
    KI_APR = 28,
    OV_Value = 29,
    GREF = 30,
    Deta = 31,
    V_BW = 32,
    IQ_c1 = 33,
    VL_c1 = 34,
    can_br = 35,
    sub_ver = 36,
    u_off = 50,
    v_off = 51,
    k1 = 52,
    k2 = 53,
    m_off = 54,
    dir = 55,
    p_m = 80,
    xout = 81,
};

#pragma pack()

typedef struct {
    float q_max;
    float dq_max;
    float tau_max;
} LimitParam;

// 电机 PMAX/DQMAX/TAUMAX 参数
static LimitParam limit_param[Num_Of_Motor] =
{
    {12.5, 30, 10 },    // DM4310
    {12.5, 50, 10 },    // DM4310_48V
    {12.5, 8, 28 },     // DM4340
    {12.5, 10, 28 },    // DM4340_48V
    {12.5, 45, 20 },    // DM6006
    {12.566, 20, 120 }, // DM6248P
    {12.5, 45, 40 },    // DM8006
    {12.5, 45, 54 },    // DM8009
    {12.5,25,  200},    // DM10010L
    {12.5,20, 200},     // DM10010
    {12.5,28,1},        // DMH3510
    {12.5,45,10},       // DMH6215
    {12.5,45,10},      // DMG6220
    {12.5,10,12}        // DMJH11
};

class Motor {
private:
    MotorId master_id;
    MotorId slave_id;
    float state_q = 0;
    float state_dq = 0;
    float state_tau = 0;
    uint64_t state_seq = 0;
    LimitParam limit_param{};
    DmMotorType motor_type;

    union ValueUnion {
        float float_value;
        uint32_t uint32_value;
    };

    struct ValueType {
        ValueUnion value;
        bool is_float;
    };

    std::unordered_map<uint32_t, ValueType> param_map;

public:
    /**
     * @brief Construct a new Motor object
     *
     * @param motor_type 电机类型
     * @param slave_id can_id 从机ID即电机ID
     * @param master_id 主机ID建议主机ID不要都设为0x00
     *
     */
    Motor(DmMotorType motor_type, MotorId slave_id, MotorId master_id)
        : master_id(master_id), slave_id(slave_id), motor_type(motor_type) {
        this->limit_param = damiao::limit_param[motor_type];
    }

    Motor() : master_id(0x01), slave_id(0x11), motor_type(DM4310) {
        this->limit_param = damiao::limit_param[DM4310];
    }

    void receive_data(float q, float dq, float tau) {
        this->state_q = q;
        this->state_dq = dq;
        this->state_tau = tau;
        ++this->state_seq;
    }

    DmMotorType get_motor_type() const { return this->motor_type; }

    /**
     * @brief get master id 获取主机ID
     * @return MasterID
     */
    MotorId get_master_id() const { return this->master_id; }

    /**
     * @brief get motor slave id(can id)  获取电机CAN ID
     * @return SlaveID
     */
    MotorId get_slave_id() const { return this->slave_id; }

    /**
     * @brief get motor position 获取电机位置
     * @return motor position 电机位置
     */
    float get_position() const { return this->state_q; }

    /**
     * @brief get motor velocity 获取电机速度
     * @return motor velocity 电机速度
     */
    float get_velocity() const { return this->state_dq; }

    /**
     * @brief get torque of the motor  获取电机实际输出扭矩
     * @return motor torque 电机实际输出扭矩
     */
    float get_tau() const { return this->state_tau; }

    uint64_t get_state_seq() const { return this->state_seq; }

    /**
     * @brief get limit param 获取电机限制参数
     * @return limit_param 电机限制参数
     */
    LimitParam get_limit_param() { return limit_param; }

    void set_param(int key, float value) {
        ValueType v{};
        v.value.float_value = value;
        v.is_float = true;
        param_map[key] = v;
    }

    void set_param(int key, uint32_t value) {
        ValueType v{};
        v.value.uint32_value = value;
        v.is_float = false;
        param_map[key] = v;
    }

    float get_param_as_float(int key) const {
        auto it = param_map.find(key);
        if(it != param_map.end()) {
            if(it->second.is_float) {
                return it->second.value.float_value;
            }
            else {
                return 0;
            }
        }
        return 0;
    }

    uint32_t get_param_as_uint32(int key) const {
        auto it = param_map.find(key);
        if(it != param_map.end()) {
            if(!it->second.is_float) {
                return it->second.value.uint32_value;
            }
            else {
                return 0;
            }
        }
        return 0;
    }

    bool has_param(int key) const {
        return param_map.find(key) != param_map.end();
    }

    bool is_have_param(int key) const {
        return has_param(key);
    }

    void clear_param(int key) {
        param_map.erase(key);
    }

    void clear_all_params() {
        param_map.clear();
    }
};


/**
 * @brief motor control class 电机控制类
 * 使用USB转CAN进行通信，linux做虚拟串口
 */
class MotorControl {
public:

    /**
     * @brief 构造电机控制对象
     * @param channel CAN 通道
     */
    MotorControl(std::shared_ptr<serial_arm::transport::CanChannel> channel) : channel_(std::move(channel)) {
        if(channel_ == nullptr) {
            throw std::invalid_argument("CAN channel is null");
        }
    }

    ~MotorControl()
        = default;

    /**
     * @brief 使能电机
     * @param motor 电机对象
     */
    bool enable(const Motor& motor) {
        const bool sent = control_cmd(motor.get_slave_id(), 0xFC);
        usleep(100000); // 100ms
        this->receive();
        return sent;
    }

    /**
     * @brief enable motor which is old version 使能达妙旧款电机固件 使用旧版本固件建议尽快升级成新版本
     * @param motor object 电机对象
     * @param mode 控制模式  damiao::MIT_MODE, damiao::POS_VEL_MODE, damiao::VEL_MODE, damiao::POS_FORCE_MODE
     */
    void enable_old(const Motor& motor, DmControlMode mode) {
        uint32_t id = ((mode - 1) << 2) + motor.get_slave_id();
        control_cmd(id, 0xFC);
        usleep(100000);
        this->receive();
    }

    /**
     * @brief 刷新电机状态
     * @param motor 电机对象
     */
    bool refresh_motor_status(const Motor& motor) {
        uint32_t id = 0x7FF;
        uint8_t can_low = motor.get_slave_id() & 0xff; // id low 8 bit
        uint8_t can_high = (motor.get_slave_id() >> 8) & 0xff; //id high 8 bit
        std::array<uint8_t, 8> data_buf = { can_low,can_high, 0xCC, 0x00, 0x00, 0x00, 0x00, 0x00 };
        const uint64_t previous_seq = motor.get_state_seq();
        if(!send_frame(id, data_buf)) return false;

        constexpr int status_receive_attempts = 3;
        for(int attempt = 0; attempt < status_receive_attempts; ++attempt) {
            this->receive();
            if(motor.get_state_seq() != previous_seq) return true;
        }
        return false;
    }
    /**
     * @brief 失能电机
     * @param motor 电机对象
     */
    bool disable(const Motor& motor) {
        const bool sent = control_cmd(motor.get_slave_id(), 0xFD);
        usleep(100000);
        this->receive();
        return sent;
    }

    /**
     * @brief 将当前位置设为零点
     * @param motor 电机对象
     */
    void set_zero_position(const Motor& motor) {
        control_cmd(motor.get_slave_id(), 0xFE);
        usleep(100000);
        this->receive();
    }

    /**
     * @brief MIT 控制模式，具体参数定义请参考达妙手册
     * @param motor 电机对象
     * @param kp 比例系数
     * @param kd 微分系数
     * @param q 位置
     * @param dq 速度
     * @param tau 扭矩
     */
    bool control_mit(Motor& motor, float kp, float kd, float q, float dq, float tau, bool receive_feedback = true) {
        // 位置、速度和扭矩采用线性映射的关系将浮点型数据转换成有符号的定点数据
        static auto float_to_uint = [](float x, float xmin, float xmax, uint8_t bits) -> uint16_t {
            x = std::clamp(x, xmin, xmax);
            const float span = xmax - xmin;
            const float data_norm = (x - xmin) / span;
            return static_cast<uint16_t>(data_norm * ((1u << bits) - 1));
            };
        MotorId id = motor.get_slave_id();
        if(motors.find(id) == motors.end()) {
            throw std::runtime_error("MotorControl id not found");
        }
        auto& m = motors[id];
        uint16_t kp_uint = float_to_uint(kp, 0, 500, 12);
        uint16_t kd_uint = float_to_uint(kd, 0, 5, 12);
        LimitParam limit_param_cmd = m->get_limit_param();
        uint16_t q_uint = float_to_uint(q, -limit_param_cmd.q_max, limit_param_cmd.q_max, 16);
        uint16_t dq_uint = float_to_uint(dq, -limit_param_cmd.dq_max, limit_param_cmd.dq_max, 12);
        uint16_t tau_uint = float_to_uint(tau, -limit_param_cmd.tau_max, limit_param_cmd.tau_max, 12);

        std::array<uint8_t, 8> data_buf{};
        data_buf[0] = (q_uint >> 8) & 0xff;
        data_buf[1] = q_uint & 0xff;
        data_buf[2] = dq_uint >> 4;
        data_buf[3] = ((dq_uint & 0xf) << 4) | ((kp_uint >> 8) & 0xf);
        data_buf[4] = kp_uint & 0xff;
        data_buf[5] = kd_uint >> 4;
        data_buf[6] = ((kd_uint & 0xf) << 4) | ((tau_uint >> 8) & 0xf);
        data_buf[7] = tau_uint & 0xff;

        const bool sent = send_frame(id, data_buf);
        if(receive_feedback) this->receive();
        return sent;
    }

    /**
     * @brief 位置速度控制模式
     * @param motor 电机对象
     * @param pos 位置
     * @param vel 速度
     */
    void control_pos_vel(Motor& motor, float pos, float vel) {
        MotorId id = motor.get_slave_id();
        if(motors.find(id) == motors.end()) {
            throw std::runtime_error("POS_VEL ERROR : MotorControl id not found");
        }
        std::array<uint8_t, 8> data_buf{};
        memcpy(data_buf.data(), &pos, sizeof(float));
        memcpy(data_buf.data() + 4, &vel, sizeof(float));
        id += POS_MODE;
        (void)send_frame(id, data_buf);
        this->receive();
    }

    /**
     * @brief 速度控制模式
     * @param motor 电机对象
     * @param vel 速度
     */
    void control_vel(Motor& motor, float vel) {
        MotorId id = motor.get_slave_id();
        if(motors.find(id) == motors.end()) {
            throw std::runtime_error("VEL ERROR : id not found");
        }
        std::array<uint8_t, 8> data_buf = { 0 };
        memcpy(data_buf.data(), &vel, sizeof(float));
        id = id + SPEED_MODE;
        (void)send_frame(id, data_buf);
        this->receive();
    }

    /**
     * @brief 力位混合控制模式
     * @param motor 电机对象
     * @param pos 位置
     * @param vel 速度（范围 0-10000，详见手册）
     * @param i 电流（范围 0-10000，详见手册）
     */
    void control_pos_force(Motor& motor, float pos, uint16_t vel, uint16_t i) {
        MotorId id = motor.get_slave_id();
        if(motors.find(id) == motors.end()) {
            throw std::runtime_error("pos_force ERROR : MotorControl id not found");
        }
        std::array<uint8_t, 8> data_buf{};
        memcpy(data_buf.data(), &pos, sizeof(float));
        memcpy(data_buf.data() + 4, &vel, sizeof(uint16_t));
        memcpy(data_buf.data() + 6, &i, sizeof(uint16_t));
        id = id + POSI_MODE;
        (void)send_frame(id, data_buf);
        this->receive();
    }


    /**
     * @brief 周期同步位置速度控制模式
     * @param motor 电机对象
     * @param pos 位置
     * @param vel 速度
     */
    void control_pos_vel_csp(Motor& motor, float pos, float vel) {
        MotorId id = motor.get_slave_id();
        if(motors.find(id) == motors.end()) {
            throw std::runtime_error("POS_VEL_CSP ERROR : MotorControl id not found");
        }
        std::array<uint8_t, 8> data_buf{};
        memcpy(data_buf.data(), &pos, sizeof(float));
        memcpy(data_buf.data() + 4, &vel, sizeof(float));
        id += POS_CSP_MODE;
        (void)send_frame(id, data_buf);
        this->receive();
    }

    /**
     * @brief 周期同步速度控制模式
     * @param motor 电机对象
     * @param vel 速度
     */
    void control_vel_csp(Motor& motor, float vel) {
        MotorId id = motor.get_slave_id();
        if(motors.find(id) == motors.end()) {
            throw std::runtime_error("VEL ERROR : id not found");
        }
        std::array<uint8_t, 8> data_buf = { 0 };
        memcpy(data_buf.data(), &vel, sizeof(float));
        id = id + SPEED_CSP_MODE;
        (void)send_frame(id, data_buf);
        this->receive();
    }

    /**
     * @brief 周期同步力矩控制模式
     * @param motor 电机对象
     * @param tor 力矩
     */
    void control_tor_csp(Motor& motor, float tor) {
        MotorId id = motor.get_slave_id();
        if(motors.find(id) == motors.end()) {
            throw std::runtime_error("VEL ERROR : id not found");
        }
        std::array<uint8_t, 8> data_buf = { 0 };
        memcpy(data_buf.data(), &tor, sizeof(float));
        id = id + TOR_CSP_MODE;
        (void)send_frame(id, data_buf);
        this->receive();
    }

    /**
     * @brief 接收并解析电机 CAN 反馈数据
     */
    bool receive() {
        auto maybe_frame = channel_->receive(std::chrono::milliseconds(2));
        if(!maybe_frame) return false;
        const auto receive_data = *maybe_frame;

        static auto uint_to_float = [](uint16_t x, float xmin, float xmax, uint8_t bits) -> float {
            float span = xmax - xmin;
            float data_norm = float(x) / ((1 << bits) - 1);
            float data = data_norm * span + xmin;
            return data;
            };

        auto& data = receive_data.data;

        uint16_t q_uint = (uint16_t(data[1]) << 8) | data[2];
        uint16_t dq_uint = (uint16_t(data[3]) << 4) | (data[4] >> 4);
        uint16_t tau_uint = (uint16_t(data[4] & 0xf) << 8) | data[5];
        if(receive_data.id != 0x00) {
            if(motors.find(receive_data.id) == motors.end()) {
                return false;
            }

            auto m = motors[receive_data.id];
            LimitParam limit_param_receive = m->get_limit_param();
            float receive_q = uint_to_float(q_uint, -limit_param_receive.q_max, limit_param_receive.q_max, 16);
            float receive_dq = uint_to_float(dq_uint, -limit_param_receive.dq_max, limit_param_receive.dq_max, 12);
            float receive_tau = uint_to_float(tau_uint, -limit_param_receive.tau_max, limit_param_receive.tau_max, 12);
            m->receive_data(receive_q, receive_dq, receive_tau);
        }
        else {
            uint32_t slave_id = data[0] & 0x0f;
            if(motors.find(slave_id) == motors.end()) {
                return false;
            }
            auto m = motors[slave_id];
            LimitParam limit_param_receive = m->get_limit_param();
            float receive_q = uint_to_float(q_uint, -limit_param_receive.q_max, limit_param_receive.q_max, 16);
            float receive_dq = uint_to_float(dq_uint, -limit_param_receive.dq_max, limit_param_receive.dq_max, 12);
            float receive_tau = uint_to_float(tau_uint, -limit_param_receive.tau_max, limit_param_receive.tau_max, 12);
            m->receive_data(receive_q, receive_dq, receive_tau);
        }
        return true;
    }

    void receive_param() {
        auto maybe_frame = channel_->receive(std::chrono::milliseconds(2));
        if(!maybe_frame) return;
        const auto receive_data = *maybe_frame;

        auto& data = receive_data.data;
        if(data[2] == PARAM_READ_CMD or data[2] == PARAM_WRITE_CMD) {
            uint32_t slave_id = (uint32_t(data[1]) << 8) | data[0];
            uint8_t reg_id = data[3];
            if(motors.find(slave_id) == motors.end()) {
                return;
            }
            if(is_in_ranges(reg_id)) {
                uint32_t data_uint32 = (uint32_t(data[7]) << 24) | (uint32_t(data[6]) << 16) | (uint32_t(data[5]) << 8) | data[4];
                motors[slave_id]->set_param(reg_id, data_uint32);
            }
            else {
                float data_float = uint8_to_float(data.data() + 4);
                motors[slave_id]->set_param(reg_id, data_float);
            }
            return;
        }
    }

    /**
     * @brief 添加电机到控制器
     * @param motor 电机对象指针
     */
    void add_motor(Motor* motor) {
        motors.insert({ motor->get_slave_id(), motor });
        if(motor->get_master_id() != 0) {
            motors.insert({ motor->get_master_id(), motor });
        }
    }

    /**
     * @brief 读取电机寄存器参数
     * @param motor 电机对象
     * @param reg_id 寄存器 ID，例如 damiao::UV_Value
     * @return 查询到的参数值；未查询到时返回 0
     */
    float read_motor_param(Motor& motor, uint8_t reg_id) {
        motor.clear_param(reg_id);
        uint32_t id = motor.get_slave_id();
        uint8_t can_low = id & 0xff;
        uint8_t can_high = (id >> 8) & 0xff;
        std::array<uint8_t, 8> data_buf{ can_low, can_high, PARAM_READ_CMD, reg_id, 0x00, 0x00, 0x00, 0x00 };
        (void)send_frame(0x7FF, data_buf);
        for(uint8_t i = 0; i < MAX_RETRIES; i++) {
            usleep(RETRY_INTERVAL_US);
            receive_param();
            if(motors[motor.get_slave_id()]->has_param(reg_id)) {
                if(is_in_ranges(reg_id)) {
                    return float(motors[motor.get_slave_id()]->get_param_as_uint32(reg_id));
                }
                else {
                    return motors[motor.get_slave_id()]->get_param_as_float(reg_id);
                }
            }
        }

        return 0;
    }


    /**
     * @brief 切换电机控制模式
     * @param motor 电机对象
     * @param mode 控制模式，如 damiao::MIT_MODE
     */
    bool switch_control_mode(Motor& motor, DmControlMode mode) {
        constexpr uint8_t reg_id = CTRL_MODE;
        motor.clear_param(reg_id);
        uint8_t write_data[4] = { (uint8_t)mode, 0x00, 0x00, 0x00 };
        write_motor_param(motor, reg_id, write_data);
        if(motors.find(motor.get_slave_id()) == motors.end()) {
            return false;
        }
        for(uint8_t i = 0; i < MAX_RETRIES; i++) {
            usleep(RETRY_INTERVAL_US);
            receive_param();
            if(motors[motor.get_slave_id()]->has_param(reg_id)) {
                return motors[motor.get_slave_id()]->get_param_as_uint32(reg_id) == mode;
            }
        }
        return false;
    }

    /**
     * @brief 修改电机寄存器参数
     * @param motor 电机对象
     * @param reg_id 寄存器 ID
     * @param data 参数值
     * @return 修改成功返回 true，否则返回 false
     */
    bool change_motor_param(Motor& motor, uint8_t reg_id, float data) {
        motor.clear_param(reg_id);
        if(is_in_ranges(reg_id)) {
            //居然传进来的是整型的范围 救一下
            uint32_t data_uint32 = float_to_uint32(data);
            uint8_t* data_uint8;
            data_uint8 = (uint8_t*)&data_uint32;
            write_motor_param(motor, reg_id, data_uint8);
        }
        else {
            //is float
            uint8_t* data_uint8;
            data_uint8 = (uint8_t*)&data;
            write_motor_param(motor, reg_id, data_uint8);
        }
        if(motors.find(motor.get_slave_id()) == motors.end()) {
            return false;
        }
        for(uint8_t i = 0; i < MAX_RETRIES; i++) {
            usleep(RETRY_INTERVAL_US);
            receive_param();
            if(motors[motor.get_slave_id()]->has_param(reg_id)) {
                if(is_in_ranges(reg_id)) {
                    return motors[motor.get_slave_id()]->get_param_as_uint32(reg_id) == float_to_uint32(data);
                }
                else {
                    return fabsf(motors[motor.get_slave_id()]->get_param_as_float(reg_id) - data) < 0.1f;
                }
            }
        }
        return false;
    }


    /**
     * @brief 将电机参数保存到 Flash
     * @param motor 电机对象
     */
    void save_motor_param(Motor& motor) {
        disable(motor);
        uint32_t id = motor.get_slave_id();
        uint8_t id_low = id & 0xff;
        uint8_t id_high = (id >> 8) & 0xff;
        std::array<uint8_t, 8> data_buf{ id_low, id_high, PARAM_SAVE_CMD, 0x01, 0x00, 0x00, 0x00, 0x00 };
        (void)send_frame(0x7FF, data_buf);
        usleep(100000); // 100ms wait for save
    }

    /**
     * @brief 修改电机限制参数（非寄存器参数）
     * @param motor 电机对象
     * @param p_max 位置上限
     * @param q_max 速度上限
     * @param t_max 扭矩上限
     */
    static void change_motor_limit(Motor& motor, float p_max, float q_max, float t_max) {
        limit_param[motor.get_motor_type()] = { p_max, q_max, t_max };
    }

private:
    bool control_cmd(MotorId id, uint8_t cmd) {
        std::array<uint8_t, 8> data_buf = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, cmd };
        return send_frame(id, data_buf);
    }

    void write_motor_param(Motor& motor, uint8_t reg_id, const uint8_t data[4]) {
        uint32_t id = motor.get_slave_id();
        uint8_t can_low = id & 0xff;
        uint8_t can_high = (id >> 8) & 0xff;
        std::array<uint8_t, 8> data_buf{ can_low, can_high, PARAM_WRITE_CMD, reg_id, 0x00, 0x00, 0x00, 0x00 };
        data_buf[4] = data[0];
        data_buf[5] = data[1];
        data_buf[6] = data[2];
        data_buf[7] = data[3];
        (void)send_frame(0x7FF, data_buf);
    }

    static bool is_in_ranges(int number) {
        return (7 <= number && number <= 10) ||
            (13 <= number && number <= 16) ||
            (35 <= number && number <= 36);
    }

    static uint32_t float_to_uint32(float value) {
        return static_cast<uint32_t>(value);
    }

    static float uint32_to_float(uint32_t value) {
        return static_cast<float>(value);
    }

    static float uint8_to_float(const uint8_t data[4]) {
        uint32_t combined = (static_cast<uint32_t>(data[3]) << 24) |
            (static_cast<uint32_t>(data[2]) << 16) |
            (static_cast<uint32_t>(data[1]) << 8) |
            static_cast<uint32_t>(data[0]);
        float result;
        memcpy(&result, &combined, sizeof(result));
        return result;
    }

    bool send_frame(MotorId id, const std::array<uint8_t, 8>& data) {
        serial_arm::transport::CanFrame frame;
        frame.id = id;
        frame.size = 8;
        frame.data = data;
        return channel_->send(frame).has_value();
    }

    std::unordered_map<MotorId, Motor*> motors;
    std::shared_ptr<serial_arm::transport::CanChannel> channel_;
};

};
