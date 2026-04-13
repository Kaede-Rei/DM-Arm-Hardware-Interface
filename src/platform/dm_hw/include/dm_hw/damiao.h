#ifndef DAMIAO_H
#define DAMIAO_H

#include "SerialPort.h"
#include <cmath>
#include <utility>
#include <unordered_map>
#include <array>
#include <cstdint>
#include <cmath>

#define POS_MODE 0x100
#define SPEED_MODE 0x200
#define POSI_MODE 0x300

#define POS_CSP_MODE 0x400
#define SPEED_CSP_MODE 0x500
#define TOR_CSP_MODE 0x600

#define max_retries 20
#define retry_interval 50000
namespace damiao {
#pragma pack(1)
#define MotorId uint32_t

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

typedef struct {
    uint8_t frame_header;
    uint8_t cmd;// 命令 0x00: 心跳
    //     0x01: receive fail 0x11: receive success
    //     0x02: send fail 0x12: send success
    //     0x03: set baudrate fail 0x13: set baudrate success
    //     0xEE: communication error 此时格式段为错误码
    //     8: 超压 9: 欠压 A: 过流 B: MOS过温 C: 电机线圈过温 D: 通讯丢失 E: 过载
    uint8_t canDataLen : 6; // 数据长度
    uint8_t can_ide : 1; // 0: 标准帧 1: 扩展帧
    uint8_t canRtr : 1; // 0: 数据帧 1: 远程帧
    uint32_t can_id; // 电机反馈的ID
    uint8_t canData[8];
    uint8_t frameEnd; // 帧尾
} CanReceiveFrame;

typedef struct can_send_frame {
    uint8_t frame_header[2] = { 0x55, 0xAA }; // 帧头
    uint8_t FrameLen = 0x1e; // 帧长
    uint8_t cmd = 0x03; // 命令 1：转发CAN数据帧 2：PC与设备握手，设备反馈OK 3: 非反馈CAN转发，不反馈发送状态
    uint32_t send_times = 1; // 发送次数
    uint32_t time_interval = 10; // 时间间隔
    uint8_t id_type = 0; // ID类型 0：标准帧 1：扩展帧
    uint32_t can_id = 0x01; // CAN ID 使用电机ID作为CAN ID
    uint8_t frame_type = 0; // 帧类型 0： 数据帧 1：远程帧
    uint8_t len = 0x08; // len
    uint8_t id_acc = 0;
    uint8_t data_acc = 0;
    uint8_t data[8] = { 0 };
    uint8_t crc = 0; // 未解析，任意值

    void modify(const MotorId id, const uint8_t* send_data) {
        can_id = id;
        std::copy(send_data, send_data + 8, data);
    }

} can_send_frame;

#pragma pack()

typedef struct {
    float q_max;
    float dq_max;
    float tau_max;
} LimitParam;

//电机PMAX DQMAX TAUMAX参数
static LimitParam limit_param[Num_Of_Motor] =
{
    {12.5, 30, 10 }, // DM4310
    {12.5, 50, 10 }, // DM4310_48V
    {12.5, 8, 28 },  // DM4340
    {12.5, 10, 28 }, // DM4340_48V
    {12.5, 45, 20 }, // DM6006
    {12.566, 20, 120 }, // DM6248P
    {12.5, 45, 40 }, // DM8006
    {12.5, 45, 54 }, // DM8009
    {12.5,25,  200}, // DM10010L
    {12.5,20, 200},  // DM10010
    {12.5,280,1},    // DMH3510
    {12.5,45,10},    // DMH6215
    {12.5,45,10} ,    // DMG6220
    {12.5,10,12}     // DMJH11
};

class Motor {
private:
    /* data */
    MotorId master_id;
    MotorId slave_id;
    float state_q = 0;
    float state_dq = 0;
    float state_tau = 0;
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
    }

    DmMotorType get_motor_type() const { return this->motor_type; }

    /*
     * @brief get master id 获取主机ID
     * @return MasterID
     */
    MotorId get_master_id() const { return this->master_id; }

    /*
     * @brief get motor slave id(can id)  获取电机CAN ID
     * @return SlaveID
     */
    MotorId get_slave_id() const { return this->slave_id; }

    /*
     * @brief get motor position 获取电机位置
     * @return motor position 电机位置
     */
    float get_position() const { return this->state_q; }

    /*
     * @brief get motor velocity 获取电机速度
     * @return motor velocity 电机速度
     */
    float get_velocity() const { return this->state_dq; }

    /*
     * @brief get torque of the motor  获取电机实际输出扭矩
     * @return motor torque 电机实际输出扭矩
     */
    float get_tau() const { return this->state_tau; }

    /*
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

    bool is_have_param(int key) const {
        return param_map.find(key) != param_map.end();
    }

};


/**
 * @brief motor control class 电机控制类
 * 使用USB转CAN进行通信，linux做虚拟串口
 */
class Motor_Control {
public:

    /*
    * @brief 定义电机控制对象
    * @param serial 串口对象
    * 默认串口为/dev/ttyACM0
    */
    Motor_Control(SerialPort::SharedPtr serial = nullptr) : serial_(std::move(serial)) {
        if(serial_ == nullptr) {
            //default serial port
            serial_ = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
        }
    }

    ~Motor_Control()
        = default;

        /*
        * @brief enable the motor 使能电机
        * @param motor 电机对象
        */
    void enable(const Motor& motor) {
        control_cmd(motor.get_slave_id(), 0xFC);
        usleep(100000);//100ms
        this->receive();
    }

    /*
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

    /*
     * @brief refresh motor status 刷新电机状态
     * @param motor object 电机对象
     */
    void refresh_motor_status(const Motor& motor) {
        uint32_t id = 0x7FF;
        uint8_t can_low = motor.get_slave_id() & 0xff; // id low 8 bit
        uint8_t can_high = (motor.get_slave_id() >> 8) & 0xff; //id high 8 bit
        std::array<uint8_t, 8> data_buf = { can_low,can_high, 0xCC, 0x00, 0x00, 0x00, 0x00, 0x00 };
        send_data.modify(id, data_buf.data());
        serial_->send((uint8_t*)&send_data, sizeof(can_send_frame));
        this->receive();
    }
    /*
    * @brief  disable the motor 失能电机
    * @param  motor object 电机对象
    */
    void disable(const Motor& motor) {
        control_cmd(motor.get_slave_id(), 0xFD);
        usleep(100000);
        this->receive();
    }

    /*
    * @brief set the now position as zero point 保存当前位置为电机零点
    * @param motor object 电机对象
    */
    void set_zero_position(const Motor& motor) {
        control_cmd(motor.get_slave_id(), 0xFE);
        usleep(100000);
        this->receive();
    }

    /* @description: MIT Control Mode MIT控制模式 具体描述请参考达妙手册
     *@param kp: 比例系数
     *@param kd: 微分系数
     *@param q: position 位置
     *@param dq: velocity 速度
     *@param tau: torque 扭矩
    */
    void control_mit(Motor& DM_Motor, float kp, float kd, float q, float dq, float tau) {
        // 位置、速度和扭矩采用线性映射的关系将浮点型数据转换成有符号的定点数据
        static auto float_to_uint = [](float x, float xmin, float xmax, uint8_t bits) -> uint16_t {
            float span = xmax - xmin;
            float data_norm = (x - xmin) / span;
            uint16_t data_uint = data_norm * ((1 << bits) - 1);
            return data_uint;
            };
        MotorId id = DM_Motor.get_slave_id();
        if(motors.find(id) == motors.end()) {
            throw std::runtime_error("Motor_Control id not found");
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

        send_data.modify(id, data_buf.data());
        serial_->send((uint8_t*)&send_data, sizeof(can_send_frame));
        this->receive();
    }

    /*
     * @description: Position Control Mode with velocity  位置速度控制模式
     * @param pos: position 位置
     * @param vel: velocity 速度
     * @param DM_Motor: Motor object 电机对象
     */
    void control_pos_vel(Motor& DM_Motor, float pos, float vel) {
        MotorId id = DM_Motor.get_slave_id();
        if(motors.find(id) == motors.end()) {
            throw std::runtime_error("POS_VEL ERROR : Motor_Control id not found");
        }
        std::array<uint8_t, 8> data_buf{};
        memcpy(data_buf.data(), &pos, sizeof(float));
        memcpy(data_buf.data() + 4, &vel, sizeof(float));
        id += POS_MODE;
        send_data.modify(id, data_buf.data());
        serial_->send(reinterpret_cast<uint8_t*>(&send_data), sizeof(can_send_frame));
        this->receive();
    }

    /*
     * @description: velocity control mode 速度控制模式
     * @param DM_Motor: motor object 电机对象
     * @param vel: velocity 速度
     */
    void control_vel(Motor& DM_Motor, float vel) {
        MotorId id = DM_Motor.get_slave_id();
        if(motors.find(id) == motors.end()) {
            throw std::runtime_error("VEL ERROR : id not found");
        }
        std::array<uint8_t, 8> data_buf = { 0 };
        memcpy(data_buf.data(), &vel, sizeof(float));
        id = id + SPEED_MODE;
        send_data.modify(id, data_buf.data());
        serial_->send((uint8_t*)&send_data, sizeof(can_send_frame));
        this->receive();
    }

    /*
     * @description: Position control mode with torque  力位混合控制模式
     * @param DM_Motor: motor object 电机对象
     * @param pos: position 位置
     * @param vel: velocity 速度 范围0-10000 具体参考达妙手册
     * @param i: current 电流 范围0-10000具体参考达妙手册
     */
    void control_pos_force(Motor& DM_Motor, float pos, uint16_t vel, uint16_t i) {
        MotorId id = DM_Motor.get_slave_id();
        if(motors.find(id) == motors.end()) {
            throw std::runtime_error("pos_force ERROR : Motor_Control id not found");
        }
        std::array<uint8_t, 8> data_buf{};
        memcpy(data_buf.data(), &pos, sizeof(float));
        memcpy(data_buf.data() + 4, &vel, sizeof(uint16_t));
        memcpy(data_buf.data() + 6, &i, sizeof(uint16_t));
        id = id + POSI_MODE;
        send_data.modify(id, data_buf.data());
        serial_->send((uint8_t*)&send_data, sizeof(can_send_frame));
        this->receive();
    }


    /*
     * @description: Position Control Mode with velocity  周期同步位置速度控制模式
     * @param pos: position 位置
     * @param vel: velocity 速度
     * @param DM_Motor: Motor object 电机对象
     */
    void control_pos_vel_csp(Motor& DM_Motor, float pos, float vel) {
        MotorId id = DM_Motor.get_slave_id();
        if(motors.find(id) == motors.end()) {
            throw std::runtime_error("POS_VEL_CSP ERROR : Motor_Control id not found");
        }
        std::array<uint8_t, 8> data_buf{};
        memcpy(data_buf.data(), &pos, sizeof(float));
        memcpy(data_buf.data() + 4, &vel, sizeof(float));
        id += POS_CSP_MODE;
        send_data.modify(id, data_buf.data());
        serial_->send(reinterpret_cast<uint8_t*>(&send_data), sizeof(can_send_frame));
        this->receive();
    }

    /*
     * @description: velocity control mode 周期同步速度控制模式
     * @param DM_Motor: motor object 电机对象
     * @param vel: velocity 速度
     */
    void control_vel_csp(Motor& DM_Motor, float vel) {
        MotorId id = DM_Motor.get_slave_id();
        if(motors.find(id) == motors.end()) {
            throw std::runtime_error("VEL ERROR : id not found");
        }
        std::array<uint8_t, 8> data_buf = { 0 };
        memcpy(data_buf.data(), &vel, sizeof(float));
        id = id + SPEED_CSP_MODE;
        send_data.modify(id, data_buf.data());
        serial_->send((uint8_t*)&send_data, sizeof(can_send_frame));
        this->receive();
    }

    /*
     * @description: torque control mode 周期同步力矩控制模式
     * @param DM_Motor: motor object 电机对象
     * @param tor: torque 力矩
     */
    void control_tor_csp(Motor& DM_Motor, float tor) {
        MotorId id = DM_Motor.get_slave_id();
        if(motors.find(id) == motors.end()) {
            throw std::runtime_error("VEL ERROR : id not found");
        }
        std::array<uint8_t, 8> data_buf = { 0 };
        memcpy(data_buf.data(), &tor, sizeof(float));
        id = id + TOR_CSP_MODE;
        send_data.modify(id, data_buf.data());
        serial_->send((uint8_t*)&send_data, sizeof(can_send_frame));
        this->receive();
    }

    /*
     * @description 函数库内部调用,用于解算电机can线返回的电机参数
     */
    void receive() {
        serial_->recv((uint8_t*)&receive_data, 0xAA, sizeof(CanReceiveFrame));

        if(receive_data.cmd == 0x11 && receive_data.frameEnd == 0x55) // receive success
        {
            static auto uint_to_float = [](uint16_t x, float xmin, float xmax, uint8_t bits) -> float {
                float span = xmax - xmin;
                float data_norm = float(x) / ((1 << bits) - 1);
                float data = data_norm * span + xmin;
                return data;
                };

            auto& data = receive_data.canData;

            uint16_t q_uint = (uint16_t(data[1]) << 8) | data[2];
            uint16_t dq_uint = (uint16_t(data[3]) << 4) | (data[4] >> 4);
            uint16_t tau_uint = (uint16_t(data[4] & 0xf) << 8) | data[5];
            if(receive_data.can_id != 0x00)   //make sure the motor id is not 0x00
            {
                if(motors.find(receive_data.can_id) == motors.end()) {
                    return;
                }

                auto m = motors[receive_data.can_id];
                LimitParam limit_param_receive = m->get_limit_param();
                float receive_q = uint_to_float(q_uint, -limit_param_receive.q_max, limit_param_receive.q_max, 16);
                float receive_dq = uint_to_float(dq_uint, -limit_param_receive.dq_max, limit_param_receive.dq_max, 12);
                float receive_tau = uint_to_float(tau_uint, -limit_param_receive.tau_max, limit_param_receive.tau_max, 12);
                m->receive_data(receive_q, receive_dq, receive_tau);
            }
            else //why the user set the masterid as 0x00 ???
            {
                uint32_t slaveID = data[0] & 0x0f;
                if(motors.find(slaveID) == motors.end()) {
                    return;
                }
                auto m = motors[slaveID];
                LimitParam limit_param_receive = m->get_limit_param();
                float receive_q = uint_to_float(q_uint, -limit_param_receive.q_max, limit_param_receive.q_max, 16);
                float receive_dq = uint_to_float(dq_uint, -limit_param_receive.dq_max, limit_param_receive.dq_max, 12);
                float receive_tau = uint_to_float(tau_uint, -limit_param_receive.tau_max, limit_param_receive.tau_max, 12);
                m->receive_data(receive_q, receive_dq, receive_tau);
            }
            return;
        }
        else if(receive_data.cmd == 0x01) // receive fail
        {
            /* code */
        }
        else if(receive_data.cmd == 0x02) // send fail
        {
            /* code */
        }
        else if(receive_data.cmd == 0x03) // send success
        {
                /* code */
        }
        else if(receive_data.cmd == 0xEE) // communication error
        {
            /* code */
        }
    }

    void receive_param() {
        serial_->recv((uint8_t*)&receive_data, 0xAA, sizeof(CanReceiveFrame));

        if(receive_data.cmd == 0x11 && receive_data.frameEnd == 0x55) // receive success
        {
            auto& data = receive_data.canData;
            if(data[2] == 0x33 or data[2] == 0x55) {
                uint32_t slaveID = (uint32_t(data[1]) << 8) | data[0];
                uint8_t RID = data[3];
                if(motors.find(slaveID) == motors.end()) {
                    //can not found motor id
                    return;
                }
                if(is_in_ranges(RID)) {
                    uint32_t data_uint32 = (uint32_t(data[7]) << 24) | (uint32_t(data[6]) << 16) | (uint32_t(data[5]) << 8) | data[4];
                    motors[slaveID]->set_param(RID, data_uint32);
                }
                else {
                    float data_float = uint8_to_float(data + 4);
                    motors[slaveID]->set_param(RID, data_float);
                }
            }
            return;
        }
    }

    /**
     * @brief add motor to class 添加电机
     * @param DM_Motor : motor object 电机对象
     */
    void addMotor(Motor* DM_Motor) {
        motors.insert({ DM_Motor->get_slave_id(), DM_Motor });
        if(DM_Motor->get_master_id() != 0) {
            motors.insert({ DM_Motor->get_master_id(), DM_Motor });
        }
    }

    /*
     * @description: read motor register param 读取电机内部寄存器参数，具体寄存器列表请参考达妙的手册
     * @param DM_Motor: motor object 电机对象
     * @param RID: register id 寄存器ID  example: damiao::UV_Value
     * @return: motor param 电机参数 如果没查询到返回的参数为0
     */
    float read_motor_param(Motor& DM_Motor, uint8_t RID) {
        uint32_t id = DM_Motor.get_slave_id();
        uint8_t can_low = id & 0xff;
        uint8_t can_high = (id >> 8) & 0xff;
        std::array<uint8_t, 8> data_buf{ can_low, can_high, 0x33, RID, 0x00, 0x00, 0x00, 0x00 };
        send_data.modify(0x7FF, data_buf.data());
        serial_->send((uint8_t*)&send_data, sizeof(can_send_frame));
        for(uint8_t i = 0; i < max_retries; i++) {
            usleep(retry_interval);
            receive_param();
            if(motors[DM_Motor.get_slave_id()]->is_have_param(RID)) {
                if(is_in_ranges(RID)) {
                    return float(motors[DM_Motor.get_slave_id()]->get_param_as_uint32(RID));
                }
                else {
                    return motors[DM_Motor.get_slave_id()]->get_param_as_float(RID);
                }
            }
        }

        return 0;
    }


    /*
     * @description: switch control mode 切换电机控制模式
     * @param DM_Motor: motor object 电机对象
     * @param mode: control mode 控制模式 like:damiao::MIT_MODE, damiao::POS_VEL_MODE, damiao::VEL_MODE, damiao::POS_FORCE_MODE
     */
    bool switch_control_mode(Motor& DM_Motor, DmControlMode mode) {
        uint8_t write_data[4] = { (uint8_t)mode, 0x00, 0x00, 0x00 };
        uint8_t RID = 10;
        write_motor_param(DM_Motor, RID, write_data);
        if(motors.find(DM_Motor.get_slave_id()) == motors.end()) {
            return false;
        }
        for(uint8_t i = 0; i < max_retries; i++) {
            usleep(retry_interval);
            receive_param();
            if(motors[DM_Motor.get_slave_id()]->is_have_param(RID)) {
                return motors[DM_Motor.get_slave_id()]->get_param_as_uint32(RID) == mode;
            }
        }
        return false;
    }

    /*
     * @description: change motor param 修改电机内部寄存器参数 具体寄存器列表请参考达妙手册
     * @param DM_Motor: motor object 电机对象
     * @param RID: register id 寄存器ID
     * @param data: param data 参数数据,大部分数据是float类型，其中如果是uint32类型的数据也可以直接输入整型的就行，函数内部有处理
     * @return: bool true or false  是否修改成功
     */
    bool change_motor_param(Motor& DM_Motor, uint8_t RID, float data) {
        if(is_in_ranges(RID)) {
            //居然传进来的是整型的范围 救一下
            uint32_t data_uint32 = float_to_uint32(data);
            uint8_t* data_uint8;
            data_uint8 = (uint8_t*)&data_uint32;
            write_motor_param(DM_Motor, RID, data_uint8);
        }
        else {
            //is float
            uint8_t* data_uint8;
            data_uint8 = (uint8_t*)&data;
            write_motor_param(DM_Motor, RID, data_uint8);
        }
        if(motors.find(DM_Motor.get_slave_id()) == motors.end()) {
            return false;
        }
        for(uint8_t i = 0; i < max_retries; i++) {
            usleep(retry_interval);
            receive_param();
            if(motors[DM_Motor.get_slave_id()]->is_have_param(RID)) {
                if(is_in_ranges(RID)) {
                    return motors[DM_Motor.get_slave_id()]->get_param_as_uint32(RID) == float_to_uint32(data);
                }
                else {
                    return fabsf(motors[DM_Motor.get_slave_id()]->get_param_as_float(RID) - data) < 0.1f;
                }
            }
        }
        return false;
    }


    /*
     * @description: save all param to motor flash 保存电机的所有参数到flash里面
     * @param DM_Motor: motor object 电机对象
     * 电机默认参数不会写到flash里面，需要进行写操作
     */
    void save_motor_param(Motor& DM_Motor) {
        disable(DM_Motor);
        uint32_t id = DM_Motor.get_slave_id();
        uint8_t id_low = id & 0xff;
        uint8_t id_high = (id >> 8) & 0xff;
        std::array<uint8_t, 8> data_buf{ id_low, id_high, 0xAA, 0x01, 0x00, 0x00, 0x00, 0x00 };
        send_data.modify(0x7FF, data_buf.data());
        serial_->send((uint8_t*)&send_data, sizeof(can_send_frame));
        usleep(100000);//100ms wait for save
    }

    /*
     * @description: change motor limit param 修改电机限制参数，这个修改的不是电机内部的寄存器参数，而是电机的限制参数
     * @param DM_Motor: motor object 电机对象
     * @param P_MAX: position max 位置最大值
     * @param q_max: velocity max 速度最大值
     * @param T_MAX: torque max 扭矩最大值
     */
    static void change_motor_limit(Motor& DM_Motor, float P_MAX, float q_max, float T_MAX) {
        limit_param[DM_Motor.get_motor_type()] = { P_MAX,q_max,T_MAX };
    }

    /*
     * @description: change motor PMAX 修改电机的最大PMAX
     * @param DM_Motor: motor object 电机对象
     * @param P_MAX: position max 位置最大值
     */
    static void change_motor_pmax(Motor& DM_Motor, float P_MAX) {
        limit_param[DM_Motor.get_motor_type()].q_max = P_MAX;
    }

private:
    void control_cmd(MotorId id, uint8_t cmd) {
        std::array<uint8_t, 8> data_buf = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, cmd };
        send_data.modify(id, data_buf.data());
        serial_->send((uint8_t*)&send_data, sizeof(can_send_frame));
    }

    void write_motor_param(Motor& DM_Motor, uint8_t RID, const uint8_t data[4]) {
        uint32_t id = DM_Motor.get_slave_id();
        uint8_t can_low = id & 0xff;
        uint8_t can_high = (id >> 8) & 0xff;
        std::array<uint8_t, 8> data_buf{ can_low, can_high, 0x55, RID, 0x00, 0x00, 0x00, 0x00 };
        data_buf[4] = data[0];
        data_buf[5] = data[1];
        data_buf[6] = data[2];
        data_buf[7] = data[3];
        send_data.modify(0x7FF, data_buf.data());
        serial_->send((uint8_t*)&send_data, sizeof(can_send_frame));
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

    std::unordered_map<MotorId, Motor*> motors;
    SerialPort::SharedPtr serial_;  //serial port
    can_send_frame send_data; //send data frame
    CanReceiveFrame receive_data{};//receive data frame
};

};

#endif