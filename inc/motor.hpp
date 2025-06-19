#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <stdint.h>

// Motor 类定义，用于控制和监控电机
class Motor {
public:
    // 使用紧凑的内存布局，确保结构体成员之间没有padding
    #pragma pack(1)
    
    /**
     * @brief 通信模式和电机ID
     * 使用联合体来允许按位访问或作为整体访问
     */
    typedef union {
        struct {
            uint8_t id : 4;      // 电机ID，4位
            uint8_t status : 3;  // 电机状态，3位
            uint8_t reserve : 1; // 保留位，1位
        };
        uint8_t mode;            // 整个字节的访问
    } __attribute__((packed)) MotorMode_t;

    /**
     * @brief 电机控制数据
     * 包含发送给电机的控制指令
     */
    typedef struct {
        int16_t tor_des;  // 目标转矩
        int16_t spd_des;  // 目标速度
        int32_t pos_des;  // 目标位置
        int16_t k_pos;    // 位置增益
        int16_t k_spd;    // 速度增益
    } __attribute__((packed)) MotorCmd_t;

    /**
     * @brief 电机反馈数据
     * 包含从电机接收到的反馈信息
     */
    typedef struct {
        int16_t torque;        // 当前转矩
        int16_t speed;         // 当前速度
        int32_t pos;           // 当前位置
        int8_t temp;           // 当前温度
        uint8_t  MError :3;    // 电机错误标识: 0.正常 1.过热 2.过流 3.过压 4.编码器故障 5-7.保留
        uint16_t force  :12;   // 足端气压传感器数据 12bit (0-4095)
        uint8_t  none   :1;    // 保留位
    } __attribute__((packed)) MotorData_t;

    /**
     * @brief 电机控制数据包
     * 用于发送给电机的完整数据包结构
     */
    typedef struct {
        uint8_t head[2];    // 数据包头
        MotorMode_t mode;   // 电机模式
        MotorCmd_t comd;    // 控制命令
        uint16_t CRC16;     // CRC校验码
    } __attribute__((packed)) ControlData_t;

    /**
     * @brief 电机反馈数据包
     * 从电机接收到的完整数据包结构
     */
    typedef struct {
        uint8_t head[2];    // 数据包头
        MotorMode_t mode;   // 电机模式
        MotorData_t fbk;    // 反馈数据
        uint16_t CRC16;     // CRC校验码
    } __attribute__((packed)) RecvData_t;

    #pragma pack()  // 恢复默认的内存对齐

    // 构造函数
    Motor();

    // 设置电机控制参数
    void setControlParams(float tor_des, float spd_des, float pos_des, float k_pos, float k_spd);
    
    // 更新电机反馈数据
    void updateFeedback(const RecvData_t& recv_data);
    
    // 创建控制数据包
    ControlData_t createControlPacket(uint8_t motor_id) const;

    // 获取当前转矩
    float getTorque() const { return tor; }
    
    // 获取当前速度
    float getSpeed() const { return spd; }
    
    // 获取当前位置
    float getPosition() const { return pos; }
    
    // 获取当前温度
    float getTemperature() const { return temp; }
    
    // 获取当前错误代码
    uint16_t getError() const { return err; }

    // 获取发送计数
    uint64_t getSendCount() const { return send_count; }
    
    // 获取接收计数
    uint64_t getReceiveCount() const { return receive_count; }

    // 增加发送计数
    void incrementSendCount() { ++send_count; }
    
    // 增加接收计数
    void incrementReceiveCount() { ++receive_count; }
    
    // 重置统计信息
    void resetStats();
    // 设置电机控制参数（重载函数，使用int16_t类型的ID和num）id为腿编号，num为每条腿上的电机编号
    void Motor_SetControlParams(int16_t id, int16_t num, float tor_des, float spd_des, float pos_des, float k_pos, float k_spd);

private:
    // 控制参数
    float tor_des;  // 目标转矩
    float spd_des;  // 目标速度
    float pos_des;  // 目标位置
    float k_pos;    // 位置增益
    float k_spd;    // 速度增益

    // 反馈数据
    float tor;      // 当前转矩
    float spd;      // 当前速度
    float pos;      // 当前位置
    float temp;     // 当前温度
    uint16_t err;   // 错误代码

    // 统计信息
    uint64_t send_count;     // 发送计数
    uint64_t receive_count;  // 接收计数
};

// CRC 相关函数声明
// 计算单个字节的CRC值
uint16_t crc_ccitt_byte(uint16_t crc, const uint8_t c);

// 计算一串数据的CRC值
uint16_t crc_ccitt(uint16_t crc, const uint8_t *buffer, uint16_t len);

#endif // MOTOR_H