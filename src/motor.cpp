#include "motor.hpp"
#include <cstring>
#include <iostream>
#include "common.hpp"
// CRC（循环冗余校验）查找表
// 这个表用于快速计算CRC值，提高计算效率
static const uint16_t crc_ccitt_table[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};


// 计算单个字节的CRC值
// 参数:
//   crc: 当前的CRC值
//   c: 要计算的字节
// 返回: 更新后的CRC值
uint16_t crc_ccitt_byte(uint16_t crc, const uint8_t c) {
    return (crc >> 8) ^ crc_ccitt_table[(crc ^ c) & 0xff];
}

// 计算一串数据的CRC值
// 参数:
//   crc: 初始CRC值
//   buffer: 要计算的数据缓冲区
//   len: 数据长度
// 返回: 计算得到的CRC值
uint16_t crc_ccitt(uint16_t crc, const uint8_t *buffer, uint16_t len) {
    while (len--)
        crc = (crc >> 8) ^ crc_ccitt_table[(crc ^ *buffer++) & 0xff];
    return crc;
}

// 发送帧头定义
// 用于标识数据包的开始
static const uint8_t SEND_FRAME_HEADER[] = {0xFE, 0xEE};

// Motor类的构造函数
// 初始化所有成员变量为默认值
Motor::Motor() : tor_des(0), spd_des(0), pos_des(0), k_pos(0), k_spd(0),
                 tor(0), spd(0), pos(0), temp(0), err(0),
                 send_count(0), receive_count(0) {}

// 设置电机控制参数
// 参数:
//   tor_des: 目标转矩
//   spd_des: 目标速度
//   pos_des: 目标位置
//   k_pos: 位置增益
//   k_spd: 速度增益
// void Motor::setControlParams(float tor_des, float spd_des, float pos_des, float k_pos, float k_spd) {
//     // this->tor_des = tor_des;
//     // this->spd_des = spd_des;
//     // this->pos_des = pos_des;
//     // this->k_pos = k_pos;
//     // this->k_spd = k_spd;
// }

void Motor::Motor_SetControlParams(int16_t id, int16_t num, float tor_des, float spd_des, float pos_des, float k_pos, float k_spd)
{
    if(id == 0)
    {
        if(num == 0) {
            this->tor_des =  tor_des / GEAR_RATIO;     // 转子端转矩 = 输出端转矩 / 减速比
            this->spd_des =  spd_des * GEAR_RATIO;     // 转子端速度 = 输出端速度 * 减速比
            this->pos_des =  (pos_des + 0.917742) * GEAR_RATIO;     // 转子端位置 = 输出端位置 * 减速比
            this->k_pos =  k_pos / GEAR_RATIO / GEAR_RATIO;  // 位置增益需要考虑两次减速比
            this->k_spd =  k_spd / GEAR_RATIO / GEAR_RATIO;   // 速度增益需要考虑两次减速比
        }
        else if(num == 1)
        {
            this->tor_des =  -tor_des / GEAR_RATIO;     // 转子端转矩 = 输出端转矩 / 减速比
            this->spd_des =  -spd_des * GEAR_RATIO;     // 转子端速度 = 输出端速度 * 减速比
            this->pos_des =  -(pos_des - 1.775659) * GEAR_RATIO;     // 转子端位置 = 输出端位置 * 减速比
            this->k_pos =  k_pos / GEAR_RATIO / GEAR_RATIO;  // 位置增益需要考虑两次减速比
            this->k_spd =  k_spd / GEAR_RATIO / GEAR_RATIO;   // 速度增益需要考虑两次减速比

        }
        else if(num == 2)
        {
            this->tor_des =  tor_des / GEAR_RATIO / 1.88;     // 转子端转矩 = 输出端转矩 / 减速比
            this->spd_des =  spd_des * GEAR_RATIO * 1.88;     // 转子端速度 = 输出端速度 * 减速比
            this->pos_des =  (pos_des + 3.205968) * GEAR_RATIO * 1.88 ;     // 转子端位置 = 输出端位置 * 减速比
            this->k_pos =  k_pos / GEAR_RATIO / GEAR_RATIO;  // 位置增益需要考虑两次减速比
            this->k_spd =  k_spd / GEAR_RATIO / GEAR_RATIO;   // 速度增益需要考虑两次减速比
        }
        else
        {
            // 如果num不在预期范围内，打印错误信息
            std::cerr << "Error: Invalid motor number " << num << ". Valid numbers are 0, 1, or 2." << std::endl;
            return;
        }
    }
    else if (id == 1)
    {
        if(num == 0) {
            this->tor_des =  tor_des / GEAR_RATIO;     // 转子端转矩 = 输出端转矩 / 减速比
            this->spd_des =  spd_des * GEAR_RATIO;     // 转子端速度 = 输出端速度 * 减速比
            this->pos_des =  (pos_des + 0.83411) * GEAR_RATIO;     // 转子端位置 = 输出端位置 * 减速比
            this->k_pos =  k_pos / GEAR_RATIO / GEAR_RATIO;  // 位置增益需要考虑两次减速比
            this->k_spd =  k_spd / GEAR_RATIO / GEAR_RATIO;   // 速度增益需要考虑两次减速比
        }
        else if(num == 1)
        {
            this->tor_des =  tor_des / GEAR_RATIO;     // 转子端转矩 = 输出端转矩 / 减速比
            this->spd_des =  spd_des * GEAR_RATIO;     // 转子端速度 = 输出端速度 * 减速比
            this->pos_des =  (pos_des - 0.950479) * GEAR_RATIO;     // 转子端位置 = 输出端位置 * 减速比
            this->k_pos =  k_pos / GEAR_RATIO / GEAR_RATIO;  // 位置增益需要考虑两次减速比
            this->k_spd =  k_spd / GEAR_RATIO / GEAR_RATIO;   // 速度增益需要考虑两次减速比

        }
        else if(num == 2)
        {
            this->tor_des =  -tor_des / GEAR_RATIO / 1.88;     // 转子端转矩 = 输出端转矩 / 减速比
            this->spd_des =  -spd_des * GEAR_RATIO * 1.88;     // 转子端速度 = 输出端速度 * 减速比
            this->pos_des =  -(pos_des + 2.6572986) * GEAR_RATIO * 1.88;     // 转子端位置 = 输出端位置 * 减速比
            this->k_pos =  k_pos / GEAR_RATIO / GEAR_RATIO;  // 位置增益需要考虑两次减速比
            this->k_spd =  k_spd / GEAR_RATIO / GEAR_RATIO;   // 速度增益需要考虑两次减速比
        }
        else
        {
            // 如果num不在预期范围内，打印错误信息
            std::cerr << "Error: Invalid motor number " << num << ". Valid numbers are 0, 1, or 2." << std::endl;
            return;
        }
    }
    else if (id == 2)
    {
        if(num == 0) {
            this->tor_des =  -tor_des / GEAR_RATIO;     // 转子端转矩 = 输出端转矩 / 减速比
            this->spd_des =  -spd_des * GEAR_RATIO;     // 转子端速度 = 输出端速度 * 减速比
            this->pos_des =  -(pos_des + 0.036858) * GEAR_RATIO;     // 转子端位置 = 输出端位置 * 减速比
            this->k_pos =  k_pos / GEAR_RATIO / GEAR_RATIO;  // 位置增益需要考虑两次减速比
            this->k_spd =  k_spd / GEAR_RATIO / GEAR_RATIO;   // 速度增益需要考虑两次减速比
        }
        else if(num == 1)
        {
            this->tor_des =  -tor_des / GEAR_RATIO;     // 转子端转矩 = 输出端转矩 / 减速比
            this->spd_des =  -spd_des * GEAR_RATIO;     // 转子端速度 = 输出端速度 * 减速比
            this->pos_des =  -(pos_des - 1.4168) * GEAR_RATIO;     // 转子端位置 = 输出端位置 * 减速比
            this->k_pos =  k_pos / GEAR_RATIO / GEAR_RATIO;  // 位置增益需要考虑两次减速比
            this->k_spd =  k_spd / GEAR_RATIO / GEAR_RATIO;   // 速度增益需要考虑两次减速比

        }
        else if(num == 2)
        {
            this->tor_des =  tor_des / GEAR_RATIO / 1.88;     // 转子端转矩 = 输出端转矩 / 减速比
            this->spd_des =  spd_des * GEAR_RATIO * 1.88;     // 转子端速度 = 输出端速度 * 减速比
            this->pos_des =  (pos_des + 3.2397) * GEAR_RATIO * 1.88;     // 转子端位置 = 输出端位置 * 减速比
            this->k_pos =  k_pos / GEAR_RATIO / GEAR_RATIO;  // 位置增益需要考虑两次减速比
            this->k_spd =  k_spd / GEAR_RATIO / GEAR_RATIO;   // 速度增益需要考虑两次减速比
        }
        else
        {
            // 如果num不在预期范围内，打印错误信息
            std::cerr << "Error: Invalid motor number " << num << ". Valid numbers are 0, 1, or 2." << std::endl;
            return;
        }
    }
    else if (id == 3)
    {
        if(num == 0) {
            this->tor_des =  -tor_des / GEAR_RATIO;     // 转子端转矩 = 输出端转矩 / 减速比
            this->spd_des =  -spd_des * GEAR_RATIO;     // 转子端速度 = 输出端速度 * 减速比
            this->pos_des =  -(pos_des - 0.414653) * GEAR_RATIO;     // 转子端位置 = 输出端位置 * 减速比
            this->k_pos =  k_pos / GEAR_RATIO / GEAR_RATIO;  // 位置增益需要考虑两次减速比
            this->k_spd =  k_spd / GEAR_RATIO / GEAR_RATIO;   // 速度增益需要考虑两次减速比
        }
        else if(num == 1)
        {
            this->tor_des =  tor_des / GEAR_RATIO;     // 转子端转矩 = 输出端转矩 / 减速比
            this->spd_des =  spd_des * GEAR_RATIO;     // 转子端速度 = 输出端速度 * 减速比
            this->pos_des =  (pos_des - 0.42181) * GEAR_RATIO;     // 转子端位置 = 输出端位置 * 减速比
            this->k_pos =  k_pos / GEAR_RATIO / GEAR_RATIO;  // 位置增益需要考虑两次减速比
            this->k_spd =  k_spd / GEAR_RATIO / GEAR_RATIO;   // 速度增益需要考虑两次减速比

        }
        else if(num == 2)
        {
            this->tor_des =  -tor_des / GEAR_RATIO / 1.88;     // 转子端转矩 = 输出端转矩 / 减速比
            this->spd_des =  -spd_des * GEAR_RATIO * 1.88;     // 转子端速度 = 输出端速度 * 减速比
            this->pos_des =  -(pos_des + 2.231182) * GEAR_RATIO * 1.88;     // 转子端位置 = 输出端位置 * 减速比
            this->k_pos =  k_pos / GEAR_RATIO / GEAR_RATIO;  // 位置增益需要考虑两次减速比
            this->k_spd =  k_spd / GEAR_RATIO / GEAR_RATIO;   // 速度增益需要考虑两次减速比
        }
        else
        {
            // 如果num不在预期范围内，打印错误信息
            std::cerr << "Error: Invalid motor number " << num << ". Valid numbers are 0, 1, or 2." << std::endl;
            return;
        }
    }
    else
    {
        // 如果id不在预期范围内，打印错误信息
        std::cerr << "Error: Invalid motor ID " << id << ". Valid IDs are 0, 1, 2, or 3." << std::endl;
        return;
    }
}


// 更新电机反馈数据
// 参数:
//   recv_data: 接收到的数据包
void Motor::updateFeedback(const RecvData_t& recv_data) {
    // 将接收到的数据转换为实际物理量
    tor = ((float)recv_data.fbk.torque) / 256.0f;  // 转矩
    spd = ((float)recv_data.fbk.speed / 256.0f) * 6.28318f;  // 速度 (rad/s)
    pos = 6.28318f * ((float)recv_data.fbk.pos) / 32768.0f;  // 位置 (rad)
    temp = (float)recv_data.fbk.temp;  // 温度
    err = recv_data.fbk.MError;  // 错误代码
}

float Motor::getTorque(int16_t id,int16_t num) const
{
    if(id == 0)
    {
        if(num == 0)
            return (tor * GEAR_RATIO); // 转子端转矩 = 输出端转矩 / 减速比
        else if(num == 1)
            return -(tor * GEAR_RATIO); // 转子端转矩 = 输出端转矩 / 减速比
        else if(num == 2)
            return (tor * GEAR_RATIO * 1.88); // 转子端转矩 = 输出端转矩 / 减速比
        else
            return 0.0f; // 如果num不在预期范围内，返回0
    }
    else if (id == 1)
    {
        if(num == 0)
            return (tor * GEAR_RATIO); // 转子端转矩 = 输出端转矩 / 减速比
        else if(num == 1)
            return (tor * GEAR_RATIO); // 转子端转矩 = 输出端转矩 / 减速比
        else if(num == 2)
            return -(tor * GEAR_RATIO * 1.88); // 转子端转矩 = 输出端转矩 / 减速比
        else
            return 0.0f; // 如果num不在预期范围内，返回0
    }
    else if (id == 2)
    {
        if(num == 0)
            return -(tor * GEAR_RATIO); // 转子端转矩 = 输出端转矩 / 减速比
        else if(num == 1)
            return -(tor * GEAR_RATIO); // 转子端转矩 = 输出端转矩 / 减速比
        else if(num == 2)
            return (tor * GEAR_RATIO * 1.88); // 转子端转矩 = 输出端转矩 / 减速比
        else
            return 0.0f; // 如果num不在预期范围内，返回0
    }
    else if (id == 3)
    {
        if(num == 0)
            return -(tor * GEAR_RATIO); // 转子端转矩 = 输出端转矩 / 减速比
        else if(num == 1)
            return (tor * GEAR_RATIO); // 转子端转矩 = 输出端转矩 / 减速比
        else if(num == 2)
            return -(tor * GEAR_RATIO * 1.88); // 转子端转矩 = 输出端转矩 / 减速比
        else
            return 0.0f; // 如果num不在预期范围内，返回0
    }
    else
    {
        // 如果id不在预期范围内，打印错误信息并返回0
        std::cerr << "Error: Invalid motor ID " << id << ". Valid IDs are 0, 1, 2, or 3." << std::endl;
        return 0.0f;
    }
}

float Motor::getSpeed(int16_t id,int16_t num) const
{
    if(id == 0)
    {
        if(num == 0)
            return (spd / GEAR_RATIO); // 转子端速度 = 输出端速度 * 减速比
        else if(num == 1)
            return -(spd / GEAR_RATIO); // 转子端速度 = 输出端速度 * 减速比
        else if(num == 2)
            return (spd / GEAR_RATIO / 1.88); // 转子端速度 = 输出端速度 * 减速比
        else
            return 0.0f; // 如果num不在预期范围内，返回0
    }
    else if (id == 1)
    {
        if(num == 0)
            return (spd / GEAR_RATIO); // 转子端速度 = 输出端速度 * 减速比
        else if(num == 1)
            return (spd / GEAR_RATIO); // 转子端速度 = 输出端速度 * 减速比
        else if(num == 2)
            return -(spd / GEAR_RATIO / 1.88); // 转子端速度 = 输出端速度 * 减速比
        else
            return 0.0f; // 如果num不在预期范围内，返回0
    }
    else if (id == 2)
    {
        if(num == 0)
            return -(spd / GEAR_RATIO); // 转子端速度 = 输出端速度 * 减速比
        else if(num == 1)
            return -(spd / GEAR_RATIO); // 转子端速度 = 输出端速度 * 减速比
        else if(num == 2)
            return (spd / GEAR_RATIO / 1.88); // 转子端速度 = 输出端速度 * 减速比
        else
            return 0.0f; // 如果num不在预期范围内，返回0
    }
    else if (id == 3)
    {
        if(num == 0)
            return -(spd / GEAR_RATIO); // 转子端速度 = 输出端速度 * 减速比
        else if(num == 1)
            return (spd / GEAR_RATIO); // 转子端速度 = 输出端速度 * 减速比
        else if(num == 2)
            return -(spd / GEAR_RATIO / 1.88); // 转子端速度 = 输出端速度 * 减速比
        else
            return 0.0f; // 如果num不在预期范围内，返回0
    }
    else
    {
        // 如果id不在预期范围内，打印错误信息并返回0
        std::cerr << "Error: Invalid motor ID " << id << ". Valid IDs are 0, 1, 2, or 3." << std::endl;
        return 0.0f;
    }
}

float Motor::getPosition(int16_t id,int16_t num) const
{
    if(id == 0)
    {
        if(num == 0)
            return (pos / GEAR_RATIO - 0.917742); // 转子端位置 = 输出端位置 * 减速比
        else if(num == 1)
            return (-(pos / GEAR_RATIO)  + 1.775659); // 转子端位置 = 输出端位置 * 减速比
        else if(num == 2)
            return (pos / GEAR_RATIO / 1.88 - 3.205968); // 转子端位置 = 输出端位置 * 减速比
        else
            return 0.0f; // 如果num不在预期范围内，返回0
    }
    else if (id == 1)
    {
        if(num == 0)
            return (pos / GEAR_RATIO  - 0.83411); // 转子端位置 = 输出端位置 * 减速比
        else if(num == 1)
            return (pos / GEAR_RATIO + 0.950479); // 转子端位置 = 输出端位置 * 减速比
        else if(num == 2)
            return (-(pos / GEAR_RATIO / 1.88) - 2.6572986); // 转子端位置 = 输出端位置 * 减速比
        else
            return 0.0f; // 如果num不在预期范围内，返回0
    }
    else if (id == 2)
    {
        if(num == 0)
            return (-(pos / GEAR_RATIO) - 0.036858); // 转子端位置 = 输出端位置 * 减速比
        else if(num == 1)
            return (-(pos / GEAR_RATIO) + 1.4168); // 转子端位置 = 输出端位置 * 减速比
        else if(num == 2)
            return (pos / GEAR_RATIO / 1.88 - 3.2397); // 转子端位置 = 输出端位置 * 减速比
        else
            return 0.0f; // 如果num不在预期范围内，返回0
    }
    else if (id == 3)
    {
        if(num == 0)
            return (-(pos / GEAR_RATIO) + 0.414653); // 转子端位置 = 输出端位置 * 减速比
        else if(num == 1)
            return (pos / GEAR_RATIO + 0.42181); // 转子端位置 = 输出端位置 * 减速比
        else if(num == 2)
            return (-(pos / GEAR_RATIO / 1.88) - 2.231182); // 转子端位置 = 输出端位置 * 减速比
        else
            return 0.0f; // 如果num不在预期范围内，返回0
    }
    else
    {
        // 如果id不在预期范围内，打印错误信息并返回0
        std::cerr << "Error: Invalid motor ID " << id << ". Valid IDs are 0, 1, 2, or 3." << std::endl;
        return 0.0f;
    }
}

// 创建控制数据包
// 参数:
//   motor_id: 电机ID
// 返回: 构建好的控制数据包
Motor::ControlData_t Motor::createControlPacket(uint8_t motor_id) const {
    ControlData_t packet;
    
    // 设置帧头
    memcpy(packet.head, SEND_FRAME_HEADER, sizeof(SEND_FRAME_HEADER));
    
    // 设置电机ID和状态
    packet.mode.id = motor_id;
    packet.mode.status = 1;  // 1表示正常工作状态
    packet.mode.reserve = 0;  // 保留位，设为0
    
    // 将控制参数转换为实际发送的数据格式
    packet.comd.tor_des = (int16_t)(tor_des * 256.0f);  // 转矩
    packet.comd.spd_des = (int16_t)(spd_des / 6.28318f * 256.0f);  // 速度
    packet.comd.pos_des = (int32_t)(pos_des / 6.28318f * 32768.0f);  // 位置
    packet.comd.k_pos = (int16_t)(k_pos / 25.6f * 32768.0f);  // 位置增益
    packet.comd.k_spd = (int16_t)(k_spd / 25.6f * 32768.0f);  // 速度增益

    // 计算并设置CRC校验值
    packet.CRC16 = crc_ccitt(0x2cbb, (uint8_t*)&packet, sizeof(ControlData_t) - 2);

    return packet;
}

// 重置统计信息
// 将发送和接收计数器归零
void Motor::resetStats() {
    send_count = 0;
    receive_count = 0;
}