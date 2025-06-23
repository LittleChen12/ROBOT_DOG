#ifndef    IMU_HPP
#define    IMU_HPP

#include <stdint.h>

#define LINE(arg...) arg

class IMU {
public:
    // 使用紧凑的内存布局，确保结构体成员之间没有padding
    #pragma pack(1)

    typedef struct{
        int 				BootStatus;
        int					RxStatus;
        int 				RxType;
        int 				RxDataLeft;
        int		 			RxNumber;
        int		 			TxNumber;
        int 				CRC8_Verify;
        int 				CRC16_Verify;
        uint32_t 			BufferIndex;
        uint8_t 			FDILink_Frame_Buffer[12];
        uint8_t 			Buffer[256];
    }FDILink_Status_t;


    /**
     * @brief imu数据
     * 用于存储从IMU接收到的数据
     */
    typedef struct {
        float RollSpeed;  // 横滚速度
        float aPitchSpeedcc_y;  // 俯仰速度
        float HeadingSpeed;  // 航向速度
        float Roll; // 横滚角
        float Pitch; // 俯仰角
        float Heading; // 航向角
        float Q1;   // 四元数 Q1
		float Q2;   // 四元数 Q2
		float Q3;   // 四元数 Q3
		float Q4;   // 四元数 Q4
		int64_t Timestamp; // 时间戳
    } __attribute__((packed)) IMUData_t;

    #pragma pack()  // 恢复默认的内存对齐

    //构造函数
    IMU();
    //初始化串口
    void serial_init(const char* port_name); 
    //配置串口参数，提高稳定性
    bool configure_imu_serial(int fd);
    //获取IMU数据包
    bool get_imu_packet(uint8_t imu_ID); 
    //构建IMU请求数据包，返回帧长度
    int create_imu_packet(uint8_t* buffer, FDILink_Status_t* FDILink, uint8_t type, void* buf, int len);
    //获取IMU数据
    void getData();

	IMUData_t imu_data; // 存储IMU数据的成员变量
private:
    FDILink_Status_t FDILink_Status; // 存储FDILink状态的成员变量
    int imu_fd;
};

extern IMU imu; // 声明IMU类的全局实例

extern uint64_t imu_tick;

const uint8_t CRC8Table[] ={
	0,  94, 188, 226, 97, 63, 221, 131,
	194, 156, 126, 32, 163, 253, 31, 65,
	157, 195, 33, 127, 252, 162, 64, 30,
	95, 1, 227, 189, 62, 96, 130, 220,
	35, 125, 159, 193, 66, 28, 254, 160,
	225, 191, 93, 3, 128, 222, 60, 98,
	190, 224, 2, 92, 223, 129, 99, 61,
	124, 34, 192, 158, 29, 67, 161, 255,
	70, 24, 250, 164, 39, 121, 155, 197,
	132, 218, 56, 102, 229, 187, 89, 7,
	219, 133, 103, 57, 186, 228, 6, 88,
	25, 71, 165, 251, 120, 38, 196, 154,
	101, 59, 217, 135, 4, 90, 184, 230,
	167, 249, 27, 69, 198, 152, 122, 36,
	248, 166, 68, 26, 153, 199, 37, 123,
	58, 100, 134, 216, 91, 5, 231, 185,
	140, 210, 48, 110, 237, 179, 81, 15,
	78, 16, 242, 172, 47, 113, 147, 205,
	17, 79, 173, 243, 112, 46, 204, 146,
	211, 141, 111, 49, 178, 236, 14, 80,
	175, 241, 19, 77, 206, 144, 114, 44,
	109, 51, 209, 143, 12, 82, 176, 238,
	50, 108, 142, 208, 83, 13, 239, 177,
	240, 174, 76, 18, 145, 207, 45, 115,
	202, 148, 118, 40, 171, 245, 23, 73,
	8, 86, 180, 234, 105, 55, 213, 139,
	87, 9, 235, 181, 54, 104, 138, 212,
	149, 203, 41, 119, 244, 170, 72, 22,
	233, 183, 85, 11, 136, 214, 52, 106,
	43, 117, 151, 201, 74, 20, 246, 168,
	116, 42, 200, 150, 21, 75, 169, 247,
	182, 232, 10, 84, 215, 137, 107, 53
};

const uint16_t CRC16Table[256] =
{
	LINE(0x0000, 0x1021, 0x2042, 0x3063),
	LINE(0x4084, 0x50A5, 0x60C6, 0x70E7),
	LINE(0x8108, 0x9129, 0xA14A, 0xB16B),
	LINE(0xC18C, 0xD1AD, 0xE1CE, 0xF1EF),
	LINE(0x1231, 0x0210, 0x3273, 0x2252),
	LINE(0x52B5, 0x4294, 0x72F7, 0x62D6),
	LINE(0x9339, 0x8318, 0xB37B, 0xA35A),
	LINE(0xD3BD, 0xC39C, 0xF3FF, 0xE3DE),
	LINE(0x2462, 0x3443, 0x0420, 0x1401),
	LINE(0x64E6, 0x74C7, 0x44A4, 0x5485),
	LINE(0xA56A, 0xB54B, 0x8528, 0x9509),
	LINE(0xE5EE, 0xF5CF, 0xC5AC, 0xD58D),
	LINE(0x3653, 0x2672, 0x1611, 0x0630),
	LINE(0x76D7, 0x66F6, 0x5695, 0x46B4),
	LINE(0xB75B, 0xA77A, 0x9719, 0x8738),
	LINE(0xF7DF, 0xE7FE, 0xD79D, 0xC7BC),
	LINE(0x48C4, 0x58E5, 0x6886, 0x78A7),
	LINE(0x0840, 0x1861, 0x2802, 0x3823),
	LINE(0xC9CC, 0xD9ED, 0xE98E, 0xF9AF),
	LINE(0x8948, 0x9969, 0xA90A, 0xB92B),
	LINE(0x5AF5, 0x4AD4, 0x7AB7, 0x6A96),
	LINE(0x1A71, 0x0A50, 0x3A33, 0x2A12),
	LINE(0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E),
	LINE(0x9B79, 0x8B58, 0xBB3B, 0xAB1A),
	LINE(0x6CA6, 0x7C87, 0x4CE4, 0x5CC5),
	LINE(0x2C22, 0x3C03, 0x0C60, 0x1C41),
	LINE(0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD),
	LINE(0xAD2A, 0xBD0B, 0x8D68, 0x9D49),
	LINE(0x7E97, 0x6EB6, 0x5ED5, 0x4EF4),
	LINE(0x3E13, 0x2E32, 0x1E51, 0x0E70),
	LINE(0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC),
	LINE(0xBF1B, 0xAF3A, 0x9F59, 0x8F78),
	LINE(0x9188, 0x81A9, 0xB1CA, 0xA1EB),
	LINE(0xD10C, 0xC12D, 0xF14E, 0xE16F),
	LINE(0x1080, 0x00A1, 0x30C2, 0x20E3),
	LINE(0x5004, 0x4025, 0x7046, 0x6067),
	LINE(0x83B9, 0x9398, 0xA3FB, 0xB3DA),
	LINE(0xC33D, 0xD31C, 0xE37F, 0xF35E),
	LINE(0x02B1, 0x1290, 0x22F3, 0x32D2),
	LINE(0x4235, 0x5214, 0x6277, 0x7256),
	LINE(0xB5EA, 0xA5CB, 0x95A8, 0x8589),
	LINE(0xF56E, 0xE54F, 0xD52C, 0xC50D),
	LINE(0x34E2, 0x24C3, 0x14A0, 0x0481),
	LINE(0x7466, 0x6447, 0x5424, 0x4405),
	LINE(0xA7DB, 0xB7FA, 0x8799, 0x97B8),
	LINE(0xE75F, 0xF77E, 0xC71D, 0xD73C),
	LINE(0x26D3, 0x36F2, 0x0691, 0x16B0),
	LINE(0x6657, 0x7676, 0x4615, 0x5634),
	LINE(0xD94C, 0xC96D, 0xF90E, 0xE92F),
	LINE(0x99C8, 0x89E9, 0xB98A, 0xA9AB),
	LINE(0x5844, 0x4865, 0x7806, 0x6827),
	LINE(0x18C0, 0x08E1, 0x3882, 0x28A3),
	LINE(0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E),
	LINE(0x8BF9, 0x9BD8, 0xABBB, 0xBB9A),
	LINE(0x4A75, 0x5A54, 0x6A37, 0x7A16),
	LINE(0x0AF1, 0x1AD0, 0x2AB3, 0x3A92),
	LINE(0xFD2E, 0xED0F, 0xDD6C, 0xCD4D),
	LINE(0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9),
	LINE(0x7C26, 0x6C07, 0x5C64, 0x4C45),
	LINE(0x3CA2, 0x2C83, 0x1CE0, 0x0CC1),
	LINE(0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C),
	LINE(0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8),
	LINE(0x6E17, 0x7E36, 0x4E55, 0x5E74),
	LINE(0x2E93, 0x3EB2, 0x0ED1, 0x1EF0)
};

#endif