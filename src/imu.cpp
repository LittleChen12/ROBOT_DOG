#include "imu.hpp"
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include "serial_init.hpp"
#include <cstring>
#include <chrono>
#include <iomanip>
uint64_t imu_tick;
IMU imu;

uint8_t CRC8_Table(uint8_t* p, uint8_t counter)
{
	uint8_t crc8 = 0;
	for (int i = 0; i < counter; i++)
	{
		uint8_t value = p[i];
		uint8_t new_index = crc8 ^ value;
		crc8 = CRC8Table[new_index];
	}
	return (crc8);
}

uint16_t CRC16_Table(uint8_t* p, uint8_t counter)
{
	uint16_t crc16 = 0;
	for (int i = 0; i < counter; i++)
	{
		uint8_t value = p[i];
		crc16 = CRC16Table[((crc16 >> 8) ^ value) & 0xff] ^ (crc16 << 8);
	}
	return (crc16);
}

//初始化IMU类的构造函数
IMU::IMU() {
    // 初始化IMU数据
    imu_data.aPitchSpeedcc_y = 0.0f;
    imu_data.RollSpeed = 0.0f;
    imu_data.HeadingSpeed = 0.0f;
    imu_data.Roll = 0.0f;
    imu_data.Pitch = 0.0f;  
    imu_data.Heading = 0.0f;
    imu_data.Q1 = 0.0f;
    imu_data.Q2 = 0.0f;
    imu_data.Q3 = 0.0f;
    imu_data.Q4 = 0.0f;
    imu_data.Timestamp = 0;
}

//获取IMU数据
void IMU::getData() {
    // 在这里可以添加获取IMU数据的逻辑

}


//创建IMU请求数据包，返回帧长度
int IMU::create_imu_packet(uint8_t* buffer, IMU::FDILink_Status_t* FDILink, uint8_t type, void* buf, int len)
{
    if(len > 250)
    {
        return -1; //数据长度超过限制
    }

	buffer[0] = 0xfc;//帧头
	buffer[1] = type;//指令类别
	buffer[2] = len;//数据长度
	buffer[3] = FDILink->TxNumber++;//发送帧计数

    //CRC8与CRC16计算
	uint8_t CRC8 = CRC8_Table(buffer, 4);
	buffer[4] = CRC8;
	uint8_t* buf_data = buffer + 7;
	for(int i = 0;i < len;i++)
	{
		buf_data[i] = ((uint8_t*)buf)[i];
	}
	uint16_t CRC16 = CRC16_Table(buf_data, len);
	buffer[5] = (CRC16 >> 8);//高八位
	buffer[6] = (CRC16 & 0xFF);//第八位
	buffer[8 + len - 1] = 0XFD;//帧尾
	return 8 + len;
}
bool IMU::get_imu_packet(uint8_t imu_ID)
{
    // 发送IMU请求数据包
    uint8_t p_buff[12];
    uint8_t buffer[4] = {imu_ID, 0, 0, 0};
    create_imu_packet(p_buff, &FDILink_Status, 0xA0, buffer, sizeof(buffer));
    tcflush(imu_fd, TCIFLUSH);
    ssize_t bytes_written = write(imu_fd, p_buff, 12);
    if (bytes_written != 12) {
        std::cerr << "Failed to write to IMU" << std::endl;
        return false;
    }

    uint8_t recv_buffer[256];
    int recv_len = 0;
    auto start_time = std::chrono::steady_clock::now();

    while (std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - start_time).count() < COMM_TIMEOUT_MS)
    {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(imu_fd, &readfds);

        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 3000; // 3ms

        int ready = select(imu_fd + 1, &readfds, NULL, NULL, &timeout);
        if (ready > 0 && FD_ISSET(imu_fd, &readfds))
        {
            ssize_t bytes_read = read(imu_fd, recv_buffer + recv_len, sizeof(recv_buffer) - recv_len);
            if (bytes_read > 0) {
                recv_len += bytes_read;

                // 查找完整数据包
                for (int i = 0; i <= recv_len - 56; ++i) {
                    if (recv_buffer[i] == 0xFC && recv_buffer[i + 1] == imu_ID) {
                        uint8_t data_len = recv_buffer[i + 2];
                        if (data_len != 48) continue; // 只处理48字节数据区的数据包
                        if (recv_buffer[i + 55] != 0xFD) continue; // 帧尾校验

                        // CRC8校验
                        uint8_t crc8 = CRC8_Table(recv_buffer + i, 4);
                        if (crc8 != recv_buffer[i + 4]) {
                            std::cerr << "[调试] CRC8校验失败" << std::endl;
                            continue;
                        }

                        // CRC16校验
                        uint16_t crc16_recv = (recv_buffer[i + 5] << 8) | recv_buffer[i + 6];
                        uint16_t crc16_calc = CRC16_Table(recv_buffer + i + 7, 48);
                        if (crc16_recv != crc16_calc) {
                            std::cerr << "[调试] CRC16校验失败" << std::endl;
                            continue;
                        }

                        // 解析数据区到IMUData_t
                        std::memcpy(&imu_data, recv_buffer + i + 7, sizeof(IMUData_t)); // IMUData_t应为48字节

                        //std::cout << "[调试] IMU数据包已写入结构体，起始位置: " << i << std::endl;
                        imu_tick++;
                        return true;
                    }
                }
                if (recv_len > 200) recv_len = 0;
            }
        }
    }
    std::cerr << "Timeout or no valid IMU packet received!" << std::endl;
    return false;
}

void IMU::serial_init(const char* port_name) {
    // 打开串口设备
    imu_fd = open(port_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (imu_fd < 0) {
        std::cerr << "Error opening " << port_name << ": " << strerror(errno) << std::endl;
        return;
    }

    // 配置串口参数
    if (!configure_imu_serial(imu_fd)) {
        close(imu_fd);
        std::cerr << "Failed to configure serial port" << std::endl;
        return;
    }

    // 设置串口为阻塞模式
    fcntl(imu_fd, F_SETFL, 0);
}

/**
 * 配置串口参数，提高稳定性
 */
bool IMU::configure_imu_serial(int fd) {
    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    // 获取当前串口设置
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Error from tcgetattr: " << strerror(errno) << std::endl;
        return false;
    }

    // 设置波特率 (根据实际设备调整)
    cfsetospeed(&tty, B921600);
    cfsetispeed(&tty, B921600);

    // 配置串口参数
    tty.c_cflag |= (CLOCAL | CREAD);  // 本地连接，接收使能
    tty.c_cflag &= ~PARENB;           // 无校验位
    tty.c_cflag &= ~CSTOPB;           // 1个停止位
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;               // 8位数据位
    tty.c_cflag &= ~CRTSCTS;          // 无硬件流控

    // 输入模式配置
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);

    // 输出模式配置
    tty.c_oflag &= ~OPOST;            // 原始输出

    // 本地模式配置
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    // 控制字符配置
    tty.c_cc[VMIN] = 0;               // 非规范模式读取时的最小字符数
    tty.c_cc[VTIME] = 10;             // 非规范模式读取时的超时时间(0.1秒)

    // 清空输入输出缓冲区
    tcflush(fd, TCIOFLUSH);

    // 应用新的串口设置
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error from tcsetattr: " << strerror(errno) << std::endl;
        return false;
    }

    return true;
}



