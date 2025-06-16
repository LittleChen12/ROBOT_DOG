#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include "serial_init.hpp"
#include <thread>
/**
 * 发送数据包并等待响应
 */
bool send_command_and_wait(int fd, Motor::ControlData_t& cmd, Motor::RecvData_t& response, int motor_id) {
    // 清空输入缓冲区
    tcflush(fd, TCIFLUSH);

    // 发送命令
    ssize_t bytes_written = write(fd, &cmd, sizeof(Motor::ControlData_t));
    if (bytes_written != sizeof(Motor::ControlData_t)) {
        std::cerr << "Failed to send command to motor " << motor_id
                  << ". Bytes written: " << bytes_written << std::endl;
        return false;
    }

    // 设置超时计时器
    auto start_time = std::chrono::steady_clock::now();

    // 等待响应
    uint8_t recv_buffer[MAX_BUFFER_SIZE];
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - start_time).count() < COMM_TIMEOUT_MS) {

        // 检查是否有数据可读
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fd, &readfds);

        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 1000;  // 1ms

        int ready = select(fd + 1, &readfds, NULL, NULL, &timeout);
        if (ready > 0 && FD_ISSET(fd, &readfds)) {
            // 读取数据
            ssize_t bytes_read = read(fd, recv_buffer, MAX_BUFFER_SIZE);
            if (bytes_read >= sizeof(Motor::RecvData_t)) {
                Motor::RecvData_t* recv_packet = reinterpret_cast<Motor::RecvData_t*>(recv_buffer);

                // 验证数据包头
                if (recv_packet->head[0] == 0xFD && recv_packet->head[1] == 0xEE) {
                    // 验证CRC
                    uint16_t calculated_crc = crc_ccitt(0x2cbb, 
                        reinterpret_cast<uint8_t*>(recv_packet), sizeof(Motor::RecvData_t) - 2);

                    // 验证电机ID
                    if (calculated_crc == recv_packet->CRC16 && 
                        recv_packet->mode.id == motor_id) {

                        // 复制有效响应
                        response = *recv_packet;
                        return true;
                    } else if (calculated_crc != recv_packet->CRC16) {
                        std::cerr << "CRC error in response from motor " 
                                  << recv_packet->mode.id << std::endl;
                    } else if (recv_packet->mode.id != motor_id) {
                        std::cerr << "Unexpected motor ID in response: " 
                                  << recv_packet->mode.id << " (expected " << motor_id << ")" << std::endl;
                    }
                }
            }
        }
    }

    std::cerr << "Timeout waiting for response from motor " << motor_id << std::endl;
    return false;
}

/**
 * 初始化串口
 * @param port_name 串口设备名
 * @return 成功返回文件描述符，失败返回-1
 */
int initialize_serial_port(const char* port_name) {
    // 以读写方式打开串口设备
    int fd = open(port_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        std::cerr << "Error opening " << port_name << ": " << strerror(errno) << std::endl;
        return -1;
    }

    // 配置串口参数
    if (!configure_serial_port(fd)) {
        close(fd);
        return -1;
    }

    return fd;
}

/**
 * 配置串口参数，提高稳定性
 */
bool configure_serial_port(int fd) {
    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    // 获取当前串口设置
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Error from tcgetattr: " << strerror(errno) << std::endl;
        return false;
    }

    // 设置波特率 (根据实际设备调整)
    cfsetospeed(&tty, B4000000);
    cfsetispeed(&tty, B4000000);

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