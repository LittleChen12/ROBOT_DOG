#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <atomic>
#include <mutex>
#include "motor_control.hpp"
#include "serial_init.hpp"
#include "common.hpp"

// 全局变量
std::vector<std::vector<Motor>> g_motors(NUM_CHANNELS, std::vector<Motor>(MOTORS_PER_CHANNEL));
std::atomic<bool> g_running(true);
std::mutex g_motor_mutex;

/**
 * 电机控制线程函数
 */
void channel_thread(int channel) {
    // 构造串口设备名
    char port_name[20];
    snprintf(port_name, sizeof(port_name), "/dev/ttyACM%d", channel);

    // 初始化串口
    int fd = initialize_serial_port(port_name);
    if (fd < 0) {
        std::cerr << "Failed to initialize port " << port_name << std::endl;
        return;
    }

    // 切换回阻塞模式
    fcntl(fd, F_SETFL, 0);

    uint8_t recv_buffer[MAX_BUFFER_SIZE];
    int current_motor = 0;
    int retry_count = 0;

    // 初始化发送缓冲区
    std::vector<Motor::ControlData_t> send_buffer(MOTORS_PER_CHANNEL);
    for (int i = 0; i < MOTORS_PER_CHANNEL; ++i) {
        send_buffer[i] = g_motors[channel][i].createControlPacket(i);
    }

    auto next_send_time = std::chrono::steady_clock::now();

    while (g_running) {
        next_send_time += std::chrono::milliseconds(1);
        for (int motor_idx = 0; motor_idx < MOTORS_PER_CHANNEL; ++motor_idx) {
            // 获取当前电机的控制参数
            Motor::ControlData_t cmd;
            {
                std::lock_guard<std::mutex> lock(g_motor_mutex);
                cmd = g_motors[channel][current_motor].createControlPacket(current_motor);
            }

            // 发送命令并等待响应
            Motor::RecvData_t response;
            bool success = false;

            // 尝试发送和接收，最多重试MAX_RETRY_COUNT次
            for (retry_count = 0; retry_count < MAX_RETRY_COUNT; ++retry_count) {
                success = send_command_and_wait(fd, cmd, response, current_motor);
                if (success) break;

                // 重试前短暂延时
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            if (success) {
                // 更新电机反馈数据
                {
                    std::lock_guard<std::mutex> lock(g_motor_mutex);
                    g_motors[channel][current_motor].updateFeedback(response);
                    g_motors[channel][current_motor].incrementSendCount();
                    g_motors[channel][current_motor].incrementReceiveCount();
                }
            } else {
                // 记录发送失败
                {
                    std::lock_guard<std::mutex> lock(g_motor_mutex);
                    g_motors[channel][current_motor].incrementSendCount();
                }
                std::cerr << "Failed to communicate with motor " << current_motor 
                          << " after " << MAX_RETRY_COUNT << " retries" << std::endl;
            }
            // 切换到下一个电机
            current_motor = (current_motor + 1) % MOTORS_PER_CHANNEL;
        }
        // 等待直到下一次发送时间
        std::this_thread::sleep_until(next_send_time);
    }

    // 关闭串口
    close(fd);
}