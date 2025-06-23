#include "algorithm_control.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include "imu.hpp"

// 全局变量，用于控制线程运行状态
extern std::atomic<bool> g_running;

void algorithm_control_thread() {
    std::cout << "Algorithm control thread started." << std::endl;
    auto next_send_time = std::chrono::steady_clock::now();
    while (g_running) {
        // 在这里实现算法控制逻辑
        next_send_time += std::chrono::milliseconds(5);
        imu.get_imu_packet(0x41); // 获取IMU数据包
        std::cout << "IMU Data: "
                  << "Roll: " << imu.imu_data.Roll << ", "
                  << "Pitch: " << imu.imu_data.Pitch << ", "
                  << "Heading: " << imu.imu_data.Heading << std::endl;
        
        // 等待直到下一次发送时间
        std::this_thread::sleep_until(next_send_time);
    }

    std::cout << "Algorithm control thread stopped." << std::endl;
}