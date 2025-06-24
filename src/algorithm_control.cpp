#include "algorithm_control.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include "imu.hpp"
#include <iomanip> // 用于设置浮点数显示格式

// 全局变量，用于控制线程运行状态
extern std::atomic<bool> g_running;

void algorithm_control_thread() {
    std::cout << "Algorithm control thread started." << std::endl;
    auto next_send_time = std::chrono::steady_clock::now();

    // 打印表头
    std::cout << "\033[2J\033[H";
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "------------------- IMU Data -------------------\n";
    std::cout << std::setw(10) << "RollSpd" << std::setw(10) << "PitchSpd" << std::setw(10) << "HeadSpd"
              << std::setw(10) << "Roll" << std::setw(10) << "Pitch" << std::setw(10) << "Heading"
              << std::setw(10) << "Q1" << std::setw(10) << "Q2" << std::setw(10) << "Q3" << std::setw(10) << "Q4"
              << std::setw(15) << "Timestamp" << "\n";
    std::cout << "---------------- Body Velocity -----------------\n";
    std::cout << std::setw(12) << "Vel_X" << std::setw(12) << "Vel_Y" << std::setw(12) << "Vel_Z" << "\n";
    std::cout << "-------------- Body Acceleration --------------\n";
    std::cout << std::setw(12) << "Acc_X" << std::setw(12) << "Acc_Y" << std::setw(12) << "Acc_Z" << std::setw(12) << "G_force" << "\n";
    int imu_error_count = 0;
    while (g_running) {
        next_send_time += std::chrono::milliseconds(6);
        if(imu.get_imu_packet({0x41,0x60,0x62}) == false) {
            imu_error_count++;
            if (imu_error_count > 10) {
                std::cerr << "IMU data retrieval failed too many times, stopping thread." << std::endl;
                g_running = false;
                break;
            }
        }
        std::cout << "\033[2J\033[H"; // 清屏

        // IMU数据
        std::cout << "------------------- IMU Data -------------------\n";
        std::cout << std::setw(10) << imu.imu_data.RollSpeed
                  << std::setw(10) << imu.imu_data.aPitchSpeedcc_y
                  << std::setw(10) << imu.imu_data.HeadingSpeed
                  << std::setw(10) << imu.imu_data.Roll
                  << std::setw(10) << imu.imu_data.Pitch
                  << std::setw(10) << imu.imu_data.Heading
                  << std::setw(10) << imu.imu_data.Q1
                  << std::setw(10) << imu.imu_data.Q2
                  << std::setw(10) << imu.imu_data.Q3
                  << std::setw(10) << imu.imu_data.Q4
                  << std::setw(15) << imu.imu_data.Timestamp << "\n";

        // 速度
        std::cout << "---------------- Body Velocity -----------------\n";
        std::cout << std::setw(12) << imu.imu_body_vel.Velocity_X
                  << std::setw(12) << imu.imu_body_vel.Velocity_Y
                  << std::setw(12) << imu.imu_body_vel.Velocity_Z << "\n";

        // 加速度
        std::cout << "-------------- Body Acceleration --------------\n";
        std::cout << std::setw(12) << imu.imu_body_acc.Body_acceleration_X
                  << std::setw(12) << imu.imu_body_acc.Body_acceleration_Y
                  << std::setw(12) << imu.imu_body_acc.Body_acceleration_Z
                  << std::setw(12) << imu.imu_body_acc.G_force << "\n";

         // 打印错误计数
        std::cout << "IMU Error Count: " << imu_error_count << std::endl;

        std::this_thread::sleep_until(next_send_time);
    }
    std::cout << "Algorithm control thread stopped." << std::endl;
}