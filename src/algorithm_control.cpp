#include "algorithm_control.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>

// 全局变量，用于控制线程运行状态
extern std::atomic<bool> g_running;

void algorithm_control_thread() {
    std::cout << "Algorithm control thread started." << std::endl;

    while (g_running) {
        // 在这里实现算法控制逻辑


        // 模拟计算或控制逻辑
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "Algorithm control thread stopped." << std::endl;
}