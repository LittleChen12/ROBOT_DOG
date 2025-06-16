#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP

#include <cstdint>
#include <vector>
#include "motor.hpp"
#include "common.hpp"

// 函数声明
void channel_thread(int channel);

// 全局变量声明
extern std::vector<std::vector<Motor>> g_motors;
extern std::atomic<bool> g_running;
extern std::mutex g_motor_mutex;

#endif