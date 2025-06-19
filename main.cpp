#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <atomic>
#include <pthread.h>
#include <mutex>
#include <condition_variable>
#include <queue>
#include "inc/motor.hpp"
#include "inc/common.hpp"
#include "inc/motor_control.hpp"

// 函数声明
void print_statistics();
pthread_t get_pthread_id(std::thread& t);

// 定义全局控制参数
float g_tor_des = 0.0f;  // 目标转矩
float g_spd_des = 0.0f;  // 目标速度
float g_pos_des = 0.0f;  // 目标位置
float g_k_pos = 0.0f;    // 位置增益
float g_k_spd = 0.0f;    // 速度增益

int main(int argc, char* argv[]) {
    // 检查命令行参数
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <mode>\n";
        std::cerr << "Modes: stop, tor, speed\n";
        return 1;
    }
    std::cout << "Starting motor control in mode: " << argv[1] << std::endl;
    // 根据命令行参数设置控制模式
    if (strcmp(argv[1], "stop") == 0) {
        // 停止模式：所有参数设为0
        g_tor_des = 0.0f;
        g_spd_des = 0.0f;
        g_pos_des = 0.0f;
        g_k_pos = 0.0f;
        g_k_spd = 0.0f;
    } else if (strcmp(argv[1], "tor") == 0) {
        // 转矩模式：设置目标转矩
        g_tor_des = 0.25f;
        g_spd_des = 0.0f;
        g_pos_des = 0.0f;
        g_k_pos = 0.0f;
        g_k_spd = 0.0f;
    } else if (strcmp(argv[1], "speed") == 0) {
        // 速度模式：设置目标速度和速度增益
        g_tor_des = 0.0f;
        g_spd_des = 6.28f;
        g_pos_des = 0.0f;
        g_k_pos = 0.0f;
        g_k_spd = 0.4f;
    }
    else if (strcmp(argv[1], "pos") == 0) {
        // 速度模式：设置目标速度和速度增益
        g_tor_des = 0.0f;
        g_spd_des = 0.0f;
        g_pos_des = 0.0f;
        g_k_pos = 30.0f;
        g_k_spd = 5.0f;
    }  
    else {
        std::cerr << "Invalid mode. Use 'stop', 'tor', or 'speed'.\n";
        return 1;
    }

    std::vector<std::thread> threads;

    // 初始化电机控制参数
    for (int i = 0; i < NUM_CHANNELS; ++i) {
        for (int j = 0; j < MOTORS_PER_CHANNEL; ++j) {
            g_motors[i][j].setControlParams(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);  // 初始化状态为0
        }
    }

    // 为每个通道创建线程
    for (int i = 0; i < NUM_CHANNELS; ++i) {
        threads.emplace_back([i]() {
            // 通道线程函数
            channel_thread(i);
        });

        // 获取线程ID并设置优先级
        pthread_t thread_id = get_pthread_id(threads.back());
        struct sched_param param;
        param.sched_priority = sched_get_priority_max(SCHED_FIFO);
        if (pthread_setschedparam(thread_id, SCHED_FIFO, &param) != 0) {
            std::cerr << "Failed to set thread priority for channel " << i << std::endl;
        }
    }
    std::cout << "All channel threads started." << std::endl;

    // 主循环
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        {
            std::lock_guard<std::mutex> lock(g_motor_mutex);

            for (int i = 0; i < NUM_CHANNELS; ++i) {
                for (int j = 0; j < MOTORS_PER_CHANNEL; ++j) {
                    // 更新电机控制参数，考虑减速比
                    if(j == 0) {
                        g_tor_des = 0; 
                        g_spd_des = 0; 
                        g_pos_des = 0; 
                        g_motors[i][j].setControlParams(
                            g_tor_des / GEAR_RATIO,     // 转子端转矩 = 输出端转矩 / 减速比
                            g_spd_des * GEAR_RATIO,     // 转子端速度 = 输出端速度 * 减速比
                            (g_pos_des + 0.83411) * GEAR_RATIO,     // 转子端位置 = 输出端位置 * 减速比
                            g_k_pos / GEAR_RATIO / GEAR_RATIO,  // 位置增益需要考虑两次减速比
                            g_k_spd / GEAR_RATIO / GEAR_RATIO   // 速度增益需要考虑两次减速比
                        );
                    }
                    else if(j == 1)
                    {
                        g_tor_des = 0; 
                        g_spd_des = 0; 
                        g_pos_des = 1.36; 
                        g_motors[i][j].setControlParams(
                            g_tor_des / GEAR_RATIO,     // 转子端转矩 = 输出端转矩 / 减速比
                            g_spd_des * GEAR_RATIO,     // 转子端速度 = 输出端速度 * 减速比
                            (g_pos_des - 0.950479) * GEAR_RATIO,     // 转子端位置 = 输出端位置 * 减速比
                            g_k_pos / GEAR_RATIO / GEAR_RATIO,  // 位置增益需要考虑两次减速比
                            g_k_spd / GEAR_RATIO / GEAR_RATIO   // 速度增益需要考虑两次减速比
                        );
                    }
                    else
                    {
                        g_tor_des = 0; 
                        g_spd_des = 0; 
                        g_pos_des = -2.8; 
                        g_motors[i][j].setControlParams(
                            g_tor_des / GEAR_RATIO / 1.88,     // 转子端转矩 = 输出端转矩 / 减速比
                            g_spd_des * GEAR_RATIO * 1.88,     // 转子端速度 = 输出端速度 * 减速比
                            -(g_pos_des + 2.569027) * GEAR_RATIO * 1.88 - 3.1415926,     // 转子端位置 = 输出端位置 * 减速比
                            g_k_pos / GEAR_RATIO / GEAR_RATIO,  // 位置增益需要考虑两次减速比
                            g_k_spd / GEAR_RATIO / GEAR_RATIO   // 速度增益需要考虑两次减速比
                        );
                    }
                }
            }
        }

        // 打印电机状态
        {
            std::lock_guard<std::mutex> lock(g_motor_mutex);

            for (int i = 0; i < NUM_CHANNELS; ++i) {
                for (int j = 0; j < MOTORS_PER_CHANNEL; ++j) {
                    // 获取电机状态并转换为输出端数据
                    float tor = 0.0f;
                    float spd = 0.0f; 
                    float pos = 0.0f;
                    if(i == 0)
                    {
                        if(j == 0)
                        {
                            tor = g_motors[i][j].getTorque() * GEAR_RATIO;  // 转换为输出端转矩
                            spd = g_motors[i][j].getSpeed() / GEAR_RATIO;   // 转换为输出端速度
                            pos = g_motors[i][j].getPosition() / GEAR_RATIO - 0.917742;// 转换为输出端位置          
                        }
                        else if(j == 1)
                        {
                            tor = -g_motors[i][j].getTorque() * GEAR_RATIO;  // 转换为输出端转矩
                            spd = -g_motors[i][j].getSpeed() / GEAR_RATIO;   // 转换为输出端速度
                            pos = -(g_motors[i][j].getPosition() / GEAR_RATIO - 1.775659);// 转换为输出端位置                          
                        }
                        else
                        {
                            tor = g_motors[i][j].getTorque() * GEAR_RATIO * 1.88;  // 转换为输出端转矩   (1.88是小腿电机额外齿轮的减速比)
                            spd = g_motors[i][j].getSpeed() / GEAR_RATIO/ 1.88;   // 转换为输出端速度    (1.88是小腿电机额外齿轮的减速比)
                            pos = g_motors[i][j].getPosition() / GEAR_RATIO / 1.88 - 3.380047;// 转换为输出端位置
                        }
                    }
                    if(i == 1)
                    {
                        if(j == 0)
                        {
                            tor = g_motors[i][j].getTorque() * GEAR_RATIO;  // 转换为输出端转矩
                            spd = g_motors[i][j].getSpeed() / GEAR_RATIO;   // 转换为输出端速度
                            pos = g_motors[i][j].getPosition() / GEAR_RATIO - 0.83411;// 转换为输出端位置         
                        }
                        else if(j == 1)
                        {
                            tor = g_motors[i][j].getTorque() * GEAR_RATIO;  // 转换为输出端转矩
                            spd = g_motors[i][j].getSpeed() / GEAR_RATIO;   // 转换为输出端速度
                            pos = g_motors[i][j].getPosition() / GEAR_RATIO + 0.950479;// 转换为输出端位置                           
                        }
                        else
                        {
                            tor = -g_motors[i][j].getTorque() * GEAR_RATIO * 1.88;  // 转换为输出端转矩   (1.88是小腿电机额外齿轮的减速比)
                            spd = -g_motors[i][j].getSpeed() / GEAR_RATIO/ 1.88;   // 转换为输出端速度    (1.88是小腿电机额外齿轮的减速比)
                            pos = -g_motors[i][j].getPosition()/ GEAR_RATIO / 1.88 - 2.831412;// 转换为输出端位置
                        }
                    }
                    if(i == 2)
                    {
                        if(j == 0)
                        {
                            tor = -g_motors[i][j].getTorque() * GEAR_RATIO;  // 转换为输出端转矩
                            spd = -g_motors[i][j].getSpeed() / GEAR_RATIO;   // 转换为输出端速度
                            pos = -g_motors[i][j].getPosition() / GEAR_RATIO - 0.036858;// 转换为输出端位置        
                        }
                        else if(j == 1)
                        {
                            tor = -g_motors[i][j].getTorque() * GEAR_RATIO;  // 转换为输出端转矩
                            spd = -g_motors[i][j].getSpeed() / GEAR_RATIO;   // 转换为输出端速度
                            pos = -g_motors[i][j].getPosition() / GEAR_RATIO + 1.4168;// 转换为输出端位置                        
                        }
                        else
                        {
                            tor = g_motors[i][j].getTorque() * GEAR_RATIO * 1.88;  // 转换为输出端转矩   (1.88是小腿电机额外齿轮的减速比)
                            spd = g_motors[i][j].getSpeed() / GEAR_RATIO/ 1.88;   // 转换为输出端速度    (1.88是小腿电机额外齿轮的减速比)
                            pos = g_motors[i][j].getPosition() / GEAR_RATIO / 1.88 - 3.2397;// 转换为输出端位置
                        }
                    }
                    if(i == 3)
                    {
                        if(j == 0)
                        {
                            tor = -g_motors[i][j].getTorque() * GEAR_RATIO;  // 转换为输出端转矩
                            spd = -g_motors[i][j].getSpeed() / GEAR_RATIO;   // 转换为输出端速度
                            pos = -g_motors[i][j].getPosition() / GEAR_RATIO + 0.414653;// 转换为输出端位置        
                        }
                        else if(j == 1)
                        {
                            tor = g_motors[i][j].getTorque() * GEAR_RATIO;  // 转换为输出端转矩
                            spd = g_motors[i][j].getSpeed() / GEAR_RATIO;   // 转换为输出端速度
                            pos = g_motors[i][j].getPosition() / GEAR_RATIO + 0.42181;// 转换为输出端位置                       
                        }
                        else
                        {
                            tor = -g_motors[i][j].getTorque() * GEAR_RATIO * 1.88;  // 转换为输出端转矩   (1.88是小腿电机额外齿轮的减速比)
                            spd = -g_motors[i][j].getSpeed() / GEAR_RATIO/ 1.88;   // 转换为输出端速度    (1.88是小腿电机额外齿轮的减速比)
                            pos = -g_motors[i][j].getPosition() / GEAR_RATIO / 1.88 - 2.231182;// 转换为输出端位置
                        }
                    }

                    float temp = g_motors[i][j].getTemperature();
                    uint16_t err = g_motors[i][j].getError();

                    // 打印电机状态（输出端的值）
                    std::cout << "Channel " << i << ", Motor " << j
                              << " - Output Torque: " << tor
                              << ", Output Speed: " << spd
                              << ", Output Position: " << pos
                              << ", Temp: " << temp
                              << ", Error: " << err << std::endl;
                }
            }
        }

        // 打印统计信息
        print_statistics();
    }

    // 清理资源
    g_running = false;  // 停止所有线程
    for (auto& thread : threads) {
        thread.join();  // 等待所有线程结束
    }

    return 0;
}

/**
 * 打印统计信息
 */
void print_statistics() {
    std::lock_guard<std::mutex> lock(g_motor_mutex);

    std::cout << "Channel | Motor | Sent | Received | Lost | Loss Rate | Errors\n";
    std::cout << "--------|-------|------|----------|------|-----------|-------\n";

    for (int i = 0; i < NUM_CHANNELS; ++i) {
        for (int j = 0; j < MOTORS_PER_CHANNEL; ++j) {
            uint64_t sent = g_motors[i][j].getSendCount();
            uint64_t received = g_motors[i][j].getReceiveCount();
            uint64_t errors = 0;
            int64_t lost = (sent > received) ? sent - received : 0;
            double loss_rate = (sent > 0) ? (double)lost / sent * 100 : 0;

            printf("%7d | %5d | %4lu | %8lu | %4ld | %9.2f%% | %5lu\n",
                   i, j, sent, received, lost, loss_rate, errors);

            g_motors[i][j].resetStats();
        }
    }
    std::cout << std::endl;
}

// 辅助函数，用于将std::thread转换为pthread_t
pthread_t get_pthread_id(std::thread& t) {
    pthread_t native_handle;
    #ifdef __GLIBCXX__
    native_handle = t.native_handle();
    #endif
    return native_handle;
}
