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
#include "inc/algorithm_control.hpp"
#include "inc/motor_protect.hpp"
#include "inc/imu.hpp"

// 函数声明
void print_statistics();
pthread_t get_pthread_id(std::thread& t);

// 定义全局控制参数
float g_tor_des = 0.0f;  // 目标转矩
float g_spd_des = 0.0f;  // 目标速度
float g_pos_des = 0.0f;  // 目标位置
float g_k_pos = 0.0f;    // 位置增益
float g_k_spd = 0.0f;    // 速度增益

float _targetPos_1[12] = {0.0, 1.36, -2.65, 0.0, 1.36, -2.65,
                            -0.2, 1.36, -2.65, 0.2, 1.36, -2.65};

float _targetPos_2[12] = {0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
                            0.0, 0.67, -1.3, 0.0, 0.67, -1.3};

float _targetPos_3[12] = {-0.35, 1.36, -2.65, 0.35, 1.36, -2.65,
                            -0.5, 1.36, -2.65, 0.5, 1.36, -2.65};

float _startPos[12];

float _duration_0 = 500;                        
float _duration_1 = 500;   
float _duration_2 = 500; 
float _duration_3 = 1000;   
float _duration_4 = 900;   
float _duration_error = 1000; //电机错误状态的持续时间

float _percent_0 = 0;
float _percent_1 = 0;    
float _percent_2 = 0;    
float _percent_3 = 0;    
float _percent_4 = 0;
float _percent_error = 0; //电机错误状态  

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
        // 位置模式：设置目标位置和位置增益
        g_tor_des = 0.0f;
        g_spd_des = 0.0f;
        g_k_pos = 60.0f;
        g_k_spd = 5.0f;
        for (int i = 0; i < NUM_CHANNELS; ++i) {
            for (int j = 0; j < MOTORS_PER_CHANNEL; ++j) {
                // 更新电机控制参数，考虑减速比
                g_motors[i][j].Motor_SetControlParams(i, j, 0, 0, 0, 0, 0);//上电后先让电机处于停止状态
            }
        }
    } 
    else {
        std::cerr << "Invalid mode. Use 'stop', 'tor', or 'speed'.\n";
        return 1;
    }

    imu.serial_init("/dev/ttyUSB0"); // 初始化IMU串口

    std::vector<std::thread> threads;
    // 为每个通道创建线程
    for (int i = 0; i < NUM_CHANNELS; ++i) {
        threads.emplace_back([i]() {
            // 获取当前线程ID
            pthread_t thread_id = pthread_self();

            // 设置线程优先级
            struct sched_param param;//声明线程调度结构体
            param.sched_priority = sched_get_priority_max(SCHED_FIFO);//获取该策略最高优先级
            if (pthread_setschedparam(thread_id, SCHED_FIFO, &param) != 0) {//给指定ID线程设置调度策略和优先级
                // 如果设置失败，输出错误信息
                std::cerr << "Failed to set thread priority for channel " << i << std::endl;
            }

            // 设置线程的 CPU 亲和性（分配到核心 0），哪个位置1表示线程可以在哪个核心上运行
            cpu_set_t cpuset;
            CPU_ZERO(&cpuset);// 初始化 CPU 集合
            CPU_SET(0, &cpuset); // 核心 0（这个线程只在核心 0 上运行）
            if (pthread_setaffinity_np(thread_id, sizeof(cpu_set_t), &cpuset) != 0) {//将线程绑定到指定的 CPU 集合
                // 如果设置失败，输出错误信息
                std::cerr << "Failed to set CPU affinity for channel " << i << std::endl;
            }
            //为什么这么写？
            /*
            假设系统有 4 个 CPU 核心（0~3），可以这样分配：
            核心 0：专用通道控制线程（实时性要求极高）；
            核心 1：算法控制线程（计算密集型）；
            核心 2~3：留给操作系统和其他非关键任务；
            
            这种分配方式可以确保通道控制线程在核心 0 上运行，避免与其他线程竞争 CPU 时间，从而提高实时性和响应速度。
            另外，算法控制线程在核心 1 上运行，可以充分利用多核 CPU 的计算能力，同时避免与通道控制线程的实时性冲突。
            这种设计可以提高系统的整体性能和响应速度，特别是在需要高实时性和低延迟的应用场景中。
            例如，在机器人控制系统中，通道控制线程需要实时处理传感器数据并控制电机，而算法控制线程需要进行复杂的计算和决策。通过将它们分配到不同的核心，可以确保系统在处理实时任务时不会受到计算密集型任务的干扰。
            这种设计可以提高系统的整体性能和响应速度，特别是在需要高实时性和低延迟的应用场景中。
            */

            // 通道线程函数
            channel_thread(i);
        });
    }

    // 初始化算法控制线程
    threads.emplace_back([]() {
        // 获取当前线程ID
        pthread_t thread_id = pthread_self();

        // 设置线程优先级
        struct sched_param param;
        param.sched_priority = sched_get_priority_max(SCHED_FIFO);
        if (pthread_setschedparam(thread_id, SCHED_FIFO, &param) != 0) {
            std::cerr << "Failed to set thread priority for algorithm control thread." << std::endl;
        }

        // 设置线程的 CPU 亲和性（分配到核心 1）
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(1, &cpuset); // 核心 1
        if (pthread_setaffinity_np(thread_id, sizeof(cpu_set_t), &cpuset) != 0) {
            std::cerr << "Failed to set CPU affinity for algorithm control thread." << std::endl;
        }

        // 调用算法控制线程的功能性内容
        algorithm_control_thread();
    });
    
    std::cout << "All channel threads started." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));//等待通道线程串口的打开

    // 主循环
    int16_t tick_tick = 0; // 用于计时的变量
    while (g_running) 
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "imu_tick = "<< imu_tick << std::endl;
        imu_tick = 0; // 重置 IMU tick
        {
            std::lock_guard<std::mutex> lock(g_motor_mutex);

            // // 状态0 停止电机，并获取电机的当前位置
            // if (_percent_0 < 1) 
            // {
            //     _percent_0 += (float)1 / _duration_0;
            //     _percent_0 = _percent_0 > 1 ? 1 : _percent_0;
            //     for (int i = 0; i < NUM_CHANNELS; ++i) {
            //         for (int j = 0; j < MOTORS_PER_CHANNEL; ++j) {
            //             // 获取电机的当前位置
            //             g_motors[i][j].Motor_SetControlParams(i, j, 0, 0, 0, 0, 0);
            //             _startPos[i * MOTORS_PER_CHANNEL + j] = g_motors[i][j].getPosition(i,j);
            //         }
            //     }
            // }
            // // 状态0 -> 状态1 进入目标位置1
            // if ((_percent_0 == 1) &&(_percent_1 < 1))
            // {
            //     _percent_1 += (float)1 / _duration_1;
            //     _percent_1 = _percent_1 > 1 ? 1 : _percent_1;
            //     for (int i = 0; i < NUM_CHANNELS; ++i) {
            //         for (int j = 0; j < MOTORS_PER_CHANNEL; ++j) {
            //             // 更新电机控制参数，考虑减速比
            //             g_pos_des = (1 - _percent_1) * _startPos[i * MOTORS_PER_CHANNEL + j] + _percent_1 * _targetPos_1[i * MOTORS_PER_CHANNEL + j];  // 使用预定义的目标位置
            //             g_motors[i][j].Motor_SetControlParams(i, j, g_tor_des, g_spd_des, g_pos_des, g_k_pos, g_k_spd);
            //         }
            //     }
            // }
            // // 状态1 -> 状态2 起立
            // if ((_percent_1 == 1)&&(_percent_2 < 1))
            // {
            //     _percent_2 += (float)1 / _duration_2;
            //     _percent_2 = _percent_2 > 1 ? 1 : _percent_2;
            //     for (int i = 0; i < NUM_CHANNELS; ++i) {
            //         for (int j = 0; j < MOTORS_PER_CHANNEL; ++j) {
            //             // 更新电机控制参数，考虑减速比
            //             g_pos_des = (1 - _percent_2) * _targetPos_1[i * MOTORS_PER_CHANNEL + j] + _percent_2 * _targetPos_2[i * MOTORS_PER_CHANNEL + j];  // 使用预定义的目标位置
                        
            //             g_motors[i][j].Motor_SetControlParams(i, j, g_tor_des, g_spd_des, g_pos_des, g_k_pos, g_k_spd);
            //         }
            //     }                
            // }
            // // 状态2 -> 状态3 维持机身
            // if ((_percent_1 == 1)&&(_percent_2 == 1)&&(_percent_3<1))
            // {
            //     _percent_3 += (float)1 / _duration_3;
            //     _percent_3 = _percent_3 > 1 ? 1 : _percent_3;
            //     for (int i = 0; i < NUM_CHANNELS; ++i) {
            //         for (int j = 0; j < MOTORS_PER_CHANNEL; ++j) {
            //             // 更新电机控制参数，考虑减速比
            //             g_pos_des = _targetPos_2[i * MOTORS_PER_CHANNEL + j];  // 使用预定义的目标位置
                        
            //             g_motors[i][j].Motor_SetControlParams(i, j, g_tor_des, g_spd_des, g_pos_des, g_k_pos, g_k_spd);
            //         }
            //     }                    
    
            // }
            // // 状态3 -> 状态4
            // if ((_percent_1 == 1)&&(_percent_2 == 1)&&(_percent_3==1)&&((_percent_4<=1)))
            // {
            //     _percent_4 += (float)1 / _duration_4;
            //     _percent_4 = _percent_4 > 1 ? 1 : _percent_4;
            //     for (int i = 0; i < NUM_CHANNELS; ++i) {
            //         for (int j = 0; j < MOTORS_PER_CHANNEL; ++j) {
            //             // 更新电机控制参数，考虑减速比
            //             g_pos_des = (1 - _percent_4) * _targetPos_2[i * MOTORS_PER_CHANNEL + j] + _percent_4 * _targetPos_3[i * MOTORS_PER_CHANNEL + j];  // 使用预定义的目标位置
                        
            //             g_motors[i][j].Motor_SetControlParams(i, j, g_tor_des, g_spd_des, g_pos_des, g_k_pos, g_k_spd);
            //         }
            //     }      
            // }
            // // 电机发生错误状态
            // if (_percent_error == 1)
            // {
            //     _percent_error += (float)1 / _duration_error;
            //     _percent_error = _percent_error > 1 ? 1 : _percent_error;

            //     // 进入阻尼保护模式
            //     motor_protect();
            // }
            
        }
        // if(tick_tick++ > 100) // 每100次循环打印一次状态
        // {
        //     tick_tick = 0; // 重置计时器
        // // 更新电机状态
        //     {
        //         std::lock_guard<std::mutex> lock(g_motor_mutex);

        //         for (int i = 0; i < NUM_CHANNELS; ++i) {
        //             for (int j = 0; j < MOTORS_PER_CHANNEL; ++j) {
        //                 // 获取电机状态并转换为输出端数据
        //                 float tor = 0.0f;
        //                 float spd = 0.0f; 
        //                 float pos = 0.0f;
        //                 tor = g_motors[i][j].getTorque(i,j);  // 输出端转矩
        //                 spd = g_motors[i][j].getSpeed(i,j);   // 输出端速度
        //                 pos = g_motors[i][j].getPosition(i,j);// 输出端位置          
        //                 float temp = g_motors[i][j].getTemperature();
        //                 uint16_t err = g_motors[i][j].getError();

        //                 if (err != 0) {
        //                     _percent_error = 1; // 设置错误状态为1，表示发生错误
        //                 }

        //                 //打印电机状态（输出端的值）
        //                 std::cout << "Channel " << i << ", Motor " << j
        //                         << " - Output Torque: " << tor
        //                         << ", Output Speed: " << spd
        //                         << ", Output Position: " << pos
        //                         << ", Temp: " << temp
        //                         << ", Error: " << err << std::endl;
        //             }
        //         }
        //     }

        //     //打印统计信息
        //     print_statistics();
        // }
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
