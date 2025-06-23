#include <iostream>
#include "motor_protect.hpp"
#include "common.hpp"
#include "motor.hpp"
#include "motor_control.hpp"


/**
 * 电机阻尼保护函数
 */
void motor_protect() {           
    for (int i = 0; i < NUM_CHANNELS; ++i) {
                    for (int j = 0; j < MOTORS_PER_CHANNEL; ++j) {
                        // 设置电机进入阻尼保护模式
                        g_motors[i][j].Motor_SetControlParams(i, j, 0, 0, 0, 0, 5.0f); // 设置阻尼保护参数,速度增益
                    }
                }
}