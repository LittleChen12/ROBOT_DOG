#ifndef ALGORITHM_CONTROL_HPP
#define ALGORITHM_CONTROL_HPP

#include <iostream>
#include <vector>
#include <stdio.h>
#include <torch/torch.h>
#include <torch/script.h> 
#include "motor.hpp"
#include "common.hpp"
#include "motor_control.hpp"
#include "motor_protect.hpp"
#include "mathTools.h"
#include "mathTypes.h"
#include "enumClass.h"
//键盘监听
#include <termios.h>
#include <unistd.h>

// 算法控制线程函数声明
void algorithm_control_thread();
void rl_run();
void keyboard_thread();


class RL_ROTDOG {
public:
    std::string model_path;
    void init_policy();
    void load_policy();
    void handleMessage();
    float pd_control(float target_q, float curr_q, float target_qd, float curr_qd);

    float Kp = 30.0;
    float Kd = 0.75;

    //gamepad
    float smooth = 0.03;
    float dead_zone = 0.01;

    float cmd_x = 0.;
    float cmd_y = 0.;
    float cmd_rate = 0.;

    std::vector<float> action;
    std::vector<float> action_temp;
    std::vector<float> prev_action;

    torch::Tensor action_buf;
    torch::Tensor obs_buf;
    torch::Tensor last_action;

    // default values
    int action_refresh=0;
    int history_length = 10;
    float init_pos[12] = {0.1,0.8,-1.5, -0.1,0.8,-1.5, 0.1,1.0,-1.5, -0.1,1.0,-1.5};
    float eu_ang_scale= 1.0;
    float omega_scale=  0.25;
    float pos_scale =   1.0;
    float vel_scale =   0.05;
    float lin_vel = 2.0;
    float ang_vel = 0.25;
    float action_scale[12] = {0.25,0.25,0.25, 0.25,0.25,0.25, 0.25,0.25,0.25, 0.25,0.25,0.25};
    float action_delta_max = 1.0;
    float action_delta_min = -1.0;

    float g_tor;
    float curr_pos[12];
    float curr_vel[12];
    float curr_tor[12];
    float output_tor[12];
    float new_target = 0.0;
    torch::jit::script::Module model;
    torch::DeviceType device;
private:

   
};


// 全局变量
extern int rl_start; // RL控制开始标志
#endif // ALGORITHM_CONTROL_HPP