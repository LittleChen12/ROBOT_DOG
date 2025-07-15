#include "algorithm_control.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include "imu.hpp"
#include <iomanip> // 用于设置浮点数显示格式
#include <valarray>

using namespace torch::indexing;
using namespace std;
// 全局变量，用于控制线程运行状态
extern std::atomic<bool> g_running;

// 观测空间
float command[3] = {0,0,0};
float ev_angle[3];
float omega[3];
float q[12];
float q_dot[12];
float tau[12];
RL_ROTDOG rl_rotdog;
int net2motor[4] = {1, 0, 3, 2}; // 电机i对应网络输出的腿下标

int rl_start = 0; // RL控制开始标志
int rl_protect = 0; // RL保护标志

float RL_ROTDOG::pd_control(float target_q, float curr_q, float target_qd, float curr_qd)
{
    // PD控制器
    float error = target_q - curr_q;
    float error_dot = target_qd - curr_qd;

    // 计算控制力矩
    float control_torque = Kp * error + Kd * error_dot;

    // 更新目标转矩
    return control_torque;
}

void RL_ROTDOG::handleMessage()
{
    std::vector<float> obs; // 45个观测
    float imu_out;//用来存放归一化后的航向角
    obs.push_back(imu.imu_data.RollSpeed * omega_scale); // 绕x轴的角速度
    obs.push_back(imu.imu_data.aPitchSpeedcc_y * omega_scale); // 绕y轴的角速度
    obs.push_back(-imu.imu_data.HeadingSpeed * omega_scale); // 绕z轴的角速度

    //需要先将角度归一化到[-pi, pi]范围内
    if (imu.imu_data.Heading > M_PI) {
        imu_out = imu.imu_data.Heading - 2 * M_PI;
    } else if (imu.imu_data.Heading < -M_PI) {
        imu_out = imu.imu_data.Heading + 2 * M_PI;
    } else {
        imu_out = imu.imu_data.Heading;
    }
    obs.push_back(imu.imu_data.Roll * eu_ang_scale); // 绕x轴的角度
    obs.push_back(imu.imu_data.Pitch * eu_ang_scale); // 绕y轴的角度
    obs.push_back(-imu_out * eu_ang_scale); // 绕z轴的角度

    obs.push_back(cmd_x * lin_vel); // 期望的x轴角速度
    obs.push_back(cmd_y * lin_vel); // 期望的y轴角速度
    obs.push_back(cmd_rate * ang_vel); // 期望的z轴角速度

    // 关节位置观测（按网络顺序 FL, FR, RL, RR）
    for(int i=0; i<NUM_CHANNELS; i++) {
        int net_leg = net2motor[i];
        for(int j=0; j<MOTORS_PER_CHANNEL; j++) {
            float pos = g_motors[i][j].getPosition(i, j);
            int idx = net_leg * MOTORS_PER_CHANNEL + j;
            curr_pos[idx] = pos; // 更新当前关节位置
        }
    }

    for(int i=0; i<12; i++) {
        float pos_actor = (curr_pos[i] - init_pos[i]) * pos_scale; // 归一化关节位置
        obs.push_back(pos_actor); // 归一化关节位置
    }

    // 关节速度观测（按网络顺序 FL, FR, RL, RR）
    for(int i=0; i<NUM_CHANNELS; i++) {
        int net_leg = net2motor[i];
        for(int j=0; j<MOTORS_PER_CHANNEL; j++) {
            float vel = g_motors[i][j].getSpeed(i, j);
            int idx = net_leg * MOTORS_PER_CHANNEL + j;
            curr_vel[idx] = vel; // 更新当前关节速度
        }
    }

    for(int idx=0; idx<12; idx++) {
        float vel_actor = curr_vel[idx] * vel_scale; // 归一化关节速度
        obs.push_back(vel_actor); // 归一化关节速度
    }

    for(int i=0; i<12; i++) {
        obs.push_back(action_temp[i]); // 动作
    }
    // std::cout << "obs: ";
    // for (size_t i = 0; i < obs.size(); ++i) {
    //     std::cout << std::fixed << std::setprecision(4) << obs[i] << " ";
    // }
    // std::cout << std::endl;

    auto options = torch::TensorOptions().dtype(torch::kFloat32);
    torch::Tensor obs_tensor = torch::from_blob(obs.data(), {1, 45},options).to(device);

    auto obs_buf_batch = this->obs_buf.unsqueeze(0);

    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(obs_tensor.to(torch::kHalf));
    inputs.push_back(obs_buf_batch.to(torch::kHalf));

    //----------网络推理----------
    torch::Tensor action_tensor = model.forward(inputs).toTensor();

    action_buf = torch::cat( {action_buf.index({ Slice(1, None),Slice()}),action_tensor} , 0 );

    bool has_nan = false;
    for (float val : obs) {
        if (std::isnan(val)) {
            has_nan = true;
            break;
        }
    }
    if (has_nan) {
        std::cerr << "Warning: NaN detected in observation data." << std::endl;
        getchar();
    }
    //-----------------------------网络输出滤波--------------------------------
    torch::Tensor action_blend_tensor = 0.8*action_tensor + 0.2*last_action;
    last_action = action_tensor.clone();
 
    this->obs_buf = torch::cat({this->obs_buf.index({Slice(1, None), Slice()}), obs_tensor}, 0); // 历史观测移位
    // //----------------------------------------------------------------
    torch::Tensor action_raw = action_blend_tensor.squeeze(0);
    // move to cpu
    action_raw = action_raw.to(torch::kFloat32);
    action_raw = action_raw.to(torch::kCPU);
    // // assess the result
    auto action_getter = action_raw.accessor <float,1>();
    for (int j = 0; j < 12; j++)
    {
        float action_flt = action_getter[j];
        new_target = action_flt * action_scale[j] + init_pos[j];

        action[j] = new_target; // 更新目标位置
        action_temp[j] = action_getter[j];//网络的最后一个维度的输入
    }
     
}

void RL_ROTDOG::load_policy()
{   
    std::cout << model_path << std::endl;
    // load model from check point
    std::cout << "cuda::is_available():" << torch::cuda::is_available() << std::endl;
    device= torch::kCPU;
    if (torch::cuda::is_available()&&1){
        device = torch::kCUDA;
    }
    std::cout<<"device:"<<device<<endl;
    model = torch::jit::load(model_path);
    std::cout << "load model is successed!" << std::endl;
    model.to(device);
    std::cout << "LibTorch Version: " << TORCH_VERSION_MAJOR << "." 
              << TORCH_VERSION_MINOR << "." 
              << TORCH_VERSION_PATCH << std::endl;
    model.to(torch::kHalf);
    std::cout << "load model to device!" << std::endl;
    model.eval();
}
 
void RL_ROTDOG::init_policy(){
 // load policy
    std::cout << "RL model thread start"<<endl;
    cout <<"cuda_is_available:"<< torch::cuda::is_available() << endl;
    cout <<"cudnn_is_available:"<< torch::cuda::cudnn_is_available() << endl;
    
    model_path = "/home/zhu/Desktop/ROBOT_DOG/pre_train/model_jitt.pt";//载入jit模型
    load_policy();

 // initialize record
    action_buf = torch::zeros({history_length,12},device);
    this->obs_buf = torch::zeros({history_length,45}, device);//历史观测
    last_action = torch::zeros({1,12},device);

    action_buf = action_buf.to(torch::kHalf);
    this->obs_buf = this->obs_buf.to(torch::kHalf);
    last_action = last_action.to(torch::kHalf);

    for (int j = 0; j < 12; j++)
    {
        action_temp.push_back(0.0);
	    action.push_back(init_pos[j]);
        prev_action.push_back(init_pos[j]);
    }
    //hot start
    for (int i = 0; i < history_length; i++)//为历史观测初始化
    {
        // 将 data 转为 tensor 类型，输入到模型
        std::vector<float> obs;
        //---------------Push data into obsbuf--------------------
        obs.push_back(0);//request->omega[0]*omega_scale);
        obs.push_back(0);//request->omega[1]*omega_scale);
        obs.push_back(0);//request->omega[2]*omega_scale);

        obs.push_back(0);//request->eu_ang[0]*eu_ang_scale);
        obs.push_back(0);//request->eu_ang[1]*eu_ang_scale);
        obs.push_back(0);//request->eu_ang[2]*eu_ang_scale);

        // cmd
        obs.push_back(0);//控制指令x
        obs.push_back(0);//控制指令y
        obs.push_back(0);//控制指令yaw rate

        // pos q joint
        for (int i = 0; i < 12; ++i)
        {
            float pos = 0;//(request->q[i]  - init_pos[i])* pos_scale;
            obs.push_back(pos);
            action[i]=init_pos[i];
        }
        // vel q joint
        for (int i = 0; i < 12; ++i)
        {
            float vel = 0;//request->dq[i] * vel_scale;
            obs.push_back(vel);
        }
        // last action
        for (int i = 0; i < 12; ++i)
        {
            obs.push_back(0);//历史  self.cfg.env.history_len, self.num_dofs
        }
        auto options = torch::TensorOptions().dtype(torch::kFloat32);
        torch::Tensor obs_tensor = torch::from_blob(obs.data(),{1,45},options).to(device);
    }

}

int rl_tick = 0; // RL控制周期计数器
void algorithm_control_thread() {
    std::cout << "Algorithm control thread started." << std::endl;

    rl_rotdog.init_policy(); // 初始化策略
    auto next_send_time = std::chrono::steady_clock::now();

    int imu_error_count = 0;
    while (g_running) {
        next_send_time += std::chrono::milliseconds(1); 
//------------------------------------------------------rl控制
        if(rl_start >= 1)
        {
            rl_tick++;
        }
        if(rl_tick % 6 == 0 && rl_start >= 1) { // 每5次循环处理一次 200hz，写的是6,但实际是5次完整的循环
            rl_tick = 0;
            if(imu.get_imu_packet({0x41,0x60,0x62}) == false) {
                imu_error_count++;
                if (imu_error_count > 10) {
                    // std::cerr << "IMU data retrieval failed too many times, stopping thread." << std::endl;
                    // g_running = false;
                    // break;
                }
            }
        }
        if(rl_start == 10) 
        {
            // 更新电机位置和速度 1khz
            for(int i=0; i<NUM_CHANNELS; i++) {
                int net_leg_1 = net2motor[i];
                for(int j=0; j<MOTORS_PER_CHANNEL; j++) {
                    float pos = g_motors[i][j].getPosition(i, j);
                    int idx_1 = net_leg_1 * MOTORS_PER_CHANNEL + j;
                    rl_rotdog.curr_pos[idx_1] = pos; // 更新当前关节位置
                }
            }
            for(int i=0; i<NUM_CHANNELS; i++) {
                int net_leg_2 = net2motor[i];
                for(int j=0; j<MOTORS_PER_CHANNEL; j++) {
                    float vel = g_motors[i][j].getSpeed(i, j);
                    int idx_2 = net_leg_2 * MOTORS_PER_CHANNEL + j;
                    rl_rotdog.curr_vel[idx_2] = vel; // 更新当前关节速度
                }
            }
            // 计算PD控制力矩 1khz
            for(int i=0; i<12; i++){
                rl_rotdog.curr_tor[i] = rl_rotdog.pd_control(rl_rotdog.action[i], rl_rotdog.curr_pos[i], 0.0, rl_rotdog.curr_vel[i]);
                rl_rotdog.output_tor[i] = rl_rotdog.curr_tor[i]; // 更新输出力矩 curr_tor主要是为了预防网络输出太猛 output_tor才是实际的控制

                // 对实际输出力矩进行限制
                if(rl_rotdog.output_tor[i] > 12) {
                    rl_rotdog.output_tor[i] = 12; // 限制最大力矩
                } else if(rl_rotdog.output_tor[i] < -12) {
                    rl_rotdog.output_tor[i] = -12; // 限制最小力矩
                }
            }
        }
        if(rl_protect == 0 && rl_start == 10){
        //电机控制的顺序是FR，FL，RR，RL，网络输出是FL，FR，RL，RR
        // 这里需要将输出的动作重新排列为FR，FL，RR，RL
            for(int i=0; i<NUM_CHANNELS; i++){
                for(int j=0; j<MOTORS_PER_CHANNEL; j++) {
                        int net_leg = net2motor[i];
                        int idx = net_leg * MOTORS_PER_CHANNEL + j;
                        //网络输出限制，如果太大了，肯定是网络输出有问题
                        if (rl_rotdog.curr_tor[idx] > 25 || rl_rotdog.curr_tor[idx] < -25) {
                            std::cout << "Torque out of bounds, triggering protection!" << std::endl;
                            rl_start = 0; // 停止控制
                            rl_protect = 1;
                            motor_protect();
                        }
                        //力矩限制
                        if (rl_rotdog.curr_tor[idx] <= 25 && rl_rotdog.curr_tor[idx] >= -25) {
                            g_motors[i][j].Motor_SetControlParams(i, j, rl_rotdog.output_tor[idx], 0, 0, 0, 0);
                        }
                        
                }
            }
        }
        if((rl_start>1))
        {
            if(rl_rotdog.curr_pos[0] > 0.8 || rl_rotdog.curr_pos[0] < -0.6 ||
                rl_rotdog.curr_pos[1] > 1.6 || rl_rotdog.curr_pos[1] < 0.0 ||
                rl_rotdog.curr_pos[3] > 0.6 || rl_rotdog.curr_pos[3] < -0.8 ||
                rl_rotdog.curr_pos[4] > 1.6 || rl_rotdog.curr_pos[4] < 0.0 ||
                rl_rotdog.curr_pos[6] > 0.8 || rl_rotdog.curr_pos[6] < -0.6 ||
                rl_rotdog.curr_pos[7] > 1.7 || rl_rotdog.curr_pos[7] < 0.3 ||
                rl_rotdog.curr_pos[9] > 0.6  || rl_rotdog.curr_pos[9] < -0.8 ||
                rl_rotdog.curr_pos[10] > 1.7 || rl_rotdog.curr_pos[10] < 0.3) {
                    cout << "Position out of bounds, triggering protection!" << std::endl;
                    rl_start = 0; // 停止控制
                    rl_protect = 1; // 触发保护
                    motor_protect();
                }
        }
        if(rl_protect == 1) {
            rl_start = 0; // 停止控制
            motor_protect(); // 执行电机保护
        }
        if(imu.imu_data.Roll > 1.0 || imu.imu_data.Roll < -1.0 ||
           imu.imu_data.Pitch > 1.0 || imu.imu_data.Pitch < -1.0) {
            std::cout << "IMU data out of bounds, triggering protection!" << std::endl;
            rl_protect = 1; // 触发保护
            rl_start = 0; // 停止控制
            motor_protect(); // 执行电机保护
        }
//--------------------------------------------------------------------imu数据打印
        // if(rl_tick % 10 == 0) { // 每10次循环打印一次IMU数据
        // std::cout << "\033[2J\033[H"; // 清屏
        
        // std::cout << std::fixed << std::setprecision(6) << std::right;

        // float imu_out;
        // if (imu.imu_data.Heading > M_PI) {
        //     imu_out = imu.imu_data.Heading - 2 * M_PI;
        // } else if (imu.imu_data.Heading < -M_PI) {
        //     imu_out = imu.imu_data.Heading + 2 * M_PI;
        // } else {
        //     imu_out = imu.imu_data.Heading;
        // }

        // std::cout << "------------------- IMU Data -------------------\n";
        // std::cout << std::setw(12) << imu.imu_data.RollSpeed << " "
        //         << std::setw(12) << imu.imu_data.aPitchSpeedcc_y << " "
        //         << std::setw(12) << imu.imu_data.HeadingSpeed << " "
        //         << std::setw(12) << imu.imu_data.Roll << " "
        //         << std::setw(12) << imu.imu_data.Pitch << " "
        //         << std::setw(12) << imu_out << " "
        //         << std::setw(12) << imu.imu_data.Q1 << " "
        //         << std::setw(12) << imu.imu_data.Q2 << " "
        //         << std::setw(12) << imu.imu_data.Q3 << " "
        //         << std::setw(12) << imu.imu_data.Q4 << " "
        //         << std::setw(15) << imu.imu_data.Timestamp << "\n";

        // // std::cout << "---------------- Body Velocity -----------------\n";
        // // std::cout << std::setw(12) << imu.imu_body_vel.Velocity_X << " "
        // //         << std::setw(12) << imu.imu_body_vel.Velocity_Y << " "
        // //         << std::setw(12) << imu.imu_body_vel.Velocity_Z << "\n";

        // // std::cout << "-------------- Body Acceleration --------------\n";
        // // std::cout << std::setw(12) << imu.imu_body_acc.Body_acceleration_X << " "
        // //         << std::setw(12) << imu.imu_body_acc.Body_acceleration_Y << " "
        // //         << std::setw(12) << imu.imu_body_acc.Body_acceleration_Z << " "
        // //         << std::setw(12) << imu.imu_body_acc.G_force << "\n";

        //  // 打印错误计数
        // std::cout << "IMU Error Count: " << imu_error_count << std::endl;
        
        // for(int i = 0; i < NUM_CHANNELS; i++) {
        //     for (int j = 0; j < MOTORS_PER_CHANNEL; j++) {
        //         int idx = net2motor[i] * MOTORS_PER_CHANNEL + j;
        //         std::cout << "Motor " << i << "-" << j << ": "
        //                   << ", Tor: " << std::setw(10) << rl_rotdog.curr_tor[idx] 
        //                   << "\n";
        //     }
        // }
        // }

        std::this_thread::sleep_until(next_send_time);
}
    std::cout << "Algorithm control thread stopped." << std::endl;
}


//rl策略运行线程
void rl_run() {
    std::cout << "Algorithm control thread started." << std::endl;
    auto next_send_time = std::chrono::steady_clock::now();

    while (g_running) {
        next_send_time += std::chrono::milliseconds(20); 

        if(rl_start >= 1) { // 每20次循环处理一次 50hz
            rl_rotdog.handleMessage(); // 处理消息,推理网络，计算力矩
            if(rl_start<10){
                rl_start++; // 预热网络
            }
        }

        std::this_thread::sleep_until(next_send_time);
    }
}

// 设置终端为非阻塞、无缓冲模式
void set_terminal_mode(bool enable) {
    static struct termios oldt, newt;
    if (enable) {
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    } else {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }
}
void keyboard_thread() {
    set_terminal_mode(true);
    std::cout << "方向键控制cmd_x/cmd_y，Q/E控制cmd_rate，松开清零，e退出" << std::endl;
    while (g_running) {
        // 1. 先清零
        rl_rotdog.cmd_x = 0;
        rl_rotdog.cmd_y = 0;
        rl_rotdog.cmd_rate = 0;

        // 2. 使用select实现非阻塞检测
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(STDIN_FILENO, &rfds);
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 20000; // 20ms

        int ret = select(STDIN_FILENO + 1, &rfds, NULL, NULL, &tv);
        if (ret > 0 && FD_ISSET(STDIN_FILENO, &rfds)) {
            char c = getchar();
            if (c == '\033') { // 方向键转义序列
                if (getchar() == '[') {
                    char dir = getchar();
                    switch (dir) {
                        case 'A': 
                            rl_rotdog.cmd_x = 1; 
                            break;  // 上
                        case 'B': 
                            rl_rotdog.cmd_x = -1; 
                            break; // 下
                        case 'C': 
                            rl_rotdog.cmd_y = -1; 
                            break;  // 左
                        case 'D': 
                            rl_rotdog.cmd_y = 1; 
                            break; // 右
                    }
                }
            } else if (c == 'q' || c == 'Q') {
                rl_rotdog.cmd_rate = 1;
            } else if (c == 'e' || c == 'E') {
                rl_rotdog.cmd_rate = -1;
            } else if (c == 'x' || c == 'X') {
                // g_running = false;
                std::cout << "[KEY] 退出程序" << std::endl;
                break;
            }
        }
        // 20ms刷新一次
        usleep(20000);
    }
    set_terminal_mode(false);
}