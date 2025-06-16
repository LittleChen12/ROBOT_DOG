#ifndef SERIAL_INIT_HPP
#define SERIAL_INIT_HPP

#include <cstdint>
#include "motor.hpp"
#include "common.hpp"

int initialize_serial_port(const char* port_name);
bool configure_serial_port(int fd);
bool send_command_and_wait(int fd, Motor::ControlData_t& cmd, Motor::RecvData_t& response, int motor_id);
#endif 