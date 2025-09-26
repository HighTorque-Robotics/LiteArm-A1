#ifndef _CANBOARD_H_
#define _CANBOARD_H_
#include <iostream>
#include <vector>
#include "canport.hpp"
#include "parse_robot_params.hpp"


class canboard
{
private:
    int CANport_num;
    std::vector<canport*> CANport;

public:
    canboard(int _CANboard_ID, std::vector<serial_driver *> *ser, CANBoardParams &canboard_params);
    ~canboard() {}

    std::vector<canport*>& get_CANport();
    int get_CANport_num();
    void push_CANport(std::vector<canport*> *_CANport);
    void motor_send_2();
    void set_stop();
    void set_reset();
    float set_port_motor_num();
    void send_get_motor_state_cmd();
    void send_get_motor_state_cmd2();
    void send_get_motor_version_cmd();
    void set_fun_v(fun_version v);
    void set_data_reset();
    void set_reset_zero();
    void set_motor_runzero();
    void set_time_out(int16_t t_ms);
    void set_time_out(uint8_t portx, int16_t t_ms);
    void canboard_bootloader();
    void canboard_fdcan_reset();
};
#endif
