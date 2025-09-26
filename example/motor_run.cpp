#include "robot.hpp"
#include <iostream>

int main(int argc, char **argv)
{
    hightorque_robot::robot rb;
    rb.lcm_enable();
    const int motor_num = rb.Motors.size();
    int cont = 0;
    float angle = 0.2;
    while(1)
    {
        for (int i = 0; i < motor_num; i++)
        {
            if(i < (int)motor_num/2)
            {
                rb.Motors[i]->pos_vel_MAXtqe(angle, 0.1, 10);  
            }
            else
            {
                rb.Motors[i]->pos_vel_MAXtqe(-angle, 0.1, 10);  
            }
        }
        cont++;
        if(cont>=500)
        {
            cont = 0;
            angle*=-1;
        }
        rb.motor_send_2();
        ////////////////////////recv
        for (motor *m : rb.Motors)
        {
            motor_back_t motor;
            motor = *m->get_current_motor_state();
            ROS_INFO("ID:%2d,mode: %2d,fluat: %2X,pos: %8f,vel: %8f,tor: %8f\n", motor.ID, motor.mode, motor.fault, motor.position, motor.velocity, motor.torque);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}