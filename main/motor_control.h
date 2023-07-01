#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "driver/mcpwm.h"
#include <stdint.h>

//PID antigo
//    float kp = 1.26;
//    float ki = 0.0481;
//    float kd = 0;

struct PID {
    // float kp = 0.9;
    // float ki = 0.0481;
    // float kd = 0.25;
    float kp = 1.26;
    float ki = 0.0481;
    float kd = 0;
    float target_velocity = 0;
    float error_sum = 0;
    float error_diff = 0;
    float prev_error = 0;
};

class MotorControl {
    mcpwm_unit_t mcpwm_unit;

    PID pid;
    
    public:
    MotorControl(mcpwm_unit_t mcpwm_unit, uint8_t pin_a, uint8_t pin_b);
    void set_duty_cycle(float duty_cycle);
    void set_pid_target_velocity(float target_velocity);
    void update_pid(float current_velocity);
};

#endif