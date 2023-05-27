#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "driver/mcpwm.h"
#include <stdint.h>

class MotorControl {
    mcpwm_unit_t mcpwm_unit;
    
    public:
    MotorControl(mcpwm_unit_t mcpwm_unit, uint8_t pin_a, uint8_t pin_b);
    void set_duty_cycle(float duty_cycle);
};

#endif