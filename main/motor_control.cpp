
#include "motor_control.h"
#include "driver/mcpwm.h"
#include "driver/pcnt.h"
#include "Types.h"

#define MOTOR_CTRL_MCPWM_TIMER MCPWM_TIMER_0

MotorControl::MotorControl(mcpwm_unit_t mcpwm_unit, uint8_t pin_a, uint8_t pin_b, int multiplier)
        : mcpwm_unit(mcpwm_unit),
          multiplier(multiplier) {
    // configure motor control pins
    mcpwm_gpio_init(mcpwm_unit, MCPWM0A, pin_a);
    mcpwm_gpio_init(mcpwm_unit, MCPWM0B, pin_b);

    // configure motor control timer
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;     //frequency = 1kHz,
    pwm_config.cmpr_a = 0;                              //initial duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;                              //initial duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;         //up counting mode
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(mcpwm_unit, MOTOR_CTRL_MCPWM_TIMER, &pwm_config);    //Configure PWM0A & PWM0B with above settings

    if (NEW_ROBOT) {
        pid.kd = 2 * pid.kd;
        pid.ki = 2 * pid.ki;
        pid.kp = 2 * pid.kp;
    }
}

void MotorControl::set_duty_cycle(float duty_cycle) {
    /* motor moves in forward direction, with duty cycle = duty % */
    duty_cycle *= multiplier;
    if (duty_cycle > 0) {
        mcpwm_set_signal_low(mcpwm_unit, MOTOR_CTRL_MCPWM_TIMER, MCPWM_OPR_A);
        mcpwm_set_duty(mcpwm_unit, MOTOR_CTRL_MCPWM_TIMER, MCPWM_OPR_B, duty_cycle);
        mcpwm_set_duty_type(mcpwm_unit, MOTOR_CTRL_MCPWM_TIMER, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);  //call this each time, if operator was previously in low/high state
    } else { /* motor moves in backward direction, with duty cycle = -duty % */
        mcpwm_set_signal_low(mcpwm_unit, MOTOR_CTRL_MCPWM_TIMER, MCPWM_OPR_B);
        mcpwm_set_duty(mcpwm_unit, MOTOR_CTRL_MCPWM_TIMER, MCPWM_OPR_A, -duty_cycle);
        mcpwm_set_duty_type(mcpwm_unit, MOTOR_CTRL_MCPWM_TIMER, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
    }
}

void MotorControl::set_pid_target_velocity(float target_velocity) {
    if (pid.target_velocity != target_velocity) {
        pid.error_sum = 0;
        pid.prev_error = 0;
    }
    pid.target_velocity = target_velocity;
}

void MotorControl::update_pid(float current_velocity) {
    float error = pid.target_velocity - current_velocity;
    pid.error_sum = pid.error_sum + error;
    float error_diff = error - pid.prev_error;
    pid.prev_error = error;
    float duty_cycle = pid.kp * error + pid.ki * pid.error_sum + pid.kd * error_diff;
    // parameters adjusted for duty cycle between -1 and 1
    if (pid.target_velocity == 0) {
        set_duty_cycle(0);
        pid.error_sum = 0;
        pid.prev_error = 0;
    } else {
        set_duty_cycle(duty_cycle * 100);
    }
}