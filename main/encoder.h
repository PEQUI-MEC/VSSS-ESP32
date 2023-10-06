#ifndef ENCODER_H
#define ENCODER_H

#include "driver/pcnt.h"
#include "hal/pcnt_hal.h"
#include "Types.h"

class Encoder {
    public:

    static constexpr int16_t PCNT_HIGH_LIMIT = 100;
    static constexpr int16_t PCNT_LOW_LIMIT = -100;
    static constexpr int16_t GEAR_RATIO = NEW_ROBOT ? 30 : 75;
    static constexpr int16_t PULSES_PER_REVOLUTION = 12;
    static constexpr float WHEEL_RADIUS = 0.06/2;

    pcnt_unit_t pcnt_unit;
    int accumu_count = 0;
    volatile float velocity = 0;

    int multiplier = 1;

    Encoder(pcnt_unit_t pcnt_unit, uint8_t pin_a, uint8_t pin_b, int multiplier);
    int get_count();
    float get_velocity();
    void update_velocity(float period);
};

#endif