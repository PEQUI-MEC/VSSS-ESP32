#ifndef ENCODER_H
#define ENCODER_H

#include "driver/pcnt.h"
#include "hal/pcnt_hal.h"

class Encoder {
    public:

    static constexpr int16_t PCNT_HIGH_LIMIT = 100;
    static constexpr int16_t PCNT_LOW_LIMIT = -100;

    pcnt_unit_t pcnt_unit;
    int accumu_count = 0;

    Encoder(pcnt_unit_t pcnt_unit, uint8_t pin_a, uint8_t pin_b);
    int get_count();
};

#endif