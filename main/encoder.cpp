#include "encoder.h"
#include "math.h"

static bool is_pcnt_isr_service_installed = false;

static void pcnt_overflow_handler(void *arg) {
    Encoder * encoder = (Encoder *) arg;
    uint32_t status = 0;
    pcnt_get_event_status(encoder->pcnt_unit, &status);

    if (status & PCNT_EVT_H_LIM) {
        encoder->accumu_count += Encoder::PCNT_HIGH_LIMIT;
    } else if (status & PCNT_EVT_L_LIM) {
        encoder->accumu_count += Encoder::PCNT_LOW_LIMIT;
    }
}

Encoder::Encoder(pcnt_unit_t pcnt_unit, uint8_t pin_a, uint8_t pin_b, int multiplier)
        : pcnt_unit(pcnt_unit),
          multiplier(multiplier) {
    // Configure channel 0
    pcnt_config_t dev_config = {
        .pulse_gpio_num = pin_a,
        .ctrl_gpio_num = pin_b,
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_DEC,
        .neg_mode = PCNT_COUNT_INC,
        .counter_h_lim = PCNT_HIGH_LIMIT,
        .counter_l_lim = PCNT_LOW_LIMIT,
        .unit = pcnt_unit,
        .channel = PCNT_CHANNEL_0,
    };
    pcnt_unit_config(&dev_config);

    // Configure channel 1
    dev_config.pulse_gpio_num = pin_b;
    dev_config.ctrl_gpio_num = pin_a;
    dev_config.channel = PCNT_CHANNEL_1;
    dev_config.pos_mode = PCNT_COUNT_INC;
    dev_config.neg_mode = PCNT_COUNT_DEC;
    pcnt_unit_config(&dev_config);

    pcnt_counter_pause(pcnt_unit);
    pcnt_counter_clear(pcnt_unit);

    if (!is_pcnt_isr_service_installed) {
        pcnt_isr_service_install(0);
        // make sure pcnt isr service won't be installed more than one time
        is_pcnt_isr_service_installed = true;
    }

    pcnt_isr_handler_add(pcnt_unit, pcnt_overflow_handler, this);
    pcnt_event_enable(pcnt_unit, PCNT_EVT_H_LIM);
    pcnt_event_enable(pcnt_unit, PCNT_EVT_L_LIM);

    pcnt_set_filter_value(pcnt_unit, 80);
    pcnt_filter_enable(pcnt_unit);

    pcnt_counter_resume(pcnt_unit);
}

int Encoder::get_count() {
    int16_t count;
    pcnt_get_counter_value(pcnt_unit, &count);
    return count + accumu_count;
}

float Encoder::get_velocity() {
    return velocity;
}

void Encoder::update_velocity(float period) {
    velocity = multiplier * (float(get_count()) * 2 * M_PI * WHEEL_RADIUS) / (PULSES_PER_REVOLUTION * GEAR_RATIO * period);
    accumu_count = 0;
    pcnt_counter_clear(pcnt_unit);
}