#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_now.h"
#include "esp_crc.h"

#include "esp_now_msg.h"
#include "motor_control.h"
#include "encoder.h"
#include "imu.h"
#include "driver/timer.h"
#include "freertos/timers.h"

#include "math.h"

// static xQueueHandle encoder_queue;

// struct EncoderPacket {
//     int count_1;
//     int count_2;
// };

// void encoder_velocity_task(void * args) {
//     while (true) {
//         EncoderPacket packet;
//         read_encoder_queue(packet);
//         // log bytes received
//         std::string msg = "encoder_1: " + std::to_string(encoder_1.get_velocity()) + " encoder_2: " + std::to_string(encoder_2.get_velocity());
//         send_string_msg(BROADCAST_MAC, msg);
//     }
// }

//void send_back_task(void * args) {
//    while (true) {
//        MessagePacket packet;
//        read_msg_queue(packet);
//        // log bytes received
//        packet.data[packet.data_len] = '\0';
//        printf("%s\n", std::string(packet.data.begin(), packet.data.begin()+packet.data_len).c_str());
//        add_peer(packet.mac_addr);
//        send_msg(packet.mac_addr, packet.data, packet.data_len);
//    }
//}


void flash_init() {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

// #define MOTOR_CONTROL_TIMER_ID           TIMER_0
// #define MOTOR_CONTROL_TIMER_GROUP        TIMER_GROUP_1
// #define MOTOR_CTRL_TIMER_DIVIDER         (16)  //  Hardware timer clock divider
// #define MOTOR_CTRL_TIMER_SCALE           (TIMER_BASE_CLK / MOTOR_CTRL_TIMER_DIVIDER)  // convert counter value to seconds

// struct Encoders {
//     Encoder& left;
//     Encoder& right;
//     int ctrl_period_ms;
// };

// static bool timer_callback(void * arg) {
//     Encoders * encoders = (Encoders *) arg;
//     encoders->left.update_velocity(encoders->ctrl_period_ms);
//     encoders->right.update_velocity(encoders->ctrl_period_ms);
//     return false;
// }

// void configure_timer(Encoders& encoders) {
//     // configure timer
//     timer_config_t config = {
//         .alarm_en = TIMER_ALARM_EN,
//         .counter_en = TIMER_PAUSE,
//         .intr_type = TIMER_INTR_LEVEL,
//         .counter_dir = TIMER_COUNT_UP,
//         .auto_reload = TIMER_AUTORELOAD_EN,
//         .divider = MOTOR_CTRL_TIMER_DIVIDER,
//     }; // default clock source is APB

//     timer_init(MOTOR_CONTROL_TIMER_GROUP, MOTOR_CONTROL_TIMER_ID, &config);
//     timer_set_counter_value(MOTOR_CONTROL_TIMER_GROUP, MOTOR_CONTROL_TIMER_ID, 0);
//     timer_set_alarm_value(MOTOR_CONTROL_TIMER_GROUP, MOTOR_CONTROL_TIMER_ID, encoders.ctrl_period_ms * MOTOR_CTRL_TIMER_SCALE / 1000);
//     timer_enable_intr(MOTOR_CONTROL_TIMER_GROUP, MOTOR_CONTROL_TIMER_ID);
//     timer_isr_callback_add(MOTOR_CONTROL_TIMER_GROUP, MOTOR_CONTROL_TIMER_ID, timer_callback, &encoders, 0);
//     timer_start(MOTOR_CONTROL_TIMER_GROUP, MOTOR_CONTROL_TIMER_ID);
// }

Encoder * encoder_1_;
Encoder * encoder_2_;
MotorControl * motor_control_1_;
MotorControl * motor_control_2_;

float get_token(std::string &msg)
{
    // Find <;> or <end of string> position
    size_t delim_pos = msg.find(';');
    // Get token from substring
    std:string token = msg.substr(0,delim_pos);
    // Update msg to delete consumed token
    msg = msg.substr(delim_pos+1);

    // Convert token to Float
    float value = std::stof(token);
    return value;
}

void parse_message(void * args) {
    while (true) {
        MessagePacket packet;
        read_msg_queue(packet);

        // Set last byte to 0 to be sure it is a null terminated string
        packet.data[packet.data_len] = '\0';
        // Create C++ string from byte array
        std::string text(packet.data.begin(), packet.data.begin() + packet.data_len);

        if(packet.data[0] == 'W')
        {
            // TODO: think of a better way to read the first character
            std::string text(packet.data.begin()+1, packet.data.begin() + packet.data_len);
            float left = get_token(text);
            float right = get_token(text);
            motor_control_1_->set_pid_target_velocity(left);
            motor_control_2_->set_pid_target_velocity(right);
        }
    }
}

int64_t last_time_us = 0;
void timer_callback(TimerHandle_t xTimer) {
    // encoder_1_->update_velocity(10);
    // encoder_2_->update_velocity(10);
    // motor_control_1_->update_pid(encoder_1_->get_velocity());
    // motor_control_2_->update_pid(encoder_2_->get_velocity());
    // int count = encoder_1_->get_count();
    // float angle = (float(count) *360) / (Encoder::PULSES_PER_REVOLUTION * Encoder::GEAR_RATIO);
    // std::string msg = "angle: " + std::to_string(angle);
    // int count2 = encoder_2_->get_count();
    // float angle2 = (float(count2) *360) / (Encoder::PULSES_PER_REVOLUTION * Encoder::GEAR_RATIO);
    // msg += " angle2: " + std::to_string(angle2);
    // send_string_msg(BROADCAST_MAC, msg);
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
    float dt_s = (time_us - last_time_us) / 1000000.0;
    last_time_us = time_us;
    encoder_1_->update_velocity(dt_s);
    encoder_2_->update_velocity(dt_s);
    motor_control_1_->update_pid(encoder_1_->get_velocity());
    motor_control_2_->update_pid(encoder_2_->get_velocity());
}

extern "C" void app_main() {
    flash_init();
    setup_wifi();
    setup_espnow();
    //xTaskCreate(send_back_task, "send_back_task", 20480, NULL, 4, NULL);
    xTaskCreate(parse_message, "parse_message", 20480, NULL, 4, NULL);

    MotorControl motor_control_1(MCPWM_UNIT_0, 32, 33);
    motor_control_1.set_duty_cycle(0);

    MotorControl motor_control_2(MCPWM_UNIT_1, 19, 18);
    motor_control_2.set_duty_cycle(0);

    Encoder encoder_1(PCNT_UNIT_0, 25, 26);
    Encoder encoder_2(PCNT_UNIT_1, 5, 17);

    encoder_1_ = &encoder_1;
    encoder_2_ = &encoder_2;
    motor_control_1_ = &motor_control_1;
    motor_control_2_ = &motor_control_2;

    IMU imu(I2C_NUM_0, 21, 22);

    // Encoders encoders = {encoder_1, encoder_2, 100};

    // packet_queue = xQueueCreate(10, sizeof(EncoderPacket));

    // configure_timer(encoders);

    TimerHandle_t timer = xTimerCreate("EncoderTimer", pdMS_TO_TICKS(10), pdTRUE, ( void * ) 1, &timer_callback);
    if( xTimerStart(timer, 10 ) != pdPASS ) {
        printf("Timer start error");
    }

    int count = 0;

    // correçao para os motores "ruins"
    // float target = 0.5;
    // float correction = 0.88;
    
    while (true) {
        // if (count % 50 == 0) {
        //     // motor_control_1.set_duty_cycle(-20);
        //     // motor_control_2.set_duty_cycle(20);
        //     motor_control_1.set_pid_target_velocity(-target);
        //     motor_control_2.set_pid_target_velocity(target);
        // } else if (count % 50 == 25) {
        //     // motor_control_1.set_duty_cycle(20);
        //     // motor_control_2.set_duty_cycle(-20);
        //     motor_control_1.set_pid_target_velocity(target);
        //     motor_control_2.set_pid_target_velocity(-target);
        // }

        // motor_control_1.set_pid_target_velocity(2);
        // motor_control_2.set_pid_target_velocity(2);
        std::string msg = "encoder_1: " + std::to_string(encoder_1_->get_velocity()) + " encoder_2: " + std::to_string(encoder_2_->get_velocity());
        // std::string msg = "encoder_1: " + std::to_string(motor_control_1.debug_var) + " encoder_2: " + std::to_string(motor_control_2.debug_var);
        send_string_msg(BROADCAST_MAC, msg);

        // std::string msg = "encoder_1: " + std::to_string(encoder_1.get_count()) + " encoder_2: " + std::to_string(encoder_2.get_count());
        // send_string_msg(BROADCAST_MAC, msg);
        
        // std::string msg = "encoder_1: " + std::to_string(encoder_1.get_velocity()) + " encoder_2: " + std::to_string(encoder_2.get_velocity());
        // send_string_msg(BROADCAST_MAC, msg);
        // ImuData imu_data = imu.get_data();
        // send gyro components
        // std::string gyro_msg = std::to_string(imu_data.gyro[0]) + "\t" + std::to_string(imu_data.gyro[1]) + "\t" + std::to_string(imu_data.gyro[2]);
        // send_string_msg(BROADCAST_MAC, gyro_msg);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        count++;
    }
}
