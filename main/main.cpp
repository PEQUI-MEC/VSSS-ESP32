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

#include "UKF.h"
#include "Control.h"

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

Encoder * right_encoder_;
Encoder * left_encoder_;
MotorControl * right_motor_control_;
MotorControl * left_motor_control_;
IMU * imu_;
UKF * ukf_;
Control * control_;

float get_token(std::string &msg)
{
    // Find <;> or <end of string> position
    size_t delim_pos = msg.find(';');
    // Get token from substring
    std::string token = msg.substr(0,delim_pos);
    // Update msg to delete consumed token
    msg = msg.substr(delim_pos+1);

    // Convert token to Float
    float value = std::stof(token);
    return value;
}

static QueueHandle_t vision_queue;
static QueueHandle_t cmd_queue;

void parse_message(void * args) {
    while (true) {
        MessagePacket packet;
        read_msg_queue(packet);

        // Set last byte to 0 to be sure it is a null terminated string
        packet.data[packet.data_len] = '\0';
        // Create C++ string from byte array
        std::string text(packet.data.begin(), packet.data.begin() + packet.data_len);

        // Debug print to console
        // printf("Packet = %s\n",text.c_str());

        // TODO: think of a better way to read the first character
        std::string text_data(packet.data.begin()+1, packet.data.begin() + packet.data_len);
        if(packet.data[0] == 'W') {
            float left = get_token(text_data);
            float right = get_token(text_data);
            Target target = {
                .command = ControlType::WHEEL_VELOCITY_CONTROL,
                .wheel_velocity = {left, right}
            };
            xQueueOverwrite(cmd_queue, &target);
        } else if (packet.data[0] == 'O') {
            float orientation = wrap(get_token(text_data) * M_PI / 180);
            float velocity = get_token(text_data);
            Target target = {
                .command = ControlType::ORIENTATION_CONTROL,
                .theta = orientation,
                .velocity = velocity
            };
            xQueueOverwrite(cmd_queue, &target);
        } else if (packet.data[0] == 'P') {
            float x = get_token(text_data)/100.0;
            float y = get_token(text_data)/100.0;
            float velocity = get_token(text_data);
            Target target = {
                .command = ControlType::POSITION_CONTROL,
                .x = x,
                .y = y,
                .velocity = velocity
            };
            xQueueOverwrite(cmd_queue, &target);
        } else if (packet.data[0] == 'V') {
            float theta = wrap(get_token(text_data) * M_PI / 180);
            float x = std::cos(theta) * 10;
            float y = std::sin(theta) * 10;
            float velocity = get_token(text_data);
            Target target = {
                .command = ControlType::VECTOR_CONTROL,
                .x = x,
                .y = y,
                .velocity = velocity
            };
            xQueueOverwrite(cmd_queue, &target);
        } else if (packet.data[0] == 'U') {
            float x = get_token(text_data)/100.0;
            float y = get_token(text_data)/100.0;
            float x_ref = get_token(text_data)/100.0;
            float y_ref = get_token(text_data)/100.0;
            float n = get_token(text_data);
            float velocity = get_token(text_data);
            Target target = {
                .command = ControlType::UVF_CONTROL,
                .x = x,
                .y = y,
                .velocity = velocity,
                .ref_x = x_ref,
                .ref_y = y_ref,
                .uvf_n = n
            };
            xQueueOverwrite(cmd_queue, &target);
        } else if (packet.data[0] == 'E') {
            float x = get_token(text_data)/100.0;
            float y = get_token(text_data)/100.0;
            float theta = wrap(get_token(text_data) * M_PI / 180);
            T::VisionVec vision_data = {x, y, theta};
            xQueueOverwrite(vision_queue, &vision_data);
        }
        vTaskDelay(1);
    }
}

// int64_t last_time_us = 0;
// void timer_callback(TimerHandle_t xTimer) {
//     // encoder_1_->update_velocity(10);
//     // encoder_2_->update_velocity(10);
//     // motor_control_1_->update_pid(encoder_1_->get_velocity());
//     // motor_control_2_->update_pid(encoder_2_->get_velocity());
//     // int count = encoder_1_->get_count();
//     // float angle = (float(count) *360) / (Encoder::PULSES_PER_REVOLUTION * Encoder::GEAR_RATIO);
//     // std::string msg = "angle: " + std::to_string(angle);
//     // int count2 = encoder_2_->get_count();
//     // float angle2 = (float(count2) *360) / (Encoder::PULSES_PER_REVOLUTION * Encoder::GEAR_RATIO);
//     // msg += " angle2: " + std::to_string(angle2);
//     // send_string_msg(BROADCAST_MAC, msg);
//     struct timeval tv_now;
//     gettimeofday(&tv_now, NULL);
//     int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
//     float dt_s = (time_us - last_time_us) / 1000000.0;
//     last_time_us = time_us;
//     encoder_1_->update_velocity(dt_s);
//     encoder_2_->update_velocity(dt_s);
//     motor_control_1_->update_pid(encoder_1_->get_velocity());
//     motor_control_2_->update_pid(encoder_2_->get_velocity());
// }

// float time_now() {
//     struct timeval tv_now;
//     gettimeofday(&tv_now, NULL);
//     int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
//     return time_us / 1000000.0;
// }

// once every tick
static QueueHandle_t imu_queue;
void imu_read_task(void * args) {
    const TickType_t period = 1;
    TickType_t last_wake_time = xTaskGetTickCount();
    while (true) {
        ImuData imu_data = imu_->get_data();
        xQueueOverwrite(imu_queue, &imu_data);
        vTaskDelayUntil(&last_wake_time, period);
    }
}

static QueueHandle_t ukf_queue;

float last_time = 0;
void control_task(void * args) {
    last_time = time_now();
    const TickType_t period = 10;
    TickType_t last_wake_time = xTaskGetTickCount();
    while (true) {
        float now = time_now();

        float dt = now - last_time;
        
        left_encoder_->update_velocity(dt);
        right_encoder_->update_velocity(dt);

        Pose pose;
        xQueueReceive(ukf_queue, &pose, portMAX_DELAY);

        Target target;
        if (xQueueReceive(cmd_queue, &target, 0) == pdTRUE) {
            control_->target = target;
        }

        WheelVelocity wheel_vel = control_->control_loop(pose);
        left_motor_control_->set_pid_target_velocity(wheel_vel.left);
        right_motor_control_->set_pid_target_velocity(wheel_vel.right);

        left_motor_control_->update_pid(left_encoder_->get_velocity());
        right_motor_control_->update_pid(right_encoder_->get_velocity());

        last_time = now;

        vTaskDelayUntil(&last_wake_time, period);
    }
}


float calibrate_gyro_bias() {
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    int SAMPLES = 1000;
    float sum = 0;
    float now = time_now();
    for (int i = 0; i < SAMPLES; i++) {
        ImuData imu_data;
        if (xQueueReceive(imu_queue, &imu_data, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        sum += imu_data.gyro[2];
        vTaskDelay(1);
    }
    return sum / SAMPLES;
}

float gyro_scale = 1.15;
float ukf_dt = 0;
void ukf_task(void * args) {
    float gyro_bias = calibrate_gyro_bias();

    // ImuData prev_imu_data = imu_->get_data();
    ImuData prev_imu_data = {0, 0, 0};

    const TickType_t period = 1;
    TickType_t last_wake_time = xTaskGetTickCount();

    float prev_wheel_lin_vel = 0;
    float t_prev = time_now();
    while (true) {
        ImuData imu_data;
        if (xQueueReceive(imu_queue, &imu_data, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        imu_data.gyro[2] -= gyro_bias;
        imu_data.gyro[2] *= gyro_scale;

        float t = time_now();
        float dt = t - t_prev;
        ukf_dt = dt;
    
        float left_wheel_vel = left_encoder_->get_velocity();
        float right_wheel_vel = right_encoder_->get_velocity();

        // float t1 = time_now();

        float wheel_lin_vel = (left_wheel_vel + right_wheel_vel) / 2;
        float wheel_accel = (wheel_lin_vel - prev_wheel_lin_vel) / dt;
        float gyro_ang_accel = (imu_data.gyro[2] - prev_imu_data.gyro[2]) / dt;

        T::ControlVec controls {
            wheel_accel,
            gyro_ang_accel
        };

        ukf_->predict(controls, dt);

        // update on vision data if available
        T::VisionVec vision_data;
        if (xQueueReceive(vision_queue, &vision_data, 0) == pdTRUE) {
            ukf_->update_on_vision_data(vision_data);
        } else {
            T::SensorVec sensor_data {
                imu_data.gyro[2],
                left_wheel_vel,
                right_wheel_vel
            };
            ukf_->update_on_sensor_data(sensor_data);
        }

        Pose pose(ukf_->x);
        xQueueOverwrite(ukf_queue, &pose);

        vTaskDelayUntil(&last_wake_time, period);

        // float t2 = time_now();
        // ukf_dt = t2 - t1;

        prev_imu_data = imu_data;
        prev_wheel_lin_vel = wheel_lin_vel;
        t_prev = t;
    }
}


extern "C" void app_main() {
    flash_init();
    setup_wifi();
    setup_espnow();
    //xTaskCreate(send_back_task, "send_back_task", 20480, NULL, 4, NULL);
    xTaskCreatePinnedToCore(parse_message, "parse_message", 20480, NULL, 4, NULL, 0);

    imu_queue = xQueueCreate(1, sizeof(ImuData));
    ukf_queue = xQueueCreate(1, sizeof(Pose));
    cmd_queue = xQueueCreate(1, sizeof(Target));
    vision_queue = xQueueCreate(1, sizeof(T::VisionVec));

    MotorControl right_motor_control(MCPWM_UNIT_0, 32, 33, 1);
    right_motor_control.set_duty_cycle(0);

    MotorControl left_motor_control(MCPWM_UNIT_1, 19, 18, -1);
    left_motor_control.set_duty_cycle(0);

    Encoder right_encoder(PCNT_UNIT_0, 25, 26, -1);
    Encoder left_encoder(PCNT_UNIT_1, 5, 17, 1);

    right_encoder_ = &right_encoder;
    left_encoder_ = &left_encoder;
    right_motor_control_ = &right_motor_control;
    left_motor_control_ = &left_motor_control;

    IMU imu(I2C_NUM_0, 21, 22);
    imu_ = &imu;

    Control control;
    control_ = &control;

    // Encoders encoders = {encoder_1, encoder_2, 100};

    // packet_queue = xQueueCreate(10, sizeof(EncoderPacket));

    // configure_timer(encoders);

    // TimerHandle_t timer = xTimerCreate("EncoderTimer", pdMS_TO_TICKS(10), pdTRUE, ( void * ) 1, &timer_callback);
    // if( xTimerStart(timer, 10 ) != pdPASS ) {
    //     printf("Timer start error");
    // }

    int count = 0;

    UKF ukf;
    ukf_ = &ukf;
    // xTaskCreate(ukf_task, "ukf_task", 20480, NULL, 4, NULL);
    // pinning task to core 1
    xTaskCreatePinnedToCore(ukf_task, "ukf_task", 20480, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(imu_read_task, "imu_read_task", 20480, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(control_task, "control_task", 20480, NULL, 4, NULL, 0);

    std::string msg = "Starting";
    send_string_msg(RADIO_MAC, msg);

    // correçao para os motores "ruins"
    // float target = 0.5;
    // float correction = 0.88;

    auto orientation_control = [&](float theta, float velocity) {
        Target target = {
            .command = ControlType::ORIENTATION_CONTROL,
            .theta = theta,
            .velocity = velocity
        };
        xQueueOverwrite(cmd_queue, &target);
    };

    // wait until first ukf update
    Pose pose;
    xQueueReceive(ukf_queue, &pose, portMAX_DELAY);
    int delay = 300;
    vTaskDelay(500 / portTICK_PERIOD_MS);
    orientation_control(45, 0.8);
    vTaskDelay(delay / portTICK_PERIOD_MS);
    orientation_control(0, 0.8);
    vTaskDelay(delay / portTICK_PERIOD_MS);
    orientation_control(-45, 0.8);
    vTaskDelay(delay / portTICK_PERIOD_MS);
    orientation_control(0, 0.8);
    
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
        //std::string msg = "encoder_1: " + std::to_string(encoder_1_->get_velocity()) + " encoder_2: " + std::to_string(encoder_2_->get_velocity());
        // std::string msg = "encoder_1: " + std::to_string(motor_control_1.debug_var) + " encoder_2: " + std::to_string(motor_control_2.debug_var);
        //send_string_msg(BROADCAST_MAC, msg);

        // std::string msg = "encoder1: " + std::to_string(encoder_1.get_count()) + " encoder2: " + std::to_string(encoder_2.get_count());
        // send_string_msg(BROADCAST_MAC, msg);

        // std::string msg = "testmessage";
        // send_string_msg(BROADCAST_MAC, msg);

        // send ukf ang velocity and dt
        // std::string msg = "dt: " + std::to_string(ukf_dt) + " ang_vel: " + std::to_string(ukf_->x[4]);
        // float theta_deg = ukf_->x[2] * 180 / M_PI;
        // std::string msg = "x: " + std::to_string(ukf_->x[0]) + "\t y: " + std::to_string(ukf_->x[1]) + "\t vl: " + std::to_string(left_encoder_->get_velocity()) + "\t vr: " + std::to_string(right_encoder_->get_velocity());
        // std::string msg = "x: " + std::to_string(ukf_->x[0]) + "\t y: " + std::to_string(ukf_->x[1]);
        // send_string_msg(BROADCAST_MAC, msg);

        // std::string msg = "left_vel: " + std::to_string(left_encoder_->get_velocity()) + " right_vel: " + std::to_string(right_encoder_->get_velocity());
        // send_string_msg(BROADCAST_MAC, msg);
        
        // std::string msg = "encoder_1: " + std::to_string(encoder_1.get_velocity()) + " encoder_2: " + std::to_string(encoder_2.get_velocity());
        // send_string_msg(BROADCAST_MAC, msg);
        // ImuData imu_data = imu.get_data();
        // send gyro components
        // std::string gyro_msg = std::to_string(imu_data.gyro[0]) + "\t" + std::to_string(imu_data.gyro[1]) + "\t" + std::to_string(imu_data.gyro[2]);
        // send_string_msg(BROADCAST_MAC, gyro_msg);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        count++;
    }
}
