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

void send_back_task(void * args) {
    while (true) {
        MessagePacket packet;
        read_msg_queue(packet);
        // log bytes received
        add_peer(packet.mac_addr);
        send_msg(packet.mac_addr, packet.data, packet.data_len);
    }
}

void flash_init() {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

extern "C" void app_main() {
    flash_init();
    setup_wifi();
    setup_espnow();
    xTaskCreate(send_back_task, "send_back_task", 20480, NULL, 4, NULL);

    MotorControl motor_control_1(MCPWM_UNIT_0, 32, 33);
    // motor_control_1.set_duty_cycle(40);

    MotorControl motor_control_2(MCPWM_UNIT_1, 19, 18);
    // motor_control_2.set_duty_cycle(40);

    Encoder encoder_1(PCNT_UNIT_0, 25, 26);
    Encoder encoder_2(PCNT_UNIT_1, 5, 17);

    int count = 0;

    while (true) {
        if (count % 50 == 0) {
            motor_control_1.set_duty_cycle(-40);
            motor_control_2.set_duty_cycle(40);
        } else if (count % 50 == 25) {
            motor_control_1.set_duty_cycle(40);
            motor_control_2.set_duty_cycle(-40);
        }
        std::string msg = "encoder_1: " + std::to_string(encoder_1.get_count()) + " encoder_2: " + std::to_string(encoder_2.get_count());
        send_string_msg(BROADCAST_MAC, msg);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        count++;
    }
}
