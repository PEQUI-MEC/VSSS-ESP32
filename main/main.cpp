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
    motor_control_1.set_duty_cycle(40);

    MotorControl motor_control_2(MCPWM_UNIT_1, 19, 18);
    motor_control_2.set_duty_cycle(40);

    while (true) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
