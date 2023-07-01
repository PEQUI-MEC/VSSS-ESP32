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

// typedef struct {
//     std::array<uint8_t, ESP_NOW_ETH_ALEN> mac_addr;
//     std::array<uint8_t, MAX_RECEIVE_DATA> data;
//     int data_len;
// } MessagePacket;

void print_received(void * args) {
    while (true) {
        MessagePacket packet;
        read_msg_queue(packet);
        // log bytes received
        add_peer(packet.mac_addr);
        // data is a zero-terminated string
        packet.data[packet.data_len] = '\0';
        // to string, ends at data_len
        std::string data_str(packet.data.begin(), packet.data.begin() + packet.data_len);
        // log data, no mac
        // check if data_str ends with \n, print with \n if not
        if (data_str[data_str.length() - 1] == '\n') {
            printf("%s", data_str.c_str());
        } else {
            printf("%s\n", data_str.c_str());
        }
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
    xTaskCreate(print_received, "print_received", 20480, NULL, 4, NULL);

    while (true) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
