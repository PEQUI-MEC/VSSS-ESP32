#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "string.h"
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

#define BUF_SIZE (1024)


void flash_init() {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

void uart_task(void *arg)
{
    std::string msg;
    uint8_t* data = (uint8_t*) malloc(BUF_SIZE);
    while (1) {
        // Wait for data to be received
        int len = uart_read_bytes(UART_NUM_0, data, BUF_SIZE, 1000 / portTICK_PERIOD_MS);

        if (len > 0) {
            // Process received data
            data[len] = 0; // Null-terminate the received data
            msg = std::string(reinterpret_cast<char*>(data));
            send_string_msg(BROADCAST_MAC ,msg);
        }

        // Send data
        // const char* send_data = "Hello, world!";
        // uart_write_bytes(UART_NUM_0, send_data, strlen(send_data));
       // printf("Sent data: %s\n", send_data);



        vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for 1 second
    }
}

extern "C" void app_main() {
    flash_init();
    setup_wifi();
    setup_espnow();

    // Configure UART port
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);

    // Start UART task
    xTaskCreate(uart_task, "uart_task", 4096, NULL, 10, NULL);

}

