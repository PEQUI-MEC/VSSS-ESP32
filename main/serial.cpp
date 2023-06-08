#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "string.h"
#include <string>

#define BUF_SIZE (1024)

std::string msg; 

void uart_task(void *arg)
{
    uint8_t* data = (uint8_t*) malloc(BUF_SIZE);
    while (1) {
        // Wait for data to be received
        int len = uart_read_bytes(UART_NUM_0, data, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (len > 0) {
            // Process received data
            data[len] = 0; // Null-terminate the received data
            msg = std::string(reinterpret_cast<char*>(data));
        }

        // Send data
        const char* send_data = "Hello, world!";
        uart_write_bytes(UART_NUM_0, send_data, strlen(send_data));
       // printf("Sent data: %s\n", send_data);

        vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for 1 second
    }
}

void app_main()
{
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
