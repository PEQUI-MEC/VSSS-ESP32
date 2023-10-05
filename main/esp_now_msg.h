#ifndef ESPNOW_H
#define ESPNOW_H

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
#include <array>
#include <string>

#define ESPNOW_MAXDELAY 512
#define MAX_RECEIVE_DATA 250
#define ESPNOW_QUEUE_SIZE 25
#define ESPNOW_CHANNEL 1
#define ESPNOW_PMK "pmk1234567890123"
#define ESPNOW_LMK "lmk1234567890123"
#define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_IF WIFI_IF_AP

static const char *TAG = "VSSS-ESP32";

// static std::array<uint8_t, ESP_NOW_ETH_ALEN> BROADCAST_MAC = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
// ec:62:60:9d:8a:e8
static std::array<uint8_t, ESP_NOW_ETH_ALEN> RADIO_MAC = { 0xEC, 0x62, 0x60, 0x9D, 0x8A, 0xE9 };

typedef struct {
    std::array<uint8_t, ESP_NOW_ETH_ALEN> mac_addr;
    std::array<uint8_t, MAX_RECEIVE_DATA> data;
    int data_len;
} MessagePacket;

void setup_wifi();
void setup_espnow();
void add_peer(std::array<uint8_t, ESP_NOW_ETH_ALEN>& mac);
void send_msg(std::array<uint8_t, ESP_NOW_ETH_ALEN>& mac, std::array<uint8_t, MAX_RECEIVE_DATA>& data, int len);
void send_string_msg(std::array<uint8_t, ESP_NOW_ETH_ALEN>& mac, std::string& data);
void read_msg_queue(MessagePacket& packet);

#endif