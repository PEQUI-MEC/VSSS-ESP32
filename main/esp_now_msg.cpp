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

static QueueHandle_t packet_queue;

void received_callback(const esp_now_recv_info *info, const uint8_t *data, int len) {
    MessagePacket packet;

    if (info->src_addr == NULL || data == NULL || len < 2) {
        // ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }
    
    // only accept messages from RADIO_MAC
    if (std::equal(info->src_addr, info->src_addr + ESP_NOW_ETH_ALEN, RADIO_MAC.begin()) == false) {
        return;
    }

    char id = (char) data[0];
    if (id != ROBOT_ID && id != 'X') {
        return;
    }

    std::copy(info->src_addr, info->src_addr + ESP_NOW_ETH_ALEN, packet.mac_addr.begin());
    std::copy(data + 2, data + len, packet.data.begin());
    packet.data_len = len - 2;

    if (xQueueSend(packet_queue, &packet, ESPNOW_MAXDELAY) != pdTRUE) {
        // ESP_LOGW(TAG, "Send receive queue fail");
    }
}

void send_msg(std::array<uint8_t, ESP_NOW_ETH_ALEN>& mac, std::array<uint8_t, MAX_RECEIVE_DATA>& data, int len) {
    ESP_ERROR_CHECK(esp_now_send(mac.data(), data.data(), len));
}

void send_string_msg(std::array<uint8_t, ESP_NOW_ETH_ALEN>& mac, const std::string& data) {
    std::array<uint8_t, MAX_RECEIVE_DATA> data_array;
    std::copy(data.begin(), data.end(), data_array.begin());
    send_msg(mac, data_array, data.length());
}

void read_msg_queue(MessagePacket& packet) {
    if (xQueueReceive(packet_queue, &packet, portMAX_DELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Receive queue fail");
    }
}

/* WiFi should start before using ESPNOW */
void setup_wifi() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE));
    ESP_ERROR_CHECK(esp_wifi_start());
}

void add_peer(std::array<uint8_t, ESP_NOW_ETH_ALEN>& mac) {
    if (esp_now_is_peer_exist(mac.data()) == false) {
        esp_now_peer_info_t peer;
        memset(&peer, 0, sizeof(esp_now_peer_info_t));
        peer.channel = ESPNOW_CHANNEL;
        peer.ifidx = ESPNOW_WIFI_IF;
        peer.encrypt = false;
        memcpy(peer.lmk, ESPNOW_LMK, ESP_NOW_KEY_LEN);
        memcpy(peer.peer_addr, mac.data(), ESP_NOW_ETH_ALEN);
        ESP_ERROR_CHECK(esp_now_add_peer(&peer));
    }
}

void setup_espnow() {
    packet_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(MessagePacket));

    if (packet_queue == NULL) {
        ESP_LOGE(TAG, "Create mutex fail");
        return;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK(esp_now_init());
    // ESP_ERROR_CHECK( esp_now_register_send_cb(example_espnow_send_cb) );
    ESP_ERROR_CHECK(esp_now_register_recv_cb(received_callback));
    /* Set primary master key. */
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *) ESPNOW_PMK));
    add_peer(RADIO_MAC);
}