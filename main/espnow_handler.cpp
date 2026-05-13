#include "espnow_handler.hpp"
#include "esp_now.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include <cstring>

static EspNowHandler* instance = nullptr;

static void recv_cb(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len) {
    if (instance) {
        instance->onReceive(esp_now_info->src_addr, data, data_len);
    }
}

void EspNowHandler::init() {
    instance = this;
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    esp_netif_init();
    esp_event_loop_create_default();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

    esp_now_init();
    esp_now_register_recv_cb(recv_cb);

    esp_now_peer_info_t peerInfo = {};
    peerInfo.channel = 1;
    peerInfo.encrypt = false;
    peerInfo.ifidx = WIFI_IF_STA;
    memset(peerInfo.peer_addr, 0xFF, 6);
    esp_now_add_peer(&peerInfo);
}

void EspNowHandler::sendDiscover() {
    uint8_t payload[] = "pyCAR_DISCOVER";
    uint8_t broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_send(broadcast, payload, sizeof(payload)-1);
}

void EspNowHandler::sendGamepad(uint8_t lx, uint8_t ly, uint8_t rx, uint8_t ry, uint8_t buttons) {
    uint8_t payload[6] = {67, lx, ly, rx, ry, buttons};
    const uint8_t bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_send(peer_connected ? peer_mac : bcast, payload, 6);
}

void EspNowHandler::onReceive(const uint8_t *mac, const uint8_t *data, int data_len) {
    if (data_len == 9 && memcmp(data, "pyCAR_ACK", 9) == 0) {
        memcpy(peer_mac, mac, 6);
        peer_connected = true;
        esp_now_peer_info_t peerInfo = {};
        peerInfo.channel = 1;
        peerInfo.encrypt = false;
        peerInfo.ifidx = WIFI_IF_STA;
        memcpy(peerInfo.peer_addr, peer_mac, 6);
        if (!esp_now_is_peer_exist(peer_mac)) {
            esp_now_add_peer(&peerInfo);
        }
    } else if (data_len > 0 && data[0] == 'D' && data[1] == ':') {
        std::string msg((char*)data, data_len);
        CarStatus status = {};
        
        size_t d_pos = msg.find("D:");
        if (d_pos != std::string::npos) {
            status.distance = std::stof(msg.substr(d_pos + 2));
        }
        
        size_t l_pos = msg.find("L:");
        if (l_pos != std::string::npos) {
            status.line_follower = (msg[l_pos + 2] == '1');
        }
        
        size_t x_pos = msg.find("X:");
        if (x_pos != std::string::npos) {
            status.sync_state = (msg[x_pos + 2] == '1');
        }
        
        if (status_cb) {
            status_cb(status);
        }
    }
}
