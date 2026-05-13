#pragma once

#include <stdint.h>
#include <string>
#include <functional>

struct CarStatus {
    float distance;
    bool line_follower;
    bool sync_state;
};

class EspNowHandler {
public:
    void init();
    void sendDiscover();
    void sendGamepad(uint8_t lx, uint8_t ly, uint8_t rx, uint8_t ry, uint8_t buttons);
    bool isConnected() const { return peer_connected; }

    void setStatusCallback(std::function<void(const CarStatus&)> cb) {
        status_cb = cb;
    }

    void onReceive(const uint8_t *mac, const uint8_t *data, int data_len);

private:
    bool peer_connected = false;
    uint8_t peer_mac[6];
    std::function<void(const CarStatus&)> status_cb;
};
