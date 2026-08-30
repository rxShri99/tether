#include "networking/espnow_transport.h"
#include "config/device_config.h"

#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "esp_log.h"

namespace tether {

static const char *TAG = "transport";
static const uint8_t BCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static QueueHandle_t s_rxQueue = nullptr;
static uint16_t s_seq = 0;

static void recvCb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    /* Runs on the Wi-Fi task: copy out and return immediately. */
    if (!s_rxQueue || len < (int)offsetof(TetherPacket, payload) || len > (int)sizeof(TetherPacket)) {
        return;
    }
    RxEvent ev = {};
    memcpy(&ev.pkt, data, len);
    memcpy(ev.mac, info->src_addr, 6);
    ev.rssi = info->rx_ctrl ? (int8_t)info->rx_ctrl->rssi : 0;
    ev.len = (uint8_t)len;
    if (xQueueSend(s_rxQueue, &ev, 0) != pdTRUE) {
        ESP_LOGW(TAG, "rx queue full, packet dropped");
    }
}

bool transportInit()
{
    s_rxQueue = xQueueCreate(32, sizeof(RxEvent));
    if (!s_rxQueue) return false;

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(recvCb));
    transportAddPeer(BCAST_MAC);

    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    ESP_LOGI(TAG, "ESP-NOW up on channel %d, mac=%02X:%02X:%02X:%02X:%02X:%02X",
             WIFI_CHANNEL, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return true;
}

bool transportAddPeer(const uint8_t mac[6])
{
    if (esp_now_is_peer_exist(mac)) return true;
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = WIFI_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    esp_err_t err = esp_now_add_peer(&peer);
    if (err != ESP_OK) ESP_LOGE(TAG, "add_peer failed: %s", esp_err_to_name(err));
    return err == ESP_OK;
}

bool transportSend(TetherPacket &pkt, const uint8_t *mac)
{
    pkt.version = PROTOCOL_VERSION;
    pkt.sourceId = DEVICE_ID;
    pkt.sequence = s_seq++;
    pkt.uptimeMs = (uint32_t)(esp_timer_get_time() / 1000);
    esp_err_t err = esp_now_send(mac ? mac : BCAST_MAC, (const uint8_t *)&pkt, sizeof(pkt));
    if (err != ESP_OK) ESP_LOGW(TAG, "send failed: %s", esp_err_to_name(err));
    return err == ESP_OK;
}

bool transportForward(const TetherPacket &pkt)
{
    esp_err_t err = esp_now_send(BCAST_MAC, (const uint8_t *)&pkt, sizeof(pkt));
    return err == ESP_OK;
}

bool transportReceive(RxEvent &ev, uint32_t waitMs)
{
    return s_rxQueue && xQueueReceive(s_rxQueue, &ev, pdMS_TO_TICKS(waitMs)) == pdTRUE;
}

} // namespace tether
