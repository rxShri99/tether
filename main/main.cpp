#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "config/device_config.h"
#include "networking/protocol.h"
#include "networking/espnow_transport.h"
#include "networking/peer_manager.h"
#include "sensors/imu.h"
#include "tracking/signal_sweep.h"
#include "audio/audio.h"
#include "ui/ui.h"
#include "hub/hub.h"

static const char *TAG = "tether";

using namespace tether;

static uint32_t nowMs()
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void heartbeatCb(void *)
{
    static uint32_t count = 0;
    TetherPacket pkt = {};
    pkt.destinationId = BROADCAST_ID;
    pkt.ttl = DEFAULT_TTL;
    if (count++ % DISCOVERY_EVERY_N_HEARTBEATS == 0) {
        pkt.type = MSG_DISCOVERY;
        strncpy(pkt.payload.discovery.name, deviceName(DEVICE_ID), sizeof(pkt.payload.discovery.name) - 1);
        pkt.payload.discovery.role = TETHER_ROLE;
    } else {
        pkt.type = MSG_HEARTBEAT;
    }
    transportSend(pkt);
}

extern "C" void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_LOGI(TAG, "== TETHER ==  id=%lu name=%s role=%s",
             (unsigned long)DEVICE_ID, deviceName(DEVICE_ID),
             TETHER_ROLE == TETHER_ROLE_HUB ? "HUB" : "WEARABLE");

    peersInit();

#if TETHER_ROLE == TETHER_ROLE_WEARABLE
    uiInit();
    imuInit(uiI2CBus());
    audioInit();
#else
    hubInit();
#endif

    if (!transportInit()) {
        ESP_LOGE(TAG, "transport init failed");
        return;
    }

    const esp_timer_create_args_t hbArgs = {
        .callback = heartbeatCb,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "heartbeat",
        .skip_unhandled_events = true,
    };
    esp_timer_handle_t hbTimer;
    ESP_ERROR_CHECK(esp_timer_create(&hbArgs, &hbTimer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(hbTimer, HEARTBEAT_PERIOD_MS * 1000));

    /* Main RX/app loop: dequeue transport events, update peer state, dispatch. */
    uint32_t lastLogMs = 0;
    while (true) {
        RxEvent ev;
        if (transportReceive(ev, 50)) {
            if (ev.pkt.version != PROTOCOL_VERSION) continue;
            if (ev.pkt.sourceId == DEVICE_ID) continue;

            bool fresh = peersOnPacket(ev.pkt.sourceId, ev.mac, ev.pkt.sequence, ev.rssi, nowMs());
            if (!fresh) continue; /* duplicate suppression */

#if TETHER_ROLE == TETHER_ROLE_WEARABLE
            /* feed the direction estimator with heading-tagged RSSI */
            if (ev.pkt.sourceId == FRIEND_ID) {
                sweepAddSample(SWEEP_YAW_SIGN * imuGetYawDeg(), ev.rssi, nowMs());
            }
#else
            /* Hub: SOS overlay (Phase 7) and the relay hop (Phase 8). Runs
               after the duplicate check above, so the relay cannot loop. */
            hubOnPacket(ev.pkt, ev.rssi);
#endif

            /* Phase 2 acceptance log: peer / seq / RSSI (rate-limited to 1Hz) */
            if (nowMs() - lastLogMs > 1000) {
                lastLogMs = nowMs();
                PeerState p;
                peersGet(ev.pkt.sourceId, p);
                ESP_LOGI(TAG, "peer=%lu(%s) type=%s seq=%u rssi=%d avg=%.1f loss=%.0f%% -> %s",
                         (unsigned long)ev.pkt.sourceId, deviceName(ev.pkt.sourceId),
                         messageTypeName(ev.pkt.type), ev.pkt.sequence, ev.rssi,
                         p.smoothedRssi, p.packetLossPct(), proximityLabel(p.level));
            }

            /* SOS / pairing / relay handlers arrive in Phases 5-8 */
        }
        peersTick(nowMs());
    }
}
