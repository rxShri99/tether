#include "hub/hub.h"
#include "config/device_config.h"
#include "networking/peer_manager.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

namespace tether {

#if TETHER_ROLE == TETHER_ROLE_HUB

static void dashboardTask(void *)
{
    while (true) {
        PeerState peers[8];
        int n = peersList(peers, 8);
        printf("\n== TETHER HUB ==  devices: %d\n", n);
        for (int i = 0; i < n; i++) {
            printf("  %-10s %-8s rssi=%.1f loss=%.0f%%\n",
                   deviceName(peers[i].id),
                   peers[i].online ? "ONLINE" : "OFFLINE",
                   peers[i].smoothedRssi, peers[i].packetLossPct());
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

bool hubInit()
{
    xTaskCreate(dashboardTask, "hub_dash", 4096, nullptr, 3, nullptr);
    ESP_LOGI("hub", "serial dashboard running (AMOLED UI: Phase 7)");
    return true;
}

#else

bool hubInit() { return true; }

#endif

} // namespace tether
