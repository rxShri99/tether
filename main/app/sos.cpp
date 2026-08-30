#include "app/sos.h"
#include "config/device_config.h"
#include "networking/protocol.h"
#include "networking/espnow_transport.h"
#include "audio/audio.h"

#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

namespace tether {

static const char *TAG = "sos";

#define SOS_BUTTON_GPIO GPIO_NUM_0 /* BOOT side button, active low */
constexpr int SOS_BURST_COUNT = 6;
constexpr uint32_t SOS_BURST_GAP_MS = 400;

static bool s_wasPressed = false;
static bool s_longFired = false;
static uint32_t s_pressStartMs = 0;
static int s_pendingSends = 0;
static uint32_t s_lastSendMs = 0;
static volatile uint32_t s_activeUntilMs = 0;

void sosInit()
{
    gpio_config_t cfg = {};
    cfg.mode = GPIO_MODE_INPUT;
    cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    cfg.pin_bit_mask = 1ULL << SOS_BUTTON_GPIO;
    ESP_ERROR_CHECK(gpio_config(&cfg));
    ESP_LOGI(TAG, "armed: hold BOOT %.1fs to send SOS to the hub", SOS_LONGPRESS_MS / 1000.0f);
}

void sosTick(uint32_t nowMs)
{
    bool pressed = gpio_get_level(SOS_BUTTON_GPIO) == 0;
    if (pressed && !s_wasPressed) {
        s_pressStartMs = nowMs;
        s_longFired = false;
    }
    if (pressed && !s_longFired && nowMs - s_pressStartMs >= SOS_LONGPRESS_MS) {
        s_longFired = true;
        ESP_LOGW(TAG, "SOS! tone + notifying hub");
        audioPlay(TONE_SOS);
        s_pendingSends = SOS_BURST_COUNT;
        s_lastSendMs = 0;
        s_activeUntilMs = nowMs + SOS_TIMEOUT_MS;
    }
    s_wasPressed = pressed;

    if (s_pendingSends > 0 && nowMs - s_lastSendMs >= SOS_BURST_GAP_MS) {
        s_lastSendMs = nowMs;
        s_pendingSends--;
        TetherPacket pkt = {};
        pkt.type = MSG_SOS;
        pkt.destinationId = BROADCAST_ID;
        pkt.ttl = DEFAULT_TTL;
        pkt.payload.sos.active = 1;
        transportSend(pkt);
    }
}

void sosOnPacket(const TetherPacket &pkt, uint32_t nowMs)
{
    if (pkt.type != MSG_SOS || !pkt.payload.sos.active) return;
    if (pkt.sourceId == HUB_ID) return; /* only wearables raise SOS */
    bool wasActive = sosActive();
    s_activeUntilMs = nowMs + SOS_TIMEOUT_MS;
    if (!wasActive) {
        ESP_LOGW(TAG, "SOS received from %s", deviceName(pkt.sourceId));
        audioPlay(TONE_SOS);
    }
}

bool sosActive()
{
    return (uint32_t)(esp_timer_get_time() / 1000) < s_activeUntilMs;
}

} // namespace tether
