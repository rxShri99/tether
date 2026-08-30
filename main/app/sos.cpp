#include "app/sos.h"
#include "config/device_config.h"
#include "networking/espnow_transport.h"
#include "audio/audio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"

namespace tether {

static const char *TAG = "sos";

#define SOS_BUTTON_GPIO GPIO_NUM_0 /* BOOT side button, active low */

static SemaphoreHandle_t s_lock;
static SosState s_state = SOS_IDLE;
static uint32_t s_peerId = 0;
static bool s_delivered = false;

static uint32_t s_lastSendMs = 0;
static uint32_t s_lastRxMs = 0;
static uint32_t s_lastAckMs = 0;
static uint32_t s_lastToneMs = 0;
static int s_pendingClears = 0;
static uint32_t s_lastClearMs = 0;

/* button edge tracking */
static bool s_wasPressed = false;
static bool s_longFired = false;
static uint32_t s_pressStartMs = 0;

static void setState(SosState st, uint32_t peer)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_state = st;
    s_peerId = peer;
    if (st != SOS_OUTGOING) s_delivered = false;
    xSemaphoreGive(s_lock);
}

void sosInit()
{
    s_lock = xSemaphoreCreateMutex();
    gpio_config_t cfg = {};
    cfg.mode = GPIO_MODE_INPUT;
    cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    cfg.pin_bit_mask = 1ULL << SOS_BUTTON_GPIO;
    ESP_ERROR_CHECK(gpio_config(&cfg));
    ESP_LOGI(TAG, "armed: hold BOOT %.1fs to send SOS", SOS_LONGPRESS_MS / 1000.0f);
}

static void sendSos(bool active)
{
    TetherPacket pkt = {};
    pkt.type = active ? MSG_SOS : MSG_SOS_CLEAR;
    pkt.destinationId = BROADCAST_ID; /* friend + hub together */
    pkt.ttl = DEFAULT_TTL;
    pkt.payload.sos.active = active ? 1 : 0;
    transportSend(pkt);
}

static void onLongPress(uint32_t nowMs)
{
    switch (s_state) {
    case SOS_IDLE:
        ESP_LOGW(TAG, "SOS RAISED");
        setState(SOS_OUTGOING, FRIEND_ID);
        s_lastSendMs = 0; /* send on next tick immediately */
        audioPlay(TONE_SOS);
        break;
    case SOS_OUTGOING:
        ESP_LOGW(TAG, "SOS cancelled by user");
        setState(SOS_IDLE, 0);
        s_pendingClears = 3; /* burst the clear for reliability */
        s_lastClearMs = 0;
        break;
    case SOS_INCOMING:
        setState(SOS_IDLE, 0); /* dismiss */
        break;
    }
}

static void onShortPress()
{
    if (s_state == SOS_INCOMING) {
        ESP_LOGI(TAG, "incoming SOS dismissed -> find mode");
        setState(SOS_IDLE, 0); /* home screen underneath is Find mode */
    }
}

void sosOnPacket(const TetherPacket &pkt, uint32_t nowMs)
{
    if (pkt.sourceId == HUB_ID) return; /* only wearables raise SOS */

    switch (pkt.type) {
    case MSG_SOS:
        if (pkt.payload.sos.active) {
            s_lastRxMs = nowMs;
            if (s_state == SOS_IDLE) { /* don't override our own outgoing SOS */
                ESP_LOGW(TAG, "SOS INCOMING from %s", deviceName(pkt.sourceId));
                setState(SOS_INCOMING, pkt.sourceId);
                audioPlay(TONE_SOS);
                s_lastToneMs = nowMs;
                s_lastAckMs = 0; /* ACK immediately on next tick */
            }
        } else if (s_state == SOS_INCOMING && pkt.sourceId == s_peerId) {
            setState(SOS_IDLE, 0);
        }
        break;
    case MSG_SOS_CLEAR:
        if (s_state == SOS_INCOMING && pkt.sourceId == s_peerId) {
            ESP_LOGI(TAG, "SOS cleared by %s", deviceName(pkt.sourceId));
            setState(SOS_IDLE, 0);
        }
        break;
    case MSG_ACK:
        if (s_state == SOS_OUTGOING && pkt.destinationId == DEVICE_ID) {
            if (!s_delivered) ESP_LOGI(TAG, "SOS delivered to %s", deviceName(pkt.sourceId));
            s_delivered = true;
        }
        break;
    default:
        break;
    }
}

void sosTick(uint32_t nowMs)
{
    /* ---- button ---- */
    bool pressed = gpio_get_level(SOS_BUTTON_GPIO) == 0;
    if (pressed && !s_wasPressed) {
        s_pressStartMs = nowMs;
        s_longFired = false;
    }
    if (pressed && !s_longFired && nowMs - s_pressStartMs >= SOS_LONGPRESS_MS) {
        s_longFired = true;
        onLongPress(nowMs);
    }
    if (!pressed && s_wasPressed && !s_longFired) {
        onShortPress();
    }
    s_wasPressed = pressed;

    /* ---- periodic work ---- */
    if (s_state == SOS_OUTGOING && nowMs - s_lastSendMs >= SOS_RESEND_MS) {
        s_lastSendMs = nowMs;
        sendSos(true); /* also refreshes the hub overlay */
    }
    if (s_pendingClears > 0 && nowMs - s_lastClearMs >= 300) {
        s_lastClearMs = nowMs;
        s_pendingClears--;
        sendSos(false);
    }
    if (s_state == SOS_INCOMING) {
        if (nowMs - s_lastRxMs > SOS_TIMEOUT_MS) {
            ESP_LOGW(TAG, "incoming SOS expired (sender silent)");
            setState(SOS_IDLE, 0);
        } else {
            if (nowMs - s_lastAckMs >= 2000) {
                s_lastAckMs = nowMs;
                TetherPacket ack = {};
                ack.type = MSG_ACK;
                ack.destinationId = s_peerId;
                ack.ttl = DEFAULT_TTL;
                transportSend(ack);
            }
            if (nowMs - s_lastToneMs >= 5000) {
                s_lastToneMs = nowMs;
                audioPlay(TONE_SOS);
            }
        }
    }
}

SosStatus sosGet()
{
    SosStatus st;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    st = {s_state, s_peerId, s_delivered};
    xSemaphoreGive(s_lock);
    return st;
}

} // namespace tether
