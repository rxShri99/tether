#include "audio/audio.h"

#include <cmath>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2s_std.h"
#include "esp_log.h"

namespace tether {

static const char *TAG = "audio";

/* Waveshare ESP32-S3-Touch-LCD-1.46 speaker (PCM5101) */
#define PIN_SPK_BCK  48
#define PIN_SPK_LRCK 38
#define PIN_SPK_DIN  47

constexpr uint32_t SAMPLE_RATE = 22050;
constexpr int16_t AMPLITUDE = 13000;
constexpr uint32_t EDGE_MS = 8; /* attack/decay to avoid clicks */

struct Note {
    uint16_t freqHz; /* 0 = rest */
    uint16_t ms;
};

/* Little melodies. Kept short and non-annoying on purpose. */
static const Note SEQ_BOOT[]    = {{523, 90}, {784, 130}};
static const Note SEQ_PAIRED[]  = {{880, 110}, {0, 30}, {1109, 160}};
static const Note SEQ_CONNECT[] = {{660, 80}, {0, 25}, {990, 110}};
static const Note SEQ_FOUND[]   = {{784, 95}, {988, 95}, {1319, 95}, {1568, 200}};
static const Note SEQ_LOST[]    = {{523, 130}, {0, 30}, {392, 190}};
static const Note SEQ_SOS[]     = {{988, 90}, {0, 60}, {988, 90}, {0, 60}, {988, 160}};

struct Sequence {
    const Note *notes;
    int count;
};
static const Sequence SEQUENCES[] = {
    {SEQ_BOOT, 2}, {SEQ_PAIRED, 3}, {SEQ_CONNECT, 3},
    {SEQ_FOUND, 4}, {SEQ_LOST, 3}, {SEQ_SOS, 5},
};

static i2s_chan_handle_t s_tx = nullptr;
static QueueHandle_t s_queue = nullptr;

static void playNote(const Note &n)
{
    constexpr int CHUNK_FRAMES = 512;
    static int16_t buf[CHUNK_FRAMES * 2]; /* stereo */

    const int total = (int)((int64_t)SAMPLE_RATE * n.ms / 1000);
    const int edge = (int)(SAMPLE_RATE * EDGE_MS / 1000);
    float phase = 0.0f;
    const float step = 2.0f * (float)M_PI * n.freqHz / SAMPLE_RATE;

    int done = 0;
    while (done < total) {
        int frames = total - done < CHUNK_FRAMES ? total - done : CHUNK_FRAMES;
        for (int i = 0; i < frames; i++) {
            int16_t v = 0;
            if (n.freqHz) {
                float env = 1.0f;
                int pos = done + i;
                if (pos < edge) env = (float)pos / edge;
                else if (total - pos < edge) env = (float)(total - pos) / edge;
                v = (int16_t)(AMPLITUDE * env * sinf(phase));
                phase += step;
                if (phase > 2.0f * (float)M_PI) phase -= 2.0f * (float)M_PI;
            }
            buf[i * 2] = v;
            buf[i * 2 + 1] = v;
        }
        size_t written = 0;
        i2s_channel_write(s_tx, buf, frames * 2 * sizeof(int16_t), &written, portMAX_DELAY);
        done += frames;
    }
}

static void audioTask(void *)
{
    ToneId tone;
    while (true) {
        if (xQueueReceive(s_queue, &tone, portMAX_DELAY) != pdTRUE) continue;
        const Sequence &seq = SEQUENCES[tone];
        for (int i = 0; i < seq.count; i++) playNote(seq.notes[i]);
    }
}

bool audioInit()
{
    i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    chanCfg.auto_clear = true; /* silence (not garbage) on underrun */
    if (i2s_new_channel(&chanCfg, &s_tx, nullptr) != ESP_OK) {
        ESP_LOGW(TAG, "i2s channel alloc failed");
        return false;
    }

    i2s_std_config_t stdCfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = (gpio_num_t)PIN_SPK_BCK,
            .ws = (gpio_num_t)PIN_SPK_LRCK,
            .dout = (gpio_num_t)PIN_SPK_DIN,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {},
        },
    };
    if (i2s_channel_init_std_mode(s_tx, &stdCfg) != ESP_OK ||
        i2s_channel_enable(s_tx) != ESP_OK) {
        ESP_LOGW(TAG, "i2s init failed");
        return false;
    }

    s_queue = xQueueCreate(8, sizeof(ToneId));
    xTaskCreate(audioTask, "audio", 4096, nullptr, 4, nullptr);
    ESP_LOGI(TAG, "PCM5101 up (BCK=%d LRCK=%d DIN=%d)", PIN_SPK_BCK, PIN_SPK_LRCK, PIN_SPK_DIN);
    return true;
}

void audioPlay(ToneId tone)
{
    if (s_queue) xQueueSend(s_queue, &tone, 0);
}

} // namespace tether
