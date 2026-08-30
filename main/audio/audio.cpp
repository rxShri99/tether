#include "audio/audio.h"
#include "esp_log.h"

namespace tether {

bool audioInit()
{
    ESP_LOGI("audio", "PCM5101 init deferred (tones arrive with SOS/pairing)");
    return true;
}

} // namespace tether
