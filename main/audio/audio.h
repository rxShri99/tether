#pragma once

namespace tether {

/* Short UI tones on the PCM5101 speaker. All calls are non-blocking. */
enum ToneId {
    TONE_BOOT,    /* soft startup blip */
    TONE_PAIRED,  /* pairing success (Phase 6) */
    TONE_CONNECT, /* friend came online */
    TONE_FOUND,   /* proximity reached FOUND — celebratory */
    TONE_LOST,    /* friend went offline / out of range */
    TONE_SOS,     /* urgent alert (Phase 5) */
};

bool audioInit();
void audioPlay(ToneId tone);

} // namespace tether
