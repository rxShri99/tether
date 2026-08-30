#pragma once
#include "lvgl.h"

namespace tether {

/* Full-screen SOS overlay (hidden while idle). Create last so it sits on top. */
void sosScreenCreate(lv_obj_t *parent);

} // namespace tether
