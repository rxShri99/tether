#pragma once
#include "lvgl.h"

namespace tether {

/* Builds the wearable home screen on `parent` and starts its update timer. */
void homeScreenCreate(lv_obj_t *parent);

} // namespace tether
