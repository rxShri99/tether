#pragma once
#include "driver/i2c_master.h"

namespace tether {

/* Initializes SPD2010 display + touch + LVGL and shows the home screen. */
bool uiInit();

/* Shared board I2C bus (expander/touch/IMU). nullptr until uiInit / on hub. */
i2c_master_bus_handle_t uiI2CBus();

} // namespace tether
