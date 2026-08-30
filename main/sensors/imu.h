#pragma once
#include "driver/i2c_master.h"

namespace tether {

/* QMI8658 IMU on the shared I2C bus. Non-fatal if absent. */
bool imuInit(i2c_master_bus_handle_t bus);

/* Integrated gyro-Z heading since boot, degrees (unbounded, drifts slowly). */
float imuGetYawDeg();

/* Accel magnitude in g — bump-to-pair (Phase 6). */
float imuGetAccelMagG();

} // namespace tether
