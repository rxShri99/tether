#pragma once
#include "driver/i2c_master.h"

namespace tether {

/* QMI8658 IMU on the shared I2C bus. Non-fatal if absent. */
bool imuInit(i2c_master_bus_handle_t bus);

/*
 * Heading since boot, degrees (unbounded, drifts slowly).
 *
 * Integrated from rotation about TRUE VERTICAL — the gyro vector projected
 * onto the accelerometer's up direction — so it stays correct with the device
 * flipped, tilted or on edge. Integrating gyro-Z alone only holds while the
 * board is flat and face up.
 */
float imuGetYawDeg();

/* Accel magnitude in g — bump-to-pair (Phase 6). */
float imuGetAccelMagG();

/* Angle between the screen normal and up: 0 = face up, 180 = face down. */
float imuGetTiltDeg();

/* False until the initial gyro bias calibration has completed (~1 s). */
bool imuReady();

} // namespace tether
