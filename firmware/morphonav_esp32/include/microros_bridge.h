#pragma once
#include <stdint.h>
#include "params_store.h"

namespace microros {

// Initialize the agent transport, node, publishers, and parameter server.
// Returns true once the agent has accepted us.
bool init(RuntimeParams& params);

// Pump executor + publish loop. Call from main loop.
// `now_us` is micros() at call time.
void spin(uint32_t now_us);

// Publish helpers — call from main once each sensor has a fresh sample.
// Quaternion is left as identity (orientation is computed host-side by Madgwick).
void publishImu(uint32_t now_us,
                const float a[3], const float w[3],
                const float cov_orient[3], const float cov_w[3], const float cov_a[3]);

void publishMag(uint32_t now_us, const float m[3]);

void publishGps(uint32_t now_us,
                double lat_deg, double lon_deg, double alt_m,
                int8_t status, float cov_position);

// Returns true if the runtime params changed via parameter server since last call.
// When true, caller should re-apply the params to the IMU/GPS drivers and persist to NVS.
bool paramsChanged();

// Returns the requested calibration code (via parameter "trigger_calibration"):
//   0 = idle, 1 = gyro, 2 = accel_face, 3 = mag. Caller dispatches and clears.
int  calibrationTrigger();
void clearCalibrationTrigger();

} // namespace microros
