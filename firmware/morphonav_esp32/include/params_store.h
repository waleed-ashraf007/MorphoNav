#pragma once
#include <stdint.h>

// Tier 2 (live-tunable, NVS-persisted) and Tier 3 (calibration) values.
// One struct, one save call, atomic enough for our purposes.
struct RuntimeParams {
    // --- IMU (Tier 2) ---
    uint16_t imu_publish_rate_hz;
    uint8_t  imu_accel_fsr_g;        // 2 / 4 / 8 / 16
    uint16_t imu_gyro_fsr_dps;       // 250 / 500 / 1000 / 2000
    uint8_t  imu_dlpf_hz;            // 5 / 10 / 20 / 41 / 92 / 184
    bool     imu_mag_mode_100hz;
    char     imu_axis_remap[12];     // "+x,+y,+z"
    char     mag_axis_remap[12];

    float    cov_orientation[3];
    float    cov_angular_velocity[3];
    float    cov_linear_acceleration[3];

    // --- GPS (Tier 2) ---
    uint8_t  gps_publish_rate_hz;    // 1 / 5 / 10
    uint8_t  gps_min_satellites;
    float    gps_max_hdop;
    float    gps_cov_position;

    // --- Calibration (Tier 3) ---
    float    gyro_bias[3];           // rad/s
    float    accel_bias[3];          // m/s^2
    float    mag_hard_iron[3];       // uT
    float    mag_soft_iron[9];       // 3x3 row-major, identity by default
    bool     auto_gyro_at_boot;
};

void params_reset_defaults(RuntimeParams& p);
void params_load(RuntimeParams& p);
void params_save(const RuntimeParams& p);
