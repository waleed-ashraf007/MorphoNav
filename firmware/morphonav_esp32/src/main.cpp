// MorphoNav ESP32 firmware — micro-ROS bridge for MPU9250 + NEO-7M GPS.
//
// On boot:
//   1. Load Tier 2/3 params from NVS (Preferences). First boot uses defaults from config.h.
//   2. Init MPU9250 with FSR/DLPF/mag-mode and apply calibration + axis remap.
//   3. Init NEO-7M and send UBX-CFG-RATE for the configured rate.
//   4. Init micro-ROS publishers and parameter server.
//   5. Optionally auto-calibrate gyro bias (hold robot stationary at boot).
//
// Loop:
//   - Sample IMU at imu.publish_rate_hz, publish /imu/data_raw + /imu/mag.
//   - Pump GPS UART, publish /gps/fix when a fresh fix arrives at gps.publish_rate_hz.
//   - Spin micro-ROS executor (handles parameter changes).
//   - If parameters changed: re-apply to drivers and persist to NVS.
//   - If calibration trigger set: dispatch the routine.

#include <Arduino.h>
#include "config.h"
#include "params_store.h"
#include "mpu9250.h"
#include "gps_neo7m.h"
#include "microros_bridge.h"
#include <string.h>

static RuntimeParams params;
static MPU9250       imu;
static GpsNeo7M      gps;
static bool          imu_ok = false;
static bool          gps_ok = false;

static void applyImuParams() {
    imu.setAccelFsr(params.imu_accel_fsr_g);
    imu.setGyroFsr(params.imu_gyro_fsr_dps);
    imu.setDlpfBandwidthHz(params.imu_dlpf_hz);
    imu.setMagMode(params.imu_mag_mode_100hz);
    imu.setAccelGyroRemap(params.imu_axis_remap);
    imu.setMagRemap(params.mag_axis_remap);
    imu.setGyroBias(params.gyro_bias);
    imu.setAccelBias(params.accel_bias);
    imu.setMagCal(params.mag_hard_iron, params.mag_soft_iron);
}

static void applyGpsParams() {
    gps.setMinSatellites(params.gps_min_satellites);
    gps.setMaxHdop(params.gps_max_hdop);
    gps.setUpdateRateHz(params.gps_publish_rate_hz);
}

static void runCalibration(int which) {
    if (which == 1) {  // gyro
        imu.autoCalibrateGyro();
        // The MPU9250 driver stores the bias internally; mirror to RuntimeParams.
        // (We don't expose a getter — set directly via repeating the same routine
        // with the params struct as the destination.)
        const int N = 200;
        double sum[3] = {0,0,0};
        float a[3], w[3], m[3]; bool mv;
        for (int i = 0; i < N; ++i) { imu.read(a, w, m, mv); sum[0]+=w[0]; sum[1]+=w[1]; sum[2]+=w[2]; delay(10); }
        // After applying internal bias, the residual w ~ 0; restore + add back to existing.
        // Simpler approach: zero existing and re-collect raw means.
        for (int i = 0; i < 3; ++i) params.gyro_bias[i] = (float)(sum[i] / N) + params.gyro_bias[i];
        applyImuParams();
        params_save(params);
    }
    // accel_face / mag_start / mag_stop are driven by the host calibration script
    // via direct parameter writes (e.g., it writes the resulting hard/soft iron
    // straight into mag_hard/soft, then the param-changed callback persists).
}

void setup() {
    params_load(params);

    // Bring up micro-ROS first so the agent can connect even if hardware init fails.
    // This avoids "silent ESP32" when the IMU isn't wired or the I2C bus floats.
    microros::init(params);

    imu_ok = imu.begin(MPU9250_I2C_ADDR, AK8963_I2C_ADDR);
    if (imu_ok) {
        applyImuParams();
        if (params.auto_gyro_at_boot) {
            delay(500);
            imu.autoCalibrateGyro();
        }
    }

    gps.begin(GPS_UART_NUM, GPS_UART_RX_PIN, GPS_UART_TX_PIN, GPS_BAUD);
    applyGpsParams();
    gps_ok = true;
}

void loop() {
    
    static uint32_t last_imu_us  = 0;
    static uint32_t last_gps_us  = 0;
    const uint32_t now           = micros();

    const uint32_t imu_period_us = (params.imu_publish_rate_hz > 0)
                                     ? (1000000UL / params.imu_publish_rate_hz) : 10000UL;
    const uint32_t gps_period_us = (params.gps_publish_rate_hz > 0)
                                     ? (1000000UL / params.gps_publish_rate_hz) : 200000UL;

    // --- IMU tick ---
    if (imu_ok && (uint32_t)(now - last_imu_us) >= imu_period_us) {
        last_imu_us = now;
        float a[3], w[3], m[3]; bool mag_valid;
        if (imu.read(a, w, m, mag_valid)) {
            microros::publishImu(now, a, w,
                                 params.cov_orientation,
                                 params.cov_angular_velocity,
                                 params.cov_linear_acceleration);
            if (mag_valid) microros::publishMag(now, m);
        }
    }

    // --- GPS tick ---
    if (gps_ok) gps.poll();
    if (gps_ok && (uint32_t)(now - last_gps_us) >= gps_period_us) {
        last_gps_us = now;
        if (gps.hasValidFix()) {
            microros::publishGps(now,
                                 gps.latitude(), gps.longitude(), gps.altitude(),
                                 0 /* STATUS_FIX */, params.gps_cov_position);
        }
    }

    // --- micro-ROS housekeeping ---
    microros::spin(now);

    if (microros::paramsChanged()) {
        applyImuParams();
        applyGpsParams();
        params_save(params);
    }

    if (int trig = microros::calibrationTrigger()) {
        runCalibration(trig);
        microros::clearCalibrationTrigger();
    }
}
