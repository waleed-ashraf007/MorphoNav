#include "params_store.h"
#include "config.h"
#include <Preferences.h>
#include <string.h>

static Preferences prefs;
static const char* NS = "morphonav";

void params_reset_defaults(RuntimeParams& p) {
    p.imu_publish_rate_hz = DEF_IMU_PUBLISH_RATE_HZ;
    p.imu_accel_fsr_g     = DEF_IMU_ACCEL_FSR_G;
    p.imu_gyro_fsr_dps    = DEF_IMU_GYRO_FSR_DPS;
    p.imu_dlpf_hz         = DEF_IMU_DLPF_HZ;
    p.imu_mag_mode_100hz  = DEF_IMU_MAG_MODE_100HZ;
    strncpy(p.imu_axis_remap, DEF_ACCEL_GYRO_REMAP, sizeof(p.imu_axis_remap));
    strncpy(p.mag_axis_remap, DEF_MAG_REMAP,        sizeof(p.mag_axis_remap));

    for (int i = 0; i < 3; ++i) {
        p.cov_orientation[i]         = 0.01f;
        p.cov_angular_velocity[i]    = 0.001f;
        p.cov_linear_acceleration[i] = 0.04f;
    }

    p.gps_publish_rate_hz = DEF_GPS_PUBLISH_RATE_HZ;
    p.gps_min_satellites  = DEF_GPS_MIN_SATELLITES;
    p.gps_max_hdop        = DEF_GPS_MAX_HDOP;
    p.gps_cov_position    = DEF_GPS_COV_POSITION;

    for (int i = 0; i < 3; ++i) {
        p.gyro_bias[i]     = 0.0f;
        p.accel_bias[i]    = 0.0f;
        p.mag_hard_iron[i] = 0.0f;
    }
    // Soft-iron starts as identity matrix
    for (int i = 0; i < 9; ++i) p.mag_soft_iron[i] = (i % 4 == 0) ? 1.0f : 0.0f;

    p.auto_gyro_at_boot = DEF_AUTO_GYRO_AT_BOOT;
}

void params_load(RuntimeParams& p) {
    params_reset_defaults(p);
    prefs.begin(NS, true);
    if (prefs.getBool("seeded", false)) {
        p.imu_publish_rate_hz = prefs.getUShort("imu_rate",  p.imu_publish_rate_hz);
        p.imu_accel_fsr_g     = prefs.getUChar ("imu_acc",   p.imu_accel_fsr_g);
        p.imu_gyro_fsr_dps    = prefs.getUShort("imu_gyr",   p.imu_gyro_fsr_dps);
        p.imu_dlpf_hz         = prefs.getUChar ("imu_dlpf",  p.imu_dlpf_hz);
        p.imu_mag_mode_100hz  = prefs.getBool  ("mag_100",   p.imu_mag_mode_100hz);
        prefs.getString("imu_remap", p.imu_axis_remap, sizeof(p.imu_axis_remap));
        prefs.getString("mag_remap", p.mag_axis_remap, sizeof(p.mag_axis_remap));
        prefs.getBytes ("cov_o", p.cov_orientation,         sizeof(p.cov_orientation));
        prefs.getBytes ("cov_w", p.cov_angular_velocity,    sizeof(p.cov_angular_velocity));
        prefs.getBytes ("cov_a", p.cov_linear_acceleration, sizeof(p.cov_linear_acceleration));
        p.gps_publish_rate_hz = prefs.getUChar ("gps_rate",  p.gps_publish_rate_hz);
        p.gps_min_satellites  = prefs.getUChar ("gps_minsv", p.gps_min_satellites);
        p.gps_max_hdop        = prefs.getFloat ("gps_hdop",  p.gps_max_hdop);
        p.gps_cov_position    = prefs.getFloat ("gps_cov",   p.gps_cov_position);
        prefs.getBytes ("cal_g",  p.gyro_bias,     sizeof(p.gyro_bias));
        prefs.getBytes ("cal_a",  p.accel_bias,    sizeof(p.accel_bias));
        prefs.getBytes ("cal_mh", p.mag_hard_iron, sizeof(p.mag_hard_iron));
        prefs.getBytes ("cal_ms", p.mag_soft_iron, sizeof(p.mag_soft_iron));
        p.auto_gyro_at_boot   = prefs.getBool   ("auto_gyr", p.auto_gyro_at_boot);
    }
    prefs.end();
}

void params_save(const RuntimeParams& p) {
    prefs.begin(NS, false);
    prefs.putUShort("imu_rate",  p.imu_publish_rate_hz);
    prefs.putUChar ("imu_acc",   p.imu_accel_fsr_g);
    prefs.putUShort("imu_gyr",   p.imu_gyro_fsr_dps);
    prefs.putUChar ("imu_dlpf",  p.imu_dlpf_hz);
    prefs.putBool  ("mag_100",   p.imu_mag_mode_100hz);
    prefs.putString("imu_remap", p.imu_axis_remap);
    prefs.putString("mag_remap", p.mag_axis_remap);
    prefs.putBytes ("cov_o", p.cov_orientation,         sizeof(p.cov_orientation));
    prefs.putBytes ("cov_w", p.cov_angular_velocity,    sizeof(p.cov_angular_velocity));
    prefs.putBytes ("cov_a", p.cov_linear_acceleration, sizeof(p.cov_linear_acceleration));
    prefs.putUChar ("gps_rate",  p.gps_publish_rate_hz);
    prefs.putUChar ("gps_minsv", p.gps_min_satellites);
    prefs.putFloat ("gps_hdop",  p.gps_max_hdop);
    prefs.putFloat ("gps_cov",   p.gps_cov_position);
    prefs.putBytes ("cal_g",  p.gyro_bias,     sizeof(p.gyro_bias));
    prefs.putBytes ("cal_a",  p.accel_bias,    sizeof(p.accel_bias));
    prefs.putBytes ("cal_mh", p.mag_hard_iron, sizeof(p.mag_hard_iron));
    prefs.putBytes ("cal_ms", p.mag_soft_iron, sizeof(p.mag_soft_iron));
    prefs.putBool  ("auto_gyr", p.auto_gyro_at_boot);
    prefs.putBool  ("seeded",   true);
    prefs.end();
}
