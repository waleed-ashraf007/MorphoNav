#include "microros_bridge.h"
#include "config.h"
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc_parameter/rclc_parameter.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <sensor_msgs/msg/nav_sat_status.h>
#include <string.h>

namespace microros {

static rcl_allocator_t          allocator;
static rclc_support_t           support;
static rcl_node_t               node;
static rclc_executor_t          executor;
static rclc_parameter_server_t  param_server;

static rcl_publisher_t pub_imu, pub_mag, pub_gps;
static sensor_msgs__msg__Imu           msg_imu;
static sensor_msgs__msg__MagneticField msg_mag;
static sensor_msgs__msg__NavSatFix     msg_gps;

static RuntimeParams* params_ref = nullptr;
static bool   params_dirty = false;
static int    cal_trigger  = 0;   // 0=idle, 1=gyro, 2=accel_face, 3=mag

#define RCCHECK(fn) { rcl_ret_t rc = (fn); if (rc != RCL_RET_OK) return false; }

// ---- parameter callback: bridges micro-ROS param changes -> RuntimeParams ----
static bool on_param_changed(const Parameter* old_p, const Parameter* new_p, void*) {
    if (!new_p || !params_ref) return true;
    const char* name = new_p->name.data;

    if      (!strcmp(name, "imu.publish_rate_hz"))   params_ref->imu_publish_rate_hz = (uint16_t)new_p->value.integer_value;
    else if (!strcmp(name, "imu.accel_fsr_g"))       params_ref->imu_accel_fsr_g     = (uint8_t) new_p->value.integer_value;
    else if (!strcmp(name, "imu.gyro_fsr_dps"))      params_ref->imu_gyro_fsr_dps    = (uint16_t)new_p->value.integer_value;
    else if (!strcmp(name, "imu.dlpf_hz"))           params_ref->imu_dlpf_hz         = (uint8_t) new_p->value.integer_value;
    else if (!strcmp(name, "imu.mag_mode_100hz"))    params_ref->imu_mag_mode_100hz  = new_p->value.bool_value;
    else if (!strcmp(name, "gps.publish_rate_hz"))   params_ref->gps_publish_rate_hz = (uint8_t) new_p->value.integer_value;
    else if (!strcmp(name, "gps.min_satellites"))    params_ref->gps_min_satellites  = (uint8_t) new_p->value.integer_value;
    else if (!strcmp(name, "gps.max_hdop"))          params_ref->gps_max_hdop        = (float)   new_p->value.double_value;
    else if (!strcmp(name, "gps.cov_position"))      params_ref->gps_cov_position    = (float)   new_p->value.double_value;
    else if (!strcmp(name, "auto_gyro_at_boot"))     params_ref->auto_gyro_at_boot   = new_p->value.bool_value;
    else if (!strcmp(name, "trigger_calibration"))   cal_trigger = (int)new_p->value.integer_value;
    else { return true; }   // unknown name, accept silently

    params_dirty = true;
    return true;
}

bool init(RuntimeParams& params) {
    params_ref = &params;
    Serial.begin(MICRO_ROS_AGENT_BAUD);
    set_microros_serial_transports(Serial);
    delay(2000);

    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

    // Publishers (best-effort for high-rate IMU)
    RCCHECK(rclc_publisher_init_best_effort(
        &pub_imu, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        IMU_RAW_TOPIC));

    RCCHECK(rclc_publisher_init_best_effort(
        &pub_mag, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
        MAG_TOPIC));

    RCCHECK(rclc_publisher_init_default(
        &pub_gps, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),
        GPS_TOPIC));

    // Parameter server (Tier 2 + calibration trigger)
    rclc_parameter_options_t param_opts = {
        .notify_changed_over_dds = true,
        .max_params              = 16,
        .allow_undeclared_parameters = false,
        .low_mem_mode            = false,
    };
    RCCHECK(rclc_parameter_server_init_with_option(&param_server, &node, &param_opts));

    // Executor needs slots for parameter server callbacks
    RCCHECK(rclc_executor_init(&executor, &support.context, RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES, &allocator));
    RCCHECK(rclc_executor_add_parameter_server(&executor, &param_server, on_param_changed));

    // Declare parameters with their current values
    // Note: String parameters are not settable via rclc_parameter_set_string (not available in this library version)
    // but they can still be updated via DDS and handled in on_param_changed callback
    rclc_add_parameter(&param_server, "imu.publish_rate_hz",  RCLC_PARAMETER_INT);
    rclc_add_parameter(&param_server, "imu.accel_fsr_g",      RCLC_PARAMETER_INT);
    rclc_add_parameter(&param_server, "imu.gyro_fsr_dps",     RCLC_PARAMETER_INT);
    rclc_add_parameter(&param_server, "imu.dlpf_hz",          RCLC_PARAMETER_INT);
    rclc_add_parameter(&param_server, "imu.mag_mode_100hz",   RCLC_PARAMETER_BOOL);
    rclc_add_parameter(&param_server, "gps.publish_rate_hz",  RCLC_PARAMETER_INT);
    rclc_add_parameter(&param_server, "gps.min_satellites",   RCLC_PARAMETER_INT);
    rclc_add_parameter(&param_server, "gps.max_hdop",         RCLC_PARAMETER_DOUBLE);
    rclc_add_parameter(&param_server, "gps.cov_position",     RCLC_PARAMETER_DOUBLE);
    rclc_add_parameter(&param_server, "auto_gyro_at_boot",    RCLC_PARAMETER_BOOL);
    rclc_add_parameter(&param_server, "trigger_calibration",  RCLC_PARAMETER_INT);

    rclc_parameter_set_int   (&param_server, "imu.publish_rate_hz",  params.imu_publish_rate_hz);
    rclc_parameter_set_int   (&param_server, "imu.accel_fsr_g",      params.imu_accel_fsr_g);
    rclc_parameter_set_int   (&param_server, "imu.gyro_fsr_dps",     params.imu_gyro_fsr_dps);
    rclc_parameter_set_int   (&param_server, "imu.dlpf_hz",          params.imu_dlpf_hz);
    rclc_parameter_set_bool  (&param_server, "imu.mag_mode_100hz",   params.imu_mag_mode_100hz);
    rclc_parameter_set_int   (&param_server, "gps.publish_rate_hz",  params.gps_publish_rate_hz);
    rclc_parameter_set_int   (&param_server, "gps.min_satellites",   params.gps_min_satellites);
    rclc_parameter_set_double(&param_server, "gps.max_hdop",         params.gps_max_hdop);
    rclc_parameter_set_double(&param_server, "gps.cov_position",     params.gps_cov_position);
    rclc_parameter_set_bool  (&param_server, "auto_gyro_at_boot",    params.auto_gyro_at_boot);

    // Initialize message memory (NavSatFix has fixed-size arrays; std msgs are POD)
    sensor_msgs__msg__Imu__init(&msg_imu);
    sensor_msgs__msg__MagneticField__init(&msg_mag);
    sensor_msgs__msg__NavSatFix__init(&msg_gps);
    msg_imu.header.frame_id.data     = (char*)IMU_FRAME_ID; msg_imu.header.frame_id.size = strlen(IMU_FRAME_ID);
    msg_mag.header.frame_id.data     = (char*)IMU_FRAME_ID; msg_mag.header.frame_id.size = strlen(IMU_FRAME_ID);
    msg_gps.header.frame_id.data     = (char*)GPS_FRAME_ID; msg_gps.header.frame_id.size = strlen(GPS_FRAME_ID);

    return true;
}

void spin(uint32_t /*now_us*/) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2));
}

static inline void stamp(builtin_interfaces__msg__Time& t, uint32_t now_us) {
    t.sec     = (int32_t)(now_us / 1000000UL);
    t.nanosec = (uint32_t)((now_us % 1000000UL) * 1000UL);
}

void publishImu(uint32_t now_us,
                const float a[3], const float w[3],
                const float co[3], const float cw[3], const float ca[3]) {
    stamp(msg_imu.header.stamp, now_us);
    msg_imu.linear_acceleration.x = a[0];
    msg_imu.linear_acceleration.y = a[1];
    msg_imu.linear_acceleration.z = a[2];
    msg_imu.angular_velocity.x    = w[0];
    msg_imu.angular_velocity.y    = w[1];
    msg_imu.angular_velocity.z    = w[2];
    // Identity orientation — host-side Madgwick computes the real one
    msg_imu.orientation.x = 0; msg_imu.orientation.y = 0;
    msg_imu.orientation.z = 0; msg_imu.orientation.w = 1;

    // Diagonal covariances (row-major 3x3, off-diagonals zero)
    for (int i = 0; i < 9; ++i) {
        msg_imu.orientation_covariance[i]         = 0;
        msg_imu.angular_velocity_covariance[i]    = 0;
        msg_imu.linear_acceleration_covariance[i] = 0;
    }
    msg_imu.orientation_covariance[0] = co[0]; msg_imu.orientation_covariance[4] = co[1]; msg_imu.orientation_covariance[8] = co[2];
    msg_imu.angular_velocity_covariance[0] = cw[0]; msg_imu.angular_velocity_covariance[4] = cw[1]; msg_imu.angular_velocity_covariance[8] = cw[2];
    msg_imu.linear_acceleration_covariance[0] = ca[0]; msg_imu.linear_acceleration_covariance[4] = ca[1]; msg_imu.linear_acceleration_covariance[8] = ca[2];

    rcl_ret_t rc = rcl_publish(&pub_imu, &msg_imu, NULL);
    (void)rc;  // Suppress unused result warning
}

void publishMag(uint32_t now_us, const float m[3]) {
    stamp(msg_mag.header.stamp, now_us);
    // sensor_msgs/MagneticField uses Tesla; we have uT
    msg_mag.magnetic_field.x = m[0] * 1e-6f;
    msg_mag.magnetic_field.y = m[1] * 1e-6f;
    msg_mag.magnetic_field.z = m[2] * 1e-6f;
    for (int i = 0; i < 9; ++i) msg_mag.magnetic_field_covariance[i] = 0;
    msg_mag.magnetic_field_covariance[0] = msg_mag.magnetic_field_covariance[4] = msg_mag.magnetic_field_covariance[8] = 1e-12f;
    rcl_ret_t rc = rcl_publish(&pub_mag, &msg_mag, NULL);
    (void)rc;  // Suppress unused result warning
}

void publishGps(uint32_t now_us,
                double lat, double lon, double alt,
                int8_t status, float cov_pos) {
    stamp(msg_gps.header.stamp, now_us);
    msg_gps.status.status   = status;     // -1 NO_FIX, 0 FIX, 1 SBAS, 2 GBAS
    msg_gps.status.service  = 1;          // GPS
    msg_gps.latitude        = lat;
    msg_gps.longitude       = lon;
    msg_gps.altitude        = alt;
    for (int i = 0; i < 9; ++i) msg_gps.position_covariance[i] = 0;
    msg_gps.position_covariance[0] = cov_pos;
    msg_gps.position_covariance[4] = cov_pos;
    msg_gps.position_covariance[8] = cov_pos * 4;     // altitude noisier
    msg_gps.position_covariance_type = 2; // DIAGONAL_KNOWN
    rcl_ret_t rc = rcl_publish(&pub_gps, &msg_gps, NULL);
    (void)rc;  // Suppress unused result warning
}

bool paramsChanged() { bool d = params_dirty; params_dirty = false; return d; }

int  calibrationTrigger()      { return cal_trigger; }
void clearCalibrationTrigger() { cal_trigger = 0; }

} // namespace microros
