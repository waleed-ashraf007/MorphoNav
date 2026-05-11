#pragma once

// Tier 1: build-time parameters. Reflash to change these.
// Tier 2 (live-tunable, NVS-persisted) defaults live in params_store.cpp.

// I2C bus — MPU9250 and AK8963 magnetometer share this bus
#define I2C_SDA_PIN              21
#define I2C_SCL_PIN              22
#define I2C_CLOCK_HZ             100000   // 100 kHz tolerates weak (internal) pull-ups

#define MPU9250_I2C_ADDR         0x69   // AD0 floating/high. Tie AD0 to GND for 0x68.
#define AK8963_I2C_ADDR          0x0C

// GPS UART (NEO-7M on UART2)
#define GPS_UART_NUM             2
#define GPS_UART_RX_PIN          16
#define GPS_UART_TX_PIN          17
#define GPS_BAUD                 9600   // NEO-7M factory default

// micro-ROS over USB-serial
#define MICRO_ROS_AGENT_BAUD     115200

// ROS topic and frame names
#define IMU_RAW_TOPIC            "/imu/data_raw"
#define MAG_TOPIC                "/imu/mag"
#define GPS_TOPIC                "/gps/fix"
#define IMU_FRAME_ID             "imu_link"
#define GPS_FRAME_ID             "gps_link"
#define NODE_NAME                "morphonav_esp32"

// Tier 2 defaults (used on first boot before NVS is populated)
#define DEF_IMU_PUBLISH_RATE_HZ  100
#define DEF_IMU_ACCEL_FSR_G      4
#define DEF_IMU_GYRO_FSR_DPS     500
#define DEF_IMU_DLPF_HZ          41
#define DEF_IMU_MAG_MODE_100HZ   true
#define DEF_GPS_PUBLISH_RATE_HZ  5
#define DEF_GPS_MIN_SATELLITES   4
#define DEF_GPS_MAX_HDOP         5.0f
#define DEF_GPS_COV_POSITION     5.0f
#define DEF_AUTO_GYRO_AT_BOOT    true

// AK8963 axes are physically rotated/flipped vs. MPU9250 gyro/accel axes.
// This default works for most modules; verify with the calibration script.
#define DEF_ACCEL_GYRO_REMAP     "+x,+y,+z"
#define DEF_MAG_REMAP            "+y,+x,-z"
