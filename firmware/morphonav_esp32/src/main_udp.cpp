// MorphoNav ESP32 firmware — UDP/JSON variant.
//
// Streams MPU9250 IMU samples (accel, gyro, mag-fused quaternion) over WiFi
// using UDP/JSON to the ROS 2 host. Build with:
//
//     pio run -e udp -t upload
//
// Host side: morphonav_imu / imu_udp_node listens on UDP_PORT.
//
// This is a parallel transport to main.cpp (micro-ROS over serial). Only one
// of {udp, esp32dev} is built at a time; build_src_filter excludes the other.

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <MadgwickAHRS.h>

#include "config.h"
#include "wifi_config.h"
#include "mpu9250.h"
#include "motor_control.h"

static MPU9250  imu;
static Madgwick filter;
static WiFiUDP  udp;             // outgoing IMU stream
static WiFiUDP  cmd_udp;         // incoming cmd_vel stream
static IPAddress host_ip;
static bool     imu_ok = false;

static constexpr float SAMPLE_HZ = 100.0f;
static constexpr uint32_t SAMPLE_PERIOD_US = (uint32_t)(1000000.0f / SAMPLE_HZ);
static constexpr int IMU_INIT_RETRIES = 6;

static const float DEFAULT_GYRO_BIAS[3]  = {0.0f, 0.0f, 0.0f};
static const float DEFAULT_ACCEL_BIAS[3] = {0.0f, 0.0f, 0.0f};
static const float DEFAULT_MAG_HARD[3]   = {0.0f, 0.0f, 0.0f};
static const float DEFAULT_MAG_SOFT[9]   = {1,0,0, 0,1,0, 0,0,1};


static void connectWifi() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true);
    delay(100);
    Serial.printf("WiFi: joining SSID='%s' (password length %u)\n",
                  WIFI_SSID, (unsigned)strlen(WIFI_PASSWORD));
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        // Status: 1=NO_SSID_AVAIL (AP invisible), 4=CONNECT_FAILED (bad pwd/auth),
        // 6=DISCONNECTED (still trying), 3=CONNECTED.
        Serial.printf("WiFi: status=%d, elapsed=%lus\n",
                      WiFi.status(), (unsigned long)((millis() - t0) / 1000));
        if (millis() - t0 > 20000) {
            Serial.println("WiFi: timeout — scanning visible 2.4 GHz networks:");
            int n = WiFi.scanNetworks();
            Serial.printf("WiFi: scan returned %d (negative=failure)\n", n);
            for (int i = 0; i < n; ++i) {
                Serial.printf("  '%s'  RSSI=%d  ch=%d  enc=%d\n",
                              WiFi.SSID(i).c_str(), WiFi.RSSI(i),
                              WiFi.channel(i), WiFi.encryptionType(i));
            }
            Serial.println("WiFi: retrying connect...");
            WiFi.disconnect(true);
            delay(1000);
            WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
            t0 = millis();
        }
    }
    Serial.printf("WiFi: connected, IP=%s, RSSI=%d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
}


static bool tryInitImu() {
    for (int i = 1; i <= IMU_INIT_RETRIES; ++i) {
        if (imu.begin(MPU9250_I2C_ADDR, AK8963_I2C_ADDR)) {
            Serial.printf("MPU9250 OK (attempt %d/%d)\n", i, IMU_INIT_RETRIES);
            return true;
        }
        Serial.printf("MPU9250 init attempt %d/%d failed, retrying...\n",
                      i, IMU_INIT_RETRIES);
        delay(400);
    }
    return false;
}

void setup() {
    Serial.begin(115200);
    delay(200);

    // GPIO 18 powers MPU9250 VCC on boards with broken 3V3 rail (mirrors main.cpp).
    pinMode(18, OUTPUT);
    digitalWrite(18, HIGH);
    delay(100);

    imu_ok = tryInitImu();
    if (imu_ok) {
        imu.setAccelFsr(DEF_IMU_ACCEL_FSR_G);
        imu.setGyroFsr(DEF_IMU_GYRO_FSR_DPS);
        imu.setDlpfBandwidthHz(DEF_IMU_DLPF_HZ);
        imu.setMagMode(DEF_IMU_MAG_MODE_100HZ);
        imu.setAccelGyroRemap(DEF_ACCEL_GYRO_REMAP);
        imu.setMagRemap(DEF_MAG_REMAP);
        imu.setGyroBias(DEFAULT_GYRO_BIAS);
        imu.setAccelBias(DEFAULT_ACCEL_BIAS);
        imu.setMagCal(DEFAULT_MAG_HARD, DEFAULT_MAG_SOFT);
        delay(500);
        imu.autoCalibrateGyro();   // hold robot still during boot
    } else {
        Serial.println("MPU9250 unreachable; UDP IMU disabled. Check wiring "
                       "(SDA=21, SCL=22, VCC=3V3) and pull-ups.");
    }

    filter.begin(SAMPLE_HZ);

    connectWifi();
    host_ip.fromString(HOST_IP);
    udp.begin(0);              // ephemeral local port; we only send

    cmd_udp.begin(CMD_VEL_UDP_PORT);
    motor::init();

    Serial.printf("Streaming IMU JSON to %s:%u at %.0f Hz (imu_ok=%d)\n",
                  HOST_IP, (unsigned)UDP_PORT, SAMPLE_HZ, (int)imu_ok);
    Serial.printf("Listening for cmd_vel JSON on UDP port %u\n",
                  (unsigned)CMD_VEL_UDP_PORT);
}


// Parse JSON of the form: {"vx": <float>, "wz": <float>}
// Tolerant: ignores other fields, returns false if either is missing.
static bool parseCmdVel(const char* buf, int n, float& vx, float& wz) {
    auto extract = [&](const char* key, float& out) {
        const char* p = strstr(buf, key);
        if (!p) return false;
        p = strchr(p, ':');
        if (!p) return false;
        out = strtof(p + 1, nullptr);
        return true;
    };
    return extract("\"vx\"", vx) & extract("\"wz\"", wz);
}

static void pumpCmdVel() {
    int n = cmd_udp.parsePacket();
    if (n <= 0) return;
    char buf[128];
    int got = cmd_udp.read(buf, sizeof(buf) - 1);
    if (got <= 0) return;
    buf[got] = '\0';
    float vx, wz;
    if (parseCmdVel(buf, got, vx, wz)) {
        motor::setCmdVel(vx, wz);
    }
}


void loop() {
    static uint32_t last_us = 0;
    static uint32_t last_recovery_ms = 0;
    const uint32_t now = micros();

    // Motor control runs every loop iteration regardless of IMU state.
    pumpCmdVel();
    motor::tick();

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi: link lost — reconnecting");
        motor::stop();
        connectWifi();
    }

    // If the IMU never came up, retry every 5 s. WiFi keeps running so the
    // monitor stays usable for debugging.
    if (!imu_ok) {
        const uint32_t now_ms = millis();
        if (now_ms - last_recovery_ms > 5000) {
            last_recovery_ms = now_ms;
            Serial.println("Retrying MPU9250 init...");
            imu_ok = tryInitImu();
            if (imu_ok) {
                imu.setAccelFsr(DEF_IMU_ACCEL_FSR_G);
                imu.setGyroFsr(DEF_IMU_GYRO_FSR_DPS);
                imu.setDlpfBandwidthHz(DEF_IMU_DLPF_HZ);
                imu.setMagMode(DEF_IMU_MAG_MODE_100HZ);
                imu.setAccelGyroRemap(DEF_ACCEL_GYRO_REMAP);
                imu.setMagRemap(DEF_MAG_REMAP);
                imu.autoCalibrateGyro();
            }
        }
        delay(10);
        return;
    }

    if ((uint32_t)(now - last_us) < SAMPLE_PERIOD_US) return;
    last_us = now;

    float a[3], w[3], m[3]; bool mag_valid;
    if (!imu.read(a, w, m, mag_valid)) return;

    // Madgwick wants gyro in deg/s (Arduino library convention).
    const float RAD2DEG = 57.29577951308232f;
    const float gx = w[0] * RAD2DEG;
    const float gy = w[1] * RAD2DEG;
    const float gz = w[2] * RAD2DEG;

    if (mag_valid) {
        filter.update(gx, gy, gz, a[0], a[1], a[2], m[0], m[1], m[2]);
    } else {
        filter.updateIMU(gx, gy, gz, a[0], a[1], a[2]);
    }

    // Madgwick stores quaternion internally; expose via roll/pitch/yaw, then
    // rebuild quaternion (the library exposes q via .getRoll/.getPitch/.getYaw
    // and additionally q via .q1..q4 in newer versions; portable fallback uses
    // the ZYX Euler convention.)
    const float roll  = filter.getRoll()  * 0.017453292519943295f;
    const float pitch = filter.getPitch() * 0.017453292519943295f;
    const float yaw   = filter.getYaw()   * 0.017453292519943295f;
    const float cr = cosf(roll  * 0.5f), sr = sinf(roll  * 0.5f);
    const float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    const float cy = cosf(yaw   * 0.5f), sy = sinf(yaw   * 0.5f);
    const float qw = cr*cp*cy + sr*sp*sy;
    const float qx = sr*cp*cy - cr*sp*sy;
    const float qy = cr*sp*cy + sr*cp*sy;
    const float qz = cr*cp*sy - sr*sp*cy;

    char buf[256];
    int n = snprintf(buf, sizeof(buf),
        "{\"ax\":%.4f,\"ay\":%.4f,\"az\":%.4f,"
        "\"gx\":%.5f,\"gy\":%.5f,\"gz\":%.5f,"
        "\"qx\":%.6f,\"qy\":%.6f,\"qz\":%.6f,\"qw\":%.6f,"
        "\"timestamp\":%u}",
        a[0], a[1], a[2],
        w[0], w[1], w[2],
        qx, qy, qz, qw,
        (unsigned)(now / 1000));   // ms since boot

    if (n > 0 && (size_t)n < sizeof(buf)) {
        udp.beginPacket(host_ip, UDP_PORT);
        udp.write((const uint8_t*)buf, (size_t)n);
        udp.endPacket();
    }
}
