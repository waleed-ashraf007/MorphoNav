#pragma once
#include <stdint.h>
#include <cstddef>

class MPU9250 {
public:
    bool begin(uint8_t mpu_addr, uint8_t mag_addr);

    // Live re-config (Tier 2)
    bool setAccelFsr(uint8_t g);            // 2 / 4 / 8 / 16
    bool setGyroFsr(uint16_t dps);          // 250 / 500 / 1000 / 2000
    bool setDlpfBandwidthHz(uint8_t hz);    // 5 / 10 / 20 / 41 / 92 / 184
    bool setMagMode(bool hz_100);

    // Calibration application (Tier 3)
    void setGyroBias(const float bias[3]);
    void setAccelBias(const float bias[3]);
    void setMagCal(const float hard[3], const float soft[9]);

    void setAccelGyroRemap(const char* spec);   // "+x,+y,+z"
    void setMagRemap(const char* spec);

    // Auto gyro bias estimation. Caller must hold the device stationary.
    // Blocks ~2 s, samples at ~100 Hz, averages, stores in gyro_bias_.
    void autoCalibrateGyro();

    // Read sensors. Output:
    //   a[3] = accel m/s^2  (calibrated, remapped, bias-removed)
    //   w[3] = gyro  rad/s  (calibrated, remapped, bias-removed)
    //   m[3] = mag   uT     (calibrated, remapped, valid only if mag_valid)
    bool read(float a[3], float w[3], float m[3], bool& mag_valid);

private:
    uint8_t  mpu_addr_, mag_addr_;
    float    accel_scale_, gyro_scale_;     // raw -> SI
    float    mag_factory_adjust_[3];        // AK8963 ASA values applied
    bool     mag_16bit_;

    float    gyro_bias_[3]   = {0,0,0};
    float    accel_bias_[3]  = {0,0,0};
    float    mag_hard_[3]    = {0,0,0};
    float    mag_soft_[9]    = {1,0,0, 0,1,0, 0,0,1};

    int8_t   accel_perm_[3]  = {0,1,2};
    int8_t   accel_sign_[3]  = {1,1,1};
    int8_t   gyro_perm_[3]   = {0,1,2};
    int8_t   gyro_sign_[3]   = {1,1,1};
    int8_t   mag_perm_[3]    = {1,0,2};     // default AK8963 -> body
    int8_t   mag_sign_[3]    = {1,1,-1};

    bool writeReg(uint8_t addr, uint8_t reg, uint8_t val);
    bool readReg (uint8_t addr, uint8_t reg, uint8_t& val);
    bool readBytes(uint8_t addr, uint8_t reg, uint8_t* buf, size_t n);

    static void parseRemap(const char* spec, int8_t perm[3], int8_t sign[3]);
    static void applyRemap(float v[3], const int8_t perm[3], const int8_t sign[3]);
    static void applySoftIron(float m[3], const float soft[9]);
};
