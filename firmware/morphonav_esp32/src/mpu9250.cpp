#include "mpu9250.h"
#include "config.h"
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <cstddef>

// MPU9250 registers (subset)
constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
constexpr uint8_t REG_PWR_MGMT_2   = 0x6C;
constexpr uint8_t REG_CONFIG       = 0x1A;     // DLPF
constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_CONFIG2= 0x1D;
constexpr uint8_t REG_INT_PIN_CFG  = 0x37;
constexpr uint8_t REG_USER_CTRL    = 0x6A;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_WHO_AM_I_MPU = 0x75;     // 0x71/0x73=MPU9250/9255, 0x70=MPU6500, 0x68=clone

// AK8963 registers
constexpr uint8_t REG_AK_WIA       = 0x00;     // expect 0x48
constexpr uint8_t REG_AK_ST1       = 0x02;
constexpr uint8_t REG_AK_HXL       = 0x03;
constexpr uint8_t REG_AK_CNTL1     = 0x0A;
constexpr uint8_t REG_AK_ASAX      = 0x10;

// ----------------------------------------------------------------------------
// Helpers
// ----------------------------------------------------------------------------
bool MPU9250::writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

bool MPU9250::readReg(uint8_t addr, uint8_t reg, uint8_t& val) {
    return readBytes(addr, reg, &val, 1);
}

bool MPU9250::readBytes(uint8_t addr, uint8_t reg, uint8_t* buf, std::size_t n) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    size_t got = Wire.requestFrom(addr, (uint8_t)n);
    if (got != n) return false;
    for (size_t i = 0; i < n; ++i) buf[i] = Wire.read();
    return true;
}

// "+x,+y,+z" -> perm={0,1,2}, sign={1,1,1}
// "-y,+x,+z" -> perm={1,0,2}, sign={-1,1,1}
void MPU9250::parseRemap(const char* spec, int8_t perm[3], int8_t sign[3]) {
    int idx = 0;
    const char* p = spec;
    while (*p && idx < 3) {
        int8_t s = 1;
        if (*p == '+') { p++; }
        else if (*p == '-') { s = -1; p++; }
        if      (*p == 'x' || *p == 'X') perm[idx] = 0;
        else if (*p == 'y' || *p == 'Y') perm[idx] = 1;
        else if (*p == 'z' || *p == 'Z') perm[idx] = 2;
        else { perm[idx] = idx; s = 1; }
        sign[idx] = s;
        p++;
        if (*p == ',') p++;
        idx++;
    }
    while (idx < 3) { perm[idx] = idx; sign[idx] = 1; idx++; }
}

void MPU9250::applyRemap(float v[3], const int8_t perm[3], const int8_t sign[3]) {
    float src[3] = { v[0], v[1], v[2] };
    for (int i = 0; i < 3; ++i) v[i] = sign[i] * src[perm[i]];
}

void MPU9250::applySoftIron(float m[3], const float s[9]) {
    float x = m[0], y = m[1], z = m[2];
    m[0] = s[0]*x + s[1]*y + s[2]*z;
    m[1] = s[3]*x + s[4]*y + s[5]*z;
    m[2] = s[6]*x + s[7]*y + s[8]*z;
}

// ----------------------------------------------------------------------------
// Public API
// ----------------------------------------------------------------------------
bool MPU9250::begin(uint8_t mpu_addr, uint8_t mag_addr) {
    mpu_addr_ = mpu_addr;
    mag_addr_ = mag_addr;
    mag_16bit_ = true;

    // Bus-line probe: enable internal pull-ups, then read line states to detect
    // stuck/floating lines BEFORE starting I2C. A healthy idle bus reads HIGH on both.
    pinMode(I2C_SDA_PIN, INPUT_PULLUP);
    pinMode(I2C_SCL_PIN, INPUT_PULLUP);
    delay(10);
    int sda_state = digitalRead(I2C_SDA_PIN);
    int scl_state = digitalRead(I2C_SCL_PIN);
    Serial.printf("I2C bus probe: SDA=%s SCL=%s\n",
                  sda_state ? "HIGH" : "LOW",
                  scl_state ? "HIGH" : "LOW");
    if (!sda_state && !scl_state) {
        Serial.println("  -> BOTH lines LOW. Likely no power to MPU module, or"
                       " GND not connected, or SDA/SCL shorted to GND.");
    } else if (!sda_state) {
        Serial.println("  -> SDA stuck LOW. Likely SDA shorted to GND, or wire"
                       " on wrong pin (should be GPIO 21).");
    } else if (!scl_state) {
        Serial.println("  -> SCL stuck LOW. Likely SCL shorted to GND, or wire"
                       " on wrong pin (should be GPIO 22).");
    } else {
        Serial.println("  -> Both lines HIGH (bus idle, OK to start I2C).");
    }

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_CLOCK_HZ);
    delay(100);

    // Full bus scan — lists every device that ACKs. Helps diagnose whether
    // the bus itself is healthy when the MPU itself is misbehaving.
    Serial.print("I2C scan:");
    int found = 0;
    for (uint8_t a = 0x03; a <= 0x77; ++a) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) {
            Serial.printf(" 0x%02X", a);
            found++;
        }
    }
    Serial.printf("  (%d device%s found)\n", found, found == 1 ? "" : "s");

    // Probe configured address; if it doesn't ACK, try the alternate (AD0 high vs low).
    auto probe = [](uint8_t a) {
        Wire.beginTransmission(a);
        return Wire.endTransmission() == 0;
    };
    if (!probe(mpu_addr_)) {
        uint8_t alt = (mpu_addr_ == 0x68) ? 0x69 : 0x68;
        Serial.printf("MPU9250: 0x%02X did not ACK; trying 0x%02X\n", mpu_addr_, alt);
        if (!probe(alt)) {
            Serial.printf("MPU9250: no ACK on 0x68 or 0x69 — bus is dead.\n");
            return false;
        }
        mpu_addr_ = alt;
        Serial.printf("MPU9250: found at 0x%02X\n", mpu_addr_);
    }

    // Soft reset (DEVICE_RESET = bit 7) before any other config. The chip
    // can latch into a stuck state on cold boot, where it ACKs its address
    // but NACKs every register write until reset.
    writeReg(mpu_addr_, REG_PWR_MGMT_1, 0x80);
    delay(100);
    // After reset, sleep bit is set. Clear it with internal 20 MHz osc first;
    // PLL select is unstable until the gyros have spun up.
    if (!writeReg(mpu_addr_, REG_PWR_MGMT_1, 0x00)) {
        Serial.println("MPU9250: clear-sleep write failed after reset.");
        return false;
    }
    delay(50);
    // Now switch to PLL (better clock).
    writeReg(mpu_addr_, REG_PWR_MGMT_1, 0x01);
    delay(10);

    uint8_t who = 0;
    readReg(mpu_addr_, REG_WHO_AM_I_MPU, who);
    Serial.printf("MPU9250: WHO_AM_I = 0x%02X\n", who);
    if (who != 0x71 && who != 0x73 && who != 0x70 && who != 0x68) {
        Serial.println("MPU9250: WHO_AM_I unrecognised, aborting.");
        return false;
    }

    // Enable I2C bypass so we can talk to AK8963 directly on the same bus
    writeReg(mpu_addr_, REG_USER_CTRL, 0x00);
    writeReg(mpu_addr_, REG_INT_PIN_CFG, 0x02);
    delay(10);

    // Read AK8963 factory ASA values
    writeReg(mag_addr_, REG_AK_CNTL1, 0x00); delay(10);   // power down
    writeReg(mag_addr_, REG_AK_CNTL1, 0x0F); delay(10);   // fuse-ROM access
    uint8_t asa[3] = {128,128,128};
    readBytes(mag_addr_, REG_AK_ASAX, asa, 3);
    for (int i = 0; i < 3; ++i) {
        // sensitivity adjust: H_adj = H * ((ASA-128)*0.5/128 + 1)
        mag_factory_adjust_[i] = ((float)(asa[i] - 128)) * 0.5f / 128.0f + 1.0f;
    }
    writeReg(mag_addr_, REG_AK_CNTL1, 0x00); delay(10);

    // Defaults
    setAccelFsr(DEF_IMU_ACCEL_FSR_G);
    setGyroFsr(DEF_IMU_GYRO_FSR_DPS);
    setDlpfBandwidthHz(DEF_IMU_DLPF_HZ);
    setMagMode(DEF_IMU_MAG_MODE_100HZ);

    return true;
}

bool MPU9250::setAccelFsr(uint8_t g) {
    uint8_t bits;
    switch (g) {
        case 2:  bits = 0x00; accel_scale_ = 2.0f  / 32768.0f * 9.80665f; break;
        case 4:  bits = 0x08; accel_scale_ = 4.0f  / 32768.0f * 9.80665f; break;
        case 8:  bits = 0x10; accel_scale_ = 8.0f  / 32768.0f * 9.80665f; break;
        case 16: bits = 0x18; accel_scale_ = 16.0f / 32768.0f * 9.80665f; break;
        default: return false;
    }
    return writeReg(mpu_addr_, REG_ACCEL_CONFIG, bits);
}

bool MPU9250::setGyroFsr(uint16_t dps) {
    uint8_t bits;
    const float deg2rad = 0.017453292519943295f;
    switch (dps) {
        case 250:  bits = 0x00; gyro_scale_ = 250.0f  / 32768.0f * deg2rad; break;
        case 500:  bits = 0x08; gyro_scale_ = 500.0f  / 32768.0f * deg2rad; break;
        case 1000: bits = 0x10; gyro_scale_ = 1000.0f / 32768.0f * deg2rad; break;
        case 2000: bits = 0x18; gyro_scale_ = 2000.0f / 32768.0f * deg2rad; break;
        default: return false;
    }
    return writeReg(mpu_addr_, REG_GYRO_CONFIG, bits);
}

bool MPU9250::setDlpfBandwidthHz(uint8_t hz) {
    // CONFIG register DLPF_CFG values for gyro/temp
    uint8_t cfg;
    switch (hz) {
        case 184: cfg = 1; break;
        case 92:  cfg = 2; break;
        case 41:  cfg = 3; break;
        case 20:  cfg = 4; break;
        case 10:  cfg = 5; break;
        case 5:   cfg = 6; break;
        default: return false;
    }
    bool ok = writeReg(mpu_addr_, REG_CONFIG, cfg);
    // mirror for accel low-pass (accel_config2)
    ok &= writeReg(mpu_addr_, REG_ACCEL_CONFIG2, cfg);
    return ok;
}

bool MPU9250::setMagMode(bool hz_100) {
    // 16-bit output (bit 4 = 1) + continuous mode 2 (100 Hz) or mode 1 (8 Hz)
    uint8_t cntl = (mag_16bit_ ? 0x10 : 0x00) | (hz_100 ? 0x06 : 0x02);
    writeReg(mag_addr_, REG_AK_CNTL1, 0x00); delay(10);
    return writeReg(mag_addr_, REG_AK_CNTL1, cntl);
}

void MPU9250::setGyroBias (const float b[3]) { for (int i=0;i<3;++i) gyro_bias_[i]  = b[i]; }
void MPU9250::setAccelBias(const float b[3]) { for (int i=0;i<3;++i) accel_bias_[i] = b[i]; }
void MPU9250::setMagCal   (const float h[3], const float s[9]) {
    for (int i=0;i<3;++i) mag_hard_[i] = h[i];
    for (int i=0;i<9;++i) mag_soft_[i] = s[i];
}

void MPU9250::setAccelGyroRemap(const char* spec) {
    parseRemap(spec, accel_perm_, accel_sign_);
    parseRemap(spec, gyro_perm_,  gyro_sign_);
}
void MPU9250::setMagRemap(const char* spec) {
    parseRemap(spec, mag_perm_, mag_sign_);
}

void MPU9250::autoCalibrateGyro() {
    const int N = 200;
    double sum[3] = {0,0,0};
    for (int i = 0; i < N; ++i) {
        uint8_t buf[14];
        if (readBytes(mpu_addr_, REG_ACCEL_XOUT_H, buf, 14)) {
            int16_t gx = (int16_t)((buf[8]  << 8) | buf[9]);
            int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
            int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);
            sum[0] += gx * gyro_scale_;
            sum[1] += gy * gyro_scale_;
            sum[2] += gz * gyro_scale_;
        }
        delay(10);
    }
    for (int i = 0; i < 3; ++i) gyro_bias_[i] = (float)(sum[i] / N);
}

bool MPU9250::read(float a[3], float w[3], float m[3], bool& mag_valid) {
    uint8_t buf[14];
    if (!readBytes(mpu_addr_, REG_ACCEL_XOUT_H, buf, 14)) return false;

    int16_t ax_r = (int16_t)((buf[0]  << 8) | buf[1]);
    int16_t ay_r = (int16_t)((buf[2]  << 8) | buf[3]);
    int16_t az_r = (int16_t)((buf[4]  << 8) | buf[5]);
    int16_t gx_r = (int16_t)((buf[8]  << 8) | buf[9]);
    int16_t gy_r = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t gz_r = (int16_t)((buf[12] << 8) | buf[13]);

    a[0] = ax_r * accel_scale_;
    a[1] = ay_r * accel_scale_;
    a[2] = az_r * accel_scale_;
    w[0] = gx_r * gyro_scale_;
    w[1] = gy_r * gyro_scale_;
    w[2] = gz_r * gyro_scale_;

    // Apply remap then bias removal (order matters: bias is in body frame)
    applyRemap(a, accel_perm_, accel_sign_);
    applyRemap(w, gyro_perm_,  gyro_sign_);
    for (int i = 0; i < 3; ++i) { a[i] -= accel_bias_[i]; w[i] -= gyro_bias_[i]; }

    // Magnetometer — only valid when ST1.DRDY is set
    mag_valid = false;
    uint8_t st1 = 0;
    if (readReg(mag_addr_, REG_AK_ST1, st1) && (st1 & 0x01)) {
        uint8_t mb[7];
        if (readBytes(mag_addr_, REG_AK_HXL, mb, 7)) {
            // ST2 (mb[6]) MUST be read to clear DRDY; HOFL (bit 3) -> overflow, drop
            if (!(mb[6] & 0x08)) {
                int16_t mx_r = (int16_t)((mb[1] << 8) | mb[0]);   // little-endian on AK8963
                int16_t my_r = (int16_t)((mb[3] << 8) | mb[2]);
                int16_t mz_r = (int16_t)((mb[5] << 8) | mb[4]);
                const float scale = mag_16bit_ ? 0.15f : 0.6f;     // uT/LSB
                m[0] = mx_r * scale * mag_factory_adjust_[0];
                m[1] = my_r * scale * mag_factory_adjust_[1];
                m[2] = mz_r * scale * mag_factory_adjust_[2];
                applyRemap(m, mag_perm_, mag_sign_);
                for (int i = 0; i < 3; ++i) m[i] -= mag_hard_[i];
                applySoftIron(m, mag_soft_);
                mag_valid = true;
            }
        }
    }
    return true;
}
