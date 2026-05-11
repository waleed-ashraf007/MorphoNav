// Standalone MPU9250 diagnostic — no micro-ROS, prints to USB serial.
// Build:   pio run -e imu_test -t upload
// Monitor: pio device monitor -e imu_test
// Flash production firmware back with: pio run -e esp32dev -t upload

#include <Arduino.h>
#include <Wire.h>

constexpr uint8_t SDA_PIN  = 21;
constexpr uint8_t SCL_PIN  = 22;
constexpr uint32_t I2C_HZ  = 400000;

constexpr uint8_t MPU_ADDR = 0x68;
constexpr uint8_t MAG_ADDR = 0x0C;

constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
constexpr uint8_t REG_INT_PIN_CFG  = 0x37;
constexpr uint8_t REG_USER_CTRL    = 0x6A;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_WHO_AM_I_MPU = 0x75;
constexpr uint8_t REG_AK_WIA       = 0x00;
constexpr uint8_t REG_AK_CNTL1     = 0x0A;
constexpr uint8_t REG_AK_HXL       = 0x03;
constexpr uint8_t REG_AK_ST1       = 0x02;

static bool writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

static bool readBytes(uint8_t addr, uint8_t reg, uint8_t* buf, size_t n) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    size_t got = Wire.requestFrom(addr, (uint8_t)n);
    if (got != n) return false;
    for (size_t i = 0; i < n; ++i) buf[i] = Wire.read();
    return true;
}

static bool readReg(uint8_t addr, uint8_t reg, uint8_t& val) {
    return readBytes(addr, reg, &val, 1);
}

static void i2cScan() {
    Serial.println("[i2c] scanning 0x03..0x77 ...");
    int found = 0;
    for (uint8_t a = 0x03; a <= 0x77; ++a) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  found 0x%02X\n", a);
            found++;
        }
    }
    Serial.printf("[i2c] %d device(s) responded\n", found);
    if (found == 0) {
        Serial.println("[i2c] NO DEVICES — check VCC, GND, SDA->21, SCL->22, AD0->GND, pull-ups");
    }
}

void setup() {
    Serial.begin(115200);
    delay(2000);  // give the host time to attach the monitor
    Serial.println();
    Serial.println("=== MorphoNav IMU diagnostic ===");

    Wire.begin(SDA_PIN, SCL_PIN, I2C_HZ);

    i2cScan();

    // --- MPU9250 ---
    Serial.println("[mpu] waking PWR_MGMT_1 = 0x01");
    if (!writeReg(MPU_ADDR, REG_PWR_MGMT_1, 0x01)) {
        Serial.println("[mpu] WRITE FAILED — chip not ACKing on 0x68. Stop.");
        return;
    }
    delay(50);

    uint8_t who = 0xFF;
    if (!readReg(MPU_ADDR, REG_WHO_AM_I_MPU, who)) {
        Serial.println("[mpu] WHO_AM_I read failed");
        return;
    }
    Serial.printf("[mpu] WHO_AM_I = 0x%02X (expect 0x71 or 0x73; 0x68 = clone)\n", who);
    if (who != 0x71 && who != 0x73 && who != 0x68) {
        Serial.println("[mpu] unexpected WHO_AM_I — wrong chip, bad power, or noisy bus");
    }

    // Enable I2C bypass so we can talk to the AK8963 magnetometer directly.
    writeReg(MPU_ADDR, REG_USER_CTRL, 0x00);
    writeReg(MPU_ADDR, REG_INT_PIN_CFG, 0x02);
    delay(10);

    // --- AK8963 magnetometer ---
    uint8_t wia = 0xFF;
    if (!readReg(MAG_ADDR, REG_AK_WIA, wia)) {
        Serial.println("[mag] WIA read failed (bypass not active or mag dead)");
    } else {
        Serial.printf("[mag] WIA = 0x%02X (expect 0x48)\n", wia);
    }

    // Put mag in continuous-mode-2 16-bit so we can read it later.
    writeReg(MAG_ADDR, REG_AK_CNTL1, 0x00); delay(10);
    writeReg(MAG_ADDR, REG_AK_CNTL1, 0x16); delay(10);

    Serial.println("=== streaming raw accel/gyro/mag at 5 Hz — move the board, values should change ===");
}

void loop() {
    uint8_t buf[14];
    if (readBytes(MPU_ADDR, REG_ACCEL_XOUT_H, buf, 14)) {
        int16_t ax = (buf[0]  << 8) | buf[1];
        int16_t ay = (buf[2]  << 8) | buf[3];
        int16_t az = (buf[4]  << 8) | buf[5];
        int16_t t  = (buf[6]  << 8) | buf[7];
        int16_t gx = (buf[8]  << 8) | buf[9];
        int16_t gy = (buf[10] << 8) | buf[11];
        int16_t gz = (buf[12] << 8) | buf[13];
        Serial.printf("acc=%6d %6d %6d  gyr=%6d %6d %6d  T=%d  ",
                      ax, ay, az, gx, gy, gz, t);
    } else {
        Serial.print("acc/gyr READ FAIL  ");
    }

    // Mag: read ST1, then HXL..HZH (6 bytes), then ST2 to release.
    uint8_t st1 = 0;
    if (readReg(MAG_ADDR, REG_AK_ST1, st1) && (st1 & 0x01)) {
        uint8_t mb[7];
        if (readBytes(MAG_ADDR, REG_AK_HXL, mb, 7)) {
            int16_t mx = (int16_t)((mb[1] << 8) | mb[0]);
            int16_t my = (int16_t)((mb[3] << 8) | mb[2]);
            int16_t mz = (int16_t)((mb[5] << 8) | mb[4]);
            Serial.printf("mag=%6d %6d %6d", mx, my, mz);
        } else {
            Serial.print("mag READ FAIL");
        }
    } else {
        Serial.print("mag (no fresh sample)");
    }
    Serial.println();
    delay(200);
}
