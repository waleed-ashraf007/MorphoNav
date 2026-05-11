#pragma once
#include <stdint.h>

class GpsNeo7M {
public:
    void begin(uint8_t uart_num, int rx_pin, int tx_pin, uint32_t baud);

    // Send UBX-CFG-RATE for 1 / 5 / 10 Hz. Returns false on unsupported rate.
    bool setUpdateRateHz(uint8_t hz);

    // Pump bytes from UART into TinyGPS. Call frequently from loop().
    void poll();

    // Most-recent fix data. Valid only when hasValidFix() returns true.
    bool hasValidFix() const;
    double latitude()  const;     // degrees
    double longitude() const;     // degrees
    double altitude()  const;     // meters MSL
    uint8_t satellites() const;
    float   hdop() const;
    int8_t  fixStatus() const;    // mirrors sensor_msgs/NavSatStatus.STATUS_*

    // Drop fixes that fail these gates (called from main).
    void setMinSatellites(uint8_t n);
    void setMaxHdop(float h);

private:
    uint32_t serial_num_ = 0;     // HardwareSerial index
    uint8_t  min_sats_   = 4;
    float    max_hdop_   = 5.0f;
};
