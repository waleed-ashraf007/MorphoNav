#include "gps_neo7m.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>

static HardwareSerial* gps_serial = nullptr;
static TinyGPSPlus     gps;

void GpsNeo7M::begin(uint8_t uart_num, int rx_pin, int tx_pin, uint32_t baud) {
    serial_num_ = uart_num;
    gps_serial = new HardwareSerial(uart_num);
    gps_serial->begin(baud, SERIAL_8N1, rx_pin, tx_pin);
}

// Compute UBX 8-bit Fletcher checksum over class+id+length+payload
static void ubxChecksum(const uint8_t* data, size_t n, uint8_t& ck_a, uint8_t& ck_b) {
    ck_a = 0; ck_b = 0;
    for (size_t i = 0; i < n; ++i) { ck_a += data[i]; ck_b += ck_a; }
}

bool GpsNeo7M::setUpdateRateHz(uint8_t hz) {
    if (!gps_serial) return false;
    uint16_t meas_ms;
    switch (hz) {
        case 1:  meas_ms = 1000; break;
        case 5:  meas_ms = 200;  break;
        case 10: meas_ms = 100;  break;
        default: return false;
    }
    // UBX-CFG-RATE: class 0x06 id 0x08 len 6 payload(measRate, navRate=1, timeRef=GPS=1)
    uint8_t pkt[14] = {
        0xB5, 0x62,
        0x06, 0x08,
        0x06, 0x00,
        (uint8_t)(meas_ms & 0xFF), (uint8_t)(meas_ms >> 8),
        0x01, 0x00,
        0x01, 0x00,
        0,0
    };
    ubxChecksum(&pkt[2], 4 + 6, pkt[12], pkt[13]);
    gps_serial->write(pkt, sizeof(pkt));
    gps_serial->flush();
    return true;
}

void GpsNeo7M::poll() {
    if (!gps_serial) return;
    while (gps_serial->available()) {
        gps.encode(gps_serial->read());
    }
}

bool GpsNeo7M::hasValidFix() const {
    if (!gps.location.isValid() || gps.location.age() > 2000) return false;
    if (!gps.satellites.isValid() || gps.satellites.value() < min_sats_) return false;
    if (gps.hdop.isValid() && gps.hdop.hdop() > max_hdop_) return false;
    return true;
}

double  GpsNeo7M::latitude()   const { return gps.location.lat(); }
double  GpsNeo7M::longitude()  const { return gps.location.lng(); }
double  GpsNeo7M::altitude()   const { return gps.altitude.meters(); }
uint8_t GpsNeo7M::satellites() const { return gps.satellites.isValid() ? gps.satellites.value() : 0; }
float   GpsNeo7M::hdop()       const { return gps.hdop.isValid() ? gps.hdop.hdop() : 99.9f; }
int8_t  GpsNeo7M::fixStatus()  const { return hasValidFix() ? 0 /*STATUS_FIX*/ : -1 /*STATUS_NO_FIX*/; }

void GpsNeo7M::setMinSatellites(uint8_t n) { min_sats_ = n; }
void GpsNeo7M::setMaxHdop(float h)         { max_hdop_ = h; }
