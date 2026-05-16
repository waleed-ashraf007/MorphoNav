// Differential-drive motor control implementation.
//
// LEDC PWM is configured per channel at MOTOR_PWM_FREQ_HZ / MOTOR_PWM_RES_BITS.
// Each motor uses two GPIO pins (H-bridge convention):
//   IN1 high (PWM) + IN2 low  -> forward
//   IN1 low  + IN2 high (PWM) -> reverse
//   both low                  -> coast (or brake if both high — we use coast)

#include "motor_control.h"
#include "config.h"
#include <Arduino.h>

namespace motor {

namespace {

constexpr uint32_t PWM_MAX     = (1u << MOTOR_PWM_RES_BITS) - 1u;
constexpr uint32_t PWM_TIMEOUT = MOTOR_CMD_TIMEOUT_MS;

uint32_t last_cmd_ms = 0;

struct Wheel {
    int      pin_in1;
    int      pin_in2;
    uint8_t  ch_in1;
    uint8_t  ch_in2;
};

#ifdef DRIVE_MODE_AWD
constexpr int NUM_WHEELS = 4;
Wheel wheels[NUM_WHEELS] = {
    {MOTOR_REAR_LEFT_IN1,  MOTOR_REAR_LEFT_IN2,  0, 1},
    {MOTOR_REAR_RIGHT_IN1, MOTOR_REAR_RIGHT_IN2, 2, 3},
    {MOTOR_FRONT_LEFT_IN1, MOTOR_FRONT_LEFT_IN2, 4, 5},
    {MOTOR_FRONT_RIGHT_IN1,MOTOR_FRONT_RIGHT_IN2,6, 7},
};
// side[i] = -1 for left wheels (apply v_l), +1 for right wheels (apply v_r)
const int side[NUM_WHEELS] = {-1, +1, -1, +1};
#else
constexpr int NUM_WHEELS = 2;
Wheel wheels[NUM_WHEELS] = {
    {MOTOR_REAR_LEFT_IN1,  MOTOR_REAR_LEFT_IN2,  0, 1},
    {MOTOR_REAR_RIGHT_IN1, MOTOR_REAR_RIGHT_IN2, 2, 3},
};
const int side[NUM_WHEELS] = {-1, +1};
#endif

void driveWheel(const Wheel& w, float v_mps) {
    // Linearly map |v| in [0, MOTOR_MAX_LINEAR_MPS] to PWM duty.
    float mag = fabsf(v_mps);
    if (mag > MOTOR_MAX_LINEAR_MPS) mag = MOTOR_MAX_LINEAR_MPS;
    uint32_t duty = (uint32_t)(mag / MOTOR_MAX_LINEAR_MPS * (float)PWM_MAX);
    if (v_mps >= 0.0f) {
        ledcWrite(w.ch_in1, duty);
        ledcWrite(w.ch_in2, 0);
    } else {
        ledcWrite(w.ch_in1, 0);
        ledcWrite(w.ch_in2, duty);
    }
}

} // anonymous namespace

void init() {
    for (int i = 0; i < NUM_WHEELS; ++i) {
        pinMode(wheels[i].pin_in1, OUTPUT);
        pinMode(wheels[i].pin_in2, OUTPUT);
        ledcSetup(wheels[i].ch_in1, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RES_BITS);
        ledcSetup(wheels[i].ch_in2, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RES_BITS);
        ledcAttachPin(wheels[i].pin_in1, wheels[i].ch_in1);
        ledcAttachPin(wheels[i].pin_in2, wheels[i].ch_in2);
        ledcWrite(wheels[i].ch_in1, 0);
        ledcWrite(wheels[i].ch_in2, 0);
    }
    last_cmd_ms = millis();
}

void setCmdVel(float linear_x, float angular_z) {
    const float half_b = MOTOR_TRACK_WIDTH_M * 0.5f;
    const float v_l    = linear_x - angular_z * half_b;
    const float v_r    = linear_x + angular_z * half_b;

    for (int i = 0; i < NUM_WHEELS; ++i) {
        driveWheel(wheels[i], side[i] < 0 ? v_l : v_r);
    }
    last_cmd_ms = millis();
}

void stop() {
    for (int i = 0; i < NUM_WHEELS; ++i) {
        ledcWrite(wheels[i].ch_in1, 0);
        ledcWrite(wheels[i].ch_in2, 0);
    }
}

void tick() {
    if (millis() - last_cmd_ms > PWM_TIMEOUT) stop();
}

} // namespace motor
