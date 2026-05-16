// Differential-drive motor control for MorphoNav.
//
// 4 brushed DC motors via two H-bridges (each motor: IN1+IN2 with LEDC PWM
// on IN1 for forward, IN2 for reverse). The same /cmd_vel Twist drives both
// rear wheels (RWD) or all four wheels (AWD) depending on the build flag.
//
// Build flags (set in platformio.ini):
//   -D DRIVE_MODE_AWD     all four motors driven (skid-steer)
//   (default = RWD)       only rear pair driven
//
// Hardware pins are defined in config.h (MOTOR_*_IN1, MOTOR_*_IN2).
// Map cmd_vel -> per-side wheel velocity:
//   v_l = linear_x - angular_z * track_width / 2
//   v_r = linear_x + angular_z * track_width / 2
// Then linearly scale |v| in [0, MOTOR_MAX_LINEAR_MPS] to PWM [0, 255].

#pragma once

#include <stdint.h>

namespace motor {

// Call once from setup(). Configures LEDC channels and direction pins.
void init();

// Drive command in robot frame. Wheel velocities are derived inside.
//   linear_x  : forward (+) / reverse (-) m/s
//   angular_z : CCW (+) / CW (-) rad/s
void setCmdVel(float linear_x, float angular_z);

// Stop all motors immediately. Safe to call repeatedly.
void stop();

// Watchdog: caller should invoke periodically (e.g. every loop). If no
// setCmdVel() has arrived within MOTOR_CMD_TIMEOUT_MS, motors are stopped.
void tick();

} // namespace motor
