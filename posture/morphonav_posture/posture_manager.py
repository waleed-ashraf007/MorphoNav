"""Pitch-based posture manager.

Subscribes to /imu/data, extracts pitch from the quaternion, applies
smoothing + hysteresis, and publishes a mode string to /robot_posture
only when the mode changes.

Published modes:  "normal"  |  "crouch"

Parameters
----------
uphill_threshold   : float  degrees of positive pitch to trigger crouch (default 10)
downhill_threshold : float  degrees of negative pitch to trigger crouch (default 10)
hysteresis         : float  dead-band in degrees before switching back to normal (default 3)
smoothing_alpha    : float  EMA weight on new sample, 0-1 (default 0.2, lower = smoother)
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String


class PostureManager(Node):
    def __init__(self):
        super().__init__('posture_manager')

        self.declare_parameter('uphill_threshold',   10.0)
        self.declare_parameter('downhill_threshold', 10.0)
        self.declare_parameter('hysteresis',          3.0)
        self.declare_parameter('smoothing_alpha',     0.2)

        self._up_thr   = self.get_parameter('uphill_threshold').value
        self._dn_thr   = self.get_parameter('downhill_threshold').value
        self._hyst     = self.get_parameter('hysteresis').value
        self._alpha    = self.get_parameter('smoothing_alpha').value

        self._smoothed_pitch = 0.0
        self._current_mode   = 'normal'
        self._first_sample   = True

        self._pub_mode  = self.create_publisher(String,  '/robot_posture', 10)
        self._pub_pitch = self.create_publisher(Float32, '/robot_pitch',   10)

        self.create_subscription(
            Imu, '/imu/data', self._imu_cb, qos_profile_sensor_data)

        self.get_logger().info(
            f'posture_manager ready  uphill={self._up_thr}°  '
            f'downhill={self._dn_thr}°  hysteresis={self._hyst}°')

    # ── helpers ──────────────────────────────────────────────────────────────

    @staticmethod
    def _pitch_from_quat(q) -> float:
        """Return pitch in degrees from a ROS Imu quaternion (ENU, REP-103).

        Positive pitch = nose up (robot climbing a ramp).
        """
        x, y, z, w = q.x, q.y, q.z, q.w
        # clamp to avoid domain errors from floating-point noise
        sinp = 2.0 * (w * y - z * x)
        sinp = max(-1.0, min(1.0, sinp))
        return math.degrees(math.asin(sinp))

    def _update_mode(self, pitch_deg: float) -> str:
        if self._current_mode == 'normal':
            if pitch_deg > self._up_thr or pitch_deg < -self._dn_thr:
                return 'crouch'
        else:  # currently crouching
            if abs(pitch_deg) < (min(self._up_thr, self._dn_thr) - self._hyst):
                return 'normal'
        return self._current_mode

    # ── callback ─────────────────────────────────────────────────────────────

    def _imu_cb(self, msg: Imu):
        raw_pitch = self._pitch_from_quat(msg.orientation)

        # seed the filter on the very first sample to avoid a slow warm-up
        if self._first_sample:
            self._smoothed_pitch = raw_pitch
            self._first_sample = False
        else:
            self._smoothed_pitch = (
                self._alpha * raw_pitch
                + (1.0 - self._alpha) * self._smoothed_pitch
            )

        # publish pitch for debugging / visualisation
        self._pub_pitch.publish(Float32(data=float(self._smoothed_pitch)))

        new_mode = self._update_mode(self._smoothed_pitch)
        if new_mode != self._current_mode:
            self._current_mode = new_mode
            self._pub_mode.publish(String(data=self._current_mode))
            self.get_logger().info(
                f'Posture → {self._current_mode}  '
                f'(pitch={self._smoothed_pitch:.1f}°)')


def main():
    rclpy.init()
    node = PostureManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
