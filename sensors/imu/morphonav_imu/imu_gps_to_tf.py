"""Smoke-test TF bridge: drives `world -> base_footprint` from /imu/data + /gps/fix.

This node is for the standalone `sensor_test.launch.py` ONLY. It must NOT run
alongside the EKF (P4 onward) — robot_localization owns odom -> base_footprint
in normal operation, and two TF publishers on the same edge would collide.

Behavior:
- Rotation: latest /imu/data orientation quaternion (fused by Madgwick).
- Translation: local-ENU offset from the first valid /gps/fix as datum. If no
  fix has been received yet, translation stays at (0, 0, 0) so that rotation
  testing works indoors. Approximation: equirectangular at the datum latitude.
- Broadcast at 30 Hz independent of input rates.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu, NavSatFix
from tf2_ros import TransformBroadcaster

EARTH_R = 6378137.0  # WGS-84 equatorial radius in metres


class ImuGpsToTf(Node):
    def __init__(self):
        super().__init__('imu_gps_to_tf')
        self.declare_parameter('world_frame',  'world')
        self.declare_parameter('child_frame',  'base_footprint')
        self.declare_parameter('publish_rate_hz', 30.0)

        self.world_frame = self.get_parameter('world_frame').value
        self.child_frame = self.get_parameter('child_frame').value

        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 1.0

        # GPS datum is set on first valid fix
        self.datum_lat = None
        self.datum_lon = None
        self.datum_alt = 0.0
        self.east  = 0.0
        self.north = 0.0
        self.up    = 0.0

        self.create_subscription(Imu,        '/imu/data', self._imu_cb, 50)
        self.create_subscription(NavSatFix,  '/gps/fix',  self._gps_cb, 10)

        self.bcaster = TransformBroadcaster(self)
        period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(period, self._tick)

    def _imu_cb(self, msg: Imu):
        self.qx = msg.orientation.x
        self.qy = msg.orientation.y
        self.qz = msg.orientation.z
        self.qw = msg.orientation.w

    def _gps_cb(self, msg: NavSatFix):
        if msg.status.status < 0:
            return  # NavSatStatus.STATUS_NO_FIX
        if self.datum_lat is None:
            self.datum_lat = msg.latitude
            self.datum_lon = msg.longitude
            self.datum_alt = msg.altitude
            self.get_logger().info(
                f'GPS datum set: lat={self.datum_lat:.7f}, lon={self.datum_lon:.7f}'
            )
            return
        # Equirectangular ENU (good for short distances, < few km)
        lat_rad = math.radians(self.datum_lat)
        d_lat = math.radians(msg.latitude  - self.datum_lat)
        d_lon = math.radians(msg.longitude - self.datum_lon)
        self.north = d_lat * EARTH_R
        self.east  = d_lon * EARTH_R * math.cos(lat_rad)
        self.up    = msg.altitude - self.datum_alt

    def _tick(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.world_frame
        t.child_frame_id  = self.child_frame
        t.transform.translation.x = self.east
        t.transform.translation.y = self.north
        t.transform.translation.z = self.up
        # quaternion is normalized by Madgwick; pass through
        t.transform.rotation.x = self.qx
        t.transform.rotation.y = self.qy
        t.transform.rotation.z = self.qz
        t.transform.rotation.w = self.qw
        self.bcaster.sendTransform(t)


def main():
    rclpy.init()
    node = ImuGpsToTf()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
