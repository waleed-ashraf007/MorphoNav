"""Logs GPS fix quality once a second.

Useful for verifying outdoors before launching gps_outdoor mode. Prints fix
status, position covariance trace, and stale-fix age.
"""
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class FixHealth(Node):
    def __init__(self):
        super().__init__('gps_fix_health')
        self.last_msg = None
        self.last_rx  = 0.0
        self.create_subscription(NavSatFix, '/gps/fix', self._cb, 10)
        self.create_timer(1.0, self._tick)

    def _cb(self, msg: NavSatFix):
        self.last_msg = msg
        self.last_rx  = time.monotonic()

    def _tick(self):
        if self.last_msg is None:
            self.get_logger().info('no /gps/fix received yet')
            return
        age = time.monotonic() - self.last_rx
        m = self.last_msg
        cov_trace = m.position_covariance[0] + m.position_covariance[4] + m.position_covariance[8]
        status_name = {-1: 'NO_FIX', 0: 'FIX', 1: 'SBAS_FIX', 2: 'GBAS_FIX'}.get(m.status.status, '?')
        self.get_logger().info(
            f'status={status_name} lat={m.latitude:.6f} lon={m.longitude:.6f} '
            f'alt={m.altitude:.1f} cov_trace={cov_trace:.2f} age={age:.1f}s'
        )


def main():
    rclpy.init()
    node = FixHealth()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
