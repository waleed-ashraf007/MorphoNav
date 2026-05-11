"""UDP -> ROS 2 bridge for the ESP32 UDP/JSON IMU firmware.

Listens on UDP `port` and converts each JSON packet from the ESP32 into:
  /imu/data_raw  (sensor_msgs/Imu, no orientation)  -> feeds host Madgwick
  /imu/data_udp  (sensor_msgs/Imu, on-device fused) -> for inspection / standalone

The ESP32 already runs Madgwick on-device, but the rest of the morphonav
pipeline expects /imu/data_raw -> imu_filter_madgwick -> /imu/data so the EKF
gets the same covariance-injected message it always has. We publish both: the
"raw" variant (with orientation_covariance[0] = -1 to signal "no orientation",
per REP-145) drives the existing chain unchanged; the "udp" variant carries
the on-board quaternion for users who want the standalone /imu odometry path.

Frame: imu_link (matches the URDF and existing /imu/data_raw producer).
"""
import json
import socket

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class ImuUdpNode(Node):
    def __init__(self):
        super().__init__('imu_udp_node')

        self.declare_parameter('port', 8888)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('raw_topic', '/imu/data_raw')
        self.declare_parameter('fused_topic', '/imu/data_udp')
        self.declare_parameter('publish_fused', True)
        self.declare_parameter('cov_angular_velocity',    [0.001, 0.001, 0.001])
        self.declare_parameter('cov_linear_acceleration', [0.04, 0.04, 0.04])
        self.declare_parameter('cov_orientation',         [0.01, 0.01, 0.01])

        self.port      = int(self.get_parameter('port').value)
        self.frame_id  = self.get_parameter('frame_id').value
        self.raw_topic   = self.get_parameter('raw_topic').value
        self.fused_topic = self.get_parameter('fused_topic').value
        self.publish_fused = bool(self.get_parameter('publish_fused').value)
        self.cw = self.get_parameter('cov_angular_velocity').value
        self.ca = self.get_parameter('cov_linear_acceleration').value
        self.co = self.get_parameter('cov_orientation').value

        self.pub_raw = self.create_publisher(Imu, self.raw_topic, qos_profile_sensor_data)
        self.pub_fused = (
            self.create_publisher(Imu, self.fused_topic, qos_profile_sensor_data)
            if self.publish_fused else None
        )

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', self.port))
        self.sock.setblocking(False)

        # Drain the socket from a timer rather than a thread — keeps everything
        # in the rclpy executor and avoids GIL hand-offs per packet.
        self.create_timer(0.001, self._drain)

        self.n_received = 0
        self.create_timer(2.0, self._stats)
        self.get_logger().info(f'Listening on UDP/{self.port}, publishing {self.raw_topic}')

    def _diag3(self, v):
        return [v[0], 0.0, 0.0,
                0.0, v[1], 0.0,
                0.0, 0.0, v[2]]

    def _drain(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(2048)
            except BlockingIOError:
                return
            try:
                pkt = json.loads(data.decode('utf-8', errors='replace'))
            except (ValueError, UnicodeDecodeError):
                continue
            self._publish(pkt)
            self.n_received += 1

    def _publish(self, pkt):
        stamp = self.get_clock().now().to_msg()

        raw = Imu()
        raw.header.stamp = stamp
        raw.header.frame_id = self.frame_id
        raw.linear_acceleration.x = float(pkt.get('ax', 0.0))
        raw.linear_acceleration.y = float(pkt.get('ay', 0.0))
        raw.linear_acceleration.z = float(pkt.get('az', 0.0))
        raw.angular_velocity.x = float(pkt.get('gx', 0.0))
        raw.angular_velocity.y = float(pkt.get('gy', 0.0))
        raw.angular_velocity.z = float(pkt.get('gz', 0.0))
        raw.linear_acceleration_covariance = self._diag3(self.ca)
        raw.angular_velocity_covariance    = self._diag3(self.cw)
        # REP-145: orientation_covariance[0] = -1 means "orientation not provided"
        raw.orientation_covariance = [-1.0] + [0.0] * 8
        self.pub_raw.publish(raw)

        if self.pub_fused is not None:
            fused = Imu()
            fused.header.stamp = stamp
            fused.header.frame_id = self.frame_id
            fused.linear_acceleration = raw.linear_acceleration
            fused.angular_velocity    = raw.angular_velocity
            fused.linear_acceleration_covariance = raw.linear_acceleration_covariance
            fused.angular_velocity_covariance    = raw.angular_velocity_covariance
            fused.orientation.x = float(pkt.get('qx', 0.0))
            fused.orientation.y = float(pkt.get('qy', 0.0))
            fused.orientation.z = float(pkt.get('qz', 0.0))
            fused.orientation.w = float(pkt.get('qw', 1.0))
            fused.orientation_covariance = self._diag3(self.co)
            self.pub_fused.publish(fused)

    def _stats(self):
        self.get_logger().info(f'UDP packets received in last 2s window: {self.n_received}')
        self.n_received = 0


def main():
    rclpy.init()
    node = ImuUdpNode()
    try:
        rclpy.spin(node)
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
