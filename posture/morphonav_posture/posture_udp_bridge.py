"""Posture UDP bridge — ROS → ESP32.

Subscribes to /robot_posture (std_msgs/String) and forwards each mode
change to the embedded servo ESP32 as a UDP/JSON packet:

    {"posture": "crouch"}
    {"posture": "normal"}

The ESP32 target IP and port are ROS2 parameters so you can change them
without touching the code.

Parameters
----------
target_ip   : str  IP address of the servo ESP32 (default '192.168.1.100')
target_port : int  UDP port the ESP32 listens on   (default 9999)
"""

import json
import socket

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PostureUdpBridge(Node):
    def __init__(self):
        super().__init__('posture_udp_bridge')

        self.declare_parameter('target_ip',   '192.168.1.100')
        self.declare_parameter('target_port', 9999)

        self._ip   = self.get_parameter('target_ip').value
        self._port = int(self.get_parameter('target_port').value)

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.create_subscription(String, '/robot_posture', self._posture_cb, 10)

        self.get_logger().info(
            f'posture_udp_bridge ready  target={self._ip}:{self._port}')

    def _posture_cb(self, msg: String):
        packet = json.dumps({'posture': msg.data}).encode()
        try:
            self._sock.sendto(packet, (self._ip, self._port))
            self.get_logger().info(f'Sent → {packet.decode()}')
        except OSError as e:
            self.get_logger().error(f'UDP send failed: {e}')

    def destroy_node(self):
        self._sock.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = PostureUdpBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
