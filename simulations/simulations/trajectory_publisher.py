#!/usr/bin/env python3
"""
Trajectory Publisher — Gazebo simulation version
Subscribes to /odom, builds a nav_msgs/Path of past poses,
publishes to /trajectory so RViz draws a continuous trail.

use_sim_time: true — reads Gazebo clock, not wall clock
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class TrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('trajectory_publisher')

        # Max poses to keep (0 = infinite trail)
        self.declare_parameter('max_poses', 0)
        self.max_poses = self.get_parameter('max_poses').value

        self.path = Path()
        self.path.header.frame_id = 'odom'

        self.sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.pub = self.create_publisher(Path, '/trajectory', 10)

        self.get_logger().info(
            f'Trajectory publisher started — max_poses={self.max_poses or "unlimited"}')

    def odom_callback(self, msg: Odometry):
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = 'odom'
        pose.pose = msg.pose.pose

        self.path.poses.append(pose)

        # Trim oldest poses if limit is set
        if self.max_poses > 0 and len(self.path.poses) > self.max_poses:
            self.path.poses = self.path.poses[-self.max_poses:]

        self.path.header.stamp = msg.header.stamp
        self.pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
