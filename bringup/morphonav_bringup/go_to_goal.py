#!/usr/bin/env python3
"""
go_to_goal.py — CLI Nav2 goal sender.

Sends a NavigateToPose action goal to Nav2. Frame is `map`.

Usage:
    python3 go_to_goal.py X Y                  # real robot
    python3 go_to_goal.py X Y --sim            # simulation
    python3 go_to_goal.py X Y YAW              # specify heading (radians)
    python3 go_to_goal.py X Y --frame odom     # change reference frame

--sim sets use_sim_time=true on this node so the goal timestamp aligns with
the Gazebo clock; Nav2's action server otherwise rejects stale stamps.
"""

import argparse
import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class GoToGoal(Node):
    def __init__(self, use_sim_time: bool):
        super().__init__('go_to_goal')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.parameter.Parameter.Type.BOOL, use_sim_time)])
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send(self, x: float, y: float, yaw: float, frame: str) -> int:
        self.get_logger().info(
            f'Waiting for navigate_to_pose action server (use_sim_time={self.get_parameter("use_sim_time").value})...')
        if not self.client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('Action server not available after 15 s.')
            return 2

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f'Sending goal: frame={frame}  x={x:.3f}  y={y:.3f}  yaw={yaw:.3f} rad')
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if not handle or not handle.accepted:
            self.get_logger().error('Goal rejected by server.')
            return 3

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        status = result_future.result().status
        self.get_logger().info(f'Action finished with status code {status}')
        # 4 = STATUS_SUCCEEDED in action_msgs/GoalStatus
        return 0 if status == 4 else 1


def main():
    parser = argparse.ArgumentParser(description='Send a Nav2 NavigateToPose goal.')
    parser.add_argument('x',   type=float, help='X coordinate in target frame (m)')
    parser.add_argument('y',   type=float, help='Y coordinate in target frame (m)')
    parser.add_argument('yaw', type=float, nargs='?', default=0.0,
                        help='Yaw in radians (optional, default 0)')
    parser.add_argument('--sim',   action='store_true',
                        help='Use simulation time (set use_sim_time=true)')
    parser.add_argument('--frame', default='map',
                        help='Reference frame for the goal (default: map)')
    args = parser.parse_args()

    rclpy.init()
    node = GoToGoal(use_sim_time=args.sim)
    try:
        rc = node.send(args.x, args.y, args.yaw, args.frame)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
