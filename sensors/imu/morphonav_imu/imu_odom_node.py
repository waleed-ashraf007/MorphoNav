"""IMU-only odometry for the standalone Parts-1-6 path.

Subscribes /imu (sensor_msgs/Imu with on-device fused orientation) and:
  1. Rotates body-frame acceleration into the world frame via the quaternion
  2. Removes gravity (9.81 m/s^2 on world Z)
  3. Low-pass filters the world-frame acceleration
  4. Applies a noise-floor threshold
  5. Integrates acceleration -> velocity -> position with dt from message stamps
  6. ZUPT: when |a| ~ g and |omega| ~ 0, snap velocity to zero
  7. Publishes nav_msgs/Odometry on /imu/odom
  8. Broadcasts TF odom -> base_link

DO NOT run this node alongside slam_toolbox / robot_localization — they own
odom -> base_footprint and a competing publisher will fight them. This node
is intended for the standalone test launch (imu_udp_standalone.launch.py)
where neither EKF nor SLAM is active.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


GRAVITY = 9.80665


def quat_to_rotmat(qx, qy, qz, qw):
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if n < 1e-12:
        return np.eye(3)
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n
    return np.array([
        [1 - 2*(qy*qy + qz*qz),   2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),       1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),       2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)],
    ])


class ImuOdomNode(Node):
    def __init__(self):
        super().__init__('imu_odom_node')

        self.declare_parameter('input_topic',  '/imu')
        self.declare_parameter('odom_topic',   '/imu/odom')
        self.declare_parameter('odom_frame',   'odom')
        self.declare_parameter('child_frame',  'base_link')
        self.declare_parameter('publish_tf',   True)

        # Freeze position at origin and only animate rotation. Useful when
        # the IMU is uncalibrated and accel double-integration runs away —
        # gives a clean visual of orientation in RViz without translation
        # drift sending the arrow off-screen.
        self.declare_parameter('freeze_position', False)

        self.declare_parameter('lpf_alpha',         0.2)   # 0=heavy, 1=none
        self.declare_parameter('accel_threshold',   0.05)  # m/s^2 noise floor
        self.declare_parameter('zupt_accel_window', 0.3)   # |a|-g tolerance
        self.declare_parameter('zupt_gyro_window',  0.05)  # rad/s
        self.declare_parameter('zupt_min_count',    10)    # samples to confirm

        self.input_topic = self.get_parameter('input_topic').value
        self.odom_topic  = self.get_parameter('odom_topic').value
        self.odom_frame  = self.get_parameter('odom_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        self.publish_tf  = bool(self.get_parameter('publish_tf').value)
        self.freeze_position = bool(self.get_parameter('freeze_position').value)

        self.alpha = float(self.get_parameter('lpf_alpha').value)
        self.a_thresh = float(self.get_parameter('accel_threshold').value)
        self.zupt_a = float(self.get_parameter('zupt_accel_window').value)
        self.zupt_w = float(self.get_parameter('zupt_gyro_window').value)
        self.zupt_n = int(self.get_parameter('zupt_min_count').value)

        self.pub  = self.create_publisher(Odometry, self.odom_topic, 30)
        self.sub  = self.create_subscription(Imu, self.input_topic, self._cb,
                                             qos_profile_sensor_data)
        self.tf_b = TransformBroadcaster(self) if self.publish_tf else None

        self.last_t = None
        self.vel = np.zeros(3)
        self.pos = np.zeros(3)
        self.a_lpf = np.zeros(3)
        self.zupt_count = 0
        self.last_quat = (0.0, 0.0, 0.0, 1.0)

    def _cb(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_t is None:
            self.last_t = t
            return
        dt = t - self.last_t
        self.last_t = t
        if dt <= 0.0 or dt > 0.5:
            return  # bad stamp / dropout — skip this step

        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        if (qx*qx + qy*qy + qz*qz + qw*qw) < 1e-9:
            # Orientation not provided (raw IMU). This node needs orientation.
            return
        self.last_quat = (qx, qy, qz, qw)

        a_body = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ])
        w_body = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ])

        R = quat_to_rotmat(qx, qy, qz, qw)
        a_world = R @ a_body
        a_world[2] -= GRAVITY                                # remove gravity

        self.a_lpf = self.alpha * a_world + (1.0 - self.alpha) * self.a_lpf
        a = self.a_lpf.copy()
        a[np.abs(a) < self.a_thresh] = 0.0                   # noise floor

        # ZUPT: detect rest condition via |a_body| ≈ g AND ||omega|| ≈ 0
        a_mag = float(np.linalg.norm(a_body))
        w_mag = float(np.linalg.norm(w_body))
        at_rest = (abs(a_mag - GRAVITY) < self.zupt_a) and (w_mag < self.zupt_w)
        if at_rest:
            self.zupt_count += 1
            if self.zupt_count >= self.zupt_n:
                self.vel[:] = 0.0
                a[:] = 0.0
        else:
            self.zupt_count = 0

        self.vel += a * dt
        self.pos += self.vel * dt

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.child_frame
        odom.pose.pose.position.x = float(self.pos[0])
        odom.pose.pose.position.y = float(self.pos[1])
        odom.pose.pose.position.z = float(self.pos[2])
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = float(self.vel[0])
        odom.twist.twist.linear.y = float(self.vel[1])
        odom.twist.twist.linear.z = float(self.vel[2])
        odom.twist.twist.angular.x = float(w_body[0])
        odom.twist.twist.angular.y = float(w_body[1])
        odom.twist.twist.angular.z = float(w_body[2])
        self.pub.publish(odom)

        if self.tf_b is not None:
            tf = TransformStamped()
            tf.header.stamp = msg.header.stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id  = self.child_frame
            tf.transform.translation.x = float(self.pos[0])
            tf.transform.translation.y = float(self.pos[1])
            tf.transform.translation.z = float(self.pos[2])
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_b.sendTransform(tf)


def main():
    rclpy.init()
    node = ImuOdomNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
