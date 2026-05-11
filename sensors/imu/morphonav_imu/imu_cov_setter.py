"""Republishes /imu/data_madgwick on /imu/data with non-zero covariances.

`imu_filter_madgwick` publishes orientation but leaves the covariance
diagonals at zero. robot_localization treats zero covariance as either
"perfectly known" or "ignore", depending on phase — both are wrong. This
node injects sane diagonal covariances so the EKF accepts the orientation.

Defaults follow the values in the morphonav plan: orientation 0.01, gyro
0.001, accel 0.04 (variance, units rad^2 / (rad/s)^2 / (m/s^2)^2).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuCovSetter(Node):
    def __init__(self):
        super().__init__('imu_cov_setter')

        self.declare_parameter('input_topic',  '/imu/data_madgwick')
        self.declare_parameter('output_topic', '/imu/data')
        self.declare_parameter('cov_orientation',         [0.01, 0.01, 0.01])
        self.declare_parameter('cov_angular_velocity',    [0.001, 0.001, 0.001])
        self.declare_parameter('cov_linear_acceleration', [0.04, 0.04, 0.04])

        in_topic  = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.co = self.get_parameter('cov_orientation').value
        self.cw = self.get_parameter('cov_angular_velocity').value
        self.ca = self.get_parameter('cov_linear_acceleration').value

        self.pub = self.create_publisher(Imu, out_topic, 50)
        self.create_subscription(Imu, in_topic, self._cb, 50)

    def _cb(self, msg: Imu):
        msg.orientation_covariance         = self._diag3(self.co)
        msg.angular_velocity_covariance    = self._diag3(self.cw)
        msg.linear_acceleration_covariance = self._diag3(self.ca)
        self.pub.publish(msg)

    @staticmethod
    def _diag3(v):
        return [v[0], 0.0, 0.0,
                0.0, v[1], 0.0,
                0.0, 0.0, v[2]]


def main():
    rclpy.init()
    node = ImuCovSetter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
