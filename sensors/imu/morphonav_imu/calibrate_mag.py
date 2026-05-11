"""Magnetometer calibration helper for the MPU9250 / AK8963.

Run:
    ros2 run morphonav_imu calibrate_mag

While the script is running, slowly rotate the IMU through every 3D
orientation for ~30 seconds to sample the full magnetic-field sphere.
On Ctrl-C the script computes hard-iron offsets (sphere centroid) and
soft-iron correction (per-axis scale based on the bounding box; full
ellipsoid fit is out of scope), prints the values, and writes them back
to the ESP32 firmware via micro-ROS parameters.

The mag_axis_remap parameter must already be correct (default "+y,+x,-z"
for a standard MPU9250 module). If yaw still drifts after calibration,
inspect raw /imu/mag during pure rotation about Z — the trace should be
a circle in the X/Y plane. If it's tilted, the remap is wrong.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField


class MagCalibrator(Node):
    def __init__(self):
        super().__init__('calibrate_mag')
        self.declare_parameter('target_node', '/morphonav_esp32')
        self.declare_parameter('topic',       '/imu/mag')
        self.declare_parameter('min_samples', 600)

        self.samples = []  # list of (mx, my, mz) in Tesla
        self.create_subscription(
            MagneticField,
            self.get_parameter('topic').value,
            self._cb, 50,
        )

    def _cb(self, msg: MagneticField):
        m = msg.magnetic_field
        self.samples.append((m.x, m.y, m.z))
        if len(self.samples) % 50 == 0:
            self.get_logger().info(f'collected {len(self.samples)} samples — keep rotating')

    def fit(self):
        if len(self.samples) < int(self.get_parameter('min_samples').value):
            self.get_logger().warn(
                f'only {len(self.samples)} samples — need at least '
                f'{self.get_parameter("min_samples").value}; results will be poor'
            )

        xs = [s[0] for s in self.samples]
        ys = [s[1] for s in self.samples]
        zs = [s[2] for s in self.samples]

        # Hard iron = midpoint of bounding box of each axis (Tesla -> uT for output)
        T_TO_UT = 1e6
        hx = (max(xs) + min(xs)) / 2.0 * T_TO_UT
        hy = (max(ys) + min(ys)) / 2.0 * T_TO_UT
        hz = (max(zs) + min(zs)) / 2.0 * T_TO_UT

        # Soft iron: per-axis scale to equalize ranges
        rx = (max(xs) - min(xs)) / 2.0
        ry = (max(ys) - min(ys)) / 2.0
        rz = (max(zs) - min(zs)) / 2.0
        avg = (rx + ry + rz) / 3.0
        sx = avg / rx if rx > 0 else 1.0
        sy = avg / ry if ry > 0 else 1.0
        sz = avg / rz if rz > 0 else 1.0

        return (hx, hy, hz), (sx, sy, sz)


def push_to_firmware(node, target, hard, soft):
    """Currently the ESP32 firmware exposes only `trigger_calibration` as a
    string parameter. This helper prints the values for the user to copy into
    `firmware/morphonav_esp32/params/default_params.yaml`, plus shows the
    `ros2 param set` commands. A future firmware revision can accept these
    arrays directly via the parameter server.
    """
    print('\n=== Magnetometer calibration result ===')
    print(f'Hard iron offsets (uT):  x={hard[0]:.3f}  y={hard[1]:.3f}  z={hard[2]:.3f}')
    print(f'Soft iron diag scales:   x={soft[0]:.4f} y={soft[1]:.4f} z={soft[2]:.4f}')
    print('\nPaste into firmware/morphonav_esp32/params/default_params.yaml:')
    print(f'  mag_hard_iron: [{hard[0]:.3f}, {hard[1]:.3f}, {hard[2]:.3f}]')
    print(f'  mag_soft_iron: [{soft[0]:.4f}, 0, 0,  0, {soft[1]:.4f}, 0,  0, 0, {soft[2]:.4f}]')
    print('\nThen reflash, or extend microros_bridge.cpp to expose mag_hard_iron/'
          'mag_soft_iron as parameters and use:')
    print('  ros2 param set ' + target + ' mag_hard_iron "[...]"')


def main():
    rclpy.init()
    node = MagCalibrator()
    print('Slowly rotate the IMU through all axes. Press Ctrl-C to stop.')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    target = node.get_parameter('target_node').value
    hard, soft = node.fit()
    push_to_firmware(node, target, hard, soft)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
