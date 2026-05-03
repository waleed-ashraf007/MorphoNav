#!/usr/bin/env python3
"""
MorphoNav Teleop — teleop_twist_keyboard style with auto-stop.

Hold a key to move, release for 0.5 s and the robot stops automatically.
No accumulated state — what you press is what you get.

Layout (matches the well-known teleop_twist_keyboard):

         u    i    o
         j    k    l       k or SPACE = full stop
         m    ,    .

   i / ,  : forward / reverse
   j / l  : turn left / right (in place)
   u o m .: forward+turn / reverse+turn (curve)

Speed scaling:
   q / z  : speed up / slow down both linear and angular by 10%
   w / x  : linear only
   e / c  : angular only

Quit:
   CTRL-C
"""

import sys
import select
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


BANNER = """
╔════════════════════════════════════════════════════╗
║  MorphoNav Teleop  (teleop_twist style)            ║
╠════════════════════════════════════════════════════╣
║   u    i    o      i / ,  forward / reverse        ║
║   j    k    l      j / l  turn left / right        ║
║   m    ,    .      u o m. curving moves            ║
║                                                    ║
║   k or SPACE  full stop                            ║
║   q / z       speed   ±10%  (both)                 ║
║   w / x       speed   ±10%  (linear only)          ║
║   e / c       speed   ±10%  (angular only)         ║
║   CTRL-C      quit                                 ║
║                                                    ║
║   Hold key to move, release ⇒ auto-stop in 0.5 s   ║
╚════════════════════════════════════════════════════╝
"""

# (key) -> (linear factor, angular factor)  in -1..+1
MOVE_BINDINGS = {
    'i': ( 1.0,  0.0), ',': (-1.0,  0.0),
    'j': ( 0.0,  1.0), 'l': ( 0.0, -1.0),
    'u': ( 1.0,  1.0), 'o': ( 1.0, -1.0),
    'm': (-1.0,  1.0), '.': (-1.0, -1.0),
}

# (key) -> (lin_scale, ang_scale)
SPEED_BINDINGS = {
    'q': (1.10, 1.10),
    'z': (0.90, 0.90),
    'w': (1.10, 1.00),
    'x': (0.90, 1.00),
    'e': (1.00, 1.10),
    'c': (1.00, 0.90),
}

STOP_KEYS = (' ', 'k')
CTRL_C    = '\x03'


def read_key(timeout=0.05):
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        return sys.stdin.read(1)
    return ''


class TwistTeleop(Node):

    KEY_TIMEOUT = 0.5   # seconds of no movement key → publish 0 twist

    def __init__(self):
        super().__init__('twist_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lin_speed = 0.5    # m/s   (max linear, scaled by q/z/w/x)
        self.ang_speed = 1.0    # rad/s (max angular, scaled by q/z/e/c)
        self.cur_lin = 0.0      # current factor in -1..+1
        self.cur_ang = 0.0

    def publish(self):
        msg = Twist()
        msg.linear.x  = self.cur_lin * self.lin_speed
        msg.angular.z = self.cur_ang * self.ang_speed
        self.pub.publish(msg)

    def status(self):
        v = self.cur_lin * self.lin_speed
        w = self.cur_ang * self.ang_speed
        print(
            f'\r  max v={self.lin_speed:.2f} m/s  ω={self.ang_speed:.2f} rad/s  '
            f'│  cmd  v={v:+.2f}  ω={w:+.2f}     ',
            end='', flush=True
        )

    def run(self):
        print(BANNER)
        settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            last_move_key = 0.0
            while rclpy.ok():
                key = read_key(0.05)
                now = time.time()

                if key == CTRL_C:
                    break

                if key in MOVE_BINDINGS:
                    self.cur_lin, self.cur_ang = MOVE_BINDINGS[key]
                    last_move_key = now

                elif key in SPEED_BINDINGS:
                    s_lin, s_ang = SPEED_BINDINGS[key]
                    self.lin_speed = max(0.05, min(self.lin_speed * s_lin, 5.0))
                    self.ang_speed = max(0.05, min(self.ang_speed * s_ang, 5.0))

                elif key in STOP_KEYS:
                    self.cur_lin = 0.0
                    self.cur_ang = 0.0
                    last_move_key = 0.0

                # auto-stop if user isn't holding any movement key
                if now - last_move_key > self.KEY_TIMEOUT:
                    self.cur_lin = 0.0
                    self.cur_ang = 0.0

                self.publish()
                self.status()
                rclpy.spin_once(self, timeout_sec=0.0)

        finally:
            self.cur_lin = 0.0
            self.cur_ang = 0.0
            self.publish()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            print('\n  Stopped. Terminal restored.')


def main(args=None):
    if not sys.stdin.isatty():
        print('teleop requires a terminal.')
        return
    rclpy.init(args=args)
    node = TwistTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
