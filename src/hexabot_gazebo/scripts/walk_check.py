#!/usr/bin/env python3
"""Drive the simulated robot and compare leg odometry with Gazebo ground truth.

    ros2 run hexabot_gazebo walk_check.py     (with sim.launch.py running)

Waits until the robot stands, walks forward, strafes and turns, then prints
the distance travelled in Gazebo and the odometry error.  Exit code 1 if the
robot did not move or fell over.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from hexabot_interfaces.msg import LocomotionState


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Check(Node):
    def __init__(self):
        super().__init__('walk_check')
        self.state = None
        self.truth = None
        self.odom = None
        self.create_subscription(LocomotionState, '/locomotion/state', lambda m: setattr(self, 'state', m), 10)
        self.create_subscription(Odometry, '/ground_truth/odom', lambda m: setattr(self, 'truth', m.pose.pose), 10)
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom', m.pose.pose), 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def spin_for(self, seconds, cmd=None):
        end = time.time() + seconds
        while time.time() < end:
            if cmd is not None:
                self.pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    rclpy.init()
    n = Check()
    t0 = time.time()
    while time.time() - t0 < 120 and not (n.state and n.state.mode == 'standing' and n.truth):
        n.spin_for(0.5)
    if not (n.state and n.state.mode == 'standing'):
        print('robot did not stand up')
        sys.exit(1)
    n.spin_for(2.0)
    start, odom0 = n.truth, n.odom
    print(f'standing: base_link z = {start.position.z:.3f} m')
    legs = [('forward 0.12 m/s', Twist(), 10.0), ('sideways 0.06 m/s', Twist(), 6.0), ('turn 0.4 rad/s', Twist(), 6.0)]
    legs[0][1].linear.x = 0.12
    legs[1][1].linear.y = 0.06
    legs[2][1].angular.z = 0.4
    for name, cmd, dur in legs:
        a = n.truth
        n.spin_for(dur, cmd)
        n.spin_for(1.5, Twist())
        b = n.truth
        d = math.hypot(b.position.x - a.position.x, b.position.y - a.position.y)
        dyaw = math.degrees(math.atan2(math.sin(yaw_of(b.orientation) - yaw_of(a.orientation)),
                                       math.cos(yaw_of(b.orientation) - yaw_of(a.orientation))))
        print(f'{name:18s} moved {d:.3f} m, turned {dyaw:+.1f} deg, height {b.position.z:.3f} m')
    end = n.truth
    travelled = math.hypot(end.position.x - start.position.x, end.position.y - start.position.y)
    if n.odom and odom0:
        o = math.hypot(n.odom.position.x - odom0.position.x, n.odom.position.y - odom0.position.y)
        print(f'total: ground truth {travelled:.3f} m, leg odometry {o:.3f} m')
    tilt = abs(n.state.roll) + abs(n.state.pitch)
    ok = travelled > 0.5 and end.position.z > 0.06 and tilt < 0.3
    print('PASS' if ok else 'FAIL')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
