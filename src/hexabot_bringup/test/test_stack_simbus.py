"""Integration test: real driver code on a simulated servo bus.

robot_state_publisher + ros2_control (Sts3215System, simulated_bus) +
controllers + locomotion.  The robot must stand up by itself and walk
forward when a velocity is commanded.
"""
import os
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from hexabot_interfaces.msg import LocomotionState


@pytest.mark.launch_test
def generate_test_description():
    share = get_package_share_directory('hexabot_bringup')
    robot = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'robot.launch.py')),
        launch_arguments={'hardware': 'simbus', 'start_mode': 'stand'}.items())
    return launch.LaunchDescription([robot, launch_testing.actions.ReadyToTest()])


class TestStack(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('stack_test')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, predicate, timeout):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def test_stand_and_walk(self):
        state = {}
        odom = {}
        joints = {}
        self.node.create_subscription(LocomotionState, '/locomotion/state',
                                      lambda m: state.update(mode=m.mode, walking=m.walking), 10)
        self.node.create_subscription(Odometry, '/odom',
                                      lambda m: odom.update(x=m.pose.pose.position.x), 10)
        self.node.create_subscription(JointState, '/joint_states',
                                      lambda m: joints.update(dict(zip(m.name, m.position))), 10)
        self.assertTrue(self.spin_until(lambda: state.get('mode') == 'standing', 60),
                        f'robot did not stand up, last state {state}')
        self.assertEqual(len([n for n in joints if n.endswith('_joint')]), 20)
        pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        cmd = Twist()
        cmd.linear.x = 0.1
        x0 = odom.get('x', 0.0)
        end = time.time() + 4.0
        while time.time() < end:
            pub.publish(cmd)
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(state.get('walking'), 'gait did not start')
        self.assertGreater(odom.get('x', 0.0) - x0, 0.1, f'odometry did not move forward: {odom}')
