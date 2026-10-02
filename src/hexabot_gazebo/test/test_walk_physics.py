"""Launch test: Hexabot stands up and walks in Gazebo (physics-only world).

Runs sim.launch.py world:=physics headless:=true and scripts/walk_check.py,
which compares the motion against Gazebo ground truth.  Skipped when Gazebo
(the `gz` command) is not installed.
"""
import os
import shutil
import unittest

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

if shutil.which('gz') is None:
    pytest.skip('Gazebo (gz) not installed', allow_module_level=True)


@pytest.mark.launch_test
def generate_test_description():
    share = get_package_share_directory('hexabot_gazebo')
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'sim.launch.py')),
        launch_arguments={'world': 'physics', 'headless': 'true'}.items())
    check = ExecuteProcess(
        cmd=[os.path.join(get_package_prefix('hexabot_gazebo'), 'lib', 'hexabot_gazebo', 'walk_check.py')],
        output='screen')
    return launch.LaunchDescription([
        # Gazebo transport ignores ROS_DOMAIN_ID: keep other gz servers out.
        SetEnvironmentVariable('GZ_PARTITION', f'hexabot_test_{os.getpid()}'),
        sim,
        TimerAction(period=3.0, actions=[check]),
        launch_testing.actions.ReadyToTest(),
    ]), {'check': check}


class TestWalk(unittest.TestCase):
    def test_walk_check_finishes(self, proc_info, check):
        proc_info.assertWaitForShutdown(process=check, timeout=200)


@launch_testing.post_shutdown_test()
class TestWalkResult(unittest.TestCase):
    def test_walk_check_passed(self, proc_info, check):
        launch_testing.asserts.assertExitCodes(proc_info, process=check)
