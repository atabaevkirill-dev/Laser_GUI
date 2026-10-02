"""The xacro model expands for every hardware flavour and matches geometry.yaml."""
import math
import os
import subprocess
import xml.etree.ElementTree as ET

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

SHARE = get_package_share_directory('hexabot_description')
XACRO = os.path.join(SHARE, 'urdf', 'hexabot.urdf.xacro')
LEGS = ['lf', 'lm', 'lr', 'rf', 'rm', 'rr']


def expand(*args):
    out = subprocess.run(['xacro', XACRO, *args], check=True, capture_output=True, text=True)
    return ET.fromstring(out.stdout)


def geometry():
    with open(os.path.join(SHARE, 'config', 'geometry.yaml')) as f:
        return yaml.safe_load(f)['/**']['ros__parameters']['geometry']


@pytest.mark.parametrize('args, plugin', [
    ((), 'hexabot_hardware/Sts3215System'),
    (('use_mock_hardware:=true',), 'mock_components/GenericSystem'),
    (('use_sim:=true', 'controllers_file:=/tmp/c.yaml'), 'gz_ros2_control/GazeboSimSystem'),
])
def test_hardware_plugin(args, plugin):
    robot = expand(*args)
    plugins = [p.text for p in robot.iter('plugin') if p.text and '/' in p.text]
    assert plugin in plugins
    joints = robot.find('ros2_control').findall('joint')
    assert len(joints) == 20


def test_joint_tree():
    robot = expand('use_mock_hardware:=true')
    joints = {j.get('name'): j for j in robot.findall('joint')}
    for leg in LEGS:
        for part in ('coxa', 'femur', 'tibia'):
            j = joints[f'{leg}_{part}_joint']
            assert j.get('type') == 'revolute'
            axis = [float(v) for v in j.find('axis').get('xyz').split()]
            assert axis == ([0, 0, 1] if part == 'coxa' else [0, -1, 0])
        assert f'{leg}_foot_joint' in joints
    for name in ('head_pan_joint', 'head_tilt_joint', 'lrf_joint', 'laser_joint',
                 'depth_camera_joint', 'imu_joint', 'thermal_camera_joint'):
        assert name in joints


def test_leg_chain_matches_geometry():
    g = geometry()
    robot = expand('use_mock_hardware:=true')
    joints = {j.get('name'): j for j in robot.findall('joint')}
    for i, leg in enumerate(LEGS):
        o = joints[f'{leg}_coxa_joint'].find('origin')
        xyz = [float(v) for v in o.get('xyz').split()]
        rpy = [float(v) for v in o.get('rpy').split()]
        assert xyz[0] == pytest.approx(g['mount_x'][i])
        assert xyz[1] == pytest.approx(g['mount_y'][i])
        assert rpy[2] == pytest.approx(g['mount_yaw'][i])
        fem = [float(v) for v in joints[f'{leg}_femur_joint'].find('origin').get('xyz').split()]
        tib = [float(v) for v in joints[f'{leg}_tibia_joint'].find('origin').get('xyz').split()]
        foot = [float(v) for v in joints[f'{leg}_foot_joint'].find('origin').get('xyz').split()]
        assert fem[0] == pytest.approx(g['coxa_length'])
        assert tib[0] == pytest.approx(g['femur_length'])
        assert foot[2] == pytest.approx(-g['tibia_length'])
        lim = joints[f'{leg}_tibia_joint'].find('limit')
        assert float(lim.get('lower')) == pytest.approx(g['tibia_limits'][0])


def test_servo_ids_are_unique():
    robot = expand()
    ids = [int(p.text) for j in robot.find('ros2_control').findall('joint')
           for p in j.findall('param') if p.get('name') == 'id']
    assert sorted(ids) == list(range(1, 21))


def test_initial_pose_is_the_neutral_stance():
    """Mock/sim start standing: initial joint values equal the IK of the stance."""
    g = geometry()
    L1, L2, L3 = g['coxa_length'], g['femur_length'], g['tibia_length']
    r, z = g['stance_radius'] - L1, -g['body_height']
    c = (r * r + z * z - L2 * L2 - L3 * L3) / (2 * L2 * L3)
    knee = -math.acos(c)
    femur = math.atan2(z, r) - math.atan2(L3 * math.sin(knee), L2 + L3 * math.cos(knee))
    robot = expand('use_mock_hardware:=true')
    for j in robot.find('ros2_control').findall('joint'):
        if not j.get('name').endswith('femur_joint'):
            continue
        init = float(j.find("state_interface[@name='position']/param").text)
        assert init == pytest.approx(femur, abs=2e-3)
