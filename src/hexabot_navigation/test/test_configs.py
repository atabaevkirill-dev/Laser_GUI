"""Navigation configs parse and agree with each other and with the robot."""
import os

import yaml

CFG = os.environ.get('CONFIG_DIR', os.path.join(os.path.dirname(__file__), '..', 'config'))


def load(name):
    with open(os.path.join(CFG, name)) as f:
        return yaml.safe_load(f)


def test_frames_are_consistent():
    nav = load('nav2_params.yaml')
    ekf = load('ekf.yaml')['ekf_filter_node']['ros__parameters']
    slam = load('slam_toolbox.yaml')['slam_toolbox']['ros__parameters']
    assert ekf['base_link_frame'] == slam['base_frame'] == 'base_footprint'
    assert nav['bt_navigator']['ros__parameters']['robot_base_frame'] == 'base_footprint'
    for cm in ('local_costmap', 'global_costmap'):
        assert nav[cm][cm]['ros__parameters']['robot_base_frame'] == 'base_footprint'


def test_holonomic_limits_match_the_gait():
    nav = load('nav2_params.yaml')
    mppi = nav['controller_server']['ros__parameters']['FollowPath']
    assert mppi['motion_model'] == 'Omni'
    # Tripod gait limit: 0.08 m stride / (0.5 * 0.8 s) = 0.2 m/s.
    assert mppi['vx_max'] <= 0.2 and mppi['vy_max'] <= 0.2
    smoother = nav['velocity_smoother']['ros__parameters']
    assert smoother['max_velocity'][0] == mppi['vx_max']
    for node in ('controller_server', 'velocity_smoother', 'behavior_server'):
        assert nav[node]['ros__parameters']['enable_stamped_cmd_vel'] is False


def test_low_obstacles_are_stepped_over():
    nav = load('nav2_params.yaml')
    depth = nav['local_costmap']['local_costmap']['ros__parameters']['voxel_layer']['depth']
    # Must stay above the step height the legs can clear (cad/params.py STEP_HEIGHT_MAX = 80 mm).
    assert 0.08 < depth['min_obstacle_height'] < 0.12


def test_patrol_route():
    patrol = load('patrol.yaml')['patrol']
    assert len(patrol['waypoints']) >= 3
    assert any(w.get('observe') for w in patrol['waypoints'])
