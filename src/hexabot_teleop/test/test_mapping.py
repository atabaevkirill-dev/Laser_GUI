from hexabot_teleop.mapping import Mapper, deadband, next_gait
from hexabot_teleop.teleop_node import PARAMS


def joy(axes=None, pressed=()):
    a = [0.0] * 6
    for i, v in (axes or {}).items():
        a[i] = v
    b = [0] * 15
    for i in pressed:
        b[i] = 1
    return a, b


def test_deadman_required_for_motion():
    m = Mapper(PARAMS)
    cmd = m.update(*joy({1: 1.0}))
    assert cmd.vx == 0.0
    cmd = m.update(*joy({1: 1.0, 0: -1.0, 2: 1.0}, pressed=[9]))
    assert cmd.vx == PARAMS['max_forward']
    assert cmd.vy == -PARAMS['max_sideways']       # stick right -> move right
    assert cmd.wz == PARAMS['max_turn']             # stick left -> turn left


def test_fast_button_and_deadband():
    m = Mapper(PARAMS)
    cmd = m.update(*joy({1: 1.0}, pressed=[9, 10]))
    assert cmd.vx == PARAMS['max_forward'] * PARAMS['fast_factor']
    assert deadband(0.05) == 0.0 and deadband(1.0) == 1.0


def test_buttons_fire_once_per_press():
    m = Mapper(PARAMS)
    assert m.update(*joy(pressed=[0])).actions == ['stand_sit']
    assert m.update(*joy(pressed=[0])).actions == []
    assert m.update(*joy()).actions == []
    assert set(m.update(*joy(pressed=[2, 3, 6])).actions) == {'next_gait', 'illuminator', 'measure'}


def test_dpad_moves_head():
    m = Mapper(PARAMS)
    cmd = m.update(*joy(pressed=[11, 13]))
    assert cmd.tilt_rate > 0 and cmd.pan_rate > 0


def test_gait_cycle():
    assert next_gait('tripod') == 'ripple'
    assert next_gait('wave') == 'tripod'
    assert next_gait(None) == 'tripod'
