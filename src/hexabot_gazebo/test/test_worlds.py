"""Generated worlds are valid XML and contain the whole course."""
import json
import os
import xml.etree.ElementTree as ET

import pytest

WORLDS = os.environ.get('WORLD_DIR', os.path.join(os.path.dirname(__file__), '..', 'worlds'))
COURSE = os.environ.get('COURSE', os.path.join(os.path.dirname(__file__), '..', '..', '..', 'sim', 'course.json'))


@pytest.mark.parametrize('name', ['scout_range_day.sdf', 'scout_range_night.sdf', 'scout_range_physics.sdf'])
def test_world(name):
    root = ET.parse(os.path.join(WORLDS, name)).getroot()
    world = root.find('world')
    models = {m.get('name') for m in world.findall('model')}
    with open(COURSE) as f:
        course = json.load(f)
    for i, o in enumerate(course['obstacles']):
        assert f'{o["kind"]}_{i}' in models
    assert {'ground', 'drone', 'vehicle'} <= models
    assert sum(1 for m in models if m.startswith('person_')) == len(course['people'])
    plugins = {p.get('filename') for p in world.findall('plugin')}
    assert {'gz-sim-physics-system', 'gz-sim-imu-system', 'gz-sim-navsat-system'} <= plugins
    assert ('gz-sim-sensors-system' in plugins) == ('physics' not in name)
    # Every visual has a temperature for the thermal camera.
    for v in world.iter('visual'):
        assert v.find('plugin') is not None
