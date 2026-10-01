#!/usr/bin/env python3
"""Generate the Gazebo worlds from sim/course.json (shared with the web sim).

    python3 src/hexabot_gazebo/scripts/generate_world.py

writes worlds/scout_range_day.sdf and worlds/scout_range_night.sdf.
Objects carry temperatures for the thermal camera (gz Thermal system).
"""
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
COURSE = json.loads((ROOT / 'sim' / 'course.json').read_text())


def mulberry32(seed):
    """Same PRNG as the web simulation, so the trees stand in the same places."""
    state = [seed & 0xFFFFFFFF]

    def imul(a, b):
        return (a * b) & 0xFFFFFFFF

    def rnd():
        state[0] = (state[0] + 0x6D2B79F5) & 0xFFFFFFFF
        s = state[0]
        t = imul(s ^ (s >> 15), 1 | s)
        t = ((t + imul(t ^ (t >> 7), 61 | t)) & 0xFFFFFFFF) ^ t
        return ((t ^ (t >> 14)) & 0xFFFFFFFF) / 4294967296
    return rnd


def inside(x, y, pts):
    c = False
    j = len(pts) - 1
    for i in range(len(pts)):
        xi, yi = pts[i]
        xj, yj = pts[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            c = not c
        j = i
    return c


def thermal(kelvin):
    return (f'<plugin filename="gz-sim-thermal-system" name="gz::sim::systems::Thermal">'
            f'<temperature>{kelvin:.1f}</temperature></plugin>')


def material(rgb):
    r, g, b = rgb
    return (f'<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>'
            f'<specular>0.1 0.1 0.1 1</specular></material>')


def box_model(name, x, y, z, sx, sy, sz, rgb, kelvin, yaw=0.0, pitch=0.0, static=True):
    geo = f'<box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box>'
    return f'''
    <model name="{name}">
      <static>{str(static).lower()}</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 {pitch:.5f} {yaw:.4f}</pose>
      <link name="link">
        <collision name="c"><geometry>{geo}</geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
        <visual name="v"><geometry>{geo}</geometry>{material(rgb)}{thermal(kelvin)}</visual>
      </link>
    </model>'''


def person(name, x, y, yaw, rgb):
    return f'''
    <model name="{name}">
      <static>true</static>
      <pose>{x:.2f} {y:.2f} 0 0 0 {yaw:.3f}</pose>
      <link name="link">
        <collision name="c"><pose>0 0 0.87 0 0 0</pose><geometry><cylinder><radius>0.22</radius><length>1.74</length></cylinder></geometry></collision>
        <visual name="legs"><pose>0 0 0.43 0 0 0</pose><geometry><box><size>0.22 0.32 0.86</size></box></geometry>{material((0.17, 0.18, 0.2))}{thermal(303)}</visual>
        <visual name="torso"><pose>0 0 1.17 0 0 0</pose><geometry><box><size>0.26 0.42 0.62</size></box></geometry>{material(rgb)}{thermal(305)}</visual>
        <visual name="head"><pose>0 0 1.6 0 0 0</pose><geometry><sphere><radius>0.11</radius></sphere></geometry>{material((0.78, 0.61, 0.48))}{thermal(309)}</visual>
      </link>
    </model>'''


def drone(d):
    a = d['phase']
    x, y = d['x'] + d['r'] * math.cos(a), d['y'] + d['r'] * math.sin(a)
    rotors = ''.join(
        f'<visual name="rotor{k}"><pose>{0.2 * math.cos(math.pi / 4 + k * math.pi / 2):.3f} '
        f'{0.2 * math.sin(math.pi / 4 + k * math.pi / 2):.3f} 0.03 0 0 0</pose>'
        f'<geometry><cylinder><radius>0.1</radius><length>0.01</length></cylinder></geometry>'
        f'{material((0.6, 0.62, 0.65))}{thermal(318)}</visual>' for k in range(4))
    return f'''
    <model name="drone">
      <static>true</static>
      <pose>{x:.2f} {y:.2f} {d['alt']:.1f} 0 0 {a + math.pi / 2:.3f}</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>0.5 0.5 0.12</size></box></geometry></collision>
        <visual name="body"><geometry><box><size>0.22 0.14 0.08</size></box></geometry>{material((0.16, 0.17, 0.19))}{thermal(306)}</visual>
        <visual name="arms"><geometry><box><size>0.42 0.03 0.02</size></box></geometry>{material((0.16, 0.17, 0.19))}{thermal(300)}</visual>
        {rotors}
      </link>
    </model>'''


def vehicle(v):
    return f'''
    <model name="vehicle">
      <static>true</static>
      <pose>{v['x']} {v['y']} 0 0 0 {v['yaw']}</pose>
      <link name="link">
        <collision name="c"><pose>0 0 0.9 0 0 0</pose><geometry><box><size>4.5 1.9 1.8</size></box></geometry></collision>
        <visual name="body"><pose>0 0 0.78 0 0 0</pose><geometry><box><size>4.5 1.9 0.75</size></box></geometry>{material((0.33, 0.38, 0.42))}{thermal(289)}</visual>
        <visual name="engine"><pose>1.55 0 0.8 0 0 0</pose><geometry><box><size>1.4 1.86 0.6</size></box></geometry>{material((0.33, 0.38, 0.42))}{thermal(333)}</visual>
        <visual name="cabin"><pose>-0.3 0 1.45 0 0 0</pose><geometry><box><size>2.5 1.75 0.6</size></box></geometry>{material((0.1, 0.13, 0.16))}{thermal(287)}</visual>
      </link>
    </model>'''


def tree(name, x, y, h):
    return f'''
    <model name="{name}">
      <static>true</static>
      <pose>{x:.2f} {y:.2f} 0 0 0 0</pose>
      <link name="link">
        <collision name="c"><pose>0 0 {h * 0.5:.2f} 0 0 0</pose><geometry><cylinder><radius>{h * 0.06:.2f}</radius><length>{h:.2f}</length></cylinder></geometry></collision>
        <visual name="trunk"><pose>0 0 {h * 0.22:.2f} 0 0 0</pose><geometry><cylinder><radius>0.18</radius><length>{h * 0.45:.2f}</length></cylinder></geometry>{material((0.36, 0.28, 0.2))}{thermal(284)}</visual>
        <visual name="crown"><pose>0 0 {h * 0.62:.2f} 0 0 0</pose><geometry><cone><radius>{h * 0.28:.2f}</radius><length>{h * 0.75:.2f}</length></cone></geometry>{material((0.33, 0.4, 0.25))}{thermal(283)}</visual>
      </link>
    </model>'''


def world(night, rendering=True):
    c = COURSE
    models = []
    colors = {'rock': (0.49, 0.45, 0.41), 'beam': (0.54, 0.42, 0.27),
              'platform': (0.6, 0.6, 0.57), 'wall': (0.65, 0.65, 0.61)}
    for i, o in enumerate(c['obstacles']):
        models.append(box_model(f'{o["kind"]}_{i}', (o['x0'] + o['x1']) / 2, (o['y0'] + o['y1']) / 2, o['h'] / 2,
                                o['x1'] - o['x0'], o['y1'] - o['y0'], o['h'], colors[o['kind']],
                                281 if night else 295))
    for i, r in enumerate(c['ramps']):
        length = r['x1'] - r['x0']
        if abs(r['h1'] - r['h0']) < 1e-6:
            models.append(box_model(f'ramp_{i}', (r['x0'] + r['x1']) / 2, (r['y0'] + r['y1']) / 2, r['h0'] / 2,
                                    length, r['y1'] - r['y0'], r['h0'], colors['platform'], 281 if night else 295))
        else:
            ang = math.atan2(r['h1'] - r['h0'], length)
            slab = 0.02
            hyp = math.hypot(length, r['h1'] - r['h0'])
            zc = (r['h0'] + r['h1']) / 2 - slab / 2 * math.cos(ang)
            models.append(box_model(f'ramp_{i}', (r['x0'] + r['x1']) / 2, (r['y0'] + r['y1']) / 2, zc,
                                    hyp, r['y1'] - r['y0'], slab, colors['platform'], 281 if night else 295,
                                    pitch=-ang))
    for i, p in enumerate(c['people']):
        (x0, y0), (x1, y1) = p['path'][0], p['path'][1]
        rgb = tuple(int(p['color'][k:k + 2], 16) / 255 for k in (1, 3, 5))
        models.append(person(f'person_{i}', x0, y0, math.atan2(y1 - y0, x1 - x0), rgb))
    models.append(drone(c['drone']))
    models.append(vehicle(c['vehicle']))
    rnd = mulberry32(c['trees_seed'])
    for i in range(c['trees_count']):
        a = rnd() * math.pi * 2
        d = 16 + rnd() * 90
        x, y = math.cos(a) * d + 20, math.sin(a) * d
        if inside(x, y, c['zone']['pts']):
            continue
        h = 4 + rnd() * 5
        models.append(tree(f'tree_{i}', x, y, h))

    if night:
        scene = ('<scene><ambient>0.04 0.05 0.07 1</ambient><background>0.02 0.03 0.05 1</background>'
                 '<shadows>true</shadows></scene>')
        sun = ('<light type="directional" name="moon"><cast_shadows>false</cast_shadows>'
               '<pose>0 0 30 0 0 0</pose><diffuse>0.06 0.07 0.11 1</diffuse><specular>0 0 0 1</specular>'
               '<direction>0.3 0.4 -0.85</direction></light>')
        ambient_k = 279
    else:
        scene = ('<scene><ambient>0.55 0.57 0.6 1</ambient><background>0.77 0.82 0.86 1</background>'
                 '<shadows>true</shadows></scene>')
        sun = ('<light type="directional" name="sun"><cast_shadows>true</cast_shadows>'
               '<pose>0 0 30 0 0 0</pose><diffuse>0.95 0.92 0.85 1</diffuse><specular>0.2 0.2 0.2 1</specular>'
               '<direction>-0.4 0.5 -0.75</direction></light>')
        ambient_k = 293
    name = 'scout_range_night' if night else 'scout_range_day'
    sensors = ('<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">\n'
               '      <render_engine>ogre2</render_engine>\n    </plugin>') if rendering else \
        '<!-- physics only: no rendering sensors (CI, machines without OpenGL) -->'
    if not rendering:
        name = 'scout_range_physics'
    return f'''<?xml version="1.0"?>
<!-- GENERATED by hexabot_gazebo/scripts/generate_world.py from sim/course.json -->
<sdf version="1.9">
  <world name="{name}">
    <physics name="2ms" type="dart">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    {sensors}
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    <atmosphere type="adiabatic"><temperature>{ambient_k}</temperature><temperature_gradient>-0.0065</temperature_gradient></atmosphere>
    <!-- Set to your test site for meaningful GNSS coordinates. -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>0.0</latitude_deg>
      <longitude_deg>0.0</longitude_deg>
      <elevation>0.0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
    {scene}
    {sun}
    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>400 400</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>400 400</size></plane></geometry>{material((0.72, 0.69, 0.62))}{thermal(ambient_k + (-6 if night else 9))}</visual>
      </link>
    </model>
{''.join(models)}
  </world>
</sdf>
'''


if __name__ == '__main__':
    out = HERE.parent / 'worlds'
    out.mkdir(exist_ok=True)
    for fname, night, rendering in (('scout_range_day.sdf', False, True),
                                     ('scout_range_night.sdf', True, True),
                                     ('scout_range_physics.sdf', False, False)):
        path = out / fname
        path.write_text(world(night, rendering))
        print('wrote', path.relative_to(ROOT))
