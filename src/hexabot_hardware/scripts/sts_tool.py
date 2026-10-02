#!/usr/bin/env python3
"""Set up and check the Feetech STS3215 servo bus of Hexabot.

    sts_tool scan                      list servos that answer (ids 0..253)
    sts_tool set-id OLD NEW            change a servo id (one servo on the bus!)
    sts_tool center ID|all             move to step 2048 before fitting the horn
    sts_tool read [ID ...]             position / load / voltage / temperature
    sts_tool torque on|off [ID ...]
    sts_tool move ID STEPS
    sts_tool calibrate --out FILE      record offsets with the robot in the
                                       calibration pose (see docs/assembly.md)

Common options: --port /dev/ttyUSB0 --baud 1000000
"""
import argparse
import math
import sys
import time

try:
    import serial
except ImportError:  # pragma: no cover
    sys.exit('pyserial is missing: sudo apt install python3-serial')

PING, READ, WRITE, SYNC_WRITE = 0x01, 0x02, 0x03, 0x83
ADDR_ID, ADDR_LOCK, ADDR_TORQUE, ADDR_GOAL, ADDR_POS = 5, 55, 40, 42, 56

# Calibration pose: every leg straight out, femur horizontal, tibia vertical
# (all joint angles zero), head looking forward.
JOINTS = [f'{leg}_{part}_joint' for leg in ('lf', 'lm', 'lr', 'rf', 'rm', 'rr')
          for part in ('coxa', 'femur', 'tibia')] + ['head_pan_joint', 'head_tilt_joint']
DIRECTIONS = {'coxa': 1, 'femur': -1, 'tibia': -1, 'pan': 1, 'tilt': -1}


def checksum(body):
    return (~sum(body)) & 0xFF


def packet(sid, instr, params=()):
    body = [sid, len(params) + 2, instr, *params]
    return bytes([0xFF, 0xFF, *body, checksum(body)])


class Bus:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.02)

    def transact(self, sid, instr, params=(), expect=True):
        self.ser.reset_input_buffer()
        self.ser.write(packet(sid, instr, params))
        if not expect or sid == 0xFE:
            return None
        head = self.ser.read(5)
        if len(head) < 5 or head[0] != 0xFF or head[1] != 0xFF:
            return None
        rest = self.ser.read(head[3] - 1)
        data = rest[:-1]
        if len(rest) != head[3] - 1 or rest[-1] != checksum([head[2], head[3], head[4], *data]):
            return None
        return data

    def ping(self, sid):
        return self.transact(sid, PING) is not None

    def read(self, sid, addr, n):
        return self.transact(sid, READ, (addr, n))

    def write(self, sid, addr, data):
        return self.transact(sid, WRITE, (addr, *data))


def u16(v):
    return [v & 0xFF, (v >> 8) & 0xFF]


def state(bus, sid):
    d = bus.read(sid, ADDR_POS, 8)
    if d is None:
        return None
    sign = lambda v, b: -(v & ((1 << b) - 1)) if v & (1 << b) else v  # noqa: E731
    return {
        'position': d[0] | d[1] << 8,
        'speed': sign(d[2] | d[3] << 8, 15),
        'load': sign(d[4] | d[5] << 8, 10) / 10.0,
        'voltage': d[6] / 10.0,
        'temperature': d[7],
    }


def ids_arg(values):
    return list(range(1, 21)) if not values or values == ['all'] else [int(v) for v in values]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default='/dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=1000000)
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('scan')
    p = sub.add_parser('set-id'); p.add_argument('old', type=int); p.add_argument('new', type=int)
    p = sub.add_parser('center'); p.add_argument('ids', nargs='*')
    p = sub.add_parser('read'); p.add_argument('ids', nargs='*')
    p = sub.add_parser('torque'); p.add_argument('state', choices=['on', 'off']); p.add_argument('ids', nargs='*')
    p = sub.add_parser('move'); p.add_argument('id', type=int); p.add_argument('steps', type=int)
    p = sub.add_parser('calibrate'); p.add_argument('--out', default='servo_calibration.yaml')
    a = ap.parse_args(argv)
    bus = Bus(a.port, a.baud)

    if a.cmd == 'scan':
        found = [i for i in range(254) if bus.ping(i)]
        print('servos:', ' '.join(map(str, found)) or 'none')
        missing = sorted(set(range(1, 21)) - set(found))
        if missing:
            print('missing for Hexabot (1..20):', ' '.join(map(str, missing)))
    elif a.cmd == 'set-id':
        if not bus.ping(a.old):
            sys.exit(f'no servo with id {a.old}')
        if bus.ping(a.new):
            sys.exit(f'id {a.new} is already taken')
        bus.write(a.old, ADDR_LOCK, [0])         # unlock EEPROM
        bus.write(a.old, ADDR_ID, [a.new])
        bus.write(a.new, ADDR_LOCK, [1])         # lock again
        time.sleep(0.05)
        print('ok' if bus.ping(a.new) else 'failed: new id does not answer')
    elif a.cmd == 'center':
        for sid in ids_arg(a.ids):
            bus.write(sid, ADDR_TORQUE, [1])
            bus.write(sid, ADDR_GOAL, u16(2048))
            print(f'{sid}: centred')
    elif a.cmd == 'read':
        for sid in ids_arg(a.ids):
            s = state(bus, sid)
            print(f'{sid:3d}: ' + ('no answer' if s is None else
                  '{position:4d} steps  {speed:+5d} st/s  load {load:+6.1f} %  '
                  '{voltage:4.1f} V  {temperature:3d} C'.format(**s)))
    elif a.cmd == 'torque':
        for sid in ids_arg(a.ids):
            bus.write(sid, ADDR_TORQUE, [1 if a.state == 'on' else 0])
    elif a.cmd == 'move':
        bus.write(a.id, ADDR_TORQUE, [1])
        bus.write(a.id, ADDR_GOAL, u16(max(0, min(4095, a.steps))))
    elif a.cmd == 'calibrate':
        input('Torque will be switched OFF. Put the robot in the calibration pose '
              '(legs straight out, femurs horizontal, tibias vertical, head forward), '
              'then press Enter... ')
        lines = ['# Generated by sts_tool calibrate', 'servos:']
        for i, name in enumerate(JOINTS):
            sid = i + 1
            bus.write(sid, ADDR_TORQUE, [0])
            s = state(bus, sid)
            if s is None:
                print(f'{name} (id {sid}) does not answer, offset left at 0')
                steps = 2048
            else:
                steps = s['position']
            key = name.split('_')[1] if not name.startswith('head') else name.split('_')[1]
            direction = DIRECTIONS[key]
            # Joint angle is zero here, so offset = -direction * angle(steps).
            offset = -direction * (steps - 2048) * 2 * math.pi / 4096
            lines.append(f'  {name}: {{id: {sid}, direction: {direction}, offset: {offset:.5f}}}')
            print(f'{name:16s} id {sid:2d}  steps {steps:4d}  offset {math.degrees(offset):+6.2f} deg')
        with open(a.out, 'w') as f:
            f.write('\n'.join(lines) + '\n')
        print(f'wrote {a.out}; copy it to hexabot_description/config/servo_calibration.yaml')


if __name__ == '__main__':
    main()
