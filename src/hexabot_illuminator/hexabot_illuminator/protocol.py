"""Protocol of the IR laser illuminator (ported from the former Laser_GUI).

Packet (both directions, 7 bytes):  FF | ADDR | CMD1 | CMD2 | D1 | D2 | CHK
CHK = (ADDR + CMD1 + CMD2 + D1 + D2) & 0xFF.  Transport: TCP (default
192.168.0.7:20108) or a serial port (default 9600 8N1).
"""
from dataclasses import dataclass
from typing import Iterator, Optional

HEADER = 0xFF
SPOT_MIN_DEG = 1.8
SPOT_MAX_DEG = 71.0
MOTOR_MAX = 0x4000

# (cmd1, cmd2) pairs
POWER = (0x01, 0x01)            # d1: 1 on / 0 off
BRIGHTNESS = (0x01, 0x03)       # d1: 0..255
ZOOM_STEP = (0x01, 0x04)        # d1: 1 wider / 0 narrower
MOTOR_POSITION = (0x01, 0x05)   # d1 d2: 0x0000..0x4000
MOTOR_RESET = (0x01, 0x06)
SPOT_ANGLE = (0x08, 0x01)       # d1 d2: internal motor value for an angle
SET_ADDRESS = (0x03, 0x11)      # d2: new address
SET_BAUD = (0x03, 0x13)         # d2: baud code
QUERY_POWER = (0x02, 0x01)
QUERY_BRIGHTNESS = (0x02, 0x03)
QUERY_MOTOR = (0x02, 0x05)
QUERY_FAN = (0x02, 0x0F)
QUERY_SPOT = (0x09, 0x01)       # reply: angle in 0.01 deg
QUERY_VERSION = (0x05, 0x10)    # replies 20 00 (version), 21 00 (date), 22 00 (module type)

#: Commands that are answered with a 7-byte packet.
REQUESTS = {QUERY_POWER, QUERY_BRIGHTNESS, QUERY_MOTOR, QUERY_FAN, QUERY_SPOT, QUERY_VERSION}

BAUD_CODES = {1200: 0x00, 2400: 0x01, 4800: 0x02, 9600: 0x03,
              19200: 0x04, 38400: 0x05, 57600: 0x06, 115200: 0x07}


def checksum(body) -> int:
    return sum(body) & 0xFF


def packet(cmd, d1: int = 0, d2: int = 0, address: int = 0x01) -> bytes:
    body = [address & 0xFF, cmd[0], cmd[1], d1 & 0xFF, d2 & 0xFF]
    return bytes([HEADER, *body, checksum(body)])


def spot_to_motor(deg: float) -> int:
    """Spot angle -> internal value of command 08 01 (as in Laser_GUI)."""
    deg = min(SPOT_MAX_DEG, max(SPOT_MIN_DEG, deg))
    v = round((SPOT_MAX_DEG - deg) / (SPOT_MAX_DEG - SPOT_MIN_DEG) * MOTOR_MAX)
    return max(0, min(MOTOR_MAX, v))


def power(on: bool, address: int = 1) -> bytes:
    return packet(POWER, 1 if on else 0, 0, address)


def brightness(value: int, address: int = 1) -> bytes:
    return packet(BRIGHTNESS, max(0, min(255, int(value))), 0, address)


def spot_angle(deg: float, address: int = 1) -> bytes:
    v = spot_to_motor(deg)
    return packet(SPOT_ANGLE, v >> 8, v & 0xFF, address)


def motor_position(value: int, address: int = 1) -> bytes:
    v = max(0, min(MOTOR_MAX, int(value)))
    return packet(MOTOR_POSITION, v >> 8, v & 0xFF, address)


def set_baud(baud: int, address: int = 1) -> bytes:
    return packet(SET_BAUD, 0, BAUD_CODES[baud], address)


def set_address(new: int, address: int = 1) -> bytes:
    if not 1 <= new <= 254:
        raise ValueError('address must be 1..254')
    return packet(SET_ADDRESS, 0, new, address)


@dataclass
class Reply:
    address: int
    cmd: tuple
    d1: int
    d2: int

    @property
    def value16(self) -> int:
        return self.d1 << 8 | self.d2


class Parser:
    def __init__(self) -> None:
        self._buf = bytearray()
        self.checksum_errors = 0

    def feed(self, data: bytes) -> Iterator[Reply]:
        self._buf.extend(data)
        while len(self._buf) >= 7:
            if self._buf[0] != HEADER:
                del self._buf[0]
                continue
            pkt = self._buf[:7]
            if checksum(pkt[1:6]) != pkt[6]:
                self.checksum_errors += 1
                del self._buf[0]
                continue
            del self._buf[:7]
            yield Reply(pkt[1], (pkt[2], pkt[3]), pkt[4], pkt[5])


@dataclass
class State:
    power: Optional[bool] = None
    brightness: Optional[int] = None
    motor: Optional[int] = None
    spot_deg: Optional[float] = None
    fan: Optional[bool] = None
    firmware: str = ''


def apply(reply: Reply, state: State) -> None:
    """Update the device state from a reply (decoding as in Laser_GUI)."""
    c = reply.cmd
    if c == QUERY_POWER:
        state.power = reply.d1 == 0x01
    elif c == QUERY_BRIGHTNESS:
        state.brightness = reply.d1
    elif c == QUERY_MOTOR:
        state.motor = reply.value16
    elif c == QUERY_FAN:
        state.fan = reply.d1 == 0x01
    elif c == QUERY_SPOT:
        state.spot_deg = min(SPOT_MAX_DEG, max(SPOT_MIN_DEG, reply.value16 * 0.01))
    elif c == (0x20, 0x00):
        state.firmware = f'{reply.d1}.{reply.d2}'
