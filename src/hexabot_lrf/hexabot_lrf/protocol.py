"""Protocol of the 3 km eye-safe laser ranging module (user manual, section 6).

Frame:  EE 16 | LEN | 03 | CMD | PARAMS (0..4) | CHK
LEN is the number of bytes in DEVICE + CMD + PARAMS, CHK the low 8 bits of
their sum.  UART TTL 3.3 V, 115200 8N1 by default (57600 and 9600 possible).
"""
from dataclasses import dataclass
from enum import IntEnum
from typing import Iterator, List, Optional

HEADER = bytes([0xEE, 0x16])
DEVICE = 0x03

MIN_RANGE_M = 15.0      # the module cannot measure closer targets
MAX_RANGE_M = 4200.0    # building targets; people ~2000 m, small UAV ~1000 m
BEAM_DIVERGENCE_RAD = 0.0007
RECEIVE_FOV_RAD = 0.0049


class Cmd(IntEnum):
    SELF_CHECK = 0x01
    SINGLE = 0x02
    SET_TARGET = 0x03
    CONTINUOUS = 0x04
    STOP = 0x05
    ANOMALY = 0x06
    WAKE_UP = 0x07
    SET_BAUD = 0xA0
    SET_FREQUENCY = 0xA1
    SET_MIN_GATE = 0xA2
    GET_MIN_GATE = 0xA3
    SET_MAX_GATE = 0xA4
    GET_MAX_GATE = 0xA5
    FPGA_VERSION = 0xA6
    MCU_VERSION = 0xA7
    HW_VERSION = 0xA8
    SERIAL_NUMBER = 0xA9
    TOTAL_SHOTS = 0x90
    SHOTS_SINCE_POWER_ON = 0x91


class Target(IntEnum):
    FIRST = 0x01
    LAST = 0x02
    MULTI = 0x03


def checksum(body: bytes) -> int:
    return sum(body) & 0xFF


def build(cmd: int, params: bytes = b'') -> bytes:
    body = bytes([DEVICE, int(cmd)]) + bytes(params)
    return HEADER + bytes([len(body)]) + body + bytes([checksum(body)])


# Command builders ----------------------------------------------------------

def self_check() -> bytes:
    return build(Cmd.SELF_CHECK)


def single_ranging() -> bytes:
    return build(Cmd.SINGLE)


def continuous_ranging() -> bytes:
    return build(Cmd.CONTINUOUS)


def stop_ranging() -> bytes:
    return build(Cmd.STOP)


def set_target(mode: int) -> bytes:
    if int(mode) not in (1, 2, 3):
        raise ValueError('target mode must be 1 (first), 2 (last) or 3 (multi)')
    return build(Cmd.SET_TARGET, bytes([int(mode)]))


def set_frequency(hz: int) -> bytes:
    if not 1 <= int(hz) <= 10:
        raise ValueError('ranging frequency must be 1..10 Hz')
    return build(Cmd.SET_FREQUENCY, bytes([int(hz), 0x00]))


def _gate(cmd: int, metres: int) -> bytes:
    if not 10 <= int(metres) <= 20000:
        raise ValueError('gating distance must be 10..20000 m')
    m = int(metres)
    return build(cmd, bytes([(m >> 8) & 0xFF, m & 0xFF]))


def set_min_gate(metres: int) -> bytes:
    return _gate(Cmd.SET_MIN_GATE, metres)


def set_max_gate(metres: int) -> bytes:
    return _gate(Cmd.SET_MAX_GATE, metres)


def set_baud(baud: int) -> bytes:
    if baud not in (9600, 57600, 115200):
        raise ValueError('baud rate must be 9600, 57600 or 115200')
    return build(Cmd.SET_BAUD, baud.to_bytes(4, 'big'))


def query(cmd: int) -> bytes:
    return build(cmd)


# Parsing --------------------------------------------------------------------

@dataclass
class Frame:
    cmd: int
    params: bytes


class Parser:
    """Stream parser: feed raw bytes, iterate complete frames (bad ones dropped)."""

    def __init__(self) -> None:
        self._buf = bytearray()
        self.checksum_errors = 0

    def feed(self, data: bytes) -> Iterator[Frame]:
        self._buf.extend(data)
        while True:
            start = self._buf.find(HEADER)
            if start < 0:
                # Keep a trailing 0xEE that may start the next header.
                del self._buf[:-1 if self._buf.endswith(b'\xee') else len(self._buf)]
                return
            del self._buf[:start]
            if len(self._buf) < 3:
                return
            length = self._buf[2]
            if not 2 <= length <= 18:
                del self._buf[:1]
                continue
            total = 3 + length + 1
            if len(self._buf) < total:
                return
            body = bytes(self._buf[3:3 + length])
            chk = self._buf[3 + length]
            if body[0] != DEVICE or chk != checksum(body):
                self.checksum_errors += 1
                del self._buf[:1]
                continue
            del self._buf[:total]
            yield Frame(body[1], body[2:])


@dataclass
class RangeResult:
    range_m: float
    status: int
    index: int            # result number in multi-target mode
    out_of_range: bool
    front_target: bool
    rear_target: bool


def decode_range(params: bytes) -> RangeResult:
    """Single / continuous ranging response (6.2.2, 6.2.4)."""
    status, hi, lo, dec = params[0], params[1], params[2], params[3]
    code = status & 0x0F
    return RangeResult(
        range_m=hi * 256 + lo + dec * 0.1,
        status=status,
        index=(status >> 4) & 0x0F,
        out_of_range=code == 0x04,
        front_target=code in (0x01, 0x03),
        rear_target=code in (0x02, 0x03),
    )


def encode_range(range_m: float, status: int = 0) -> bytes:
    """Response params for a range (used by the simulator and tests)."""
    whole = int(range_m)
    dec = int(round((range_m - whole) * 10)) % 10
    return bytes([status, (whole >> 8) & 0xFF, whole & 0xFF, dec])


STATUS1_BITS = ('fpga_ok', 'laser_output', 'main_wave', 'echo', 'bias_on', 'bias_ok',
                'temperature_ok', 'light_off_valid')


def decode_self_check(params: bytes) -> dict:
    """Self-check response (6.2.1): Status3 reserved, Status2 echo intensity,
    Status1 bit field, Status0 bit0 = 5V6 power."""
    status2, status1, status0 = params[1], params[2], params[3]
    out = {name: bool(status1 & (1 << bit)) for bit, name in enumerate(STATUS1_BITS)}
    out['echo_intensity'] = status2
    out['power_5v6_ok'] = bool(status0 & 0x01)
    return out


def decode_anomaly(params: bytes) -> dict:
    """Ranging anomaly (6.2.6): only Status1 is meaningful."""
    status1 = params[3]
    return {name: bool(status1 & (1 << bit)) for bit, name in enumerate(STATUS1_BITS)}


def decode_gate(params: bytes) -> int:
    return params[0] << 8 | params[1]


def decode_version(params: bytes) -> str:
    """FPGA / MCU version (6.2.13-14): 'V1.0 2023-07-21'."""
    ver, day, mon_year = params[0], params[1], params[2]
    return f'V{ver >> 4}.{ver & 0x0F} {2020 + (mon_year & 0x0F)}-{mon_year >> 4:02d}-{day:02d}'


def decode_counter(params: bytes) -> int:
    """24-bit light output counters (6.2.17-18)."""
    return params[0] << 16 | params[1] << 8 | params[2]


def collect_ranges(results: List[RangeResult]) -> Optional[List[float]]:
    """Valid ranges of one measurement, closest first; None when out of range."""
    valid = sorted(r.range_m for r in results if not r.out_of_range and r.range_m > 0)
    return valid or None
