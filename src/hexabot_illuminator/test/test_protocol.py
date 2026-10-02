"""Packets must match what the former Laser_GUI sent to the illuminator."""
import pytest

from hexabot_illuminator import protocol as p


def test_power_packets():
    assert p.power(True) == bytes([0xFF, 0x01, 0x01, 0x01, 0x01, 0x00, 0x04])
    assert p.power(False) == bytes([0xFF, 0x01, 0x01, 0x01, 0x00, 0x00, 0x03])


def test_brightness_and_queries():
    assert p.brightness(200) == bytes([0xFF, 0x01, 0x01, 0x03, 200, 0x00, (1 + 1 + 3 + 200) & 0xFF])
    assert p.packet(p.QUERY_POWER) == bytes([0xFF, 0x01, 0x02, 0x01, 0x00, 0x00, 0x04])
    assert p.packet(p.QUERY_VERSION, 0x01, 0x01) == bytes([0xFF, 0x01, 0x05, 0x10, 0x01, 0x01, 0x18])


@pytest.mark.parametrize('deg, motor', [(71.0, 0), (1.8, 0x4000), (36.4, 0x2000), (100, 0), (0, 0x4000)])
def test_spot_angle_mapping(deg, motor):
    assert p.spot_to_motor(deg) == motor


def test_spot_angle_packet():
    v = p.spot_to_motor(12.0)
    assert p.spot_angle(12.0)[2:6] == bytes([0x08, 0x01, v >> 8, v & 0xFF])


def test_reply_parsing_and_state():
    st = p.State()
    parser = p.Parser()
    stream = b'\x00\x13' + p.packet(p.QUERY_POWER, 1) + p.packet(p.QUERY_SPOT, 0x04, 0xB0) \
        + bytes([0xFF, 1, 2, 3, 0, 0, 0x99]) + p.packet(p.QUERY_FAN, 1)
    replies = list(parser.feed(stream))
    for r in replies:
        p.apply(r, st)
    assert st.power is True and st.fan is True
    assert st.spot_deg == pytest.approx(12.0)
    assert parser.checksum_errors == 1


def test_baud_and_address():
    assert p.set_baud(115200)[5] == 0x07
    with pytest.raises(ValueError):
        p.set_address(0)
