"""Frames from the module manual, section 6.3 (instruction examples)."""
import pytest

from hexabot_lrf import protocol as p


def h(s):
    return bytes.fromhex(s)


@pytest.mark.parametrize('frame, expected', [
    (p.self_check(), 'ee 16 02 03 01 04'),
    (p.single_ranging(), 'ee 16 02 03 02 05'),
    (p.continuous_ranging(), 'ee 16 02 03 04 07'),
    (p.stop_ranging(), 'ee 16 02 03 05 08'),
    (p.set_target(p.Target.FIRST), 'ee 16 03 03 03 01 07'),
    (p.set_target(p.Target.LAST), 'ee 16 03 03 03 02 08'),
    (p.set_target(p.Target.MULTI), 'ee 16 03 03 03 03 09'),
    (p.set_frequency(1), 'ee 16 04 03 a1 01 00 a5'),
    (p.set_frequency(5), 'ee 16 04 03 a1 05 00 a9'),
    (p.query(p.Cmd.GET_MIN_GATE), 'ee 16 02 03 a3 a6'),
    (p.query(p.Cmd.GET_MAX_GATE), 'ee 16 02 03 a5 a8'),
    (p.query(p.Cmd.FPGA_VERSION), 'ee 16 02 03 a6 a9'),
    (p.query(p.Cmd.MCU_VERSION), 'ee 16 02 03 a7 aa'),
    (p.query(p.Cmd.HW_VERSION), 'ee 16 02 03 a8 ab'),
    (p.query(p.Cmd.SERIAL_NUMBER), 'ee 16 02 03 a9 ac'),
    (p.query(p.Cmd.TOTAL_SHOTS), 'ee 16 02 03 90 93'),
    (p.query(p.Cmd.SHOTS_SINCE_POWER_ON), 'ee 16 02 03 91 94'),
])
def test_commands_match_manual(frame, expected):
    assert frame == h(expected)


def test_self_check_response():
    frames = list(p.Parser().feed(h('ee 16 06 03 01 ff 00 f7 ff f9')))
    assert len(frames) == 1 and frames[0].cmd == p.Cmd.SELF_CHECK
    s = p.decode_self_check(frames[0].params)
    assert s['fpga_ok'] and s['laser_output'] and s['main_wave']
    assert not s['echo']            # bit 3 of 0xF7
    assert s['bias_on'] and s['bias_ok'] and s['temperature_ok'] and s['light_off_valid']
    assert s['power_5v6_ok'] and s['echo_intensity'] == 0


def test_out_of_range_response():
    frames = list(p.Parser().feed(h('ee 16 06 03 02 04 00 00 00 09')))
    r = p.decode_range(frames[0].params)
    assert r.out_of_range and p.collect_ranges([r]) is None


def test_continuous_stream_split_across_reads():
    parser = p.Parser()
    data = h('ee 16 06 03 04 04 00 00 00 0b') * 2 + h('ee 16 02 03 05 08')
    frames = []
    for i in range(0, len(data), 3):
        frames += list(parser.feed(data[i:i + 3]))
    assert [f.cmd for f in frames] == [p.Cmd.CONTINUOUS, p.Cmd.CONTINUOUS, p.Cmd.STOP]


def test_range_encoding_round_trip():
    params = p.encode_range(1234.5)
    assert params == bytes([0x00, 0x04, 0xD2, 0x05])
    frame = p.build(p.Cmd.SINGLE, params)
    r = p.decode_range(next(p.Parser().feed(frame)).params)
    assert r.range_m == pytest.approx(1234.5)
    assert not r.out_of_range and r.index == 0


def test_multi_target_status():
    rs = [p.decode_range(p.encode_range(812.3, 0x03)), p.decode_range(p.encode_range(140.0, 0x13))]
    assert rs[0].front_target and rs[0].rear_target and rs[1].index == 1
    assert p.collect_ranges(rs) == [140.0, 812.3]


def test_parser_rejects_noise_and_bad_checksum():
    parser = p.Parser()
    junk = h('00 ee 16 06 03 02 00 00 10 00 ff') + h('ee ee 16 06 03 02 00 00 10 00 15')
    frames = list(parser.feed(junk))
    assert len(frames) == 1 and p.decode_range(frames[0].params).range_m == 16.0
    assert parser.checksum_errors == 1


def test_gate_baud_and_versions():
    assert p.set_min_gate(15) == h('ee 16 04 03 a2 00 0f b4')
    assert p.decode_gate(bytes([0x4E, 0x20])) == 20000
    assert p.set_baud(115200) == p.build(p.Cmd.SET_BAUD, bytes([0x00, 0x01, 0xC2, 0x00]))
    assert p.decode_version(bytes([0x10, 21, 0x73, 0x6C])) == 'V1.0 2023-07-21'
    assert p.decode_counter(bytes([0x01, 0x02, 0x03])) == 0x010203
    with pytest.raises(ValueError):
        p.set_frequency(11)
    with pytest.raises(ValueError):
        p.set_min_gate(5)
