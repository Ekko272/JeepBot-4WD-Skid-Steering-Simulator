import math
from jeepbot_sim.protocol import encode_rpm, decode_rpm, wheel_mps_to_erpm, erpm_to_wheel_mps


def test_round_trip():
    for erpm in (-1200, -1, 0, 1, 1200):
        identifier, payload = encode_rpm(3, erpm)
        assert identifier == 0x303
        assert decode_rpm(identifier, payload) == (3, erpm)


def test_other_packet_ignored():
    assert decode_rpm(0x901, b'\0\0\0\0') is None


def test_velocity_round_trip():
    speed = 0.32
    rpm = wheel_mps_to_erpm(speed, 0.16, 1, 7)
    assert math.isclose(erpm_to_wheel_mps(rpm, 0.16, 1, 7), speed, abs_tol=0.003)
