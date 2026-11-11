"""Minimal VESC CAN RPM packet encoding. Command frame type 3."""
import struct

RPM_PACKET_ID = 3
WHEEL_IDS = {'FL': 1, 'FR': 2, 'RL': 3, 'RR': 4}


def encode_rpm(vesc_id: int, erpm: int):
    if not 0 <= vesc_id <= 255:
        raise ValueError('CAN VESC ID outside 0..255')
    return (RPM_PACKET_ID << 8) | vesc_id, struct.pack('>i', erpm)


def decode_rpm(arbitration_id: int, data: bytes):
    if (arbitration_id >> 8) != RPM_PACKET_ID or len(data) != 4:
        return None
    return arbitration_id & 0xFF, struct.unpack('>i', data)[0]


def wheel_mps_to_erpm(speed, radius, ratio, pole_pairs):
    return round(speed * 60.0 * ratio * pole_pairs / (2.0 * 3.141592653589793 * radius))


def erpm_to_wheel_mps(erpm, radius, ratio, pole_pairs):
    return (erpm / (ratio * pole_pairs)) * (2.0 * 3.141592653589793 * radius / 60.0)
