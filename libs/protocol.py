"""RPLidar A2M8 wire-format helpers (sample encoding + packet framing)."""

CMD_START = 0x10
CMD_STOP = 0x20


def encode_sample(quality, angle_deg, distance_mm, is_new_scan):
    angle_q6 = int(angle_deg * 64) & 0x7FFF
    distance_q2 = int(distance_mm * 4) & 0xFFFF
    s = 1 if is_new_scan else 0
    not_s = 0 if is_new_scan else 1
    byte0 = ((quality & 0x3F) << 2) | (not_s << 1) | s
    byte1 = ((angle_q6 & 0x7F) << 1) | 1
    byte2 = (angle_q6 >> 7) & 0xFF
    byte3 = distance_q2 & 0xFF
    byte4 = (distance_q2 >> 8) & 0xFF
    return bytes([byte0, byte1, byte2, byte3, byte4])


def frame_packet(payload):
    size = len(payload)
    header = bytes([0xA5, 0x10, (size >> 16) & 0xFF, (size >> 8) & 0xFF, size & 0xFF])
    return header + payload
