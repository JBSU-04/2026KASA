#!/usr/bin/env python3
"""Pure-Python codecs for MORAI SIM 25.S4 UDP packets."""

import math
import struct
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple


PACKET_TAIL = b'\r\n'


@dataclass(frozen=True)
class MoraiPacket:
    type_name: str
    payload: bytes
    aux: bytes


def pack_packet(type_name: str, payload: bytes, aux_size: int = 12) -> bytes:
    """Create the common MORAI packet: type, length, aux, data, CRLF."""
    if not type_name or '$' in type_name:
        raise ValueError('invalid MORAI packet type')
    if aux_size < 0:
        raise ValueError('aux_size must be non-negative')
    type_header = ('#{}$'.format(type_name)).encode('ascii')
    return type_header + struct.pack('<i', len(payload)) + bytes(aux_size) + payload + PACKET_TAIL


def unpack_packet(data: bytes) -> MoraiPacket:
    """Decode a packet and infer its auxiliary-header length from total size."""
    if len(data) < 9 or data[0:1] != b'#':
        raise ValueError('missing MORAI packet header')
    marker = data.find(b'$', 1, 64)
    if marker < 2:
        raise ValueError('invalid MORAI type header')
    length_offset = marker + 1
    if len(data) < length_offset + 6:
        raise ValueError('truncated MORAI packet')
    payload_size = struct.unpack_from('<i', data, length_offset)[0]
    if payload_size < 0:
        raise ValueError('negative MORAI payload size')
    if not data.endswith(PACKET_TAIL):
        raise ValueError('missing MORAI packet tail')
    tail_size = 2
    aux_size = len(data) - length_offset - 4 - payload_size - tail_size
    if aux_size < 0:
        raise ValueError('payload size exceeds datagram size')
    payload_offset = length_offset + 4 + aux_size
    payload = data[payload_offset:payload_offset + payload_size]
    if len(payload) != payload_size:
        raise ValueError('truncated MORAI payload')
    return MoraiPacket(
        type_name=data[1:marker].decode('ascii'),
        payload=payload,
        aux=data[length_offset + 4:payload_offset],
    )


def _finite(value: float, fallback: float = 0.0) -> float:
    value = float(value)
    return value if math.isfinite(value) else fallback


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def pack_ctrl_payload(
    longl_cmd_type: int,
    accel: float,
    brake: float,
    steering_rad: float,
    velocity_kph: float,
    acceleration_mps2: float,
    max_wheel_angle_rad: float,
    ctrl_mode: int = 2,
    gear: int = 4,
) -> bytes:
    """Encode CtrlCmd. UDP steering is wheel angle normalized by its limit."""
    if max_wheel_angle_rad <= 0.0:
        raise ValueError('max_wheel_angle_rad must be positive')
    steer_normalized = _clamp(
        _finite(steering_rad) / max_wheel_angle_rad,
        -1.0,
        1.0,
    )
    return struct.pack(
        '<BBB5f',
        int(_clamp(ctrl_mode, 0, 255)),
        int(_clamp(gear, 0, 255)),
        int(_clamp(longl_cmd_type, 1, 3)),
        max(0.0, _finite(velocity_kph)),
        _finite(acceleration_mps2),
        _clamp(_finite(accel), 0.0, 1.0),
        _clamp(_finite(brake), 0.0, 1.0),
        steer_normalized,
    )


def pack_ctrl_packet(*args, aux_size: int = 12, **kwargs) -> bytes:
    return pack_packet('MoraiCtrlCmd', pack_ctrl_payload(*args, **kwargs), aux_size)


def parse_competition_payload(payload: bytes) -> Dict[str, object]:
    """Decode the 152-byte Competition Vehicle Status payload used by 25.S4."""
    if len(payload) < 152:
        raise ValueError('MoraiInfo payload must contain at least 152 bytes')
    get_f32 = lambda offset: struct.unpack_from('<f', payload, offset)[0]
    return {
        'secs': struct.unpack_from('<I', payload, 0)[0],
        'nsecs': struct.unpack_from('<I', payload, 4)[0],
        'ctrl_mode': payload[8],
        'gear': payload[9],
        'speed_kph': get_f32(10),
        'map_data_id': struct.unpack_from('<i', payload, 14)[0],
        'accel': get_f32(18),
        'brake': get_f32(22),
        'vehicle_size': (get_f32(26), get_f32(30), get_f32(34)),
        'front_overhang': get_f32(38),
        'wheelbase': get_f32(42),
        'rear_overhang': get_f32(46),
        'orientation_deg': (get_f32(62), get_f32(66), get_f32(70)),
        'velocity_kph': (get_f32(74), get_f32(78), get_f32(82)),
        'angular_velocity_deg_s': (get_f32(86), get_f32(90), get_f32(94)),
        'acceleration_mps2': (get_f32(98), get_f32(102), get_f32(106)),
        'wheel_angle_deg': get_f32(110),
        'link_id': payload[114:152].split(b'\0', 1)[0].decode('ascii', errors='ignore').strip(),
    }


def parse_competition_packet(data: bytes) -> Dict[str, object]:
    packet = unpack_packet(data)
    if packet.type_name != 'MoraiInfo':
        raise ValueError('expected MoraiInfo, got {}'.format(packet.type_name))
    return parse_competition_payload(packet.payload)


def parse_collision_payload(payload: bytes) -> Dict[str, object]:
    """Decode both the legacy 140-byte and timestamped 148-byte layouts."""
    if len(payload) >= 8 and (len(payload) - 8) % 28 == 0:
        secs, nsecs = struct.unpack_from('<II', payload, 0)
        offset = 8
    elif len(payload) % 28 == 0:
        secs, nsecs = 0, 0
        offset = 0
    else:
        raise ValueError('unsupported CollisionData payload size {}'.format(len(payload)))

    objects: List[Dict[str, object]] = []
    while offset + 28 <= len(payload):
        object_type, unique_id, px, py, pz, gx, gy, gz = struct.unpack_from(
            '<hh6f', payload, offset
        )
        offset += 28
        # 비어 있는 고정 슬롯은 모두 0으로 채워진다.
        if object_type == 0 and unique_id == 0 and not any((px, py, pz, gx, gy, gz)):
            continue
        objects.append({
            'type': int(object_type),
            'unique_id': int(unique_id),
            'position': (px, py, pz),
            'global_offset': (gx, gy, gz),
        })
    return {'secs': secs, 'nsecs': nsecs, 'objects': objects}


def parse_collision_packet(data: bytes) -> Dict[str, object]:
    packet = unpack_packet(data)
    if packet.type_name != 'CollisionData':
        raise ValueError('expected CollisionData, got {}'.format(packet.type_name))
    return parse_collision_payload(packet.payload)


def parse_imu_packet_with_stamp(data: bytes) -> Tuple[Tuple[int, int], Tuple[float, ...]]:
    """Decode IMUData. 25.S4 prefixes the 10 doubles with an 8-byte secs/nsecs
    stamp (88-byte payload); older/legacy captures omit it (80-byte payload)."""
    packet = unpack_packet(data)
    if packet.type_name != 'IMUData':
        raise ValueError('expected IMUData, got {}'.format(packet.type_name))
    payload = packet.payload
    if len(payload) == 88:
        stamp = struct.unpack_from('<II', payload, 0)
        values = struct.unpack_from('<10d', payload, 8)
    elif len(payload) == 80:
        stamp = (0, 0)
        values = struct.unpack('<10d', payload)
    else:
        raise ValueError('IMUData payload must be 80 or 88 bytes')
    if not all(math.isfinite(value) for value in values):
        raise ValueError('IMUData contains a non-finite value')
    return stamp, values


def parse_imu_packet(data: bytes) -> Tuple[float, ...]:
    """Backward-compatible IMU decoder returning only sensor values."""
    _, values = parse_imu_packet_with_stamp(data)
    return values


def _checksum_valid(sentence: str) -> bool:
    if not sentence.startswith('$'):
        return False
    if '*' not in sentence:
        return True
    body, checksum_text = sentence[1:].split('*', 1)
    checksum = 0
    for character in body:
        checksum ^= ord(character)
    try:
        return checksum == int(checksum_text[:2], 16)
    except ValueError:
        return False


def _nmea_coordinate(value: str, hemisphere: str) -> float:
    raw = float(value)
    degrees = int(raw // 100.0)
    coordinate = degrees + (raw - degrees * 100.0) / 60.0
    return -coordinate if hemisphere in ('S', 'W') else coordinate


def parse_nmea_datagram(data: bytes) -> Optional[Dict[str, object]]:
    """Return the newest GGA/RMC fix from one MORAI GPS datagram.

    The returned dict carries 'sentence' ('GGA' or 'RMC') so callers can drop
    the duplicate: MORAI 25.S4 sends one fix as two datagrams (RMC then GGA,
    under a millisecond apart, identical lat/lon).
    """
    fixes: List[Dict[str, object]] = []
    text = data.decode('ascii', errors='ignore').replace('\0', '')
    for raw_line in text.replace('\r', '\n').split('\n'):
        line = raw_line.strip()
        if not line or not _checksum_valid(line):
            continue
        fields = line.split('*', 1)[0].split(',')
        sentence_type = fields[0][-3:]
        try:
            if sentence_type == 'GGA' and len(fields) > 9 and fields[2] and fields[4]:
                status = int(fields[6] or 0)
                fixes.append({
                    'sentence': 'GGA',
                    'latitude': _nmea_coordinate(fields[2], fields[3]),
                    'longitude': _nmea_coordinate(fields[4], fields[5]),
                    'altitude': float(fields[9] or 0.0),
                    'status': status,
                })
            elif sentence_type == 'RMC' and len(fields) > 6 and fields[3] and fields[5]:
                fixes.append({
                    'sentence': 'RMC',
                    'latitude': _nmea_coordinate(fields[3], fields[4]),
                    'longitude': _nmea_coordinate(fields[5], fields[6]),
                    'altitude': 0.0,
                    'status': 1 if fields[2] == 'A' else 0,
                })
        except (ValueError, IndexError):
            continue
    if not fixes:
        return None
    for fix in reversed(fixes):
        if fix['status']:
            return fix
    return fixes[-1]


class CameraFrameAssembler:
    """Join MORAI camera UDP fragments into one compressed JPEG frame."""

    def __init__(self, max_frame_bytes: int = 8 * 1024 * 1024):
        self._buffer = bytearray()
        self._max_frame_bytes = max_frame_bytes

    def reset(self) -> None:
        self._buffer.clear()

    def feed(self, datagram: bytes) -> Optional[bytes]:
        if len(datagram) < 21:
            self.reset()
            return None
        chunk = datagram[19:-2]
        tail = datagram[-2:]
        if not self._buffer and not chunk.startswith(b'\xff\xd8'):
            return None
        self._buffer.extend(chunk)
        if len(self._buffer) > self._max_frame_bytes:
            self.reset()
            raise ValueError('camera frame exceeded configured size')
        if tail != b'EI':
            return None
        frame = bytes(self._buffer)
        self.reset()
        if not frame.startswith(b'\xff\xd8'):
            return None
        return frame
