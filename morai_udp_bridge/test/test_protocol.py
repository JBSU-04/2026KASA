#!/usr/bin/env python3
import math
import struct
import unittest

from morai_udp_bridge.protocol import (
    CameraFrameAssembler,
    pack_ctrl_packet,
    pack_packet,
    parse_collision_packet,
    parse_competition_packet,
    parse_imu_packet,
    parse_imu_packet_with_stamp,
    parse_nmea_datagram,
    unpack_packet,
)


class ProtocolTest(unittest.TestCase):
    def test_common_packet_round_trip(self):
        encoded = pack_packet('Example', b'payload', aux_size=12)
        decoded = unpack_packet(encoded)
        self.assertEqual(decoded.type_name, 'Example')
        self.assertEqual(decoded.payload, b'payload')
        self.assertEqual(len(decoded.aux), 12)

    def test_common_packet_rejects_missing_tail(self):
        with self.assertRaises(ValueError):
            unpack_packet(pack_packet('Example', b'payload')[:-2])

    def test_ctrl_packet_matches_25s4_layout(self):
        encoded = pack_ctrl_packet(
            longl_cmd_type=1,
            accel=0.4,
            brake=0.0,
            steering_rad=math.radians(20.0),
            velocity_kph=30.0,
            acceleration_mps2=1.2,
            max_wheel_angle_rad=math.radians(40.0),
            ctrl_mode=2,
            gear=4,
        )
        self.assertEqual(len(encoded), 55)
        packet = unpack_packet(encoded)
        self.assertEqual(packet.type_name, 'MoraiCtrlCmd')
        values = struct.unpack('<BBB5f', packet.payload)
        self.assertEqual(values[:3], (2, 4, 1))
        self.assertAlmostEqual(values[3], 30.0)
        self.assertAlmostEqual(values[7], 0.5)

    def test_competition_status(self):
        payload = bytearray(152)
        struct.pack_into('<II', payload, 0, 10, 20)
        payload[8:10] = bytes((2, 4))
        struct.pack_into('<f', payload, 10, 36.0)
        struct.pack_into('<i', payload, 14, 7)
        struct.pack_into('<ff', payload, 18, 0.5, 0.1)
        struct.pack_into('<fff', payload, 62, 1.0, 2.0, 123.0)
        struct.pack_into('<fff', payload, 74, 36.0, 0.0, 0.0)
        struct.pack_into('<f', payload, 110, -12.5)
        payload[114:120] = b'A21987'
        state = parse_competition_packet(pack_packet('MoraiInfo', payload))
        self.assertEqual(state['secs'], 10)
        self.assertEqual(state['gear'], 4)
        self.assertAlmostEqual(state['velocity_kph'][0], 36.0)
        self.assertAlmostEqual(state['wheel_angle_deg'], -12.5)
        self.assertEqual(state['link_id'], 'A21987')

    def test_collision_layouts(self):
        record = struct.pack('<hh6f', 1, 42, 1.0, 2.0, 3.0, 300000.0, 4100000.0, 0.0)
        old_payload = record + bytes(28 * 4)
        old_state = parse_collision_packet(pack_packet('CollisionData', old_payload))
        self.assertEqual(len(old_state['objects']), 1)
        self.assertEqual(old_state['objects'][0]['unique_id'], 42)

        new_payload = struct.pack('<II', 3, 4) + old_payload
        new_state = parse_collision_packet(pack_packet('CollisionData', new_payload))
        self.assertEqual((new_state['secs'], new_state['nsecs']), (3, 4))

    def test_imu_aux_header_is_inferred(self):
        values = (1.0, 0.0, 0.0, 0.0, 0.1, 0.2, 0.3, 1.0, 2.0, 3.0)
        encoded = pack_packet('IMUData', struct.pack('<10d', *values), aux_size=20)
        self.assertEqual(parse_imu_packet(encoded), values)

    def test_imu_25s4_payload_has_secs_nsecs_prefix(self):
        # 25.S4 실측 페이로드: 88 bytes = <II secs,nsecs> + <10d> 값들.
        values = (
            -0.879981279373169, -0.009661000221967697, -0.010871159844100475,
            -0.47478577494621277, -0.0008053713971186034, 0.001223101133935847,
            -0.002208072815565469, -0.10390184372480048, 0.2723256080285765,
            9.818628304280901,
        )
        payload = struct.pack('<II', 1786448555, 617000000) + struct.pack('<10d', *values)
        encoded = pack_packet('IMUData', payload, aux_size=12)
        self.assertEqual(len(encoded), 115)
        decoded = parse_imu_packet(encoded)
        stamp, decoded_with_stamp = parse_imu_packet_with_stamp(encoded)
        self.assertEqual(stamp, (1786448555, 617000000))
        for actual, expected in zip(decoded, values):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(decoded_with_stamp, values):
            self.assertAlmostEqual(actual, expected)

    def test_nmea_gga(self):
        body = 'GPGGA,123519,3723.2475,N,12701.1234,E,1,08,0.9,12.3,M,0.0,M,,'
        checksum = 0
        for character in body:
            checksum ^= ord(character)
        fix = parse_nmea_datagram('${}*{:02X}\r\n'.format(body, checksum).encode('ascii'))
        self.assertIsNotNone(fix)
        self.assertAlmostEqual(fix['latitude'], 37.3874583333, places=7)
        self.assertAlmostEqual(fix['longitude'], 127.0187233333, places=7)
        self.assertAlmostEqual(fix['altitude'], 12.3)
        self.assertEqual(fix['sentence'], 'GGA')

    def test_nmea_sentence_type_distinguishes_duplicate_rmc(self):
        # MORAI 25.S4는 같은 fix를 RMC와 GGA 두 datagram으로 따로 보낸다.
        # 브리지가 중복을 버리려면 문장 종류를 구분할 수 있어야 한다.
        def sentence(body):
            checksum = 0
            for character in body:
                checksum ^= ord(character)
            return '${}*{:02X}\r\n'.format(body, checksum).encode('ascii')

        rmc = parse_nmea_datagram(sentence(
            'GPRMC,123519,A,3723.2475,N,12701.1234,E,0.0,0.0,110826,,,A'))
        gga = parse_nmea_datagram(sentence(
            'GPGGA,123519,3723.2475,N,12701.1234,E,1,08,0.9,12.3,M,0.0,M,,'))
        self.assertEqual(rmc['sentence'], 'RMC')
        self.assertEqual(gga['sentence'], 'GGA')
        # 같은 fix이므로 좌표는 동일하고 고도만 GGA에 있다.
        self.assertAlmostEqual(rmc['latitude'], gga['latitude'], places=9)
        self.assertAlmostEqual(rmc['longitude'], gga['longitude'], places=9)
        self.assertEqual(rmc['altitude'], 0.0)

    def test_camera_assembler_publishes_once(self):
        assembler = CameraFrameAssembler()
        jpeg = b'\xff\xd8abc\xff\xd9'
        self.assertIsNone(assembler.feed(bytes(19) + jpeg[:4] + b'BI'))
        self.assertEqual(assembler.feed(bytes(19) + jpeg[4:] + b'EI'), jpeg)
        self.assertIsNone(assembler.feed(bytes(19) + b'bad' + b'EI'))


if __name__ == '__main__':
    unittest.main()
