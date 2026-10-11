#!/usr/bin/env python3
"""
MORAI UDP Ego Vehicle Status(MoraiInfo) 패킷을 받아서 기존 코드가 그대로 쓰는
/Competition_topic(morai_msgs/EgoVehicleStatus)으로 재발행하는 브리지.

패킷 파싱은 MORAI 공식 오픈소스 예제(MORAI-Autonomous/MORAI-DriveExample_UDP,
network/receiver/ego_info_receiver.py)의 오프셋을 그대로 사용한다.

주의: MORAI 버전에 따라 패킷 앞에 8바이트 Timestamp가 추가되는 등 오프셋이
달라질 수 있다고 알려져 있어서, 헤더 문자열/길이가 안 맞으면 아래에서
경고를 찍도록 해뒀다 - 그 경우 offset을 다시 맞춰야 한다.
"""
import struct

import rospy
import socket
import threading

from geometry_msgs.msg import Vector3
from morai_msgs.msg import EgoVehicleStatus


HEADER = '#MoraiInfo$'
DATA_LENGTH = 216


def parse_ego_info(raw_data):
    if len(raw_data) < 15:
        return None
    if raw_data[0:11].decode(errors='ignore') != HEADER:
        return None
    data_length = struct.unpack('i', raw_data[11:15])[0]
    if data_length != DATA_LENGTH:
        rospy.logwarn_throttle(
            5.0,
            '[udp_ego_info_bridge] unexpected data_length=%d (expected %d) - '
            'MORAI 버전이 달라서 패킷 오프셋이 안 맞을 수 있음',
            data_length, DATA_LENGTH)
        return None

    ctrl_mode = struct.unpack('b', raw_data[35:36])[0]
    gear = struct.unpack('b', raw_data[36:37])[0]
    signed_vel_kmh = struct.unpack('f', raw_data[37:41])[0]
    accel = struct.unpack('f', raw_data[45:49])[0]
    brake = struct.unpack('f', raw_data[49:53])[0]
    pos_x, pos_y, pos_z = struct.unpack('fff', raw_data[77:89])
    roll, pitch, yaw = struct.unpack('fff', raw_data[89:101])
    vel_x, vel_y, vel_z = struct.unpack('fff', raw_data[101:113])
    acc_x, acc_y, acc_z = struct.unpack('fff', raw_data[125:137])
    front_steer, rear_steer = struct.unpack('ff', raw_data[137:145])

    return {
        'ctrl_mode': ctrl_mode, 'gear': gear, 'signed_vel_kmh': signed_vel_kmh,
        'accel': accel, 'brake': brake,
        'pos': (pos_x, pos_y, pos_z), 'rpy': (roll, pitch, yaw),
        'vel': (vel_x, vel_y, vel_z), 'acc': (acc_x, acc_y, acc_z),
        'front_steer': front_steer, 'rear_steer': rear_steer,
    }


def main():
    rospy.init_node('udp_ego_info_bridge')

    bind_host = rospy.get_param('~bind_host', '0.0.0.0')
    ego_info_port = rospy.get_param('~ego_info_port', 9110)  # Ego Vehicle Status Destination PORT (>1024, root 없이 바인딩 가능)

    pub = rospy.Publisher('/Competition_topic', EgoVehicleStatus, queue_size=10)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((bind_host, ego_info_port))
    rospy.loginfo('[udp_ego_info_bridge] listening UDP %s:%d -> /Competition_topic',
                  bind_host, ego_info_port)

    def loop():
        while not rospy.is_shutdown():
            try:
                raw_data, _ = sock.recvfrom(65535)
            except OSError:
                return

            parsed = parse_ego_info(raw_data)
            if parsed is None:
                continue

            msg = EgoVehicleStatus()
            msg.header.stamp = rospy.Time.now()
            msg.position = Vector3(*parsed['pos'])
            msg.velocity = Vector3(*parsed['vel'])
            msg.acceleration = Vector3(*parsed['acc'])
            msg.heading = parsed['rpy'][2]  # yaw, deg (MORAI 원본 단위 그대로)
            msg.accel = parsed['accel']
            msg.brake = parsed['brake']
            msg.front_steer_angle = parsed['front_steer']
            msg.rear_steer_angle = parsed['rear_steer']
            # wheel_angle은 UDP 패킷 압축 필드에 별도로 없어 front_steer로 대체
            # (진단/로그용으로만 쓰이는 필드라 제어 루프에는 영향 없음)
            msg.wheel_angle = parsed['front_steer']
            pub.publish(msg)

    threading.Thread(target=loop, daemon=True).start()
    rospy.spin()
    sock.close()


if __name__ == '__main__':
    main()
