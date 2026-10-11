#!/usr/bin/env python3
"""
/ctrl_cmd(morai_msgs/CtrlCmd)를 MORAI 공식 UDP 프로토콜(MoraiCmdController)로 변환해서
시뮬레이터로 송신하는 브리지.

패킷 포맷은 MORAI 공식 문서(help-morai-sim.scrollhelp.site, 23.R1.0 "통신 메시지
프로토콜" - UDP 통신 메시지 - Ego Network - Ego Ctrl Cmd) 원문 그대로:

전체 패킷 55바이트, 데이터부 3+20=23바이트.
헤더: b'#MoraiCtrlCmd$' + int32(23, 데이터부 길이) + int32(0)*3(예약)
데이터부(23바이트):
  byte(CtrlMode: 1=KeyBoard, 2=AutoMode)
  byte(Gear: 0=M 1=P 2=R 3=N 4=D 5=L)
  byte(longCmdType: 1=Throttle 2=Velocity 3=Acceleration)
  float32(velocity, km/h, longCmdType==2일 때 사용)
  float32(acceleration, m/s^2, longCmdType==3일 때 사용)   <- velocity와 별도 필드, 공용 아님
  float32(accel, 0~1)
  float32(brake, 0~1)
  float32(steering, -1~1)  = 원하는 스티어링 각도(deg) / 최대 조향각(deg)
                              (ioniq 기준 문서 예시값 = 36.25deg)
꼬리: b'\r\n'
"""
import socket
import struct

import rospy
from morai_msgs.msg import CtrlCmd


class CtrlCmdUdpSender:
    def __init__(self, ip, port, gear, max_steering_rad):
        self.address = (ip, port)
        self.gear = gear
        self.max_steering_rad = max_steering_rad
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.message_name = '#MoraiCtrlCmd$'.encode()
        self.aux_data = struct.pack('iii', 0, 0, 0)
        self.tail = '\r\n'.encode()

    def send(self, msg):
        # 매 전송마다 다시 읽어서, 노드 재시작 없이
        # `rosparam set /udp_ctrl_bridge/mode 2` 로 바로 Auto 전환 가능하게 함.
        # (Q키는 이 브릿지가 20Hz로 mode=1을 계속 덮어써서 안 먹힘)
        current_mode = rospy.get_param('~mode', 1)
        mode = struct.pack('b', current_mode)
        gear = struct.pack('b', self.gear)
        cmd_type = int(msg.longlCmdType)
        cmd_type_b = struct.pack('b', cmd_type)

        velocity = struct.pack('f', msg.velocity)          # km/h
        acceleration = struct.pack('f', msg.acceleration)   # m/s^2
        accel = struct.pack('f', msg.accel)                 # 0~1
        brake = struct.pack('f', msg.brake)                 # 0~1

        normalized_steer = msg.steering / self.max_steering_rad
        normalized_steer = max(-1.0, min(1.0, normalized_steer))
        steering = struct.pack('f', normalized_steer)

        body = mode + gear + cmd_type_b + velocity + acceleration + accel + brake + steering
        header = self.message_name + struct.pack('i', len(body)) + self.aux_data
        packet = header + body + self.tail
        self.sock.sendto(packet, self.address)

        rospy.loginfo_throttle(
            1.0,
            '[udp_ctrl_bridge] sent mode=%d gear=%d cmd_type=%d velocity_kmh=%.2f accel=%.2f brake=%.2f '
            'steering_rad=%.3f steering_norm=%.3f -> %s:%d',
            current_mode, self.gear, cmd_type, msg.velocity, msg.accel, msg.brake,
            msg.steering, normalized_steer, self.address[0], self.address[1])


def main():
    rospy.init_node('udp_ctrl_bridge')

    sim_ip = rospy.get_param('~sim_ip', '127.0.0.1')
    ctrl_cmd_port = rospy.get_param('~ctrl_cmd_port', 9093)  # MORAI Network Settings의 Ego Ctrl Cmd Host PORT
    gear = rospy.get_param('~gear', 4)  # 4: Drive
    max_steering_deg = rospy.get_param('~max_steering_deg', 36.25)  # MORAI 문서의 ioniq 기준값
    max_steering_rad = max_steering_deg / 180.0 * 3.14159265358979

    sender = CtrlCmdUdpSender(sim_ip, ctrl_cmd_port, gear, max_steering_rad)

    rospy.Subscriber('/ctrl_cmd', CtrlCmd, sender.send, queue_size=1)
    # 기본은 1(Keyboard) - 자동으로 Auto 전환 안 됨.
    # Auto로 바꾸려면: rosparam set /udp_ctrl_bridge/mode 2
    rospy.set_param('~mode', rospy.get_param('~mode', 1))
    rospy.loginfo('[udp_ctrl_bridge] /ctrl_cmd -> UDP %s:%d (gear=%d). '
                  'Auto 전환: rosparam set /udp_ctrl_bridge/mode 2',
                  sim_ip, ctrl_cmd_port, gear)
    rospy.spin()


if __name__ == '__main__':
    main()
