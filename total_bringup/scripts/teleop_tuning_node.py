#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""인지 튜닝용 키보드 수동 주행 (MPC 게이트).

라이다 지면제거/장애물검출을 눈으로 보며 맞추려면 원하는 지점에 차를 세우고
각도를 바꿔가며 봐야 한다. 자율주행 상태로는 그게 안 된다.

동작 방식 - 게이트(중계) 구조
    MPC ──> /control/mpc_cmd ──> [이 노드] ──> /ctrl_cmd ──> MORAI
                                     ↑
                                 키보드 입력

  자율 모드: MPC 명령을 그대로 통과시킨다.
  수동 모드: MPC 명령을 버리고 키보드 명령을 낸다.

  ※ 처음엔 이 노드가 /ctrl_cmd 를 MPC 와 '동시에' 발행하고 나중 것이 이기길
    기대했는데, ROS 토픽은 그렇게 동작하지 않는다. 두 발행자가 번갈아 나가서
    MORAI 가 둘을 교대로 받아 서로 상쇄됐다(실측: longlCmdType 이 1/2 로
    번갈아 찍힘). 그래서 MPC 출력을 다른 이름으로 돌리고 이 노드가 최종
    /ctrl_cmd 를 단독 발행하도록 바꿨다.
"""
import sys
import select
import termios
import tty

import rospy
from morai_msgs.msg import CtrlCmd, EventInfo

HELP = """
========================================================
  인지 튜닝 수동주행  (MPC 게이트)
========================================================
   w / s   : 목표속도  +/- 스텝    (s 로 음수까지 = 후진)
   a / d   : 조향  좌 / 우
   space   : 즉시 정지 (속도 0)
   x       : 조향 중립
   t       : 수동 <-> 자율 전환   ★ 자율이면 MPC 명령을 그대로 통과
   [ / ]   : 속도 스텝  느리게 / 빠르게
   g       : 기어 D + Auto 모드 재전송  (안 움직이면 이것부터)
   p       : 기어 P (주차/정지)
   q       : 종료
--------------------------------------------------------
 튜닝 팁: 장애물 옆에 세우고 a/d 로 차를 돌려가며 RViz 에서
          3 NONGROUND 색과 4 DETECTION 박스를 확인한다.
========================================================
"""

MAX_KMH = 30.0
MAX_STEER = 0.6
STEER_STEP = 0.05


def get_key(settings, timeout=0.05):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class Teleop(object):
    def __init__(self):
        self.manual = rospy.get_param('~start_manual', True)
        self.speed_kmh = 0.0
        self.steer = 0.0
        self.step = 1.0
        self.last_mpc = None
        self.mpc_seen = 0

        in_topic = rospy.get_param('~mpc_cmd_topic', '/control/mpc_cmd')
        self.pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        rospy.Subscriber(in_topic, CtrlCmd, self.mpc_cb, queue_size=1)
        self.in_topic = in_topic

        # MORAI 는 ctrl_mode=2(Auto) + gear=4(D) 로 바꿔주기 전에는 /ctrl_cmd 를
        # 무시하고 brake=1.0 을 물고 서 있는다(실측: /Competition_topic 의
        # brake 가 1.0 고정). 그래서 시작할 때 한 번 넣어준다.
        self.event_pub = rospy.Publisher('/EventCmd', EventInfo, queue_size=1)

    def send_gear(self, gear, ctrl_mode=2):
        ev = EventInfo()
        ev.option = 7          # gear + ctrl_mode 둘 다 적용
        ev.ctrl_mode = ctrl_mode
        ev.gear = gear
        self.event_pub.publish(ev)

    def mpc_cb(self, msg):
        self.last_mpc = msg
        self.mpc_seen += 1

    def publish(self):
        if self.manual:
            cmd = CtrlCmd()
            cmd.longlCmdType = 2          # velocity 제어
            cmd.velocity = self.speed_kmh
            cmd.steering = self.steer
            self.pub.publish(cmd)
        elif self.last_mpc is not None:
            self.pub.publish(self.last_mpc)


def main():
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('teleop_tuning_node')
    t = Teleop()
    rate = rospy.Rate(30)

    print(HELP)
    print(f" MPC 입력: {t.in_topic}")
    print(f"[상태] {'수동' if t.manual else '자율'}")

    # 퍼블리셔 연결이 잡힐 때까지 잠깐 기다린 뒤 기어를 넣는다.
    rospy.sleep(0.7)
    for _ in range(3):
        t.send_gear(4)          # D + Auto
        rospy.sleep(0.15)
    print(" 기어 D / Auto 모드 전송 완료 (안 움직이면 g 키로 재전송)\n")

    try:
        while not rospy.is_shutdown():
            key = get_key(settings)

            if key == 'q':
                break
            elif key == 't':
                t.manual = not t.manual
                t.speed_kmh = 0.0
                t.steer = 0.0
                sys.stdout.write(
                    f"\r[상태] {'수동 - 키보드가 차를 잡는다' if t.manual else '자율 - MPC 통과'}"
                    "                    \n")
            elif key == 'w':
                t.speed_kmh = min(MAX_KMH, t.speed_kmh + t.step)
            elif key == 's':
                t.speed_kmh = max(-MAX_KMH, t.speed_kmh - t.step)
            elif key == 'a':
                t.steer = min(MAX_STEER, t.steer + STEER_STEP)
            elif key == 'd':
                t.steer = max(-MAX_STEER, t.steer - STEER_STEP)
            elif key == 'x':
                t.steer = 0.0
            elif key == ' ':
                t.speed_kmh = 0.0
            elif key == '[':
                t.step = max(0.5, t.step - 0.5)
            elif key == ']':
                t.step = min(5.0, t.step + 0.5)
            elif key == 'g':
                t.send_gear(4)
                sys.stdout.write("\r[기어] D + Auto 재전송                    \n")
            elif key == 'p':
                t.speed_kmh = 0.0
                t.send_gear(1)
                sys.stdout.write("\r[기어] P (주차)                          \n")

            t.publish()

            if t.manual:
                sys.stdout.write(
                    f"\r 수동 | 목표 {t.speed_kmh:6.1f} km/h | 조향 {t.steer:+.2f} rad"
                    f" | 스텝 {t.step:.1f}    ")
            else:
                sys.stdout.write(
                    f"\r 자율 | MPC 수신 {t.mpc_seen}     ")
            sys.stdout.flush()
            rate.sleep()
    finally:
        cmd = CtrlCmd()
        cmd.longlCmdType = 2
        cmd.velocity = 0.0
        cmd.steering = 0.0
        for _ in range(5):
            t.pub.publish(cmd)
            rospy.sleep(0.05)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("\n종료 - 정지 명령 발행")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
