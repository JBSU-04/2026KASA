#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
명령 arbiter — MPC 출력(/ctrl_cmd_raw) 하류에서 안전명령을 최우선 융합해 /ctrl_cmd 발행.

Apollo Guardian / Autoware AEB 패턴: 정상 제어명령을 통과시키되, 안전결정층(aeb_decision,
intersection_manager)이 개입을 요구하면 그 명령을 덮어쓴다. MPC 소스는 건드리지 않고 launch
remap 으로 MPC /ctrl_cmd -> /ctrl_cmd_raw 만 바꿔 이 노드를 사이에 끼운다.

융합 규칙(보수적 = 가장 제한적):
  - 어느 producer든 emergency(FULL)  -> longlCmdType=1, brake=full, accel=0 (조향은 MPC 유지)
  - 부분제동(PB1/PB2)                 -> longlCmdType=1, brake=max(brakes), accel=0
  - 완만감속(FCW/HOLD, target_speed)  -> longlCmdType=2, velocity=min(raw.velocity, caps)
  - localization invalid / raw stale  -> Type 1 full brake를 계속 발행
조향(steering)은 항상 MPC raw 를 유지 = 제동 중 차선 유지.

입력 : /ctrl_cmd_raw(morai_msgs/CtrlCmd), /aeb/command, /intersection/command(aeb_safety/AebCommand)
출력 : /ctrl_cmd(morai_msgs/CtrlCmd)
"""
import rospy
from morai_msgs.msg import CtrlCmd
from aeb_safety.msg import AebCommand
from std_msgs.msg import Bool


class AebArbiter(object):
    def __init__(self):
        rospy.init_node('aeb_arbiter', anonymous=False)

        self.cmd_timeout = rospy.get_param('~cmd_timeout', 0.5)   # 이보다 오래된 명령은 무시
        self.raw_timeout = rospy.get_param('~raw_timeout', 0.5)   # MPC 명령 stale 시 안전정지
        self.full_brake = rospy.get_param('~full_brake', 1.0)

        self.aeb = None
        self.aeb_t = 0.0
        self.xing = None
        self.xing_t = 0.0
        self.last_raw_t = 0.0
        self.localization_valid = False
        self.localization_seen = False

        self.pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        rospy.Subscriber('/ctrl_cmd_raw', CtrlCmd, self.raw_cb, queue_size=1)
        rospy.Subscriber('/aeb/command', AebCommand, self.aeb_cb, queue_size=1)
        rospy.Subscriber('/intersection/command', AebCommand, self.xing_cb, queue_size=1)
        rospy.Subscriber('/localization/valid', Bool, self.localization_cb, queue_size=1)
        # MPC 가 죽어 raw 가 끊겨도 안전정지를 유지하기 위한 워치독
        self.timer = rospy.Timer(rospy.Duration(0.1), self.watchdog)
        rospy.loginfo('[aeb_arbiter] ready. /ctrl_cmd_raw -> (fuse aeb/intersection) -> /ctrl_cmd')

    def aeb_cb(self, msg):
        self.aeb = msg
        self.aeb_t = rospy.Time.now().to_sec()

    def xing_cb(self, msg):
        self.xing = msg
        self.xing_t = rospy.Time.now().to_sec()

    def localization_cb(self, msg):
        self.localization_valid = bool(msg.data)
        self.localization_seen = True

    def stop_command(self, steering=0.0):
        out = CtrlCmd()
        out.longlCmdType = 1
        out.accel = 0.0
        out.brake = self.full_brake
        out.steering = steering
        out.velocity = 0.0
        out.acceleration = 0.0
        return out

    def _fresh(self, msg, t):
        return msg is not None and (rospy.Time.now().to_sec() - t) < self.cmd_timeout

    def _active(self):
        """유효한 안전명령 리스트 반환."""
        out = []
        if self._fresh(self.aeb, self.aeb_t):
            out.append(self.aeb)
        if self._fresh(self.xing, self.xing_t):
            out.append(self.xing)
        return out

    def raw_cb(self, raw):
        self.last_raw_t = rospy.Time.now().to_sec()
        if not self.localization_seen or not self.localization_valid:
            self.pub.publish(self.stop_command(raw.steering))
            return
        out = CtrlCmd()
        # 기본 = MPC 그대로 (특히 steering 은 항상 유지)
        out.longlCmdType = raw.longlCmdType
        out.accel = raw.accel
        out.brake = raw.brake
        out.steering = raw.steering
        out.velocity = raw.velocity
        out.acceleration = raw.acceleration

        cmds = self._active()
        emergency = any(c.emergency or c.stage == AebCommand.STAGE_FULL for c in cmds)
        brakes = [c.brake for c in cmds if c.stage in (AebCommand.STAGE_PB1, AebCommand.STAGE_PB2)]
        caps = [c.target_speed for c in cmds
                if c.stage in (AebCommand.STAGE_FCW, AebCommand.STAGE_HOLD) and c.target_speed >= 0.0]

        if emergency:
            out.longlCmdType = 1
            out.accel = 0.0
            out.brake = self.full_brake
            out.velocity = 0.0
        elif brakes:
            out.longlCmdType = 1
            out.accel = 0.0
            out.brake = max(brakes)
            out.velocity = 0.0
        elif caps:
            cap = min(caps)
            # raw 가 속도명령이면 캡, 아니면 속도명령으로 전환해 캡 적용
            base_v = raw.velocity if raw.longlCmdType == 2 else cap
            out.longlCmdType = 2
            out.velocity = min(base_v, cap)
            out.brake = 0.0
            out.accel = 0.0
        # else: raw passthrough

        self.pub.publish(out)

    def watchdog(self, _evt):
        """MPC(raw)가 끊긴 채 긴급명령이 살아있으면 정지 유지."""
        now = rospy.Time.now().to_sec()
        if (now - self.last_raw_t) < self.raw_timeout and self.localization_valid:
            return
        self.pub.publish(self.stop_command())


if __name__ == '__main__':
    try:
        AebArbiter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
