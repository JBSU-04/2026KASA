#!/usr/bin/env python3
"""
mission_command_adapter — 음영구간 미션을 공통 미션 계약으로 잇는다.

mission_pkg(정적·동적 장애물)는 non_gps_msgs/MissionCommand 를 내는데
mission_manager 는 mission_planner_msgs/BehaviorConstraint 를 받는다.
두 타입이 달라 그대로는 연결되지 않는다. 여기서 변환만 한다 —
미션 코드를 고치는 것보다 안전하다.

필드 대응
  active         -> active
  target_speed   -> max_speed   (km/h -> m/s)
  brake > 0      -> stop_required, stop_distance = 0 (그 자리 정지)
  priority       -> BehaviorConstraint 우선순위 척도로 재매핑

우선순위 재매핑이 필요한 이유: non_gps 는 static 50 / lane 100 / dynamic 150 을
쓰는데, BehaviorConstraint 는 NONE 0 / SPEED 50 / YIELD 100 / EMERGENCY 200 이다.
방향(클수록 우선)은 같지만 150 은 YIELD 와 EMERGENCY 사이에 끼어 의미가 없다.
"""
import rospy
from mission_planner_msgs.msg import BehaviorConstraint
from non_gps_msgs.msg import MissionCommand


class MissionCommandAdapter(object):
    def __init__(self):
        rospy.init_node('mission_command_adapter')

        # [{in, out, source}] — 입력 토픽과 출력 제약 토픽 쌍
        self.pairs = rospy.get_param('~pairs', [
            {'in': '/mission/static_obstacle/command',
             'out': '/mission/constraints/static_obstacle'},
            {'in': '/mission/dynamic_obstacle/command',
             'out': '/mission/constraints/dynamic_obstacle'},
        ])
        self.rate_hz = float(rospy.get_param('~rate', 20.0))
        self.timeout = float(rospy.get_param('~input_timeout', 0.5))
        self.stop_brake_threshold = float(rospy.get_param('~stop_brake_threshold', 0.5))
        self.comfortable_decel = float(rospy.get_param('~comfortable_decel', 2.5))
        self.emergency_decel = float(rospy.get_param('~emergency_decel', 5.0))

        self.pubs = {}
        self.latest = {}
        for p in self.pairs:
            name = p['in']
            self.pubs[name] = rospy.Publisher(p['out'], BehaviorConstraint,
                                              queue_size=1)
            rospy.Subscriber(name, MissionCommand, self._make_cb(name),
                             queue_size=1)

        rospy.Timer(rospy.Duration(1.0 / max(1.0, self.rate_hz)), self.tick)
        rospy.loginfo('[mission_command_adapter] ready. %d개 미션 변환',
                      len(self.pairs))

    def _make_cb(self, name):
        def cb(msg):
            self.latest[name] = (msg, rospy.Time.now().to_sec())
        return cb

    @staticmethod
    def map_priority(p, stop_required):
        """non_gps 우선순위를 BehaviorConstraint 척도로 옮긴다."""
        if stop_required:
            return BehaviorConstraint.PRIORITY_YIELD      # 100
        if p >= 100:
            return BehaviorConstraint.PRIORITY_YIELD
        if p > 0:
            return BehaviorConstraint.PRIORITY_SPEED      # 50
        return BehaviorConstraint.PRIORITY_NONE

    def tick(self, _evt):
        now = rospy.Time.now().to_sec()
        for p in self.pairs:
            name = p['in']
            pub = self.pubs[name]
            entry = self.latest.get(name)

            out = BehaviorConstraint()
            out.header.stamp = rospy.Time.now()
            out.source = BehaviorConstraint.SOURCE_NONE
            out.max_speed = -1.0
            out.comfortable_decel = self.comfortable_decel
            out.emergency_decel = self.emergency_decel

            if entry is None or (now - entry[1]) >= self.timeout:
                # 비활성도 계속 발행한다 — mission_manager 의 fail-closed 판정에 필요
                out.active = False
                out.priority = BehaviorConstraint.PRIORITY_NONE
                out.reason = 'no input'
                pub.publish(out)
                continue

            cmd = entry[0]
            out.active = bool(cmd.active)
            out.stop_required = bool(cmd.brake > self.stop_brake_threshold)
            if out.stop_required:
                out.stop_distance = 0.0
                out.max_speed = 0.0
            elif cmd.target_speed >= 0.0:
                out.max_speed = cmd.target_speed / 3.6      # km/h -> m/s
            out.priority = self.map_priority(int(cmd.priority), out.stop_required)
            out.reason = cmd.source or name
            pub.publish(out)


if __name__ == '__main__':
    try:
        MissionCommandAdapter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
