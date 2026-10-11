#!/usr/bin/env python3
"""
sequence_monitor — 미션 순서와 구간 체류시간을 기록만 한다.

규정 3장은 미션 순서 이행을 요구하지만, 이 노드는 아무것도 차단하지 않는다.
차단하면 한 번의 인지 실패가 이후 미션 전부를 잠근다(계획안 §5).
쓸모는 1회차 종료 시 무엇을 고칠지 즉시 아는 것이다 — 주행 기회가 2회뿐이라
이 정보가 2회차 전략을 결정한다.

입력: /mission/active_section, /mission/behavior_constraint
출력: /mission/sequence_state (std_msgs/String)  예)
      M1 DONE 12.3s | M2 SKIPPED 8.1s | M5 ACTIVE 4.0s | total 143.2s
"""
import rospy
from std_msgs.msg import String
from mission_supervisor.msg import SectionState
from mission_planner_msgs.msg import BehaviorConstraint


class SequenceMonitor(object):
    def __init__(self):
        rospy.init_node('sequence_monitor')

        self.expected = [str(s) for s in rospy.get_param('~expected_order', [])]
        self.rate_hz = float(rospy.get_param('~rate', 2.0))
        self.limit_sec = float(rospy.get_param('~time_limit_sec', 900.0))

        self.order = []          # 방문 순서
        self.log = {}            # id -> dict(dwell, constrained)
        self.current = ''
        self.constraint_active = False
        self.t0 = rospy.Time.now().to_sec()

        self.pub = rospy.Publisher('/mission/sequence_state', String, queue_size=1)
        rospy.Subscriber('/mission/active_section', SectionState,
                         self.sec_cb, queue_size=1)
        rospy.Subscriber('/mission/behavior_constraint', BehaviorConstraint,
                         self.con_cb, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / max(0.2, self.rate_hz)), self.tick)
        rospy.loginfo('[sequence_monitor] ready. 관측 전용 — 제어에 영향 없음')

    def con_cb(self, msg):
        self.constraint_active = bool(msg.active)

    def sec_cb(self, msg):
        sid = msg.section_id
        if sid and sid not in self.log:
            self.log[sid] = {'dwell': 0.0, 'constrained': False}
            self.order.append(sid)
        if sid:
            self.log[sid]['dwell'] = msg.dwell_sec
            # 그 구간에서 미션이 한 번이라도 제약을 냈는가 = 실제 수행 흔적
            if self.constraint_active:
                self.log[sid]['constrained'] = True
        self.current = sid

    def tick(self, _evt):
        elapsed = rospy.Time.now().to_sec() - self.t0
        parts = []
        for sid in self.order:
            e = self.log[sid]
            if sid == self.current:
                tag = 'ACTIVE'
            elif e['constrained']:
                tag = 'DONE'
            else:
                tag = 'SKIPPED'
            parts.append('%s %s %.1fs' % (sid, tag, e['dwell']))

        missed = [s for s in self.expected if s not in self.log]
        if missed:
            parts.append('pending=' + ','.join(missed))

        parts.append('total %.1fs/%.0fs' % (elapsed, self.limit_sec))
        text = ' | '.join(parts) if parts else 'no section entered'
        self.pub.publish(String(text))

        if elapsed > self.limit_sec:
            rospy.logwarn_throttle(10.0,
                                   '[sequence_monitor] 제한시간 초과: %.1fs', elapsed)


if __name__ == '__main__':
    try:
        SequenceMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
