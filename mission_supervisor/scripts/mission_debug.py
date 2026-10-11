#!/usr/bin/env python3
"""
mission_debug — 미션·제어 상태를 한 화면으로 모아 주기적으로 찍는다.

여러 토픽을 rostopic echo 로 따로 보다 보면 어느 단계에서 끊겼는지 알기
어렵다. 사슬 순서대로 한 줄씩 찍어서, 끊긴 지점이 바로 보이게 한다.

  구간 라우터 -> 미션 -> mission_manager -> 속도캡 -> arbiter -> /ctrl_cmd

각 줄 앞의 표시
  OK    정상 수신 중
  --    발행자가 없거나 오래됨(그 단계에서 끊김)
"""
import rospy
from std_msgs.msg import Bool, Float64, String
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
from nav_msgs.msg import Path
from mission_supervisor.msg import SectionState
from mission_planner_msgs.msg import BehaviorConstraint

try:
    from mission_planner_msgs.msg import MissionState
    HAVE_MISSION_STATE = True
except ImportError:
    HAVE_MISSION_STATE = False

RA_STATE = {0: 'IDLE', 1: 'APPROACH', 2: 'YIELD', 3: 'COMMIT',
            4: 'CIRCULATE', 5: 'EXIT', 6: 'FAILSAFE'}


class Slot(object):
    def __init__(self):
        self.msg = None
        self.t = 0.0

    def set(self, m):
        self.msg = m
        self.t = rospy.Time.now().to_sec()

    def fresh(self, timeout=1.0):
        return self.msg is not None and \
            (rospy.Time.now().to_sec() - self.t) < timeout


class MissionDebug(object):
    def __init__(self):
        rospy.init_node('mission_debug')
        self.period = float(rospy.get_param('~period', 1.0))

        self.s = {k: Slot() for k in
                  ('sec', 'ra', 'bc', 'cap', 'seq', 'valid',
                   'raw', 'out', 'ego', 'path', 'solve')}

        sub = rospy.Subscriber
        sub('/mission/active_section', SectionState, lambda m: self.s['sec'].set(m), queue_size=1)
        sub('/mission/behavior_constraint', BehaviorConstraint, lambda m: self.s['bc'].set(m), queue_size=1)
        sub('/mission/speed_cap_mps', Float64, lambda m: self.s['cap'].set(m), queue_size=1)
        sub('/mission/sequence_state', String, lambda m: self.s['seq'].set(m), queue_size=1)
        sub('/localization/valid', Bool, lambda m: self.s['valid'].set(m), queue_size=1)
        sub('/control/raw_cmd', CtrlCmd, lambda m: self.s['raw'].set(m), queue_size=1)
        sub('/ctrl_cmd', CtrlCmd, lambda m: self.s['out'].set(m), queue_size=1)
        sub('/Competition_topic', EgoVehicleStatus, lambda m: self.s['ego'].set(m), queue_size=1)
        sub('/local_path', Path, lambda m: self.s['path'].set(m), queue_size=1)
        sub('/mpc/controller_time_ms', Float64, lambda m: self.s['solve'].set(m), queue_size=1)
        if HAVE_MISSION_STATE:
            sub('/mission/roundabout/state', MissionState, lambda m: self.s['ra'].set(m), queue_size=1)

        rospy.Timer(rospy.Duration(self.period), self.tick)
        rospy.loginfo('[mission_debug] ready. %.1f초마다 상태 출력', self.period)

    @staticmethod
    def mark(ok):
        return 'OK' if ok else '--'

    def tick(self, _evt):
        L = []
        L.append('=' * 74)

        # 1) 측위
        v = self.s['valid']
        L.append('%s 측위     valid=%s' % (
            self.mark(v.fresh()), v.msg.data if v.fresh() else '?'))

        # 2) 구간 라우터
        sec = self.s['sec']
        if sec.fresh():
            m = sec.msg
            L.append('%s 구간     %-8s csv=%d idx=%-6d 진행 %.2f  %s  exec=%s'
                     % (self.mark(True), m.section_id or '(구간밖)', m.csv, m.index,
                        m.progress, '위치유효' if m.index_valid else '속도적산',
                        ','.join(m.executors) or '-'))
        else:
            L.append('%s 구간     section_router 없음' % self.mark(False))

        # 3) 미션 (회전교차로)
        ra = self.s['ra']
        if ra.fresh():
            r = ra.msg
            L.append('%s 회전교차로 %-9s gap=%s 트랙 %d개  양보선 %.1f m  %s'
                     % (self.mark(True), RA_STATE.get(r.state, '?'), r.gap_safe,
                        r.tracked_object_count, r.distance_to_yield, r.reason[:28]))
        else:
            L.append('%s 회전교차로 미실행' % self.mark(False))

        # 4) 제약 합성
        bc = self.s['bc']
        if bc.fresh():
            b = bc.msg
            L.append('%s 제약     active=%-5s prio=%-3d max_speed=%6.2f stop=%-5s d=%.1f'
                     % (self.mark(True), b.active, b.priority, b.max_speed,
                        b.stop_required, b.stop_distance))
        else:
            L.append('%s 제약     mission_manager 없음' % self.mark(False))

        # 5) 속도캡
        cap = self.s['cap']
        capv = cap.msg.data if cap.fresh() else None
        L.append('%s 속도캡   %s' % (
            self.mark(cap.fresh()),
            ('제한 없음' if capv is not None and capv < 0 else '%.2f m/s' % capv)
            if capv is not None else '?'))

        # 6) 제어
        raw, out, ego = self.s['raw'], self.s['out'], self.s['ego']
        spd = ego.msg.velocity.x if ego.fresh() else float('nan')
        if raw.fresh():
            r = raw.msg
            L.append('%s MPC      type=%d accel=%.3f brake=%.3f steer=%+.4f'
                     % (self.mark(True), r.longlCmdType, r.accel, r.brake, r.steering))
        else:
            L.append('%s MPC      /control/raw_cmd 없음' % self.mark(False))
        if out.fresh():
            o = out.msg
            same = raw.fresh() and abs(o.accel - raw.msg.accel) < 1e-6 \
                and abs(o.brake - raw.msg.brake) < 1e-6
            L.append('%s /ctrl_cmd type=%d accel=%.3f brake=%.3f steer=%+.4f  %s'
                     % (self.mark(True), o.longlCmdType, o.accel, o.brake, o.steering,
                        '통과' if same else '<-- arbiter 개입'))
        else:
            L.append('%s /ctrl_cmd 없음' % self.mark(False))

        # 7) 참고 지표
        path = self.s['path']
        npts = len(path.msg.poses) if path.fresh() else 0
        solve = self.s['solve'].msg.data if self.s['solve'].fresh() else float('nan')
        L.append('   속도 %.2f m/s (%.1f km/h)   local_path %d점   솔브 %.1f ms'
                 % (spd, spd * 3.6, npts, solve))

        seq = self.s['seq']
        if seq.fresh():
            L.append('   %s' % seq.msg.data)

        rospy.loginfo('\n' + '\n'.join(L))


if __name__ == '__main__':
    try:
        MissionDebug()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
