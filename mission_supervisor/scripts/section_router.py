#!/usr/bin/env python3
"""
section_router — 지금 어느 미션 구간에 있는지만 판정한다.

설계 원칙(계획안 §5): 미션 활성화는 위치의 순수 함수다. "직전 미션 완료"를
조건에 넣지 않는다. 순서는 단일 루프 경로가 이미 보장하고, 조건으로 넣으면
한 번의 인지 실패가 이후 미션 전부를 잠근다.

입력:
  /final_waypoint   (waypoint_maker/Lane)  waypoints[0].waypoint_index = 현재 인덱스
  /gps_state        (waypoint_maker/State) lane_number = 현재 path_number
  /Competition_topic(morai_msgs/EgoVehicleStatus) velocity.x = 실측 전진속도

출력:
  /mission/active_section (mission_supervisor/SectionState)

GPS 상실(M8) 폴백:
  /final_waypoint 가 끊기면 인덱스를 알 수 없다. 이때는 마지막 유효 인덱스에서
  /Competition_topic 속도를 적산해 인덱스를 추정한다(웨이포인트 간격 0.1 m).
  절대 위치는 흘러도 되지만, 구간 종료 판정에는 이 추정이 유일한 수단이다.
"""
import rospy
from std_msgs.msg import Header, Bool
import os
import sys
import rospkg
sys.path.insert(0, os.path.join(rospkg.RosPack().get_path("new_lane"), "scripts"))
from tunnel_region import read_region, contains
from morai_msgs.msg import EgoVehicleStatus
from waypoint_maker.msg import Lane, State
from mission_supervisor.msg import SectionState


class SectionRouter(object):
    def __init__(self):
        rospy.init_node('section_router')

        self.sections = rospy.get_param('~sections', [])
        self.spacing = float(rospy.get_param('~waypoint_spacing_m', 0.1))
        self.enter_margin = int(rospy.get_param('~enter_margin_idx', 0))
        self.exit_margin = int(rospy.get_param('~exit_margin_idx', 0))
        self.index_timeout = float(rospy.get_param('~index_timeout', 0.5))
        self.rate_hz = float(rospy.get_param('~rate', 20.0))

        self.index = -1
        self.csv = -1
        self.speed_state = -1
        self.index_stamp = 0.0
        self.speed_mps = 0.0
        self.speed_stamp = 0.0
        self.est_index = None          # 폴백 추정 인덱스(float)
        self.est_stamp = 0.0
        self.current_id = ''
        self.section_enter_t = 0.0

        if not self.sections:
            rospy.logwarn('[section_router] sections 파라미터가 비어 있다. '
                          '모든 구간이 비활성으로 나간다.')

        self.tunnel_region = read_region(rospy.get_param)
        self.random_pub = rospy.Publisher("/tunnel/random_obstacle_active", Bool, queue_size=1, latch=True)
        self.pub = rospy.Publisher('/mission/active_section', SectionState,
                                   queue_size=1)
        rospy.Subscriber('/final_waypoint', Lane, self.wp_cb, queue_size=1)
        rospy.Subscriber('/gps_state', State, self.state_cb, queue_size=1)
        rospy.Subscriber('/Competition_topic', EgoVehicleStatus,
                         self.ego_cb, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / max(1.0, self.rate_hz)), self.tick)
        rospy.loginfo('[section_router] ready. sections=%d', len(self.sections))

    # ---------- 입력 ----------

    def wp_cb(self, msg):
        if not msg.waypoints:
            return
        w = msg.waypoints[0]
        self.index = int(w.waypoint_index)
        self.speed_state = int(w.mission_state)
        if w.lane_number >= 0:
            self.csv = int(w.lane_number)
        self.index_stamp = rospy.Time.now().to_sec()
        # 유효 인덱스를 받은 순간 폴백 추정치를 실측으로 되돌린다.
        self.est_index = float(self.index)
        self.est_stamp = self.index_stamp

    def state_cb(self, msg):
        self.csv = int(msg.lane_number)

    def ego_cb(self, msg):
        self.speed_mps = float(msg.velocity.x)
        self.speed_stamp = rospy.Time.now().to_sec()

    # ---------- 인덱스 확보 ----------

    def resolve_index(self, now):
        """(index, valid) 반환. valid=False 면 속도 적산 추정치다."""
        if self.index >= 0 and (now - self.index_stamp) < self.index_timeout:
            return self.index, True

        # 폴백: 마지막 유효 인덱스에서 주행거리를 적산한다.
        if self.est_index is None:
            return -1, False
        if (now - self.speed_stamp) < self.index_timeout:
            dt = max(0.0, now - self.est_stamp)
            self.est_index += (self.speed_mps * dt) / max(1e-6, self.spacing)
        self.est_stamp = now
        return int(self.est_index), False

    # ---------- 구간 판정 ----------

    def match(self, csv, index):
        """히스테리시스 적용: 진입은 일찍, 이탈은 늦게."""
        for sec in self.sections:
            if int(sec.get('csv', -1)) != csv:
                continue
            idx = sec.get('idx', [0, 0])
            lo, hi = int(idx[0]), int(idx[1])
            active = (sec.get('id', '') == self.current_id)
            # A section may need an exact physical boundary (for example the
            # GPS-shadow handoff). Keep the global hysteresis as the default,
            # but allow that boundary to override either margin explicitly.
            enter_margin = int(sec.get('enter_margin_idx', self.enter_margin))
            exit_margin = int(sec.get('exit_margin_idx', self.exit_margin))
            lo_eff = lo - enter_margin
            hi_eff = hi + (exit_margin if active else 0)
            if lo_eff <= index <= hi_eff:
                return sec
        return None

    def tick(self, _evt):
        now = rospy.Time.now().to_sec()
        index, valid = self.resolve_index(now)

        msg = SectionState()
        msg.header = Header(stamp=rospy.Time.now())
        msg.csv = self.csv
        msg.index = index
        msg.speed_state = self.speed_state
        msg.index_valid = valid

        sec = self.match(self.csv, index) if index >= 0 else None
        if sec is None:
            if self.current_id:
                rospy.loginfo('[section_router] 구간 이탈: %s (체류 %.1fs)',
                              self.current_id, now - self.section_enter_t)
            self.current_id = ''
            msg.section_id = ''
            msg.section_name = ''
            msg.progress = 0.0
            msg.speed_limit_exempt = False
            msg.dwell_sec = 0.0
            msg.executors = []
        else:
            sid = str(sec.get('id', ''))
            if sid != self.current_id:
                rospy.loginfo('[section_router] 구간 진입: %s (%s) idx=%d',
                              sid, sec.get('name', ''), index)
                self.current_id = sid
                self.section_enter_t = now
            lo, hi = int(sec['idx'][0]), int(sec['idx'][1])
            span = max(1, hi - lo)
            msg.section_id = sid
            msg.section_name = str(sec.get('name', ''))
            msg.progress = min(1.0, max(0.0, (index - lo) / float(span)))
            msg.speed_limit_exempt = bool(sec.get('speed_limit_exempt', False))
            msg.dwell_sec = now - self.section_enter_t
            msg.executors = [str(e) for e in sec.get('executors', [])]

        random_active = bool(sec is not None and msg.section_id in ('M7B','M8') and valid and
                             contains(self.tunnel_region,self.csv,index))
        if random_active and msg.section_id == 'M7B':
            msg.executors = ['static_obstacle','dynamic_obstacle']
        self.random_pub.publish(Bool(data=random_active))
        self.pub.publish(msg)


if __name__ == '__main__':
    try:
        SectionRouter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
