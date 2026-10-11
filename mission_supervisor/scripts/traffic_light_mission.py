#!/usr/bin/env python3
"""
traffic_light_mission — 신호등 정지선을 공통 미션 계약으로 포장한다.

계산은 이미 kcity_localizer/map_localizer 가 한다. mgeo 정지선과
/traffic_light_color(YOLO 추론)를 결합해 다음을 낸다.
  /localization/stop_ahead    Float32MultiArray [dist, stop_x, stop_y, ...]  (없으면 dist<0)
  /localization/stop_required Bool

이 노드는 그것을 BehaviorConstraint 로 바꿔 mission_manager 에 넣는다.
속도 제어는 constraint_to_speed_cap 이 v=sqrt(2*a*d) 로 환산해 MPC 에 준다.
여기서 페달이나 조향을 직접 만지지 않는다.

규정 채점은 '앞바퀴가 정지선을 넘는 순간'의 신호로 판정한다(3장).
map_localizer 의 거리는 차량 기준점(후륜축) 기준이므로 전륜까지의
오프셋(축거 + 전방 오버행)을 빼서 정지 목표를 앞바퀴에 맞춘다.
"""
import rospy
from std_msgs.msg import Bool, Float32MultiArray, String
from mission_planner_msgs.msg import BehaviorConstraint
from mission_supervisor.msg import SectionState


class TrafficLightMission(object):
    def __init__(self):
        rospy.init_node('traffic_light_mission')

        self.rate_hz = float(rospy.get_param('~rate', 20.0))
        self.timeout = float(rospy.get_param('~input_timeout', 0.7))
        # 규정 2-6-1 Ioniq5: 축거 3.000 + 전방 오버행 0.845
        self.front_axle_offset_m = float(
            rospy.get_param('~front_axle_offset_m', 3.845))
        # 정지선 앞에 남길 여유
        self.stop_buffer_m = float(rospy.get_param('~stop_buffer_m', 1.0))
        self.approach_speed_mps = float(
            rospy.get_param('~approach_speed_mps', -1.0))
        self.comfortable_decel = float(rospy.get_param('~comfortable_decel', 2.5))
        self.emergency_decel = float(rospy.get_param('~emergency_decel', 5.0))
        # 신호등 정지는 대회 미션 구간에서만 허용한다. path_number(csv)가
        # 바뀌면 waypoint index가 0부터 다시 시작하므로 csv도 함께 확인한다.
        self.active_csv = int(rospy.get_param('~active_csv', 0))
        self.active_idx_ranges = self._load_idx_ranges(
            rospy.get_param('~active_idx_ranges',
                            [[1800, 4600], [5400, 6200]]))

        self.stop_dist = -1.0
        self.stop_dist_t = 0.0
        self.stop_required = False
        self.stop_required_t = 0.0
        self.color = 'none'
        self.section_id = ''
        self.section_csv = -1
        self.section_index = -1
        self.section_t = 0.0

        self.pub = rospy.Publisher('/mission/constraints/traffic_light',
                                   BehaviorConstraint, queue_size=1)
        rospy.Subscriber('/localization/stop_ahead', Float32MultiArray,
                         self.ahead_cb, queue_size=1)
        rospy.Subscriber('/localization/stop_required', Bool,
                         self.req_cb, queue_size=1)
        rospy.Subscriber('/traffic_light_color', String,
                         self.color_cb, queue_size=1)
        rospy.Subscriber('/mission/active_section', SectionState,
                         self.sec_cb, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / max(1.0, self.rate_hz)), self.tick)
        rospy.loginfo('[traffic_light_mission] ready. csv=%d, idx 구간=%s',
                      self.active_csv,
                      ', '.join('%d~%d' % bounds
                                for bounds in self.active_idx_ranges))

    @staticmethod
    def _load_idx_ranges(raw_ranges):
        ranges = []
        for raw in raw_ranges:
            if not isinstance(raw, (list, tuple)) or len(raw) != 2:
                raise ValueError('active_idx_ranges 항목은 [start, end] 형식이어야 합니다: %r'
                                 % (raw,))
            start, end = int(raw[0]), int(raw[1])
            if start > end:
                raise ValueError('active_idx_ranges 시작값이 끝값보다 큽니다: %r'
                                 % (raw,))
            ranges.append((start, end))
        if not ranges:
            raise ValueError('active_idx_ranges는 하나 이상 필요합니다')
        return ranges

    def ahead_cb(self, msg):
        self.stop_dist = float(msg.data[0]) if msg.data else -1.0
        self.stop_dist_t = rospy.Time.now().to_sec()

    def req_cb(self, msg):
        self.stop_required = bool(msg.data)
        self.stop_required_t = rospy.Time.now().to_sec()

    def color_cb(self, msg):
        self.color = msg.data

    def sec_cb(self, msg):
        self.section_id = msg.section_id
        self.section_csv = int(msg.csv)
        self.section_index = int(msg.index)
        self.section_t = rospy.Time.now().to_sec()

    def _fresh(self, t):
        return (rospy.Time.now().to_sec() - t) < self.timeout

    def _in_active_idx_range(self):
        if self.section_csv != self.active_csv or not self._fresh(self.section_t):
            return False
        return any(start <= self.section_index <= end
                   for start, end in self.active_idx_ranges)

    def tick(self, _evt):
        out = BehaviorConstraint()
        out.header.stamp = rospy.Time.now()
        out.max_speed = -1.0
        out.comfortable_decel = self.comfortable_decel
        out.emergency_decel = self.emergency_decel
        out.priority = BehaviorConstraint.PRIORITY_NONE

        in_section = self._in_active_idx_range()
        have = self._fresh(self.stop_dist_t) and self._fresh(self.stop_required_t)

        if not (in_section and have and self.stop_required and self.stop_dist >= 0.0):
            # 비활성도 계속 발행한다 - mission_manager 의 fail-closed 판정에 필요
            out.active = False
            if not in_section:
                out.reason = 'outside traffic-light mission index range'
            else:
                out.reason = 'no stop required' if have else 'localizer stale'
            self.pub.publish(out)
            return

        # 앞바퀴가 정지선을 넘지 않도록 목표를 당긴다.
        d = self.stop_dist - self.front_axle_offset_m - self.stop_buffer_m

        out.active = True
        out.priority = BehaviorConstraint.PRIORITY_YIELD
        out.stop_required = True
        out.stop_distance = max(0.0, d)
        if self.approach_speed_mps > 0.0:
            out.max_speed = self.approach_speed_mps
        out.reason = 'traffic light %s, stop in %.1f m' % (self.color, out.stop_distance)
        self.pub.publish(out)


if __name__ == '__main__':
    try:
        TrafficLightMission()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
