#!/usr/bin/env python3
"""
localization_validity — /localization/state 에서 측위 유효 여부를 만들어 발행한다.

회전교차로 미션(mission_planner)은 require_localization: true 라
/localization/valid 를 요구하는데, 이 워크스페이스에는 그걸 내는 노드가
morai_sensor_fusion 뿐이고 우리는 그것을 띄우지 않는다. 그래서 미션이
FAILSAFE 로 빠진다.

map_localizer 가 이미 내는 /localization/state 로 만든다.
  [x, y, yaw, lateral, s, max_speed, match_dist]
kcity_localizer README 기준 match_dist 5 m 초과는 도로 이탈이다.

기존 코드는 건드리지 않는다. 판정 기준만 여기에 모아 둔다.
"""
import rospy
from std_msgs.msg import Bool, Float32MultiArray


class LocalizationValidity(object):
    IDX_MATCH_DIST = 6

    def __init__(self):
        rospy.init_node('localization_validity')

        self.max_match_dist = float(rospy.get_param('~max_match_dist', 5.0))
        self.timeout = float(rospy.get_param('~state_timeout', 0.5))
        self.rate_hz = float(rospy.get_param('~rate', 20.0))
        # 연속 확인을 요구해 한두 프레임 튐으로 미션이 흔들리지 않게 한다.
        self.confirm = int(rospy.get_param('~confirm_frames', 3))

        self.state = None
        self.stamp = 0.0
        self.good = 0
        self.bad = 0
        self.valid = False

        self.pub = rospy.Publisher('/localization/valid', Bool,
                                   queue_size=1, latch=True)
        rospy.Subscriber('/localization/state', Float32MultiArray,
                         self.cb, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / max(1.0, self.rate_hz)), self.tick)
        rospy.loginfo('[localization_validity] ready. match_dist <= %.1f m',
                      self.max_match_dist)

    def cb(self, msg):
        self.state = msg.data
        self.stamp = rospy.Time.now().to_sec()

    def tick(self, _evt):
        now = rospy.Time.now().to_sec()
        ok = False
        if self.state is not None and (now - self.stamp) < self.timeout:
            if len(self.state) > self.IDX_MATCH_DIST:
                md = float(self.state[self.IDX_MATCH_DIST])
                ok = 0.0 <= md <= self.max_match_dist

        if ok:
            self.good += 1
            self.bad = 0
        else:
            self.bad += 1
            self.good = 0

        if not self.valid and self.good >= self.confirm:
            self.valid = True
            rospy.loginfo('[localization_validity] 측위 유효')
        elif self.valid and self.bad >= self.confirm:
            self.valid = False
            rospy.logwarn('[localization_validity] 측위 무효 — '
                          '미션이 FAILSAFE 로 갈 수 있다')

        self.pub.publish(Bool(self.valid))


if __name__ == '__main__':
    try:
        LocalizationValidity()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
