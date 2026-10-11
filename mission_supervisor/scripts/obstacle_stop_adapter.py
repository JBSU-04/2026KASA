#!/usr/bin/env python3
"""
obstacle_stop_adapter — obstacle_stop_node 의 정지 명령을 미션 계약으로 옮긴다.

정동적 트리의 obstacle_stop_node 는 원래 /ctrl_cmd 를 직접 발행했다.
그대로 두면 control_arbiter 와 함께 /ctrl_cmd publisher 가 둘이 되어
단일 publisher 계약이 깨지고 MPC 와 종방향 권한도 겹친다.

그래서 그 노드는 손대지 않고(검증된 로직) 출력 토픽만
/control/obstacle_stop_cmd 로 돌려둔 뒤, 여기서 BehaviorConstraint 로
바꿔 mission_manager 에 넣는다. 정지는 constraint_to_speed_cap 이
속도캡으로 환산해 MPC 가 스스로 계획한다.

입력: /control/obstacle_stop_cmd (morai_msgs/CtrlCmd)  brake > 0 이면 정지 요구
출력: /mission/constraints/obstacle_stop (BehaviorConstraint)
"""
import rospy
from morai_msgs.msg import CtrlCmd
from mission_planner_msgs.msg import BehaviorConstraint
from mission_supervisor.msg import SectionState


class ObstacleStopAdapter(object):
    def __init__(self):
        rospy.init_node('obstacle_stop_adapter')

        self.rate_hz = float(rospy.get_param('~rate', 20.0))
        self.timeout = float(rospy.get_param('~input_timeout', 0.7))
        self.brake_threshold = float(rospy.get_param('~brake_threshold', 0.1))
        self.comfortable_decel = float(rospy.get_param('~comfortable_decel', 2.5))
        self.emergency_decel = float(rospy.get_param('~emergency_decel', 5.0))
        # 이 구간에서만 정지를 허용한다. 비우면 전 구간.
        # 동적 장애물 구간(M3) 밖에서 남의 차를 보고 서버리는 것을 막는다.
        self.sections = [s for s in rospy.get_param('~sections', []) if s]

        self.cmd = None
        self.cmd_t = 0.0
        self.section_id = ''

        self.pub = rospy.Publisher('/mission/constraints/obstacle_stop',
                                   BehaviorConstraint, queue_size=1)
        rospy.Subscriber('/control/obstacle_stop_cmd', CtrlCmd,
                         self.cmd_cb, queue_size=1)
        rospy.Subscriber('/mission/active_section', SectionState,
                         self.sec_cb, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / max(1.0, self.rate_hz)), self.tick)
        rospy.loginfo('[obstacle_stop_adapter] ready. 구간 제한=%s',
                      ','.join(self.sections) or '없음(전 구간)')

    def cmd_cb(self, msg):
        self.cmd = msg
        self.cmd_t = rospy.Time.now().to_sec()

    def sec_cb(self, msg):
        self.section_id = msg.section_id

    def tick(self, _evt):
        out = BehaviorConstraint()
        out.header.stamp = rospy.Time.now()
        out.max_speed = -1.0
        out.comfortable_decel = self.comfortable_decel
        out.emergency_decel = self.emergency_decel
        out.priority = BehaviorConstraint.PRIORITY_NONE

        now = rospy.Time.now().to_sec()
        fresh = self.cmd is not None and (now - self.cmd_t) < self.timeout
        in_section = (not self.sections) or (self.section_id in self.sections)

        if fresh and in_section and self.cmd.brake > self.brake_threshold:
            out.active = True
            out.priority = BehaviorConstraint.PRIORITY_YIELD
            out.stop_required = True
            out.stop_distance = 0.0          # 그 자리 정지
            out.max_speed = 0.0
            out.reason = 'dynamic obstacle stop (brake %.2f)' % self.cmd.brake
        else:
            out.active = False
            out.reason = 'no stop' if fresh else 'obstacle_stop_node stale'

        self.pub.publish(out)


if __name__ == '__main__':
    try:
        ObstacleStopAdapter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
