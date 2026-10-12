#!/usr/bin/env python3
"""
roundabout_launch_assist — 회전교차로 진입 승인 직후 출발 페달만 보조한다.

2026-10-11 수동 시연(19:58~19:59 백): 앞차가 합류점을 지나면 풀 가속해
0->5 m/s 약 1.3 s. 자율(10-10 18:04~18:05 백)은 MPC 페달이 PI 비례항
(throttle_kp 0.22 x 오차)이라 0.3~0.45 에 그쳐 3.4 s 걸렸다.
MPC 피팅값은 건드리지 않고, MPC 출력과 정지 노드 사이에서 출발 구간만
가속 페달을 올린다.

  MPC -> /ctrl_cmd_mpc -> 이 노드 -> /ctrl_cmd_raw -> obstacle_stop -> /ctrl_cmd

보조 조건(모두 만족할 때만, 아니면 MPC 명령 그대로 통과):
  - /mission/roundabout/state 가 COMMIT, 또는 CIRCULATE 로 바뀐 뒤 circulate_window_s 이내
  - state/상한/속도 메시지가 stale_s 이내
  - MPC 브레이크 0
  - 속도 < launch_speed_max
  - 목표속도 상한(/smpc/target_speed_mps_gated, 앞차 추종 상한 포함) - 속도 >= min_margin
페달 = max(MPC, min(accel_max, base + gain * (상한 - 속도))). 조향/브레이크는 그대로.
"""
import rospy
from std_msgs.msg import Float64
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
from mission_planner_msgs.msg import MissionState

COMMIT, CIRCULATE = 3, 4


def assist_accel(cmd_accel, cmd_brake, state, circulate_age, speed, cap, p):
    """보조 페달 값을 돌려준다. 보조하지 않으면 None."""
    if cmd_brake > 0.0 or cap is None or speed is None:
        return None
    if state == COMMIT:
        pass
    elif state == CIRCULATE and circulate_age is not None and circulate_age <= p['circulate_window_s']:
        pass
    else:
        return None
    if speed >= p['launch_speed_max'] or cap - speed < p['min_margin']:
        return None
    pedal = min(p['accel_max'], p['base'] + p['gain'] * (cap - speed))
    return max(cmd_accel, pedal)


class RoundaboutLaunchAssist(object):
    def __init__(self):
        g = rospy.get_param
        self.p = dict(
            circulate_window_s=float(g('~circulate_window_s', 3.0)),
            launch_speed_max=float(g('~launch_speed_max', 5.0)),
            min_margin=float(g('~min_margin', 0.5)),
            accel_max=float(g('~accel_max', 1.0)),
            base=float(g('~base', 0.6)),
            gain=float(g('~gain', 0.2)))
        self.stale = float(g('~stale_s', 0.3))
        self.enabled = bool(g('~enabled', True))
        self.state = self.state_t = None
        self.circ_since = None
        self.cap = self.cap_t = None
        self.speed = self.speed_t = None
        self.active = False
        self.pub = rospy.Publisher(g('~output_topic', '/ctrl_cmd_raw'), CtrlCmd, queue_size=1)
        self.dbg = rospy.Publisher('/mission/roundabout/launch_assist', Float64, queue_size=1)
        rospy.Subscriber('/mission/roundabout/state', MissionState, self.state_cb, queue_size=1)
        rospy.Subscriber(g('~cap_topic', '/smpc/target_speed_mps_gated'), Float64, self.cap_cb, queue_size=1)
        rospy.Subscriber('/Competition_topic', EgoVehicleStatus, self.ego_cb, queue_size=1)
        rospy.Subscriber(g('~input_topic', '/ctrl_cmd_mpc'), CtrlCmd, self.cmd_cb, queue_size=1, tcp_nodelay=True)

    def state_cb(self, m):
        now = rospy.Time.now().to_sec()
        if m.state == CIRCULATE and self.state != CIRCULATE:
            self.circ_since = now
        self.state, self.state_t = m.state, now

    def cap_cb(self, m):
        self.cap, self.cap_t = m.data, rospy.Time.now().to_sec()

    def ego_cb(self, m):
        self.speed, self.speed_t = m.velocity.x, rospy.Time.now().to_sec()

    def fresh(self, t, now):
        return t is not None and now - t <= self.stale

    def cmd_cb(self, cmd):
        now = rospy.Time.now().to_sec()
        acc = None
        if (self.enabled and cmd.longlCmdType == 1 and self.fresh(self.state_t, now)
                and self.fresh(self.cap_t, now) and self.fresh(self.speed_t, now)):
            age = now - self.circ_since if self.circ_since is not None else None
            acc = assist_accel(cmd.accel, cmd.brake, self.state, age, self.speed, self.cap, self.p)
        if acc is not None and acc > cmd.accel:
            out = CtrlCmd(longlCmdType=cmd.longlCmdType, accel=acc, brake=0.0, steering=cmd.steering,
                          velocity=cmd.velocity, acceleration=cmd.acceleration)
            if not self.active:
                rospy.loginfo('[launch_assist] 출발 보조 시작 v=%.2f cap=%.2f pedal %.2f->%.2f',
                              self.speed, self.cap, cmd.accel, acc)
            self.active = True
            self.pub.publish(out)
            self.dbg.publish(Float64(acc))
            return
        if self.active:
            rospy.loginfo('[launch_assist] 출발 보조 종료 v=%.2f', self.speed or 0.0)
        self.active = False
        self.pub.publish(cmd)
        self.dbg.publish(Float64(-1.0))


if __name__ == '__main__':
    rospy.init_node('roundabout_launch_assist')
    RoundaboutLaunchAssist()
    rospy.spin()
