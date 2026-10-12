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
from std_msgs.msg import Float64, Float64MultiArray
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
from mission_planner_msgs.msg import MissionState

COMMIT, CIRCULATE = 3, 4


EXIT = 5


def acc_accel(cmd_accel, cmd_brake, state, acc, speed, cap, p):
    """2026-10-12 ACC 거리 유지 가속 보조. acc = (active, a_des, v_des). 보조하지 않으면 None.

    진입/순환/이탈 중 미션 ACC 가 가속을 요구(a_des >= acc_min_accel)하고, MPC 가 제동 중이 아니며,
    상한이 현재 속도보다 높을 때만 a_des 에 비례한 페달을 보탠다. 감속은 MPC 가 v_des 상한으로 한다."""
    if acc is None or cmd_brake > 0.0 or cap is None or speed is None:
        return None
    active, a_des, v_des = acc
    if not active or state not in (COMMIT, CIRCULATE, EXIT) or a_des < p['acc_min_accel']:
        return None
    if cap - speed < p['acc_min_margin'] or v_des - speed < p['acc_min_margin']:
        return None
    pedal = min(p['accel_max'], p['acc_base'] + p['acc_gain'] * a_des)
    return max(cmd_accel, pedal)


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
            gain=float(g('~gain', 0.2)),
            acc_min_accel=float(g('~acc_min_accel', 0.2)),
            acc_min_margin=float(g('~acc_min_margin', 0.2)),
            acc_base=float(g('~acc_base', 0.15)),
            acc_gain=float(g('~acc_gain', 0.25)))
        self.handback_s = float(g('~handback_s', 0.3))
        self.acc = None; self.acc_t = None
        self.last_assist = 0.0; self.last_assist_t = -1e9
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
        rospy.Subscriber('/mission/roundabout/acc', Float64MultiArray, self.acc_cb, queue_size=1)
        rospy.Subscriber(g('~input_topic', '/ctrl_cmd_mpc'), CtrlCmd, self.cmd_cb, queue_size=1, tcp_nodelay=True)

    def state_cb(self, m):
        now = rospy.Time.now().to_sec()
        if m.state == CIRCULATE and self.state != CIRCULATE:
            self.circ_since = now
        self.state, self.state_t = m.state, now

    def cap_cb(self, m):
        self.cap, self.cap_t = m.data, rospy.Time.now().to_sec()

    def acc_cb(self, m):
        d = list(m.data)
        if len(d) >= 7:
            self.acc, self.acc_t = (d[0] > 0.5, d[6], d[5]), rospy.Time.now().to_sec()

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
            acc_state = self.acc if self.fresh(self.acc_t, now) else None
            follow = acc_accel(cmd.accel, cmd.brake, self.state, acc_state, self.speed, self.cap, self.p)
            if follow is not None:
                acc = follow if acc is None else max(acc, follow)
        # 보조가 끝나면 handback_s 동안 페달을 MPC 값으로 서서히 넘긴다(MPC 적분기 차이로 인한 페달 급변 방지).
        #   감속 요구(상한 < 속도)나 MPC 제동이 있으면 바로 MPC 명령을 따른다.
        faded = False
        if (acc is None and cmd.brake <= 0.0 and now - self.last_assist_t < self.handback_s and
                self.cap is not None and self.speed is not None and self.cap >= self.speed):
            fade = self.last_assist * (1.0 - (now - self.last_assist_t) / max(1e-3, self.handback_s))
            if fade > cmd.accel:
                acc, faded = fade, True
        if acc is not None and acc > cmd.accel:
            out = CtrlCmd(longlCmdType=cmd.longlCmdType, accel=acc, brake=0.0, steering=cmd.steering,
                          velocity=cmd.velocity, acceleration=cmd.acceleration)
            if not self.active:
                rospy.loginfo('[launch_assist] 출발 보조 시작 v=%.2f cap=%.2f pedal %.2f->%.2f',
                              self.speed, self.cap, cmd.accel, acc)
            self.active = True
            if not faded:
                self.last_assist, self.last_assist_t = acc, now
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
