#!/usr/bin/env python3
"""
constraint_to_speed_cap — 미션 제약을 속도 상한 하나로 환산한다.

MPC 가 종방향의 유일한 실행기이므로(계획안 §2), 미션은 페달을 만지지 않고
속도캡만 낸다. 제외한 control_constraint_gateway 의 제동곡선 계산을 그대로
쓰되 출력이 페달이 아니라 속도라는 점만 다르다.

  v_cap = min(max_speed, sqrt(2 * a_comfort * d_stop))
  d_stop < stop_epsilon 이면 0

입력: /mission/behavior_constraint (mission_planner_msgs/BehaviorConstraint)
출력: /mission/speed_cap_mps (std_msgs/Float64), 캡 없으면 -1
"""
import math
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from mission_planner_msgs.msg import BehaviorConstraint
from mission_supervisor.msg import SectionState
try:
    from waypoint_maker.msg import Lane as _WpLane
except ImportError:
    _WpLane = None


def lead_speed_cap(c, ego_speed, standstill, time_gap, default_decel):
    if (c.lead_profile_override and math.isfinite(c.lead_standstill_distance)
            and c.lead_standstill_distance >= 0.0
            and math.isfinite(c.lead_time_gap) and c.lead_time_gap > 0.0):
        standstill, time_gap = c.lead_standstill_distance, c.lead_time_gap
    usable_gap = max(0.0, c.lead_distance - standstill)
    a = c.comfortable_decel if c.comfortable_decel > 0.0 else default_decel
    braking_cap = math.sqrt(max(0.0, c.lead_speed * c.lead_speed + 2.0 * a * usable_gap))
    headway_cap = max(0.0, c.lead_speed + (usable_gap - time_gap * ego_speed) / max(0.1, time_gap))
    return min(braking_cap, headway_cap)


class ConstraintToSpeedCap(object):
    def __init__(self):
        rospy.init_node('constraint_to_speed_cap')

        self.default_decel = float(rospy.get_param('~comfortable_decel', 2.5))
        self.stop_epsilon = float(rospy.get_param('~stop_epsilon_m', 0.5))
        self.timeout = float(rospy.get_param('~constraint_timeout', 0.5))
        self.fail_closed = bool(rospy.get_param('~fail_closed_after_active', True))
        self.rate_hz = float(rospy.get_param('~rate', 20.0))

        # --- 구간 전환 선제 감속 ---
        # 구간이 바뀌는 순간 상한이 계단처럼 떨어지면 MPC 가 그때부터 급제동한다.
        #   실측 2026-09-15(12:44 백) M1(15.0) -> M2(9.5):
        #     t=29.6 14.71 m/s -> t=30.6 9.76 m/s, 1.0초에 -5.0 m/s^2.
        # 전방의 더 낮은 상한을 미리 보고 v = sqrt(v_next^2 + 2*a*d) 로 캡을
        # 미리 내려두면 같은 전환을 설계 감속률(기본 1.0)로 부드럽게 넘는다.
        self.prep_enabled = bool(
            rospy.get_param('~section_prep_enabled', True))
        self.prep_decel = max(0.05, float(
            rospy.get_param('~section_prep_decel_mps2', 1.0)))
        self.prep_min_lookahead = float(
            rospy.get_param('~section_prep_min_lookahead_m', 80.0))
        self.prep_lookahead_scale = float(
            rospy.get_param('~section_prep_lookahead_scale', 1.2))
        # 고주로(M6/M7)는 ACC 가 종방향 주인이라 기본은 건드리지 않는다.
        self.prep_in_exempt = bool(
            rospy.get_param('~section_prep_apply_in_exempt', False))
        self.waypoint_spacing = float(
            rospy.get_param('/section_router/waypoint_spacing_m', 0.1))
        self.section_csv = -1
        self.section_index = -1
        self.section_index_valid = False

        self.msg = None
        self.stamp = 0.0
        self.seen_active = False

        # 구간별 속도 상한. missions.yaml 의 sections[].speed_limit_mps 를 읽는다.
        # 정동적 장애물 미션은 자기 트리에서 도심 10.0 m/s(36km/h)로 검증됐다.
        # 0901 기본값 15.0 으로 그 구간을 달리면 회피 기동이 검증 범위를 벗어난다.
        # CSV mission_state 를 건드리지 않고 여기서 구간 상한만 얹는다.
        # sections 는 missions.yaml 에서 section_router 네임스페이스로 로드된다.
        # 같은 목록에서 speed_limit_exempt(고주로 M6/M7)도 같이 읽는다 - 이 구간은
        # SMPC(ACC/차선변경)가 /smpc/target_speed_mps 의 주인이어야 하므로
        # 별도 하드코딩 목록을 두지 않고 missions.yaml 을 그대로 따른다.
        self.section_speed = {}
        self.exempt_sections = set()
        # 선제 감속용 구간 배치표: csv -> [(idx_lo, idx_hi, section_id, exempt)]
        self.section_layout = {}
        sections_param = rospy.get_param('~sections_param',
                                         '/section_router/sections')
        for sec in rospy.get_param(sections_param, []):
            sid = sec.get('id')
            if not sid:
                continue
            v = sec.get('speed_limit_mps')
            if v is not None:
                self.section_speed[str(sid)] = float(v)
            if sec.get('speed_limit_exempt'):
                self.exempt_sections.add(str(sid))
            idx = sec.get('idx')
            csv = sec.get('csv')
            if isinstance(idx, (list, tuple)) and len(idx) == 2 and csv is not None:
                self.section_layout.setdefault(int(csv), []).append(
                    (int(idx[0]), int(idx[1]), str(sid),
                     bool(sec.get('speed_limit_exempt'))))
        for csv in self.section_layout:
            self.section_layout[csv].sort()

        # mpc_params.yaml 의 /speed_limit/sections 에 정의된 구간 속도 상한을 우선 적용
        mpc_section_limits = rospy.get_param('/speed_limit/sections', {})
        if isinstance(mpc_section_limits, dict):
            for sid, v in mpc_section_limits.items():
                if v is not None:
                    try:
                        self.section_speed[str(sid)] = float(v)
                    except (ValueError, TypeError):
                        pass

        self.section_id = ''
        self.section_t = 0.0
        self.in_exempt_section = False
        if self.section_speed:
            rospy.loginfo('[constraint_to_speed_cap] 구간 속도상한: %s',
                          ', '.join('%s=%.1f' % kv for kv in
                                    sorted(self.section_speed.items())))
        if self.exempt_sections:
            rospy.loginfo('[constraint_to_speed_cap] SMPC 전담 구간(속도상한 발행 안 함): %s',
                          ', '.join(sorted(self.exempt_sections)))

        # MPC 는 direct_topic 을 받으면 ref_v 와 speed_upperBound 에 min 으로
        # 접어 넣어 감속을 '스스로 계획'한다. 페달 단에서 덮어쓰면 MPC 는 캡을
        # 모른 채 가속을 내고 중재기는 제동을 걸어 서로 싸운다. 그래서 캡은
        # MPC 입력으로 준다.
        #
        # MPC 가 실제로 구독하는 이름(direct_topic, mpc_path_tracking.launch 의
        # external_acc_topic 과 반드시 같아야 함)은 smpc_lane_change 원본이 쓰는
        # /smpc/target_speed_mps 그 자체가 아니라 별도로 뗀 이름이다
        # (기본 /smpc/target_speed_mps_gated). SMPC 내부에서는
        # acc_speed_planner_node 가 /smpc/target_speed_mps 를 내고
        # smpc_decision_node 가 같은 이름을 자기 예측용으로 다시 구독하므로,
        # 그 이름 자체를 건드리면 SMPC 내부 배선이 깨진다 - 그래서 그 이름은
        # 원본 그대로 두고(raw_acc_topic 으로 구독만 함), 우리가 발행하는
        # "MPC 가 실제로 듣는" 이름을 따로 둬서 이 노드가 유일한 발행자가
        # 되게 한다.
        #
        # exempt 구간(M6/M7, missions.yaml speed_limit_exempt)에서는 우리
        # 미션 캡 대신 방금 받은 raw ACC 값을 그대로 중계한다 - "ACC는 고주로
        # 미션에서만 사용" 을 만족시키는 지점이 여기다: ACC 는 랩 전체에서
        # 계속 값을 내고 있어도, 우리가 그 값을 실제로 중계하는 건 exempt
        # 구간뿐이라 그 밖에서는 MPC 에 전혀 도달하지 않는다.
        self.pub = rospy.Publisher('/mission/speed_cap_mps', Float64, queue_size=1)
        self.direct_topic = rospy.get_param('~direct_speed_topic',
                                            '/smpc/target_speed_mps_gated')
        self.pub_direct = (rospy.Publisher(self.direct_topic, Float64, queue_size=1)
                           if self.direct_topic else None)

        self.raw_acc_topic = rospy.get_param('~raw_acc_topic', '/smpc/target_speed_mps')
        self.acc_timeout = float(rospy.get_param('~acc_timeout', 0.5))
        self.raw_acc_mps = None
        self.raw_acc_t = 0.0
        # ACC 중계 금지 구간 [[csv, idx_lo, idx_hi], ...] (idx_hi < 0 = csv 끝까지).
        #   exempt 구간이라도 여기 들면 ACC 원본을 중계하지 않고 미션 캡 경로로
        #   간다(캡이 없으면 발행 안 함 -> MPC 가 ACC 제한 없이 자기 목표속도).
        #   2026-09-29: 3.csv idx 5200 이후 톨게이트 구조물에 ACC 가 걸려 서행.
        self.acc_disable_ranges = []
        for r in rospy.get_param('~acc_disable_ranges', []):
            if isinstance(r, (list, tuple)) and len(r) == 3:
                self.acc_disable_ranges.append((int(r[0]), int(r[1]), int(r[2])))
        if self.acc_disable_ranges:
            rospy.loginfo('[constraint_to_speed_cap] ACC relay disabled in %s',
                          self.acc_disable_ranges)

        self.ego_speed = 0.0
        self.lead_standstill_dist = float(rospy.get_param('~lead_standstill_dist', 4.0))
        self.lead_time_gap = float(rospy.get_param('~lead_time_gap', 1.5))

        rospy.Subscriber('/odom', Odometry, self.odom_cb, queue_size=1)
        # 2026-10-07(21:07~21:09 백): 터널 후반 동적 장애물(보행자)이 벽 쪽 구조물 뒤에서 앞 5~7 m 로 갑자기
        #   나와 10 m/s 에서 서행 없이 급정지/근접. 인지로는 더 일찍 볼 수 없으므로 그 구간(3.csv idx)을
        #   미리 감속한다. [[lane, idx_lo, idx_hi, cap_mps], ...]. 터널 index 는 /final_waypoint(추측항법)로 본다.
        #   2026-10-08: 음영구간 장애물 위치는 랜덤이라 고정 구간 감속은 기본 비활성(빈 터널 서행 유발).
        self.idx_zone_caps = [tuple(z) for z in rospy.get_param('~idx_zone_caps', [])]
        self.idx_zone_decel = float(rospy.get_param('~idx_zone_decel', 2.0))
        self.wp_lane, self.wp_idx, self.wp_t = -1, -1, 0.0
        if _WpLane is not None and self.idx_zone_caps:
            rospy.Subscriber('/final_waypoint', _WpLane, self.wp_cb, queue_size=1)
        rospy.Subscriber('/mission/behavior_constraint', BehaviorConstraint,
                         self.cb, queue_size=1)
        rospy.Subscriber('/mission/active_section', SectionState,
                         self.sec_cb, queue_size=1)
        rospy.Subscriber(self.raw_acc_topic, Float64, self.acc_cb, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / max(1.0, self.rate_hz)), self.tick)
        rospy.loginfo('[constraint_to_speed_cap] ready. -> /mission/speed_cap_mps, '
                      'direct=%s, raw_acc=%s', self.direct_topic, self.raw_acc_topic)

    def odom_cb(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.ego_speed = math.hypot(vx, vy)

    def cb(self, msg):
        self.msg = msg
        self.stamp = rospy.Time.now().to_sec()
        if msg.active:
            self.seen_active = True

    def sec_cb(self, msg):
        self.section_id = msg.section_id
        self.section_t = rospy.Time.now().to_sec()
        self.in_exempt_section = msg.section_id in self.exempt_sections
        self.section_csv = int(msg.csv)
        self.section_index = int(msg.index)
        self.section_index_valid = bool(msg.index_valid)

    def section_prep_cap(self):
        """전방 구간 경계를 훑어 '지금 허용되는 최대 속도'를 낸다.

        현재 구간의 다음 하나만 보지 않고 lookahead 안의 모든 경계를 본다 -
        필요 감속거리가 현재 구간 길이보다 길 수 있고(M1 60 m 인데 M1->M2 는
        67 m 필요), 구간 사이에 공백도 있기 때문이다(M1 끝 2300 / M2 시작 2400).
        캡은 상한이므로 현재 속도보다 높으면 자연히 무시된다.
        """
        if not self.prep_enabled:
            return float('inf')
        if (rospy.Time.now().to_sec() - self.section_t) >= self.timeout:
            return float('inf')
        # GPS 음영(M8)에서 index 는 속도 적산 추정치다. 거리 기반 계산을
        # 그 값으로 하면 캡이 엉뚱하게 움직인다.
        if not self.section_index_valid or self.section_index < 0:
            return float('inf')
        if self.in_exempt_section and not self.prep_in_exempt:
            return float('inf')

        layout = self.section_layout.get(self.section_csv)
        if not layout:
            return float('inf')

        v = max(0.0, self.ego_speed)
        # 빠를수록 더 멀리 본다. 느리면 min_lookahead 로 충분하다.
        reach = max(self.prep_min_lookahead,
                    (v * v) / (2.0 * self.prep_decel) * self.prep_lookahead_scale)
        spacing = max(1e-3, self.waypoint_spacing)

        best = float('inf')
        for lo, hi, sid, exempt in layout:
            if exempt:
                continue
            cap = self.section_speed.get(sid)
            if cap is None:
                continue
            # 이미 그 구간 안이면 거리 0 - 구간 상한이 그대로 걸린다.
            d = max(0.0, (lo - self.section_index) * spacing)
            if self.section_index > hi:
                continue          # 지나온 구간
            if d > reach:
                break             # 정렬돼 있으므로 더 볼 것 없다
            if cap >= v:
                continue          # 지금보다 빠른 상한은 제약이 아니다
            best = min(best, math.sqrt(cap * cap + 2.0 * self.prep_decel * d))
        return best

    def wp_cb(self, msg):
        if msg.waypoints:
            self.wp_lane = int(msg.waypoints[0].lane_number)
            self.wp_idx = int(msg.waypoints[0].waypoint_index)
            self.wp_t = rospy.Time.now().to_sec()

    def zone_cap(self):
        """idx 구간 상한(진입 전 idx_zone_decel 로 미리 줄임). 없으면 inf."""
        if self.wp_idx < 0 or rospy.Time.now().to_sec() - self.wp_t > 1.0:
            return float('inf')
        best = float('inf')
        for lane, lo, hi, cap in self.idx_zone_caps:
            if int(lane) != self.wp_lane or self.wp_idx > hi:
                continue
            d = max(0.0, (lo - self.wp_idx) * self.waypoint_spacing)
            best = min(best, math.sqrt(float(cap) ** 2 + 2.0 * self.idx_zone_decel * d))
        return best

    def acc_cb(self, msg):
        self.raw_acc_mps = msg.data
        self.raw_acc_t = rospy.Time.now().to_sec()

    def acc_disabled_here(self):
        if not self.acc_disable_ranges or self.section_csv < 0 or self.section_index < 0:
            return False
        if (rospy.Time.now().to_sec() - self.section_t) >= self.timeout:
            return False
        for csv, lo, hi in self.acc_disable_ranges:
            if (self.section_csv == csv and self.section_index >= lo and
                    (hi < 0 or self.section_index <= hi)):
                return True
        return False

    def section_cap(self):
        """현재 구간의 속도 상한. 없으면 inf."""
        if (rospy.Time.now().to_sec() - self.section_t) >= self.timeout:
            return float('inf')
        return self.section_speed.get(self.section_id, float('inf'))

    def tick(self, _evt):
        now = rospy.Time.now().to_sec()
        fresh = self.msg is not None and (now - self.stamp) < self.timeout

        # 구간 상한과 선제 감속 캡 중 낮은 쪽. 미션 제약이 없는 경로에도
        # 적용해야 하므로(구간 사이 공백, 제약 비활성) 여기서 합친다.
        sec_cap = min(self.section_cap(), self.section_prep_cap())

        if not fresh:
            # 한 번이라도 활성이던 제약 스트림이 끊기면 fail-closed 로 정지.
            if self.fail_closed and self.seen_active:
                self.publish(0.0)
            else:
                self.publish(-1.0 if math.isinf(sec_cap) else sec_cap)
            return

        c = self.msg
        if not c.active:
            self.publish(-1.0 if math.isinf(sec_cap) else sec_cap)
            return

        cap = sec_cap
        if c.max_speed >= 0.0:
            cap = c.max_speed

        if c.stop_required:
            d = max(0.0, c.stop_distance)
            if d < self.stop_epsilon:
                cap = 0.0
            else:
                a = c.comfortable_decel if c.comfortable_decel > 0.0 else self.default_decel
                cap = min(cap, math.sqrt(2.0 * a * d))

        if c.lead_vehicle_active:
            lead_cap = lead_speed_cap(c, self.ego_speed, self.lead_standstill_dist,
                                      self.lead_time_gap, self.default_decel)
            cap = min(cap, lead_cap)

        self.publish(-1.0 if math.isinf(cap) else cap)

    def publish(self, cap):
        """cap < 0 이면 '제한 없음'. /mission/speed_cap_mps 에는 진단용으로
        항상 우리 미션 캡을 낸다(exempt 구간에서도 - mission_debug 등에서
        참고용으로 보임, MPC 입력에는 안 쓰임).

        MPC 가 실제로 듣는 direct_topic 에는 딱 하나의 값만 나간다:
          - exempt 구간(M6/M7)  : 방금 받은 raw ACC 값을 그대로 중계.
            ACC 가 랩 전체에서 계속 값을 내더라도, 우리가 중계하는 건 이
            구간뿐이라 MPC 는 그 구간에서만 ACC 의 영향을 받는다.
            ACC 신호가 끊기면(초기화 지연 등) 안전하게 우리 미션 캡으로
            폴백한다 - exempt 구간은 대개 무제한(cap<0)이라 폴백해도
            사실상 "그냥 두기"에 가깝다.
          - 그 외 구간            : 우리 미션 캡. 음수(무제한)는 안 보낸다 -
            MPC 가 0 으로 clamp 해 정지 명령이 되기 때문이다.
        """
        zcap = self.zone_cap()
        if not math.isinf(zcap):
            cap = zcap if cap < 0.0 else min(cap, zcap)
        self.pub.publish(Float64(cap))
        if self.pub_direct is None:
            return

        if self.in_exempt_section and not self.acc_disabled_here():
            acc_fresh = (self.raw_acc_mps is not None and
                        (rospy.Time.now().to_sec() - self.raw_acc_t) < self.acc_timeout)
            if acc_fresh:
                out = self.raw_acc_mps
                # section_prep_apply_in_exempt 를 켠 경우에만, ACC 중계값에도
                # 선제 감속 캡을 씌운다. 끄면(기본) ACC 가 그대로 나간다.
                if self.prep_in_exempt and cap >= 0.0:
                    out = min(out, cap)
                self.pub_direct.publish(Float64(out))
                return
            # ACC 신호가 없으면 우리 캡으로 폴백(아래 공통 경로로 이어짐).

        if cap >= 0.0:
            self.pub_direct.publish(Float64(cap))


if __name__ == '__main__':
    try:
        ConstraintToSpeedCap()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
