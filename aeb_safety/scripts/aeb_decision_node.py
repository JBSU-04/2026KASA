#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AEB 결정 노드 (단일 라이다 코리도 + RSS 안전거리).

merged_lidar(2라이다 동기화 5Hz)의 지연을 피하려고 **단일 원본 라이다(/lidar3d_1)** 를
그대로 받는다. 기하 파이프라인:

  1) 점군을 자차(차체) 프레임으로 오프셋 변환 (회전 없음 = 절대 yaw 불필요)
  2) 지면/자차 footprint 제거
  3) 자차 코리도(±lane_half, yaw-rate 로 곡선 구부림) 안의 점만 게이트
  4) 코리도 안 최근접(전방) 점 = 장애물 거리 d
  5) 위험도 판정 (아래 2-트랙)
  6) 단계 제동(FCW/PB1/PB2/FULL) AebCommand 발행

위험도 판정 — 2026-08-04 RSS 도입 (Autoware autonomous_emergency_braking 방식)
--------------------------------------------------------------------------
기존에는 객체 속도추정을 신뢰할 수 없어 **"장애물은 정지해 있다"** 고 가정하고
ttc = d/v_ego 만 썼다. 이는 선행차가 같은 방향으로 달리는 경우 과도하게 보수적이라
불필요한 급제동을 유발하고, 반대로 선행차가 급감속 중일 때는 그 감속을 고려하지
못한다. lidar_ttc_tracker 가 CTRV EKF 로 상대속도를 내주게 되면서(Phase 1) 이제
RSS(Responsibility-Sensitive Safety) 정지거리를 쓸 수 있다:

    ego_stop  = v_ego·t_response + v_ego²/(2·|a_ego_min|)
    obj_stop  = v_obj²/(2·|a_obj_min|)          # 선행차가 제동해 멈출 때까지 갈 거리
    rss_dist  = ego_stop - obj_stop + margin    # 접근 시. 이 거리보다 가까우면 위험
    (obj 가 정지/역주행이면 obj_stop=0 -> 기존 보수적 동작과 같아짐)

★안전 폴백: 코리도 안에 신뢰할 만한 트랙(hits>=min_hits, 신선도 OK)이 없으면 곧바로
"정지 가정"(v_obj=0) 으로 되돌아간다. 즉 RSS 는 **완화 방향으로만** 작동하지 않고,
트랙이 없을 때의 동작은 종전과 100% 동일하다. 트랙이 오히려 위험을 키우는 방향
(선행차가 접근해 오는 경우)에는 RSS 가 더 일찍 제동한다.

곡선 대응: 절대 pose 없이 자차 yaw-rate ω 로 코리도를 곡률 κ=ω/v 로 구부린다(CTRV).
cut-in: 인접차선 차가 코리도에 들어오면 최근접점 d 가 급감 -> 자동으로 제동.

입력 : ~lidar_topic(PointCloud2, 기본 /lidar3d_1), /odom(속도+yaw-rate),
        ~tracked_topic(TrackedObjectArray, 기본 /tracked_objects) — 선택적, 없으면 폴백
출력 : ~out_topic(AebCommand, 기본 /aeb/command), /aeb/markers
"""
import math
import numpy as np
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from aeb_safety.msg import AebCommand

try:
    from lidar_ttc_tracker.msg import TrackedObjectArray
    _HAVE_TRACKS = True
except ImportError:
    # lidar_ttc_tracker 가 빌드되지 않은 환경에서도 이 노드는 기존 동작으로 살아있어야
    # 한다(AEB 는 안전 최후단이므로 임포트 실패로 죽으면 안 됨).
    TrackedObjectArray = None
    _HAVE_TRACKS = False


class AebDecision(object):
    def __init__(self):
        rospy.init_node('aeb_decision', anonymous=False)

        # 입력 라이다 (단일 원본). lidar_x/y = 그 라이다의 차체(후축) 기준 장착 오프셋.
        self.lidar_topic = rospy.get_param('~lidar_topic', '/lidar3d_1')
        self.lidar_x = rospy.get_param('~lidar_x', 1.1)   # 전방[m] — MORAI SensorInfo pos.x 와 일치
        self.lidar_y = rospy.get_param('~lidar_y', 0.0)

        # 점 필터
        self.min_range = rospy.get_param('~min_range', 1.0)
        self.max_range = rospy.get_param('~max_range', 40.0)
        self.ground_pct = rospy.get_param('~ground_percentile', 20.0)
        self.ground_clearance = rospy.get_param('~ground_clearance', 0.3)
        self.z_max = rospy.get_param('~z_max', 2.5)
        # 자차 footprint 제거 (라이다 센서 프레임 박스)
        self.ego_x_min = rospy.get_param('~ego_x_min', -2.09)
        self.ego_x_max = rospy.get_param('~ego_x_max', 2.95)
        self.ego_y_half = rospy.get_param('~ego_y_half', 1.3)

        # 코리도
        self.ref_v = rospy.get_param('~ref_v', 12.0)
        self.lane_half = rospy.get_param('~lane_half_width', 1.75)
        self.corridor_margin = rospy.get_param('~corridor_margin', 0.3)
        self.bend_by_yawrate = rospy.get_param('~bend_by_yawrate', True)
        # 노이즈 단일점 방지: 최근접 근처 window 안에 최소 이만큼 점이 있어야 장애물로 확정
        self.min_points = rospy.get_param('~min_points', 3)
        self.confirm_window = rospy.get_param('~confirm_window', 0.5)

        # 접촉/제동 단계
        self.ego_front_overhang = rospy.get_param('~ego_front_overhang', 2.0)
        self.ttc_fcw = rospy.get_param('~ttc_fcw', 2.6)
        self.ttc_pb1 = rospy.get_param('~ttc_pb1', 2.0)
        self.ttc_pb2 = rospy.get_param('~ttc_pb2', 1.6)
        self.ttc_full = rospy.get_param('~ttc_full', 1.0)
        self.ttc_hyst = rospy.get_param('~ttc_hysteresis', 0.3)
        self.a_full = rospy.get_param('~decel_full', 6.0)   # required decel 이 이 이상이면 긴급
        self.brake_pb1 = rospy.get_param('~brake_pb1', 0.35)
        self.brake_pb2 = rospy.get_param('~brake_pb2', 0.7)
        self.brake_full = rospy.get_param('~brake_full', 1.0)
        self.comfort_decel = rospy.get_param('~comfort_decel', 2.0)
        self.stop_margin = rospy.get_param('~stop_margin', 4.0)

        # ---- RSS (Autoware autonomous_emergency_braking 대응) ----
        # use_rss=False 로 두면 2026-08-04 이전의 '장애물 정지 가정' 동작으로 복귀.
        self.use_rss = rospy.get_param('~use_rss', True)
        self.rss_t_response = rospy.get_param('~rss_t_response', 1.0)   # 반응지연[s]
        self.rss_a_ego_min = abs(rospy.get_param('~rss_a_ego_min', 3.0))  # 자차 최대감속
        self.rss_a_obj_min = abs(rospy.get_param('~rss_a_obj_min', 1.0))  # 상대차 가정 감속
        self.rss_margin = rospy.get_param('~rss_margin', 1.0)           # 정지 후 여유[m]
        # 트랙 신뢰 조건 — 이보다 낮으면 '정지 가정' 폴백
        self.track_min_hits = rospy.get_param('~track_min_hits', 3)
        self.track_timeout = rospy.get_param('~track_timeout', 0.5)
        # 트랙-최근접점 연결: 코리도 최근접거리 d 와 이만큼 이내인 트랙만 그 점의 주인으로 인정
        self.track_match_dist = rospy.get_param('~track_match_dist', 4.0)

        self.have_ego = False
        self.ego_v = 0.0
        self.ego_w = 0.0
        self._prev_stage = 0
        self._tracks = []
        self._tracks_t = 0.0

        out_topic = rospy.get_param('~out_topic', '/aeb/command')
        self.cmd_pub = rospy.Publisher(out_topic, AebCommand, queue_size=1)
        self.mk_pub = rospy.Publisher('/aeb/markers', MarkerArray, queue_size=1)

        rospy.Subscriber('/odom', Odometry, self.odom_cb, queue_size=1)
        # 2026-10-07: buff_size 가 메시지(터널 /nonground 수 MB)보다 작으면 queue_size=1 이어도 TCP 버퍼에
        #   옛 스캔이 쌓여 2~4 s 묵은 점군을 처리했다(16:18/16:19 터널 백, 근거리 인지 지연 p50 2.5 s).
        rospy.Subscriber(self.lidar_topic, PointCloud2, self.cloud_cb, queue_size=1, buff_size=2 ** 26)

        self.tracked_topic = rospy.get_param('~tracked_topic', '/tracked_objects')
        self.rss_enabled = bool(self.use_rss and _HAVE_TRACKS)
        if self.rss_enabled:
            rospy.Subscriber(self.tracked_topic, TrackedObjectArray,
                             self.tracks_cb, queue_size=1)
        elif self.use_rss and not _HAVE_TRACKS:
            rospy.logwarn('[aeb_decision] lidar_ttc_tracker msgs 없음 -> RSS 비활성, '
                          '정지가정 폴백으로 동작')

        rospy.loginfo('[aeb_decision] corridor AEB. lidar=%s lidar_x=%.2f corridor=±%.2fm '
                      'ttc[fcw/pb1/pb2/full]=%.1f/%.1f/%.1f/%.1f rss=%s',
                      self.lidar_topic, self.lidar_x, self.lane_half,
                      self.ttc_fcw, self.ttc_pb1, self.ttc_pb2, self.ttc_full,
                      'on' if self.rss_enabled else 'off')

    def odom_cb(self, msg):
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        self.ego_v = abs(v) if math.isfinite(v) else 0.0
        self.ego_w = w if math.isfinite(w) else 0.0
        self.have_ego = True
        # ego pose (map) — 트랙 좌표가 map 이므로 코리도 매칭에 필요
        self._ex = msg.pose.pose.position.x
        self._ey = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._eyaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def tracks_cb(self, msg):
        self._tracks = list(msg.objects)
        self._tracks_t = rospy.Time.now().to_sec()

    def _lead_closing_speed(self, d):
        """코리도 최근접점 d[m] 의 주인 트랙을 찾아 그 '전방 성분 절대속도'를 돌려준다.

        반환: (v_obj, matched)  — matched=False 면 신뢰할 트랙 없음 -> 정지가정 폴백.
        v_obj 는 자차 진행방향 성분의 **절대속도**(m/s). 선행차가 같은 방향으로 달리면
        양수, 마주 오면 음수. RSS 에서 obj_stop 항 계산에 쓴다.
        """
        if not self.rss_enabled or not self._tracks:
            return 0.0, False
        if (rospy.Time.now().to_sec() - self._tracks_t) > self.track_timeout:
            return 0.0, False
        if not hasattr(self, '_ex'):
            return 0.0, False

        c, s = math.cos(self._eyaw), math.sin(self._eyaw)
        best = None
        for o in self._tracks:
            if int(getattr(o, 'hits', 0)) < self.track_min_hits:
                continue
            # 자차 프레임 전방/횡 성분
            dx, dy = float(o.x) - self._ex, float(o.y) - self._ey
            fwd = dx * c + dy * s
            lat = -dx * s + dy * c
            if fwd <= 0.0:
                continue                      # 후방 객체는 전방충돌과 무관
            if abs(lat) > (self.lane_half + self.corridor_margin + 1.0):
                continue                      # 코리도 밖
            # 최근접점과 같은 물체인지: 전방거리 차이로 판정
            # (d 는 차체 기준, fwd 도 차체 기준이므로 직접 비교 가능)
            if abs(fwd - d) > self.track_match_dist:
                continue
            if best is None or abs(fwd - d) < best[0]:
                # 절대속도의 자차 진행방향 성분
                v_fwd = float(o.vx) * c + float(o.vy) * s
                best = (abs(fwd - d), v_fwd)
        if best is None:
            return 0.0, False
        return best[1], True

    def _rss_risk(self, eff, v_obj):
        """RSS 안전거리 대비 위험도. (rss_dist, is_unsafe) 반환.

        ego_stop = v_ego·t_response + v_ego²/(2·a_ego_min)
        obj_stop = v_obj²/(2·a_obj_min)      (v_obj>0 인 경우만; 정지/역주행이면 0)
        rss_dist = ego_stop - obj_stop + margin
        """
        v = max(self.ego_v, 0.0)
        ego_stop = v * self.rss_t_response + (v * v) / (2.0 * max(self.rss_a_ego_min, 0.1))
        obj_stop = 0.0
        if v_obj > 0.0:
            obj_stop = (v_obj * v_obj) / (2.0 * max(self.rss_a_obj_min, 0.1))
        rss_dist = ego_stop - obj_stop + self.rss_margin
        # 음수(선행차가 더 빨라 거리가 벌어지는 상황)는 위험 없음으로 클램프
        rss_dist = max(rss_dist, 0.0)
        return rss_dist, (eff <= rss_dist)

    def _corridor_nearest(self, msg):
        """코리도 안 최근접(전방) 장애물 거리 d 반환. 없으면 None. (전부 자차 프레임)"""
        pts = np.array(list(point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)), dtype=np.float32)
        if pts.shape[0] == 0:
            return None, 0
        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
        # 자차 footprint 제거 (센서 프레임)
        ego = (x > self.ego_x_min) & (x < self.ego_x_max) & (np.abs(y) < self.ego_y_half)
        x, y, z = x[~ego], y[~ego], z[~ego]
        if x.shape[0] == 0:
            return None, 0
        r = np.hypot(x, y)
        keep = (r > self.min_range) & (r < self.max_range)
        x, y, z = x[keep], y[keep], z[keep]
        if x.shape[0] == 0:
            return None, 0
        # 지면 제거 (적응형 z 밴드)
        gz = np.percentile(z, self.ground_pct)
        band = (z > gz + self.ground_clearance) & (z < gz + self.z_max)
        x, y = x[band], y[band]
        if x.shape[0] == 0:
            return None, 0
        # 차체 프레임 (오프셋만, 회전 없음)
        fx = x + self.lidar_x
        fy = y + self.lidar_y
        # 전방만
        fwd = fx > 0.0
        fx, fy = fx[fwd], fy[fwd]
        if fx.shape[0] == 0:
            return None, 0
        # 코리도(곡선 구부림): 기대 횡오프셋 e(s) = (1-cos(κs))/κ, κ=ω/v
        half = self.lane_half + self.corridor_margin
        if self.bend_by_yawrate and self.ego_v > 1.0 and abs(self.ego_w) > 1e-3:
            kappa = self.ego_w / self.ego_v
            ks = np.clip(kappa * fx, -1.4, 1.4)   # |κs|<~80deg 범위로 근사 유효
            e = (1.0 - np.cos(ks)) / kappa
        else:
            e = 0.0
        incorr = np.abs(fy - e) < half
        cf = fx[incorr]
        if cf.shape[0] == 0:
            return None, 0
        d = float(cf.min())
        # 노이즈 방지: 최근접 근처 window 안에 min_points 이상이어야 확정
        cnt = int(np.count_nonzero(cf < d + self.confirm_window))
        if cnt < self.min_points:
            return None, cnt
        return d, cnt

    def cloud_cb(self, msg):
        if not self.have_ego:
            return
        d, cnt = self._corridor_nearest(msg)

        cmd = AebCommand()
        cmd.header = Header(stamp=rospy.Time.now(), frame_id='base_link')
        cmd.source = 'aeb'
        cmd.stage = AebCommand.STAGE_NONE
        cmd.emergency = False
        cmd.target_speed = -1.0
        cmd.brake = 0.0
        cmd.decel = 0.0
        cmd.target_id = 0
        cmd.ttc = -1.0
        cmd.distance = -1.0 if d is None else d
        cmd.reason = 'clear'

        if d is not None:
            eff = max(d - self.ego_front_overhang, 0.0)
            v = max(self.ego_v, 1e-3)

            # 선행 객체 속도 (트랙 있으면 실측, 없으면 0 = 정지가정 폴백)
            v_obj, matched = self._lead_closing_speed(d)
            # 접근속도: 자차속도 - 선행차 전방속도. 선행차가 더 빠르면 접근하지 않음.
            closing = v - v_obj if matched else v
            closing = max(closing, 1e-3)

            ttc = eff / closing                 # matched 아니면 기존 식과 동일
            a_req = closing * closing / (2.0 * max(eff, 0.1))
            cmd.ttc = ttc

            # RSS 위반 여부 (트랙 매칭 여부와 무관하게 평가 — 미매칭이면 v_obj=0 이라
            # 자연히 기존의 보수적 판정이 된다)
            rss_dist, rss_unsafe = self._rss_risk(eff, v_obj if matched else 0.0)
            cmd.distance = d

            hy = self.ttc_hyst if self._prev_stage >= AebCommand.STAGE_PB1 else 0.0

            # 진단용 접미사: 트랙 매칭 여부와 RSS 여유를 항상 남긴다
            src = ('trk v_obj=%.1f' % v_obj) if matched else 'notrk'
            tail = ' [%s rss=%.1f]' % (src, rss_dist)

            if ttc <= self.ttc_full or a_req >= self.a_full or rss_unsafe:
                cmd.stage = AebCommand.STAGE_FULL
                cmd.emergency = True
                cmd.brake = self.brake_full
                cmd.decel = self.a_full
                trig = 'rss' if (rss_unsafe and ttc > self.ttc_full and
                                 a_req < self.a_full) else 'ttc'
                cmd.reason = 'FULL(%s) d=%.1f ttc=%.2f areq=%.1f pts=%d%s' % (
                    trig, d, ttc, a_req, cnt, tail)
            elif ttc <= self.ttc_pb2 + hy:
                cmd.stage = AebCommand.STAGE_PB2
                cmd.brake = self.brake_pb2
                cmd.decel = 4.0
                cmd.reason = 'PB2 d=%.1f ttc=%.2f%s' % (d, ttc, tail)
            elif ttc <= self.ttc_pb1 + hy:
                cmd.stage = AebCommand.STAGE_PB1
                cmd.brake = self.brake_pb1
                cmd.decel = 2.5
                cmd.reason = 'PB1 d=%.1f ttc=%.2f%s' % (d, ttc, tail)
            elif ttc <= self.ttc_fcw + hy:
                cmd.stage = AebCommand.STAGE_FCW
                v_prof = math.sqrt(2.0 * self.comfort_decel * max(d - self.stop_margin, 0.0))
                cmd.target_speed = min(self.ref_v, v_prof) * 3.6
                cmd.decel = self.comfort_decel
                cmd.reason = 'FCW d=%.1f ttc=%.2f v=%.1f%s' % (
                    d, ttc, cmd.target_speed, tail)

        self._prev_stage = cmd.stage
        self.cmd_pub.publish(cmd)
        self._publish_markers(cmd)

    def _publish_markers(self, cmd):
        ma = MarkerArray()
        m = Marker()
        m.header = cmd.header
        m.ns = 'aeb_status'
        m.id = 0
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = 0.0
        m.pose.position.y = 0.0
        m.pose.position.z = 3.0
        m.scale.z = 1.2
        names = ['NONE', 'FCW', 'PB1', 'PB2', 'FULL', 'HOLD']
        m.text = 'AEB: %s\n%s' % (names[cmd.stage] if cmd.stage < len(names) else '?', cmd.reason)
        if cmd.emergency:
            m.color.r, m.color.g, m.color.b = 1.0, 0.1, 0.1
        elif cmd.stage >= AebCommand.STAGE_PB1:
            m.color.r, m.color.g, m.color.b = 1.0, 0.55, 0.0
        elif cmd.stage == AebCommand.STAGE_FCW:
            m.color.r, m.color.g, m.color.b = 1.0, 0.9, 0.2
        else:
            m.color.r, m.color.g, m.color.b = 0.4, 0.9, 0.4
        m.color.a = 1.0
        m.lifetime = rospy.Duration(0.4)
        ma.markers.append(m)

        if cmd.distance >= 0.0:
            s = Marker()
            s.header = cmd.header
            s.ns = 'aeb_nearest'
            s.id = 0
            s.type = Marker.SPHERE
            s.action = Marker.ADD
            s.pose.position.x = cmd.distance
            s.pose.position.y = 0.0
            s.pose.position.z = 0.5
            s.pose.orientation.w = 1.0
            s.scale.x = s.scale.y = s.scale.z = 1.0
            s.color.r, s.color.g, s.color.b, s.color.a = 1.0, 0.2, 0.2, 0.9
            s.lifetime = rospy.Duration(0.4)
            ma.markers.append(s)
        self.mk_pub.publish(ma)


if __name__ == '__main__':
    try:
        AebDecision()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
