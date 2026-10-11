#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
K-City 2025 mgeo 기반 차량 측위 노드.

입력 (MORAI 시뮬레이터 native):
  /gps  (morai_msgs/GPSMessage)   : lat/lon + east/northOffset(맵 원점)
  /imu  (sensor_msgs/Imu)         : orientation (heading)
대안 입력:
  /odom (nav_msgs/Odometry)       : ~use_odom:=true 시 gps 대신 사용

처리:
  GPS(lat,lon) -> UTM52N -> 맵 로컬좌표(= UTM - offset) 로 변환.
  GPS 위치 + IMU 가속도/자이로를 KF([x,y,vx,vy])로 융합:
    - IMU 콜백 주기로 등가속 예측, GPS 콜백에서 위치 보정 (점프 게이트 포함)
    - yaw는 MORAI IMU orientation 사용, twist에 전진속도/yaw rate 채움
  추출된 mgeo 링크망에 정합하여 현재 주행 중인 링크/노드와 가로오프셋을 산출.

출력:
  /localization/current_pose  (geometry_msgs/PoseStamped)  현재 차량 pose (frame=map)
  /localization/current_link  (std_msgs/String)            현재 링크 idx
  /localization/state         (std_msgs/Float32MultiArray) [x,y,yaw,lateral,s,max_speed,match_dist]
  /localization/ego_marker    (visualization_msgs/Marker)  RViz 차량 화살표
  /localization/map_markers   (visualization_msgs/MarkerArray) 차선 도색/신호등/정지선/횡단보도
                              (최초 1회, latched. ~show_link_graph:=true 면 링크망 그래프도)
  /odom                       (nav_msgs/Odometry)          하위호환용 (gps 입력일 때만)
  tf: map -> base_link
"""
import os
import sys
import math
import json

import rospy
import tf
from std_msgs.msg import String, Float32MultiArray, Bool
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from mgeo_loader import MGeoMap

try:
    from morai_msgs.msg import GPSMessage
    HAS_MORAI = True
except ImportError:
    HAS_MORAI = False

try:
    from morai_msgs.msg import GetTrafficLightStatus
    HAS_MORAI_TL = True
except ImportError:
    HAS_MORAI_TL = False


try:
    from pyproj import Proj
except ImportError:
    Proj = None


def yaw_to_quat(yaw):
    return tf.transformations.quaternion_from_euler(0, 0, yaw)


def quat_to_yaw(q):
    return tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


class MapLocalizer(object):
    def __init__(self):
        rospy.init_node("kcity_map_localizer", anonymous=False)

        data_dir = rospy.get_param("~data_dir", "")
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.use_odom = rospy.get_param("~use_odom", False)
        self.utm_zone = rospy.get_param("~utm_zone", 52)
        self.publish_tf = rospy.get_param("~publish_tf", True)
        self.publish_odom = rospy.get_param("~publish_odom", True)
        # IMU yaw 규약 스위치. 실측(2026-07-19, 단일라이다 리그): IMU 원시 yaw 가
        # Ego heading 과 일치(ENU, 오프셋 -0.03°) → 기본 끔. NED/나침반 세팅의
        # 리그(2026-07-12 실측)에서는 ~imu_ned:=true 로 복원.
        # 켠 채 ENU IMU 를 받으면 yaw=90°-실제(거울상) → v_fwd=v·cos(2ψ-90°) 로
        # 속도 저평가·heading 90° 부근 0 붕괴 발생하니 주의.
        self.imu_ned = rospy.get_param("~imu_ned", False)
        self.max_sensor_skew = rospy.get_param("~max_sensor_skew", 0.25)
        self.odom_timeout = rospy.get_param("~odom_timeout", 0.3)
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        rate_hz = rospy.get_param("~rate", 20.0)
        # RViz 지도 시각화: 실제 차선 도색(lane_boundary, 색/실선/점선) 렌더링.
        # 링크망 그래프(파란선+노드)는 디버그용이라 기본 숨김 (~show_link_graph:=true 로 복원)
        self.show_lane_markings = rospy.get_param("~show_lane_markings", True)
        self.show_link_graph = rospy.get_param("~show_link_graph", False)

        rospy.loginfo("[localizer] loading mgeo ...")
        self.map = MGeoMap(data_dir if data_dir else None)
        rospy.loginfo("[localizer] %s", self.map.summary())

        if Proj is None:
            rospy.logfatal("pyproj 미설치: apt install python3-pyproj")
            raise SystemExit(1)
        self.proj_utm = Proj(proj="utm", zone=self.utm_zone, ellps="WGS84",
                             preserve_units=False)

        # ------------------------------------------------------- EKF 상태
        # 상태벡터 X = [x, y, vx, vy] (map frame)
        # 예측: IMU 가속도(바디->맵 회전)로 등가속 모델, 보정: GPS 위치
        self.X = [0.0, 0.0, 0.0, 0.0]
        # 공분산 (대각만 유지해도 충분하지만 4x4 풀로 관리)
        self.P = [[100.0 if i == j else 0.0 for j in range(4)] for i in range(4)]
        # 프로세스 노이즈 (가속도 백색잡음 표준편차 [m/s^2])
        self.sigma_acc = rospy.get_param("~sigma_acc", 0.6)
        # GPS 측정 노이즈 표준편차 [m] (MORAI noise 옵션 켰을 때 대비 여유)
        self.sigma_gps = rospy.get_param("~sigma_gps", 0.35)
        # GPS 점프 게이트 [m]: 예측 위치 대비 이 이상 튀면 1스텝 보정 무시
        self.gps_gate = rospy.get_param("~gps_gate", 5.0)

        self.yaw = 0.0
        self.wz = 0.0            # IMU yaw rate
        self.ax_body = 0.0       # IMU 종가속
        self.ay_body = 0.0
        self.last_imu_t = None
        self.last_gps_t = None
        self.last_odom_t = None
        self.synced_imu_t = None
        self.gate_skip_cnt = 0
        self.have_pos = False
        self.have_yaw = False
        self.east_off = self.map.origin[0]
        self.north_off = self.map.origin[1]

        # publisher
        self.pub_pose = rospy.Publisher("/localization/current_pose",
                                        PoseStamped, queue_size=1)
        self.pub_link = rospy.Publisher("/localization/current_link",
                                        String, queue_size=1)
        self.pub_state = rospy.Publisher("/localization/state",
                                         Float32MultiArray, queue_size=1)
        self.pub_ego = rospy.Publisher("/localization/ego_marker",
                                       Marker, queue_size=1)
        self.pub_map = rospy.Publisher("/localization/map_markers",
                                       MarkerArray, queue_size=1, latch=True)
        # 전방 신호등 정지점: [dist, stop_x, stop_y, tl_x, tl_y, tl_z] (없으면 dist=-1)
        self.stop_lookahead = rospy.get_param("~stop_lookahead", 80.0)
        self.pub_stop = rospy.Publisher("/localization/stop_ahead",
                                        Float32MultiArray, queue_size=1)
        self.pub_stop_tl = rospy.Publisher("/localization/stop_tl",
                                           String, queue_size=1)
        self.pub_stop_marker = rospy.Publisher("/localization/stop_marker",
                                               Marker, queue_size=1)
        # 신호 상태 결합 정지 판단 (True = 전방 신호 정지 필요)
        # 신호색은 YOLO 추론 노드(traffic_light_detector)의 /traffic_light_color 사용
        self.pub_stop_req = rospy.Publisher("/localization/stop_required",
                                            Bool, queue_size=1)
        self.tl_color = "none"        # red|yellow|green|none
        self.tl_color_t = 0.0
        self.morai_tl_idx = ""
        self.morai_tl_status = -1
        self.morai_tl_t = 0.0
        self.tl_status_timeout = rospy.get_param("~tl_status_timeout", 1.5)
        # 신호색 미상일 때 이 거리 안쪽이면 보수적으로 정지 [m]
        self.unknown_stop_dist = rospy.get_param("~unknown_stop_dist", 40.0)
        self.stop_on_unknown = rospy.get_param("~stop_on_unknown", False)
        # Temporary test mode: keep traffic-light perception visible, but never
        # turn a mapped red/yellow signal into an MPC stop request.
        self.ignore_traffic_lights = rospy.get_param(
            "~ignore_traffic_lights", False)
        # 정지선 근처에서 정지에 들어가면 지도 매칭이 잠깐 끊겨도 green 전까지 유지.
        self.stop_hold_dist = rospy.get_param("~stop_hold_dist", 12.0)
        self.approach_stop_dist = rospy.get_param("~approach_stop_dist", 50.0)
        # 터널 등 신호등 검출이 필요 없는 링크. 쉼표로 여러 링크 지정 가능.
        # 이 링크에서는 지도상의 정지선/YOLO 적색 오인식과 관계없이 통과한다.
        ignored_tl_links = rospy.get_param(
            "~traffic_light_ignore_links", "A2256W000448")
        if isinstance(ignored_tl_links, str):
            self.traffic_light_ignore_links = {
                x.strip() for x in ignored_tl_links.split(",") if x.strip()
            }
        else:
            self.traffic_light_ignore_links = {
                str(x).strip() for x in ignored_tl_links if str(x).strip()
            }
        # 현재 정지 위치에서만 차량 제어용 정지 마진을 추가한다.
        # 지도상의 정지선 좌표는 유지하고, MPC에 전달하는 거리만 늘린다.
        extra_margin_links = rospy.get_param(
            "~traffic_light_extra_margin_links", "A2256W000308")
        if isinstance(extra_margin_links, str):
            self.traffic_light_extra_margin_links = {
                x.strip() for x in extra_margin_links.split(",") if x.strip()
            }
        else:
            self.traffic_light_extra_margin_links = {
                str(x).strip() for x in extra_margin_links if str(x).strip()
            }
        self.traffic_light_extra_margin = rospy.get_param(
            "~traffic_light_extra_margin", 2.0)
        self.stop_hold = False
        # 녹색으로 출발한 직후 이 시간(초) 동안은 새 정지 요구를 내지 않는다.
        # 코스상 신호등이 연달아 붙어 있지 않으므로, 방금 통과한 교차로의
        # 측면/건너편 정지선을 자기 것으로 잡는 오검출만 걸러진다.
        #   실측 2026-09-14(15:39 백): t=157.9 녹색 출발 -> t=164.1 에
        #   교차로 측면 신호(C1256W000046)의 정지선을 잡아 영구 정지.
        #   그 정지선은 실제로 40 m 옆(전방 투영 -9.95 m)인데
        #   control_dist = max(xy_forward, 0.0) 이 0 으로 눌러 즉시 정지가 됐다.
        #   같은 백의 정상 정지는 전부 dist 38~54 m + 확정 red 였고, 오검출은
        #   전부 dist 0.00 + flag -1(미상) 이었다.
        self.green_release_grace_sec = float(
            rospy.get_param("~green_release_grace_sec", 5.0))
        self.green_release_t = 0.0
        # 녹색 출발 유예를 적용할 최대 거리(m). 방금 통과한 정지선은 거리가
        # 0 에 가깝다. 이보다 멀리 있는 정지선은 진짜 다음 신호이므로
        # 유예로 막지 않는다.
        # 정지선이 진행 경로에서 이 거리(m)보다 옆으로 벗어나면 우리 정지선이
        # 아니라고 본다. 정상 정지선 횡오차는 1~2 m, 교차로 측면 신호는
        # 수십 m 라 여유가 크다.
        self.green_release_stop_dist_m = float(
            rospy.get_param("~green_release_stop_dist_m", 8.0))
        self.stop_lateral_max_m = float(
            rospy.get_param("~stop_lateral_max_m", 5.0))
        # 정지선이 이 거리(m)보다 뒤에 있으면 이미 지나온 것으로 보고 버린다.
        # '막 넘은' 상황(-0.5 m 안팎)은 그대로 0 으로 유지해야 하므로
        # 그보다 넉넉히 뒤일 때만 적용한다.
        self.stop_behind_max_m = float(
            rospy.get_param("~stop_behind_max_m", 3.0))
        # 앞범퍼가 정지선을 넘었는지 판정할 때 쓰는 오프셋(차량 원점 기준).
        # 규정 2-6-1 Ioniq5: 축거 3.000 + 전방 오버행 0.845.
        # traffic_light_mission 의 front_axle_offset_m 과 같은 값이어야 한다.
        self.front_bumper_offset_m = float(
            rospy.get_param("~front_bumper_offset_m", 3.845))
        # 범퍼가 정지선을 넘으면 정지를 해제하고 그대로 통과한다.
        self.pass_when_bumper_crossed = bool(
            rospy.get_param("~pass_when_bumper_crossed", True))
        self.last_stop_data = None
        self.route_links = []
        self.route_pos = -1
        rospy.Subscriber("/traffic_light_color", String, self.tl_cb)
        rospy.Subscriber("/global_lane_seq", String, self.route_cb,
                         queue_size=1)
        if HAS_MORAI_TL and rospy.get_param("~use_morai_tl_status", False):
            rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus,
                             self.morai_tl_cb, queue_size=1)
            rospy.loginfo("[localizer] MORAI traffic-light status enabled")
        if self.publish_odom:
            self.pub_odom = rospy.Publisher("/odom", Odometry, queue_size=1)
        self.tf_br = tf.TransformBroadcaster() if self.publish_tf else None

        # subscriber
        if self.use_odom:
            rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=1)
            rospy.loginfo("[localizer] input: %s", self.odom_topic)
        else:
            if not HAS_MORAI:
                rospy.logfatal("morai_msgs 미설치인데 use_odom=false. "
                               "morai_msgs 빌드 또는 ~use_odom:=true 필요")
                raise SystemExit(1)
            rospy.Subscriber("/gps", GPSMessage, self.gps_cb, queue_size=1)
            rospy.Subscriber("/imu", Imu, self.imu_cb, queue_size=1)
            rospy.loginfo("[localizer] input: /gps + /imu")

        self.publish_map_markers()

        self.timer = rospy.Timer(rospy.Duration(1.0 / max(rate_hz, 1.0)),
                                 self.update)
        rospy.loginfo("[localizer] ready.")

    # ----------------------------------------------------------- EKF core
    def _predict(self, dt):
        """IMU 가속도 기반 등가속 예측. dt 비정상이면 skip."""
        if dt <= 0.0 or dt > 0.5:
            return
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        # 바디 가속 -> 맵 프레임
        ax = c * self.ax_body - s * self.ay_body
        ay = s * self.ax_body + c * self.ay_body

        x, y, vx, vy = self.X
        self.X = [x + vx * dt + 0.5 * ax * dt * dt,
                  y + vy * dt + 0.5 * ay * dt * dt,
                  vx + ax * dt,
                  vy + ay * dt]

        # P = F P F^T + Q  (F = [[I, dt*I],[0, I]])
        P = self.P
        for i in range(2):
            j = i + 2
            P[i][i] += dt * (P[j][i] + P[i][j]) + dt * dt * P[j][j]
            P[i][j] += dt * P[j][j]
            P[j][i] = P[i][j]
        q = self.sigma_acc * self.sigma_acc
        qd2 = q * dt * dt
        P[0][0] += 0.25 * qd2 * dt * dt
        P[1][1] += 0.25 * qd2 * dt * dt
        P[2][2] += qd2
        P[3][3] += qd2
        P[0][2] += 0.5 * qd2 * dt
        P[2][0] = P[0][2]
        P[1][3] += 0.5 * qd2 * dt
        P[3][1] = P[1][3]

    def _correct_pos(self, zx, zy):
        """GPS 위치 관측 보정 (H = [I2, 0])."""
        if not self.have_pos:
            self.X[0], self.X[1] = zx, zy
            self.have_pos = True
            return
        # 점프 게이트
        dx, dy = zx - self.X[0], zy - self.X[1]
        if math.hypot(dx, dy) > self.gps_gate:
            self.gate_skip_cnt += 1
            if self.gate_skip_cnt <= 3:
                rospy.logwarn_throttle(1.0, "[localizer] GPS jump gated (%.1fm)",
                                       math.hypot(dx, dy))
                return
            # 연속 게이트 -> 진짜 이동으로 보고 리셋
            self.X[0], self.X[1] = zx, zy
            self.X[2] = self.X[3] = 0.0
            self.gate_skip_cnt = 0
            return
        self.gate_skip_cnt = 0

        r = self.sigma_gps * self.sigma_gps
        P = self.P
        for axis, innov in ((0, dx), (1, dy)):
            v = axis + 2
            s_inn = P[axis][axis] + r
            k_p = P[axis][axis] / s_inn      # 위치 게인
            k_v = P[v][axis] / s_inn         # 속도 게인 (교차공분산)
            self.X[axis] += k_p * innov
            self.X[v] += k_v * innov
            p_pp, p_pv, p_vv = P[axis][axis], P[axis][v], P[v][v]
            P[axis][axis] = (1.0 - k_p) * p_pp
            P[axis][v] = (1.0 - k_p) * p_pv
            P[v][axis] = P[axis][v]
            P[v][v] = p_vv - k_v * p_pv

    # ----------------------------------------------------------- callbacks
    def gps_cb(self, msg):
        t = msg.header.stamp.to_sec() if msg.header.stamp.to_sec() > 0.0 \
            else rospy.Time.now().to_sec()
        if self.last_gps_t is not None and t < self.last_gps_t:
            rospy.logwarn_throttle(
                1.0, "[localizer] out-of-order GPS dropped (dt=%.3fs)",
                t - self.last_gps_t)
            return
        self.last_gps_t = t
        if msg.eastOffset != 0.0 or msg.northOffset != 0.0:
            self.east_off = msg.eastOffset
            self.north_off = msg.northOffset
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            return
        ux, uy = self.proj_utm(msg.longitude, msg.latitude)
        self._correct_pos(ux - self.east_off, uy - self.north_off)

    def imu_cb(self, msg):
        if msg.orientation.w == 0.0 and msg.orientation.x == 0.0 \
                and msg.orientation.y == 0.0 and msg.orientation.z == 0.0:
            return
        t = msg.header.stamp.to_sec() if msg.header.stamp.to_sec() > 0.0 \
            else rospy.Time.now().to_sec()
        if self.last_gps_t is not None \
                and abs(t - self.last_gps_t) > self.max_sensor_skew:
            rospy.logwarn_throttle(
                1.0,
                "[localizer] unsynchronized IMU dropped "
                "(imu-gps=%.3fs, limit=%.3fs)",
                t - self.last_gps_t, self.max_sensor_skew)
            return
        if self.last_imu_t is not None and t < self.last_imu_t:
            rospy.logwarn_throttle(
                1.0, "[localizer] out-of-order IMU dropped (dt=%.3fs)",
                t - self.last_imu_t)
            return
        raw_yaw = quat_to_yaw(msg.orientation)
        if self.imu_ned:
            # 실측(2026-07-12): 이 MORAI 세팅의 IMU 는 나침반/NED 규약
            # (북쪽 기준 시계+). 맵은 ENU(동쪽 기준 반시계+) → 변환 필수.
            # 경로 시작 방위각(mgeo 실측 +1.86rad) vs IMU(-0.29rad) 로 확정.
            self.yaw = math.atan2(math.sin(math.pi / 2.0 - raw_yaw),
                                  math.cos(math.pi / 2.0 - raw_yaw))
            self.wz = -msg.angular_velocity.z
            if math.isfinite(msg.linear_acceleration.x):
                self.ax_body = msg.linear_acceleration.x
                self.ay_body = -msg.linear_acceleration.y
        else:
            self.yaw = raw_yaw
            self.wz = msg.angular_velocity.z
            if math.isfinite(msg.linear_acceleration.x):
                self.ax_body = msg.linear_acceleration.x
                self.ay_body = msg.linear_acceleration.y
        self.have_yaw = True
        self.synced_imu_t = t
        if self.last_imu_t is not None:
            self._predict(t - self.last_imu_t)
        self.last_imu_t = t

    def odom_cb(self, msg):
        # mgeo 는 local 좌표(UTM - origin)다. gps_cb 는 east_off/north_off 를
        # 빼서 들어오는데 이 경로만 빠져 있었다. EKF 모드(use_full_utm:=true)
        # 에서는 /odom 이 full UTM 이라, 그대로 넣으면 매칭 거리가 400만 m 가
        # 되어 정지선을 하나도 못 찾는다.
        #   실측 2026-09-14(12:08 백): M1~M4 를 전부 통과했는데
        #   /localization/stop_ahead 1256 개가 모두 dist=-1 이었다. origin 을
        #   빼고 오프라인 재현하니 매칭 오차 0.00~0.10 m 로 정지선 4 개
        #   (C1256W000091 / ...016 / ...123 등)를 정확히 찾았다.
        self.yaw = quat_to_yaw(msg.pose.pose.orientation)
        self.X[0] = msg.pose.pose.position.x - self.east_off
        self.X[1] = msg.pose.pose.position.y - self.north_off
        self.X[2] = msg.twist.twist.linear.x * math.cos(self.yaw)
        self.X[3] = msg.twist.twist.linear.x * math.sin(self.yaw)
        self.wz = msg.twist.twist.angular.z
        self.have_pos = True
        self.have_yaw = True
        self.last_odom_t = rospy.Time.now().to_sec()

    # -------------------------------------------------------------- update
    def update(self, _evt):
        if not self.have_pos or not self.have_yaw:
            return
        if self.use_odom:
            now_sec = rospy.Time.now().to_sec()
            if self.last_odom_t is None or now_sec - self.last_odom_t > self.odom_timeout:
                rospy.logwarn_throttle(1.0, "[localizer] fused odom stale; map matching held")
                return
        elif self.last_gps_t is None or self.synced_imu_t is None \
                or abs(self.last_gps_t - self.synced_imu_t) > self.max_sensor_skew:
            rospy.logwarn_throttle(
                1.0, "[localizer] TF held: waiting for synchronized GPS/IMU")
            return
        x, y, yaw = self.X[0], self.X[1], self.yaw
        vx, vy = self.X[2], self.X[3]
        # 진행방향 성분으로 부호 있는 전진속도
        v_fwd = vx * math.cos(yaw) + vy * math.sin(yaw)
        stamp = rospy.Time.now()

        # Match against the fixed global route so a parallel/merging side
        # link cannot replace the driven link near an intersection.
        if self.route_links:
            if self.route_pos >= 0:
                lo = max(0, self.route_pos - 2)
                hi = min(len(self.route_links), self.route_pos + 9)
                candidates = self.route_links[lo:hi]
                m = self.map.match_on_links(x, y, candidates)
                if m is not None:
                    self.route_pos = lo + candidates.index(m["link_idx"])
                if m is None or m["dist"] > 8.0:
                    m = self.map.match_on_links(x, y, self.route_links)
                    if m is not None:
                        self.route_pos = self.route_links.index(m["link_idx"])
            else:
                m = self.map.match_on_links(x, y, self.route_links)
                if m is not None:
                    self.route_pos = self.route_links.index(m["link_idx"])
        else:
            m = self.map.match(x, y)

        # pose
        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = self.frame_id
        ps.pose.position.x = x
        ps.pose.position.y = y
        q = yaw_to_quat(yaw)
        ps.pose.orientation.x, ps.pose.orientation.y = q[0], q[1]
        ps.pose.orientation.z, ps.pose.orientation.w = q[2], q[3]
        self.pub_pose.publish(ps)

        if m is not None:
            self.pub_link.publish(String(data=m["link_idx"]))
            st = Float32MultiArray()
            st.data = [float(x), float(y), float(yaw),
                       float(m["lateral"]), float(m["s"]),
                       float(m["max_speed"] or 0.0), float(m["dist"])]
            self.pub_state.publish(st)

        self.publish_ego(x, y, yaw, stamp, m)
        self.publish_stop_ahead(m, stamp)

        if self.publish_tf and self.tf_br is not None:
            self.tf_br.sendTransform((x, y, 0.0), tuple(q), stamp,
                                     "base_link", self.frame_id)

        if self.publish_odom and not self.use_odom:
            od = Odometry()
            od.header.stamp = stamp
            od.header.frame_id = self.frame_id
            od.child_frame_id = "base_link"
            od.pose.pose = ps.pose
            od.twist.twist.linear.x = v_fwd
            od.twist.twist.angular.z = self.wz
            # 대각 공분산 (소비 노드가 신뢰도 판단할 수 있게)
            od.pose.covariance[0] = self.P[0][0]
            od.pose.covariance[7] = self.P[1][1]
            od.twist.covariance[0] = self.P[2][2] + self.P[3][3]
            self.pub_odom.publish(od)

    # -------------------------------------------------------- stop ahead
    def tl_cb(self, msg):
        self.tl_color = msg.data
        self.tl_color_t = rospy.Time.now().to_sec()

    def route_cb(self, msg):
        links = [x.strip() for x in msg.data.split(",") if x.strip()]
        if links:
            self.route_links = links
            self.route_pos = -1
            rospy.loginfo("[localizer] global route received: %d links",
                          len(links))

    def morai_tl_cb(self, msg):
        self.morai_tl_idx = msg.trafficLightIndex
        self.morai_tl_status = int(msg.trafficLightStatus)
        self.morai_tl_t = rospy.Time.now().to_sec()

    def _stop_required(self, _tl_idx):
        """YOLO 신호색으로 정지 필요 판단.

        green -> 통과(False), red/yellow -> 정지(True),
        none/타임아웃 -> None (미상; 소비측에서 보수적으로 처리).
        """
        now = rospy.Time.now().to_sec()
        # MORAI의 closest-TL 상태를 우선 사용한다. 고정 경로에서는 MGeo가
        # 선택한 다음 신호 ID와 일치할 때만 적용해 옆 교차로 신호를 배제한다.
        if (self.morai_tl_idx and _tl_idx and self.morai_tl_idx == _tl_idx and
                now - self.morai_tl_t <= self.tl_status_timeout):
            if self.morai_tl_status == 1:
                return True
            if self.morai_tl_status == 4:
                return True
            if self.morai_tl_status in (16, 32):
                return False
        if now - self.tl_color_t > self.tl_status_timeout:
            return None
        if self.tl_color == "green":
            return False
        if self.tl_color in ("red", "yellow"):
            return True
        return None

    def publish_stop_ahead(self, m, stamp):
        """전방 신호등 정지점 + 신호상태 결합 정지판단 발행.

        /localization/stop_ahead: [dist, stop_x, stop_y, tl_x, tl_y, tl_z,
                                   stop_flag]  (없으면 dist=-1)
        stop_flag: 1=정지필요, 0=통과가능, -1=신호상태 미상
        """
        now = rospy.Time.now().to_sec()
        if self.ignore_traffic_lights:
            self.stop_hold = False
            self.last_stop_data = None
            self.pub_stop.publish(Float32MultiArray(
                data=[-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
            self.pub_stop_tl.publish(String(data=""))
            self.pub_stop_req.publish(Bool(data=False))
            return
        info = None
        # 최근접 정점과의 거리는 정지선 탐색 게이트로 사용하면 안 된다.
        # 정점 간격보다 멀어지는 순간 신호 정보가 끊겨 늦게 제동하는 문제가 생긴다.
        if m is not None:
            # 터널 구간: 신호등 정지 판단을 완전히 우회한다. 이미 이전
            # 프레임에서 걸린 hold도 해제하여 터널 입구에서 멈추지 않는다.
            if str(m.get("link_idx", "")) in self.traffic_light_ignore_links:
                self.stop_hold = False
                arr = Float32MultiArray()
                arr.data = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                self.pub_stop.publish(arr)
                self.pub_stop_tl.publish(String(data=""))
                self.pub_stop_req.publish(Bool(data=False))
                rospy.logwarn_throttle(
                    2.0, "[TL_DEBUG] traffic-light stop bypass: tunnel link=%s",
                    str(m.get("link_idx", "")))
                return
            if self.route_links and self.route_pos >= 0:
                info = self.map.stop_ahead_route(
                    self.route_links, self.route_pos, m["s"],
                    lookahead=self.stop_lookahead)
            else:
                info = self.map.stop_ahead(m["link_idx"], m["s"],
                                           lookahead=self.stop_lookahead)
        arr = Float32MultiArray()
        if info is None:
            if self.stop_hold and self.tl_color != "green" and self.last_stop_data:
                held = list(self.last_stop_data)
                held[0] = 0.0
                held[6] = 1.0
                arr.data = held
                self.pub_stop.publish(arr)
                self.pub_stop_req.publish(Bool(data=True))
                return
            self.stop_hold = False
            arr.data = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.pub_stop.publish(arr)
            self.pub_stop_tl.publish(String(data=""))
            self.pub_stop_req.publish(Bool(data=False))
            return
        need = self._stop_required(info["tl_idx"])
        flag = -1.0 if need is None else (1.0 if need else 0.0)
        tp = info["tl_point"]
        # Use the actual map XY stop point for the control distance.  The
        # graph/path distance can jump when the nearest link changes, while
        # projecting the stop-point vector onto the vehicle heading remains
        # continuous and prevents stopping past the line.
        dx = float(info["stop_x"]) - float(self.X[0])
        dy = float(info["stop_y"]) - float(self.X[1])
        xy_dist = math.hypot(dx, dy)
        xy_forward = dx * math.cos(self.yaw) + dy * math.sin(self.yaw)
        xy_lateral = -dx * math.sin(self.yaw) + dy * math.cos(self.yaw)
        # 횡방향 게이트는 '뒤에 있는' 정지선에만 건다(xy_forward < 0).
        # 그 경우에만 아래 max(xy_forward, 0.0) 이 거리를 0 으로 눌러
        # "코앞에 정지선"을 만들기 때문이다.
        #   실측 2026-09-14(15:39 백): 첫 정지선 통과 직후 C1256W000046 의
        #   정지선(-18.4,-141.4)을 잡았는데 전방 -9.95 m / 횡 -38.9 m,
        #   즉 40 m 옆이었다. 0 으로 눌린 채 옆에 있으니 통과할 수도 없어
        #   19초간 영구 정지했다.
        #   백 3개에서 xy_forward < 0 인 프레임의 |횡오차| 분포는 0 m 아니면
        #   39~50 m 로 완전히 갈린다 - 0 m 는 내 정지선을 방금 넘은 정상
        #   상황이고, 39 m 이상은 전부 교차로 측면/건너편 신호다.
        # 전방에 있는 정지선은 거리가 왜곡되지 않으므로 건드리지 않는다 -
        # 커브 진입 전에는 내 정지선도 횡오차가 18 m 까지 나오기 때문에
        # 여기에 게이트를 걸면 진짜 정지선을 늦게 본다.
        # 조건 2: 정지선이 한참 뒤에 있으면 이미 지나온 것이다.
        #   실측 2026-09-14(18:07 백) t=111.8: 지도 매칭이 A2256W000220 에서
        #   A2256W000759 로 튀었는데, 그 링크는 진행 반대 방향이라 s 가
        #   25.8 -> 23.8 로 '감소'했다. 거기 달린 정지선(C1256W000115)은
        #   이미 22 m 전에 지나온 것인데 전방 -22.1 m / 횡 +4.1 m 였다.
        #   횡오차가 4.1 m 라 위 횡방향 게이트(5.0)를 못 넘어 통과했고,
        #   max(xy_forward, 0.0) 이 -22.1 을 0 으로 눌러 15초간 정지했다.
        #   '막 넘은' 정상 상황은 -0.5 m 안팎이라 3.0 m 문턱에 안 걸린다.
        # 조건 3: 앞범퍼가 이미 정지선을 넘었으면 그냥 통과시킨다.
        #
        #   xy_forward 는 차량 원점(후륜축) 기준이라, 앞범퍼는 그보다
        #   front_bumper_offset_m 만큼 앞에 있다. 범퍼가 선을 넘은 뒤에는
        #   정지해도 위반이고, 그 위치에서는 신호등이 거의 머리 위라
        #   전방/상향 카메라 모두 화각을 벗어나 색이 'none' 으로 굳는다.
        #   그러면 stop_hold 가 초록을 영영 못 받아 영구 정지한다.
        #     실측 2026-09-17(23:49 백): t=95 에 stop_dist 가 0.00 으로
        #     고정된 뒤 신호색이 none 93.5% / green 0회, 브레이크 1.00 이
        #     96초간 유지되며 주행이 끝났다.
        #   넘었으면 초록과 동일하게 취급해 계속 진행한다.
        bumper_forward = xy_forward - self.front_bumper_offset_m
        crossed = (self.pass_when_bumper_crossed and
                   bumper_forward < 0.0 and
                   abs(xy_lateral) <= self.stop_lateral_max_m)
        if crossed:
            self.stop_hold = False
            self.last_stop_data = None
            arr.data = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.pub_stop.publish(arr)
            self.pub_stop_tl.publish(String(data=""))
            self.pub_stop_req.publish(Bool(data=False))
            rospy.logwarn_throttle(
                2.0,
                "[TL_DEBUG] bumper crossed stop line: tl=%s fwd=%.2fm "
                "bumper=%.2fm lat=%.2fm -> proceed",
                str(info["tl_idx"]), float(xy_forward), float(bumper_forward),
                float(xy_lateral))
            return
        behind = xy_forward < -self.stop_behind_max_m
        off_path = xy_forward < 0.0 and abs(xy_lateral) > self.stop_lateral_max_m
        if behind or off_path:
            self.stop_hold = False
            self.last_stop_data = None
            arr.data = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.pub_stop.publish(arr)
            self.pub_stop_tl.publish(String(data=""))
            self.pub_stop_req.publish(Bool(data=False))
            rospy.logwarn_throttle(
                2.0,
                "[TL_DEBUG] stop line rejected (%s): tl=%s fwd=%.2fm lat=%.2fm "
                "(behind_limit %.1fm lat_limit %.1fm)",
                "behind" if behind else "off-path",
                str(info["tl_idx"]), float(xy_forward), float(xy_lateral),
                float(self.stop_behind_max_m), float(self.stop_lateral_max_m))
            return
        # Once the vehicle reaches/passes the stop point, never publish a
        # negative distance: zero keeps the MPC in full-stop mode instead of
        # accidentally releasing the traffic-light stop after crossing it.
        control_dist = max(xy_forward, 0.0)
        # 이 링크에서만 유효 정지거리를 1 m 늘려 MPC 전달 거리를 9 m로 만든다.
        # TL_DEBUG의 원래 stop_dist/좌표는 진단을 위해 그대로 보존한다.
        control_dist_for_mpc = control_dist
        if str(m.get("link_idx", "")) in self.traffic_light_extra_margin_links:
            control_dist_for_mpc = max(
                control_dist + float(self.traffic_light_extra_margin), 0.0)
        arr.data = [float(control_dist),
                    float(info["stop_x"]), float(info["stop_y"]),
                    float(tp[0]), float(tp[1]), float(tp[2]), flag]
        if str(m.get("link_idx", "")) in self.traffic_light_extra_margin_links:
            arr.data[0] = float(control_dist_for_mpc)
            rospy.logwarn_throttle(
                2.0,
                "[TL_DEBUG] extra stop margin %.1fm: link=%s effective_dist=%.2fm",
                float(self.traffic_light_extra_margin), str(m.get("link_idx", "")),
                float(control_dist_for_mpc))
        self.pub_stop.publish(arr)
        self.pub_stop_tl.publish(String(data=info["tl_idx"]))
        if need is None:
            # 대회에서는 카메라 판정이 아직 미정이어도 정지선을 통과하지
            # 않게 할 수 있다. 선언만 되고 쓰이지 않던 stop_on_unknown과
            # unknown_stop_dist를 실제 정지 판단에 적용한다.
            stop = (self.stop_on_unknown and
                    control_dist <= self.unknown_stop_dist)
        else:
            stop = need
        self.last_stop_data = list(arr.data)
        # hold는 실제 red/yellow 판정으로 정지한 경우에만 건다.
        if need is True and control_dist <= self.stop_hold_dist:
            self.stop_hold = True
        if need is False:
            # 녹색이 보이는 동안 계속 갱신한다. 그래야 신호를 놓친 순간부터
            # 유예가 시작되고, 녹색이 계속 보이는 동안은 애초에 정지 요구가
            # 없으므로 유예가 소모되지 않는다.
            self.green_release_t = now
            self.stop_hold = False

        # 녹색 출발 직후 유예: 방금 지난 교차로의 다른 정지선을 자기 것으로
        # 잡아 다시 서는 것을 막는다.
        # 확정 red/yellow 도 함께 막는다 - 코스상 신호등이 연달아 붙어 있지
        # 않으므로, 녹색으로 출발한 지 몇 초 안에 보이는 신호는 방금 지난
        # 교차로의 다른 신호(측면/건너편)일 수밖에 없다.
        #   실측 15:39 백: 미상(flag=-1)만 막으면 t=165.1 에 그 유령 정지선
        #   에서 yellow(flag=1)가 잡혀 유예를 뚫고 다시 섰다. 그 신호는 실제
        #   검출이지만 우리 정지선이 아니다.
        # ★ 단, '방금 지난 정지선'에만 적용해야 한다. 아직 멀리 있는 정지선은
        #   진짜 다음 신호이므로 막으면 안 된다.
        #     실측 2026-09-15(11:54 백): t=6.3 녹색(41 m 앞) -> t=10.0 에
        #     15.9 m 앞에서 황색/적색으로 바뀌었는데, 유예가 아직 살아 있어
        #     2.4초간 정지를 막았다. 풀렸을 땐 5.5 m 밖에 안 남아 제동이
        #     불가능했고 정지선을 넘어 2.47 m 지점에 섰다.
        #   green_release_stop_dist_m 안쪽(방금 통과한 정지선은 거리가 0 에
        #   가깝다)일 때만 억제한다.
        if (stop and
                control_dist <= self.green_release_stop_dist_m and
                0.0 < now - self.green_release_t < self.green_release_grace_sec):
            rospy.logwarn_throttle(
                1.0,
                "[TL_DEBUG] green-release grace %.1fs: suppress stop "
                "(tl=%s dist=%.2fm flag=%s)",
                self.green_release_grace_sec, str(info["tl_idx"]),
                float(control_dist), str(flag))
            stop = False
            self.stop_hold = False
        rospy.logwarn_throttle(
            0.5,
            "[TL_DEBUG] link=%s s=%.2f match=%.2fm stop_dist=%.2fm xy=%.2fm "
            "ego=(%.2f,%.2f) stop_xy=(%.2f,%.2f) tl_id=%s yolo=%s need=%s stop=%s hold=%s",
            str(m.get("link_idx", "")), float(m.get("s", 0.0)),
            float(m.get("dist", -1.0)), float(control_dist), float(xy_dist),
            float(self.X[0]), float(self.X[1]),
            float(info["stop_x"]), float(info["stop_y"]),
            str(info["tl_idx"]), self.tl_color, str(need), str(stop),
            str(self.stop_hold))
        self.pub_stop_req.publish(Bool(data=stop))

        mk = Marker()
        mk.header.frame_id = self.frame_id
        mk.header.stamp = stamp
        mk.ns = "stop_ahead"
        mk.id = 0
        mk.type = Marker.CYLINDER
        mk.action = Marker.ADD
        mk.pose.position.x = info["stop_x"]
        mk.pose.position.y = info["stop_y"]
        mk.pose.position.z = 0.05
        mk.pose.orientation.w = 1.0
        mk.scale.x = mk.scale.y = 2.0
        mk.scale.z = 0.1
        # 가까울수록 빨강
        near = max(0.0, min(1.0, 1.0 - info["dist"] / self.stop_lookahead))
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = 1.0, 1.0 - near, 0.0, 0.9
        mk.lifetime = rospy.Duration(0.5)
        self.pub_stop_marker.publish(mk)

    # ------------------------------------------------------------- markers
    def publish_ego(self, x, y, yaw, stamp, m):
        mk = Marker()
        mk.header.frame_id = self.frame_id
        mk.header.stamp = stamp
        mk.ns = "ego"
        mk.id = 0
        mk.type = Marker.ARROW
        mk.action = Marker.ADD
        mk.pose.position.x = x
        mk.pose.position.y = y
        mk.pose.position.z = 0.2
        q = yaw_to_quat(yaw)
        mk.pose.orientation.x, mk.pose.orientation.y = q[0], q[1]
        mk.pose.orientation.z, mk.pose.orientation.w = q[2], q[3]
        mk.scale.x, mk.scale.y, mk.scale.z = 3.0, 1.0, 1.0
        # 링크 매칭되면 녹색, 이탈(>5m)이면 빨강
        on_road = (m is not None and m["dist"] < 5.0)
        mk.color.r = 0.0 if on_road else 1.0
        mk.color.g = 1.0 if on_road else 0.0
        mk.color.b = 0.0
        mk.color.a = 1.0
        self.pub_ego.publish(mk)

    def publish_map_markers(self):
        arr = MarkerArray()
        if self.show_link_graph:
            line = Marker()
            line.header.frame_id = self.frame_id
            line.header.stamp = rospy.Time.now()
            line.ns = "links"
            line.id = 0
            line.type = Marker.LINE_LIST
            line.action = Marker.ADD
            line.scale.x = 0.3
            line.color.r, line.color.g, line.color.b, line.color.a = 0.2, 0.6, 1.0, 0.6
            line.pose.orientation.w = 1.0
            for lk in self.map.links.values():
                pts = lk.points
                for k in range(len(pts) - 1):
                    a, b = pts[k], pts[k + 1]
                    line.points.append(Point(a[0], a[1], 0.0))
                    line.points.append(Point(b[0], b[1], 0.0))
            arr.markers.append(line)

            nodes = Marker()
            nodes.header.frame_id = self.frame_id
            nodes.header.stamp = rospy.Time.now()
            nodes.ns = "nodes"
            nodes.id = 1
            nodes.type = Marker.SPHERE_LIST
            nodes.action = Marker.ADD
            nodes.scale.x = nodes.scale.y = nodes.scale.z = 1.5
            nodes.color.r, nodes.color.g, nodes.color.b, nodes.color.a = 1.0, 0.9, 0.1, 0.8
            nodes.pose.orientation.w = 1.0
            for (nx, ny) in self.map.nodes.values():
                nodes.points.append(Point(nx, ny, 0.0))
            arr.markers.append(nodes)

        if self.show_lane_markings:
            self._append_lane_marking_markers(arr)

        self._append_hdmap_feature_markers(arr)

        self.pub_map.publish(arr)
        rospy.loginfo("[localizer] map markers published (%d link-pts, %d nodes)",
                      sum(len(l.points) for l in self.map.links.values()),
                      len(self.map.nodes))

    # --------------------------------------------------- lane markings
    LANE_COLORS = {
        "yellow": (1.0, 0.78, 0.05),
        "white": (0.92, 0.92, 0.92),
        "blue": (0.25, 0.5, 1.0),
    }

    def _append_lane_marking_markers(self, arr):
        """lane_boundary_set 의 노면 도색 차선을 실제 색/형태로 렌더링.

        링크망(주행경로 그래프)과 달리 진짜 차선 폴리라인이라 내비 지도처럼 보인다.
        - lane_color: yellow/white 그대로 반영
        - lane_shape: solid=실선, broken=점선(dash_interval_L1 도색 / L2 공백),
          'solid broken' 류 겹선은 double_line_interval 만큼 벌려 두 줄로.
        정지선(530)은 _append_hdmap_feature_markers 가 따로 그리므로 제외.
        """
        boundaries = self._load_json("lane_boundary_set.json")
        if not boundaries:
            return

        # (color, 'solid'|'broken') -> LINE_LIST 세그먼트 누적 (마커 수 최소화)
        buckets = {}

        def emit(pts, color, shape, dash):
            segs = buckets.setdefault((color, shape), [])
            if shape == "broken":
                segs.extend(self._dash_segments(pts, dash[0], dash[1]))
            else:
                for i in range(len(pts) - 1):
                    segs.append((pts[i], pts[i + 1]))

        for item in boundaries:
            if 530 in (item.get("lane_type") or []):
                continue
            raw = item.get("points") or []
            if len(raw) < 2:
                continue
            pts = [(float(p[0]), float(p[1])) for p in raw]
            color = (item.get("lane_color") or ["white"])[0]
            shape_tokens = ((item.get("lane_shape") or ["solid"])[0] or "solid").split()
            l1 = float(item.get("dash_interval_L1") or 0) or 3.0
            l2 = float(item.get("dash_interval_L2") or 0) or 5.0

            if len(shape_tokens) >= 2:
                # 겹선: 좌우로 반씩 벌려 두 줄 (예: 중앙 실선+점선)
                gap = float(item.get("double_line_interval") or 0) or 0.3
                left, right = self._offset_polyline(pts, gap * 0.5)
                emit(left, color, shape_tokens[0], (l1, l2))
                emit(right, color, shape_tokens[1], (l1, l2))
            else:
                emit(pts, color, shape_tokens[0], (l1, l2))

        mid = 20
        for (color, shape), segs in sorted(buckets.items()):
            if not segs:
                continue
            mk = Marker()
            mk.header.frame_id = self.frame_id
            mk.header.stamp = rospy.Time.now()
            mk.ns = "lane_%s_%s" % (color, shape)
            mk.id = mid
            mid += 1
            mk.type = Marker.LINE_LIST
            mk.action = Marker.ADD
            mk.scale.x = 0.15
            r, g, b = self.LANE_COLORS.get(color, self.LANE_COLORS["white"])
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = r, g, b, 0.95
            mk.pose.orientation.w = 1.0
            for pa, pb in segs:
                mk.points.append(Point(pa[0], pa[1], 0.02))
                mk.points.append(Point(pb[0], pb[1], 0.02))
            arr.markers.append(mk)

    @staticmethod
    def _dash_segments(pts, paint, gap):
        """폴리라인을 아크길이 기준 도색(paint)/공백(gap) 반복으로 잘라 반환."""
        period = paint + gap
        segs = []
        s = 0.0
        for i in range(len(pts) - 1):
            ax, ay = pts[i]
            bx, by = pts[i + 1]
            seg_len = math.hypot(bx - ax, by - ay)
            if seg_len < 1e-9:
                continue
            s1 = s + seg_len
            k = int(s // period)
            while k * period < s1:
                p0 = max(s, k * period)
                p1 = min(s1, k * period + paint)
                if p1 > p0:
                    t0 = (p0 - s) / seg_len
                    t1 = (p1 - s) / seg_len
                    segs.append(((ax + (bx - ax) * t0, ay + (by - ay) * t0),
                                 (ax + (bx - ax) * t1, ay + (by - ay) * t1)))
                k += 1
            s = s1
        return segs

    @staticmethod
    def _offset_polyline(pts, half):
        """폴리라인을 진행방향 좌/우로 half 만큼 평행이동한 두 폴리라인 반환."""
        left, right = [], []
        n = len(pts)
        for i in range(n):
            a = pts[max(i - 1, 0)]
            b = pts[min(i + 1, n - 1)]
            dx, dy = b[0] - a[0], b[1] - a[1]
            norm = math.hypot(dx, dy) or 1.0
            nx, ny = -dy / norm, dx / norm
            left.append((pts[i][0] + nx * half, pts[i][1] + ny * half))
            right.append((pts[i][0] - nx * half, pts[i][1] - ny * half))
        return left, right

    def _load_json(self, name):
        path = os.path.join(self.map.data_dir, name)
        if not os.path.exists(path):
            return []
        try:
            with open(path, "r") as f:
                return json.load(f)
        except Exception as e:
            rospy.logwarn("[localizer] %s load failed: %s", name, e)
            return []

    def _append_hdmap_feature_markers(self, arr):
        traffic_lights = self._load_json("traffic_light_set.json")
        if traffic_lights:
            tl_pole = Marker()
            tl_pole.header.frame_id = self.frame_id
            tl_pole.header.stamp = rospy.Time.now()
            tl_pole.ns = "traffic_light_poles"
            tl_pole.id = 10
            tl_pole.type = Marker.LINE_LIST
            tl_pole.action = Marker.ADD
            tl_pole.scale.x = 0.18
            tl_pole.color.r, tl_pole.color.g, tl_pole.color.b, tl_pole.color.a = 0.75, 0.75, 0.75, 0.95
            tl_pole.pose.orientation.w = 1.0

            tl_head = Marker()
            tl_head.header.frame_id = self.frame_id
            tl_head.header.stamp = rospy.Time.now()
            tl_head.ns = "traffic_light_heads"
            tl_head.id = 11
            tl_head.type = Marker.CUBE_LIST
            tl_head.action = Marker.ADD
            tl_head.scale.x = 0.55
            tl_head.scale.y = 0.25
            tl_head.scale.z = 0.9
            tl_head.color.r, tl_head.color.g, tl_head.color.b, tl_head.color.a = 1.0, 0.15, 0.15, 0.95
            tl_head.pose.orientation.w = 1.0
            for item in traffic_lights:
                p = item.get("point")
                if p:
                    x = float(p[0])
                    y = float(p[1])
                    base_z = 0.0
                    head_z = 2.4
                    tl_pole.points.append(Point(x, y, base_z))
                    tl_pole.points.append(Point(x, y, head_z))
                    tl_head.points.append(Point(x, y, head_z))
            arr.markers.append(tl_pole)
            arr.markers.append(tl_head)

        stop_lines = self._load_json("lane_boundary_set.json")
        if stop_lines:
            stop = Marker()
            stop.header.frame_id = self.frame_id
            stop.header.stamp = rospy.Time.now()
            stop.ns = "stop_lines"
            stop.id = 12
            stop.type = Marker.CUBE_LIST
            stop.action = Marker.ADD
            stop.scale.x = 0.22
            stop.scale.y = 0.55
            stop.scale.z = 0.04
            stop.color.r, stop.color.g, stop.color.b, stop.color.a = 1.0, 1.0, 1.0, 0.95
            stop.pose.orientation.w = 1.0
            for item in stop_lines:
                lane_types = item.get("lane_type") or []
                if 530 not in lane_types:
                    continue
                pts = item.get("points") or []
                for i in range(len(pts) - 1):
                    a, b = pts[i], pts[i + 1]
                    ax = float(a[0])
                    ay = float(a[1])
                    bx = float(b[0])
                    by = float(b[1])
                    seg_len = math.hypot(bx - ax, by - ay)
                    samples = max(1, int(seg_len / 0.25))
                    for j in range(samples + 1):
                        t = float(j) / float(samples)
                        stop.points.append(Point(ax + t * (bx - ax), ay + t * (by - ay), 0.02))
            arr.markers.append(stop)

        crosswalks = self._load_json("singlecrosswalk_set.json")
        if crosswalks:
            cw = Marker()
            cw.header.frame_id = self.frame_id
            cw.header.stamp = rospy.Time.now()
            cw.ns = "crosswalks"
            cw.id = 13
            cw.type = Marker.TRIANGLE_LIST
            cw.action = Marker.ADD
            # TRIANGLE_LIST 는 scale 이 0 이면 rviz 에서 안 보임
            cw.scale.x = cw.scale.y = cw.scale.z = 1.0
            cw.color.r, cw.color.g, cw.color.b, cw.color.a = 1.0, 0.9, 0.2, 0.35
            cw.pose.orientation.w = 1.0
            for item in crosswalks:
                pts = item.get("points") or []
                if len(pts) < 4:
                    continue
                poly = pts[:-1] if pts[0] == pts[-1] else pts
                if len(poly) < 3:
                    continue
                origin = poly[0]
                for i in range(1, len(poly) - 1):
                    a = origin
                    b = poly[i]
                    c = poly[i + 1]
                    cw.points.append(Point(float(a[0]), float(a[1]), 0.01))
                    cw.points.append(Point(float(b[0]), float(b[1]), 0.01))
                    cw.points.append(Point(float(c[0]), float(c[1]), 0.01))
            arr.markers.append(cw)


if __name__ == "__main__":
    try:
        MapLocalizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
