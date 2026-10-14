#!/usr/bin/env python3
"""Early M2 static-obstacle proposals from gravity-aligned raw LiDAR.

GroundGrid remains the authoritative near-field detector.  This node only
recovers sparse, above-ground returns at 12--40 m by accumulating them in the
map frame for a short time.  A proposal needs evidence from multiple scans and
must overlap the reference-path corridor, so a single road return cannot start
an avoidance manoeuvre.
"""

import json
import math
import os
import sys
sys.path.insert(0,os.path.dirname(os.path.realpath(__file__)))
from tunnel_map_geometry import fields_array
from temporal_components import connected_components
from m8_filtered_input import WallSurface, filtered_cloud, far_ground_working_set
from obstacle_geometry import measured_geometry, pose_at, regional_plane_heights
from collections import deque

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from waypoint_maker.msg import Lane
from mission_supervisor.msg import SectionState

try:
    from scipy.spatial import cKDTree
except ImportError:
    cKDTree = None


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _xyz(msg):
    """Zero-copy xyz extraction for ordinary PointCloud2 layouts."""
    fields = {f.name: f for f in msg.fields}
    if not all(k in fields for k in ('x', 'y', 'z')):
        return np.empty((0, 3), np.float32)
    endian = '>' if msg.is_bigendian else '<'
    names, formats, offsets = [], [], []
    for k in ('x', 'y', 'z'):
        f = fields[k]
        if f.datatype != 7:  # sensor_msgs/PointField.FLOAT32
            return np.empty((0, 3), np.float32)
        names.append(k); formats.append(endian + 'f4'); offsets.append(f.offset)
    dtype = np.dtype({'names': names, 'formats': formats,
                      'offsets': offsets, 'itemsize': msg.point_step})
    count = msg.width * msg.height
    arr = np.frombuffer(msg.data, dtype=dtype, count=count)
    out = np.column_stack((arr['x'], arr['y'], arr['z'])).astype(np.float32, copy=False)
    return out[np.isfinite(out).all(axis=1)]


class FarStaticDetector:
    def __init__(self):
        self.active_sections = set(rospy.get_param('~active_sections', ['M2']))
        self.static_preview = False
        self.preview_start_idx = int(rospy.get_param('~static_preview_start_idx', 2200))
        self.preview_end_idx = int(rospy.get_param('~static_preview_end_idx', 2600))
        # 구간별 자차 기준 좌우 창 [y_min, y_max] (좌 +). 터널(M8)은 벽을 이 느슨한
        # 희소 점 검출기에 절대 넣지 않도록 벽 안쪽만 모은다(obstacle_stop 의
        # tunnel_wall_roi: y < -2.4 또는 y > 5.5 가 벽).
        self.section_y_limits = rospy.get_param('~section_y_limits', {'M8': [-2.0, 5.0]})
        # 구간별 mission_mode 태그(병합 노드가 근거리 없을 때 이 값을 쓴다).
        self.section_modes = rospy.get_param('~section_mission_modes', {'M8': 'tunnel'})
        # 구간별 지면 평면 추정(고정 평면 대신). 터널 바닥은 좌로 ~3% 횡경사 +
        # 종경사가 있어 고정 평면 기준으로 0.1~0.35 m 짜리 가짜 물체가 계속 생겼다
        # (17:09/20:27 백). 매 스캔 z = a + b x + c y 를 반복 최소제곱으로 맞춘다.
        self.plane_sections = set(rospy.get_param('~plane_ground_sections', ['M8']))
        self.regional_ground = bool(rospy.get_param('~regional_ground', True))
        self.plane_fit_x = rospy.get_param('~plane_fit_x_range_m', [5.0, 40.0])
        # ★ 2026-09-29 (19:23/19:49 백): 터널에서 횡 범위를 '자차 기준 y' 가 아니라
        #   '기준경로 횡거리'로 자른다. 진입 곡선에서 자차 헤딩이 터널과 어긋나 있으면
        #   차로 안(경로 -1.5 m) 장애물이 자차 기준 -3.3~-4.2 m 로 보여 y 범위
        #   [-2, 5] 에서 잘렸다(19:49: 25~32 m 에서 보였는데 13 m 에서야 제안).
        #   이 값이 있는 구간은 section_y_limits 를 바닥 평면 맞춤 후보에만 쓴다.
        self.section_path_lat = rospy.get_param('~section_path_lat_limits', {})
        # ★ 2026-09-29 (20:27 백): 터널 idx 별 실측 벽/턱 경계(config/tunnel_wall_profile.yaml).
        #   구간(section_profile_sections)에서는 점의 기준경로 횡거리가 경계 안쪽
        #   profile_margin_m 보다 바깥이면 버린다. 경계 없는 idx 는 위 상수 범위를 쓴다.
        prof = rospy.get_param('~tunnel_wall_profile', [])
        self.profile_sections = set(rospy.get_param('~section_profile_sections', ['M8']))
        self.profile_margin = float(rospy.get_param('~profile_margin_m', 0.5))
        self.wprof = None
        if len(prof) >= 6 and len(prof) % 3 == 0:
            a = np.asarray(prof, dtype=np.float64).reshape(-1, 3)
            self.wprof = (a[:, 0], a[:, 1], a[:, 2])
        # 구간별 최소 높이. 터널은 종단 경사가 바뀌는 노면(idx ~7805~7960)이 평면 대비
        # 0.075~0.19 m 로 떠서 물체로 제안됐다(장애물 없는 16:23 백 포함). 실제 낮은
        # 장애물은 0.21~0.46 m.
        self.section_h_min = rospy.get_param('~section_min_height_m', {})
        self.plane_fit_band = float(rospy.get_param('~plane_fit_band_m', 0.6))
        self.plane_fit_min_points = int(rospy.get_param('~plane_fit_min_points', 60))
        # 구간별 최대 높이(터널 천장/조명 잔여 점 제거).
        self.section_h_max = rospy.get_param('~section_max_height_m', {'M8': 2.0})
        self.x_min = float(rospy.get_param('~x_min_m', 12.0))
        self.x_max = float(rospy.get_param('~x_max_m', 40.0))
        self.y_max = float(rospy.get_param('~y_half_m', 4.5))
        self.ground_z = float(rospy.get_param('~ground_z', -1.765))
        self.sensor_x = float(rospy.get_param('~sensor_x_m', 1.1))
        self.sensor_y = float(rospy.get_param('~sensor_y_m', 0.0))
        self.h_min = float(rospy.get_param('~min_height_m', 0.10))
        self.h_max = float(rospy.get_param('~max_height_m', 2.8))
        # ★ 2단계 코리도 (근거리 obstacle_perception 과 같은 구조).
        #   gather_half_width_m : 점을 모으는 폭. 물체 모양을 온전히 잡는다.
        #   path_corridor_half_width_m : 만들어진 물체의 '안쪽 끝'이 이 안이면 제안.
        #   예전에는 1.9 m 하나로 누적 전에 점을 잘라서, 경로에서 1.9 m 밖으로
        #   나간 물체의 바깥 끝이 잘린 채 폭을 쟀다.
        #     실측(09-23 16개 주행): 원거리 바깥 끝 p50 1.89 / p90 1.92,
        #     근거리 p50 2.05 / p90 2.26. 1.9 m 초과 비율 6.9% vs 64.7%.
        #     카고박스·우측 저상물 모두 바깥 0.15~0.35 m 가 잘렸다.
        # 근거리 obstacle_perception 의 path_corridor_half_width(1.7)와 같은 값.
        self.path_half = float(rospy.get_param('~path_corridor_half_width_m', 1.7))
        # 2026-10-14(17:39:16 백): 터널 후반 벽에 붙은 장애물(안쪽 끝 -1.5 / +5.75 m)이 1.7 m 규칙과 벽 띠 군집 탈락으로
        #   원거리에서 한 번도 제안되지 않았다. M8 은 벽 너머 점을 사전 필터가 지우므로 벽 안쪽 전체를 본다.
        #   원거리 단독 물체는 병합에서 확정되지 않고 정지 근거도 아니다(미리 인지·선서행용).
        self.tunnel_path_half = float(rospy.get_param('/tunnel_recovery/far_tunnel_corridor_half_width_m', 7.0))
        self.gather_half = max(self.path_half, float(rospy.get_param('~gather_half_width_m', 7.0)))
        # 넓게 모으면 우측 연석점도 들어와 연석 옆 장애물이 연석과 한 덩어리로
        # 묶여(max_cluster_extent 초과) 통째 버려질 수 있다. 회피 노드의 연석
        # 기반 폭 곡선(road_width_profile)으로 연석선 = 우측한계 + 반폭 + 여유
        # 를 구해, 그 선에서 curb_exclude_margin_m 안쪽부터 바깥 점은 뺀다.
        self.profile_param = rospy.get_param('~road_width_profile_param',
                                             '/obstacle_avoidance_node/road_width_profile')
        self.curb_offset = float(rospy.get_param('~curb_line_offset_m', 1.10))   # 0.95 + 0.15
        self.curb_margin = float(rospy.get_param('~curb_exclude_margin_m', 0.10))
        self.right_open = float(rospy.get_param('~right_open_limit_m', 3.5))
        self.profile = None
        self.profile_t = 0.0
        self.window = float(rospy.get_param('~accumulation_sec', 0.55))
        self.voxel = float(rospy.get_param('~voxel_m', 0.16))
        self.eps = float(rospy.get_param('~cluster_eps_m', 0.65))
        self.min_points = int(rospy.get_param('~min_accumulated_points', 5))
        self.min_scans = int(rospy.get_param('~min_distinct_scans', 2))
        self.max_extent = float(rospy.get_param('~max_cluster_extent_m', 7.0))
        self.extent_trim = float(rospy.get_param('~extent_trim_pct', 5.0))
        # ★ 제안 ID 승계. 예전에는 id 를 중심좌표 해시로 만들었다:
        #       oid = 900000 + abs(hash((round(cx,1), round(cy,1)))) % 90000
        #   중심 추정이 몇 cm 만 움직여도 round 가 다른 값에 떨어져 해시가
        #   무관한 숫자로 튄다. 실측 2026-09-22(17:49:26 백): 같은 물체가
        #   1초에 ID 를 7번 갈아탔다(985323 -> 973612 -> 959612 -> ...,
        #   연속 world 거리는 0.08~0.34 m 뿐).
        #   obstacle_avoidance_node 는 primary 장애물이 바뀌면 회피 방향
        #   락(committed_avoid_side_)을 푼다. ID 가 매 스캔 바뀌니 락이
        #   걸릴 새가 없었고, 4주행에서 d_end 부호가 14/16/6/4회 뒤집혔다.
        #   그래서 이전 주기 제안과 최근접 매칭으로 ID 를 물려준다.
        self.match_dist = float(rospy.get_param('~id_match_distance_m', 1.6))
        self.id_timeout = float(rospy.get_param('~id_timeout_sec', 1.4))
        self.hold_sec = float(rospy.get_param('~proposal_hold_sec', 1.25))
        self.tracked = []      # [{'id', 'xy', 'stamp', 'proposal'}]
        self.next_id = 900001
        self.cpp_prefilter=bool(rospy.get_param("/tunnel_recovery/cpp_prefilter_enabled",False))
        import threading
        self.far_queue=threading.Condition();self.far_pending=None;self.far_wall_pending=None;self.far_processing=threading.Lock()
        self.filtered_pipeline=bool(rospy.get_param("/tunnel_recovery/filtered_pipeline_enabled",False))
        self.wall_surface=WallSurface.load(rospy.get_param("/tunnel_recovery/wall_profile_file",None)) if self.filtered_pipeline else None
        self.verified_wall=bool(rospy.get_param('/tunnel_recovery/verified_wall_surface_only_enabled',False)) and self.cpp_prefilter
        self.filtered_pub=rospy.Publisher('/tunnel/filtered_far',PointCloud2,queue_size=1) if not self.verified_wall else None
        self.wall_input_pub=rospy.Publisher('/tunnel/far_wall_input',PointCloud2,queue_size=1)
        if self.verified_wall:rospy.Subscriber('/tunnel/filtered_far',PointCloud2,self._cloud,queue_size=1,buff_size=2**24)

        self.map_build_pub=rospy.Publisher("/tunnel/filtered_background_input",PointCloud2,queue_size=1)
        self.map_enabled=bool(rospy.get_param("/tunnel_recovery/enabled",True)) and not self.filtered_pipeline
        self.map_stamp=-1e9
        self.section = ''
        self.pose = None
        self.pose_history=deque(maxlen=300)
        self.sensor_z=float(rospy.get_param("/lidar_extrinsic/z",1.33))
        self.route = np.empty((0, 2), np.float64)
        self.route_idx = np.empty((0,), np.int64)
        self.route_tan = np.empty((0, 2), np.float64)
        self.route_tree = None
        self.route_pack = (self.route, self.route_idx, self.route_tan, None)
        self.frames = deque()
        self.scan_seq = 0
        self.pub = rospy.Publisher('/obstacle_info_far', String, queue_size=2)
        self.marker_pub = rospy.Publisher('/obstacle_markers_far', MarkerArray, queue_size=1)
        rospy.Subscriber('/mission/active_section', SectionState, self._section, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self._odom, queue_size=1)
        rospy.Subscriber('/final_waypoint', Lane, self._route, queue_size=1)
        if not self.filtered_pipeline:rospy.Subscriber('/tunnel/map_foreground',PointCloud2,self._map_cloud,queue_size=1,buff_size=2**24)
        rospy.Subscriber('/perception/points_gravity', PointCloud2, self._cloud,
                         queue_size=1, buff_size=8 * 1024 * 1024)

    def _section(self, msg):
        if msg.section_id != self.section and msg.section_id not in self.active_sections:
            self.frames.clear()
            self.tracked = []   # 구간을 벗어나면 ID 승계도 끊는다
        self.section = msg.section_id

    def _odom(self, msg):
        stamp=msg.header.stamp.to_sec() or rospy.Time.now().to_sec()
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                     _yaw(msg.pose.pose.orientation),msg.pose.pose.position.z)
        if self.pose_history and stamp<self.pose_history[-1][0]:
            self.pose_history.clear();self.frames.clear();self.tracked=[]
        if not self.pose_history or stamp>self.pose_history[-1][0]:self.pose_history.append((stamp,self.pose))

    def _route(self, msg):
        # 경로 배열들은 다른 스레드(점군 콜백)가 읽는다. 하나씩 교체하면 길이가
        # 다른 배열을 섞어 읽어 IndexError 가 난다 -> 다 만든 뒤 한 번에 교체.
        self.static_preview = bool(msg.waypoints and
            getattr(msg.waypoints[0], 'lane_number', -1) == 0 and
            self.preview_start_idx <= msg.waypoints[0].waypoint_index <= self.preview_end_idx and
            self.section not in self.active_sections)
        pts = [(w.pose.pose.position.x, w.pose.pose.position.y) for w in msg.waypoints]
        route = np.asarray(pts, dtype=np.float64) if pts else np.empty((0, 2))
        idx = np.asarray([w.waypoint_index for w in msg.waypoints], dtype=np.int64)
        if len(route) >= 2:
            tan = np.gradient(route, axis=0)
            tan = tan / np.maximum(1e-9, np.hypot(tan[:, 0], tan[:, 1]))[:, None]
        else:
            tan = np.empty((0, 2))
        tree = cKDTree(route) if cKDTree is not None and len(route) else None
        self.route_pack = (route, idx, tan, tree)
        self.route, self.route_idx, self.route_tan, self.route_tree = route, idx, tan, tree

    def _curb_line(self, idx):
        """각 점의 CSV idx 에서 우측 연석선 횡거리(양수, m). 연석 없음 = inf."""
        now = rospy.Time.now().to_sec()
        if self.profile is None or now - self.profile_t > 5.0:
            raw = rospy.get_param(self.profile_param, [])
            self.profile = np.asarray(raw, dtype=np.float64).reshape(-1, 3) if len(raw) >= 6 else np.empty((0, 3))
            self.profile_t = now
        out = np.full(len(idx), np.inf)
        if len(self.profile) < 2:
            return out
        inside = (idx >= self.profile[0, 0]) & (idx <= self.profile[-1, 0])
        r = np.interp(idx[inside], self.profile[:, 0], self.profile[:, 1])
        line = np.where(r < self.right_open - 1e-3, r + self.curb_offset, np.inf)
        out[inside] = line
        return out

    def _plane_height(self, p):
        """중력정렬 점군에서 바닥 평면 z = a + b x + c y 를 맞춰 점별 높이를 돌려준다."""
        lim = self.section_y_limits.get(self.section, [-self.y_max, self.y_max])
        cand = p[(p[:, 0] >= float(self.plane_fit_x[0])) & (p[:, 0] <= float(self.plane_fit_x[1])) &
                 (p[:, 1] >= float(lim[0])) & (p[:, 1] <= float(lim[1])) &
                 (np.abs(p[:, 2] - self.ground_z) <= self.plane_fit_band)]
        if len(cand) < self.plane_fit_min_points:
            return None
        A = np.column_stack((np.ones(len(cand)), cand[:, 0], cand[:, 1])).astype(np.float64)
        z = cand[:, 2].astype(np.float64)
        keep = np.ones(len(cand), bool)
        coef = None
        for tol in (0.20, 0.10, 0.06):
            if keep.sum() < self.plane_fit_min_points:
                return None
            coef, *_ = np.linalg.lstsq(A[keep], z[keep], rcond=None)
            res = z - A.dot(coef)
            # 물체 점은 평면 위로만 튄다 -> 아래쪽은 넉넉히, 위쪽만 좁힌다.
            keep = (res <= tol) & (res >= -2.0 * tol)
        ground = coef[0] + coef[1] * p[:, 0] + coef[2] * p[:, 1]
        return p[:, 2] - ground

    def _mode(self):
        return 'static_preview' if self.static_preview else str(self.section_modes.get(self.section, 'static'))

    def _empty(self, stamp):
        self.pub.publish(String(data=json.dumps({
            'stamp': stamp, 'mission_mode': self._mode(), 'obstacles': [],
            'collision_objects': [], 'source': 'raw_temporal_far'})))
        clear = Marker(); clear.action = Marker.DELETEALL
        self.marker_pub.publish(MarkerArray(markers=[clear]))

    def _map_cloud(self,msg):
        if self.filtered_pipeline or self.section!='M8' or not getattr(self,'map_enabled',True):return
        self.map_stamp=msg.header.stamp.to_sec();self._cloud(msg)

    def _far_loop(self):
        while not rospy.is_shutdown():
            with self.far_queue:
                if self.far_pending is None and self.far_wall_pending is None:self.far_queue.wait(.1)
                if self.far_pending is None and self.far_wall_pending is None:continue
                if self.far_wall_pending is not None:msg=self.far_wall_pending;self.far_wall_pending=None
                else:msg=self.far_pending;self.far_pending=None
            with self.far_processing:
                if self.section!='M8':continue
                try:self._cloud_process(msg)
                except Exception as exc:rospy.logerr_throttle(2.,'M8 far worker: %s',exc)

    def _cloud(self,msg):
        if self.cpp_prefilter and self.section=='M8':
            import threading
            if not hasattr(self,'far_worker'):
                self.far_worker=threading.Thread(target=self._far_loop,daemon=True);self.far_worker.start()
            with self.far_queue:
                if any(f.name=='m8_wall_verified' for f in msg.fields):
                    self.far_wall_pending=msg;self.far_queue.notify();return
                if self.far_pending is None or msg.header.stamp>self.far_pending.header.stamp:
                    self.far_pending=msg;self.far_queue.notify()
            return
        with self.far_processing:self._cloud_process(msg)

    def _cloud_process(self, msg):
        wall_filtered=self.section=='M8' and any(f.name=='m8_wall_verified' for f in msg.fields)
        annotated=self.section=='M8' and any(f.name=='map_label' for f in msg.fields)
        if self.section=='M8' and not annotated and rospy.Time.now().to_sec()-getattr(self,'map_stamp',-1e9)<.35:return
        stamp = msg.header.stamp.to_sec() or rospy.Time.now().to_sec()
        if (self.section not in self.active_sections and not self.static_preview) or self.pose is None or len(self.route) < 2:
            self._empty(stamp); return
        scan_pose=pose_at(list(self.pose_history),stamp)
        if scan_pose is None:
            self._empty(stamp);return
        p = _xyz(msg)
        if self.cpp_prefilter and self.section=='M8' and not wall_filtered:
            p=far_ground_working_set(p,self.x_min,self.x_max,self.y_max,self.plane_fit_x,self.section_y_limits.get(self.section,[-self.y_max,self.y_max]))
        height = p[:, 2] - self.ground_z
        h_max = float(self.section_h_max.get(self.section, self.h_max))
        if self.section in self.plane_sections and not annotated and not wall_filtered:
            plane_h = self._plane_height(p)
            if plane_h is None:
                if self.section=="M8" and self.filtered_pipeline and self.filtered_pub is not None:self.filtered_pub.publish(filtered_cloud(msg.header,np.empty((0,4))))
                self._empty(stamp); return      # 지면을 못 잡으면 제안하지 않는다
            height = plane_h
            # 2026-10-09(14:56:06 백): 평면 하나는 종단 경사 변화 구간에서 30 m 앞 노면을 0.2 m 위로 띄워
            #   노면(고정 지면 기준 0.17 m)을 물체로 제안했고, 그 원거리 단독 부모가 정지를 일으켰다.
            #   국소 평면(6x4 m 칸) 높이와 전역 평면 높이가 둘 다 최소 높이를 넘어야 물체로 본다.
            #   국소 평면을 못 잡은 칸은 제안하지 않는다(원거리는 근거리가 다시 확인한다).
            if self.regional_ground:
                reg_h, reg_ok, _ = regional_plane_heights(p, self.ground_z)
                height = np.where(reg_ok, np.minimum(plane_h, reg_h), -np.inf)
        if annotated:
            a=fields_array(msg,['x','y','z','map_ground','map_label']);finite=np.isfinite(a[:,:3]).all(axis=1)
            a=a[finite];height=a[:,2]-a[:,3];height=np.where(a[:,4]==1,-np.inf,height)
        if wall_filtered:height=fields_array(msg,['x','y','z','ground_relative'])[:,3]
        h_min = .08 if annotated else float(self.section_h_min.get(self.section, self.h_min))
        keep = ((p[:, 0] >= (0. if annotated else self.x_min)) & (p[:, 0] <= self.x_max) &
                (np.abs(p[:, 1]) <= self.y_max) &
                (height >= h_min) & (height <= h_max))
        lim = self.section_y_limits.get(self.section)
        path_lim = self.section_path_lat.get(self.section)
        if lim is not None and len(lim) >= 2 and not (path_lim is not None and len(path_lim) >= 2):
            keep &= (p[:, 1] >= float(lim[0])) & (p[:, 1] <= float(lim[1]))
        p = p[keep]
        hloc = height[keep]
        if self.section=="M8" and self.filtered_pipeline and not wall_filtered:
            rows=np.column_stack([p,hloc])
            self.map_build_pub.publish(filtered_cloud(msg.header,rows))
            if self.verified_wall:
                self.wall_input_pub.publish(filtered_cloud(msg.header,rows));return
            rows=rows[self.wall_surface.keep(rows,scan_pose,(self.sensor_x,self.sensor_y))]
            self.filtered_pub.publish(filtered_cloud(msg.header,rows))
            p=rows[:,:3].astype(np.float32);hloc=rows[:,3]
        if len(p):
            # One sample per voxel per scan prevents a dense scan from satisfying
            # the temporal confirmation rule by itself.
            key = np.floor(p / self.voxel).astype(np.int32)
            _, take = np.unique(key, axis=0, return_index=True)
            p = p[take]
            hloc = hloc[take]
        x0,y0,yaw,pose_z=scan_pose
        c, s = math.cos(yaw), math.sin(yaw)
        route, route_idx, route_tan, route_tree = self.route_pack
        # points_gravity keeps the LiDAR origin. Convert to base_link before
        # map compensation and before reporting position_ego.
        if len(p):
            p[:, 0] += self.sensor_x
            p[:, 1] += self.sensor_y
        # Promote BEFORE arithmetic: assigning a float32 UTM sum into a float64
        # destination cannot recover the already quantized coordinates.
        p = p.astype(np.float64, copy=False)
        world = np.empty_like(p, dtype=np.float64)
        if len(p):
            world[:, 0] = x0 + c * p[:, 0] - s * p[:, 1]
            world[:, 1] = y0 + s * p[:, 0] + c * p[:, 1]
            world[:, 2] = p[:, 2]+pose_z+self.sensor_z
            if route_tree is not None:
                d, j = route_tree.query(world[:, :2], k=1)
            else:
                dd = np.sqrt(((world[:, None, :2] - route[None, :, :]) ** 2).sum(2))
                j = dd.argmin(1); d = dd.min(1)
            keep = d <= self.gather_half
            # 부호 있는 기준경로 횡거리(좌 +). 없으면 부호 없는 거리로 대신한다.
            lat = d.copy()
            if len(route_tan) == len(route):
                tx, ty = route_tan[j, 0], route_tan[j, 1]
                lat = -ty * (world[:, 0] - route[j, 0]) + tx * (world[:, 1] - route[j, 1])
                if path_lim is not None and len(path_lim) >= 2 and not (self.section=="M8" and self.filtered_pipeline):
                    lo = np.full(len(lat), float(path_lim[0]))
                    hi = np.full(len(lat), float(path_lim[1]))
                    if (self.wprof is not None and self.section in self.profile_sections and
                            len(route_idx) == len(route)):
                        ik, rk, lk = self.wprof
                        pidx = route_idx[j].astype(np.float64)
                        inside = (pidx >= ik[0]) & (pidx <= ik[-1])
                        r = np.interp(pidx, ik, rk)
                        l = np.interp(pidx, ik, lk)
                        r_ok = inside & (np.interp(pidx, ik, (rk < 0).astype(float)) >= 0.999)
                        l_ok = inside & (np.interp(pidx, ik, (lk > 0).astype(float)) >= 0.999)
                        lo = np.where(r_ok, r + self.profile_margin, lo)
                        hi = np.where(l_ok, l - self.profile_margin, hi)
                    keep &= (lat >= lo) & (lat <= hi)
                # 우측 연석 제외: 횡거리가 연석선 - margin 보다 바깥이면 뺀다.
                if len(route_idx) == len(route) and not (self.section=="M8" and self.filtered_pipeline):
                    line = self._curb_line(route_idx[j])
                    keep &= ~(lat < -(line - self.curb_margin))
            world = world[keep]
            d = lat[keep]
            hloc = hloc[keep]
        self.scan_seq += 1
        if len(world):
            scans = np.full((len(world), 1), self.scan_seq, dtype=np.float64)
            stamps = np.full((len(world), 1), stamp, dtype=np.float64)
            # 6번째 열 = 부호 있는 기준경로 횡거리(좌 +). 안쪽 끝·폭 계산용
            # 7번째 열 = 판정에 쓴 노면 위 높이(top_elevation 을 같은 기준으로 보고)
            self.frames.append((stamp, np.hstack((world, scans, stamps, d[:, None], hloc[:, None]))))
        while self.frames and stamp - self.frames[0][0] > self.window:
            self.frames.popleft()
        proposals = self._cluster(stamp, x0, y0, yaw, route_tan, route_tree, len(route),pose_z+self.sensor_z)
        payload = {'stamp': stamp, 'mission_mode': self._mode(), 'obstacles': proposals,
                   'collision_objects': list(proposals), 'source': 'raw_temporal_far'}
        self.pub.publish(String(data=json.dumps(payload, separators=(',', ':'))))
        self._markers(msg.header, proposals)

    def _cluster(self, stamp, x0, y0, yaw, route_tan=None, route_tree=None, n_route=0, height_origin=0.):
        if cKDTree is None or not self.frames:
            return self._with_hold([], stamp, x0, y0, yaw)
        a = np.vstack([v for _, v in self.frames])
        if len(a) < self.min_points:
            return self._with_hold([], stamp, x0, y0, yaw)
        groups = connected_components(a[:, :2], self.eps)
        out = []
        c, s = math.cos(yaw), math.sin(yaw)
        for ids in groups:
            q = a[ids]
            if len(q) < self.min_points or len(np.unique(q[:, 3])) < self.min_scans:
                continue
            # 안쪽 끝(기준경로에 가장 가까운 점)이 코리도 밖이면 경로와 무관
            latp = q[:, 5]
            half = (getattr(self, 'tunnel_path_half', self.path_half)
                    if (getattr(self, 'section', '') == 'M8' and getattr(self, 'filtered_pipeline', False)) else self.path_half)
            if float(np.min(np.abs(latp))) > half:
                continue
            xy = q[:, :2]
            span = np.ptp(xy, axis=0)
            if float(np.hypot(span[0], span[1])) > self.max_extent:
                continue
            reference_yaw=yaw
            if route_tree is not None and route_tan is not None and len(route_tan)==n_route:
                _,jc=route_tree.query(np.median(xy,axis=0))
                reference_yaw=math.atan2(route_tan[jc,1],route_tan[jc,0])
            g=measured_geometry(xy,reference_yaw)
            # Fresh extrema remain collision evidence, but a single-scan
            # outlier must not seed the permanently retained dimensions.
            repeat=np.zeros(len(q),dtype=bool)
            neighbours=cKDTree(xy).query_pairs(1.5*getattr(self,'voxel',.16))
            for a_idx,b_idx in neighbours:
                if q[a_idx,3]!=q[b_idx,3]:repeat[a_idx]=True;repeat[b_idx]=True
            supported=q[repeat]
            seed=measured_geometry(supported[:,:2],reference_yaw) if len(supported) else g
            center=np.asarray(g['position_world']);length,width=g['length'],g['width']
            dx,dy=center[0]-x0,center[1]-y0
            ex,ey=c*dx+s*dy,-s*dx+c*dy
            if ex<self.x_min-1. or ex>self.x_max+1.:continue
            top=float(np.max(q[:,6])) if q.shape[1]>6 else float(np.max(q[:,2])-height_origin-self.ground_z)
            out.append({
                'id': -1, 'class_name': 'obstacle',
                'position_ego': [round(ex, 3), round(ey, 3), round(float(np.median(q[:, 2]))-height_origin, 3)],
                'position_world': [round(float(center[0]), 3), round(float(center[1]), 3)],
                'footprint_yaw':g['footprint_yaw'],
                'footprint_world':g['footprint_world'],
                'geometry_seed_footprint_world':seed['footprint_world'],
                'geometry_repeated_points':int(len(supported)),
                'support_points_world':supported[:,:3].tolist(),
                'length': round(length, 3), 'width': round(width, 3),
                'radius': round(0.5 * math.hypot(length, width), 3),
                'speed': 0.0, 'velocity': [0.0, 0.0], 'motion_state': 'static',
                'velocity_valid': False, 'confirmed': True, 'passed': False,
                'in_trigger_corridor': True, 'top_elevation': round(top, 3),
                'last_seen': float(np.max(q[:, 4])), 'source': 'raw_temporal_far',
                'evidence_scans': int(len(np.unique(q[:, 3]))),
                'evidence_points': int(len(q))})
        self._assign_ids(out, stamp)
        return self._with_hold(out, stamp, x0, y0, yaw)

    def _with_hold(self, out, stamp, x0, y0, yaw):
        fresh_ids = {p['id'] for p in out}
        c, s = math.cos(yaw), math.sin(yaw)
        for track in self.tracked:
            if track['id'] in fresh_ids or stamp - track['stamp'] > self.hold_sec:
                continue
            held = dict(track['proposal'])
            held['held_prediction'] = True
            held['observation_age'] = round(max(0.0, stamp - track['stamp']), 3)
            dx, dy = held['position_world'][0] - x0, held['position_world'][1] - y0
            ex, ey = c * dx + s * dy, -s * dx + c * dy
            # Hand over to the authoritative near detector once the old
            # proposal has passed the far field; never preserve a passed box.
            if ex < self.x_min - 1.0 or ex > self.x_max + 1.0:
                continue
            held['position_ego'] = [round(ex, 3), round(ey, 3), held['position_ego'][2]]
            out.append(held)
        for p in out:
            p.setdefault('held_prediction', stamp - p['last_seen'] > 0.15)
            p.setdefault('observation_age', round(max(0.0, stamp - p['last_seen']), 3))
        return sorted(out, key=lambda z: z['position_ego'][0])

    def _assign_ids(self, proposals, stamp):
        """이전 주기 제안과 최근접 매칭으로 ID 를 물려준다.

        정적 물체만 다루므로 world 좌표가 거의 고정이다. 가까운 쌍부터
        greedy 로 1:1 매칭하고(한 트랙이 두 제안에 붙지 않게), 남은 제안은
        새 ID 를 받는다. 매칭 거리 상한은 클러스터 중심 흔들림(실측 0.08~
        0.34 m)보다 넉넉하되 이웃 물체를 흡수하지 않을 값으로 둔다.
        """
        alive = [t for t in self.tracked if stamp - t['stamp'] <= self.id_timeout]
        pairs = []
        for pi, p in enumerate(proposals):
            px, py = p['position_world']
            for ti, t in enumerate(alive):
                dist = math.hypot(px - t['xy'][0], py - t['xy'][1])
                if dist <= self.match_dist:
                    pairs.append((dist, pi, ti))
        pairs.sort()
        used_p, used_t = set(), set()
        for dist, pi, ti in pairs:
            if pi in used_p or ti in used_t:
                continue
            proposals[pi]['id'] = alive[ti]['id']
            used_p.add(pi); used_t.add(ti)
        for pi, p in enumerate(proposals):
            if pi not in used_p:
                p['id'] = self.next_id
                self.next_id += 1
        # 이번 주기에 안 보인 트랙도 id_timeout 까지는 보관한다. 그러지
        # 않으면 점군이 한 스캔만 비어도(원거리에서는 흔하다 - 실측 17~20%
        # 프레임) 기억이 통째로 날아가 다음 검출이 새 ID 를 받는다.
        keep = [t for ti, t in enumerate(alive)
                if ti not in used_t and stamp - t['stamp'] <= self.id_timeout]
        self.tracked = keep + [{'id': p['id'], 'xy': tuple(p['position_world']),
                                'stamp': p['last_seen'], 'proposal': dict(p)}
                               for p in proposals]

    def _markers(self, header, obstacles):
        ma = MarkerArray()
        clear = Marker(); clear.action = Marker.DELETEALL; ma.markers.append(clear)
        for i, o in enumerate(obstacles):
            m = Marker(); m.header = header; m.header.frame_id = 'map'
            m.ns = 'far_static_proposal'; m.id = i; m.type = Marker.CUBE; m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y = o['position_world']
            m.pose.position.z = self.ground_z + max(0.2, o['top_elevation']) * 0.5
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = o['length'], o['width'], max(0.2, o['top_elevation'])
            m.color.r, m.color.g, m.color.b = 1.0, 0.45, 0.0
            m.color.a = 0.35 if o.get('held_prediction', False) else 0.8
            m.lifetime = rospy.Duration(0.35); ma.markers.append(m)
        self.marker_pub.publish(ma)


if __name__ == '__main__':
    rospy.init_node('far_static_obstacle_detector')
    FarStaticDetector()
    rospy.spin()
