#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Velodyne(또는 merged_lidar) 점군 기반 장애물 검출-추적-TTC 노드.

입력
----
  ~cloud_topic (sensor_msgs/PointCloud2) : 라이다 점군 (센서 프레임). 기본 /merged_lidar
  /odom        (nav_msgs/Odometry)       : 자차 위치/속도 (기본 pose 소스)
  /imu         (sensor_msgs/Imu)         : 자차 yaw. heading_source=imu 일 때 우선
  /Ego_topic   (morai_msgs/EgoVehicleStatus) : use_ego_topic:=true 시 위치/속도/heading 소스

파이프라인
----------
  1) 점군 필터: 거리 게이트 -> 적응형 지면 제거 -> 머리위 제거
  2) ★스캔시각(stamp - cloud_latency) 의 ego 포즈/yaw 를 링버퍼에서 보간해 map(ENU) 변환
     (rosbridge 지연 미보정으로 생기던 진행방향 편이·선회 스미어·유령속도 제거)
  3) BEV 격자 연결요소 클러스터링(scipy.ndimage.label) + PCA OBB
  4) MultiObjectTracker(칼만 CV + 헝가리안)로 ID 유지, 절대/상대속도 추정
  5) 속도 히스테리시스로 정적/동적 분류
  6) ★TTC: 차체 오버행 보정 + CPA 미스거리 코리도 게이트 + 전방 in-path 만 유효 +
     지연 보상(closing·latency) — '제동 늦추는' 계통 낙관 오차 제거
  7) /tracked_objects(TrackedObjectArray) + /tracked_objects_markers 발행

좌표계 주의
-----------
  map 원점은 pose 소스(odom/Ego_topic)의 좌표. 검출/추적/TTC 는 전부 자기 일관.
"""
import os
import sys
import math
from collections import deque

_THIS_DIR = os.path.dirname(os.path.realpath(__file__))
if _THIS_DIR not in [os.path.realpath(p or '.') for p in sys.path]:
    sys.path.insert(0, _THIS_DIR)

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, Imu
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from morai_msgs.msg import EgoVehicleStatus

from lidar_ttc_tracker.msg import TrackedObject, TrackedObjectArray
from object_tracking import MultiObjectTracker
from apollo_motion_bridge import load_library

try:
    from scipy.ndimage import label as cc_label
    _HAVE_SCIPY = True
except Exception:
    _HAVE_SCIPY = False


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class LidarTTCTrackerNode(object):
    def __init__(self):
        rospy.init_node('lidar_ttc_tracker', anonymous=False)

        # ---- 토픽 ----
        self.cloud_topic = rospy.get_param('~cloud_topic', '/merged_lidar')
        self.ground_input_mode = rospy.get_param('~ground_input_mode', 'raw')
        self.use_ego_topic = rospy.get_param('~use_ego_topic', False)
        self.ego_topic = rospy.get_param('~ego_topic', '/Ego_topic')
        self.odom_topic = rospy.get_param('~odom_topic', '/odom')
        self.imu_topic = rospy.get_param('~imu_topic', '/imu')
        self.heading_source = rospy.get_param('~heading_source', 'imu')

        # ---- 라이다 장착 오프셋 (차량 기준 전방 x / 좌측 y) ----
        # MORAI SensorInfo 의 Lidar3D pos.x 와 반드시 같아야 한다.
        # 기본값 3.5 는 2라이다 merged_lidar 시절의 lidar1 프레임 값이었는데,
        # 규정상 라이다 1대(/velodyne_points)로 바뀐 뒤로는 맞지 않는다.
        self.lidar_x = rospy.get_param('~lidar_x', 1.1)
        self.lidar_y = rospy.get_param('~lidar_y', 0.0)

        # ---- ★시간 동기화: 점군 스캔 지연 보정 ----
        # dogm_node 의 cloud_latency 선례와 동일 규약. 스캔 stamp 에서 이만큼 뺀 시점의
        # ego 포즈/yaw 로 변환한다. merged_lidar(~5Hz) 는 0.15~0.25 사이 실측 튜닝.
        self.cloud_latency = rospy.get_param('~cloud_latency', 0.15)
        self.pose_buffer_sec = rospy.get_param('~pose_buffer_sec', 1.0)

        # ---- 포인트 필터 ----
        self.min_range = rospy.get_param('~min_range', 1.5)
        self.max_range = rospy.get_param('~max_range', 50.0)
        # ★자차 footprint 제거: 센서(lidar1, 차 앞 ~3.5m) 프레임의 차체 박스.
        #   dogm_node 와 동일 규약/값. 이 박스 안 점(자차 후드·지붕·미러 자기반사)은
        #   제거하지 않으면 '바로 앞 장애물'로 잡혀 AEB 가 긴급정지 -> 출발 불가/계속 멈춤.
        self.ego_x_min = rospy.get_param('~ego_x_min', -5.0)   # 뒤(라이다 기준 뒤쪽 차체)
        self.ego_x_max = rospy.get_param('~ego_x_max', 0.6)    # 앞(라이다 약간 앞 범퍼)
        self.ego_y_half = rospy.get_param('~ego_y_half', 1.3)  # 반폭(차폭+미러 여유)
        # 차량 기준 ROI. 장착 오프셋을 반영한 뒤 적용하므로, 센서 위치가 바뀌어도
        # 같은 차량/차선 영역을 유지한다. 기본은 기존 동작 호환을 위해 비활성화.
        self.roi_enabled = rospy.get_param('~roi_enabled', False)
        self.roi_x_min = rospy.get_param('~roi_x_min', -70.0)
        self.roi_x_max = rospy.get_param('~roi_x_max', 120.0)
        self.roi_y_half = rospy.get_param('~roi_y_half', 5.5)
        self.ground_pct = rospy.get_param('~ground_percentile', 20.0)
        self.ground_clearance = rospy.get_param('~ground_clearance', 0.38)
        self.ground_bin_size = rospy.get_param('~ground_bin_size', 2.0)
        # 지면 기준으로 쓸 bin 의 자격. 노면 링은 ROI 폭을 연속으로 덮고 물체는
        # 좁은 덩어리 몇 개에 그친다 - 횡방향 점유 셀 수가 이에 못 미치는 bin 은
        # 지면을 정의하지 못하고 이웃에서 보간받는다. 근거는 _points_map 참고.
        self.ground_y_cell = rospy.get_param('~ground_y_cell', 1.0)
        # 노면 링은 ROI 폭 11m 를 11셀 전부 덮는다. 물체 여러 개가 흩어져 있어도
        # 5셀 안팎에 그치므로(실측: 43m 차 2대 = 5셀, 23m 박스 2개 = 3셀) 문턱을
        # 그 위로 둬야 물체가 자기 지면을 정의하는 것을 막는다.
        self.ground_min_y_cells = rospy.get_param('~ground_min_y_cells', 8)
        self.ground_min_points = rospy.get_param('~ground_min_points', 8)
        # 차량 로컬 원점(후륜축) 기준 장착고. MORAI SensorInfo 의 pos.z 와 같은
        # 기준이며, TF(base_link->velodyne)에 쓰는 값이다.
        self.lidar_z = rospy.get_param('~lidar_z', 1.33)
        # 지면 기준 장착고. 위와 기준이 다르다 - 차량 원점은 지면이 아니므로
        # (서스펜션·타이어·바디 오프셋만큼) 두 값이 같을 이유가 없다.
        # 아래 nominal_ground_z 는 '점이 하나도 없는 bin 의 지면을 무엇으로
        # 가정할지'라서 반드시 지면 기준이어야 한다.
        # 값의 단일 출처는 total_bringup/config/lidar_extrinsic.yaml 의
        # z_from_ground 다. 여기 기본값은 그 파일이 없을 때의 폴백일 뿐이라,
        # 바꿀 일이 있으면 그 파일을 고친다(노드마다 따로 갖고 있다가 서로
        # 어긋나서 실제로 9cm 오차가 방치됐던 이력이 있다).
        # 실측 2026-09-08 (10프레임): 두 방법이 정확히 일치했다.
        #   A) 링 반경 역산 h = r_med*tan|각도|   B) z 중앙값 h = -median(z)
        #   ring0(-15) r= 6.03 -> 1.617   ring4(-7) r=12.97 -> 1.592
        #   ring1(-13) r= 6.99 -> 1.615   ring5(-5) r=17.93 -> 1.568
        #   ring2(-11) r= 8.28 -> 1.609   ring6(-3) r=28.75 -> 1.507
        #   ring3( -9) r=10.12 -> 1.603   ring7(-1) r=80.48 -> 1.405
        # 원거리 링일수록 작아지는 건 노면이 앞으로 갈수록 낮아져서다.
        # 새 위치는 2026-09-11 GroundGrid 지면점 40프레임 평면 피팅 실측값이다.
        self.ground_z_below_sensor = rospy.get_param(
            '~ground_z_below_sensor', 1.765)
        # 공용 기준값과 어긋나면 경고한다. 노드마다 값을 따로 들고 있는 구조라
        # 한쪽만 고치는 실수가 실제로 있었고(9cm 오차가 방치됨), 조용히 틀리는
        # 것보다 로그에 남는 편이 낫다.
        _shared_h = rospy.get_param('/lidar_extrinsic/z_from_ground', None)
        if _shared_h is not None and abs(float(_shared_h) - self.ground_z_below_sensor) > 0.005:
            rospy.logwarn(
                '[lidar_ttc_tracker] 센서고 불일치: 이 노드 %.3f m vs '
                '공용 /lidar_extrinsic/z_from_ground %.3f m - '
                'config/lidar_extrinsic.yaml 기준으로 맞출 것',
                self.ground_z_below_sensor, float(_shared_h))
        self.z_max = rospy.get_param('~z_max', 2.5)

        # ---- 클러스터링/박스 ----
        self.cluster_res = rospy.get_param('~cluster_res', 0.5)
        self.min_cluster_pts = rospy.get_param('~min_cluster_pts', 3)
        self.max_cluster_pts = rospy.get_param('~max_cluster_pts', 5000)
        self.cluster_fit_max_pts = int(rospy.get_param('~cluster_fit_max_pts', 2000))
        self.min_box_size = rospy.get_param('~min_box_size', 0.2)
        self.max_box_size = rospy.get_param('~max_box_size', 8.0)
        # 지면제거가 지면+ground_clearance 아래를 이미 잘라내므로, 여기 보이는
        # 높이는 물체의 실제 높이가 아니라 '문턱 위로 드러난 부분'이다. 0.20 은
        # 그 위에 다시 20cm 를 요구해 낙하물·저상 장애물을 통째로 지웠다. 납작한
        # 노면 잔차 방어는 아래 size_z/extent 비율 게이트가 맡는다.
        self.min_box_height = rospy.get_param('~min_box_height', 0.12)
        # --- 링 연속성 보강(원거리 물체 복원). 자세한 근거는 _ring_object_mask 참고 ---
        self.ring_continuity_enabled = rospy.get_param('~ring_continuity_enabled', True)
        # 이 거리 밖에서만 적용. 근거리는 기존 z 문턱이 이미 잘 동작한다.
        self.ring_min_range = rospy.get_param('~ring_min_range', 18.0)
        # 같은 링 인접 점 대비 이만큼 가까우면 물체 앞면으로 본다.
        # 실측상 노면은 방위각 이웃 간 r 차이가 0.3m 를 잘 안 넘는다.
        self.ring_range_step_m = rospy.get_param('~ring_range_step_m', 1.0)
        # 지면 추정치 + 이 높이 아래는 링이 끊겨도 노면으로 간주(오탐 방지).
        self.ring_floor_margin = rospy.get_param('~ring_floor_margin', 0.15)
        # 방위각이 이보다 벌어진 이웃은 비교하지 않는다(스캔 결손 구간).
        self.ring_max_az_gap_deg = rospy.get_param('~ring_max_az_gap_deg', 1.0)
        # 가장자리에서 물체 앞면을 따라 확장할 때 '평탄'으로 볼 인접 거리차(m).
        # 실측상 물체 앞면 내부는 0.1m 미만, 노면은 방위각당 0.3m 안팎 변한다.
        self.ring_flat_tol_m = rospy.get_param('~ring_flat_tol_m', 0.25)
        # 확장 반복 상한(= 한 물체가 차지하는 방위각 점 수 상한).
        self.ring_max_grow_steps = rospy.get_param('~ring_max_grow_steps', 40)
        # 이어진 점이 이 개수 미만인 덩어리는 버린다. 높이 조건을 못 쓰는 대신
        # 노면의 우발적 불연속(한두 점)을 걸러내는 주 방어선이다.
        self.ring_min_run_points = rospy.get_param('~ring_min_run_points', 4)

        # --- 링 기반 원거리 지면 판별 (_ring_ground_mask) ---
        # bin 퍼센타일이 무너지는 구간(링 사이에 노면 점이 없는 원거리)에서
        # 노면을 따로 지운다. 지금은 형상 필터(far_min_box_width 등)로 노면
        # '선'을 사후에 거르고 있는데, 애초에 점 단계에서 지우는 편이 낫다.
        self.ring_ground_enabled = rospy.get_param('~ring_ground_enabled', True)
        # ★ 고주로 원거리 고정 노면(2026-09-25). VLP-16 은 32 m 밖에서 노면 점이
        #   없고(-3deg 링이 32 m 에서 노면에 닿고 -1deg 링은 ~100 m) 차에는 -1deg 링
        #   한 줄만 닿는다. GroundGrid 는 그 칸의 유일한 점(=차)을 지면으로 잡아
        #   ground_relative ~0 을 주고, 아래 원거리 판정이 '고도 0 = 노면 잔차' 로
        #   버렸다. 실측(19:58 백): 앞차 원시 점이 41~44 m 에 스캔당 7~26 개 있었는데
        #   36 m 에서야(3 s 늦게) 추적 시작.
        #   고주로는 평지(GPS 고도 30 m 내 경사 최대 0.33 %)이고, 입력 점
        #   (/groundgrid/segmented_sensor)은 이미 IMU 로 중력 보정돼 있어 스캔별
        #   노면 높이 흔들림이 5 cm 이내다(여기서 롤/피치를 다시 돌리면 이중 보정으로
        #   31 cm 로 나빠진다). 그래서 이 거리 밖 점은 고정 노면(센서 아래
        #   ground_z_below_sensor - far_fixed_ground_offset_m)도 지면 후보로 보고, GroundGrid 지면과 둘 중
        #   낮은 쪽을 쓴다(차가 지면에 흡수되는 것만 막는다). 고정 노면 위
        #   far_fixed_ground_min_height 이상이면 지면 위 점으로 본다.
        #   미션 구간 ID 가 M6/M7 로 최신 수신될 때만 켠다(터널 M8 등 제외).
        self.far_fixed_ground_enabled = rospy.get_param('~far_fixed_ground_enabled', True)
        self.far_fixed_ground_min_range = float(rospy.get_param('~far_fixed_ground_min_range', 28.0))
        self.far_fixed_ground_max_range = float(rospy.get_param('~far_fixed_ground_max_range', 90.0))
        self.far_fixed_ground_offset_m = float(rospy.get_param('~far_fixed_ground_offset_m', 0.05))
        self.far_fixed_ground_min_height = float(rospy.get_param('~far_fixed_ground_min_height', 0.20))
        self.far_fixed_ground_sections = list(rospy.get_param('~far_fixed_ground_sections', ['M6', 'M7']))
        # 이 거리 밖에서만 적용. 근거리는 bin 퍼센타일이 이미 잘 동작한다
        # (실측 제거율 5-10m 74%).
        self.ring_ground_min_range = rospy.get_param('~ring_ground_min_range', 18.0)
        # 방위각 섹터 수. 실측상 섹터 내 산포는 작고(ring6 까지 1.2m) 섹터 간
        # 편차가 크므로(ring5 에서 6.2m) 방향별로 기준을 따로 잡아야 한다.
        self.ring_ground_sectors = rospy.get_param('~ring_ground_sectors', 36)
        # 섹터 노면 반경에서 이만큼 안이면 노면으로 본다.
        self.ring_ground_tolerance_m = rospy.get_param('~ring_ground_tolerance_m', 1.5)
        # 기준을 세울 최소 점 개수. 적으면 물체에 끌려간다.
        self.ring_ground_min_points = rospy.get_param('~ring_ground_min_points', 5)
        # RViz only: false separates confirmed vehicles, persistent far
        # candidates and ordinary unknown objects by namespace/color.
        self.marker_vehicles_only = rospy.get_param('~marker_vehicles_only', False)
        self.marker_far_candidate_min_range = rospy.get_param(
            '~marker_far_candidate_min_range', 25.0)
        self.marker_far_candidate_min_hits = rospy.get_param(
            '~marker_far_candidate_min_hits', 3)
        self.marker_far_candidate_min_obstacle_confidence = rospy.get_param(
            '~marker_far_candidate_min_obstacle_confidence', 0.55)
        # 이 거리 밖에서는 박스 '높이' 하한만 완화한다(링 한 줄만 닿아 size_z~0).
        self.far_size_gate_range_m = rospy.get_param('~far_size_gate_range_m', 25.0)
        # 높이는 링 한 줄이면 물리적으로 0 이므로 풀 수밖에 없다.
        self.far_min_box_height = rospy.get_param('~far_min_box_height', 0.0)
        # 폭까지 같이 풀었더니(0.0) 25m 밖 트랙이 5개 -> 125개로 폭증했고 그중
        # 94% 가 size_y 중앙값 0.16m 인 '선' 이었다(실측 2026-09-06 20:34 백).
        # 노면·차선·가드레일은 링을 따라 가늘고 길게 잡히므로, 높이를 못 쓰는
        # 만큼 폭이 유일한 형상 방어선이다. 차량 폭(1.8m)의 최소한만 요구한다.
        self.far_min_box_width = rospy.get_param('~far_min_box_width', 0.6)
        # 링 한 줄짜리 검출은 가늘고 길수록 노면일 확률이 높다. 이 비(길이/폭)를
        # 넘으면 원거리에서는 버린다.
        self.far_max_aspect_ratio = rospy.get_param('~far_max_aspect_ratio', 4.0)
        self.min_box_width = rospy.get_param('~min_box_width', 0.30)

        # --- 상공 구조물(신호등, 가늠대, 표지판 등) 배제 파라미터 ---
        # z_sensor_ceiling(0.9m)은 기존 튜닝값을 유지한다.
        # 일반 승용차(1.5m) 및 SUV/소형화물(~2.0m)은 온전히 수용하고, 3.5~5m 상공의 신호등을 완전 배제.
        self.z_sensor_ceiling = float(rospy.get_param('~z_sensor_ceiling', 0.90))
        self.max_vehicle_elevation = float(rospy.get_param('~max_vehicle_elevation', 2.10))
        self.ground_cand_z_max = float(rospy.get_param('~ground_cand_z_max', -0.30))

        # --- 단일 링 원거리 차량 가설 ---
        # VLP-16(2deg 간격)이 승용차(1.5m)를 때리는 원거리 구간에서는
        # 물리적으로 관측 불가능해지고, 관측량은 뒷면을 가로지르는 현 하나뿐이다.
        # 그래서 이 구간은 근거리 형상 기준을 완화하는 대신 관측 모델을 바꾼다.
        self.single_ring_max_size_z = rospy.get_param('~single_ring_max_size_z', 0.18)
        # 기존 튜닝값을 유지한다.
        self.single_ring_min_range = rospy.get_param('~single_ring_min_range', 32.0)
        # 현 길이 = 차량 폭. 승용 1.7~1.9, 화물 2.5 안팎. (신호등 바 1.06m는 배제)
        self.far_chord_soft_min = rospy.get_param('~far_chord_soft_min', 0.8)
        self.far_chord_ideal_min = rospy.get_param('~far_chord_ideal_min', 1.2)
        self.far_chord_ideal_max = rospy.get_param('~far_chord_ideal_max', 2.7)
        self.far_chord_soft_max = rospy.get_param('~far_chord_soft_max', 3.4)
        # 접선성: 현이 시선에 수직이면 1, 나란하면 0. 차량 뒷면은 수직이고
        # 가드레일/차선/노면 가장자리는 시선과 나란하다. 높이를 못 쓰는 구간에서
        # 오탐(실측 2026-09-06: 25m 밖 트랙 5 -> 125개, 94%가 가늘고 긴 '선')을
        # 막는 주 방어선이라 종횡비보다 이쪽이 직접적이다.
        self.far_tangential_min = rospy.get_param('~far_tangential_min', 0.50)
        self.far_tangential_full = rospy.get_param('~far_tangential_full', 0.80)
        # 지면 위 고도.
        # 60m 0.65m, 80m 0.30m 를 때린다. 노면 잔차는 0 근처이므로 이것이
        # 유일하게 남은 수직 단서다.
        self.far_elev_soft_min = rospy.get_param('~far_elev_soft_min', 0.05)
        self.far_elev_ideal_min = rospy.get_param('~far_elev_ideal_min', 0.15)
        self.far_elev_ideal_max = rospy.get_param('~far_elev_ideal_max', 1.6)
        self.far_elev_soft_max = rospy.get_param('~far_elev_soft_max', 2.0)
        # 관측된 것은 자차를 향한 앞면뿐이라 길이/높이는 관측량이 아니다. 차량
        # 가설이 성립할 때만 공칭값으로 박스를 세우고, 앞면 위치를 보존하도록
        # 중심을 시선 방향으로 length/2 밀어 TTC/차간거리를 면 기준으로 유지한다.
        self.far_vehicle_width_min = rospy.get_param('~far_vehicle_width_min', 1.5)
        self.far_vehicle_width_max = rospy.get_param('~far_vehicle_width_max', 2.6)
        self.far_vehicle_length = rospy.get_param('~far_vehicle_length', 4.5)
        self.far_vehicle_height = rospy.get_param('~far_vehicle_height', 1.5)
        # 차량 가설은 실패했지만 지면 위로 올라와 있는 원거리 물체(낙하물 등).
        # 오탐을 만든 것은 가늘고 길게 이어지는 노면·가드레일이므로 길이로 막고,
        # footprint 하한은 하류 target_selector 의 min_object_length/width(0.7/0.4)
        # 를 통과하도록 잡는다.
        self.far_obstacle_max_chord = rospy.get_param('~far_obstacle_max_chord', 2.0)
        self.far_obstacle_min_footprint = rospy.get_param(
            '~far_obstacle_min_footprint', 0.8)
        self.far_obstacle_min_height = rospy.get_param('~far_obstacle_min_height', 0.3)

        # --- 장애물성(차량성과 분리) ---
        # 차량성은 '승용차 박스 형상에 맞는가'를 보므로 낮은 장애물과 단일 링
        # 원거리 검출에 0 을 준다. 그 0 을 안전 경로가 그대로 쓰면 '차 모양이
        # 아니라서' 실재하는 장애물을 버린다. 그래서 형상과 무관하게 '무언가
        # 실제로 있다'는 증거만 따로 매긴다.
        self.obstacle_elev_soft_min = rospy.get_param('~obstacle_elev_soft_min', 0.03)
        self.obstacle_elev_ideal_min = rospy.get_param('~obstacle_elev_ideal_min', 0.10)
        self.obstacle_elev_ideal_max = rospy.get_param('~obstacle_elev_ideal_max', 1.8)
        self.obstacle_elev_soft_max = rospy.get_param('~obstacle_elev_soft_max', 2.1)
        self.obstacle_full_points = rospy.get_param('~obstacle_full_points', 12)
        self.obstacle_extent_soft_min = rospy.get_param('~obstacle_extent_soft_min', 0.10)
        self.obstacle_extent_ideal_min = rospy.get_param('~obstacle_extent_ideal_min', 0.25)
        self.obstacle_extent_ideal_max = rospy.get_param('~obstacle_extent_ideal_max', 8.0)
        self.obstacle_extent_soft_max = rospy.get_param('~obstacle_extent_soft_max', 12.0)
        # PCA 장축 신뢰도: 고유값 분리도와 점 수가 모두 낮으면 yaw 갱신/매칭에
        # 약하게만 반영한다. 차량성 점수와는 별도 개념이다.
        self.yaw_confidence_full_points = max(
            self.min_cluster_pts + 1,
            int(rospy.get_param('~yaw_confidence_full_points', 20)))
        self.yaw_confidence_anisotropy_min = max(
            1.0, float(rospy.get_param('~yaw_confidence_anisotropy_min', 1.5)))
        self.yaw_confidence_anisotropy_full = max(
            self.yaw_confidence_anisotropy_min + 1e-3,
            float(rospy.get_param('~yaw_confidence_anisotropy_full', 4.0)))
        self.yaw_confidence_axis_swap_penalty = float(np.clip(
            rospy.get_param('~yaw_confidence_axis_swap_penalty', 0.25), 0.0, 1.0))

        # ---- 차량성(geometry) 점수 ----
        # 이 단계에서는 semantic detector 없이 OBB의 길이/폭/높이/종횡비만 사용한다.
        # 중요한 점은 비차량 군집도 추적은 계속하고, 점수와 시간 누적을 근거로
        # 차량 후보 여부만 분리한다는 것이다. 그래서 가려진 차량을 즉시 버리지
        # 않으면서도 벽·기둥·터널 구조물이 SMPC의 차량 후보가 되는 일을 줄인다.
        self.vehicle_score_enabled = rospy.get_param('~vehicle_score_enabled', True)
        self.vehicle_length_soft_min = rospy.get_param('~vehicle_length_soft_min', 0.7)
        self.vehicle_length_ideal_min = rospy.get_param('~vehicle_length_ideal_min', 1.8)
        self.vehicle_length_ideal_max = rospy.get_param('~vehicle_length_ideal_max', 8.0)
        self.vehicle_length_soft_max = rospy.get_param('~vehicle_length_soft_max', 12.0)
        self.vehicle_width_soft_min = rospy.get_param('~vehicle_width_soft_min', 0.45)
        self.vehicle_width_ideal_min = rospy.get_param('~vehicle_width_ideal_min', 1.1)
        self.vehicle_width_ideal_max = rospy.get_param('~vehicle_width_ideal_max', 3.0)
        self.vehicle_width_soft_max = rospy.get_param('~vehicle_width_soft_max', 4.2)
        self.vehicle_height_soft_min = rospy.get_param('~vehicle_height_soft_min', 0.25)
        self.vehicle_height_ideal_min = rospy.get_param('~vehicle_height_ideal_min', 0.65)
        self.vehicle_height_ideal_max = rospy.get_param('~vehicle_height_ideal_max', 3.2)
        self.vehicle_height_soft_max = rospy.get_param('~vehicle_height_soft_max', 4.2)
        self.vehicle_aspect_soft_min = rospy.get_param('~vehicle_aspect_soft_min', 0.85)
        self.vehicle_aspect_ideal_min = rospy.get_param('~vehicle_aspect_ideal_min', 1.25)
        self.vehicle_aspect_ideal_max = rospy.get_param('~vehicle_aspect_ideal_max', 5.5)
        self.vehicle_aspect_soft_max = rospy.get_param('~vehicle_aspect_soft_max', 9.0)

        # ---- 추적기 (칼만 + 헝가리안) ----
        motion_backend = rospy.get_param('/motion_tracking/backend', 'legacy')
        if motion_backend == 'apollo_measurement':
            load_library()
        self.tracker = MultiObjectTracker(
            motion_backend=motion_backend,
            motion_config=rospy.get_param('/motion_tracking/highway', {}),
            gate_dist=rospy.get_param('~gate_dist', 3.0),
            max_age=rospy.get_param('~max_age', 5),
            min_hits=rospy.get_param('~min_hits', 3),
            max_coast_age=rospy.get_param('~max_coast_age', 2),
            max_coast_sec=rospy.get_param('~max_coast_sec', 0.8),
            max_track_age_sec=rospy.get_param('~max_track_age_sec', 1.0),
            sigma_acc=rospy.get_param('~sigma_acc', 2.0),
            sigma_meas=rospy.get_param('~sigma_meas', 0.3),
            yaw_alpha=rospy.get_param('~tracker_yaw_alpha', 0.25),
            vehicle_confidence_alpha=rospy.get_param(
                '~vehicle_confidence_alpha', 0.35),
            yaw_confidence_alpha=rospy.get_param('~yaw_confidence_alpha', 0.35),
            yaw_update_min_confidence=rospy.get_param(
                '~yaw_update_min_confidence', 0.35),
            yaw_maturity_hits=rospy.get_param('~yaw_maturity_hits', 3),
            yaw_maturity_confidence=rospy.get_param(
                '~yaw_maturity_confidence', 0.55),
            association_size_weight=rospy.get_param('~association_size_weight', 0.35),
            association_yaw_weight=rospy.get_param('~association_yaw_weight', 0.75),
            association_yaw_min_confidence=rospy.get_param(
                '~association_yaw_min_confidence', 0.55),
            association_feature_min_hits=rospy.get_param(
                '~association_feature_min_hits', 3),
            vehicle_confirm_score=rospy.get_param(
                '~vehicle_confirm_score', 0.60),
            vehicle_confirm_hits=rospy.get_param(
                '~vehicle_confirm_hits', 4),
            vehicle_release_score=rospy.get_param(
                '~vehicle_release_score', 0.25),
            vehicle_release_frames=rospy.get_param(
                '~vehicle_release_frames', 7),
            motion_velocity_coast_sec=rospy.get_param(
                '~motion_velocity_coast_sec', 0.6),
            motion_velocity_coast_std_growth=rospy.get_param(
                '~motion_velocity_coast_std_growth', 1.5))

        # ---- 정적/동적 분류 (강건화: 거리비례문턱 + 속도방향 일관성 + 자차동행 게이트 + 지속성) ----
        # 정적물이 동적으로 오분류되던 3원인을 겨냥:
        #   [A] ego yaw오차×거리 -> 거리비례 문턱(먼 물체는 더 빨라야 동적)
        #   [A][B] yaw지터/GPS점프 -> 속도방향 일관성(무작위 지터는 합성길이 낮음)
        #   [C] 벽/건물 중심슬라이딩 -> '자차 동행'(겉보기속도≈ego속도벡터) 게이트
        self.dyn_on_speed = rospy.get_param('~dynamic_on_speed', 1.0)
        self.dyn_off_speed = rospy.get_param('~dynamic_off_speed', 0.5)
        self.dyn_on_per_m = rospy.get_param('~dynamic_on_per_m', 0.05)   # 거리비례 가산 [m/s per m]
        self.dyn_persist_up = rospy.get_param('~dynamic_persist_up', 2)  # 동적 확정 연속프레임
        self.dyn_persist_down = rospy.get_param('~dynamic_persist_down', 3)
        self.dyn_vhist_len = rospy.get_param('~dynamic_vhist_len', 6)
        self.dyn_vconsistency_min = rospy.get_param('~dynamic_vconsistency_min', 0.6)
        self.dyn_with_ego_eps = rospy.get_param('~dynamic_with_ego_eps', 0.8)  # ego동행 판정 [m/s]
        self._dyn = {}
        self._ego_vx = self._ego_vy = 0.0

        # ---- TTC ----
        self.ttc_range_margin = rospy.get_param('~ttc_range_margin', 0.0)
        self.min_closing_speed = rospy.get_param('~min_closing_speed', 0.1)
        # ★차체 보정: ego 기준점(후축/GPS안테나) -> 앞범퍼 오버행 [m]
        self.ego_front_overhang = rospy.get_param('~ego_front_overhang', 2.0)
        # ★CPA 코리도 게이트: 예측 최근접거리가 (차선반폭 + 이 여유 + 객체반폭) 초과면
        #   비껴갈 물체로 보고 TTC 무효화 -> 옆차선 대향차 오경보 제거
        self.lane_half_width = rospy.get_param('~lane_half_width', 1.75)
        self.cpa_gate_margin = rospy.get_param('~cpa_gate_margin', 0.5)

        self.frame_id = rospy.get_param('~frame_id', 'map')

        # ---- ego 상태 ----
        self.have_ego = False
        self.ego_x = self.ego_y = 0.0
        self.ego_v = 0.0
        self.ego_heading = 0.0
        self.imu_yaw = None
        self.prev_stamp = None
        # 스캔시각 보간에 쓰는 ego 포즈/yaw 링버퍼: (t, x, y), (t, yaw)
        self._pos_buf = deque()
        self._yaw_buf = deque()
        # 이번 프레임 변환/TTC 에 실제로 쓴 스캔시각 ego 포즈(자기일관 유지용)
        self._sx = self._sy = 0.0
        self._syaw = 0.0

        if self.use_ego_topic:
            rospy.Subscriber(self.ego_topic, EgoVehicleStatus, self.ego_cb, queue_size=1)
        else:
            rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber(self.imu_topic, Imu, self.imu_cb, queue_size=1)
        self.obj_pub = rospy.Publisher('/tracked_objects', TrackedObjectArray, queue_size=1)
        self.marker_pub = rospy.Publisher('/tracked_objects_markers', MarkerArray, queue_size=1)

        # ---- 구간 게이팅 (M6/M7 고주로에서만 실행 및 시각화) ----
        self.section_gating_enabled = rospy.get_param('~section_gating_enabled', True)
        self.active_section_msg = None
        self.active_section_t = 0.0
        self.current_lane = -1
        self.current_wp_idx = -1
        try:
            from mission_supervisor.msg import SectionState as _SecState
            rospy.Subscriber('/mission/active_section', _SecState,
                             self.active_section_cb, queue_size=1)
        except Exception:
            rospy.logwarn('[lidar_ttc_tracker] /mission/active_section 구독 생략')

        try:
            from waypoint_maker.msg import Lane as _WpLane
            rospy.Subscriber('/final_waypoint', _WpLane,
                             self.final_waypoint_cb, queue_size=1)
        except Exception:
            pass

        if not _HAVE_SCIPY:
            rospy.logwarn('[lidar_ttc_tracker] scipy 없음 -> numpy fallback 클러스터링/greedy 매칭 사용.')
        pose_src = self.ego_topic if self.use_ego_topic else self.odom_topic
        rospy.loginfo('[lidar_ttc_tracker] cloud=%s pose=%s imu=%s heading=%s latency=%.2fs',
                      self.cloud_topic, pose_src, self.imu_topic, self.heading_source,
                      self.cloud_latency)

        # 2026-10-07: buff_size 가 메시지(터널 /nonground 수 MB)보다 작으면 queue_size=1 이어도 TCP 버퍼에
        #   옛 스캔이 쌓여 2~4 s 묵은 점군을 처리했다(16:18/16:19 터널 백, 근거리 인지 지연 p50 2.5 s).
        rospy.Subscriber(self.cloud_topic, PointCloud2, self.cloud_cb, queue_size=1, buff_size=2 ** 26)

    def active_section_cb(self, msg):
        self.active_section_msg = msg
        self.active_section_t = rospy.Time.now().to_sec()

    def final_waypoint_cb(self, msg):
        if msg.waypoints:
            self.current_wp_idx = int(msg.waypoints[0].waypoint_index)
            self.current_lane = int(getattr(msg.waypoints[0], 'lane_number', -1))

    def _far_fixed_ground_active(self):
        if not self.far_fixed_ground_enabled:
            return False
        if self.active_section_msg is None or \
                (rospy.Time.now().to_sec() - self.active_section_t) >= 1.5:
            return False
        return getattr(self.active_section_msg, 'section_id', '') in self.far_fixed_ground_sections

    def _in_highway_section(self):
        """지금이 고주로(M6/M7, SMPC) 미션 구간인가."""
        if not self.section_gating_enabled:
            return True

        if self.active_section_msg is not None and (rospy.Time.now().to_sec() - self.active_section_t) < 1.5:
            sec_id = getattr(self.active_section_msg, 'section_id', '')
            execs = getattr(self.active_section_msg, 'executors', [])
            exempt = getattr(self.active_section_msg, 'speed_limit_exempt', False)
            if sec_id in ('M6', 'M7') or 'smpc' in execs or exempt:
                return True
            return False

        if self.current_lane in (1, 2, 3):
            return True
        # 구간 정보가 없을 때의 폴백. missions.yaml highway_approach 시작(10500)과 맞춘다.
        if self.current_lane == 0 and self.current_wp_idx >= 10500:
            return True
        return False

    # ================= 링버퍼 =================
    def _push(self, buf, t, val):
        buf.append((t, val))
        cutoff = t - self.pose_buffer_sec
        while len(buf) > 2 and buf[0][0] < cutoff:
            buf.popleft()

    @staticmethod
    def _interp(buf, tq, default, wrap=False):
        """버퍼에서 tq 시각 값을 선형보간. 값은 스칼라(yaw) 또는 (x,y) 튜플 모두 지원.
        범위 밖이면 최근접 clamp, 비면 default."""
        if not buf:
            return default
        if tq <= buf[0][0]:
            return buf[0][1]
        if tq >= buf[-1][0]:
            return buf[-1][1]
        # 이분 아닌 선형 탐색(버퍼 짧음)
        for i in range(1, len(buf)):
            t1, v1 = buf[i]
            if t1 >= tq:
                t0, v0 = buf[i - 1]
                if t1 - t0 < 1e-6:
                    return v1
                a = (tq - t0) / (t1 - t0)
                if isinstance(v0, (tuple, list)):
                    return tuple(v0[k] + a * (v1[k] - v0[k]) for k in range(len(v0)))
                if wrap:
                    return _wrap(v0 + a * _wrap(v1 - v0))
                return v0 + a * (v1 - v0)
        return buf[-1][1]

    # ================= 콜백: ego / imu =================
    def ego_cb(self, msg):
        self.ego_x = msg.position.x
        self.ego_y = msg.position.y
        self.ego_v = msg.velocity.x
        self.ego_heading = math.radians(msg.heading)
        self.have_ego = True
        t = rospy.Time.now().to_sec()
        self._push(self._pos_buf, t, (self.ego_x, self.ego_y))
        if self.heading_source != 'imu':
            self._push(self._yaw_buf, t, self.ego_heading)

    def odom_cb(self, msg):
        self.ego_x = msg.pose.pose.position.x
        self.ego_y = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        # kcity_localizer twist 는 바디프레임 전방속도(linear.y=0). |v| 로 스칼라만 취함.
        self.ego_v = math.hypot(vx, vy)
        # 헤딩은 orientation(쿼터니언) 우선 — twist 바디프레임 폴백(atan2(0,v)=0) 함정 회피.
        self.ego_heading = quat_to_yaw(msg.pose.pose.orientation)
        self.have_ego = True
        t = msg.header.stamp.to_sec() if msg.header.stamp.to_sec() > 0.0 \
            else rospy.Time.now().to_sec()
        self._push(self._pos_buf, t, (self.ego_x, self.ego_y))
        if self.heading_source != 'imu':
            self._push(self._yaw_buf, t, self.ego_heading)

    def imu_cb(self, msg):
        q = msg.orientation
        self.imu_yaw = quat_to_yaw(q)

        if self.heading_source == 'imu':
            t = msg.header.stamp.to_sec() if msg.header.stamp.to_sec() > 0.0 \
                else rospy.Time.now().to_sec()
            self._push(self._yaw_buf, t, self.imu_yaw)

    def _ego_yaw(self):
        if self.heading_source == 'imu' and self.imu_yaw is not None:
            return self.imu_yaw
        return self.ego_heading

    def _scan_pose(self, stamp):
        """스캔시각(stamp - cloud_latency) 의 ego (x, y, yaw) 를 버퍼에서 보간."""
        tq = stamp.to_sec() - self.cloud_latency
        x, y = self._interp(self._pos_buf, tq, (self.ego_x, self.ego_y))
        yaw = self._interp(self._yaw_buf, tq, self._ego_yaw(), wrap=True)
        return x, y, yaw

    # PointCloud2 버퍼에서 x/y/z 만 numpy 로 바로 뽑는다.
    # 필드 오프셋은 첫 메시지에서 한 번만 확인하고 캐시한다.
    def _parse_xyz(self, msg):
        # 반환: (N,6) = x, y, z, ring, ground_relative, label intensity.
        # GroundGrid 필드가 없으면 마지막 두 열은 NaN이다.
        # ring 은 원거리 지면/차량 판별(_ring_object_mask)에 쓴다. VLP-16 에서
        # 원거리 차량은 링 하나(-1deg)에만 찍히고 지면 위 0.5m 남짓만 올라와서
        # z 문턱만으로는 지면과 못 가른다 - 같은 링 안의 방위각 이웃과 비교해야
        # 한다(실측 2026-09-06 20:17 백: 물체가 56m 부터 보이는데 z 기준
        # 지면제거로 16m 까지 전멸했다).
        key = (msg.point_step, tuple((f.name, f.offset, f.datatype) for f in msg.fields))
        cache = getattr(self, '_xyz_layout', None)
        if cache is None or cache[0] != key:
            off = {}
            for f in msg.fields:
                if f.name in ('x', 'y', 'z'):
                    # 7 = FLOAT32. 다른 타입이면 안전하게 기존 경로로 폴백한다.
                    if f.datatype != 7:
                        self._xyz_layout = (key, None)
                        return self._parse_xyz_fallback(msg)
                    off[f.name] = f.offset
            if len(off) != 3:
                self._xyz_layout = (key, None)
                return self._parse_xyz_fallback(msg)
            # ring 은 보통 UINT16(=4). 없거나 다른 타입이면 그냥 안 쓴다.
            ring_layout = None
            for f in msg.fields:
                if f.name == 'ring' and f.datatype in (2, 4):
                    ring_layout = (f.offset, np.uint8 if f.datatype == 2 else np.uint16)
                    break
            extra = {f.name: (f.offset, f.datatype) for f in msg.fields
                     if f.name in ('ground_relative', 'intensity')}
            self._xyz_layout = (key, (off['x'], off['y'], off['z'], ring_layout,
                                      extra.get('ground_relative'), extra.get('intensity')))
        layout = self._xyz_layout[1]
        if layout is None:
            return self._parse_xyz_fallback(msg)

        ox, oy, oz, ring_layout, relative_layout, intensity_layout = layout
        step = msg.point_step
        n = msg.width * msg.height
        if n == 0:
            return None
        raw = np.frombuffer(msg.data, dtype=np.uint8, count=n * step).reshape(n, step)
        out = np.full((n, 6), np.nan, dtype=np.float32)
        out[:, 0] = raw[:, ox:ox + 4].copy().view(np.float32).ravel()
        out[:, 1] = raw[:, oy:oy + 4].copy().view(np.float32).ravel()
        out[:, 2] = raw[:, oz:oz + 4].copy().view(np.float32).ravel()
        if ring_layout is None:
            out[:, 3] = -1.0
        else:
            roff, rdt = ring_layout
            w = np.dtype(rdt).itemsize
            out[:, 3] = raw[:, roff:roff + w].copy().view(rdt).ravel().astype(np.float32)
        for col, field_layout in ((4, relative_layout), (5, intensity_layout)):
            if field_layout is not None:
                field_offset, datatype = field_layout
                if datatype == 7:  # FLOAT32
                    out[:, col] = raw[:, field_offset:field_offset + 4].copy().view(
                        np.float32).ravel()
        finite = np.isfinite(out[:, :3]).all(axis=1)
        return out[finite]

    def _parse_xyz_fallback(self, msg):
        pts = np.array(list(point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)), dtype=np.float32)
        if not pts.shape[0]:
            return None
        # 상위 경로와 열 수를 맞춘다(ring 없음 = -1).
        out = np.full((pts.shape[0], 6), np.nan, dtype=np.float32)
        out[:, :3] = pts
        out[:, 3] = -1.0
        return out

    # ================= 링 기반 원거리 지면 판별 =================
    def _ring_ground_mask(self, x, y, z, ring):
        """링별 '예상 노면 반경'과 비교해 노면 점을 찾는다.

        왜 필요한가. bin 퍼센타일 방식은 원거리에서 무너진다 - VLP-16 은 노면을
        동심원으로 때리는데 링 사이 x 구간에는 노면 점이 아예 없고, 그 구간에
        물체가 있으면 bin 의 점이 곧 물체뿐이라 '지면 추정치 = 물체 높이' 가
        된다(실측: 물체 z=-0.76, 그 bin 지면추정 -0.76, 차이 0.000).

        링 기반은 그 함정이 없다. 평평한 지면에서 링 k 는 반경
            r = 센서고 / tan(|하향각|)
        의 원을 그린다. 물체가 있으면 그 방위각에서 반경이 '짧아진다'(앞에서
        막히므로). 이웃 점이 없어도 자기 링 기준만 있으면 판정되므로 원거리에서
        성립한다.

        경사 보정. 순수 기하만 쓰면 오르막에서 노면이 통째로 물체가 된다.
        실측(2026-09-08)에서 이론 반경 대비 오차가 ring6 -1.7m, ring7 -11.1m 로
        원거리일수록 커졌다. 그래서 이론값이 아니라 '섹터별로 관측된 노면 반경의
        중앙값'을 기준으로 쓴다. 같은 섹터 안 산포는 작고(ring6 까지 1.2m)
        섹터 간 편차만 크다(ring5 에서 6.2m) - 즉 방위각별로 기준을 따로
        잡아야 한다는 뜻이고, 중앙값은 노면이 다수라서 물체에 끌려가지 않는다.
        """
        n = x.shape[0]
        out = np.zeros(n, dtype=bool)
        if n == 0 or ring is None:
            return out
        valid = ring >= 0
        if not valid.any():
            return out

        r = np.hypot(x, y)
        az = np.degrees(np.arctan2(y, x))
        n_sect = max(4, int(self.ring_ground_sectors))
        sect = np.clip(((az + 180.0) / (360.0 / n_sect)).astype(np.int64),
                       0, n_sect - 1)
        rings = ring.astype(np.int64)
        key = rings * n_sect + sect

        # (링, 섹터)별 반경 중앙값 = 그 방향 노면 반경 추정치.
        # 점이 적은 칸은 물체에 끌려갈 수 있어 기준으로 쓰지 않는다.
        order = np.argsort(key, kind='stable')
        k_sorted = key[order]
        r_sorted = r[order]
        uniq, starts = np.unique(k_sorted, return_index=True)
        ends = np.append(starts[1:], len(k_sorted))
        tol = float(self.ring_ground_tolerance_m)
        min_pts = int(self.ring_ground_min_points)

        for u, a, b in zip(uniq, starts, ends):
            cnt = b - a
            if cnt < min_pts:
                continue
            seg = r_sorted[a:b]
            base = float(np.median(seg))
            # 노면은 기준 반경 근처, 물체는 그보다 뚜렷이 가깝다.
            # 기준보다 먼 점은 노면 결손 너머라 노면으로 본다(물체가 아니다).
            near = np.abs(seg - base) <= tol
            out[order[a:b][near]] = True
        return out

    # ================= 링 연속성 기반 물체 판별 =================
    def _ring_object_mask(self, x, y, z, ring, z_ground_local):
        """같은 링 안에서 노면 연속성을 끊는 점들의 '덩어리' = 물체 후보.

        왜 필요한가. VLP-16 은 원거리에서 차량을 링 하나(-1deg)로만 스친다.
        그 링은 차량 상단이 아니라 범퍼 근처를 때려서 지면 위 0.5m 남짓밖에 안
        올라오고, ground_clearance(0.38m)에 노면 굴곡·추정오차가 겹치면 통째로
        잘린다. 게다가 원거리에서는 물체가 자기 x bin 의 유일한 점이라 그 bin 의
        지면 추정치가 물체 높이 그 자체로 잡힌다(실측: 물체 z=-0.76, 같은 bin
        지면추정 -0.76 -> z-zg=0.000). 즉 z 기반 판정은 원리적으로 이 구간을
        살릴 수 없다. 실측(2026-09-06 20:17 백)에서 물체가 56m 부터 연속으로
        보이는데 트랙 최초 검출은 19.8m 였다 - 36m, 5.8초를 놓쳤다.

        그래서 높이 대신 방위각 방향 거리 연속성만 쓴다. 물체 앞면은 방위각을
        따라 거리가 '평평'하고 양 끝에서만 노면으로 튄다(실측: 물체 내부 11개
        점의 인접 dr 이 0.1m 미만, 가장자리에서 11.0m / 2.6m 점프).
          1) 인접 대비 range_step 이상 가까워지는 가장자리를 씨앗으로 잡고
          2) 거기서 방위각 양방향으로 거리가 완만히(flat_tol) 이어지는 동안 확장

        구현상 함정 두 가지를 피해야 한다.
          - 씨앗 탐색을 지면 문턱으로 미리 거른 점에서 하면, 불연속을 만들어주던
            노면 이웃이 사라져 씨앗이 하나도 안 생긴다.
          - 이 함수는 ROI(|y|<roi_y_half) 로 잘린 뒤의 점을 받는다. 물체 옆
            노면이 잘려나가 방위각이 크게 벌어지면(실측 167deg) 씨앗 판정이
            깨진다. 그래서 az 간격 상한은 '한쪽 이웃만 있어도' 판단할 수 있도록
            양방향 각각 따로 본다.
        """
        n = x.shape[0]
        out = np.zeros(n, dtype=bool)
        if n == 0 or ring is None:
            return out
        has_ring = ring >= 0
        if not has_ring.any():
            return out

        r = np.hypot(x, y)
        az = np.degrees(np.arctan2(y, x))
        idx = np.nonzero(has_ring)[0]
        order = idx[np.lexsort((az[idx], ring[idx]))]
        m = order.shape[0]
        if m < 2:
            return out
        rr = r[order]
        rg = ring[order]
        aa = az[order]

        same = rg[1:] == rg[:-1]
        d_az = np.abs(aa[1:] - aa[:-1])
        pair = same & (d_az <= self.ring_max_az_gap_deg)
        gap = rr[1:] - rr[:-1]

        step = self.ring_range_step_m
        seed = np.zeros(m, dtype=bool)
        seed[1:] |= pair & (gap < -step)
        seed[:-1] |= pair & (gap > step)
        # ROI 절단으로 한쪽 이웃이 사라진 구간: 같은 링인데 az 가 크게 벌어졌고
        # 거리도 크게 줄었다면 그 지점도 물체 앞면 시작으로 본다.
        cut = same & (d_az > self.ring_max_az_gap_deg)
        seed[1:] |= cut & (gap < -step)
        seed[:-1] |= cut & (gap > step)
        if not seed.any():
            return out

        flat = pair & (np.abs(gap) <= self.ring_flat_tol_m)
        grow = seed.copy()
        for _ in range(int(self.ring_max_grow_steps)):
            nxt = grow.copy()
            nxt[1:] |= grow[:-1] & flat
            nxt[:-1] |= grow[1:] & flat
            if np.array_equal(nxt, grow):
                break
            grow = nxt

        sel = order[grow]
        # 근거리는 기존 z 문턱이 이미 잘 잡으므로 건드리지 않는다.
        # 높이 조건은 걸지 않는다 - 위에 적었듯 이 구간에선 지면 추정치가
        # 물체 자신이라 z-zg 가 0 이 되어 무조건 탈락한다. 대신 '덩어리 크기'로
        # 노면 오탐을 막는다: 노면의 우발적 불연속은 한두 점에 그치고, 실제
        # 물체 앞면은 연속된 점 다발을 만든다(실측 11~13개).
        keep = r[sel] > self.ring_min_range
        sel = sel[keep]
        if sel.shape[0] == 0:
            return out
        out[sel] = True

        if self.ring_min_run_points > 1:
            # grow 로 이어진 덩어리별 크기를 세서 작은 것은 버린다.
            pos = np.nonzero(grow)[0]
            if pos.shape[0]:
                brk = np.nonzero(np.diff(pos) != 1)[0]
                starts = np.concatenate(([0], brk + 1))
                ends = np.concatenate((brk + 1, [pos.shape[0]]))
                for s0, e0 in zip(starts, ends):
                    if (e0 - s0) < self.ring_min_run_points:
                        out[order[pos[s0:e0]]] = False
        return out

    # ================= 점군 -> map 좌표 =================
    def _points_map(self, msg, sx, sy, syaw):
        # read_points() 는 점마다 Python 튜플을 만들어 list 로 쌓기 때문에
        # VLP-16(스캔당 약 3만 점, 10Hz)에서 이 한 줄이 CPU 의 대부분을 먹는다.
        # 실측 102% (코어 1개 포화) -> 다른 노드 스케줄링까지 밀렸다.
        # 버퍼를 numpy 로 직접 해석해 같은 x/y/z 를 얻는다.
        pts = self._parse_xyz(msg)
        if pts is None or pts.shape[0] == 0:
            return None
        x, y, z, ring = pts[:, 0], pts[:, 1], pts[:, 2], pts[:, 3]
        ground_relative, ground_label = pts[:, 4], pts[:, 5]
        # ★자차 footprint 제거 (센서 프레임, 회전 무관) — 거리게이트보다 먼저.
        #   min_range 원형 게이트만으론 라이다 뒤쪽 차체(최대 5m 뒤)를 못 지운다.
        ego = (x > self.ego_x_min) & (x < self.ego_x_max) & (np.abs(y) < self.ego_y_half)
        x, y, z, ring = x[~ego], y[~ego], z[~ego], ring[~ego]
        ground_relative, ground_label = ground_relative[~ego], ground_label[~ego]
        if x.shape[0] == 0:
            return None
        r = np.hypot(x, y)
        keep = (r > self.min_range) & (r < self.max_range)
        x, y, z, ring = x[keep], y[keep], z[keep], ring[keep]
        ground_relative, ground_label = ground_relative[keep], ground_label[keep]
        if x.shape[0] == 0:
            return None

        # 차량 기준 3차선 ROI. raw LiDAR 점을 차량 원점으로 옮긴 뒤 자르므로
        # lidar_x/lidar_y 장착 위치와 무관하게 전후방/좌우 범위가 일정하다.
        if self.roi_enabled:
            xv = x + self.lidar_x
            yv = y + self.lidar_y
            roi = ((xv > self.roi_x_min) & (xv < self.roi_x_max) &
                   (np.abs(yv) < self.roi_y_half))
            x, y, z, ring = x[roi], y[roi], z[roi], ring[roi]
            ground_relative, ground_label = ground_relative[roi], ground_label[roi]
            if x.shape[0] == 0:
                return None

        # ---------------- 종방향 구간 분할(Longitudinal Binned) 국소 지면 제거 ----------------
        # 차량 종방향(x) 구간별(bin_size=2.0m) 국소 지면 높이를 추정하여
        # 도로 종단 경사(오르막/내리막 램프) 및 가감속 피칭으로 인한 지면 오탐을 완벽 제거.
        bin_size = max(0.5, float(self.ground_bin_size))
        bins = np.arange(self.roi_x_min, self.roi_x_max + bin_size, bin_size)
        xv = x + self.lidar_x
        bin_idx = np.clip(np.digitize(xv, bins) - 1, 0, len(bins) - 1)

        # 지면 후보 점 필터: 센서 장착 높이 기준 현실적인 노면 범위 내 점만 지면 추정에 사용.
        # 상공의 가늠대/신호등/표지판(z > +1.5m)이 도로 폭을 덮는다고 해서 지면으로 오인되는 치명적 오류 방지
        cand_mask = z < float(getattr(self, 'ground_cand_z_max', -0.30))
        if cand_mask.any():
            z_cand = z[cand_mask]
            bin_cand = bin_idx[cand_mask]
            order = np.lexsort((z_cand, bin_cand))
            sorted_bins = bin_cand[order]
            sorted_z = z_cand[order]

            uniq_bins, idx_starts = np.unique(sorted_bins, return_index=True)
            nominal_ground_z = -float(self.ground_z_below_sensor)
            ground_by_bin = np.full(len(bins), nominal_ground_z, dtype=np.float32)

            idx_ends = np.append(idx_starts[1:], len(sorted_z))
            counts = idx_ends - idx_starts
            p_offset = np.clip((counts * (float(self.ground_pct) / 100.0)).astype(int), 0, counts - 1)
            ground_by_bin[uniq_bins] = sorted_z[idx_starts + p_offset]
        else:
            uniq_bins = np.array([], dtype=int)
            counts = np.array([], dtype=int)
            nominal_ground_z = -float(self.ground_z_below_sensor)
            ground_by_bin = np.full(len(bins), nominal_ground_z, dtype=np.float32)

        # ---- bin 이 지면 기준이 될 자격이 있는지 판정 ----
        # VLP-16 은 노면을 동심원 '링'으로 때린다. 링과 링 사이 x 구간에는 노면
        # 점이 아예 없고(실측 2026-09-07: 링이 5/10/14/19/25/31m, 32m 밖은 전무),
        # 그 구간에 물체가 있으면 bin 의 점이 곧 물체뿐이라 퍼센타일이 물체 높이
        # 그 자체가 된다. 실제로 23m 박스 2개의 고도가 0.03m / 0.00m 로 계산되어
        # 차량성·장애물성 양쪽에서 노면으로 폐기됐다.
        #
        # 판별자는 횡방향 '점유율'이다. 노면 링은 ROI 폭을 연속으로 덮지만
        # 물체는 좁은 덩어리 몇 개에 그친다. 단순한 y 최대-최소 폭은 쓸 수 없다:
        # 같은 거리에 물체가 둘 있으면(실측: 43m 에 차 2대가 6.9m 간격) 폭만으로는
        # 노면처럼 넓어 보여서 지면이 차 지붕 높이로 잡힌다. 자격 없는 bin 은 자기
        # 값을 버리고 양옆 자격 있는 bin 에서 보간한다 - 아래 경사 제한은 오염된
        # 값을 이웃 쪽으로 '당기기만' 할 뿐 버리지는 못해서, 클립 폭(0.3m) 안에
        # 들어오는 30cm 급 장애물을 그대로 통과시킨다.
        y_cell = np.floor((y + self.roi_y_half) /
                          max(self.ground_y_cell, 0.1)).astype(np.int64)
        n_y_cells = int(2.0 * self.roi_y_half / max(self.ground_y_cell, 0.1)) + 2
        cell_key = bin_idx.astype(np.int64) * n_y_cells + np.clip(y_cell, 0, n_y_cells - 1)
        occupied_bins = (np.unique(cell_key) // n_y_cells).astype(np.int64)
        coverage = np.bincount(occupied_bins, minlength=len(bins))[:len(bins)]
        count_by_bin = np.zeros(len(bins), dtype=np.int32)
        count_by_bin[uniq_bins] = counts
        reliable = ((count_by_bin >= self.ground_min_points) &
                    (coverage >= self.ground_min_y_cells))
        if reliable.any():
            all_idx = np.arange(len(bins))
            ground_by_bin = np.interp(all_idx, all_idx[reliable],
                                      ground_by_bin[reliable]).astype(np.float32)

        # 물리적 최대 경사 제한 (최대 15% 등판각)으로 결측 및 오탐 bin 스무딩
        max_dz = 0.15 * bin_size
        for b in range(1, len(bins)):
            ground_by_bin[b] = np.clip(ground_by_bin[b], ground_by_bin[b - 1] - max_dz, ground_by_bin[b - 1] + max_dz)
        for b in range(len(bins) - 2, -1, -1):
            ground_by_bin[b] = np.clip(ground_by_bin[b], ground_by_bin[b + 1] - max_dz, ground_by_bin[b + 1] + max_dz)

        z_ground_local = ground_by_bin[bin_idx]
        is_above_ground = (z > z_ground_local + self.ground_clearance)

        if self.ground_input_mode == 'groundgrid_labeled':
            valid_groundgrid = np.isfinite(ground_relative) & np.isfinite(ground_label)
            if valid_groundgrid.any():
                # Use the shared terrain estimate.  Full labeled input remains
                # available to ring continuity below, which can recover a
                # distant vehicle if sparse returns were conservatively labeled.
                z_ground_local = np.where(valid_groundgrid,
                                          z - ground_relative,
                                          z_ground_local)
                is_above_ground = np.where(valid_groundgrid,
                                           ground_label >= 90.0,
                                           is_above_ground)
            else:
                rospy.logwarn_throttle(1.0,
                    '[lidar_ttc_tracker] groundgrid_labeled input lacks '
                    'ground_relative/intensity fields; using legacy estimate')

        # ---------------- 고주로 원거리 고정 노면(far_fixed_ground_* 주석) ----------------
        if self._far_fixed_ground_active():
            rr = np.hypot(x, y)
            farm = (rr >= self.far_fixed_ground_min_range) & (rr <= self.far_fixed_ground_max_range)
            if farm.any():
                plane = -float(self.ground_z_below_sensor) + self.far_fixed_ground_offset_m
                h_fixed = z - plane
                z_fixed_ground = z - h_fixed
                z_ground_local = np.where(farm, np.minimum(z_ground_local, z_fixed_ground),
                                          z_ground_local)
                is_above_ground = np.where(
                    farm & (h_fixed >= self.far_fixed_ground_min_height), True, is_above_ground)

        # ---------------- 링 연속성 보강 ----------------
        # VLP-16 은 원거리에서 차량을 링 하나(-1deg, ring7)로만 스친다. 그 링은
        # 차량 상단이 아니라 범퍼 근처를 때려서 지면 위 0.5m 남짓밖에 안 올라오고,
        # ground_clearance(0.38m)에 노면 굴곡·추정오차가 겹치면 통째로 잘린다.
        # 실측(2026-09-06 20:17 백): 물체가 56m 부터 연속으로 보이는데 z 기준
        # 지면제거를 통과한 점은 16m 부터였다 - 36m, 5.8초를 놓쳤다.
        #
        # 같은 링 안에서 방위각 순으로 보면 노면은 거리 r 이 완만하게 변하고,
        # 물체는 그 구간에서 r 이 뚝 떨어진다. z 문턱과 달리 지면 높이 추정에
        # 의존하지 않으므로 원거리에서도 성립한다. band 를 대체하지 않고 OR 로
        # 얹어서, 기존 근거리 거동은 그대로 두고 놓치던 것만 되살린다.
        if self.ring_continuity_enabled:
            is_above_ground = is_above_ground | self._ring_object_mask(x, y, z, ring, z_ground_local)

        # ---------------- 링 기반 원거리 노면 제거 ----------------
        # 위 두 판정(bin 퍼센타일 + 링 연속성)은 '물체를 살리는' 쪽이라, 원거리
        # 노면이 같이 올라온다. 실측(2026-09-06 20:34 백): 장애물이 없는데도
        # 트랙이 63개 잡혔고 그중 94% 가 폭 0.16m 짜리 노면 '선' 이었다.
        # 지금은 형상 게이트(far_min_box_width/aspect)로 사후에 막고 있는데,
        # 점 단계에서 노면을 지우는 편이 근본적이다.
        if self.ring_ground_enabled:
            far = np.hypot(x, y) > self.ring_ground_min_range
            ring_ground = self._ring_ground_mask(x, y, z, ring)
            is_above_ground = is_above_ground & ~(far & ring_ground)

        # 천장 상한 엄격 적용: 지면 위 z_max 이하 AND 센서 기준 현실적 차고 상한(z_sensor_ceiling)
        # 상공의 신호등/가늠대/표지판 점이 유입되는 것을 원천 차단
        z_ceiling = float(getattr(self, 'z_sensor_ceiling', 0.90))
        band = is_above_ground & (z < z_ground_local + self.z_max) & (z < z_ceiling)

        x, y, z, z_ground_local = x[band], y[band], z[band], z_ground_local[band]
        if x.shape[0] == 0:
            return None
        xs = x + self.lidar_x
        ys = y + self.lidar_y
        c, s = math.cos(syaw), math.sin(syaw)
        xm = sx + c * xs - s * ys
        ym = sy + s * xs + c * ys
        # 4번째 열은 그 점 아래 지면 높이. 원거리에서는 링이 한 줄만 닿아 박스
        # '높이'(z 범위)가 0 이 되지만 지면 위 '고도'(z - 지면)는 살아 있고,
        # 그것이 노면 잔차와 차량을 가르는 유일한 수직 단서다.
        return np.stack([xm, ym, z, z_ground_local], axis=1)

    @staticmethod
    def _label_connected(mask):
        """8-neighbor connected components fallback for scipy.ndimage.label."""
        h, w = mask.shape
        labeled = np.zeros((h, w), dtype=np.int32)
        label_id = 0
        ys, xs = np.nonzero(mask)

        for sy, sx in zip(ys, xs):
            if labeled[sy, sx] != 0:
                continue

            label_id += 1
            labeled[sy, sx] = label_id
            stack = [(sy, sx)]
            while stack:
                y, x = stack.pop()
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        if dy == 0 and dx == 0:
                            continue
                        ny, nx = y + dy, x + dx
                        if ny < 0 or ny >= h or nx < 0 or nx >= w:
                            continue
                        if mask[ny, nx] == 0 or labeled[ny, nx] != 0:
                            continue
                        labeled[ny, nx] = label_id
                        stack.append((ny, nx))

        return labeled, label_id

    # ================= BEV 연결요소 클러스터링 =================
    def _cluster(self, P):
        if P.shape[0] < self.min_cluster_pts:
            return []
        xy = P[:, :2]
        mn = xy.min(axis=0)
        gx = np.floor((xy[:, 0] - mn[0]) / self.cluster_res).astype(np.int32)
        gy = np.floor((xy[:, 1] - mn[1]) / self.cluster_res).astype(np.int32)
        mask = np.zeros((int(gy.max()) + 1, int(gx.max()) + 1), dtype=np.uint8)
        mask[gy, gx] = 1
        if _HAVE_SCIPY:
            labeled, n = cc_label(mask, structure=np.ones((3, 3), dtype=np.uint8))
        else:
            labeled, n = self._label_connected(mask)
        if n == 0:
            return []
        pt_label = labeled[gy, gx]
        clusters = []
        for lab in range(1, n + 1):
            idx = np.where(pt_label == lab)[0]
            if self.min_cluster_pts <= idx.shape[0] <= self.max_cluster_pts:
                clusters.append(idx)
        return clusters

    # ================= PCA 방향성 OBB =================
    @staticmethod
    def _soft_band_score(value, soft_min, ideal_min, ideal_max, soft_max):
        """0~1 trapezoid score; values in the ideal band get one point.

        Soft bounds deliberately retain partly occluded cars.  A pillar or a
        long wall may match one dimension, but it is very unlikely to satisfy
        all dimensions and aspect ratio at the same time.
        """
        if not np.isfinite(value) or value <= soft_min or value >= soft_max:
            return 0.0
        if ideal_min <= value <= ideal_max:
            return 1.0
        if value < ideal_min:
            return (value - soft_min) / max(ideal_min - soft_min, 1e-6)
        return (soft_max - value) / max(soft_max - ideal_max, 1e-6)

    def _vehicle_likeness(self, length, width, height):
        """OBB geometry based vehicle score, not a semantic class label."""
        if not self.vehicle_score_enabled:
            return 1.0
        length = max(float(length), 0.1)
        width = max(float(width), 0.1)
        height = max(float(height), 0.1)
        if width > length:
            length, width = width, length
        aspect = length / width
        length_score = self._soft_band_score(
            length, self.vehicle_length_soft_min, self.vehicle_length_ideal_min,
            self.vehicle_length_ideal_max, self.vehicle_length_soft_max)
        width_score = self._soft_band_score(
            width, self.vehicle_width_soft_min, self.vehicle_width_ideal_min,
            self.vehicle_width_ideal_max, self.vehicle_width_soft_max)
        height_score = self._soft_band_score(
            height, self.vehicle_height_soft_min, self.vehicle_height_ideal_min,
            self.vehicle_height_ideal_max, self.vehicle_height_soft_max)
        aspect_score = self._soft_band_score(
            aspect, self.vehicle_aspect_soft_min, self.vehicle_aspect_ideal_min,
            self.vehicle_aspect_ideal_max, self.vehicle_aspect_soft_max)
        if height_score <= 0.0:
            return 0.0
        return float(np.clip(0.32 * length_score + 0.28 * width_score +
                             0.22 * height_score + 0.18 * aspect_score,
                             0.0, 1.0))

    def _fit_obb(self, pts):
        xy = pts[:, :2]
        z = pts[:, 2]
        c = xy.mean(axis=0)
        d = xy - c
        try:
            w, v = np.linalg.eigh(np.cov(d.T))
        except Exception:
            return None
        major = v[:, np.argmax(w)]
        eig_major = max(float(np.max(w)), 0.0)
        eig_minor = max(float(np.min(w)), 0.0)
        yaw = math.atan2(major[1], major[0])
        ax1 = np.array([math.cos(yaw), math.sin(yaw)])
        ax2 = np.array([-math.sin(yaw), math.cos(yaw)])
        p1, p2 = d.dot(ax1), d.dot(ax2)
        size_x = float(p1.max() - p1.min())
        size_y = float(p2.max() - p2.min())
        size_z = float(z.max() - z.min())
        cx = c[0] + ax1[0] * (p1.max() + p1.min()) * 0.5 + ax2[0] * (p2.max() + p2.min()) * 0.5
        cy = c[1] + ax1[1] * (p1.max() + p1.min()) * 0.5 + ax2[1] * (p2.max() + p2.min()) * 0.5
        cz = float((z.max() + z.min()) * 0.5)
        # size_x must always be the longitudinal (longer) side.  PCA's
        # eigenvector sign is still ambiguous, but this removes the 90-degree
        # axis swap before temporal yaw filtering.
        axis_swap = size_y > size_x
        if axis_swap:
            yaw = _wrap(yaw + 0.5 * math.pi)
            size_x, size_y = size_y, size_x

        # 지면 위 '고도'. 박스 높이(z 범위)와 달리 링이 한 줄만 닿아도 살아남는다.
        zg = pts[:, 3] if pts.shape[1] > 3 else None
        elevation = float(np.median(z - zg)) if zg is not None else float('nan')

        # 상공 구조물(신호등, 가늠대, 표지판 등) 배제:
        # 1) 센서 기준 중심 높이(cz)가 천장 상한 초과이거나
        # 2) 지면 위 고도(elevation)가 차고 상한(2.1m) 초과인 경우
        z_ceiling = float(getattr(self, 'z_sensor_ceiling', 0.90))
        max_elev = float(getattr(self, 'max_vehicle_elevation', 2.10))
        if cz > z_ceiling or (np.isfinite(elevation) and elevation > max_elev):
            return None

        # cx/cy 는 map(UTM) 절대좌표다 - 자차 기준 거리로 환산해야 한다.
        box_r = math.hypot(cx - self._sx, cy - self._sy)
        far = box_r > self.far_size_gate_range_m

        # 24m 를 넘으면 승용차에 링이 한 줄만 닿아 size_z 가 0 이 된다. 이 구간은
        # 관측량이 3D 박스가 아니라 '뒷면 현' 하나뿐이므로, 근거리 형상 게이트를
        # 완화하는 대신 관측 모델 자체를 바꾼 별도 가설로 판정한다.
        if box_r >= self.single_ring_min_range and size_z <= self.single_ring_max_size_z:
            return self._far_single_ring_detection(
                pts.shape[0], cx, cy, cz, yaw, size_x, elevation, box_r)

        min_h = self.far_min_box_height if far else self.min_box_height
        min_w = self.far_min_box_width if far else self.min_box_width
        if size_x < self.min_box_size or size_x > self.max_box_size:
            return None
        if size_z < min_h:
            return None
        if size_y < min_w:
            return None
        # 0.6m 미만 박스를 납작하다고 버리던 조건을 0.12m로 완화하여
        # 라바콘, 박스 등 저상 장애물이 지면 잔여물로 오인되어 삭제되지 않도록 보존
        if not far and size_z / max(size_x, size_y) < 0.04 and size_z < 0.12:
            return None
        anisotropy = eig_major / max(eig_minor, 1e-8)
        eig_confidence = np.clip(
            (math.log(max(anisotropy, 1.0)) -
             math.log(self.yaw_confidence_anisotropy_min)) /
            max(math.log(self.yaw_confidence_anisotropy_full) -
                math.log(self.yaw_confidence_anisotropy_min), 1e-6),
            0.0, 1.0)
        point_support = np.clip(
            (float(pts.shape[0]) - float(self.min_cluster_pts)) /
            float(self.yaw_confidence_full_points - self.min_cluster_pts),
            0.0, 1.0)
        yaw_confidence = float(np.clip(
            eig_confidence * point_support, 0.0, 1.0))
        if axis_swap:
            yaw_confidence *= self.yaw_confidence_axis_swap_penalty
        length = max(size_x, 0.1)
        width = max(size_y, 0.1)
        height = max(size_z, 0.1)
        return {
            'x': cx, 'y': cy, 'z': cz, 'yaw': yaw,
            'length': length, 'width': width, 'height': height,
            'yaw_confidence': yaw_confidence,
            'vehicle_score': self._vehicle_likeness(length, width, height),
            'obstacle_score': self._obstacle_likeness(
                pts.shape[0], elevation, length, width),
        }

    def _far_single_ring_detection(self, point_count, fx, fy, fz, chord_yaw,
                                   chord, elevation, box_r):
        """단일 링만 닿는 원거리 검출을 '차량 뒷면 현' 가설로 해석한다.

        왜 별도 경로인가. VLP-16(2deg 간격)이
        승용차(1.5m)를 때리는 링은 32m 까지 2개(-3deg,-1deg), 35m 부터 1개(-1deg),
        100m 에서 0개다. -3deg 링이 지면에 닿는 32.2m 를 넘는 순간 높이는
        '작게 관측되는' 것이 아니라 '관측 불가'가 되고,
        남는 관측량은 뒷면을 가로지르는 현 하나뿐이다. 그런데 PCA 축 정렬상 그
        현은 size_x(장축)=차량 폭, size_y(단축)=현 두께(~0.1m)로 들어오므로,
        근거리 기준을 그대로 쓰면 height 도 width 도 0 점이 되어 차량성이 반드시
        0 이 된다(그리고 하류 target_selector 의 0.42 안전 문턱에도 못 올라가
        객체째 폐기된다). 완화가 아니라 관측 모델 교체가 필요한 이유다.

        이 구간에서 실제로 관측 가능한 세 가지만 쓴다.
          - 현 길이 = 차량 폭. 25m 에서 21점, 60m 에서 9점으로 충분히 측정된다.
          - 접선성 = 현이 시선에 수직인가. 차량 뒷면은 수직, 가드레일·차선·노면
            가장자리는 시선과 나란하다.
          - 지면 위 고도. -1deg 링은 25m 1.25m, 40m 0.99m, 60m 0.64m 를 때리고
            노면 잔차는 0 근처다.
        셋 중 하나라도 0 이면 버린다 - 높이를 못 쓰는 만큼 나머지가 유일한
        방어선이라, 폭만 풀었다가 트랙이 5개에서 125개로 터진 전례가 있다.
        """
        if not np.isfinite(elevation) or box_r < 1e-6:
            return None
        z_ceiling = float(getattr(self, 'z_sensor_ceiling', 0.90))
        max_elev = float(getattr(self, 'max_vehicle_elevation', 2.10))
        if fz > z_ceiling or elevation > max_elev:
            return None
        dx, dy = fx - self._sx, fy - self._sy
        los_yaw = math.atan2(dy, dx)
        tangentiality = abs(math.sin(_wrap(chord_yaw - los_yaw)))

        chord_score = self._soft_band_score(
            chord, self.far_chord_soft_min, self.far_chord_ideal_min,
            self.far_chord_ideal_max, self.far_chord_soft_max)
        tangential_score = float(np.clip(
            (tangentiality - self.far_tangential_min) /
            max(self.far_tangential_full - self.far_tangential_min, 1e-6),
            0.0, 1.0))
        elevation_score = self._soft_band_score(
            elevation, self.far_elev_soft_min, self.far_elev_ideal_min,
            self.far_elev_ideal_max, self.far_elev_soft_max)
        if elevation_score <= 0.0:
            # 지면 위로 올라와 있지 않으면 노면 잔차다. 높이를 못 쓰는 구간에서
            # 이것이 첫 번째 방어선이라 차량/장애물 어느 쪽으로도 내보내지 않는다.
            return None

        if chord_score <= 0.0 or tangential_score <= 0.0:
            # 차량 가설 실패. 그렇다고 없는 것은 아니다 - 낙하물·저상 장애물은
            # 현길이가 차량 폭에 한참 못 미치고(실측 2026-09-07: 23m 박스 0.69m /
            # 0.89m) 차량으로 볼 근거가 없지만, 지면 위로 40cm 올라온 실재하는
            # 물체다. 차량성 0 으로 두되 장애물로는 살려 보내 ACC/AEB 가 보수적
            # 장애물로 유지하게 한다.
            #
            # 오탐 폭증(25m 밖 트랙 5 -> 125개)을 만든 것은 '가늘고 길게' 이어지는
            # 노면·차선·가드레일이었으므로, 길이 상한으로 그것들만 배제한다.
            if chord > self.far_obstacle_max_chord:
                return None
            footprint = float(max(chord, self.far_obstacle_min_footprint))
            return {
                'x': fx, 'y': fy, 'z': fz,
                'yaw': los_yaw,
                'length': footprint,
                # 깊이는 관측량이 아니다. 정사각 footprint 가 가장 보수적이다.
                'width': footprint,
                'height': float(max(elevation, self.far_obstacle_min_height)),
                'yaw_confidence': 0.0,
                'vehicle_score': 0.0,
                'obstacle_score': self._obstacle_likeness(
                    point_count, elevation, chord, footprint),
            }

        vehicle_score = float(np.clip(0.45 * chord_score +
                                      0.35 * tangential_score +
                                      0.20 * elevation_score, 0.0, 1.0))

        # 관측된 것은 자차를 향한 앞면뿐이다. 폭만 관측값을 쓰고 길이/높이는
        # 차량 공칭값으로 세우되, 중심을 시선 방향으로 length/2 밀어 앞면 위치를
        # 보존한다. 그래야 _compute_ttc 의 obj_reach(=length/2) 보정과 상쇄되어
        # 차간거리·TTC 가 중심이 아니라 실제로 본 면 기준으로 남는다.
        width = float(np.clip(chord, self.far_vehicle_width_min,
                              self.far_vehicle_width_max))
        length = float(self.far_vehicle_length)
        ux, uy = dx / box_r, dy / box_r
        return {
            'x': fx + ux * 0.5 * length,
            'y': fy + uy * 0.5 * length,
            'z': fz,
            # 진행방향은 관측량이 아니다. 앞차 가설에서는 시선방향이 최선의
            # 사전값이고, yaw_confidence=0 으로 넘겨 트래커의 yaw 워밍업과
            # 플래너의 차선 사전값이 대신 결정하게 한다. PCA 장축을 그대로
            # 쓰면 현을 따라 90도 틀어진 헤딩이 SMPC 예측에 주입된다.
            'yaw': los_yaw,
            'length': length,
            'width': width,
            'height': float(self.far_vehicle_height),
            'yaw_confidence': 0.0,
            'vehicle_score': vehicle_score,
            'obstacle_score': self._obstacle_likeness(
                point_count, elevation, chord, width),
        }

    def _obstacle_likeness(self, point_count, elevation, size_x, size_y):
        """'무언가 실제로 있다'는 증거. 차량성과 독립이고 안전 경로가 이걸 쓴다.

        차량성 점수는 승용차 형상 사전에 맞는지를 보므로 낮은 장애물에는 정당하게
        낮은 값을 준다. 문제는 그 값이 하류에서 '버릴지' 판단에도 그대로 쓰인다는
        점이다 - 20cm 짜리 낙하물은 차가 아닌 게 맞지만 그렇다고 없는 것은 아니다.
        그래서 형상 대신 실재 증거(지면 위 고도·점 수·크기)만 따로 매긴다.
        """
        if np.isfinite(elevation):
            elevation_score = self._soft_band_score(
                elevation, self.obstacle_elev_soft_min,
                self.obstacle_elev_ideal_min, self.obstacle_elev_ideal_max,
                self.obstacle_elev_soft_max)
        else:
            # 이웃 지면셀이 없어 지면 기준을 못 구한 구간. 지면제거가 이미
            # 통과시킨 점들이므로 판단 보류가 아니라 보수적으로 물체로 본다.
            elevation_score = 0.6
        point_score = float(np.clip(
            (float(point_count) - self.min_cluster_pts) /
            max(float(self.obstacle_full_points - self.min_cluster_pts), 1e-6),
            0.0, 1.0))
        extent_score = self._soft_band_score(
            max(float(size_x), float(size_y)), self.obstacle_extent_soft_min,
            self.obstacle_extent_ideal_min, self.obstacle_extent_ideal_max,
            self.obstacle_extent_soft_max)
        return float(np.clip(0.45 * elevation_score + 0.30 * point_score +
                             0.25 * extent_score, 0.0, 1.0))

    # ================= 정적/동적 분류 =================
    def _classify_dynamic(self, track_id, speed, vx, vy, distance):
        """거리비례 문턱 + 속도방향 일관성 + 자차동행 게이트 + 지속성 디바운스로
        정적물의 유령속도(yaw지터·GPS점프·확장물체 중심슬라이딩)를 동적 오분류에서 거른다."""
        st = self._dyn.get(track_id)
        if st is None:
            st = {'state': False, 'up': 0, 'down': 0, 'vh': deque(maxlen=self.dyn_vhist_len)}
            self._dyn[track_id] = st
        st['vh'].append((vx, vy))

        # [A] 거리비례 문턱: 먼 물체는 yaw오차 유령속도 R·Δψ/dt 가 크므로 문턱을 높인다
        on_eff = self.dyn_on_speed + self.dyn_on_per_m * distance
        off_eff = self.dyn_off_speed + self.dyn_on_per_m * distance

        # [A][B] 속도방향 일관성: 최근 속도벡터 합성길이/평균크기 (1=일관, ~0=무작위 지터)
        vh = st['vh']
        rx = sum(v[0] for v in vh)
        ry = sum(v[1] for v in vh)
        mean_mag = sum(math.hypot(v[0], v[1]) for v in vh) / max(len(vh), 1)
        consistency = math.hypot(rx, ry) / len(vh) / max(mean_mag, 1e-6)
        consistent = len(vh) >= 3 and consistency >= self.dyn_vconsistency_min

        # [C] 자차 동행 게이트: 벽 중심슬라이딩은 겉보기속도가 ego 속도벡터와 거의 같다.
        #     그런 트랙은 정적으로 본다(실제 동속 병주차량이어도 접근속도 0이라 무해).
        moves_with_ego = math.hypot(vx - self._ego_vx, vy - self._ego_vy) < self.dyn_with_ego_eps

        eligible = (speed > on_eff) and consistent and (not moves_with_ego)

        if not st['state']:
            st['up'] = st['up'] + 1 if eligible else 0
            if st['up'] >= self.dyn_persist_up:
                st['state'] = True
                st['down'] = 0
        else:
            demote = (speed < off_eff) or (not consistent) or moves_with_ego
            st['down'] = st['down'] + 1 if demote else 0
            if st['down'] >= self.dyn_persist_down:
                st['state'] = False
                st['up'] = 0
        return st['state']

    # ================= TTC =================
    def _compute_ttc(self, obj_x, obj_y, rvx, rvy, size_x, size_y):
        """(거리, 접근속도, TTC). 스캔시각 ego 포즈(self._sx/_sy/_syaw) 기준으로 자기일관.

        - 차체 오버행 + 객체 반길이 보정(중심-중심 거리 낙관 제거)
        - 전방(forward>0) in-path 만 유효: 후방 접근차는 ttc=-1
        - CPA 미스거리 코리도 게이트: 비껴갈 물체(옆차선 대향 등)는 ttc=-1
        - 지연 보상: eff 에서 closing*latency 만큼 당겨 '스캔시각->현재' 편차 보정
        접근 중 아니면 ttc=-1.
        """
        dx = obj_x - self._sx
        dy = obj_y - self._sy
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return 0.0, 0.0, 0.0
        c, s = math.cos(self._syaw), math.sin(self._syaw)
        forward = dx * c + dy * s
        # 전방만 위협 후보 (후방 추종차 제외)
        if forward <= 0.0:
            return dist, 0.0, -1.0
        closing = -(dx * rvx + dy * rvy) / dist
        # CPA 미스거리
        rv = math.hypot(rvx, rvy)
        if rv > 1e-3:
            d_cpa = abs(dx * rvy - dy * rvx) / rv
        else:
            d_cpa = abs(-dx * s + dy * c)   # 상대정지: 현재 횡거리
        corridor = self.lane_half_width + self.cpa_gate_margin + 0.5 * min(size_x, size_y)
        if d_cpa > corridor:
            return dist, closing, -1.0   # 비껴감
        # 차체/객체 보정 + 지연 보상
        obj_reach = 0.5 * max(size_x, size_y)
        eff = dist - self.ego_front_overhang - obj_reach - self.ttc_range_margin
        if closing > self.min_closing_speed:
            eff = eff - closing * self.cloud_latency
            eff = max(eff, 0.0)
            ttc = eff / closing
        else:
            ttc = -1.0
        return dist, closing, ttc

    def _bbox_corners(self, obj):
        half_l = max(obj.size_x, 0.1) * 0.5
        half_w = max(obj.size_y, 0.1) * 0.5
        half_h = max(obj.size_z, 0.1) * 0.5
        c = math.cos(obj.yaw)
        s = math.sin(obj.yaw)
        local_xy = [(half_l, half_w), (half_l, -half_w),
                    (-half_l, -half_w), (-half_l, half_w)]
        bottom, top = [], []
        for lx, ly in local_xy:
            wx = obj.x + c * lx - s * ly
            wy = obj.y + s * lx + c * ly
            bottom.append(Point(x=wx, y=wy, z=obj.z - half_h))
            top.append(Point(x=wx, y=wy, z=obj.z + half_h))
        return bottom, top

    # ================= 메인 콜백 =================
    def cloud_cb(self, msg):
        if not self.have_ego:
            return
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time(0) else rospy.Time.now()
        dt = 0.1
        if self.prev_stamp is not None:
            dt = (stamp - self.prev_stamp).to_sec()
            if dt <= 0.0:
                rospy.logwarn_throttle(2.0, 'Ignoring duplicate/out-of-order LiDAR stamp')
                return
            if dt > 1.0:
                self.tracker.trackers = []
                self._dyn.clear()
                dt = 0.1
        self.prev_stamp = stamp

        # ★스캔시각 ego 포즈/yaw 보간 — 이번 프레임 변환/TTC 전부 이 값으로 자기일관
        self._sx, self._sy, self._syaw = self._scan_pose(stamp)

        if self.section_gating_enabled and not self._in_highway_section():
            # 고주로(M6/M7) 구간 외: 점군 클러스터링 연산 스킵 및 마커 완전 소거
            if hasattr(self, 'tracker') and hasattr(self.tracker, 'trackers') and len(self.tracker.trackers) > 0:
                self.tracker.trackers = []
                self._dyn.clear()
                out = TrackedObjectArray()
                out.header = Header(stamp=stamp, frame_id=self.frame_id)
                out.min_ttc = -1.0
                out.min_ttc_id = 0
                self.obj_pub.publish(out)
            ma = MarkerArray()
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = stamp
            m.action = Marker.DELETEALL
            ma.markers.append(m)
            self.marker_pub.publish(ma)
            return

        # 1) 검출
        detections = []
        P = self._points_map(msg, self._sx, self._sy, self._syaw)
        if P is not None and P.shape[0] >= self.min_cluster_pts:
            for idx in self._cluster(P):
                # 큰 클러스터(자차 바로 옆 버스·트럭 5000+ 점)는 일정 간격으로 솎아
                # 박스·속도 계산에 넣는다. 전부 넣으면 한 프레임 처리가 0.2~0.5 s 로
                # 늘었다(19:58 백 리플레이). 외곽 크기는 거의 그대로다.
                if self.cluster_fit_max_pts > 0 and idx.shape[0] > self.cluster_fit_max_pts:
                    idx = idx[::int(math.ceil(idx.shape[0] / float(self.cluster_fit_max_pts)))]
                det = self._fit_obb(P[idx])
                if det is not None:
                    det["points_world"] = np.asarray(P[idx, :3], dtype=np.float64)
                    detections.append(det)

        # 2) 추적 (ego 속도벡터는 스캔시각 yaw 로 재구성; 동적판정 자차동행 게이트에도 사용)
        ego_vx = self.ego_v * math.cos(self._syaw)
        ego_vy = self.ego_v * math.sin(self._syaw)
        self._ego_vx, self._ego_vy = ego_vx, ego_vy
        tracked = self.tracker.update(detections, dt, ego_vx, ego_vy)

        alive = {t.id for t in tracked}
        self._dyn = {k: v for k, v in self._dyn.items() if k in alive}

        # 3) 분류 + TTC + 메시지
        out = TrackedObjectArray()
        out.header = Header(stamp=stamp, frame_id=self.frame_id)
        out.min_ttc = -1.0
        out.min_ttc_id = 0
        for t in tracked:
            o = TrackedObject()
            o.id = t.id
            o.x, o.y = t.x, t.y
            o.z = 0.5 * t.height
            o.yaw = t.yaw
            o.size_x, o.size_y, o.size_z = t.length, t.width, t.height
            o.vx, o.vy = t.vx, t.vy
            o.speed = t.speed
            o.rvx, o.rvy = t.rvx, t.rvy
            o.velocity_valid = t.velocity_valid
            o.velocity_std = t.velocity_std
            o.observation_age = t.observation_age
            dist_ego = math.hypot(t.x - self._sx, t.y - self._sy)
            o.is_dynamic = self._classify_dynamic(t.id, t.speed, t.vx, t.vy, dist_ego)
            o.vehicle_confidence = t.vehicle_confidence
            o.is_vehicle = t.is_vehicle
            o.obstacle_confidence = t.obstacle_confidence
            o.yaw_confidence = t.yaw_confidence
            o.distance, o.closing_speed, o.ttc = self._compute_ttc(
                t.x, t.y, t.rvx, t.rvy, t.length, t.width)
            o.hits, o.age = t.hits, t.age
            out.objects.append(o)
            # 전방 in-path 접근 객체만 ttc>=0 -> min_ttc 자동으로 안전한 후보만 집계
            if o.ttc >= 0.0 and (out.min_ttc < 0.0 or o.ttc < out.min_ttc):
                out.min_ttc = o.ttc
                out.min_ttc_id = o.id

        self.obj_pub.publish(out)
        self._publish_markers(out)

    # ================= RViz =================
    def _publish_markers(self, arr):
        ma = MarkerArray()
        dele = Marker()
        dele.header.frame_id = self.frame_id
        dele.action = Marker.DELETEALL
        ma.markers.append(dele)
        for o in arr.objects:
            # This is visualization only. Competition config shows confirmed
            # vehicles, persistent far candidates and ordinary unknowns in
            # distinct namespaces/colors; marker_vehicles_only can restore a
            # vehicle-only display without changing /tracked_objects.
            if self.marker_vehicles_only and not o.is_vehicle:
                continue
            range_ego = math.hypot(o.x - self._sx, o.y - self._sy)
            far_candidate = (
                not o.is_vehicle and
                range_ego >= self.marker_far_candidate_min_range and
                o.hits >= self.marker_far_candidate_min_hits and
                o.obstacle_confidence >=
                self.marker_far_candidate_min_obstacle_confidence)
            m = Marker()
            m.header = arr.header
            m.ns = ('track_vehicle_box' if o.is_vehicle else
                    'track_far_candidate_box' if far_candidate else
                    'track_unknown_box')
            m.id = int(o.id)
            m.type = Marker.LINE_LIST
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = 0.12
            if o.is_vehicle:
                m.color.r, m.color.g, m.color.b = 0.2, 0.55, 1.0
            elif far_candidate:
                m.color.r, m.color.g, m.color.b = 1.0, 0.65, 0.05
            elif o.is_dynamic:
                m.color.r, m.color.g, m.color.b = 1.0, 0.15, 0.15
            else:
                m.color.r, m.color.g, m.color.b = 0.55, 0.55, 0.55
            m.color.a = 1.0 if (arr.min_ttc >= 0.0 and o.id == arr.min_ttc_id) else 0.9
            bottom, top = self._bbox_corners(o)
            for idx in range(4):
                nxt = (idx + 1) % 4
                m.points.extend([bottom[idx], bottom[nxt]])
                m.points.extend([top[idx], top[nxt]])
                m.points.extend([bottom[idx], top[idx]])
            m.lifetime = rospy.Duration(0.3)
            ma.markers.append(m)

            txt = Marker()
            txt.header = arr.header
            txt.ns = ('track_vehicle_text' if o.is_vehicle else
                      'track_far_candidate_text' if far_candidate else
                      'track_unknown_text')
            txt.id = int(o.id)
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = o.x + 0.8
            txt.pose.position.y = o.y + 0.8
            txt.pose.position.z = max(o.size_z, 1.0) + 0.8
            txt.scale.z = 0.7
            ttc_str = ('%.1fs' % o.ttc) if o.ttc >= 0.0 else '-'
            rel_speed = math.hypot(o.rvx, o.rvy)
            kind = ('vehicle' if o.is_vehicle else
                    'far-candidate' if far_candidate else 'unknown')
            yaw_state = 'ready' if o.yaw_confidence > 0.0 else 'warmup'
            txt.text = 'ID %d %s %.2f yawQ %.2f\nrel %.1f m/s\nttc %s' % (
                o.id, kind + ' yaw-' + yaw_state, o.vehicle_confidence,
                o.yaw_confidence, rel_speed, ttc_str)
            txt.color.r = txt.color.g = txt.color.b = 1.0
            txt.color.a = 1.0
            txt.lifetime = rospy.Duration(0.3)
            ma.markers.append(txt)

            if o.is_dynamic:
                ar = Marker()
                ar.header = arr.header
                ar.ns = 'track_relvel'
                ar.id = int(o.id)
                ar.type = Marker.ARROW
                ar.action = Marker.ADD
                p0 = Point(x=o.x, y=o.y, z=0.5)
                p1 = Point(x=o.x + o.rvx, y=o.y + o.rvy, z=0.5)
                ar.points = [p0, p1]
                ar.scale.x, ar.scale.y = 0.15, 0.3
                ar.color.r, ar.color.g, ar.color.b, ar.color.a = 1.0, 0.8, 0.0, 0.9
                ar.lifetime = rospy.Duration(0.3)
                ma.markers.append(ar)
        self.marker_pub.publish(ma)


if __name__ == '__main__':
    try:
        LidarTTCTrackerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
