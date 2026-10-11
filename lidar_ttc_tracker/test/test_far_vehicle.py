#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""단일 링 원거리 차량 가설과 장애물성/차량성 분리 검증.

여기서 쓰는 점군은 VLP-16(2deg 간격, 새 위치의 지면 기준 실측 장착고
1.730m) 기하에서 계산한 값이다. 승용차(1.5m)에 닿는 링 개수는 약 33m까지
2개이고, 그 뒤에는 1개로 줄어든다.
"""
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.realpath(__file__)), '..', 'scripts'))

from lidar_ttc_tracker_node import LidarTTCTrackerNode  # noqa: E402


SENSOR_Z = -1.730         # 지면 (센서 프레임). 2026-09-11 실측값.


def make_tracker():
    """rospy 없이 기하 판정만 시험하기 위해 __init__ 을 건너뛴 인스턴스."""
    t = LidarTTCTrackerNode.__new__(LidarTTCTrackerNode)
    t._sx, t._sy, t._syaw = 0.0, 0.0, 0.0
    t.min_cluster_pts = 3
    t.min_box_size = 0.2
    t.max_box_size = 8.0
    t.min_box_height = 0.12
    t.min_box_width = 0.30
    t.far_size_gate_range_m = 25.0
    t.far_min_box_height = 0.0
    t.far_min_box_width = 0.6
    t.far_max_aspect_ratio = 4.0
    t.yaw_confidence_full_points = 20
    t.yaw_confidence_anisotropy_min = 1.5
    t.yaw_confidence_anisotropy_full = 4.0
    t.yaw_confidence_axis_swap_penalty = 0.25
    t.single_ring_max_size_z = 0.18
    t.single_ring_min_range = 32.0
    t.far_chord_soft_min = 0.8
    t.far_chord_ideal_min = 1.2
    t.far_chord_ideal_max = 2.7
    t.far_chord_soft_max = 3.4
    t.far_tangential_min = 0.50
    t.far_tangential_full = 0.80
    t.far_elev_soft_min = 0.05
    t.far_elev_ideal_min = 0.15
    t.far_elev_ideal_max = 1.8
    t.far_elev_soft_max = 2.4
    t.far_vehicle_width_min = 1.5
    t.far_vehicle_width_max = 2.6
    t.far_vehicle_length = 4.5
    t.far_vehicle_height = 1.5
    t.far_obstacle_max_chord = 2.0
    t.far_obstacle_min_footprint = 0.8
    t.far_obstacle_min_height = 0.3
    t.obstacle_elev_soft_min = 0.03
    t.obstacle_elev_ideal_min = 0.10
    t.obstacle_elev_ideal_max = 3.0
    t.obstacle_elev_soft_max = 4.0
    t.obstacle_full_points = 12
    t.obstacle_extent_soft_min = 0.10
    t.obstacle_extent_ideal_min = 0.25
    t.obstacle_extent_ideal_max = 8.0
    t.obstacle_extent_soft_max = 12.0
    t.vehicle_score_enabled = True
    t.vehicle_length_soft_min = 0.7
    t.vehicle_length_ideal_min = 1.8
    t.vehicle_length_ideal_max = 8.0
    t.vehicle_length_soft_max = 12.0
    t.vehicle_width_soft_min = 0.45
    t.vehicle_width_ideal_min = 1.1
    t.vehicle_width_ideal_max = 3.0
    t.vehicle_width_soft_max = 4.2
    t.vehicle_height_soft_min = 0.25
    t.vehicle_height_ideal_min = 0.65
    t.vehicle_height_ideal_max = 3.2
    t.vehicle_height_soft_max = 4.2
    t.vehicle_aspect_soft_min = 0.85
    t.vehicle_aspect_ideal_min = 1.25
    t.vehicle_aspect_ideal_max = 5.5
    t.vehicle_aspect_soft_max = 9.0
    return t


def ring_elevation(range_m, ring_deg=-1.0):
    """해당 링이 그 거리에서 때리는 지면 위 고도."""
    return 1.730 + range_m * math.tan(math.radians(ring_deg))


def rear_face(range_m, width=1.8, n=21, jitter=0.02):
    """자차 정면 range_m 에 있는 차량 뒷면(시선에 수직인 현)."""
    ys = np.linspace(-0.5 * width, 0.5 * width, n)
    rng = np.random.RandomState(0)
    xs = range_m + rng.uniform(-jitter, jitter, n)
    z = SENSOR_Z + ring_elevation(range_m)
    return np.stack([xs, ys, np.full(n, z), np.full(n, SENSOR_Z)], axis=1)


def radial_line(range_m, length=3.0, n=21, elevation=0.0):
    """가드레일/차선처럼 시선과 나란한 선분."""
    xs = np.linspace(range_m, range_m + length, n)
    rng = np.random.RandomState(1)
    ys = rng.uniform(-0.02, 0.02, n)
    z = SENSOR_Z + elevation
    return np.stack([xs, ys, np.full(n, z), np.full(n, SENSOR_Z)], axis=1)


class FarSingleRingVehicleTest(unittest.TestCase):
    def setUp(self):
        self.t = make_tracker()

    def test_sedan_at_40m_is_a_vehicle_candidate(self):
        """ACC 요구사항: 단일 링 구간 차량이 '차량'으로 인식되어야 한다.

        장착고 1.730m에서 25m는 아직 2링이라 근거리 3D 경로가 담당한다.
        단일 링은 -3deg가 지면에 닿는 약 33.0m 이후다.
        """
        det = self.t._fit_obb(rear_face(40.0))
        self.assertIsNotNone(det, '40m 단일 링 차량이 검출 단계에서 탈락했다')
        # vehicle_confirm_score(0.60) 를 넘어야 is_vehicle 이 확정된다.
        self.assertGreaterEqual(det['vehicle_score'], 0.60)
        # 하류 target_selector 의 min_object_* 필터를 통과할 박스여야 한다.
        self.assertGreaterEqual(det['height'], 0.20)
        self.assertGreaterEqual(det['width'], 0.4)
        self.assertGreater(det['length'], det['width'])

    def test_far_vehicle_box_preserves_the_observed_face(self):
        """합성 박스 중심 - length/2 가 실제로 본 면 거리와 같아야 ACC 간격이 맞다."""
        det = self.t._fit_obb(rear_face(40.0))
        self.assertIsNotNone(det)
        center_r = math.hypot(det['x'], det['y'])
        face_r = center_r - 0.5 * det['length']
        self.assertAlmostEqual(face_r, 40.0, delta=0.1)

    def test_far_vehicle_reports_unknown_heading(self):
        """현을 따라 90도 틀어진 PCA yaw 가 SMPC 예측에 주입되면 안 된다."""
        det = self.t._fit_obb(rear_face(40.0))
        self.assertIsNotNone(det)
        self.assertEqual(det['yaw_confidence'], 0.0)

    def test_detected_across_the_acc_range(self):
        # 단일 링 구간(33.0m 이후)만. 그 안쪽은 근거리 3D 경로가 담당한다.
        for r in (35.0, 40.0, 50.0, 60.0):
            det = self.t._fit_obb(rear_face(r))
            self.assertIsNotNone(det, '{}m 에서 차량을 놓쳤다'.format(r))
            self.assertGreaterEqual(det['vehicle_score'], 0.60,
                                    '{}m 차량성 부족'.format(r))

    def test_partial_far_vehicle_chord_is_a_candidate(self):
        """A partially visible 0.9 m rear face must survive at long range."""
        det = self.t._fit_obb(rear_face(45.0, width=0.9, n=11))
        self.assertIsNotNone(det)
        self.assertGreaterEqual(det['vehicle_score'], 0.50)

    def test_no_gap_across_the_single_ring_onset(self):
        """-3deg 링이 지면에 닿는 33.0m 전후로 검출이 끊기지 않아야 한다.

        근거리 3D 경로와 단일 링 가설의 경계라, 어느 쪽도 담당하지 않는
        거리 구멍이 생기기 쉽다.
        """
        for r in (33.0, 34.0, 35.0):
            det = self.t._fit_obb(rear_face(r))
            self.assertIsNotNone(det, '{}m 구멍이 남아 있다'.format(r))

    def test_far_low_obstacle_survives_as_obstacle_not_vehicle(self):
        """차량 가설에 실패해도 지면 위 물체는 장애물로 살려 보낸다.

        실측(2026-09-07 MORAI): 노면 박스가 현길이 0.69m / 0.89m, 지면 위
        0.43m / 0.41m 로 관측됐다. 차량 폭(1.5m+)에 한참 못 미쳐 차량은 아니지만
        ACC/AEB 가 봐야 하는 실재 장애물이다. 단일 링 구간(33.0m 이후)에서
        차량 가설이 실패해도 장애물로는 남아야 한다.
        """
        pts = rear_face(36.0, width=0.7, n=10)
        pts[:, 2] = SENSOR_Z + 0.43     # 지면 위 43cm
        det = self.t._fit_obb(pts)
        self.assertIsNotNone(det, '저상 장애물이 원거리 경로에서 폐기됐다')
        self.assertEqual(det['vehicle_score'], 0.0)
        self.assertGreaterEqual(det['obstacle_score'], 0.42)
        # 하류 target_selector 의 min_object_length/width(0.7/0.4) 통과 필요.
        self.assertGreaterEqual(det['length'], 0.7)
        self.assertGreaterEqual(det['width'], 0.4)

    def test_long_radial_line_is_not_rescued_as_obstacle(self):
        """장애물 경로가 가드레일까지 되살리면 안 된다(오탐 5->125 전례)."""
        self.assertIsNone(self.t._fit_obb(radial_line(40.0, length=3.0, elevation=0.8)))

    def test_radial_structure_is_rejected(self):
        """가드레일·차선처럼 시선과 나란한 선은 버린다(오탐 폭증 방어선)."""
        self.assertIsNone(self.t._fit_obb(radial_line(40.0, elevation=0.8)))

    def test_ground_residue_is_rejected(self):
        """지면 위 고도가 없는 노면 잔차는 접선이어도 버린다."""
        pts = rear_face(30.0)
        pts[:, 2] = SENSOR_Z + 0.01
        self.assertIsNone(self.t._fit_obb(pts))

    def test_near_range_vehicle_is_unchanged(self):
        """23m 이하 2링 차량은 기존 3D 박스 경로를 그대로 탄다.

        실차 뒷면은 평면이 아니다. -3deg 링은 범퍼를, -1deg 링은 그보다 안쪽인
        트렁크 면을 때리므로 두 링 사이에 종방향 깊이가 생긴다(min_box_width 가
        요구하는 것이 이 깊이다).
        """
        upper = rear_face(20.0)
        lower = rear_face(19.6)          # 범퍼가 0.4m 앞으로 나와 있다
        lower[:, 2] = SENSOR_Z + ring_elevation(20.0, -3.0)
        det = self.t._fit_obb(np.vstack([upper, lower]))
        self.assertIsNotNone(det)
        self.assertGreater(det['yaw_confidence'], 0.0)
        self.assertGreater(det['height'], 0.5)


class ObstacleVsVehicleTest(unittest.TestCase):
    def setUp(self):
        self.t = make_tracker()

    def test_low_obstacle_is_not_a_vehicle_but_is_an_obstacle(self):
        """낮은 장애물은 '차 모양이 아닌' 것이지 '없는' 것이 아니다.

        지면제거가 지면+ground_clearance(0.38) 아래를 이미 지우므로, 55cm 짜리
        장애물이 _fit_obb 까지 들고 오는 높이는 0.17m 뿐이다. 예전에는 그 값이
        min_box_height(0.20) 에 걸려 검출조차 안 됐고, 통과했더라도 차량성 0 이
        하류 0.42 문턱에서 객체째 폐기시켰다.
        """
        # 15m 앞 0.5m x 0.5m, 높이 55cm 짜리 낙하물. 15m 에서 방위각 분해능은
        # 5cm 라 이 크기면 면당 여러 점이 찍힌다.
        grid = np.linspace(-0.25, 0.25, 4)
        xs, ys = np.meshgrid(15.0 + grid, grid)
        xs, ys = xs.ravel(), ys.ravel()
        zs = SENSOR_Z + np.linspace(0.38, 0.55, xs.size)
        pts = np.stack([xs, ys, zs, np.full(xs.size, SENSOR_Z)], axis=1)

        det = self.t._fit_obb(pts)
        self.assertIsNotNone(det, '낮은 장애물이 검출 단계에서 사라졌다')
        # 차량은 아니다.
        self.assertLess(det['vehicle_score'], 0.60)
        # 그러나 tracked_unknown_obstacle_min_confidence(0.42) 는 넘어야
        # ACC/AEB 가 보수적 장애물로 유지한다.
        self.assertGreaterEqual(det['obstacle_score'], 0.42)

    def test_ground_residue_scores_low_on_obstacle_likeness(self):
        score = self.t._obstacle_likeness(
            point_count=3, elevation=0.04, size_x=0.15, size_y=0.12)
        self.assertLess(score, 0.42)

    def test_solid_object_scores_high_on_obstacle_likeness(self):
        score = self.t._obstacle_likeness(
            point_count=20, elevation=0.8, size_x=1.8, size_y=1.6)
        self.assertGreaterEqual(score, 0.42)

    def test_unknown_ground_reference_is_treated_conservatively(self):
        """이웃 지면셀이 없어 고도를 못 구한 점은 버리지 않고 물체로 본다."""
        score = self.t._obstacle_likeness(
            point_count=12, elevation=float('nan'), size_x=1.0, size_y=0.8)
        self.assertGreaterEqual(score, 0.42)

    def test_overhead_traffic_light_is_rejected(self):
        """상공 4.0m 에 걸려있는 신호등은 차량이나 장애물로 검출되면 안 된다."""
        # 30m 거리, 폭 1.065m, 고도 4.2m
        pts = rear_face(30.0, width=1.065, n=15)
        pts[:, 2] = SENSOR_Z + 4.2  # 지면 위 4.2m
        det = self.t._fit_obb(pts)
        self.assertIsNone(det, '상공 신호등이 차량 또는 장애물로 오검출되었다')


if __name__ == '__main__':
    unittest.main()
