"""2026-10-11 M8 최종 구조(정동적 파이프라인) 회귀 테스트; ROS master 불필요."""
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import sys
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'yolo_test'))
import obstacle_perception_node as near
from obstacle_info_merger import Merger
from test_tunnel_regional_ground import wall


def _now(t):
    return SimpleNamespace(to_sec=lambda: t)


class TunnelWallZoneTests(unittest.TestCase):
    """벽 = 연석형 제한 구역: 경계 너머 물체는 제외, 벽에 붙은 장애물은 유지."""
    def setUp(self):
        m = Merger.__new__(Merger)
        m.route = [(x, 0., 8000 + int(x * 10), 3) for x in np.arange(0., 50., .5)]
        m.tunnel_wall_rows = [[7990., -3.8, 6.9], [8600., -3.8, 6.9]]
        m.tunnel_wall_offset, m.tunnel_wall_offset_stamp = 0., -1e9
        self.m = m

    def box(self, x, y, h=.3):
        return dict(footprint_world=[[x - h, y - h], [x + h, y - h], [x + h, y + h], [x - h, y + h]],
                    position_world=[x, y])

    def test_object_beyond_right_wall_excluded(self):
        self.assertEqual(self.m.outside_tunnel_wall(self.box(10., -4.6), 10.), 'outside_tunnel_wall_right')

    def test_obstacle_touching_wall_kept(self):
        self.assertIsNone(self.m.outside_tunnel_wall(self.box(10., -3.4), 10.))

    def test_dr_drift_offset_applied(self):
        # offset -1.2(실제 차가 계산보다 1.2 m 왼쪽): 실제 횡거리 = 계산 + 1.2.
        #   계산상 -4.6 인 물체는 실제 -3.4(벽 안쪽)이므로 유지한다.
        self.m.tunnel_wall_offset, self.m.tunnel_wall_offset_stamp = -1.2, 10.
        self.assertIsNone(self.m.outside_tunnel_wall(self.box(10., -4.6), 10.))

    def test_open_entrance_has_no_wall(self):
        self.m.tunnel_wall_rows = [[7990., 0., 0.], [8600., 0., 0.]]
        self.assertIsNone(self.m.outside_tunnel_wall(self.box(10., -6.), 10.))


class NearWallShapeTests(unittest.TestCase):
    def setUp(self):
        n = near.ObstaclePerceptionNode.__new__(near.ObstaclePerceptionNode)
        xs = np.arange(0., 50., .1)
        n.path_world = np.c_[xs, np.zeros_like(xs)]
        n.path_idx = (8000 + np.arange(len(xs))).astype(int)
        n.path_lane = 3
        n.scan_pose = (0., 0., 0.)
        n.tunnel_wall_strip_m = .5
        n.tunnel_wall_profile = (np.array([7990., 8600.]), np.array([-3.8, -3.8]), np.array([6.9, 6.9]))
        self.n = n

    def test_long_cluster_along_wall_is_wall(self):
        pts = np.c_[np.arange(5., 15., .2), np.full(50, -3.0)]
        self.assertTrue(self.n._tunnel_near_wall(pts))

    def test_cluster_in_lane_is_not_wall(self):
        pts = np.c_[np.arange(5., 15., .2), np.full(50, -1.0)]
        self.assertFalse(self.n._tunnel_near_wall(pts))


class AvoidanceTargetKickTests(unittest.TestCase):
    """회피 대상으로 잡은 정적 장애물은 엔진 유효 경로가 피해 가면 선감속 제외."""
    def setUp(self):
        n = wall.TunnelWallDetector.__new__(wall.TunnelWallDetector)
        poly = np.array([[10., -.5], [11., -.5], [11., .5], [10., .5]])
        n.kick_static_footprints = [(poly, poly.min(axis=0), poly.max(axis=0))]
        n.kick_static_t = 10.
        n.kick_static_margin = .6
        n.kick_avoid_clear_m = 1.
        n.kick_avoid_path_t = 10.
        n.kick_engine_valid_t = 10.
        self.n = n

    def test_static_avoided_by_valid_path_excluded(self):
        self.n.kick_avoid_path = np.c_[np.arange(0., 30., 1.), np.full(30, 2.5)]
        with patch.object(wall.rospy.Time, 'now', return_value=_now(10.1)):
            self.assertTrue(self.n.avoidance_target(10.5, 0.))

    def test_path_through_object_keeps_kick(self):
        self.n.kick_avoid_path = np.c_[np.arange(0., 30., 1.), np.zeros(30)]
        with patch.object(wall.rospy.Time, 'now', return_value=_now(10.1)):
            self.assertFalse(self.n.avoidance_target(10.5, 0.))

    def test_no_valid_engine_path_keeps_kick(self):
        self.n.kick_avoid_path = np.c_[np.arange(0., 30., 1.), np.full(30, 2.5)]
        self.n.kick_engine_valid_t = 5.
        with patch.object(wall.rospy.Time, 'now', return_value=_now(10.1)):
            self.assertFalse(self.n.avoidance_target(10.5, 0.))


if __name__ == '__main__':
    unittest.main()
