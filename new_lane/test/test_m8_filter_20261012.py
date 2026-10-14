"""2026-10-12 M8 필터 점 정리: 벽 제외 히스테리시스, 자차와 같이 움직이는 긴 벽 트랙, 동적 예측 정지 최소 크기."""
from pathlib import Path
import sys
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'new_lane' / 'scripts'))
sys.path.insert(0, str(ROOT / 'yolo_test'))
from obstacle_info_merger import Merger
from obstacle_geometry import support_subset
import obstacle_stop_node as stop


class WallExclusionHysteresisTests(unittest.TestCase):
    def setUp(self):
        self.m = Merger.__new__(Merger)

    def test_release_needs_half_second_inside(self):
        m = self.m
        self.assertEqual(m.wall_hysteresis(5, 'outside_tunnel_wall_right', 50.8), 'outside_tunnel_wall_right')
        self.assertEqual(m.wall_hysteresis(5, None, 51.0), 'outside_tunnel_wall_right')
        self.assertEqual(m.wall_hysteresis(5, None, 51.3), 'outside_tunnel_wall_right')
        self.assertIsNone(m.wall_hysteresis(5, None, 51.6))

    def test_never_excluded_track_unaffected(self):
        self.assertIsNone(self.m.wall_hysteresis(9, None, 10.))


class CoMovingWallTrackTests(unittest.TestCase):
    def setUp(self):
        m = Merger.__new__(Merger)
        m.pose = (0., 0., 0.)
        m.ego_speed = 5.5
        self.m = m

    def test_long_track_moving_with_ego_is_wall(self):
        o = dict(length=22., width=.8, observed_velocity=[5.4, .2])
        self.assertEqual(self.m.co_moving_wall_track(o), 'tunnel_wall_co_moving_track')

    def test_short_or_crossing_or_slow_object_kept(self):
        self.assertIsNone(self.m.co_moving_wall_track(dict(length=4.5, width=1.8, observed_velocity=[5.4, 0.])))
        self.assertIsNone(self.m.co_moving_wall_track(dict(length=22., width=.8, observed_velocity=[0., 5.4])))
        self.assertIsNone(self.m.co_moving_wall_track(dict(length=22., width=.8, observed_velocity=[2., 0.])))


class WallHuggingCoMovingTests(unittest.TestCase):
    """2026-10-13(21:55:44 백): 벽 밑 0.35 m 조각이 자차 속도로 따라오면 벽 잔상."""
    def setUp(self):
        m = Merger.__new__(Merger)
        m.route = [(x, 0., 8000 + int(x * 10), 3) for x in np.arange(0., 50., .5)]
        m.tunnel_wall_rows = [[7990., -3.8, 6.9], [8600., -3.8, 6.9]]
        m.tunnel_wall_offset, m.tunnel_wall_offset_stamp = 0., -1e9
        m.pose = (0., 0., 0.)
        m.ego_speed = 5.
        self.m = m

    def box(self, x, y, v, h=.175):
        return dict(footprint_world=[[x - h, y - h], [x + h, y - h], [x + h, y + h], [x - h, y + h]],
                    position_world=[x, y], length=.35, width=.35, observed_velocity=v)

    def test_small_wall_base_fragment_moving_with_ego(self):
        self.assertEqual(self.m.co_moving_wall_track(self.box(14., -3.67, [4.6, 0.]), 10.), 'tunnel_wall_co_moving_track')

    def test_vehicle_in_lane_moving_with_ego_kept(self):
        self.assertIsNone(self.m.co_moving_wall_track(self.box(14., -1., [4.6, 0.]), 10.))

    def test_static_object_at_wall_kept(self):
        self.assertIsNone(self.m.co_moving_wall_track(self.box(14., -3.67, [0., 0.]), 10.))


class DynamicStopMinimumTests(unittest.TestCase):
    def setUp(self):
        self.n = stop.ObstacleStopNode.__new__(stop.ObstacleStopNode)

    def test_low_curb_fragment_too_small(self):
        self.assertTrue(self.n.tunnel_dynamic_too_small(dict(top_elevation=.12, length=.3, width=.3)))

    def test_user_low_obstacle_and_person_kept(self):
        self.assertFalse(self.n.tunnel_dynamic_too_small(dict(top_elevation=.45, length=.6, width=.4)))
        self.assertFalse(self.n.tunnel_dynamic_too_small(dict(top_elevation=1.7, length=.5, width=.5)))


class SupportSubsetTests(unittest.TestCase):
    def test_caps_and_keeps_surface(self):
        pts = np.c_[np.random.RandomState(0).uniform(0, 20, (20000, 2)), np.zeros(20000)]
        q = support_subset(pts)
        self.assertLessEqual(len(q), 300)
        self.assertGreater(np.ptp(q[:, 0]), 19.)

    def test_small_cloud_unchanged(self):
        pts = np.array([[0., 0., 0.], [.5, 0., 0.], [1., 0., 0.]])
        self.assertEqual(len(support_subset(pts)), 3)


if __name__ == '__main__':
    unittest.main()
