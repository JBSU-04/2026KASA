#!/usr/bin/env python3
"""Run with ROS workspace sourced; no ROS master or vehicle connection needed."""
import importlib.util
from pathlib import Path
import unittest
import numpy as np
import rospy

ROOT = Path(__file__).resolve().parents[2]
def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module
p = load(ROOT / 'yolo_test/obstacle_perception_node.py', 'perception')
s = load(ROOT / 'new_lane/scripts/obstacle_stop_node.py', 'stop')
rospy.loginfo_throttle = lambda *args: None

class MotionTests(unittest.TestCase):
    def test_utm_centimeter_precision(self):
        pose = (302535., 4124110., 0.7)
        point = np.array([10.02, 0.03])
        np.testing.assert_allclose(p.world_to_ego(p.ego_to_world(point, pose), pose), point, atol=1e-8)

    def test_fast_crossing_before_kalman_converges(self):
        track = p.ObstacleTrack('obstacle', [302535.,4124110.], -1.2, 0.,
                                min_motion_baseline_sec=.15)
        for i in range(1, 4):
            track.update([302535.+.5*i,4124110.], -1.2, i*.1, dist_to_ego=12.)
        self.assertTrue(track.observed_motion_valid)
        self.assertFalse(track.velocity_valid)
        self.assertEqual(track.motion_state, 'unknown')
        self.assertAlmostEqual(track.observed_velocity[0], 5., places=5)

    def test_alternating_jitter_is_not_crossing(self):
        track = p.ObstacleTrack('obstacle', [302535.,4124110.], 0., 0.)
        for i, dx in enumerate([.35,-.3,.3], 1):
            track.update([302535.+dx,4124110.], 0., i*.1, dist_to_ego=12.)
        self.assertFalse(track.observed_motion_valid)

    def test_yaw_interpolation_wrap(self):
        node = p.ObstaclePerceptionNode.__new__(p.ObstaclePerceptionNode)
        node.pose_history = [(1.,(0.,0.,np.deg2rad(179))), (2.,(2.,0.,np.deg2rad(-179)))]
        node.pose_max_age = .15
        pose = node._pose_at(1.5)
        self.assertAlmostEqual(pose[0], 1.)
        self.assertAlmostEqual(abs(pose[2]), np.pi)
        self.assertIsNone(node._pose_at(2.3))

    def test_dynamic_stop_rejects_stale_motion(self):
        node = s.ObstacleStopNode.__new__(s.ObstacleStopNode)
        node.motion_fresh_sec=.3
        node.dynamic_memory_sec=0.0
        node.predictive_risk={'enabled': False, 'fast_lateral_min': 1.2}
        node._dynamic_seen={}
        node._dynamic_evidence_hits={}
        obj={'id': 1, 'position_ego':[5.,0.,-1.],
             'observation_age':.05, 'motion_state':'dynamic',
             'ego_y_rate_signed':0., 'velocity':[0.,3.],
             'velocity_valid':True, 'observed_motion_valid':False,
             'dynamic_streak':2, 'speed':3.0, 'stationary_sec':0.0}
        stop, _ = node._dynamic_zone_stop(obj, True, True, True, 1.0)
        self.assertTrue(stop)
        obj['observation_age']=.5
        stop, _ = node._dynamic_zone_stop(obj, True, True, True, 2.0)
        self.assertFalse(stop)

    def test_single_box_jump_does_not_create_dynamic_stop(self):
        node = s.ObstacleStopNode.__new__(s.ObstacleStopNode)
        node.motion_fresh_sec = 1.0
        node.dynamic_memory_sec = 2.0
        node.predictive_risk = {'enabled': False, 'fast_lateral_min': 1.2,
                                'slow_lateral_min': 0.25}
        node._dynamic_seen = {}
        node._dynamic_evidence_hits = {}
        obj = {'id': 37, 'position_ego': [13.4, -0.7, -0.8],
               'observation_age': 0.1, 'motion_state': 'unknown',
               'ego_y_rate_signed': 2.77, 'speed': 5.97,
               'velocity_valid': False, 'observed_motion_valid': False,
               'dynamic_streak': 0, 'stationary_sec': 0.0}
        stop, _ = node._dynamic_zone_stop(obj, True, True, True, 1.0)
        self.assertFalse(stop)

    def test_stationary_evidence_clears_dynamic_memory(self):
        node = s.ObstacleStopNode.__new__(s.ObstacleStopNode)
        node.motion_fresh_sec = 1.0
        node.dynamic_memory_sec = 2.0
        node.predictive_risk = {'enabled': False, 'fast_lateral_min': 1.2,
                                'slow_lateral_min': 0.25}
        node._dynamic_seen = {37: 1.0}
        node._dynamic_evidence_hits = {37: 3}
        obj = {'id': 37, 'position_ego': [4.0, -1.4, -0.8],
               'observation_age': 0.1, 'motion_state': 'static',
               'ego_y_rate_signed': 0.1, 'speed': 0.1,
               'velocity_valid': True, 'observed_motion_valid': False,
               'dynamic_streak': 0, 'stationary_sec': 3.0}
        stop, _ = node._dynamic_zone_stop(obj, True, True, True, 1.1)
        self.assertFalse(stop)
        self.assertNotIn(37, node._dynamic_seen)

    def test_coherent_crossing_requires_two_observations(self):
        node = s.ObstacleStopNode.__new__(s.ObstacleStopNode)
        node.motion_fresh_sec = 1.0
        node.dynamic_memory_sec = 2.0
        node.predictive_risk = {'enabled': False, 'fast_lateral_min': 1.2,
                                'slow_lateral_min': 0.25}
        node._dynamic_seen = {}
        node._dynamic_evidence_hits = {}
        obj = {'id': 7, 'position_ego': [10.0, 0.5, -0.8],
               'observation_age': 0.1, 'motion_state': 'unknown',
               'ego_y_rate_signed': -1.5, 'speed': 2.0,
               'velocity_valid': False, 'observed_motion_valid': True,
               'dynamic_streak': 0, 'stationary_sec': 0.0}
        first, _ = node._dynamic_zone_stop(obj, True, True, True, 1.0)
        second, _ = node._dynamic_zone_stop(obj, True, True, True, 1.1)
        self.assertFalse(first)
        self.assertTrue(second)

    def test_republished_motion_sample_counts_only_once(self):
        node = s.ObstacleStopNode.__new__(s.ObstacleStopNode)
        node.motion_fresh_sec = 1.0
        node.dynamic_memory_sec = 2.0
        node.predictive_risk = {'enabled': False, 'fast_lateral_min': 1.2,
                                'slow_lateral_min': 0.25}
        node._dynamic_seen = {}
        node._dynamic_evidence_hits = {}
        node._dynamic_evidence_last_seen = {}
        obj = {'id': 32, 'position_ego': [4.2, -1.36, -1.35],
               'observation_age': 0.14, 'last_seen': 100.0,
               'motion_state': 'unknown', 'ego_y_rate_signed': -0.337,
               'speed': 0.944, 'velocity_valid': False,
               'observed_motion_valid': True, 'dynamic_streak': 0,
               'stationary_sec': 0.0}
        first, _ = node._dynamic_zone_stop(obj, True, True, True, 100.14)
        obj['observation_age'] = 0.32  # same LiDAR observation, republished
        repeated, _ = node._dynamic_zone_stop(obj, True, True, True, 100.32)
        self.assertFalse(first)
        self.assertFalse(repeated)
        obj['last_seen'] = 100.4       # a genuinely new confirming scan
        obj['observation_age'] = 0.05
        confirmed, _ = node._dynamic_zone_stop(obj, True, True, True, 100.45)
        self.assertTrue(confirmed)

if __name__ == '__main__':
    unittest.main()
