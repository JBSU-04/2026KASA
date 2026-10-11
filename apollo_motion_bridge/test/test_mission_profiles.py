import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
import rospy
from apollo_motion_bridge.mission_profiles import FILTER_KEYS, validate_profiles

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('profile_perception',
    ROOT / 'yolo_test/obstacle_perception_node.py')
perception = importlib.util.module_from_spec(spec)
spec.loader.exec_module(perception)


class ProfilesTest(unittest.TestCase):
    def setUp(self):
        self.patches = [patch.object(rospy, name) for name in
            ('init_node', 'Subscriber', 'Publisher', 'spin', 'loginfo',
             'logwarn', 'loginfo_throttle')]
        self.patches.append(patch.object(rospy, 'get_param',
            side_effect=lambda key, default=None: default))
        for p in self.patches:
            p.start()
            self.addCleanup(p.stop)
        self.node = perception.ObstaclePerceptionNode()
        self.node.scan_pose = (0., 0., 0.)
        self.node.latest_ego_pose = self.node.scan_pose
        self.node.mission_zone = 'tunnel'
        self.node.use_path_corridor = True
        self.node._path_lateral_dist = lambda xy: np.abs(xy[:, 1])
        self.node.collision_corridor_half_width = 8.
        self.node.use_ground_relative = False
        self.node.ground_z = -1.7
        self.node.align_filter_enabled = False

    def test_profile_fields_exist_and_no_ground_overrides(self):
        for key in FILTER_KEYS:
            self.assertTrue(hasattr(self.node, key), key)
        with self.assertRaises(ValueError):
            validate_profiles({'tunnel': {'ground_z': -1.7}})

    def test_wall_is_removed_outside_trigger_corridor(self):
        # A long tunnel wall at y=3 must not enter tracking/collision output.
        p = np.array([[x, 3., z] for x in np.linspace(5., 11., 30)
                      for z in [-1., 0.]])
        clusters = perception.cluster_lidar_points(p, .38, 3, 20000,
                                                    preserve_large=True)
        self.assertTrue(clusters)
        kept = self.node._reject_ground_residue(clusters)
        self.assertFalse(kept)
        self.assertFalse(self.node.boundary_objects)

    def test_transverse_barrier_not_removed_as_wall(self):
        p = np.array([[8., y, z] for y in np.linspace(-3., 3., 30)
                      for z in [-1., 0.]])
        kept = self.node._reject_ground_residue([(p.mean(axis=0), p)])
        self.assertTrue(kept)
        self.assertFalse(self.node.boundary_objects)

    def test_slow_pose_compensated_motion_is_available(self):
        track = perception.ObstacleTrack('obstacle', np.array([10., 4.]), 0., 0.)
        for i in range(1, 7):
            t = i*.1
            track.update(np.array([10., 4.-.5*t]), 0., t)
        self.assertTrue(track.observed_motion_valid)
        np.testing.assert_allclose(track.observed_velocity, [0., -.5], atol=.01)

    def test_static_observations_do_not_manufacture_direction(self):
        track = perception.ObstacleTrack('obstacle', np.array([10., 4.]), 0., 0.)
        for i in range(1, 7):
            track.update(np.array([10., 4.]), 0., i*.1)
        self.assertFalse(track.observed_motion_valid)
