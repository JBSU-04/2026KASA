#!/usr/bin/env python3
"""Entrance geometry tests without a ROS master, bag replay or control nodes."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch, MagicMock
import numpy as np

spec = importlib.util.spec_from_file_location('entry_wall_test',
    Path(__file__).resolve().parents[1] / 'scripts/tunnel_wall_detector.py')
wall = importlib.util.module_from_spec(spec)
spec.loader.exec_module(wall)


class EntryPreviewTests(unittest.TestCase):
    def setUp(self):
        self.profile = [7300, 0., 0., 7350, -3.5, 7.7,
                        7360, -3.5, 7.4, 7440, -3.3, 6.6]
        for name in ('Publisher', 'Subscriber', 'loginfo', 'logwarn', 'loginfo_throttle'):
            p = patch.object(wall.rospy, name,
                             **({'side_effect': lambda *a, **k: MagicMock()}
                                if name == 'Publisher' else {}))
            p.start(); self.addCleanup(p.stop)
        p = patch.object(wall.rospy, 'get_param', side_effect=lambda k, d=None:
            self.profile if k == '~tunnel_wall_profile' else d)
        p.start(); self.addCleanup(p.stop)
        p = patch.object(wall.rospy.Time, 'now', return_value=wall.rospy.Time.from_sec(10.))
        p.start(); self.addCleanup(p.stop)
        self.n = wall.TunnelWallDetector()
        n = self.n
        n.active_section = 'M7B'; n.section_stamp = 10.
        n.pose = (0., 0., 0.); n.pose_stamp = 10.
        n.route_lane = 3; n.route_stamp = 10.
        n.route = (np.array([[0., 0.], [1., 0.]]),
                   np.array([7100., 7110.]), np.array([[1., 0.], [1., 0.]]))

    def test_approach_gets_known_curb_before_live_detection(self):
        self.assertEqual(self.n.profile_entry_limits(10.)[:3], [1., 3.3, 6.6])
        self.assertFalse(self.n.is_tunnel_zone())  # no early dynamic detector arming

    def test_already_curb_profile_is_not_inset_twice(self):
        self.n.curb_inset = 0.5
        self.assertEqual(self.n.profile_entry_limits(10.)[1], 3.3)

    def test_actual_lateral_pose_is_reflected(self):
        self.n.pose = (0., -1., 0.)
        self.assertEqual(self.n.profile_entry_limits(10.)[1:3], [2.3, 7.6])

    def test_highway_section_does_not_enable_preview(self):
        self.n.active_section = 'M7'
        self.assertIsNone(self.n.profile_entry_limits(10.))

    def test_other_lane_does_not_enable_preview(self):
        self.n.route_lane = 2
        self.assertIsNone(self.n.profile_entry_limits(10.))

    def test_approach_far_from_entry_does_not_enable_preview(self):
        self.n.route[1][:] = [6500., 6510.]
        self.assertIsNone(self.n.profile_entry_limits(10.))

    def test_middle_and_exit_do_not_inherit_entry_corridor(self):
        self.n.route[1][:] = [8000., 8010.]
        self.n.active_section = 'M8'
        self.assertIsNone(self.n.profile_entry_limits(10.))

    def test_stale_pose_route_or_section_cannot_supply_limits(self):
        for member in ('pose_stamp', 'route_stamp', 'section_stamp'):
            setattr(self.n, member, 9.)
            self.assertIsNone(self.n.profile_entry_limits(10.))
            setattr(self.n, member, 10.)

    def test_distant_route_cannot_supply_limits(self):
        self.n.pose = (0., 6., 0.)
        self.assertIsNone(self.n.profile_entry_limits(10.))

    def test_missing_open_profile_is_not_invented(self):
        self.n.wprof[1][:] = 0.
        self.assertIsNone(self.n.profile_entry_limits(10.))

    def test_disabled_preview_retains_previous_behavior(self):
        self.n.entry_preview_enabled = False
        self.assertIsNone(self.n.profile_entry_limits(10.))

    def test_physical_profile_does_not_extend_wall_into_open_entrance(self):
        self.n.physical_boundary_policy = True
        self.assertIsNone(self.n.profile_entry_limits(10.))

    def test_physical_policy_does_not_invent_missing_wall(self):
        self.n.physical_boundary_policy = True
        self.scan((0., -4.3, 0., 7.5, True, False, False, {}))
        self.assertEqual(self.n.limits_pub.publish.call_args[0][0].data[0], 0.)

    def test_physical_policy_does_not_hold_stale_wall_at_opening(self):
        self.n.physical_boundary_policy = True
        self.n.last_limits = (wall.rospy.Time.from_sec(9.9), 0., -4.3, 0., 7.5)
        self.scan((0., 0., 0., 0., False, False, False, {}))
        self.assertEqual(self.n.limits_pub.publish.call_args[0][0].data[0], 0.)

    def scan(self, measurement):
        msg = SimpleNamespace(header=SimpleNamespace(
            stamp=wall.rospy.Time.from_sec(10.), frame_id='velodyne'))
        with patch.object(wall, 'pointcloud2_to_xyz_array', return_value=np.zeros((2, 3))), \
             patch.object(self.n, 'measure_walls', return_value=measurement), \
             patch.object(self.n, 'publish_markers'):
            self.n.lidar_cb(msg)

    def test_callback_publishes_preview_without_fake_yaw_or_offset(self):
        self.scan((0., 0., 0., 0., False, False, False, {}))
        self.assertEqual(self.n.limits_pub.publish.call_args[0][0].data[:3], [1., 3.3, 6.6])
        self.n.wall_yaw_pub.publish.assert_not_called()
        self.assertEqual(self.n.filtered_shift, 0.)

    def test_live_wall_takes_priority_but_approach_has_no_heading_correction(self):
        self.scan((0., -4.3, 0., 7.5, True, True, True, {}))
        self.assertAlmostEqual(self.n.limits_pub.publish.call_args[0][0].data[1], 4.2)
        self.n.wall_yaw_pub.publish.assert_not_called()


if __name__ == '__main__':
    unittest.main()
