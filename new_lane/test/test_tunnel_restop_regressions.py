"""M8 re-stop regressions, without a ROS master or bag replay."""
import unittest
from unittest.mock import patch
from pathlib import Path
import numpy as np
import test_tunnel_driven_stop as fixture


class TunnelRestopTests(unittest.TestCase):
    def setUp(self):
        f = fixture.DrivenStopTests(); f.setUp()
        self.n, self.o = f.n, f.o
        self.n.ego_xy = (0., 2.4)
        self.n.ego_pose_time = 10.
        self.n.ego_speed = 2.
        self.n.ego_yaw = 0.
        self.n.ttc_ego_front_m = 3.85
        self.n.stop_zone_width = 2.
        self.n.predictive_risk = dict(horizon=4., margin=.35)
        self.o.update(radius=.25, vel_std=0.)
        self.clock = patch.object(fixture.stop.rospy.Time, 'now')
        self.clock.start().return_value.to_sec.return_value = 10.
        self.addCleanup(self.clock.stop)

    def test_reference_object_does_not_collide_with_offset_driven_path(self):
        self.o['position_world'] = [5., 0.]
        self.assertEqual(self.n._driven_predictive_conflict(self.o, [0.,0.]),
                         (False, 'driven_no_time_overlap'))

    def test_real_crossing_driven_path_keeps_stop(self):
        self.o['position_world'] = [5., 0.]
        risk, reason = self.n._driven_predictive_conflict(self.o, [0.,2.])
        self.assertTrue(risk); self.assertIn('driven_swept_t', reason)

    def test_incomplete_prediction_horizon_falls_back(self):
        self.n.local_path_xy = np.array([[0.,2.4],[6.,2.4]])
        self.assertIsNone(self.n._driven_predictive_conflict(self.o, [0.,0.]))

    def test_stale_pose_or_path_falls_back(self):
        for attr in ('ego_pose_time', 'local_path_time'):
            setattr(self.n, attr, 9.)
            self.assertIsNone(self.n._driven_predictive_conflict(self.o, [0.,0.]))
            setattr(self.n, attr, 10.)

    def test_stale_unknown_cannot_refresh_hold(self):
        self.n._pos_hist = {}; self.n.local_path_xy[:,1] = 0.
        self.o['observation_age'] = .6
        self.assertEqual(self.n._dynamic_zone_stop(self.o, True, True, True, 10., tunnel=True),
                         (False, 'tunnel_stale_unknown_no_refresh'))

    def test_stale_unknown_body_proximity_still_stops(self):
        self.n._pos_hist = {}; self.o.update(observation_age=.6, position_ego=[3.,0.,0.],position_world=[3.,2.4])
        self.assertTrue(self.n._dynamic_zone_stop(self.o, True, True, True, 10., tunnel=True)[0])

    def wall_setup(self):
        n = self.n
        n.path_idx = [7201,7202]; n.tunnel_wall_piece_filter = True
        n.tunnel_wall_piece_margin = .3
        n.tunnel_wall_offset = 0.; n.tunnel_wall_offset_t = 10.
        n._bg_ids = set(); n._bg_ids_t = 10.
        n._wall_profile_at = lambda idx: (0.,0.)
        n.tunnel_limits = (2.,6.); n.tunnel_limits_t = 10.
        self.o.update(position_world=[5.,0.], observed_motion_valid=True,
                      observed_velocity=[-1.4,0.], observation_age=.1)

    def test_along_wall_motion_is_excluded_by_live_limits(self):
        self.wall_setup()
        self.assertTrue(self.n._is_tunnel_wall_piece(self.o))
        self.assertEqual(self.n._wall_reason, 'live_limits')

    def test_inward_crossing_is_not_background_even_with_map_id(self):
        self.wall_setup(); self.o.update(position_world=[5.,-1.], observed_velocity=[0.,1.])
        self.n._bg_ids = {48}
        self.assertFalse(self.n._is_tunnel_wall_piece(self.o))
        self.assertEqual(self.n._wall_reason, 'inward_motion_protected')

    def test_stale_live_limits_do_not_remove_foreground(self):
        self.wall_setup(); self.n.tunnel_limits_t = 9.
        self.assertFalse(self.n._is_tunnel_wall_piece(self.o))

    def fragment_setup(self):
        self.n._pos_hist = {}
        self.n.tunnel_measured_static = lambda o, t: True
        self.parent = dict(self.o, id=47, position_world=[10.,0.], length=2., width=2.,
                           motion_state='static', last_seen=9.9)
        self.o.update(position_world=[11.,0.], last_seen=9.9)

    def test_fragment_requires_two_distinct_scans_and_keeps_geometry(self):
        self.fragment_setup(); objects = [self.o, self.parent]
        self.n._associate_fragments(objects, 9.9)
        self.n._associate_fragments(objects, 9.9)
        self.assertNotIn('_static_parent_id', self.o)
        self.o['last_seen'] = self.parent['last_seen'] = 10.
        self.n._associate_fragments(objects, 10.)
        self.assertEqual(self.o['_static_parent_id'], 47)
        self.assertEqual(len(objects), 2)

    def test_dynamic_parent_never_lends_static_classification(self):
        self.fragment_setup(); self.parent['motion_state'] = 'dynamic'
        for t in (9.9,10.):
            self.o['last_seen'] = self.parent['last_seen'] = t
            self.n._associate_fragments([self.o,self.parent], t)
        self.assertNotIn('_static_parent_id', self.o)

    def test_planner_safety_contract(self):
        source = (Path(__file__).resolve().parents[1]/'src/obstacle_avoidance_node.cpp').read_text()
        self.assertNotIn('ROBOT_RADIUS = near_pass_robot_radius_', source)
        for stage in ('postprocess', 'original', 'clamp', 'previous'):
            self.assertIn('"'+stage+'"', source)
        self.assertIn('path.curb_violation_fraction()+path.tunnel_body_violation_fraction()', source)
        self.assertIn('path.collision_fraction(xs,ys,ROBOT_RADIUS)', source)


if __name__ == '__main__':
    unittest.main()
