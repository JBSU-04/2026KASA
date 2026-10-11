"""No ROS master: separate classification, braking, and planner ownership."""
import importlib.util
from pathlib import Path
import unittest
import numpy as np
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('driven_stop', Path(__file__).resolve().parents[1]/'scripts/obstacle_stop_node.py')
stop = importlib.util.module_from_spec(spec); spec.loader.exec_module(stop)

class DrivenStopTests(unittest.TestCase):
    def setUp(self):
        n = self.n = stop.ObstacleStopNode.__new__(stop.ObstacleStopNode)
        n.local_path_xy = np.array([[0., 2.4], [20., 2.4]])
        n.local_path_time = 10.
        n._pos_hist = {48: [(9.+i*.1, 10., 0.) for i in range(11)]}
        n._dynamic_seen = {}; n._dyn_hyst = {}; n._tunnel_dynamic_seen = {}
        n._ever_dyn = {48: 9.}; n._frame_dyn = set(); n._frame_static = set()
        n._dynamic_evidence_hits = {}; n._dynamic_evidence_last_seen = {}
        n.motion_fresh_sec = .3; n.dynamic_memory_sec = 2.
        n.tunnel_dyn_on_speed = 1.; n.tunnel_dyn_off_speed = .6
        n.tunnel_dyn_off_sec = 1.; n.tunnel_dyn_on_hits = 3
        n.tunnel_dyn_gate_span = .5; n.tunnel_dyn_min_world_speed = .6
        n.tunnel_dynamic_restatic_sec = 2.; n.tunnel_static_confirm_sec = .8
        n.tunnel_static_avoid_enabled = True; n.tunnel_far_static_inherit = False
        n.tunnel_unknown_stop_distance = 10.; n.tunnel_stale_track_sec = 1.
        n.stop_zone_max_forward = 18.; n.tunnel_far_dyn_confirm_sec = .5
        n.handoff_guard_front_overhang = 3.85
        n.tunnel_path_corridor_radius = 1.8; n.path_corridor_radius = 1.65
        n._far_static_match = lambda o: False
        n._world_fast_motion = lambda oid, yaw: None
        n._bg_moving_near = lambda pw: False
        n.predictive_risk = {'enabled': False}
        n.path_x = [0.,20.]; n.path_y = [0.,0.]
        n.tunnel_predict_max_forward = 25.; n.stop_zone_min_forward = 0.
        n._approaching_path = lambda *a, **k: (False, 'not_approaching')
        self.o = dict(id=48, position_world=[9.88, 0.], position_ego=[9.88, -4.08, -1.3],
                      width=.54, length=.4, motion_state='unknown', speed=.825,
                      stationary_sec=.1, last_seen=10., observation_age=.05, confirmed=True)
    def decide(self):
        return self.n._dynamic_zone_stop(self.o, True, True, True, 10., tunnel=True)
    def test_segment_distance_not_sparse_vertices(self):
        self.assertAlmostEqual(self.n.driven_path_edge(self.o, 10.), 2.13)
    def test_latest_id48_avoids_without_reference_path_restop(self):
        self.assertFalse(self.decide()[0])
        self.assertIn(48, self.n._frame_static)
        self.assertEqual(self.n._ever_dyn, {48: 9.})
    def test_stale_short_missing_paths_keep_unknown_stop(self):
        self.n._pos_hist = {}
        for path, time in [(None, 10.), (np.array([[0.,2.4],[5.,2.4]]),10.),
                           (np.array([[0.,2.4],[20.,2.4]]),9.)]:
            self.n.local_path_xy, self.n.local_path_time = path, time
            self.assertTrue(self.decide()[0])
    def test_in_path_new_unknown_retains_stop(self):
        self.n._pos_hist = {}; self.n.local_path_xy[:,1] = 0.
        self.assertTrue(self.decide()[0])
    def test_close_body_risk_even_if_path_clear(self):
        self.n._pos_hist = {}; self.o['position_ego'] = [3., 0., -.5]
        self.assertTrue(self.decide()[0])
    def test_dynamic_reference_safeguard_survives_clear_path(self):
        self.n._pos_hist = {}; self.n._dyn_hyst = {48: dict(on=True, low_since=None, hits=3, on_since=9.)}
        self.o.update(motion_state='dynamic', speed=2., velocity_valid=True)
        self.assertTrue(self.decide()[0])
    def test_dynamic_approaches_actual_avoidance_path(self):
        self.n._pos_hist = {}; self.n._dyn_hyst = {48: dict(on=True, low_since=None, hits=3, on_since=9.)}
        self.o.update(motion_state='dynamic', speed=2., velocity_valid=True, velocity=[0.,2.])
        result=self.n._dynamic_zone_stop(self.o,False,False,True,10.,tunnel=True)
        self.assertTrue(result[0])
    def test_known_static_body_proximity_still_stops(self):
        self.o.update(position_ego=[3.,0.,-.5],motion_state='static',speed=.1,stationary_sec=5.)
        self.assertTrue(self.decide()[0]);self.assertIn(48,self.n._frame_static)
    def test_classification_independent_of_path(self):
        self.o.update(motion_state='static', speed=.1, stationary_sec=5.)
        self.assertFalse(self.decide()[0]); self.assertIn(48,self.n._frame_static)
        self.n.local_path_xy[:,1] = 0.; self.n._frame_static.clear()
        self.assertFalse(self.decide()[0]); self.assertIn(48,self.n._frame_static)
    def test_other_missions_keep_reference_stop(self):
        self.n.m3_static_handoff_enabled=False
        self.assertTrue(self.n._dynamic_zone_stop(self.o,True,True,True,10.,tunnel=False)[0])
    def test_reset_long_anchor_does_not_erase_measured_stillness(self):
        self.assertTrue(self.n.tunnel_measured_static(self.o,10.))
    def test_missing_or_stale_measurements_not_static(self):
        self.n._pos_hist = {}; self.assertFalse(self.n.tunnel_measured_static(self.o,10.))
        self.o['last_seen'] = 9.; self.assertFalse(self.n.tunnel_measured_static(self.o,10.))
    def test_real_restart_overrides_static(self):
        self.n._pos_hist[48] = [(9.+i*.1,10.+i*.1,0.) for i in range(11)]
        self.assertFalse(self.n.tunnel_measured_static(self.o,10.))
    def test_planner_not_retargeted_to_generated_path(self):
        source=(Path(__file__).resolve().parents[1]/'src/obstacle_avoidance_node.cpp').read_text()
        self.assertIn('double d2p = distToPath(wx, wy);', source)
        self.assertIn('held_trigger_ob_x_ = trigger_ob_x;', source)
        self.assertNotIn('driven_path_edge', source)

if __name__ == '__main__': unittest.main()
