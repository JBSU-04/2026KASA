#!/usr/bin/env python3
"""Offline unit tests: no ROS master, bag playback or control publishers."""
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch

SOURCE = Path(__file__).resolve().parents[1] / 'scripts/obstacle_stop_node.py'
SPEC = importlib.util.spec_from_file_location('background_static_stop_test', SOURCE)
stop = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(stop)


class BackgroundStaticValidationTests(unittest.TestCase):
    def setUp(self):
        self.node = stop.ObstacleStopNode.__new__(stop.ObstacleStopNode)
        n = self.node
        n.tunnel_bg_static_validation_enabled = True
        n.tunnel_bg_static_fresh_sec = 0.3
        n.tunnel_bg_moving_enabled = True
        n.tunnel_bg_moving_timeout = 0.3
        n.tunnel_bg_moving_match_m = 1.5
        n.tunnel_far_static_match_m = 1.8
        n.tunnel_far_static_move_m = 0.8
        n._tunnel_far_static = [(10.0, 0.0, 9.8)]
        n._pos_hist = {6: [(9.0 + i * 0.1, 10.0, 0.02 * (i % 2))
                           for i in range(11)]}
        n._bg_validation_objects = [self.observation()]
        n.stop_zone_min_forward = 0.0
        n.tunnel_stop_max_forward = 25.0
        n.path_x = list(range(-30, 31))
        n.path_y = [0.0] * len(n.path_x)
        n.path_corridor_radius = 1.5
        n.tunnel_path_corridor_radius = 1.5
        n.tunnel_bg_moving_half_width = 0.3
        n.tunnel_approach_max_lateral = 6.0
        n.tunnel_bg_moving_min_toward = 0.3
        n._bg_moving_t = 10.0
        n._bg_motion_history={3:[(9.7,10.8,.3),(9.85,10.8,.15),(10.,10.8,0.)]}
        self.clock = patch.object(stop.rospy.Time, 'now',
                                  return_value=stop.rospy.Time.from_sec(10.0))
        self.clock.start()
        self.addCleanup(self.clock.stop)

    @staticmethod
    def observation(**updates):
        obs = dict(id=6, position_world=[10.0, 0.0], last_seen=10.0,
                   observation_age=0.0, confirmed=True, motion_state='static',
                   stationary_sec=2.0, speed=0.0, source='lidar_only')
        obs.update(updates)
        return obs

    @staticmethod
    def candidate(**updates):
        bg = dict(id=3, x=10.8, y=0.0, bx=10.8, by=0.0,
                  vx=0.6, vy=-0.5, kind='slow')
        bg.update(updates)
        return bg

    def test_static_center_wobble_does_not_stop(self):
        accepted, reason = self.node._bg_validated_moving(self.candidate())
        self.assertFalse(accepted)
        self.assertEqual(reason, 'bg_matched_static_id_6')
        self.assertFalse(self.node._bg_moving_stop(self.candidate())[0])

    def test_reversing_background_velocity_does_not_bypass_static_check(self):
        for vx, vy in [(0.6, -0.5), (-0.6, 0.5)]:
            self.assertFalse(self.node._bg_moving_stop(self.candidate(vx=vx, vy=vy))[0])

    def test_validated_candidate_is_also_required_for_dynamic_promotion(self):
        self.node._bg_moving = [self.candidate()]
        self.assertFalse(self.node._bg_moving_near([10.0, 0.0]))

    def test_static_label_cannot_veto_fast_real_start(self):
        self.node._pos_hist[6] = [(9.7 + i * 0.1, 10.0, i * 0.4)
                                  for i in range(4)]
        self.node._bg_validation_objects[0]['position_world'] = [10.0, 1.2]
        bg = self.candidate(x=10.0, y=1.2, vy=4.0, kind='fast')
        accepted, reason = self.node._bg_validated_moving(bg)
        self.assertTrue(accepted)
        self.assertEqual(reason, 'bg_new_real_motion_id_6')
        self.assertTrue(self.node._bg_moving_stop(bg)[0])

    def test_slow_real_start_overrides_old_static_and_far_anchor(self):
        self.node._pos_hist[6] = [(9.2 + i * 0.1, 10.0, i * 0.04)
                                  for i in range(9)]
        self.node._bg_validation_objects[0]['position_world'] = [10.0, 0.32]
        self.assertTrue(self.node._bg_moving_stop(self.candidate(x=10.0, y=0.32))[0])

    def test_resumed_motion_can_promote_tracker_again(self):
        self.node._pos_hist[6] = [(9.2 + i * 0.1, 10.0, i * 0.04)
                                  for i in range(9)]
        self.node._bg_validation_objects[0]['position_world'] = [10.0, 0.32]
        self.node._bg_moving = [self.candidate(x=10.0, y=0.32)]
        self.assertTrue(self.node._bg_moving_near([10.0, 0.32]))

    def test_single_measurement_jump_is_not_coherent_motion(self):
        self.node._pos_hist[6] = [(9.2 + i * 0.1, 10.0, 0.0)
                                  for i in range(9)]
        self.node._pos_hist[6][-1] = (10.0, 10.0, 1.0)
        self.assertFalse(self.node._coherent_measured_motion(6))

    def test_alternating_real_centers_are_not_coherent_motion(self):
        self.node._pos_hist[6] = [(9.2 + i * 0.1, 10.0, (-1)**i * 0.2)
                                  for i in range(9)]
        self.assertFalse(self.node._coherent_measured_motion(6))

    def test_stale_static_track_cannot_veto_background(self):
        self.node._bg_validation_objects[0]['last_seen'] = 9.5
        self.assertTrue(self.node._bg_moving_stop(self.candidate())[0])

    def test_stale_observation_age_cannot_veto_background(self):
        self.node._bg_validation_objects[0]['observation_age'] = 0.5
        self.assertTrue(self.node._bg_moving_stop(self.candidate())[0])

    def test_static_label_alone_without_history_cannot_veto(self):
        self.node._pos_hist.clear()
        self.assertTrue(self.node._bg_moving_stop(self.candidate())[0])

    def test_far_static_proposal_alone_cannot_veto(self):
        self.node._bg_validation_objects[0]['source'] = 'raw_temporal_far'
        self.assertTrue(self.node._bg_moving_stop(self.candidate())[0])

    def test_new_unmatched_crossing_retains_immediate_stop(self):
        self.node._bg_validation_objects.clear()
        self.assertTrue(self.node._bg_moving_stop(self.candidate())[0])

    def test_ambiguous_nearby_objects_do_not_hide_crossing(self):
        self.node._bg_validation_objects.append(self.observation(
            id=7, position_world=[10.1, 0.0], motion_state='dynamic'))
        self.assertTrue(self.node._bg_moving_stop(self.candidate())[0])

    def test_measured_stillness_without_far_anchor_is_sufficient(self):
        self.node._tunnel_far_static.clear()
        self.assertFalse(self.node._bg_moving_stop(self.candidate())[0])

    def test_disabling_validation_retains_previous_response(self):
        self.node.tunnel_bg_static_validation_enabled = False
        self.assertTrue(self.node._bg_moving_stop(self.candidate())[0])


if __name__ == '__main__':
    unittest.main()
