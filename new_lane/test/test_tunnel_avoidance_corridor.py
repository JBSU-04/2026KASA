"""Independent M8 driven-path corridor, without ROS master or bag replay."""
import unittest
from unittest.mock import patch
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import test_tunnel_driven_stop as fixture


class AvoidanceCorridorTests(unittest.TestCase):
    def setUp(self):
        f = fixture.DrivenStopTests(); f.setUp()
        self.n, self.o = f.n, f.o
        n = self.n
        n.mission_mode = 'tunnel'; n.current_lane_number = 3
        n.tunnel_path_corridor_radius = 1.65
        n.path_corridor_radius = 1.5
        n.tunnel_avoidance_corridor_radius = 1.3
        n.local_path_xy = np.column_stack((np.arange(0., 20.25, .25), np.zeros(81)))
        n.local_path_s = n.local_path_xy[:,0].copy()
        n.ego_xy = (0.,0.)
        n.handoff_guard_path_timeout = .5
        n.handoff_guard_min_gap = -3.
        n.handoff_guard_trigger_gap = 5.
        n._pos_hist = {}
        self.o.update(width=.5, length=.4, position_ego=[10.,1.65,0.],
                      position_world=[10.,1.65])
        self.clock = patch.object(fixture.stop.rospy.Time, 'now')
        self.clock.start().return_value.to_sec.return_value = 10.
        self.addCleanup(self.clock.stop)

    def edge(self, distance, x=10.):
        self.o.update(position_world=[x,distance+.25], position_ego=[x,distance+.25,0.])

    def decide(self, reference=True):
        return self.n._dynamic_zone_stop(self.o, reference, reference, True, 10., tunnel=True)

    def test_unknown_boundary_uses_edge_without_double_counting_width(self):
        for edge, expected in [(1.299,True),(1.3,True),(1.301,False),(1.4,False)]:
            with self.subTest(edge=edge):
                self.edge(edge)
                self.assertEqual(self.decide()[0], expected)

    def test_only_driven_threshold_changes(self):
        self.edge(1.4)
        self.assertFalse(self.decide()[0])
        self.n.tunnel_avoidance_corridor_radius = 1.5
        self.assertTrue(self.decide()[0])
        self.assertEqual(self.n.corridor_radius(True), 1.65)
        self.assertEqual(self.n.corridor_radius(False), 1.5)

    def test_proximity_guard_uses_actual_body(self):
        self.edge(1.5,x=3.)
        self.n.tunnel_avoidance_corridor_radius = .8
        self.assertFalse(self.decide()[0])
        self.edge(.9,x=3.)
        self.assertEqual(self.decide(), (True,'tunnel_body_proximity'))

    def test_dynamic_reference_protection_survives_clear_avoidance_corridor(self):
        self.edge(1.4)
        self.o.update(motion_state='dynamic',speed=2.,velocity_valid=True,velocity=[0.,0.])
        self.n._dyn_hyst[48] = dict(on=True,low_since=None,hits=3,on_since=9.)
        self.assertEqual(self.decide(), (True,'tunnel_in_path_dynamic'))

    def test_predicted_path_positions_use_new_radius(self):
        self.edge(1.75)
        self.o.update(motion_state='dynamic',speed=2.,velocity_valid=True,
                      velocity=[0.,-.2],dynamic_streak=3)
        self.n._dyn_hyst[48] = dict(on=True,low_since=None,hits=3,on_since=9.)
        self.assertFalse(self.decide(reference=False)[0])
        self.n.tunnel_avoidance_corridor_radius = 1.65
        self.assertTrue(self.decide(reference=False)[0])

    def test_missing_stale_short_path_preserves_reference_fallback(self):
        self.edge(1.4)
        for xy,stamp in [(None,10.),(self.n.local_path_xy,9.),
                         (np.array([[0.,0.],[5.,0.]]),10.)]:
            with self.subTest(stamp=stamp):
                self.n.local_path_xy, self.n.local_path_time = xy,stamp
                self.assertTrue(self.decide()[0])

    def test_handoff_uses_new_radius_but_other_missions_do_not(self):
        self.edge(1.4,x=6.)
        self.assertFalse(self.n.handoff_on_driven_path(self.o)[0])
        self.n.tunnel_avoidance_corridor_radius = 1.5
        self.assertTrue(self.n.handoff_on_driven_path(self.o)[0])
        self.n.tunnel_avoidance_corridor_radius = .5
        self.n.mission_mode = 'dynamic'
        self.assertTrue(self.n.handoff_on_driven_path(self.o)[0])

    def test_handoff_unavailable_path_returns_unknown(self):
        self.edge(1.4,x=6.)
        self.n.local_path_time = 9.
        self.assertEqual(self.n.handoff_on_driven_path(self.o), (None,None,None))
        self.n.local_path_time = 10.
        self.n.local_path_xy = np.array([[0.,0.],[5.,0.]])
        self.n.local_path_s = np.array([0.,5.])
        self.assertEqual(self.n.handoff_on_driven_path(self.o), (None,None,None))

    def test_missing_parameter_preserves_old_radius(self):
        with patch.object(fixture.stop.rospy,'get_param',side_effect=lambda key,default: default):
            self.assertEqual(self.n._load_avoidance_corridor_radius(), 1.65)
        del self.n.tunnel_avoidance_corridor_radius
        self.assertEqual(self.n._driven_corridor_radius(True), 1.65)

    def test_invalid_parameter_warns_and_falls_back(self):
        for raw in [0.,-1.,float('nan'),float('inf'),None,'invalid']:
            with self.subTest(raw=raw), patch.object(fixture.stop.rospy,'get_param',return_value=raw), \
                    patch.object(fixture.stop.rospy,'logwarn') as warn:
                self.assertEqual(self.n._load_avoidance_corridor_radius(), 1.65)
                warn.assert_called_once()
        with patch.object(fixture.stop.rospy,'get_param',return_value=1.3):
            self.assertEqual(self.n._load_avoidance_corridor_radius(), 1.3)

    def test_launch_radii_match_and_old_protection_stays(self):
        launch = Path(__file__).resolve().parents[1]/'launch'
        def params(name):
            return {p.attrib['name']:p.attrib.get('value') for p in ET.parse(launch/name).iter('param')}
        stop = params('obstacle_stop.launch'); planner = params('obstacle_avoidance.launch')
        # Tunable planner radius may be more conservative than the stop corridor.
        self.assertGreaterEqual(float(planner['robot_radius']), .95+.20)
        self.assertEqual(float(stop['tunnel_avoidance_corridor_radius']), 1.3)
        self.assertEqual(float(stop['tunnel_path_corridor_radius']), float(planner['path_corridor_radius']))


if __name__ == '__main__':
    unittest.main()
