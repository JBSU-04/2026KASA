import importlib.util
from pathlib import Path
import unittest
import json
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
def module(name,path):
    spec=importlib.util.spec_from_file_location(name,ROOT/path)
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
stop=module('motion_stop_test','new_lane/scripts/obstacle_stop_node.py')
tracking=module('motion_highway_test','lidar_ttc_tracker/scripts/object_tracking.py')

class StopIntegration(unittest.TestCase):
    def setUp(self):
        self.n=stop.ObstacleStopNode.__new__(stop.ObstacleStopNode)
        self.n.predictive_risk=dict(enabled=True,lateral_range=6.,horizon=4.,margin=.35,
            uncertainty_sigma=2.,uncertainty_cap=1.5,fast_lateral_min=1.2)
        self.n.stop_zone_min_forward=0.;self.n.stop_zone_max_forward=18.
        self.n.stop_zone_width=3.6;self.n.motion_fresh_sec=.3
        self.n.ttc_ego_front_m=2.;self.n.ego_speed=5.
        self.n.dynamic_memory_sec=2.;self.n._dynamic_seen={}
        self.n._path_yaw_at=lambda *args:0.
    def obs(self,y=4.,vy=-2.):
        return dict(position_ego=[10.,y,0.],velocity=[0.,vy],ego_yaw=0.,
            radius=.35,confirmed=True,velocity_valid=True,observation_age=0.,vel_std=.1)
    def test_outside_current_corridor_both_sides(self):
        left=self.obs();left['ego_y_rate_signed']=-2.
        right=self.obs(-4,2);right['ego_y_rate_signed']=2.
        self.assertTrue(self.n._predictive_conflict(left)[0])
        self.assertTrue(self.n._predictive_conflict(right)[0])
    def test_yaw_rotates_velocity_not_box_heading(self):
        o=self.obs();o.update(ego_yaw=np.pi/2,velocity=[2.,0.],
                              ego_y_rate_signed=-2.)
        self.assertTrue(self.n._predictive_conflict(o)[0])
    def test_static_roadside_not_extrapolated(self):
        self.assertFalse(self.n._predictive_conflict(self.obs(4.,0.))[0])
    def test_outside_prediction_requires_valid_velocity(self):
        o=self.obs();o.update(position_ego=[5,0,0],velocity_valid=False,
                              in_trigger_corridor=True)
        self.assertFalse(self.n._predictive_conflict(o)[0])

    def test_unknown_roadside_structure_is_not_occupancy(self):
        o=self.obs(-2.23, 0.);o.update(position_ego=[2.3,-2.23,0],
            velocity_valid=False,in_trigger_corridor=False,
            ego_y_rate_signed=0.03)
        self.assertFalse(self.n._predictive_conflict(o)[0])

    def test_world_velocity_jump_needs_ego_lateral_support(self):
        o=self.obs(-2.05, 3.7);o.update(position_ego=[14.,-2.05,0],
            velocity=[0.,3.7],in_trigger_corridor=False,
            ego_y_rate_signed=0.31)
        self.assertFalse(self.n._predictive_conflict(o)[0])
    def test_stale_velocity_not_reused(self):
        o=self.obs();o['observation_age']=1.
        self.assertFalse(self.n._predictive_conflict(o)[0])

    def test_slow_crossing_before_box_velocity_confirmation(self):
        for sign in [-1., 1.]:
            o = self.obs(3.*sign, -.5*sign)
            o.update(position_ego=[12., 3.*sign, 0.],
                     velocity_valid=False, observed_motion_valid=True,
                     observed_velocity=[0., -.5*sign],
                     ego_y_rate_signed=-.5*sign, motion_state='unknown', id=12)
            self.assertTrue(self.n._dynamic_zone_stop(o, False, False, True, 10.)[0])

    def test_provisional_motion_away_does_not_stop(self):
        o=self.obs(4., .5)
        o.update(observed_motion_valid=True, observed_velocity=[0., .5],
                 velocity_valid=False, ego_y_rate_signed=.5)
        self.assertFalse(self.n._predictive_conflict(o)[0])

    def test_in_path_dynamic_stops_immediately(self):
        o=self.obs();o.update(id=8,motion_state='dynamic',
                              in_trigger_corridor=True)
        stop_now,why=self.n._dynamic_zone_stop(o,True,True,True,10.)
        self.assertTrue(stop_now);self.assertEqual(why,'in_path_dynamic')

    def test_in_path_static_does_not_stop(self):
        o=self.obs();o.update(id=9,motion_state='static',
                              ego_y_rate_signed=.03,in_trigger_corridor=True)
        self.assertFalse(self.n._dynamic_zone_stop(o,True,True,True,10.)[0])

    def test_outside_dynamic_requires_velocity_and_conflict(self):
        o=self.obs();o.update(id=8,motion_state='dynamic',
                              ego_y_rate_signed=-2.,in_trigger_corridor=False)
        self.assertTrue(self.n._dynamic_zone_stop(o,False,False,True,10.)[0])
        o['velocity_valid']=False
        self.assertFalse(self.n._dynamic_zone_stop(o,False,False,True,10.1)[0])

    def test_dynamic_identity_memory_expires(self):
        o=self.obs();o.update(id=8,motion_state='dynamic',
                              in_trigger_corridor=True)
        self.assertTrue(self.n._dynamic_zone_stop(o,True,True,True,10.)[0])
        o.update(motion_state='unknown',ego_y_rate_signed=0.)
        self.assertTrue(self.n._dynamic_zone_stop(o,True,True,True,11.)[0])
        self.assertFalse(self.n._dynamic_zone_stop(o,True,True,True,12.1)[0])

class HighwayIntegration(unittest.TestCase):
    def test_shared_core_connected_to_output_and_relative_velocity(self):
        tracker=tracking.MultiObjectTracker(motion_backend='apollo_measurement',gate_dist=5.)
        cloud=np.array([[x,y,.5] for x in [-2.,0.,2.] for y in [-.8,.8]])
        for i in range(20):
            x=302499.+20.*i*.12;y=4124470.
            objects=tracker.update([dict(x=x,y=y,yaw=0.,length=4.,width=1.6,
                height=1.,vehicle_score=1.,yaw_confidence=1.,
                points_world=cloud+[x,y,0])],.12,ego_vx=18.)
        self.assertEqual(len(objects),1)
        self.assertTrue(objects[0].velocity_valid)
        self.assertAlmostEqual(objects[0].vx,20.,delta=.05)
        self.assertAlmostEqual(objects[0].rvx,2.,delta=.05)


class MissionDispatchTest(unittest.TestCase):
    def setUp(self):
        rospy = stop.rospy
        for name in ('init_node', 'Subscriber', 'Publisher', 'Timer', 'spin',
                     'loginfo', 'loginfo_throttle'):
            p = patch.object(rospy, name)
            p.start(); self.addCleanup(p.stop)
        p = patch.object(rospy, 'get_param', side_effect=lambda k, d=None: d)
        p.start(); self.addCleanup(p.stop)
        p = patch.object(rospy.Time, 'now', return_value=rospy.Time.from_sec(10.))
        p.start(); self.addCleanup(p.stop)
        self.n = stop.ObstacleStopNode()
        self.n.current_lane_number = 3
        self.n.current_waypoint_idx = 7000

    def message(self, mode):
        return SimpleNamespace(data=json.dumps(dict(mission_mode=mode,
            collision_objects=[dict(id=8, position_ego=[5., 0., 0.],
                motion_state='unknown', confirmed=True,
                in_trigger_corridor=True, observation_age=0., velocity_valid=False)])))

    def test_tunnel_unknown_stops_on_csv3(self):
        self.n.obstacle_info_callback(self.message('tunnel'))
        self.assertTrue(self.n.dynamic_obstacle_near)

    def test_highway_csv3_is_not_tunnel_without_mission_mode(self):
        self.n.obstacle_info_callback(self.message('out_of_section'))
        self.assertFalse(self.n.dynamic_obstacle_near)
