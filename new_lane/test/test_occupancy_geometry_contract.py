"""Synthetic regressions for world geometry, evidence and finite path coverage."""
import ast
import copy
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
import test_static_occupancy as occ
import test_background_driven_path as bg
from obstacle_geometry import rectangle,projection,body_overlap


class OccupancyGeometryContractTests(unittest.TestCase):
    def setUp(self):self.c=occ.m.StaticOccupancy()
    def obj(self,**extra):
        o=dict(id=8,position_world=[10.,0.],position_ego=[10.,0.,-1.],length=1.,width=1.,
               confirmed=True,in_trigger_corridor=True,last_seen=10.,observation_age=0.,motion_state='unknown',
               source='lidar_only',ego_yaw=0.,footprint_yaw=0.,stationary_sec=0.)
        o.update(extra);return o
    def update(self,o,t,pose=(0.,0.,0.)):
        return self.c.update(dict(mission_mode='static',obstacles=o[:],collision_objects=o[:]),pose,t)
    def seed(self):
        for t in [9.8,9.9,10.]:self.update([self.obj(last_seen=t)],t)
    def test_one_or_two_hits_do_not_latch_blind_object(self):
        self.update([self.obj()],10.)
        self.update([self.obj(last_seen=10.1)],10.1)
        self.assertFalse(self.update([],11.)['obstacles'])
    def test_three_unique_hits_keep_blind_object(self):
        self.seed();self.assertTrue(self.update([],20.)['obstacles'][0]['held_prediction'])
    def test_republication_is_not_three_observations(self):
        for t in [10.,10.05,10.1,10.15]:self.update([self.obj()],t)
        self.assertFalse(self.update([],11.)['obstacles'])
    def test_one_large_fit_applies_now_but_does_not_permanently_expand(self):
        self.seed();r=self.update([self.obj(length=3.,last_seen=10.1)],10.1)
        from obstacle_geometry import projection
        box=projection(r['obstacles'][0],0.)
        self.assertGreaterEqual(box[1]-box[0],3.)
        self.assertAlmostEqual(r['obstacles'][0]['length'],1.)
        self.assertAlmostEqual(self.update([],10.2)['obstacles'][0]['length'],1.)
    def test_repeated_large_fit_expands_retained_shape(self):
        self.seed()
        for t in [10.1,10.2,10.3]:self.update([self.obj(length=3.,last_seen=t)],t)
        self.assertGreaterEqual(self.update([],10.4)['obstacles'][0]['length'],3.)
    def test_yaw_change_does_not_rotate_static_world_shape(self):
        self.seed();poly=self.update([],10.1)['obstacles'][0]['footprint_world']
        out=self.update([],10.2,(0.,0.,math.pi/2))['obstacles'][0]
        self.assertEqual(poly,out['footprint_world']);self.assertAlmostEqual(out['width'],1.)
    def test_rotated_observation_projects_corners_not_raw_dimensions(self):
        o=self.obj(footprint_world=rectangle(10.,0.,math.pi/2,4.,1.))
        box=projection(o,0.);self.assertAlmostEqual(box[1]-box[0],1.);self.assertAlmostEqual(box[3]-box[2],4.)
    def test_background_three_distinct_hits_remove_parent(self):
        self.seed()
        for t in [10.1,10.2,10.3]:self.update([self.obj(last_seen=t,background_excluded=True)],t)
        self.assertFalse(self.c.parents)
    def test_wide_cached_object_outside_body_is_not_a_proximity_stop(self):
        o=self.obj(position_world=[3.404,-3.058],length=2.911,width=3.714)
        self.assertFalse(body_overlap(o,(0.,0.),0.))
    def test_actual_large_object_intrudes_and_stops(self):
        o=self.obj(position_world=[3.404,-2.9],length=2.911,width=3.714)
        self.assertTrue(body_overlap(o,(0.,0.),0.))
    def test_real_body_overlap_stays_true_after_sensor_blindness(self):
        self.assertTrue(body_overlap(self.obj(position_world=[3.,0.],observation_age=30.),(0.,0.),0.))


class IndexCallbackContractTests(unittest.TestCase):
    def load(self,name):
        p=Path(__file__).resolve().parents[2]/'yolo_test/obstacle_perception_node.py'
        tree=ast.parse(p.read_text());cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='ObstaclePerceptionNode')
        fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name==name)
        ns={};exec(compile(ast.Module(body=[fn],type_ignores=[]),str(p),'exec'),ns);return ns[name]
    def test_actual_state_message_without_index_keeps_waypoint_index(self):
        n=SimpleNamespace(current_wp_idx=2350,current_lane=0)
        self.load('gps_state_callback')(n,SimpleNamespace(current_state=0,lane_number=0,dist=0.))
        self.assertEqual(n.current_wp_idx,2350)
    def test_valid_index_message_updates_index(self):
        n=SimpleNamespace(current_wp_idx=2350,current_lane=0)
        self.load('gps_state_callback')(n,SimpleNamespace(waypoint_index=2401,lane_number=0))
        self.assertEqual(n.current_wp_idx,2401)
    def test_invalid_index_does_not_destroy_valid_index(self):
        n=SimpleNamespace(current_wp_idx=2350,current_lane=0)
        self.load('gps_state_callback')(n,SimpleNamespace(waypoint_index=-1,lane_number=0))
        self.assertEqual(n.current_wp_idx,2350)


class CoverageContractTests(unittest.TestCase):
    def setUp(self):
        f=bg.BackgroundDrivenPathTests();f.setUp();self.addCleanup(f.doCleanups);self.n=f.node
    def test_braking_distance_fits_known_prefix(self):
        self.n.local_path_xy=np.array([[0.,3.],[16.,3.]])
        cap=self.n.coverage_speed_cap(10.)
        self.assertLessEqual(cap*.3+cap*cap/5.,self.n.path_coverage(10.)+1e-8)
    def test_ego_on_path_start_float_noise_keeps_coverage(self):
        # 16:19:44 백: 정지 중 경로 첫 점 = 자차, u=-1e-9 -> 커버리지 0 -> 상한 0 교착.
        self.n.local_path_xy=np.array([[0.,3.],[0.1,3.],[16.,3.]])
        self.n.ego_xy=(-1e-9,3.)
        self.assertGreater(self.n.path_coverage(10.),10.)
        self.n.ego_xy=(-.8,3.)
        self.assertEqual(self.n.path_coverage(10.),0.)
    def test_future_two_ms_timestamp_tolerated(self):
        self.n.local_path_time=10.002
        self.assertGreater(self.n.path_coverage(10.),0.)
    def test_future_excess_or_stale_path_has_zero_cap(self):
        for stamp in [10.1,9.4]:
            self.n.local_path_time=stamp;self.assertEqual(self.n.coverage_speed_cap(10.),0.)
    def test_one_bg_jump_is_not_motion_evidence(self):
        self.n._bg_motion_history={3:[(9.8,10.,0.),(9.9,10.,0.),(10.,10.,1.)]}
        self.assertFalse(self.n.bg_motion_verified({'id':3},10.))
    def test_three_coherent_bg_observations_are_motion(self):
        self.n._bg_motion_history={3:[(9.8,10.,0.),(9.9,10.,.2),(10.,10.,.4)]}
        self.assertTrue(self.n.bg_motion_verified({'id':3},10.))
    def test_current_body_bg_overlap_needs_no_motion_history(self):
        self.n._bg_motion_history={};self.n._bg_validation_objects=[]
        self.assertTrue(self.n._bg_moving_stop(self.n_candidate())[0])
    @staticmethod
    def n_candidate():return dict(id=3,x=2.,y=3.,bx=2.,by=0.,vx=0.,vy=0.)

class MergerCurbContractTests(unittest.TestCase):
    def setUp(self):
        from unittest.mock import Mock
        from threading import RLock
        self.m=occ.m.Merger.__new__(occ.m.Merger)
        m=self.m;m.lock=RLock();m.occupancy=occ.m.StaticOccupancy()
        m.route=[(0.,0.,2400,0),(20.,0.,2600,0)]
        m.road_profile=[2400,2.3,4.55,2600,2.3,4.55];m.vehicle_half_width=.95
        m.background_ids=set();m.background_time=0.;m.pose=(0.,0.,0.);m.pose_stamp=10.
        m.near_timeout=1.;m.far_timeout=.75;m.dedup=1.5
        m.pub=Mock();m.marker_pub=Mock();m.near=({},occ.m.rospy.Time(0));m.far=({},occ.m.rospy.Time(0))
        self.clock=patch.object(occ.m.rospy.Time,'now',return_value=occ.m.rospy.Time.from_sec(10.))
        self.clock.start();self.addCleanup(self.clock.stop)
    def obj(self,y):
        return dict(id=8,position_world=[10.,y],position_ego=[10.,y,0.],length=1.,width=.8,
                    confirmed=True,in_trigger_corridor=True,last_seen=10.,observation_age=0.,motion_state='static')
    def test_complete_outside_curb_is_excluded(self):
        self.assertEqual(self.m.background_reason(self.obj(-3.8),'static',10.),'outside_measured_right_curb')
    def test_partial_intrusion_is_kept(self):
        self.assertIsNone(self.m.background_reason(self.obj(-3.5),'static',10.))
    def test_disabled_curb_filter_keeps_object(self):
        self.m.curb_enabled=False
        self.assertIsNone(self.m.background_reason(self.obj(-3.8),'static',10.))
    def test_real_publish_emits_geometry_and_markers(self):
        import json
        o=self.obj(0.);self.m.near=(dict(mission_mode='static',obstacles=[o],collision_objects=[o]),occ.m.rospy.Time.from_sec(10.))
        self.m.publish();root=json.loads(self.m.pub.publish.call_args[0][0].data)
        self.assertEqual(len(root['collision_objects'][0]['footprint_world']),4)
        self.assertGreater(len(self.m.marker_pub.publish.call_args[0][0].markers),1)


if __name__=='__main__':unittest.main()
