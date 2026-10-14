"""M3 measured motion regressions; no master, replay, or vehicle commands."""
import sys
from pathlib import Path
import unittest
from unittest.mock import patch
import time
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from m3_motion_evidence import MotionEvidence
import obstacle_stop_node as stop


def observation(stamp, y=4., **kwargs):
    o=dict(id=125, last_seen=stamp, position_world=[12.,y],
           position_ego=[12.,y,0.], length=.35,width=.64,
           motion_state='unknown',velocity_valid=False,observed_motion_valid=False)
    o.update(kwargs);return o


def node(policy=True):
    n=stop.ObstacleStopNode.__new__(stop.ObstacleStopNode)
    n.m3_measured_policy=policy;n._m3_evidence=MotionEvidence()
    n._pos_hist={};n._m3_world_vel={};n._m3_approach_hits={}
    n._path_lateral=lambda x,y:(y,1.,0.)
    n.m3_approach_lookback_s=.25;n.m3_approach_long_s=.5
    n.m3_approach_min_dt=.2;n.m3_approach_min_speed=.8
    n.m3_approach_min_disp=.45;n.m3_approach_near_disp=.6
    n.m3_jump_speed=6.;n.m3_reversal_m=.5;n.m3_onset_enabled=False
    n.m3_approach_max_angle_deg=50.;n.m3_path_corridor_radius=1.3
    n.m3_approach_horizon_s=6.;n.m3_tracker_still_speed=.4
    n.m3_front_overhang=2.95;n.m3_rear_overhang=1.75
    n.m3_approach_near_m=15.;n.m3_approach_hits=2;n.m3_approach_hits_near=1
    n.ego_yaw=0.;n.ego_speed=8.
    return n


class EvidenceTests(unittest.TestCase):
    def test_id125_stale_unknown_does_not_approach(self):
        n=node();o=observation(10., y=-4.07)
        n._m3_world_vel[125]=(0.,3.1);n._m3_approach_hits[125]=(2,10.)
        n._pos_hist[125]=[(9.8,12.,-4.7),(10.,12.,-4.07)]
        with patch.object(stop.rospy.Time,'now',return_value=stop.rospy.Time.from_sec(11.3)):
            self.assertFalse(n._m3_approach(o,125)[0])
        self.assertNotIn(125,n._m3_world_vel);self.assertNotIn(125,n._m3_approach_hits)

    def test_same_stamp_never_adds_measurement(self):
        e=MotionEvidence();o=observation(10.)
        for _ in range(6):e.update(o,10.1)
        self.assertEqual(len(e.history(o,10.1)),1)

    def test_missing_stamp_and_time_reversal(self):
        e=MotionEvidence();o=observation(10.);e.update(o,10.)
        e.update(observation(9.9),9.9)
        self.assertEqual(len(e.history(observation(9.9),9.9)),1)
        o.pop('last_seen');e.update(o,10.);self.assertNotIn(125,e.tracks)

    def test_gap_or_jump_resets_evidence(self):
        e=MotionEvidence();e.update(observation(10.),10.)
        e.update(observation(10.4),10.4)
        self.assertEqual(len(e.history(observation(10.4),10.4)),1)
        e.update(observation(10.5,y=6.),10.5);self.assertNotIn(125,e.tracks)

    def test_partial_static_surface_not_centroid_motion(self):
        e=MotionEvidence()
        points=[[12.,y] for y in [3.5,3.7,3.9,4.1,4.3,4.5]]
        e.update(observation(10.,support_points_world=points),10.)
        o=observation(10.1,y=3.7,width=1.,support_points_world=points[:4])
        e.update(o,10.1)
        self.assertEqual(e.details[125]['reason'],'stationary_surface_shape_shift')
        self.assertEqual(e.history(o,10.1)[0][2],e.history(o,10.1)[1][2])

    def test_unmatched_shape_without_surface_not_motion(self):
        e=MotionEvidence();e.update(observation(10.),10.)
        e.update(observation(10.1,y=3.6,width=1.2),10.1)
        self.assertEqual(e.details[125]['reason'],'shape_change_without_surface')

    def test_both_sides_crossing_same_cycle_as_legacy(self):
        for sign in [-1.,1.]:
            new,old=node(),node(False)
            for i in range(4):
                t=10.+i*.1;o=observation(t,sign*(4.-i*.3))
                # Same visible measured surface translating toward the lane.
                o['support_points_world']=[[12.+dx,o['position_world'][1]+dy]
                    for dx,dy in [(0,0),(.2,.01),(.05,.2),(.3,.21)]]
                new._m3_evidence.update(o,t)
                old._pos_hist.setdefault(125,[]).append((t,12.,o['position_world'][1]))
                with patch.object(stop.rospy.Time,'now',return_value=stop.rospy.Time.from_sec(t)):
                    self.assertEqual(new._m3_approach(o,125)[0],old._m3_approach(o,125)[0])
            with patch.object(stop.rospy.Time,'now',return_value=stop.rospy.Time.from_sec(t)):
                self.assertTrue(new._m3_approach(o,125)[0])
            self.assertTrue(new._m3_time_overlap(o)[0])

    def test_time_overlap_not_just_lateral_direction(self):
        n=node();o=observation(10.,y=4.)
        n._m3_world_vel[125]=(0.,-3.)
        self.assertTrue(n._m3_time_overlap(o)[0])
        n._m3_world_vel[125]=(20.,-3.)
        self.assertFalse(n._m3_time_overlap(o)[0])
        n._m3_world_vel[125]=(0.,3.)
        self.assertFalse(n._m3_time_overlap(o)[0])

    def test_low_obstacle_has_no_added_height_gate(self):
        e=MotionEvidence();o=observation(10.,top_elevation=.08)
        e.update(o,10.);o=observation(10.1,y=3.7,top_elevation=.08);e.update(o,10.1)
        self.assertEqual(len(e.history(o,10.1)),2)

    def test_expired_measurements_and_prune(self):
        e=MotionEvidence();o=observation(10.);e.update(o,10.)
        self.assertEqual(e.history(o,10.351),[])
        e.update(observation(11.),11.);e.prune(set());self.assertEqual(e.tracks,{})

    def test_surface_work_bounded(self):
        e=MotionEvidence();samples=[]
        p=np.random.RandomState(3).rand(200,2)*.4+[12.,4.]
        for i in range(150):
            t=10.+i*.1;o=observation(t,support_points_world=(p+[0.,-i*.02]).tolist(),y=4.-i*.02)
            start=time.perf_counter();e.update(o,t);samples.append((time.perf_counter()-start)*1000)
        print('M3 single-object evidence p95_ms:',round(float(np.percentile(samples,95)),3))
        self.assertLess(np.percentile(samples,95),5.)

if __name__=='__main__':unittest.main()
