import unittest
from unittest.mock import patch
import numpy as np
from tunnel_occupancy_geometry import associate,swept_owned_cells,body_cells_risk
import test_tunnel_driven_stop as fixture

class Cells(unittest.TestCase):
    def scene(self,cells,stamp=10.):return dict(geometry_stamp=stamp,cell_resolution=.1,collision_cells_world=cells,pose_world=[0,0,0])
    def obj(self,id,x,y):return dict(id=id,position_world=[x,y],observed_footprint_world=[[x-.4,y-.4],[x+.4,y-.4],[x+.4,y+.4],[x-.4,y+.4]])
    def test_separate_objects_keep_empty_gap(self):
        s=self.scene([[6,-2],[6,-1.9],[6,2],[6,2.1]]);a,_=associate(s,[self.obj(1,6,-2),self.obj(2,6,2)],10)
        self.assertEqual(len(a[1]['cells']),2);self.assertEqual(len(a[2]['cells']),2)
        self.assertFalse(swept_owned_cells(s['collision_cells_world'],[[0,0],[20,0]],[0,0],0))
    def test_side_object_not_blocked_by_other_objects_cells(self):
        s=self.scene([[6,2],[6,2.1],[6,0],[6,.1]]);a,_=associate(s,[self.obj(1,6,2),self.obj(2,6,0)],10)
        self.assertFalse(swept_owned_cells(a[1]['cells'],[[0,0],[20,0]],[0,0],0))
        self.assertTrue(swept_owned_cells(a[2]['cells'],[[0,0],[20,0]],[0,0],0))
    def test_stale_geometry_is_unknown(self):self.assertEqual(associate(self.scene([[6,0]],9),[self.obj(1,6,0)],10)[0],{})
    def test_occluded_points_preserved(self):
        a,h=associate(self.scene([[6,0]]),[self.obj(1,6,0)],10)
        b,_=associate(self.scene([[6,0]],10.1),[self.obj(1,6,0)],10.1,h)
        self.assertEqual(a[1]['cells'],b[1]['cells'])
    def test_no_first_observation_is_not_empty_space(self):
        a,_=associate(self.scene([]),[self.obj(1,6,0)],10)
        self.assertFalse(a[1]['valid'])
    def test_retained_cells_removed_by_engine_can_release_guard(self):
        a,h=associate(self.scene([[6,0]]),[self.obj(1,6,0)],10)
        scene=self.scene([],10.1);scene['measured_ray_cleared_world']=[[6,0]]
        b,_=associate(scene,[self.obj(1,6,0)],10.1,h)
        self.assertTrue(b[1]['valid']);self.assertEqual(b[1]['cells'],[])
    def test_transient_expiry_is_not_clear(self):
        _,h=associate(self.scene([[6,0]]),[self.obj(1,6,0)],10)
        b,_=associate(self.scene([],10.1),[self.obj(1,6,0)],10.1,h)
        self.assertFalse(b[1]['valid'])
    def test_leaving_sensor_roi_is_not_clear(self):
        a,h=associate(self.scene([[38,0]]),[self.obj(1,38,0)],10)
        b,_=associate(self.scene([],10.1),[self.obj(1,38,0)],10.1,h)
        self.assertFalse(b[1]['valid'])
    def test_true_body_intrusion_still_stops(self):self.assertTrue(body_cells_risk([[2,0]],[0,0],0))
    def test_beyond_path_is_unknown(self):self.assertIsNone(swept_owned_cells([[30,0]],[[0,0],[10,0]],[0,0],0))

class Motion(unittest.TestCase):
    def setUp(self):
        f=fixture.DrivenStopTests();f.setUp();self.n=f.n
        self.o=dict(id=7,last_seen=10.,observed_position_world=[8.,0.],observed_footprint_world=[[7.8,-.2],[8.2,-.2],[8.2,.2],[7.8,.2]],observed_motion_valid=True,observed_velocity=[1.,0.],velocity_valid=True,velocity=[1.,0.])
    def test_same_measurement_does_not_promote(self):
        self.assertFalse(self.n.m8_motion_evidence(self.o,10)[0]);self.assertFalse(self.n.m8_motion_evidence(self.o,10.1)[0])
    def test_real_stable_shape_motion_promotes(self):
        self.n.m8_motion_evidence(self.o,10);self.o['last_seen']=10.1;self.o['observed_position_world']=[8.1,0.];self.o['observed_footprint_world']=[[x+.1,y]for x,y in self.o['observed_footprint_world']]
        self.assertTrue(self.n.m8_motion_evidence(self.o,10.1)[0])
    def test_growing_box_not_motion(self):
        self.n.m8_motion_evidence(self.o,10);self.o.update(last_seen=10.1,observed_position_world=[8.3,0.],observed_footprint_world=[[7.8,-.2],[8.8,-.2],[8.8,.2],[7.8,.2]])
        self.assertFalse(self.n.m8_motion_evidence(self.o,10.1)[0])
    def test_velocity_direction_disagreement_rejected(self):
        self.n.m8_motion_evidence(self.o,10);self.o.update(last_seen=10.1,observed_position_world=[7.9,0.]);self.assertFalse(self.n.m8_motion_evidence(self.o,10.1)[0])

class DynamicCells(unittest.TestCase):
    def setUp(self):
        f=fixture.DrivenStopTests();f.setUp();self.n=f.n
        self.n.tunnel_cells_authoritative_enabled=True;self.n.mission_mode='tunnel';self.n.current_lane_number=3
        self.n.ego_xy=[0.,0.];self.n.ego_yaw=0.;self.n.ego_speed=4.;self.n.ego_pose_time=10.
        self.n.local_path_xy=np.array([[0.,0.],[30.,0.]]);self.n.ttc_ego_front_m=3.85;self.n.stop_zone_width=1.9
        self.o=dict(id=1,position_world=[10.,0.],collision_geometry_valid=True,collision_geometry_stamp=10.,
                    footprint_world=[[8.,-3.],[12.,-3.],[12.,3.],[8.,3.]],occupied_cells_world=[[10.,3.]])
    def predict(self,vector):
        with patch.object(fixture.stop.rospy.Time,'now')as now:
            now.return_value.to_sec.return_value=10.
            return self.n._driven_predictive_conflict(self.o,vector)
    def test_empty_space_inside_display_box_stays_clear(self):self.assertFalse(self.predict([0.,0.])[0])
    def test_real_cells_crossing_future_body_stop(self):self.assertTrue(self.predict([0.,-1.])[0])
    def test_unknown_dynamic_geometry_is_not_clear(self):
        self.o['collision_geometry_valid']=False;self.assertIsNone(self.predict([0.,0.]))
