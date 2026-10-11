import copy
import importlib.util
from pathlib import Path
import unittest
spec=importlib.util.spec_from_file_location('occupancy_merger',Path(__file__).resolve().parents[1]/'scripts/obstacle_info_merger.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
class StaticOccupancyTests(unittest.TestCase):
    def setUp(self): self.cache=m.StaticOccupancy()
    def obj(self, oid=8, x=15., y=.5, length=1., width=1., **extra):
        o=dict(id=oid,position_world=[x,y],position_ego=[x,y,-1.5],length=length,width=width,
               source='lidar_only',confirmed=True,in_trigger_corridor=True,motion_state='unknown',
               observation_age=0.,last_seen=10.,stationary_sec=0.,passed=False)
        o.update(extra);return o
    def update(self, objects, now=10., pose=(0.,0.,0.), mode='static'):
        return self.cache.update(dict(mission_mode=mode,obstacles=objects[:],collision_objects=objects[:]),pose,now)
    def test_far_near_handoff_keeps_parent_and_outer_edges(self):
        first=self.update([self.obj(source='raw_temporal_far',width=1.1)])['obstacles'][0]
        second=self.update([self.obj(oid=20,x=14.8,width=.4,last_seen=10.1)],10.1)['obstacles'][0]
        self.assertEqual(first['static_parent_id'],second['static_parent_id'])
        self.assertGreaterEqual(second['width'],1.1)
    def test_near_face_offset_within_two_meters_hands_off(self):
        first=self.update([self.obj(source='raw_temporal_far',length=.86)])['obstacles'][0]
        second=self.update([self.obj(oid=20,x=13.6,length=.41,last_seen=10.1)],10.1)
        self.assertEqual(len(second['obstacles']),1)
        self.assertEqual(second['obstacles'][0]['static_parent_id'],first['static_parent_id'])
    def retain(self,o,mode='static'):
        for stamp in (9.8,9.9,10.):
            v=copy.deepcopy(o);v['last_seen']=stamp;self.update([v],stamp,mode=mode)
    def test_near_blindness_while_stopped_does_not_release(self):
        self.retain(self.obj(x=4.))
        result=self.update([],20.)['obstacles'][0]
        self.assertTrue(result['held_prediction']);self.assertEqual(result['last_seen'],10.)
        self.assertEqual(result['observation_age'],10.)
    def test_center_passed_is_not_rear_passed(self):
        self.retain(self.obj(x=4.,length=2.))
        self.assertTrue(self.update([],11.,(7.,0.,0.))['obstacles'])
        self.assertFalse(self.update([],12.,(8.2,0.,0.))['obstacles'])
    def test_new_verified_motion_releases_static_parent(self):
        self.update([self.obj()])
        o=self.obj(last_seen=10.1,motion_state='dynamic',observed_motion_valid=True,observed_velocity=[0.,2.])
        out=self.update([o],10.1)['obstacles'];self.assertFalse(out[0].get('static_occupancy',False))
        self.assertFalse(self.cache.parents)
    def test_unconfirmed_speck_not_latched(self):
        self.update([self.obj(confirmed=False)]);self.assertFalse(self.update([],11.)['obstacles'])
    def test_ambiguous_two_parents_not_merged(self):
        self.update([self.obj(x=15.,y=-.8,width=.35),self.obj(oid=9,x=15.,y=.8,width=.35)])
        self.update([self.obj(oid=20,x=15.,y=0.,width=1.6,last_seen=10.1)],10.1)
        self.assertEqual(len(self.cache.parents),3)
    def test_other_mission_clears_occupancy(self):
        self.update([self.obj()]);self.assertFalse(self.update([],11.,mode='dynamic')['obstacles'])
        self.assertFalse(self.cache.parents)
    def test_preview_handoff_does_not_reset_parent(self):
        self.retain(self.obj(),mode='static_preview')
        first=next(iter(self.cache.parents))
        self.assertEqual(first,self.update([],11.)['obstacles'][0]['id'])
    def test_tunnel_far_proposal_alone_is_not_static_certificate(self):
        out=self.update([self.obj(source='raw_temporal_far',motion_state='static')],mode='tunnel')
        self.assertFalse(out['obstacles'][0]['static_certified'])
    def test_old_measurement_is_never_refreshed_by_republication(self):
        self.update([self.obj()]);o=self.obj(observation_age=.6,last_seen=10.)
        out=self.update([o],10.6)['obstacles'][0]
        self.assertEqual(out['last_seen'],10.);self.assertAlmostEqual(out['observation_age'],.6)
if __name__=='__main__':unittest.main()


class DuplicateParentTests(StaticOccupancyTests):
    """2026-10-07: 같은 자리 중복 부모 폭증 방지, 가까운 다른 물체는 합치지 않음."""
    def test_repeated_far_detection_does_not_multiply_parents(self):
        for k,t in enumerate((10.,10.1,10.2,10.3,10.4,10.5)):
            o=self.obj(oid=50+k,x=15.,y=.5,source='raw_temporal_far',last_seen=t)
            o2=self.obj(oid=80+k,x=15.05,y=.55,source='raw_temporal_far',last_seen=t)
            self.update([o,o2],t)
        self.assertLessEqual(len(self.cache.parents),2)
    def test_adjacent_cones_one_meter_apart_stay_separate(self):
        for t in (10.,10.1,10.2,10.3):
            self.update([self.obj(oid=1,x=15.,y=0.,last_seen=t,length=.35,width=.35),
                         self.obj(oid=2,x=16.,y=0.,last_seen=t,length=.35,width=.35)],t)
        self.assertEqual(len(self.cache.parents),2)
