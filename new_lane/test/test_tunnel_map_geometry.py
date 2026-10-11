"""3D background, independent scan confirmation and actual future-body checks."""
import sys, tempfile, unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
from tunnel_map_geometry import StaticMap3D,ScanConfirmation
import test_tunnel_driven_stop as fixture

class MapGeometry(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.file=Path(self.tmp.name)/'map.npz'
        floor=np.array([[x,y,0.] for x in np.arange(-2,3,.2) for y in np.arange(-2,3,.2)])
        wall=np.array([[2,y,z] for y in np.arange(-2,3,.2) for z in np.arange(0,3,.2)])
        np.savez(self.file,origin=[0.,0.,0.],points=np.vstack([floor,wall]),floor=floor)
        self.map=StaticMap3D(self.file)
    def test_same_xy_different_height_is_not_background(self):
        bg,h,valid=self.map.classify(np.array([[0.,0.,.09],[0.,0.,0.],[2.,0.,2.]]))
        self.assertEqual(bg.tolist(),[False,True,True]);self.assertTrue(valid.all())
    def test_wall_adjacent_low_object_is_foreground(self):
        bg,_,_=self.map.classify(np.array([[1.90,0.,.10]]));self.assertFalse(bg[0])
    def test_supported_road_registration_error_does_not_erase_low_object(self):
        from tunnel_map_geometry import classify_scan
        x,y=np.meshgrid(np.arange(5.,20.,.25),np.arange(-4.,4.,.25))
        raw=np.c_[x.ravel(),y.ravel(),np.full(x.size,-1.13)]
        floor=np.c_[raw[:,:2]+[1.1,0.],np.zeros(len(raw))]
        np.savez(self.file,origin=[0.,0.,0.],points=floor,floor=floor)
        measured=StaticMap3D(self.file)
        raw=np.vstack([raw,[[10.,.1,-1.02],[10.1,.1,-1.02],[10.2,.1,-1.02]]])
        _,bg,height,valid,_,candidate=classify_scan(raw,(0,0,0,0),measured,ScanConfirmation())
        self.assertTrue(valid[:-3].all());self.assertFalse(candidate[:-3].any())
        self.assertFalse(bg[-3:].any());self.assertTrue(candidate[-3:].all())
    def test_missing_floor_is_unknown(self):
        bg,_,valid=self.map.classify(np.array([[30.,0.,.2]]));self.assertFalse(bg[0]);self.assertFalse(valid[0])
    def test_two_distinct_scans_and_three_real_points(self):
        p=np.array([[.1,.1,.1],[.2,.1,.1],[.3,.1,.1]])
        c=ScanConfirmation();mask=np.ones(3,bool)
        self.assertFalse(c.update(p,mask,1.).any());self.assertFalse(c.update(p,mask,1.).any())
        self.assertTrue(c.update(p,mask,1.1).all())
        self.assertFalse(c.update(p,mask,2.).any())
    def test_sparse_one_scan_cannot_confirm(self):
        c=ScanConfirmation();p=np.array([[0.,0.,.1]])
        for t in [1.,1.1,1.2]:self.assertFalse(c.update(p,[True],t).any())

class IndependentComponents(unittest.TestCase):
    def test_voxel_boundary_shift_preserves_two_scan_confirmation(self):
        c=ScanConfirmation();p=np.array([[.46,.1,.1],[.48,.1,.1],[.49,.1,.1]])
        self.assertFalse(c.update(p,np.ones(3,bool),1.).any())
        self.assertTrue(c.update(p+[.07,0,0],np.ones(3,bool),1.1).all())
    def test_unknown_floor_positive_raw_evidence_is_retained(self):
        from tunnel_map_geometry import classify_scan
        p=np.array([[10.,2.,-1.65],[10.1,2.,-1.65],[10.2,2.,-1.65]])
        c=ScanConfirmation();world,bg,h,valid,inside,proposal=classify_scan(p,(0,0,0,0),None,c)
        self.assertFalse(bg.any());self.assertFalse(valid.any());self.assertTrue(proposal.all())
        c.update(world,proposal,1.);self.assertTrue(c.update(world,proposal,1.1).all())

class FutureBody(unittest.TestCase):
    def setUp(self):
        f=fixture.DrivenStopTests();f.setUp();self.n,self.o=f.n,f.o
        self.n.mission_mode='tunnel';self.n.current_lane_number=3;self.n.tunnel_swept_stop_enabled=True
        self.n.local_path_xy=np.column_stack([np.arange(-10.,25.,.25),np.zeros(140)])
        self.n.ego_xy=(0.,0.);self.n.ego_speed=2.;self.n.local_path_time=10.
    def test_side_corridor_touch_does_not_stop(self):
        self.o.update(position_world=[7.,1.6],position_ego=[7.,1.6,0.],length=.4,width=.4)
        self.assertFalse(self.n.future_body_conflict(self.o,10.))
    def test_current_body_overlap_remains_stop(self):
        self.o.update(position_world=[2.,.9],position_ego=[2.,.9,0.],length=.4,width=.4)
        self.assertTrue(self.n.future_body_conflict(self.o,10.))
    def test_old_traversed_path_not_a_future_collision(self):
        self.o.update(position_world=[-7.,0.],position_ego=[-7.,0.,0.],length=.4,width=.4)
        self.assertFalse(self.n.future_body_conflict(self.o,10.))
    def test_short_path_cannot_assert_future_clearance(self):
        self.n.local_path_xy=np.array([[0.,0.],[3.,0.]])
        self.o.update(position_world=[10.,0.],position_ego=[10.,0.,0.])
        self.assertIsNone(self.n.future_body_conflict(self.o,10.))

class PreslowMemory(unittest.TestCase):
    def test_confirmed_raw_sparse_returns_survive_ring_gap(self):
        from test_tunnel_regional_ground import wall
        n=wall.TunnelWallDetector.__new__(wall.TunnelWallDetector)
        n.map_raw_active=True;n.map_preslow_memory=4.;n.bg_memory_sec=1.5;n.kick_memory_sec=4.
        it=[0.,0.,1.,1,True,dict(confirmation_source='raw_map_2_scans')]
        self.assertEqual(n.candidate_memory_sec(it),4.)
        n.map_raw_active=False;self.assertEqual(n.candidate_memory_sec(it),1.5)
