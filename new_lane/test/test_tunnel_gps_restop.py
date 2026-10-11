"""M8 guard regression and fast floor equivalence; no ROS master needed."""
import unittest
from unittest.mock import patch
import numpy as np
import test_tunnel_driven_stop as fixture
from tunnel_map_geometry import regional_ground_fast,ScanConfirmation
from obstacle_geometry import regional_plane_heights

class GuardTests(unittest.TestCase):
    def setUp(self):
        f=fixture.DrivenStopTests();f.setUp();self.n,self.o=f.n,f.o
    def test_side_object_clears_guard_without_widening_path(self):
        with patch.object(self.n,'future_body_conflict',return_value=False):
            self.assertIs(self.n.tunnel_handoff_conflict(self.o,10.),False)
            self.assertEqual(self.n._handoff_guard_reason,'future_body_clear')
    def test_real_body_conflict_keeps_guard(self):
        with patch.object(self.n,'future_body_conflict',return_value=True):
            self.assertIs(self.n.tunnel_handoff_conflict(self.o,10.),True)
    def test_missing_path_does_not_release_guard(self):
        with patch.object(self.n,'future_body_conflict',return_value=None):
            self.assertIsNone(self.n.tunnel_handoff_conflict(self.o,10.))
    def test_stale_path_unknown(self):
        self.n.local_path_time=8.
        self.assertIsNone(self.n.tunnel_handoff_conflict(self.o,10.))

class GroundTests(unittest.TestCase):
    def test_batched_planes_match_original_with_grade_and_obstacle(self):
        x,y=np.meshgrid(np.arange(-12,36,.25),np.arange(-10,10,.25))
        p=np.c_[x.ravel(),y.ravel(),(-1.765+.015*x+.025*y).ravel()]
        obstacle=p[(p[:,0]>10)&(p[:,0]<11)&(abs(p[:,1])<.5)].copy();obstacle[:,2]+=.1
        p=np.vstack([p,obstacle]);a,v=regional_ground_fast(p,-1.765);b,w,_=regional_plane_heights(p,-1.765)
        np.testing.assert_array_equal(v,w);np.testing.assert_allclose(a,b,equal_nan=True,atol=1e-10)
    def test_unknown_empty_safe(self):
        h,v=regional_ground_fast(np.empty((0,3)),-1.765);self.assertEqual(len(h),0)
    def test_duplicate_scan_cannot_confirm(self):
        c=ScanConfirmation();p=np.array([[0,0,.1],[.1,0,.1],[.2,0,.1]])
        self.assertFalse(c.update(p,np.ones(3,bool),10).any());self.assertFalse(c.update(p,np.ones(3,bool),10).any())
        self.assertTrue(c.update(p,np.ones(3,bool),10.1).all())
