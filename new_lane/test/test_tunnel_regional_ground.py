import importlib.util
from pathlib import Path
import unittest
import numpy as np

spec=importlib.util.spec_from_file_location('regional_wall',Path(__file__).resolve().parents[1]/'scripts/tunnel_wall_detector.py')
wall=importlib.util.module_from_spec(spec);spec.loader.exec_module(wall)

class RegionalGroundTests(unittest.TestCase):
    def test_kick_pose_rejects_internal_history_gap(self):
        node=wall.TunnelWallDetector.__new__(wall.TunnelWallDetector)
        node.pose_hist=[(10.,0.,0.,0.),(10.4,4.,0.,0.)]
        self.assertIsNone(node.kick_pose_at(10.2))
        self.assertEqual(node.kick_pose_at(10.),(0.,0.,0.))
        self.assertIsNone(node.kick_pose_at(9.8))
    def test_kick_pose_interpolates_supported_time(self):
        node=wall.TunnelWallDetector.__new__(wall.TunnelWallDetector)
        node.pose_hist=[(10.,0.,0.,0.),(10.1,1.,0.,0.)]
        self.assertAlmostEqual(node.kick_pose_at(10.05)[0],.5)
        self.assertEqual(node.kick_pose_at(10.15),(1.,0.,0.))
    def road(self):
        x,y=np.meshgrid(np.arange(3,34,.2),np.arange(-5,5,.2))
        z=-1.7+.015*x+.04*y
        return np.column_stack([x.ravel(),y.ravel(),z.ravel()])
    def detect(self,p):
        return wall.regional_ground_points(p,-1.75,1.9,33.9,.25,2.5)
    def test_grade_crossfall_not_obstacles(self):
        q,valid=self.detect(self.road());self.assertEqual(len(q),0)
    def test_low_real_obstacle_retained(self):
        road=self.road();x,y=np.meshgrid(np.arange(12,13,.1),np.arange(-.5,.5,.1))
        block=np.column_stack([x.ravel(),y.ravel(),(-1.7+.015*x+.04*y+.35).ravel()])
        q,valid=self.detect(np.vstack([road,block]));self.assertGreater(len(q),50)
        self.assertTrue(valid.all());self.assertLess(np.max(np.abs(q[:,2]-.35)),.03)
    def test_unsupported_far_plane_not_confirmed(self):
        q,valid=self.detect(np.array([[20,0,-1.3],[20,.1,-1.3],[20,.2,-1.3]]));self.assertEqual(len(q),0)
    def test_unknown_near_field_not_suppressed(self):
        q,valid=self.detect(np.array([[5,0,-1.3],[5,.1,-1.3],[5,.2,-1.3]]))
        self.assertEqual(len(q),3);self.assertFalse(valid.any())
    def test_empty_nan_input_safe(self):
        for p in [np.empty((0,3)),np.array([[np.nan,0,0]])]:self.assertEqual(len(self.detect(p)[0]),0)
    def test_height_model_disagreement_cannot_suppress_real_object(self):
        road=self.road();road[:,2]=-1.7+.015*road[:,0]+.09*road[:,1]
        x,y=np.meshgrid(np.arange(12,13,.1),np.arange(0,1,.1))
        block=np.column_stack([x.ravel(),y.ravel(),(-1.7+.015*x+.09*y+.35).ravel()])
        q,compatible=self.detect(np.vstack([road,block]))
        self.assertGreater(len(q),50);self.assertTrue((~compatible).any())

if __name__=='__main__':unittest.main()
