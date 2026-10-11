import unittest
import numpy as np
import test_background_static_validation as fixture
class BackgroundDrivenPathTests(unittest.TestCase):
    def setUp(self):
        fixture.BackgroundStaticValidationTests.setUp(self)
        n=self.node;n._bg_validation_objects=[]
        n.local_path_xy=np.array([[0.,3.],[40.,3.]])
        n.local_path_time=10.;n.ego_xy=np.array([0.,3.]);n.ego_pose_time=10.
        n.ego_yaw=0.;n.ego_speed=4.;n.ttc_ego_front_m=3.85;n.stop_zone_width=1.9
        n.predictive_risk=dict(horizon=4.,margin=.2)
    observation = staticmethod(fixture.BackgroundStaticValidationTests.observation)
    candidate = staticmethod(fixture.BackgroundStaticValidationTests.candidate)
    def test_reference_blocker_clear_on_driven_path(self):
        self.assertFalse(self.node._bg_moving_stop(self.candidate(x=10.,y=0.,vx=0.,vy=0.))[0])
        self.assertEqual(self.node._bg_path_used,'local')
    def test_real_crossing_enters_driven_swept_body(self):
        self.node._bg_motion_history={3:[(9.7,10.,-.36),(9.85,10.,-.18),(10.,10.,0.)]}
        self.assertTrue(self.node._bg_moving_stop(self.candidate(x=10.,y=0.,vx=0.,vy=1.2))[0])
    def test_stale_path_keeps_reference_stop(self):
        self.node.local_path_time=9.
        self.assertTrue(self.node._bg_moving_stop(self.candidate(x=10.,y=0.,vx=0.,vy=0.))[0])
    def test_short_path_uses_prefix_and_braking_cap(self):
        self.node.local_path_xy=np.array([[0.,3.],[5.,3.]])
        self.assertFalse(self.node._bg_moving_stop(self.candidate(x=10.,y=0.,vx=0.,vy=0.))[0])
        self.assertLess(self.node.coverage_speed_cap(10.),2.)
if __name__=='__main__':unittest.main()
