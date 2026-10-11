import math
import unittest
import numpy as np
from apollo_motion_bridge import MotionObservation
from apollo_motion_bridge.risk import swept_conflict

class MotionTest(unittest.TestCase):
    def cloud(self):
        rng = np.random.RandomState(71)
        return rng.uniform([-0.7,-0.3,0], [0.7,0.3,0.5], (60,3))

    def run_motion(self, profile, velocity, origin=(302499.,4124470.,0.)):
        m = MotionObservation(profile)
        points = self.cloud()+origin
        for i in range(16):
            t = i*0.12
            m.update(points+np.array([velocity[0]*t,velocity[1]*t,0]),t)
        self.assertTrue(m.valid)
        np.testing.assert_allclose(m.velocity, velocity, atol=0.05)
        self.assertGreater(np.linalg.eigvalsh(m.covariance).min(),0)
        return m

    def test_both_crossing_directions(self):
        self.run_motion('dynamic',(0,2.4))
        self.run_motion('dynamic',(0,-2.4))

    def test_highway(self):
        self.run_motion('highway',(23.0,0.8))

    def test_direction_uncertainty_is_not_box_heading(self):
        m = self.run_motion('highway', (0., 23.))
        self.assertLess(m.direction_std, .1)
        m = self.run_motion('dynamic', (0., 0.))
        self.assertTrue(math.isinf(m.direction_std))

    def test_highway_velocity_follows_braking(self):
        m = MotionObservation('highway')
        p = self.cloud()
        x = 0.
        for i in range(35):
            speed = max(12., 24.-4.*max(0., i*.1-1.))
            x += speed*.1
            m.update(p+[x, 0., 0.], i*.1)
        self.assertTrue(m.valid)
        self.assertLess(abs(m.velocity[0]-speed), 2.5)
        self.assertLess(abs(m.velocity[1]), .05)

    def test_static(self):
        self.run_motion('dynamic',(0.,0.))

    def test_duplicate_and_out_of_order(self):
        m = self.run_motion('dynamic',(0.,2.))
        old = m.velocity.copy(); stamp=m.stamp
        self.assertFalse(m.update(self.cloud()+1000,stamp))
        self.assertFalse(m.update(self.cloud()+1000,stamp-1))
        np.testing.assert_array_equal(old,m.velocity)
        self.assertEqual(m.stamp,stamp)

    def test_gap_requires_new_baseline(self):
        m=self.run_motion('highway',(20,0))
        self.assertFalse(m.update(self.cloud(),m.stamp+2))
        self.assertFalse(m.valid)

    def test_nonfinite(self):
        m=MotionObservation()
        self.assertFalse(m.update([[0,0,0],[float('nan'),0,0]],0))

    def test_speed_jump_is_unknown_not_clamped(self):
        m=MotionObservation('dynamic')
        p=self.cloud()
        for i in range(6): m.update(p+[i*10.,0,0], i*.12)
        self.assertFalse(m.valid)

class RiskTest(unittest.TestCase):
    def risk(self,p,v):
        return swept_conflict(p,v,1.0,2.0,2.8,.4,4.0)[0]
    def test_crossing_from_both_sides(self):
        self.assertTrue(self.risk((10,4),(-5,-2)))
        self.assertTrue(self.risk((10,-4),(-5,2)))
    def test_parallel_oncoming_outside(self):
        self.assertFalse(self.risk((15,4),(-15,0)))
    def test_cleared_before_ego_arrives(self):
        self.assertFalse(self.risk((15,4),(-3,-4)))
    def test_stationary_ego_struck_by_crossing(self):
        self.assertTrue(self.risk((1,4),(0,-2)))
    def test_moving_away(self):
        self.assertFalse(self.risk((10,4),(-5,2)))

if __name__ == '__main__': unittest.main()

class VisibilityTest(unittest.TestCase):
    def test_stationary_vehicle_partial_view(self):
        p=np.array([[x,y,0.] for x in np.linspace(-2,2,30) for y in [-.8,.8]])
        m=MotionObservation('highway')
        for i in range(20):
            visible=p if i<10 else p[p[:,0]>-2+(i-9)*.15]
            m.update(visible+[302499.,4124470.,0.],i*.12)
        self.assertTrue(m.valid)
        self.assertLess(np.linalg.norm(m.velocity),.15)
