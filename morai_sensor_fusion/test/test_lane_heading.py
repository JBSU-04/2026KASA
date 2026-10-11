import math
import unittest
import numpy as np
from morai_sensor_fusion.lane_heading import fit_heading, ground_points, path_tangent, sample_rows


class LaneHeadingTest(unittest.TestCase):
    def test_curve_preview_and_lane_pair(self):
        x = np.linspace(2.1,14,20)
        center = .01*x*x
        result = fit_heading(np.column_stack((x,center+1.4)),np.column_stack((x,center-1.4)))
        self.assertAlmostEqual(result[0],math.atan(.08),places=6)
        self.assertAlmostEqual(result[1],2.8,places=6)
        self.assertIsNone(fit_heading(np.column_stack((x,center+1.4)),np.empty((0,2))))

    def test_no_slope_from_constant_lateral_offset(self):
        x = np.linspace(3,14,15)
        left = np.column_stack((x,np.full(len(x),1.6)))
        right = np.column_stack((x,np.full(len(x),-1.2)))
        self.assertAlmostEqual(fit_heading(left,right)[0],0.,places=6)

    def test_ground_projection_pitch(self):
        p = ground_points([[640,500],[640,100]],1280,720,90,1.635,math.radians(2))
        self.assertTrue(np.isfinite(p[0]).all())
        self.assertGreater(p[0,0],0)
        self.assertFalse(np.isfinite(p[1]).any())

    def test_reference_uses_segment_and_metric_preview(self):
        p = np.array([[0,0],[5,0],[5,5]],dtype=float)
        yaw,distance = path_tangent(p,3,.2,4)
        self.assertAlmostEqual(yaw,math.pi/2)
        self.assertAlmostEqual(distance,.2)
        self.assertIsNone(path_tangent(p,3,.2,20))


class SampleTimingTest(unittest.TestCase):
    def test_fresh_future_frame_uses_bounded_extrapolation(self):
        rows=[(1.,0.,0.),(1.02,.02,.01)]
        np.testing.assert_allclose(sample_rows(rows,1.03),[.03,.015])
        self.assertIsNone(sample_rows(rows,1.10))
    def test_exact_first_and_last_samples(self):
        rows=[(1.,.2),(1.02,.3)]
        self.assertEqual(sample_rows(rows,1.),[.2])
        self.assertEqual(sample_rows(rows,1.02),[.3])
    def test_wrapped_yaw_and_sensor_gap(self):
        rows=[(1.,math.radians(179)),(1.02,math.radians(-179))]
        self.assertAlmostEqual(sample_rows(rows,1.01,(1,))[0],math.pi)
        self.assertIsNone(sample_rows([(1.,0),(1.3,0)],1.1))

if __name__ == '__main__': unittest.main()
