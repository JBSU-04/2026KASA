#!/usr/bin/env python3
import math
import unittest

from morai_sensor_fusion.measurement import (
    CourseBiasEstimator, GpsIntegrityMonitor, bicycle_sideslip, gps_to_base,
    interpolate_angle, normalize_angle, position_variance)


class MeasurementMathTest(unittest.TestCase):
    def test_wrap_interpolation_crosses_pi(self):
        yaw = interpolate_angle(math.radians(179), math.radians(-179), 0.5)
        self.assertAlmostEqual(abs(yaw), math.pi, places=6)

    def test_lever_arm(self):
        x, y = gps_to_base(11.0, 5.0, 0.0, 1.0, 0.0)
        self.assertAlmostEqual(x, 10.0)
        self.assertAlmostEqual(y, 5.0)

    def test_timestamp_uncertainty_grows_at_high_speed(self):
        low = position_variance(0.35, 10.0, 0.02, 0.08, 0.0, 0.05)
        high = position_variance(0.35, 27.78, 0.02, 0.08, 0.0, 0.05)
        self.assertGreater(high, low)

    def test_normalize(self):
        self.assertAlmostEqual(normalize_angle(3.0 * math.pi), math.pi)

    def test_repeated_moving_gps_is_declared_frozen(self):
        monitor = GpsIntegrityMonitor(freeze_hits=3, recover_hits=3)
        self.assertEqual(monitor.update(0.00, 10.0, 20.0, 5.0), monitor.NORMAL)
        self.assertEqual(monitor.update(0.05, 10.0, 20.0, 5.0), monitor.NORMAL)
        self.assertEqual(monitor.update(0.10, 10.0, 20.0, 5.0), monitor.NORMAL)
        self.assertEqual(monitor.update(0.15, 10.0, 20.0, 5.0), monitor.FROZEN)

    def test_gps_requires_consistent_steps_to_reacquire(self):
        monitor = GpsIntegrityMonitor(freeze_hits=2, recover_hits=3)
        monitor.update(0.00, 0.0, 0.0, 5.0)
        monitor.update(0.05, 0.0, 0.0, 5.0)
        monitor.update(0.10, 0.0, 0.0, 5.0)
        self.assertEqual(monitor.mode, monitor.FROZEN)
        # The first large return jump is not itself enough evidence.
        monitor.update(0.15, 12.0, 0.0, 5.0)
        self.assertEqual(monitor.mode, monitor.FROZEN)
        monitor.update(0.20, 12.25, 0.0, 5.0)
        monitor.update(0.25, 12.50, 0.0, 5.0)
        monitor.update(0.30, 12.75, 0.0, 5.0)
        self.assertEqual(monitor.mode, monitor.REACQUIRING)

    def test_invalid_fix_enters_same_recovery_path(self):
        monitor = GpsIntegrityMonitor(recover_hits=2)
        monitor.update(0.0, 0.0, 0.0, 5.0)
        monitor.mark_outage()
        self.assertEqual(monitor.mode, monitor.FROZEN)
        monitor.update(1.0, 5.0, 0.0, 5.0)
        monitor.update(1.1, 5.5, 0.0, 5.0)
        monitor.update(1.2, 6.0, 0.0, 5.0)
        self.assertEqual(monitor.mode, monitor.REACQUIRING)

    def test_status_recovery_can_reacquire_while_stopped(self):
        monitor = GpsIntegrityMonitor()
        monitor.mark_outage()
        monitor.mark_reacquiring()
        self.assertEqual(monitor.mode, monitor.REACQUIRING)

    def test_quantized_slow_start_is_not_frozen(self):
        # MORAI 좌표 양자화 ~0.185 m: 1.5 m/s, 30 Hz 면 4 메시지마다 한 칸 움직인다.
        monitor = GpsIntegrityMonitor(freeze_distance=0.6)
        x = 0.0
        for k in range(60):
            if k % 4 == 3:
                x += 0.185
            self.assertEqual(monitor.update(k / 30.0, x, 0.0, 1.5), monitor.NORMAL)

    def test_real_freeze_still_detected_at_speed(self):
        monitor = GpsIntegrityMonitor(freeze_distance=0.6)
        monitor.update(0.0, 0.0, 0.0, 11.0)
        mode = None
        for k in range(1, 8):
            mode = monitor.update(k / 30.0, 0.37, 0.0, 11.0)
        self.assertEqual(mode, monitor.FROZEN)

    def test_repeated_fix_is_held_before_freeze_confirmation(self):
        monitor = GpsIntegrityMonitor(freeze_distance=0.6, freeze_hits=4)
        monitor.update(0.0, 0.0, 0.0, 11.0)
        monitor.update(1.0 / 30.0, 0.0, 0.0, 11.0)
        self.assertEqual(monitor.mode, monitor.NORMAL)
        self.assertTrue(monitor.suspected_freeze(0.30))

    def test_normal_quantized_step_clears_pre_freeze_hold(self):
        monitor = GpsIntegrityMonitor(freeze_distance=0.6, freeze_hits=4)
        monitor.update(0.0, 0.0, 0.0, 5.0)
        monitor.update(0.1, 0.0, 0.0, 5.0)
        self.assertTrue(monitor.suspected_freeze(0.30))
        monitor.update(0.2, 0.5, 0.0, 5.0)
        self.assertFalse(monitor.suspected_freeze(0.30))
        self.assertEqual(monitor.mode, monitor.NORMAL)

    def test_quantized_slow_recovery(self):
        monitor = GpsIntegrityMonitor(freeze_distance=0.6, recover_hits=3)
        monitor.mark_outage()
        x = 50.0
        for k in range(40):
            if k % 4 == 3:
                x += 0.185
            monitor.update(k / 30.0, x, 0.0, 1.5)
        self.assertEqual(monitor.mode, monitor.REACQUIRING)

    def test_course_bias_uses_distance_baseline(self):
        estimator = CourseBiasEstimator(min_distance=1.0, alpha=1.0)
        bias = math.radians(-1.4)
        estimator.update(0.0, 0.0, 0.0, 0.0, 5.0)
        result = estimator.update(
            0.3, 1.5 * math.cos(bias), 1.5 * math.sin(bias),
            0.0, 5.0)
        self.assertAlmostEqual(result, bias, places=6)

    def test_course_bias_removes_bicycle_component(self):
        estimator = CourseBiasEstimator(min_distance=1.0, alpha=1.0)
        beta = math.radians(-1.0)
        residual = math.radians(-0.3)
        course = beta + residual
        estimator.update(0.0, 0.0, 0.0, 0.0, 5.0, beta)
        result = estimator.update(
            0.3, 1.5 * math.cos(course), 1.5 * math.sin(course),
            0.0, 5.0, beta)
        self.assertAlmostEqual(result, residual, places=6)

    def test_morai_positive_right_steer_has_negative_beta(self):
        beta = bicycle_sideslip(math.radians(4.0))
        self.assertLess(beta, 0.0)
        self.assertAlmostEqual(beta, math.radians(-2.0), places=3)


if __name__ == '__main__':
    unittest.main()
