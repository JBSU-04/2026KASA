#!/usr/bin/env python3
import os
import sys
import unittest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.realpath(__file__)), '..', 'scripts'))

from object_tracking import MultiObjectTracker  # noqa: E402


def detection(score):
    return {
        'x': 30.0,
        'y': 0.0,
        'yaw': 0.0,
        'width': 1.8,
        'length': 4.5,
        'height': 1.5,
        'vehicle_score': score,
        'yaw_confidence': 0.8,
        'obstacle_score': 0.9,
    }


class VehicleClassHysteresisTest(unittest.TestCase):
    def setUp(self):
        self.tracker = MultiObjectTracker(
            min_hits=1,
            vehicle_confirm_score=0.50,
            vehicle_confirm_hits=2,
            vehicle_release_score=0.25,
            vehicle_release_frames=7,
            vehicle_confidence_alpha=0.35,
            max_coast_age=2)

    def update(self, score):
        return self.tracker.update([detection(score)], 0.1)

    def test_confirmed_vehicle_survives_sparse_geometry(self):
        self.update(1.0)
        out = self.update(1.0)
        self.assertTrue(out[0].is_vehicle)

        # Confidence crosses the old 0.50 single-threshold quickly, but the
        # latched vehicle class remains during a short partial-view interval.
        for _ in range(6):
            out = self.update(0.0)
        self.assertLess(out[0].vehicle_confidence, 0.25)
        self.assertTrue(out[0].is_vehicle)

    def test_release_requires_sustained_low_confidence(self):
        self.update(1.0)
        self.update(1.0)
        out = None
        for _ in range(12):
            out = self.update(0.0)
            if not out[0].is_vehicle:
                break
        self.assertIsNotNone(out)
        self.assertFalse(out[0].is_vehicle)

    def test_recovery_resets_release_streak(self):
        self.update(1.0)
        self.update(1.0)
        for _ in range(6):
            self.update(0.0)
        self.update(1.0)
        for _ in range(6):
            out = self.update(0.0)
        self.assertTrue(out[0].is_vehicle)


if __name__ == '__main__':
    unittest.main()
