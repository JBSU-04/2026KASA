import importlib.util
import math
from pathlib import Path
import numpy as np

spec = importlib.util.spec_from_file_location('escape_planning', Path(__file__).parents[1] / 'scripts/escape_planning.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def test_polygon_interior_is_occupied():
    points = m.polygon_points([[0, 0], [4, 0], [4, 4], [0, 4]])
    assert m.body_clearance(points, (2, 2, 0), .5, .5, .5) == 0


def test_reverse_search_keeps_offset_and_vehicle_headings():
    start = (0, 3.7, math.radians(-6.6))
    candidates = m.reverse_candidates(start, [], budget=.4)
    assert candidates
    for c in candidates:
        p = np.array(c['poses'])
        assert np.allclose(p[0], start)
        assert p[-1, 0] < -1.8
        assert max(abs(m.wrap(h-start[2])) for h in p[:, 2]) <= math.radians(15) + 1e-8
        assert max(abs(u) for u in c['controls']) <= .3
        assert max(np.linalg.norm(np.diff(p[:, :2], axis=0), axis=1)) <= .200001


def test_surrounded_start_never_returns_path():
    points = m.polygon_points([[-3, -4], [5, -4], [5, 4], [-3, 4]])
    assert not m.reverse_candidates((0, 0, 0), points, budget=.2)


def test_rear_low_object_blocks_straight_sweep():
    points = m.polygon_points([[-3, -.5], [-2.3, -.5], [-2.3, .5], [-3, .5]])
    candidates = m.reverse_candidates((0, 0, 0), points, budget=.4)
    for c in candidates:
        for p in c['poses'][1:]:
            assert m.body_clearance(points, p, 3.85, .79, .95) >= .5
    assert not candidates


def test_turning_front_corner_collision_is_detected():
    # A point outside the initial straight body enters its front corner when yaw changes.
    points = np.array([[3.5, 1.2]])
    assert m.body_clearance(points, (0, 0, 0), 3.85, .79, .95) > .2
    assert m.body_clearance(points, (0, 0, .2), 3.85, .79, .95) == 0


def test_search_has_bounded_wall_time():
    import time
    t = time.monotonic()
    m.reverse_candidates((0, 0, 0), [], budget=.03)
    assert time.monotonic()-t < .2
