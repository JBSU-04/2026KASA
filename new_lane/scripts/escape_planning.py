"""ROS-independent reverse motion search and full-body collision geometry."""
import heapq
import math
import time
import numpy as np


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def body_clearance(points, pose, front, rear, half):
    if not len(points):
        return float('inf')
    dx, dy = points[:, 0] - pose[0], points[:, 1] - pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    u, v = c * dx + s * dy, -s * dx + c * dy
    du = np.maximum(np.maximum(u - front, -rear - u), 0.)
    dv = np.maximum(np.abs(v) - half, 0.)
    return float(np.min(np.hypot(du, dv)))


def polygon_points(poly, ds=.2):
    """Filled polygon, including edges: being inside an object is a collision."""
    p = np.asarray(poly, dtype=float)
    if p.ndim != 2 or p.shape[0] < 3 or p.shape[1] < 2:
        return np.empty((0, 2))
    p = p[:, :2]
    x, y = np.meshgrid(np.arange(p[:, 0].min(), p[:, 0].max() + ds, ds),
                       np.arange(p[:, 1].min(), p[:, 1].max() + ds, ds))
    q = np.column_stack((x.ravel(), y.ravel()))
    inside = np.zeros(len(q), dtype=bool)
    for a, b in zip(p, np.roll(p, -1, axis=0)):
        if abs(b[1] - a[1]) > 1e-10:
            inside ^= ((a[1] > q[:, 1]) != (b[1] > q[:, 1])) & (
                q[:, 0] < (b[0] - a[0]) * (q[:, 1] - a[1]) / (b[1] - a[1]) + a[0])
    edges = []
    for a, b in zip(p, np.roll(p, -1, axis=0)):
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / ds)))
        edges.extend(a + (b - a) * t / n for t in range(n + 1))
    return np.vstack((q[inside], edges))


def reverse_candidates(start, points, wheelbase=3., steer_limit=.3,
                       front=3.85, rear=.79, half=.95, margin=.5,
                       min_distance=2., max_distance=10., budget=.12, goal_heading=None):
    """Bounded Hybrid A* (zero heuristic); controls obey the reverse bicycle model.

    States are discretized at 0.2 m / 5 deg, swept every 0.2 m. Candidates
    retain actual vehicle headings and steering controls, never reverse tangents.
    """
    start = tuple(float(v) for v in start)
    points = np.asarray(points, dtype=float).reshape((-1, 2))
    if body_clearance(points, start, front, rear, half) < margin:
        return []
    serial = 0
    queue = [(0., serial, start, [start], [0.], 0., float('inf'))]
    seen, buckets = {}, {}
    deadline = time.monotonic() + budget
    while queue and time.monotonic() < deadline:
        _, _, p, poses, controls, distance, clearance = heapq.heappop(queue)
        key = (round(p[0] / .2), round(p[1] / .2), round(wrap(p[2]) / math.radians(5)))
        if key in seen and seen[key] <= distance:
            continue
        seen[key] = distance
        if distance >= min_distance - 1e-6:
            bucket = (round(distance / .6), round(wrap(p[2] - start[2]) / math.radians(10)))
            cand = dict(poses=poses, controls=controls, distance=distance, clearance=clearance)
            old = buckets.get(bucket)
            if old is None or (clearance, -sum(abs(v) for v in controls)) > (
                    old['clearance'], -sum(abs(v) for v in old['controls'])):
                buckets[bucket] = cand
        if distance + .6 > max_distance + 1e-6:
            continue
        for steering in (0., -steer_limit, steer_limit):
            x, y, h = p
            segment, clear = [], clearance
            for _ in range(3):
                dh = -.2 * math.tan(steering) / wheelbase
                x -= .2 * math.cos(h + dh / 2)
                y -= .2 * math.sin(h + dh / 2)
                h = wrap(h + dh)
                clear = min(clear, body_clearance(points, (x, y, h), front, rear, half))
                if clear < margin or abs(wrap(h - start[2])) > math.radians(15):
                    break
                segment.append((x, y, h))
            if len(segment) != 3:
                continue
            serial += 1
            nd = distance + .6
            heapq.heappush(queue, (nd + .005 * abs(steering), serial, segment[-1],
                                  poses + segment, controls + [steering] * 3, nd, clear))
    # Prefer the shortest reverse; within 0.5m, clearance and smoothness decide.
    result = sorted(buckets.values(), key=lambda c: (
        int(c['distance'] / .5), -c['clearance'],
        abs(wrap(c['poses'][-1][2]-goal_heading)) if goal_heading is not None else 0.,
        sum(abs(v) for v in np.diff(c['controls']))))[:24]
    for i, c in enumerate(result):
        c['index'] = i
        if not math.isfinite(c['clearance']):
            c['clearance'] = 9999.
    return result
