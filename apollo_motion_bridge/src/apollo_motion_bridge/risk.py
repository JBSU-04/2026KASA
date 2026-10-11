"""ROS-independent constant-velocity swept occupancy in one fixed ego frame.

This is a local short-horizon approximation, not a curved-road trajectory model.
"""
import math

def interval(position, velocity, lower, upper, horizon):
    if abs(velocity) < 1e-8:
        return (0.0, horizon) if lower <= position <= upper else None
    a, b = sorted(((lower-position)/velocity, (upper-position)/velocity))
    a, b = max(a, 0.0), min(b, horizon)
    return (a, b) if a <= b else None

def swept_conflict(position, relative_velocity, half_width, front, rear,
                   radius, horizon, padding=0.0):
    values = (*position, *relative_velocity, half_width, front, rear, radius, horizon, padding)
    if not all(math.isfinite(v) for v in values):
        return False, None
    x = interval(position[0], relative_velocity[0], -rear-radius-padding,
                 front+radius+padding, horizon)
    y = interval(position[1], relative_velocity[1], -half_width-radius-padding,
                 half_width+radius+padding, horizon)
    if x is None or y is None:
        return False, None
    start, end = max(x[0], y[0]), min(x[1], y[1])
    return (True, start) if start <= end else (False, None)
