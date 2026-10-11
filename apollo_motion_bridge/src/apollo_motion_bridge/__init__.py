"""Shared mission motion observations using original Apollo C++ measurements.

This is NOT the complete Apollo MLF tracker: association and position filters
remain mission-owned; temporal selection/confidence below are local adapters.
"""
import ctypes
import os
from collections import deque
from pathlib import Path
import numpy as np

_LIB = None

def load_library():
    global _LIB
    if _LIB is not None:
        return _LIB
    candidates = [Path(p) / 'lib/libapollo_motion_measurement.so'
                  for p in os.environ.get('CMAKE_PREFIX_PATH', '').split(':') if p]
    candidates += [Path(__file__).resolve().parents[4] / 'devel/lib/libapollo_motion_measurement.so']
    for path in candidates:
        if path.is_file():
            lib = ctypes.CDLL(str(path))
            ptr = np.ctypeslib.ndpointer(dtype=np.float64, flags='C_CONTIGUOUS')
            lib.apollo_velocity_candidates.argtypes = [ptr, ctypes.c_int, ptr,
                ctypes.c_int, ctypes.c_double, ctypes.c_double, ptr]
            lib.apollo_velocity_candidates.restype = ctypes.c_int
            _LIB = lib
            return lib
    raise RuntimeError('Build/source apollo_motion_bridge before enabling apollo motion backend')

PROFILES = {
    'dynamic': dict(baseline=0.24, max_gap=0.4, smoothing=0.12,
                    position_sigma=0.08, max_speed=12.0, max_std=1.5),
    'highway': dict(baseline=0.40, max_gap=0.5, smoothing=0.25,
                    position_sigma=0.15, max_speed=45.0, max_std=2.5),
}

class MotionObservation:
    def __init__(self, profile='dynamic', **overrides):
        self.config = dict(PROFILES[profile], **overrides)
        if any(not np.isfinite(v) or v <= 0 for v in self.config.values()):
            raise ValueError('Motion profile values must be finite and positive')
        self.lib = load_library()
        self.history = deque(maxlen=32)
        self.velocity = np.zeros(2)
        self.covariance = np.eye(2) * self.config['max_speed'] ** 2
        self.valid = False
        self.quality = 0.0
        self.stamp = None

    @property
    def std(self):
        return float(np.sqrt(np.linalg.eigvalsh(self.covariance).max()))

    def update(self, points, stamp, yaw=0.0):
        points = np.asarray(points, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] < 3 or len(points) < 2 or not np.isfinite(points[:, :3]).all():
            self.valid = False
            return False
        stamp = float(stamp)
        if not np.isfinite(stamp) or not np.isfinite(yaw):
            self.valid = False
            return False
        if self.stamp is not None and stamp <= self.stamp:
            # Never turn duplicate/out-of-order scans into a positive dt.
            return False
        if self.stamp is not None and stamp-self.stamp > self.config['max_gap']:
            self.history.clear()
            self.valid = False
            self.velocity[:] = 0
            self.covariance = np.eye(2) * self.config['max_speed'] ** 2
        dt_frame = stamp-self.stamp if self.stamp is not None else 0.1
        self.stamp = stamp
        points = np.ascontiguousarray(points[:, :3])
        self.history.append((stamp, points.copy(), float(yaw)))
        # Choose the shortest available baseline meeting the mission setting.
        eligible = [h for h in self.history if
                    self.config['baseline'] <= stamp-h[0] <= 2*self.config['baseline']+dt_frame]
        # Optional fast start (2026-09-25): until one full baseline of history
        # exists, measure over the shorter startup_baseline so a new track gets
        # a velocity after ~2 scans instead of ~4.  As soon as the normal
        # baseline is available it is used again, so steady-state noise is
        # unchanged (a permanently short baseline made highway lead speeds
        # swing 9~14 m/s).  Absent key = previous behaviour.
        startup = self.config.get('startup_baseline', 0.0)
        if not eligible and 0.0 < startup < self.config['baseline']:
            eligible = [h for h in self.history if
                        startup <= stamp-h[0] < self.config['baseline']]
        if not eligible:
            self.valid = False
            return False
        old_t, old, old_yaw = eligible[-1]
        dt = stamp-old_t
        # Local origin prevents UTM float rounding in the upstream box routine.
        origin = old.mean(axis=0)
        old_local = np.ascontiguousarray(old-origin)
        new_local = np.ascontiguousarray(points-origin)
        candidates = np.zeros((6, 2), dtype=np.float64)
        rc = self.lib.apollo_velocity_candidates(old_local, len(old), new_local,
            len(points), old_yaw, dt, candidates)
        if rc or not np.isfinite(candidates).all():
            self.valid = False
            return False
        # Use recent displacement for startup; use history thereafter. Candidate
        # disagreement is retained as uncertainty, not hidden by smoothing.
        reference = candidates[0] if not self.valid else 0.65*self.velocity + 0.35*candidates[0]
        selected = candidates[np.argmin(np.sum((candidates-reference)**2, axis=1))]
        # Check the same geometry measurement over multiple time baselines.
        # These observations are correlated: scatter raises uncertainty; it is
        # never divided by the number of baselines as if they were independent.
        estimates = [selected]
        for older_t, older, older_yaw in eligible[-3:-1]:
            older_origin = older.mean(axis=0)
            alternate = np.zeros((6, 2), dtype=np.float64)
            rc_alt = self.lib.apollo_velocity_candidates(
                np.ascontiguousarray(older-older_origin), len(older),
                np.ascontiguousarray(points-older_origin), len(points),
                older_yaw, stamp-older_t, alternate)
            if not rc_alt and np.isfinite(alternate).all():
                estimates.append(alternate[np.argmin(
                    np.sum((alternate-reference)**2, axis=1))])
        estimates = np.asarray(estimates)
        selected = np.median(estimates, axis=0)
        ratio = min(len(old), len(points))/float(max(len(old),len(points)))
        scatter = float(np.median(np.sum((candidates-selected)**2, axis=1)))
        baseline_scatter = (estimates-selected).T @ (estimates-selected) / len(estimates)
        variance = 2*self.config['position_sigma']**2/(dt*dt) + scatter
        variance /= max(ratio, 0.1)
        self.quality = float(ratio/(1.0+scatter))
        alpha = 1.0 if not self.valid else dt_frame/(dt_frame+self.config['smoothing'])
        self.velocity = (1-alpha)*self.velocity + alpha*selected
        self.covariance = np.eye(2)*variance + baseline_scatter
        self.valid = bool(np.linalg.norm(selected) <= self.config['max_speed'] and
                          self.std <= self.config['max_std'])
        return self.valid

    @property
    def direction_std(self):
        """First-order angular uncertainty (radians); undefined near zero speed."""
        speed2 = float(self.velocity @ self.velocity)
        if not self.valid or speed2 < 0.01:
            return float('inf')
        gradient = np.array([-self.velocity[1], self.velocity[0]]) / speed2
        return float(np.sqrt(max(0.0, gradient @ self.covariance @ gradient)))

def adopt_velocity(filter_object, observation):
    """Replace correlated velocity marginal; do not double-count as independent KF data."""
    if not observation.valid:
        return
    filter_object.x[2:4] = observation.velocity
    filter_object.P[2:4, 2:4] = observation.covariance
    filter_object.P[:2, 2:4] = 0.0
    filter_object.P[2:4, :2] = 0.0
