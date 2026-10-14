#!/usr/bin/env python3
"""Measured M3 motion evidence; no ROS, predicted boxes, or scan waiting."""
import math
import numpy as np
from scipy.spatial import cKDTree


class MotionEvidence:
    def __init__(self, fresh_sec=.35, gap_sec=.3, jump_speed=6.):
        self.fresh_sec, self.gap_sec, self.jump_speed = fresh_sec, gap_sec, jump_speed
        self.tracks = {}
        self.details = {}

    @staticmethod
    def surface(obs):
        a = np.asarray(obs.get('support_points_world', []), dtype=float)
        if a.ndim != 2 or a.shape[1] < 2:
            return np.empty((0, 2))
        a = a[np.isfinite(a[:, :2]).all(axis=1), :2]
        # Bounded association sample only; detector inputs remain untouched.
        return a[np.linspace(0, len(a)-1, min(len(a), 32), dtype=int)] if len(a) else a

    def reset(self, oid, reason):
        self.tracks.pop(oid, None)
        self.details[oid] = dict(reason=reason, source='none', independent_scans=0)

    def update(self, obs, now):
        oid = int(obs.get('id', -1))
        stamp = obs.get('last_seen')
        if stamp is None or not math.isfinite(float(stamp)):
            self.reset(oid, 'missing_measurement_stamp'); return
        stamp = float(stamp)
        age = now-stamp
        if not -.05 <= age <= self.fresh_sec:
            self.reset(oid, 'stale_measurement'); return
        pos = np.asarray(obs.get('observed_position_world', obs.get('position_world', [])), dtype=float)
        if pos.size < 2 or not np.isfinite(pos[:2]).all():
            self.reset(oid, 'missing_measured_position'); return
        pos = pos[:2]
        size = np.array([obs.get('length', 0.), obs.get('width', 0.)], dtype=float)
        surf = self.surface(obs)
        old = self.tracks.get(oid)
        source, reason = 'measured_centroid', 'first_measurement'
        hist, anchor = [], pos.copy()
        if old is not None:
            dt = stamp-old['stamp']
            if abs(dt) <= 1e-4:
                return  # repeated publication is not a new motion observation
            if dt < 0 or dt > self.gap_sec:
                self.reset(oid, 'time_reversal' if dt < 0 else 'observation_gap')
                old = None
            else:
                delta = pos-old['pos']
                reason = 'measured_motion'
                changed = np.max(np.abs(size-old['size'])) > .25
                if len(surf) >= 3 and len(old['surface']) >= 3:
                    prev = old['surface']
                    tree = cKDTree(prev)
                    dist0, _ = tree.query(surf)
                    fixed = (dist0 <= .12).mean()
                    shifted = surf-delta
                    dist, nearest = tree.query(shifted)
                    good = dist <= .12
                    if fixed >= .65 and np.linalg.norm(delta) > .12:
                        delta = np.zeros(2); reason = 'stationary_surface_shape_shift'
                    elif good.mean() >= .5:
                        delta = np.median(surf[good]-prev[nearest[good]], axis=0)
                        reason = 'matched_surface_motion'
                    elif changed:
                        self.reset(oid, 'unmatched_shape_change'); return
                    source = 'matched_surface' if reason != 'measured_motion' else source
                elif changed:
                    self.reset(oid, 'shape_change_without_surface'); return
                if np.linalg.norm(delta)/dt > self.jump_speed:
                    self.reset(oid, 'track_jump'); return
                hist = list(old['history'])
                anchor = old['anchor']+delta
        hist.append((stamp, float(anchor[0]), float(anchor[1])))
        hist = [q for q in hist if stamp-q[0] <= 1.5]
        self.tracks[oid] = dict(stamp=stamp, pos=pos.copy(), size=size, surface=surf,
                                history=hist, anchor=anchor)
        self.details[oid] = dict(reason=reason, source=source,
                                independent_scans=len(hist), observation_age_sec=age)

    def history(self, obs, now):
        oid = int(obs.get('id', -1))
        stamp = obs.get('last_seen')
        if stamp is None or not math.isfinite(float(stamp)) or not -.05 <= now-float(stamp) <= self.fresh_sec:
            self.reset(oid, 'stale_measurement' if stamp is not None else 'missing_measurement_stamp')
            return []
        if oid in self.details:
            self.details[oid]['observation_age_sec'] = now-float(stamp)
        return self.tracks.get(oid, {}).get('history', [])

    def prune(self, live):
        self.tracks = {k:v for k,v in self.tracks.items() if k in live}
        self.details = {k:v for k,v in self.details.items() if k in live}
