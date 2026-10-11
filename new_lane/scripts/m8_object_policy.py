"""M8-only observed-surface handoff. A missing scan is not stationary evidence."""
import math
import numpy as np
from scipy.spatial import cKDTree
from tunnel_scene_policy import tag_reference_corridor


class SurfaceMotion:
    """M8 admission from independent observed surfaces, not a growing box centre.

    Opposite faces must translate together before centroid velocity can certify
    motion. Static admission also needs overlap of real returns over 0.2 s.
    No missing or duplicate scan can complete either confirmation.
    """
    def __init__(self, hold=True):
        self.hold = hold
        self.static_certificate = False
        self.translation_hits = 0
        self.history = []
        self.moving_hits = 0
        self.reason = 'surface_history_short'

    def update(self, points, stamp, measured_velocity, velocity_valid, threshold):
        points = np.asarray(points, dtype=np.float64)
        if points.ndim != 2 or len(points) < 3 or not np.isfinite(points[:, :2]).all():
            self.history = []; self.moving_hits = 0; self.static_certificate = False; self.translation_hits = 0
            self.reason = 'insufficient_surface'; return 'unknown', None
        points = points[:, :2]
        if self.history and stamp <= self.history[-1][0]:
            self.reason = 'duplicate_surface'; return 'unknown', None
        if self.history and stamp-self.history[-1][0] > .35:
            self.history = []; self.moving_hits = 0; self.static_certificate = False; self.translation_hits = 0
        self.history = [(t, p) for t, p in self.history if stamp-t <= .5]
        self.history.append((stamp, points.copy()))
        earlier = [(t, p) for t, p in self.history if stamp-t >= .2-1e-6]
        if len(self.history) < 3 or not earlier:
            self.reason = 'surface_history_short'; return 'unknown', None
        t, old = earlier[-1]; dt = stamp-t
        lo = points.min(axis=0)-old.min(axis=0)
        hi = points.max(axis=0)-old.max(axis=0)
        displacement = np.where(lo*hi > 0., np.sign(lo)*np.minimum(abs(lo),abs(hi)), 0.)
        rigid = displacement/dt
        velocity = np.asarray(measured_velocity, dtype=float)
        moving = (velocity_valid and velocity.shape == (2,) and np.isfinite(velocity).all()
                  and np.linalg.norm(velocity) > threshold
                  and np.linalg.norm(rigid) > max(.18, .25*np.linalg.norm(velocity))
                  and float(np.dot(rigid, velocity)) > 0.)
        self.translation_hits = self.translation_hits+1 if np.linalg.norm(rigid)>threshold else 0
        if self.translation_hits>=2: self.static_certificate=False
        self.moving_hits = self.moving_hits+1 if moving else 0
        if moving:
            if self.moving_hits >= 2: self.static_certificate = False
            self.reason = 'coherent_opposite_faces'
            return ('dynamic' if self.moving_hits >= 2 else 'unknown'), rigid
        # One-sided new faces or a partial view can move the centroid without
        # moving the previously visible surface. Preserve every observed point.
        forward = cKDTree(old).query(points, k=1)[0]
        backward = cKDTree(points).query(old, k=1)[0]
        overlap = max(float(np.mean(forward <= .08)), float(np.mean(backward <= .08)))
        stable = (min(len(points),len(old)) >= 3 and overlap >= .6
                  and np.linalg.norm(rigid) <= .18)
        if stable: self.static_certificate = True
        held = self.hold and self.static_certificate
        self.reason = 'repeated_stationary_surface' if stable else 'coherent_uncertain_translation' if self.translation_hits>=2 else 'stationary_certificate_partial_view' if held else 'surface_unresolved'
        return ('static', np.zeros(2)) if stable or held else ('unknown', None)


class StationaryHandoff:
    def __init__(self, seconds=5., speed=.18, displacement=.3, max_gap=.35):
        self.seconds, self.speed, self.displacement, self.max_gap = seconds, speed, displacement, max_gap
        self.states = {}
        self.version = None

    def update(self, root, stamp, route, version):
        if version != self.version:
            self.states.clear()
            self.version = version
        # Use observed support only, never the retained box/parent extent.
        objects = root.get('collision_objects', root.get('obstacles', []))
        seen = set()
        for o in objects:
            oid = (o.get('source', 'lidar_only'), int(o.get('observed_track_id', o['id'])))
            if oid in seen:
                continue
            seen.add(oid)
            pts = np.asarray(o.get('support_points_world', []), float)
            fresh = abs(float(o.get('last_seen', -1e9)) - stamp) < 1e-5
            if pts.ndim != 2 or pts.shape[1] < 2 or len(pts) < 3 or not fresh:
                if oid in self.states:
                    self.states[oid]['since'] = None
                continue
            pts = pts[:, :2]
            if not np.isfinite(pts).all():
                continue
            state = self.states.setdefault(oid, dict(ever=False, since=None, last=-1e9, points=None,
                                                     anchor=None, converted=False, moving=0))
            if stamp <= state['last']:
                continue
            dt = stamp - state['last']
            velocity = np.asarray(o.get('observed_velocity', [0., 0.]), float)
            consistent = bool(o.get('observed_motion_valid', False)) and velocity.shape == (2,) and np.isfinite(velocity).all()
            moving = consistent and np.linalg.norm(velocity) > .3
            state['moving'] = state['moving'] + 1 if moving else 0
            if o.get('dynamic_confirmed', False) or o.get('motion_state') == 'dynamic':
                state['ever'] = True
            if state['moving'] >= 2:
                state.update(converted=False, since=None, anchor=None)
            stationary = False
            if state['points'] is not None and 0 < dt <= self.max_gap:
                dist = cKDTree(state['points']).query(pts, k=1)[0]
                matched = dist <= self.displacement
                # New visible faces may grow; they do not move the old surface.
                stationary = (matched.sum() >= 3 and matched.mean() >= .3 and
                              np.median(dist[matched]) / dt <= self.speed and
                              not moving and (not consistent or np.linalg.norm(velocity) <= self.speed))
                if stationary and state['anchor'] is not None:
                    a = cKDTree(state['anchor']).query(pts[matched], k=1)[0]
                    stationary = np.median(a) <= self.displacement
            if state['ever'] and stationary:
                if state['since'] is None:
                    state['since'], state['anchor'] = state['last'], state['points'].copy()
            else:
                state['since'] = None
                if not state['converted']:
                    state['anchor'] = None
            o['ever_dynamic'] = state['ever']
            o['m8_stationary_seconds'] = 0. if state['since'] is None else stamp - state['since']
            probe = dict(o, occupied_cells_world=pts.tolist())
            tag_reference_corridor({'collision_objects': [probe]}, route)
            blocked = probe.get('in_trigger_corridor', False)
            if state['ever'] and o.get('confirmed',False) and stationary and blocked and o['m8_stationary_seconds'] >= self.seconds:
                state['converted'] = True
            if state['converted'] and state['moving'] < 2:
                o.update(m8_restatic=True, motion_state='static', is_dynamic=False, dynamic_confirmed=False,
                         static_certified=True, static_occupancy=True, observed_motion_valid=False,
                         velocity_valid=False, velocity=[0., 0.], observed_velocity=[0., 0.], speed=0.,
                         dynamic_streak=0, m8_handoff_reason='observed_stationary_blocker_5s')
            else:
                o['m8_restatic'] = False
                # A previously moving blocker must complete the full five
                # seconds; the detector's shorter stillness window is not an
                # independent handoff permission.
                if state['ever'] and o.get('motion_state') == 'static':
                    o.update(motion_state='unknown', static_certified=False,
                             static_occupancy=False, m8_handoff_reason='awaiting_stationary_blocker_5s')
            state.update(last=stamp, points=pts.copy())
        for oid, state in list(self.states.items()):
            if stamp - state['last'] > self.max_gap:
                state['since'] = None
            if stamp - state['last'] > 10.:
                del self.states[oid]
        return root
