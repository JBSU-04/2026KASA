#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LiDAR 바운딩 박스 기반 Multi-Object Tracking (MOT) 모듈.

설계 전제
---------
- 검출(bbox)은 **map(전역) 좌표계**로 들어온다(라이다 노드에서 ego pose 로 TF 변환 후 퍼블리시).
  => 칼만 필터가 추정하는 (vx, vy) 는 **절대속도**다.
  => 상대속도 = 절대속도 - 자차속도벡터 (MultiObjectTracker.update 에 ego_v 를 넘겨 계산).
- 등속도(Constant Velocity) 모델. 상태 x = [px, py, vx, vy]^T, 측정 z = [px, py]^T.
- 프레임 간 매칭은 헝가리안(scipy.optimize.linear_sum_assignment) + 게이팅.

이 모듈은 ROS 에 의존하지 않는다(rospy import 없음) -> 단위 테스트/리플레이가 쉽다.
ROS 노드(frenet_avoidance_planner.py)는 콜백에서 detection 리스트만 만들어 넘기면 된다.
"""

import math

import numpy as np
from apollo_motion_bridge import MotionObservation, adopt_velocity

try:
    from scipy.optimize import linear_sum_assignment
except Exception:
    def linear_sum_assignment(cost_matrix):
        """Small dependency-free fallback.

        SciPy's Hungarian solver is preferred.  If SciPy is unavailable on the
        competition PC, use a deterministic greedy assignment so the tracker
        still runs instead of crashing.
        """
        cost = np.asarray(cost_matrix, dtype=float)
        pairs = [(cost[r, c], r, c)
                 for r in range(cost.shape[0])
                 for c in range(cost.shape[1])]
        pairs.sort(key=lambda item: item[0])

        rows, cols = [], []
        used_rows, used_cols = set(), set()
        for _, r, c in pairs:
            if r in used_rows or c in used_cols:
                continue
            used_rows.add(r)
            used_cols.add(c)
            rows.append(r)
            cols.append(c)

        return np.asarray(rows, dtype=int), np.asarray(cols, dtype=int)


def _unit_interval(value, default=0.0):
    """Return a finite value clipped to [0, 1] without trusting optional fields."""
    try:
        value = float(value)
    except (TypeError, ValueError):
        return float(default)
    if not np.isfinite(value):
        return float(default)
    return float(np.clip(value, 0.0, 1.0))


# ----------------- 추적 결과 컨테이너 -----------------
class TrackedObject(object):
    """Frenet 플래너로 넘길 추적 객체 1개의 스냅샷.

    좌표/속도는 모두 map 좌표계. (vx, vy)=절대속도, (rvx, rvy)=자차 기준 상대속도.
    """
    __slots__ = ('id', 'x', 'y', 'vx', 'vy', 'rvx', 'rvy',
                 'yaw', 'width', 'length', 'height', 'vehicle_confidence',
                 'is_vehicle', 'obstacle_confidence', 'yaw_confidence',
                 'hits', 'age', 'time_since_update', 'time_since_update_s',
                 'velocity_valid', 'velocity_std', 'observation_age')

    def __init__(self, tid, x, y, vx, vy, yaw, width, length, height,
                 vehicle_confidence, is_vehicle, yaw_confidence=1.0,
                 obstacle_confidence=0.0):
        self.id = tid
        self.x = x
        self.y = y
        self.vx = vx          # 절대속도 x
        self.vy = vy          # 절대속도 y
        self.rvx = vx         # 상대속도(자차 기준), update 시 갱신
        self.rvy = vy
        self.yaw = yaw
        self.width = width    # 차로 횡방향 크기(m)
        self.length = length  # 진행방향 크기(m)
        self.height = height
        self.vehicle_confidence = vehicle_confidence
        self.is_vehicle = is_vehicle
        self.obstacle_confidence = _unit_interval(obstacle_confidence)
        self.yaw_confidence = _unit_interval(yaw_confidence)
        self.hits = 0
        self.age = 0
        self.time_since_update = 0
        self.velocity_valid = False
        self.velocity_std = 10.0
        self.observation_age = 0.0

    @property
    def speed(self):
        return float(np.hypot(self.vx, self.vy))


# ----------------- 등속도 칼만 필터 (객체 1개) -----------------
class KalmanBoxTracker(object):
    """CV(Constant Velocity) 모델 칼만 필터. 객체 1개의 위치/속도를 추정.

    상태 x = [px, py, vx, vy]^T,  측정 z = [px, py]^T (map 좌표).
    dt 가변(센서 비주기성)에 대응하기 위해 predict(dt) 마다 F/Q 를 dt 로 재구성한다.
    """

    _next_id = 0

    def __init__(self, z, yaw=0.0, width=1.0, length=1.0, height=1.0,
                 vehicle_score=0.0, sigma_acc=2.0, sigma_meas=0.3,
                 yaw_alpha=0.25, vehicle_confidence_alpha=0.35,
                 yaw_confidence=1.0, yaw_confidence_alpha=0.35,
                 yaw_update_min_confidence=0.35, obstacle_score=0.0):
        # 트랙 고유 ID 발급(헝가리안 매칭으로 프레임 간 유지)
        self.id = KalmanBoxTracker._next_id
        KalmanBoxTracker._next_id += 1

        # 상태/공분산 초기화 (속도는 0, 불확실성 크게)
        self.x = np.array([z[0], z[1], 0.0, 0.0], dtype=float)
        self.P = np.diag([1.0, 1.0, 100.0, 100.0]).astype(float)

        # 측정 행렬 H: 위치만 관측
        self.H = np.array([[1.0, 0.0, 0.0, 0.0],
                           [0.0, 1.0, 0.0, 0.0]], dtype=float)
        # 측정 잡음 R: 라이다 위치 분산
        self.R = np.eye(2) * (sigma_meas ** 2)
        self._sigma_acc = sigma_acc  # 프로세스(가속도) 잡음 표준편차

        # 바운딩 박스 부가정보(추적 대상은 위치지만 플래너엔 크기/자세도 필요)
        self.yaw = self._wrap_angle(yaw)
        self.width = width
        self.length = length
        self.height = height
        self.yaw_alpha = float(np.clip(yaw_alpha, 0.0, 1.0))
        self.yaw_confidence = _unit_interval(yaw_confidence)
        self.yaw_confidence_alpha = float(
            np.clip(yaw_confidence_alpha, 0.0, 1.0))
        self.yaw_update_min_confidence = float(
            np.clip(yaw_update_min_confidence, 0.0, 0.99))
        self.vehicle_confidence = float(np.clip(vehicle_score, 0.0, 1.0))
        self.vehicle_confidence_alpha = float(
            np.clip(vehicle_confidence_alpha, 0.0, 1.0))
        # 형상과 무관한 '실재' 증거. 차량성과 같은 EMA 상수를 쓴다.
        self.obstacle_confidence = float(np.clip(obstacle_score, 0.0, 1.0))

        # Vehicle class is a track-level state, not a frame-level geometry
        # result.  A VLP-16 vehicle often becomes a partial chord at long
        # range; keep the established class through those sparse scans and
        # release it only after sustained negative evidence.
        self.vehicle_confirmed = False
        self.vehicle_release_streak = 0

        # 라이프사이클 카운터
        self.hits = 1                 # 매칭된 총 횟수
        self.age = 0                  # 생성 후 경과 프레임
        self.time_since_update = 0    # 마지막 매칭 이후 미관측 프레임 수
        # 프레임 수는 센서 Hz 에 따라 의미가 달라진다(8.2Hz 에서 2프레임=0.24s,
        # 15Hz 에서는 0.13s). 같은 가림 시간에도 Hz 가 높으면 트랙이 더 빨리
        # 죽어 ID switch 가 늘었다 - 실제 경과 시간으로 함께 관리한다.
        self.time_since_update_s = 0.0

    def _F(self, dt):
        return np.array([[1.0, 0.0, dt,  0.0],
                         [0.0, 1.0, 0.0, dt],
                         [0.0, 0.0, 1.0, 0.0],
                         [0.0, 0.0, 0.0, 1.0]], dtype=float)

    def _Q(self, dt):
        # 등속도 모델의 표준 가속도 잡음(white-noise acceleration) Q.
        q = self._sigma_acc ** 2
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt3 * dt
        return q * np.array([[dt4 / 4, 0,        dt3 / 2, 0],
                             [0,        dt4 / 4, 0,        dt3 / 2],
                             [dt3 / 2, 0,        dt2,      0],
                             [0,        dt3 / 2, 0,        dt2]], dtype=float)

    def predict(self, dt):
        """다음 프레임으로 상태 전파. 매칭 전 호출."""
        if dt <= 0.0:
            dt = 1e-3
        F = self._F(dt)
        self.x = F.dot(self.x)
        self.P = F.dot(self.P).dot(F.T) + self._Q(dt)
        self.age += 1
        self.time_since_update += 1
        self.time_since_update_s += dt
        return self.x[:2].copy()  # 예측 위치(게이팅/헝가리안 비용에 사용)

    @staticmethod
    def _wrap_angle(a):
        return math.atan2(math.sin(a), math.cos(a))

    @staticmethod
    def _closest_yaw_equivalent(yaw, reference):
        """PCA OBB yaw has a 180-degree ambiguity; choose the side closest to history."""
        y0 = KalmanBoxTracker._wrap_angle(yaw)
        y1 = KalmanBoxTracker._wrap_angle(yaw + math.pi)
        if abs(KalmanBoxTracker._wrap_angle(y0 - reference)) <= \
           abs(KalmanBoxTracker._wrap_angle(y1 - reference)):
            return y0
        return y1

    def update(self, z, yaw=None, width=None, length=None, height=None,
               vehicle_score=None, yaw_confidence=None, obstacle_score=None):
        """측정으로 보정. 헝가리안 매칭 성공 시 호출."""
        z = np.asarray(z, dtype=float)
        y = z - self.H.dot(self.x)                      # 잔차
        S = self.H.dot(self.P).dot(self.H.T) + self.R   # 잔차 공분산
        K = self.P.dot(self.H.T).dot(np.linalg.inv(S))  # 칼만 이득
        self.x = self.x + K.dot(y)
        I = np.eye(4)
        self.P = (I - K.dot(self.H)).dot(self.P)

        # 바운딩 박스 부가정보 갱신(EMA 로 떨림 완화).
        # PCA 장축이 원형/희소 군집에서 불안정하면 yaw를 거의 갱신하지 않는다.
        measured_yaw_confidence = self.yaw_confidence
        if yaw_confidence is not None and np.isfinite(yaw_confidence):
            measured_yaw_confidence = _unit_interval(yaw_confidence)
            conf_alpha = self.yaw_confidence_alpha
            self.yaw_confidence = (
                (1.0 - conf_alpha) * self.yaw_confidence +
                conf_alpha * measured_yaw_confidence)
        if yaw is not None and np.isfinite(yaw):
            yaw = self._closest_yaw_equivalent(float(yaw), self.yaw)
            usable = max(0.0, measured_yaw_confidence - self.yaw_update_min_confidence)
            usable /= max(1.0 - self.yaw_update_min_confidence, 1e-6)
            alpha = self.yaw_alpha * usable
            self.yaw = self._wrap_angle(self.yaw + alpha * self._wrap_angle(yaw - self.yaw))
        if width is not None:
            self.width = 0.5 * self.width + 0.5 * width
        if length is not None:
            self.length = 0.5 * self.length + 0.5 * length
        if height is not None:
            self.height = 0.5 * self.height + 0.5 * height
        if vehicle_score is not None and np.isfinite(vehicle_score):
            score = float(np.clip(vehicle_score, 0.0, 1.0))
            alpha = self.vehicle_confidence_alpha
            self.vehicle_confidence = (1.0 - alpha) * self.vehicle_confidence + alpha * score
        if obstacle_score is not None and np.isfinite(obstacle_score):
            score = float(np.clip(obstacle_score, 0.0, 1.0))
            alpha = self.vehicle_confidence_alpha
            self.obstacle_confidence = (
                (1.0 - alpha) * self.obstacle_confidence + alpha * score)

        self.hits += 1
        self.time_since_update = 0
        self.time_since_update_s = 0.0

    @property
    def position(self):
        return self.x[0], self.x[1]

    @property
    def velocity(self):
        return self.x[2], self.x[3]


# ----------------- 다중 객체 추적기 (헝가리안) -----------------
class MultiObjectTracker(object):
    """프레임마다 detections 를 받아 트랙을 예측-매칭-갱신하고 확정 트랙 리스트를 반환.

    detections: list of dict
        {'x':float,'y':float,'yaw':float,'width':float,'length':float,
         'height':float,'vehicle_score':float,'yaw_confidence':float,
         'obstacle_score':float}
        (좌표/자세 모두 map 좌표계)
    """

    def __init__(self,
                 gate_dist=3.0,       # [m] 게이팅: 이보다 먼 예측-검출 쌍은 매칭 금지
                 max_age=5,           # 미관측이 이만큼 누적되면 트랙 삭제
                 min_hits=3,          # 확정(컨펌)에 필요한 최소 매칭 횟수
                 sigma_acc=2.0,
                 sigma_meas=0.3,
                 yaw_alpha=0.25,
                 vehicle_confidence_alpha=0.35,
                 yaw_confidence_alpha=0.35,
                 yaw_update_min_confidence=0.35,
                 yaw_maturity_hits=3,
                 yaw_maturity_confidence=0.55,
                 association_size_weight=0.35,
                 association_yaw_weight=0.75,
                 association_yaw_min_confidence=0.55,
                 association_feature_min_hits=3,
                 vehicle_confirm_score=0.60,
                 vehicle_confirm_hits=3,
                 vehicle_release_score=0.25,
                 vehicle_release_frames=7,
                 max_coast_age=2, max_coast_sec=0.8, max_track_age_sec=1.0,
                 motion_backend="legacy", motion_config=None,
                 motion_velocity_coast_sec=0.6,
                 motion_velocity_coast_std_growth=1.5):
        self.motion_backend = motion_backend
        self.motion_config = motion_config or {}
        self.motion_time = 0.0
        self.gate_dist = gate_dist
        self.max_age = max_age
        self.min_hits = min_hits
        self.max_coast_age = max(0, int(max_coast_age))
        # 시간 기준 수명. 프레임 기준과 OR 로 적용한다.
        #   실측(2026-09-16 백, 8.2 Hz): 같은 ID 재관측 공백 p50 0.48 s,
        #   p90 0.99 s. max_coast_age=2 는 0.24 s 라 공백의 0% 만 커버해
        #   한 번만 놓쳐도 새 ID 가 됐고, 새 트랙은 velocity_std=45 로
        #   시작해 속도 수렴까지 1.4 s 가 걸렸다(그 사이 27m -> 20m 접근).
        self.max_coast_sec = float(max_coast_sec)
        self.max_track_age_sec = float(max_track_age_sec)
        self.sigma_acc = sigma_acc
        self.sigma_meas = sigma_meas
        self.yaw_alpha = float(np.clip(yaw_alpha, 0.0, 1.0))
        self.vehicle_confidence_alpha = float(
            np.clip(vehicle_confidence_alpha, 0.0, 1.0))
        self.yaw_confidence_alpha = float(
            np.clip(yaw_confidence_alpha, 0.0, 1.0))
        self.yaw_update_min_confidence = float(
            np.clip(yaw_update_min_confidence, 0.0, 0.99))
        self.yaw_maturity_hits = max(1, int(yaw_maturity_hits))
        self.yaw_maturity_confidence = _unit_interval(yaw_maturity_confidence)
        self.association_size_weight = max(0.0, float(association_size_weight))
        self.association_yaw_weight = max(0.0, float(association_yaw_weight))
        self.association_yaw_min_confidence = float(
            np.clip(association_yaw_min_confidence, 0.0, 1.0))
        self.association_feature_min_hits = max(1, int(association_feature_min_hits))
        self.vehicle_confirm_score = float(np.clip(vehicle_confirm_score, 0.0, 1.0))
        self.vehicle_confirm_hits = max(1, int(vehicle_confirm_hits))
        self.vehicle_release_score = float(np.clip(
            vehicle_release_score, 0.0, self.vehicle_confirm_score))
        self.vehicle_release_frames = max(1, int(vehicle_release_frames))
        self.motion_velocity_coast_sec = max(0.0, float(motion_velocity_coast_sec))
        self.motion_velocity_coast_std_growth = max(
            0.0, float(motion_velocity_coast_std_growth))
        self.trackers = []   # list[KalmanBoxTracker]
        self.frame_count = 0

    def _update_vehicle_state(self, track):
        """Apply enter/exit hysteresis after an actual matched observation."""
        if not track.vehicle_confirmed:
            if (track.hits >= self.vehicle_confirm_hits and
                    track.vehicle_confidence >= self.vehicle_confirm_score):
                track.vehicle_confirmed = True
                track.vehicle_release_streak = 0
            return

        if track.vehicle_confidence < self.vehicle_release_score:
            track.vehicle_release_streak += 1
            if track.vehicle_release_streak >= self.vehicle_release_frames:
                track.vehicle_confirmed = False
                track.vehicle_release_streak = 0
        else:
            track.vehicle_release_streak = 0

    def _update_motion(self, track, detection):
        if self.motion_backend != 'apollo_measurement':
            return
        if not hasattr(track, 'motion_observation'):
            track.motion_observation = MotionObservation('highway', **self.motion_config)
        motion = track.motion_observation
        motion.update(detection.get('points_world', np.empty((0,3))),
                      self.motion_time, detection.get('yaw', 0.0))
        adopt_velocity(track, motion)
        if motion.valid:
            track.motion_last_valid_time = self.motion_time
            track.motion_last_valid_velocity = motion.velocity.copy()
            track.motion_last_valid_std = motion.std

    # --- 헝가리안 데이터 연관 ---
    @staticmethod
    def _axial_yaw_distance(a, b):
        """PCA OBB 장축의 180도 모호성을 제거한 각도 차이 [0, pi/2]."""
        delta = abs(KalmanBoxTracker._wrap_angle(float(a) - float(b)))
        return min(delta, abs(math.pi - delta))

    def _associate(self, predicted, detections):
        """위치 게이트 안에서 크기/신뢰 가능한 PCA 장축을 비용에 추가해 ID 교환을 줄인다."""
        N, M = len(predicted), len(detections)
        if N == 0:
            return [], [], list(range(M))
        if M == 0:
            return [], list(range(N)), []

        # 비용행렬 = 예측위치-검출위치 유클리드 거리
        pred = np.asarray(predicted, dtype=float).reshape(N, 2)
        det_xy = [(d['x'], d['y']) for d in detections]
        det = np.asarray(det_xy, dtype=float).reshape(M, 2)
        position_cost = np.linalg.norm(pred[:, None, :] - det[None, :, :], axis=2)
        cost = position_cost.copy()
        for trk_i, trk in enumerate(self.trackers):
            if trk.hits < self.association_feature_min_hits:
                continue
            trk_length = max(float(trk.length), 0.1)
            trk_width = max(float(trk.width), 0.1)
            for det_i, detection in enumerate(detections):
                det_length = max(float(detection.get('length', trk_length)), 0.1)
                det_width = max(float(detection.get('width', trk_width)), 0.1)
                size_delta = 0.5 * (
                    abs(det_length - trk_length) / max(det_length, trk_length, 0.1) +
                    abs(det_width - trk_width) / max(det_width, trk_width, 0.1))
                cost[trk_i, det_i] += self.association_size_weight * min(size_delta, 2.0)

                detection_confidence = _unit_interval(
                    detection.get('yaw_confidence', 0.0))
                yaw_confidence = min(_unit_interval(trk.yaw_confidence),
                                     detection_confidence)
                detection_yaw = detection.get('yaw', None)
                yaw_confidence_effective = np.clip(
                    (yaw_confidence - self.association_yaw_min_confidence) /
                    max(1.0 - self.association_yaw_min_confidence, 1e-6),
                    0.0, 1.0)
                if (yaw_confidence_effective > 0.0 and
                        detection_yaw is not None and np.isfinite(detection_yaw)):
                    yaw_delta = self._axial_yaw_distance(trk.yaw, detection_yaw)
                    cost[trk_i, det_i] += (
                        self.association_yaw_weight * yaw_confidence_effective *
                        (yaw_delta / (0.5 * math.pi)))

        # 게이트 초과 쌍은 매칭되지 않도록 큰 비용 부여
        big = self.gate_dist * 10.0 + 1.0
        cost_gated = np.where(position_cost > self.gate_dist, big, cost)

        row_idx, col_idx = linear_sum_assignment(cost_gated)

        matches, um_trk, um_det = [], [], []
        matched_trk, matched_det = set(), set()
        for r, c in zip(row_idx, col_idx):
            if position_cost[r, c] > self.gate_dist:   # 위치 게이트 밖 -> 매칭 무효
                continue
            matches.append((r, c))
            matched_trk.add(r)
            matched_det.add(c)
        um_trk = [r for r in range(N) if r not in matched_trk]
        um_det = [c for c in range(M) if c not in matched_det]
        return matches, um_trk, um_det

    def update(self, detections, dt, ego_vx=0.0, ego_vy=0.0):
        """한 프레임 처리. 확정 트랙들의 TrackedObject 리스트 반환.

        ego_vx, ego_vy: 자차 절대속도 벡터(map 좌표). 상대속도 계산용.
        """
        if not np.isfinite(dt) or dt <= 0.0:
            return []
        self.motion_time += dt
        self.frame_count += 1

        # 1) 전체 트랙 예측(dt 전파)
        predicted = [trk.predict(dt) for trk in self.trackers]
        # 2) 헝가리안 매칭
        matches, um_trk, um_det = self._associate(predicted, detections)

        # 3) 매칭된 트랙 보정
        for trk_i, det_i in matches:
            d = detections[det_i]
            self.trackers[trk_i].update(
                (d['x'], d['y']),
                yaw=d.get('yaw', None),
                width=d.get('width', None),
                length=d.get('length', None),
                height=d.get('height', None),
                vehicle_score=d.get('vehicle_score', None),
                yaw_confidence=d.get('yaw_confidence', None),
                obstacle_score=d.get('obstacle_score', None))
            self._update_vehicle_state(self.trackers[trk_i])
            self._update_motion(self.trackers[trk_i], d)

        # 4) 미매칭 검출 -> 새 트랙 생성
        for det_i in um_det:
            d = detections[det_i]
            self.trackers.append(KalmanBoxTracker(
                (d['x'], d['y']),
                yaw=d.get('yaw', 0.0),
                width=d.get('width', 1.0),
                length=d.get('length', 1.0),
                height=d.get('height', 1.0),
                vehicle_score=d.get('vehicle_score', 0.0),
                sigma_acc=self.sigma_acc, sigma_meas=self.sigma_meas,
                yaw_alpha=self.yaw_alpha,
                vehicle_confidence_alpha=self.vehicle_confidence_alpha,
                yaw_confidence=d.get('yaw_confidence', 0.0),
                yaw_confidence_alpha=self.yaw_confidence_alpha,
                yaw_update_min_confidence=self.yaw_update_min_confidence,
                obstacle_score=d.get('obstacle_score', 0.0)))
            self._update_vehicle_state(self.trackers[-1])
            self._update_motion(self.trackers[-1], d)

        # 5) 오래 미관측된 트랙 삭제
        # 프레임 수와 시간 중 더 관대한 쪽으로 유지한다. 센서 Hz 가 변해도
        # 같은 가림 시간을 견디게 하려는 것이다(max_track_age_sec).
        self.trackers = [
            t for t in self.trackers
            if t.time_since_update <= self.max_age
            or t.time_since_update_s <= self.max_track_age_sec]

        # 6) Confirmed tracks only.  Do not give startup-only, one-scan boxes
        # special treatment: a vehicle first seen later must obey the same
        # confirmation rule, and SMPC already requires at least two hits.
        out = []
        for t in self.trackers:
            confirmed = t.hits >= self.min_hits
            is_vehicle = t.vehicle_confirmed
            # 확정된 차량은 1~2프레임 순간 미관측 시 칼만 예측 위치로 유지(coasting)하여
            # 근거리 지면필터/반사각 노이즈로 인한 급가속 방지. 미확정 객체는 미관측 즉시 제외.
            allowed_miss = self.max_coast_age if is_vehicle else 0
            allowed_miss_s = self.max_coast_sec if is_vehicle else 0.0
            coasting_ok = (t.time_since_update <= allowed_miss
                           or t.time_since_update_s <= allowed_miss_s)
            if not confirmed or not coasting_ok:
                continue
            px, py = t.position
            vx, vy = t.velocity
            is_vehicle = t.vehicle_confirmed
            # Position/size are usable after `min_hits`, but two observations
            # do not yet provide a stable motion heading.  Publish q=0 during
            # that yaw warm-up so downstream planning uses its lane prior
            # without discarding the obstacle itself.
            yaw_ready = (t.hits >= self.yaw_maturity_hits and
                         t.yaw_confidence >= self.yaw_maturity_confidence)
            published_yaw_confidence = t.yaw_confidence if yaw_ready else 0.0
            obj = TrackedObject(t.id, px, py, vx, vy, t.yaw, t.width, t.length,
                                t.height, t.vehicle_confidence, is_vehicle,
                                published_yaw_confidence,
                                t.obstacle_confidence)
            obj.rvx = vx - ego_vx   # 상대속도 = 절대 - 자차
            obj.rvy = vy - ego_vy
            obj.hits = t.hits
            obj.age = t.age
            obj.time_since_update = t.time_since_update
            motion = getattr(t, 'motion_observation', None)
            obj.observation_age = (max(0.0, self.motion_time-motion.stamp)
                                   if motion is not None and motion.stamp is not None
                                   else t.time_since_update*dt)
            if motion is not None:
                last_valid_time = getattr(t, 'motion_last_valid_time', None)
                velocity_age = (self.motion_time - last_valid_time
                                if last_valid_time is not None else float('inf'))
                coast_valid = (not motion.valid and
                               velocity_age <= self.motion_velocity_coast_sec and
                               t.time_since_update == 0)
                if coast_valid:
                    vx, vy = getattr(t, 'motion_last_valid_velocity', (vx, vy))
                    obj.vx, obj.vy = float(vx), float(vy)
                    obj.rvx = obj.vx - ego_vx
                    obj.rvy = obj.vy - ego_vy
                    obj.velocity_std = (getattr(t, 'motion_last_valid_std', motion.std) +
                                        self.motion_velocity_coast_std_growth * velocity_age)
                else:
                    obj.velocity_std = motion.std
                obj.velocity_valid = bool(
                    (motion.valid or coast_valid) and obj.observation_age <= 0.3)
            else:
                obj.velocity_std = float(
                    np.sqrt(np.linalg.eigvalsh(t.P[2:4,2:4]).max()))
                obj.velocity_valid = bool(t.hits >= 3 and obj.observation_age <= 0.3)
            out.append(obj)
        return out
