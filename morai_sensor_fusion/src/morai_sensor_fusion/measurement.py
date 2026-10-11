"""ROS-independent measurement math used by the MORAI adapter."""

import collections
import math


class GpsIntegrityMonitor:
    """Detect a timestamp-alive GPS stream whose position has frozen.

    MORAI can keep publishing a valid status and fresh timestamp inside a
    shaded area while repeating one coordinate.  Message-age health checks
    cannot distinguish that from a real fix.  This monitor compares each GPS
    displacement with independently measured vehicle speed.
    """

    NORMAL = 'normal'
    FROZEN = 'frozen'
    REACQUIRING = 'reacquiring'

    def __init__(self, min_speed=1.0, freeze_ratio=0.15,
                 freeze_abs_step=0.03, freeze_hits=4,
                 recover_min_ratio=0.25, recover_max_ratio=2.5,
                 recover_margin=0.25, recover_hits=5, freeze_distance=0.0):
        self.min_speed = float(min_speed)
        self.freeze_ratio = float(freeze_ratio)
        self.freeze_abs_step = float(freeze_abs_step)
        self.freeze_hits_required = int(freeze_hits)
        self.recover_min_ratio = float(recover_min_ratio)
        self.recover_max_ratio = float(recover_max_ratio)
        self.recover_margin = float(recover_margin)
        self.recover_hits_required = int(recover_hits)
        # ★ 2026-09-30 GPS 좌표 양자화 대응.
        #   MORAI GPS 위경도는 1/600000 deg(~0.185 m) 단위다. 1~2 m/s 로 출발하면
        #   30 Hz 메시지 3~5 개 동안 같은 좌표가 나와, 메시지마다 '이동 0' 을 세던
        #   판정이 출발할 때마다 FROZEN 을 냈다. 복귀도 메시지 단위라 저속에선
        #   안 됐고, 그대로 서면 영원히 FROZEN -> localization invalid -> 회전교차로
        #   FAILSAFE 교착(12:34 백 109.5 s). M2 출발(12:09/12:07)도 같은 원인.
        #   좌표가 '마지막으로 바뀐 뒤 달린 거리'(속도 적분)로 판단한다:
        #     - 동결: 좌표가 안 바뀐 채 freeze_distance 이상 달렸을 때부터 센다.
        #     - 복귀: 좌표가 바뀔 때 그 변위가 그 사이 달린 거리와 맞으면 센다.
        #   freeze_distance=0 이면 예전(메시지 단위) 판정과 같다.
        self.freeze_distance = max(0.0, float(freeze_distance))
        self.mode = self.NORMAL
        self.freeze_hits = 0
        self.recover_hits = 0
        self.previous = None
        self.travelled = 0.0   # 마지막 좌표 변화 이후 속도로 적분한 거리 [m]

    def update(self, stamp_sec, x, y, speed):
        current = (float(stamp_sec), float(x), float(y))
        previous = self.previous
        self.previous = current
        if previous is None:
            self.travelled = 0.0
            return self.mode
        dt = current[0] - previous[0]
        if not 0.01 <= dt <= 0.35:
            self.freeze_hits = 0
            self.recover_hits = 0
            self.travelled = 0.0
            return self.mode

        distance = math.hypot(current[1] - previous[1],
                              current[2] - previous[2])
        moving = abs(float(speed)) >= self.min_speed
        self.travelled += abs(float(speed)) * dt
        # 좌표 변화 여부. 예전 판정(freeze_distance=0)에서는 모든 메시지를
        # 하나의 구간으로 본다.
        changed = distance > 1.0e-3 or self.freeze_distance <= 0.0
        expected = self.travelled
        if changed:
            self.travelled = 0.0
        long_enough = expected >= self.freeze_distance

        if self.mode == self.NORMAL:
            if changed:
                frozen_step = (moving and long_enough and distance <= max(
                    self.freeze_abs_step, self.freeze_ratio * expected))
                self.freeze_hits = self.freeze_hits + 1 if frozen_step else 0
            elif moving and long_enough:
                self.freeze_hits += 1
            if self.freeze_hits >= self.freeze_hits_required:
                self.mode = self.FROZEN
                self.recover_hits = 0
        elif self.mode == self.FROZEN:
            if changed:
                recovery_step = (expected > 0.05 and
                                 (moving or self.freeze_distance > 0.0) and
                                 distance >= self.recover_min_ratio * expected and
                                 distance <= self.recover_max_ratio * expected
                                 + self.recover_margin)
                self.recover_hits = self.recover_hits + 1 if recovery_step else 0
            elif moving and long_enough:
                self.recover_hits = 0
            if self.recover_hits >= self.recover_hits_required:
                self.mode = self.REACQUIRING
        return self.mode

    def mark_normal(self):
        self.mode = self.NORMAL
        self.freeze_hits = 0
        self.recover_hits = 0

    def mark_outage(self):
        """Enter dead reckoning after an invalid fix or a GPS timeout.

        Repeated-coordinate failures already reach FROZEN through ``update``.
        A real MORAI shaded-area dropout reports status <= 0 instead, so it
        must enter the same recovery path explicitly.
        """
        self.mode = self.FROZEN
        self.freeze_hits = self.freeze_hits_required
        self.recover_hits = 0
        self.previous = None
        self.travelled = 0.0

    def mark_reacquiring(self):
        if self.mode == self.FROZEN:
            self.mode = self.REACQUIRING
            self.recover_hits = self.recover_hits_required

    def suspected_freeze(self, hold_distance):
        """Return true while a moving stream is repeating one coordinate.

        This is an early guard for the EKF input, not a mode transition.  It
        lets normal quantized fixes resume without declaring an outage, while
        preventing a genuinely frozen coordinate from pulling the EKF
        backwards during the confirmation window.
        """
        return (self.mode == self.NORMAL and self.previous is not None and
                self.travelled >= max(0.0, float(hold_distance)))


class CourseBiasEstimator:
    """Estimate velocity-course minus body-yaw while GPS is trustworthy.

    ``motion_beta`` is the bicycle-model sideslip contribution at the sample.
    The filtered result is the remaining mounting/model bias.  A distance
    baseline avoids differentiating centimetre-level GPS samples.
    """

    def __init__(self, min_speed=2.0, min_distance=1.5, max_dt=1.5,
                 alpha=0.18, max_abs_bias=math.radians(5.0),
                 hold_samples=40, hold_max_std=math.radians(0.30)):
        self.min_speed = float(min_speed)
        self.min_distance = float(min_distance)
        self.max_dt = float(max_dt)
        self.alpha = float(alpha)
        self.max_abs_bias = float(max_abs_bias)
        self.anchor = None
        self.bias = 0.0
        self.initialized = False
        # --- 음영구간 진입 시 '고정할 값'을 따로 만든다 ---
        #
        # bias 는 EMA(alpha=0.18)라 마지막 샘플에 끌려간다. GPS 가 끊기는
        # 순간의 값 하나로 고정하면 그 한 샘플의 잡음이 전체 음영구간
        # 궤적을 기울인다.
        #   실측(2026-09-17 15:35 백): 학습 구간 전체 평균 +0.043 deg 인데
        #   끊기는 순간 EMA 값이 -0.367 deg 였고 그대로 고정됐다.
        #   113.3 m 주행에서 이 차이만으로 횡오차 0.73 m 가 생긴다
        #   (전체 복귀오차 1.55 m 의 절반).
        # 그래서 최근 residual 이력의 중앙값을 따로 들고 있다가, 그걸
        # 고정값으로 넘긴다. 이력이 흩어져 있으면(std 큼) 아예 0 을 쓴다 -
        # 신뢰할 수 없는 보정을 넣느니 안 넣는 편이 낫다.
        self.hold_samples = max(1, int(hold_samples))
        self.hold_max_std = float(hold_max_std)
        self.history = collections.deque(maxlen=self.hold_samples)

    def reset_anchor(self):
        self.anchor = None

    def update(self, stamp_sec, x, y, yaw, speed, motion_beta=0.0):
        sample = (float(stamp_sec), float(x), float(y), float(yaw),
                  float(motion_beta))
        if abs(float(speed)) < self.min_speed:
            self.anchor = sample
            return self.bias
        if self.anchor is None:
            self.anchor = sample
            return self.bias

        dt = sample[0] - self.anchor[0]
        if dt <= 0.0 or dt > self.max_dt:
            self.anchor = sample
            return self.bias
        dx = sample[1] - self.anchor[1]
        dy = sample[2] - self.anchor[2]
        if math.hypot(dx, dy) < self.min_distance:
            return self.bias

        course = math.atan2(dy, dx)
        yaw_mid = interpolate_angle(self.anchor[3], sample[3], 0.5)
        beta_mid = 0.5 * (self.anchor[4] + sample[4])
        residual = normalize_angle(course - yaw_mid - beta_mid)
        self.anchor = sample
        if abs(residual) > self.max_abs_bias:
            return self.bias
        self.history.append(residual)
        if not self.initialized:
            self.bias = residual
            self.initialized = True
        else:
            self.bias = interpolate_angle(self.bias, residual, self.alpha)
        return self.bias

    def hold_value(self):
        """음영구간에서 고정할 bias. 신뢰할 수 없으면 0.0 을 돌려준다.

        EMA 인 self.bias 대신 최근 residual 이력의 중앙값을 쓴다.
        표본이 부족하거나 흩어져 있으면 보정을 포기한다(0.0).
        """
        n = len(self.history)
        if n < max(5, self.hold_samples // 4):
            return 0.0
        vals = sorted(self.history)
        mid = n // 2
        median = vals[mid] if n % 2 else 0.5 * (vals[mid - 1] + vals[mid])
        mean = sum(self.history) / n
        var = sum((v - mean) ** 2 for v in self.history) / n
        if self.hold_max_std > 0.0 and math.sqrt(var) > self.hold_max_std:
            return 0.0
        return median


def bicycle_sideslip(steering, rear_fraction=0.5, steering_sign=-1.0):
    """Return the kinematic bicycle course offset from body yaw (radians)."""
    delta = float(steering_sign) * float(steering)
    return math.atan(float(rear_fraction) * math.tan(delta))


def normalize_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def interpolate_angle(yaw0, yaw1, ratio):
    ratio = max(0.0, min(1.0, float(ratio)))
    return normalize_angle(yaw0 + ratio * normalize_angle(yaw1 - yaw0))


def gps_to_base(x_gps, y_gps, yaw, lever_x, lever_y):
    """Remove a body-frame GPS antenna lever arm from a map-frame fix."""
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        x_gps - (cosine * lever_x - sine * lever_y),
        y_gps - (sine * lever_x + cosine * lever_y),
    )


def position_variance(gps_sigma, speed, timestamp_sigma,
                      lever_x, lever_y, yaw_sigma):
    """Conservative isotropic position variance including time/lever errors."""
    lever = math.hypot(lever_x, lever_y)
    return (float(gps_sigma) ** 2
            + (abs(float(speed)) * float(timestamp_sigma)) ** 2
            + (lever * float(yaw_sigma)) ** 2)
